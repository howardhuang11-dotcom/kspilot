"""Terminal descent and propulsive (booster-recovery style) landing burn onto a ground target.

Phases (loop at ~20 Hz, timed in game UT so slowed-down recording is safe):
  COAST  unpowered, engine-first (surface retrograde), airbrakes out; ignition is
         decided every tick by the hoverslam stop-height test.
  BURN   landing burn. A worker thread re-plans G-FOLD (warm-started, ~1 Hz) to a
         gate a few metres above the pad; the loop tracks the latest plan. The
         analytic hoverslam law flies whenever there is no usable plan, and the
         terminal descent below the gate (closed-loop rules from the GNC tests).
"""
import math
import os
import threading
import time
from dataclasses import replace

import numpy as np

from . import atmo, core, hud, showrun
from . import powered_descent as pdg
from .flightctl import Actuator, PadFrame, set_thrust, unit


class AeroModel:
    """Online estimate of the body normal force  a_n = -c * q * beta,
    beta = nose offset from the airflow axis (perpendicular part), q = dynamic pressure.
    Scalar recursive least squares with forgetting; c has units m^2/kg (~ C_N A / m)."""

    def __init__(self, c0=1e-3, lam=0.995):
        self.num, self.den, self.lam, self.c0 = 0.0, 0.0, lam, c0

    @property
    def c(self):
        return min(max(self.num / self.den, -0.02), 0.02) if self.den > 1e3 else self.c0

    @property
    def confident(self):
        return self.den > 3e7

    def update(self, a_n, q, beta):
        x = q * np.asarray(beta)
        if np.dot(x, x) < 1.0:          # need a real offset and some dynamic pressure
            return
        self.num = self.lam * self.num - float(np.dot(a_n, x))
        self.den = self.lam * self.den + float(np.dot(x, x))

    def soften(self, k=0.2):
        self.num *= k
        self.den *= k


def com_height(vessel):
    """Height of the CoM above the lowest point of the vessel (legs if deployed), m."""
    lo, hi = vessel.bounding_box(vessel.reference_frame)
    return -lo[1]


# --- A1e stage-3 helpers (DR-002 v2 / DR-005; pure functions: no kRPC, unit-checked offline on logged states) ---
def a1e_aoa_cap(q, aoa_max=5.0, q1=110e3, q2=150e3, aoa_hi=3.0):
    """Commanded AoA (deg) for the A1e lift-vector channel vs dynamic pressure q (Pa).
    DR-004 (time_to_peak 1.5 s): 6 deg held at 116 kPa, but pitch/yaw inputs saturated 76 % of the step -> the real
    authority limit there is ~6-7 deg. So: full aoa_max up to 110 kPa (flown and controllable with margin), linear
    taper to 3 deg at 150 kPa (beyond the 148 kPa logged maximum: never flown), 3 deg above. The aero restoring and
    fin control torques both scale with q, so no low-q limit (DR-003: 6 -> 5.3 deg at 4.8 kPa)."""
    if q <= q1:
        c = aoa_max
    elif q >= q2:
        c = aoa_hi
    else:
        c = aoa_max + (aoa_hi - aoa_max) * (q - q1) / (q2 - q1)
    return min(aoa_max, c)


class BlendDrag:
    """Drag at an intermediate AoA for the fins-only core: tab_in (straight, 0 deg) + w * (tab_out - tab_in), where
    tab_out is the belly table measured at table_deg (KSP_BELLY_TABLE, 12 deg) and w = aoa / table_deg (linear in
    AoA; tables at 15 km, 1000 m/s: CdA 5.7 -> 8.9 m^2 for 0 -> 12 deg, i.e. +1.3 m^2 at 5 deg vs the trade's +1.5),
    times k (measured / blended model, learned in flight at the flown AoA)."""

    def __init__(self, tab_in, tab_out, w, k=1.0):
        self.tab_in, self.tab_out, self.w, self.k = tab_in, tab_out, float(w), float(k)

    def __call__(self, alt, v, mach=None):
        a = self.tab_in(alt, v, mach)
        return self.k * (a + self.w * (self.tab_out(alt, v, mach) - a))


def a1e_find_ignition(imp, m, tmax, g, cda, hs, h_com=0.0, target=None, top=8000.0, target_alt=None):
    """Landing-burn ignition on a predicted (unpowered) trajectory: the first point below `top` where the lander's own
    rule (pdg.hoverslam_burn_now, WITH its lateral term unless the pad is > 1000 m away, exactly as in run()) says go.
    The lateral term dominates when the stage is fast and off the pad (DR-004: lit at 4877 m; the vertical test alone
    gives ~1.4 km). Bisection over the integrator's history (~300 m apart near ignition); inside the bracket linear in
    the vertical margin, or the midpoint when the lateral test fired. Returns dict(alt [m ASL], h [m above pad], speed,
    vz, t [s after the state]) or None. target_alt: the target's terrain height (offset aim, kspilot/aim.py);
    None = the pad."""
    from . import entry
    t, rb, vb, _, _, alt = imp.history_bf()
    hp = alt - (pdg.PAD_ALT if target_alt is None else target_alt) - h_com
    idx = [i for i in range(len(t)) if hp[i] < top]
    if not idx:
        return None
    tgt = target if target is not None else entry.KSC_PAD
    Mp = np.vstack(entry.enu_basis(*tgt))
    pad = entry.latlonalt_to_bf(tgt[0], tgt[1], entry.KSC_PAD_ALT if target_alt is None else target_alt)

    def test(i):
        up = rb[i] / np.linalg.norm(rb[i])
        vz = float(np.dot(vb[i], up))
        if vz >= -1.0:
            return False, 1e9, vz
        r_h, v_h = (Mp @ (rb[i] - pad))[:2], (Mp @ vb[i])[:2]
        far = float(np.linalg.norm(r_h)) > 1000.0
        go, mg = pdg.hoverslam_burn_now(float(hp[i]), vz, m, tmax, g, cda, None, hs,
                                        None if far else r_h, None if far else v_h)
        return bool(go), float(mg), vz

    lo, hi = idx[0], idx[-1]
    g_lo, g_hi = test(lo), test(hi)
    if g_lo[0]:                         # already at / below the ignition point
        hi, g_hi = lo, g_lo
    elif not g_hi[0]:                   # never lights on this trajectory (should not happen: h -> 0 at impact)
        return None
    else:
        while hi - lo > 1:
            mid = (lo + hi) // 2
            gm = test(mid)
            if gm[0]:
                hi, g_hi = mid, gm
            else:
                lo, g_lo = mid, gm
    if hi == lo:
        f = 0.0
    elif g_hi[1] <= 0.0 < g_lo[1]:      # vertical stop-height test: interpolate the margin's zero crossing
        f = min(max(g_lo[1] / (g_lo[1] - g_hi[1]), 0.0), 1.0)
    else:                               # lateral test fired inside the bracket
        f = 0.5
    lerp = lambda a: float(a[lo] + f * (a[hi] - a[lo]))
    sp = np.linalg.norm(vb, axis=1)
    return dict(alt=lerp(alt), h=lerp(hp), speed=lerp(sp), vz=g_hi[2], t=lerp(t - t[0]))


def a1e_predict(r_bf, v_bf, m, drag, target, heading, tmax=0.0, isp=300.0, g=9.81, cda=1.0, hs=None, h_com=0.0,
                burn_thr=0.3, target_alt=None):
    """A1e on-board predictor: deorbit.impact_correction (4 predictions: nominal + finite-difference Jacobian per m/s
    along retro / up / side) with the fins-only blended drag. With hs (HoverslamParams) and tmax > 0 the landing burn
    is included: ignition from a1e_find_ignition on the unpowered trajectory, then an air-retro burn of 0.95 x the
    ignition speed at throttle burn_thr. burn_thr 0.3 is calibrated on DR-004 (from its ignition state: flown +2524 m
    with the burn diverting back; model +2347 at 0.3, +1252 at 0.65, unpowered +3341): the unpowered impact is biased
    LONG by ~1 km at 460 m/s ignition, so aiming it at 0 would leave the stage short (unrecoverable by retro pulses).
    Returns (Correction, ignition dict or None)."""
    from . import deorbit, entry
    ign, burns = None, ()
    if hs is not None and tmax > 0:
        imp = entry.predict(r_bf, v_bf, m, drag=drag)
        ign = a1e_find_ignition(imp, m, tmax, g, cda, hs, h_com, target, target_alt=target_alt)
        if ign is not None:
            burns = (entry.Burn(t_start=0.0, thrust=float(tmax), isp=float(isp), throttle=float(burn_thr),
                                mode="air_retro", dv=0.95 * ign["speed"], alt_trigger=ign["alt"]),)
    c = deorbit.impact_correction(r_bf, v_bf, m, drag=drag, target=target, heading=heading, burns=burns,
                                  **({} if target_alt is None else {"target_alt": target_alt}))
    return c, ign


def a1e_size_pulse(c, aim_along, cap, tilt_deg=8.0, min_dv=1.0):
    """Retro trim pulse along the airflow axis (+ a bounded cross part) that moves the predicted along-track miss to
    aim_along and the cross-track miss to 0, linearised on the predictor's finite-difference Jacobian (c.jac: rows
    along / cross, columns per m/s along c.basis = retro, up, side). ONLY EVER SHORTENS: along <= aim -> no burn
    (why='short'). Cross part limited to tan(tilt_deg) x the retro part (<= 8 deg off retrograde); total capped at cap.
    Returns dict(dva, dvc, dv, dv_bf, capped, why, e_al, e_cr, lin_al, lin_cr) (lin_* = linear prediction after)."""
    J = np.asarray(c.jac, float)
    e_al, e_cr = float(c.along) - aim_along, float(c.cross)
    out = dict(dva=0.0, dvc=0.0, dv=0.0, dv_bf=np.zeros(3), capped=False, why=None, e_al=e_al, e_cr=e_cr,
               lin_al=float(c.along), lin_cr=e_cr)
    if e_al <= 0.0:
        out["why"] = "short"
        return out
    if J[0, 0] > -0.5:                  # m along per m/s retro must be clearly negative
        out["why"] = "no sensitivity (%.2f m per m/s)" % J[0, 0]
        return out
    try:
        dva, dvc = np.linalg.solve(np.array([[J[0, 0], J[0, 2]], [J[1, 0], J[1, 2]]]), [-e_al, -e_cr])
    except np.linalg.LinAlgError:
        dva, dvc = -e_al / J[0, 0], 0.0
    dva = max(float(dva), 0.0)
    lim = dva * math.tan(math.radians(tilt_deg))
    dvc = min(max(float(dvc), -lim), lim)
    dva = max(-(e_al + J[0, 2] * dvc) / J[0, 0], 0.0)      # re-solve along with the clipped cross part
    lim = dva * math.tan(math.radians(tilt_deg))
    dvc = min(max(dvc, -lim), lim)
    dv = math.hypot(dva, dvc)
    if dv > cap:
        dva, dvc, out["capped"] = dva * cap / dv, dvc * cap / dv, True
        dv = cap
    if dv < min_dv:
        out["why"] = "tiny (%.2f m/s)" % dv
        return out
    B = np.asarray(c.basis, float)
    out.update(dva=dva, dvc=dvc, dv=dv, dv_bf=dva * B[0] + dvc * B[2],
               lin_al=float(c.along) + J[0, 0] * dva + J[0, 2] * dvc, lin_cr=e_cr + J[1, 0] * dva + J[1, 2] * dvc)
    return out


def a1e_refine_pulse(pred, c, s, aim_along, cap, iters=3, tol=30.0):
    """Secant refinement of a sized pulse on the full predictor: a big retro pulse changes the drag regime and the
    landing-burn ignition (DR-004 state at 12 km: 481 m/s linearised to +150 m, re-predicted -1701 m), so the
    magnitude is iterated with the direction (retro / cross ratio) fixed, never above cap. pred(dv_bf) -> Correction.
    Returns s updated (dva, dvc, dv, dv_bf, capped) plus 'after' = (along, cross) of the last full prediction."""
    if s.get("why") or s["dv"] <= 0:
        return s
    u = np.asarray(s["dv_bf"], float) / s["dv"]
    x0, a0 = 0.0, float(c.along)
    x1 = s["dv"]
    c1 = pred(u * x1)
    for _ in range(iters):
        if abs(c1.along - aim_along) <= tol or abs(c1.along - a0) < 1e-6:
            break
        x2 = x1 - (c1.along - aim_along) * (x1 - x0) / (c1.along - a0)
        x2 = min(max(x2, 0.0), cap)
        if abs(x2 - x1) < 0.5:
            break
        x0, a0, x1 = x1, float(c1.along), x2
        c1 = pred(u * x1)
    k = x1 / s["dv"]
    s = dict(s, dva=s["dva"] * k, dvc=s["dvc"] * k, dv=x1, dv_bf=u * x1, capped=x1 >= cap - 1e-6,
             after=(float(c1.along), float(c1.cross)))
    return s


class Lander:
    # robustness defaults, chosen with an offline Monte Carlo of the terminal descent (dispersed vehicles, holdout
    # seeds; that simulator went stale and is not in this release)
    floor_g = 0.6          # landing-burn throttle floor capped at 0.6 g (a fixed 12 % hovers a high-TWR stage)
    a_lo_max = 5.0         # cap of the slowest-descent profile deceleration, m/s^2 (high TWR limit cycles)
    glide_max_deg = float(os.environ.get("KSP_GLIDE_MAX", 8.0))    # aerodynamic glide steering in the final coast (body normal force), max tilt
    glide_below = float(os.environ.get("KSP_GLIDE_BELOW", 10000.0))   # glide steering below this height
    roll_ki = float(os.environ.get("KSP_ROLLI", 1.5))       # roll damper integral gain (cancels steady aero roll torque)
    roll_damp = float(os.environ.get("KSP_ROLLDAMP", 6.0))   # roll-rate damper (A/B 0/2/6: roll rms 38.9/14.9/5.6 deg/s)   # roll-rate damper gain (control.roll per rad/s)
    xglide_along = None   # along-track belly glide above glide_below (default: craft without airbrakes)
    xglide = not os.environ.get("KSP_NOXGLIDE")   # predictor-based cross-track fin steering above glide_below (10 km cross 200 -> 81 m)
    xglide_top, xglide_T = 35000.0, 15.0
    own_att = bool(os.environ.get("KSP_OWNATT"))     # own PD attitude controller in the low coast
    own_att_top = float(os.environ.get("KSP_OWNATT_TOP", 3000.0))
    own_kp = float(os.environ.get("KSP_OWNKP", 1.5))
    own_kd = float(os.environ.get("KSP_OWNKD", 2.5))
    fins_passive_below = float(os.environ.get("KSP_FINPASSIVE", 0.0))   # fins stop steering below this height (0 = never)
    fins_off_in_burn = True   # disable all control surfaces at landing-burn ignition
    glide_c_per_kg = float(os.environ.get("KSP_GLIDE_C", 15.0))       # measured side-force gain c*m (grid fins); 0 = online estimate
    glide_drag_zem = True  # glide predicts the drift with drag-decayed horizontal velocity
    glide_zev = True       # glide: ZEM/ZEV to the burn start (position AND horizontal velocity to zero)
    glide_h_ign = 450.0    # typical landing-burn ignition height (m), end point of the glide
    glide_ki = 0.0         # glide acceleration-loop integral gain (1/s); MC: hurts, off
    sysid_deg = float(os.environ.get("KSP_SYSID_DEG", 5.0))
    sysid = False          # fixed-tilt side-force identification in the final coast (env KSP_SYSID)
    fin_steer = not os.environ.get("KSP_NOFINS")   # trailing airbrakes also steer below fin_alt (validated: 0.8/0.3/0.1 m)
    entry_guide = not os.environ.get("KSP_NOGUIDE")  # steer the entry burn to null the cross-track miss
    post_trim = bool(os.environ.get("KSP_TRIM"))   # (off: a coasting stage cannot turn even 15 deg) trim burn after the entry burn
    trim_min_m, trim_max_dv = 15.0, 8.0               # skip below 15 m predicted cross; never more than 8 m/s
    trim_tilt_deg = 15.0                              # trim burn direction: 15 deg off retrograde
    vernier = not os.environ.get("KSP_NOVERNIER")    # sideways vernier with the engine lit after the entry burn
    vernier_tilt_deg, vernier_throttle = 15.0, 0.3
    kick_thr = float(os.environ.get("KSP_KICK", 0.0))   # landing-burn throttle while the stage is not yet turning
    dm_cont = os.environ.get("KSP_DMCONT", "0") == "1"   # continuous airbrake deploy angle (OFF: the proven GF baseline flew bang-bang; never flown on GF, audit 2026-10-20)
    dm_end = 1500.0        # airbrake drag modulation runs down to here (was 3000: left ~90 m along-track)
    fin_alt = 1500.0       # enable fin steering below this height: after drag modulation (brakes then stay out)
    # CA-2 (mishap T-WorksFineGF, logs/wd_20261020_231839.out + wd_20261020_234523.out): each fin + airbrake
    # deploy takes ~3.5-5 s, so drag learned during brake-toggle transients (measured/table 0.1-0.3) dragged
    # k_out to 0.71 (true ~1.28). Every brake change stamps _brake_change (set_brakes) and learning waits 5 s.
    # (The q-safety brake hold of 204dbfc was removed: it changed the proven bang-bang reference behaviour.)
    brake_settle_s = 5.0      # no drag learning for this long after ANY brake state change (fin + airbrake animation)
    c_prior_per_kg = 0.0   # body normal-force prior (c * m); old value 37.5 was ~100x the measured one
    glide_tmin = 15.0       # floor on the glide time-to-go (s): slower than the attitude loop (MC: 3 s 59%, 10 s 72%)
    term_kp, term_kv, term_tilt = 0.1, 0.6, 12.0   # below commit_h: centring gain, lateral speed gain, tilt cap
    lat_law = "cascade"    # landing-burn lateral law above commit_h (zemzev: MC unstable with attitude lag, off): 'zemzev' or 'cascade'
    commit_h, funnel_k = 25.0, 0.2   # stop chasing position below commit_h; lateral speed cap funnel_k * h (was 60 m / 0.12)
    gate_kv, gate_tilt = 0.8, 8.0                  # below the gate: lateral damping gain, tilt cap
    commit_cap = False     # lateral speed nullable before the 60 m commit height (MC: costs divert, off)
    # offset aim (DR-012, kspilot/aim.py; Lander(target_alt=...) sets these). The class defaults ARE the pad: a run
    # without an offset target computes exactly what it did before (tests/test_aim.py)
    target_alt = None      # terrain height of the offset target (m ASL), None = the pad
    alt0 = pdg.PAD_ALT     # height of the frame origin (m ASL): pad level, or the offset target's terrain
    offsite = False        # no pad structure under the target: radar ground all the way down (CA-3 off-pad logic)
    _tkw = {}              # extra predictor kwargs (target_alt) for an offset target
    save_suffix = ""       # g5_mid / g5_term of an offset-aim run never overwrite the pad checkpoints

    def __init__(self, conn, vessel, target=core.KSC_PAD, log=print, use_gfold=True, overlay_tag="landing",
                 h_gate=4.0, v_td=1.5, hs=None, freeze_leading=True, h_com=None, legs_alt=250.0, target_alt=None):
        self.conn, self.v, self.log = conn, vessel, log
        if target_alt is not None:       # offset aim: the frame origin is the target's terrain, not the pad
            self.target_alt = self.alt0 = float(target_alt)
            self.offsite, self._tkw = True, {"target_alt": float(target_alt)}
        self.sc = conn.space_center
        self.body = vessel.orbit.body
        self.frame = PadFrame(self.body, *target)
        self.g = self.body.surface_gravity
        self.use_gfold = use_gfold
        self.freeze_leading = freeze_leading
        self.q_entry, self.entry_burn = 20000.0, False   # Pa: entry-burn trigger
        self.entry_done, self.entry_alt, self.entry_dv, self.entry_flown = False, 45000.0, 590.0, 0.0
        self.retro_until = 0.0
        self.retract_brakes_on_burn = False   # stability first: top-mounted brakes keep an engine-first stage straight
        self.h_gfold_min = 150.0     # below this the hoverslam law (overshoot-proof velocity cap) diverts
        self.law = "zem"           # 'zem' (continuous E-guidance) or 'gfold'
        self.lag = 1.5               # attitude response lag used by the lateral guidance, s
        self.kp_lat, self.kv_lat = float(os.environ.get("KSP_KPLAT", 0.2)), 0.5
        self.gimbal_limit = 0.5
        self.v_final = 1.5           # final vertical descent rate below the gate, m/s
        self.below_stall_s = 3.0     # CA-3: below pad level, |vz| < 0.5 this long without leg contact -> cut
        self.below_max_s = 40.0      # CA-3: below pad level this long without touchdown -> cut (fuel guard)
        self.thr_floor = 0.12        # minimum landing-burn throttle below 500 m (no free fall, keeps gimbal authority)
        self.tag = overlay_tag
        self.gate = np.array([0.0, 0.0, h_gate])
        # drag fades as the burn slows the stage (F ~ v^2): credit only a third of today's drag; +25 m margin
        # divert check with the authority seen in flight: the stage cannot tilt for ~3 s at ignition (high q,
        # aero stability torque) and gets ~35 % of the tilted thrust sideways -> big offsets light earlier
        self.hs = hs or pdg.HoverslamParams(v_td=v_td, drag_frac=0.35, h_stop=25.0, throttle=0.65,
                                            lat_eff=0.35, t_att=3.0)
        # below the gate: commit to a straight vertical descent, damp lateral velocity only (no position chase)
        self.terminal = replace(self.hs, h_stop=h_gate, tilt_final_deg=6.0, kp_h=0.0, kv_h=0.5)
        rf = self.frame.rf
        self.ut = conn.add_stream(getattr, self.sc, 'ut')
        self.pos = conn.add_stream(vessel.position, rf)
        self.vel = conn.add_stream(vessel.velocity, rf)
        self.mass = conn.add_stream(getattr, vessel, 'mass')
        self.thrust = conn.add_stream(getattr, vessel, 'available_thrust')
        self.sit = conn.add_stream(getattr, vessel, 'situation')
        self.drag = conn.add_stream(getattr, vessel.flight(rf), 'drag')
        self.radar = conn.add_stream(getattr, vessel.flight(rf), 'surface_altitude')
        if h_com is None:   # measure with the legs down (only safe where the air is thin)
            vessel.control.gear = True
            time.sleep(4)
            h_com = com_height(vessel)
            lo, hi = vessel.bounding_box(vessel.reference_frame)
            foot = min(abs(lo[0]), abs(hi[0]), abs(lo[2]), abs(hi[2]))
            tip = math.degrees(math.atan2(foot, max(h_com, 0.1)))
            vessel.control.gear = False
            log("CoM height above leg feet: %.2f m, footprint radius %.1f m, tip-over angle %.0f deg%s" % (
                h_com, foot, tip, "  (WARNING: narrow stance, landing must be very gentle)" if tip < 15 else ""))
        self.h_com = h_com
        self.legs_alt = legs_alt
        self.cda = 1.0
        self.est = pdg.ThrustEstimator()
        self.a_dist = np.zeros(3)
        # prior from flight data (WorksFinePayload core, brakes out): c*m ~ 0.0025*15000 m^2
        # one normal-force model for the final coast and the burn: the glide excites it and it is learned
        # online within seconds (Monte Carlo: same result for a right, 3x small or sign-flipped prior)
        # prior 0: measured in flight (fixed 3 deg tilt, final coast, 2026-10-19) the body normal force of the
        # WorksFine core is ~1 % of the old prior; a wrong large prior made aero_map refuse to steer at high q.
        # Any real normal force is learned online within seconds of the burn.
        self.aero_burn = AeroModel(c0=self.c_prior_per_kg / max(vessel.mass, 1000.0))
        self.aero_coast = self.aero_burn
        self.aero = self.aero_coast
        self.nose = conn.add_stream(vessel.direction, self.frame.rf)
        self.angvel = conn.add_stream(vessel.angular_velocity, self.frame.rf)   # rad/s (body frame axes)
        self._sf = None   # (ut, vel) for specific-force measurement
        self.prev = None
        self.plan, self.plan_ut = None, 0.0
        self._lock = threading.Lock()
        self._stop = False
        self.events = []
        self._pad_cleared = 0
        self.target = target
        self.cda_out, self.cda_in, self._dm_t, self.dm_margin = None, None, 0.0, 30.0
        self.drag_out = None
        self.drag_in = None
        self.draw = False      # presentation: HUD camera + predicted-track data (recordings)
        self.rec_v2 = False    # recording pipeline v2 (HOP_REC=full only, director v2 specification 8.3 / 6 / 11)
        self.draw_track = False   # draw the predicted track line in the scene (off: cluttered)
        self.sysid = bool(os.environ.get("KSP_SYSID"))
        # grid fins share the brakes action: below glide_below keep everything deployed and let the glide
        # steer both axes (drag modulation folding the fins mid-glide lost the steering, 69 m miss)
        self.has_fins = any("Grid Fin" in p.name for p in vessel.parts.all)
        self.has_airbrakes = any("airbrake" in p.name for p in vessel.parts.all)
        from .mission import default_entry_dv
        self.entry_dv = default_entry_dv(vessel)
        # q-safety burn limit: fins-only titanium cores fly a higher-q profile by design (no structural q limit in
        # stock KSP); 25 kPa there triggered 580 m/s of unplanned burns
        # fins-only: 45 kPa fired a 930 m/s q-hold burn at 15 km with nothing hot or broken; the
        # real limit there is HEAT (part temperature / max), q only guards against absurd values
        fins_only = self.has_fins and not self.has_airbrakes
        # (C mode flies ~105 kPa by design, theory; 93 kPa flown harmlessly)
        self.q_safety = float(os.environ.get("KSP_QSAFE", 250000.0 if fins_only else 1.25 * self.q_entry))
        self.heat_safety = float(os.environ.get("KSP_HEATSAFE", 0.85 if fins_only else 9.0))
        self.heat = 0.0                     # max over parts of temperature / max temperature (internal or skin)
        if self.xglide_along is None:
            self.xglide_along = (os.environ.get("KSP_XGLIDE_ALONG", "1" if (self.has_fins and not self.has_airbrakes) else "0") == "1")
        # fins-only craft that glide along-track keep the fins OUT (they steer); no fin-folding drag modulation
        # BELLY mode (fins-only craft, user's idea "use the tank's belly for drag", Space-Shuttle style): above the
        # glide the drag modulation sets the angle of attack 0..belly_deg (CdA 3.7 -> 19 m^2 at Mach 6); the tilt
        # side is chosen for the cross-track error and reversed when that error keeps growing (roll reversals)
        self.belly = self.has_fins and not self.has_airbrakes and os.environ.get("KSP_BELLY", "1") == "1"
        self.belly_deg = float(os.environ.get("KSP_BELLY_DEG", 45.0))
        self._belly_f, self._belly_sign, self._belly_hist = 1.0, 1.0, []
        if self.belly:
            self.xglide_along = False
        self.dm_off = self.has_fins and not self.has_airbrakes and self.xglide_along and not self.belly
        # fins-only body normal force, clean fit on 2 flights: c ~ 1.3e-4 m^2/kg at q 50-130 kPa (c*m ~ 2, not 15)
        self.c_lift0 = float(os.environ.get("KSP_CLIFT0", 1.3e-4))
        # belly lift direction: "up" (vertical plane, no cross-track push; lift-up = shallower, lower q) or the
        # old "side" with roll reversals (pushed the cross error to 1-2 km)
        self.belly_dir = os.environ.get("KSP_BELLY_DIR", "up")

        # Grid-fin orientation. The KRE fin's blade extends along its local +x (surface attach on -x).
        # T-WorksFineGF had them rotated 180 deg: blades buried in the tank, control reversed on all three
        # axes (measured from g5_term: pitch/yaw/roll +1 -> +9.7/+9.7/+80 deg/s against the wheels'
        # -1/-0.8/-37); that fight caused the coast wobble. Detect it from the geometry and compensate with
        # a negative authority limiter (the stock "inverted" flag does not change the control direction).
        self.fin_sign_detected = 1.0
        if self.has_fins:
            sc = self.conn.space_center
            rf = vessel.reference_frame
            votes = []
            for p in vessel.parts.all:
                if "Grid Fin" in p.name:
                    x_out = np.asarray(sc.transform_direction((1.0, 0.0, 0.0), p.reference_frame, rf))
                    pos = np.asarray(p.position(rf))
                    radial = np.array([pos[0], 0.0, pos[2]])
                    votes.append(float(np.dot(x_out, radial)))
            self.fin_sign_detected = -1.0 if votes and np.median(votes) < 0 else 1.0
            # NOT about mounting: KSP control surfaces assume nose-first flight; falling ENGINE-first reverses their
            # effect for any orientation (GF2T, fins correctly outward: +100% -> pitch +11 / roll +164 deg/s against
            # the wheels' -1 / -25; -100% -> -12 / -165). So the descent always needs a negative authority.
            sign = float(os.environ.get("KSP_FINSIGN", -1.0))
            auth = float(os.environ.get("KSP_FINAUTH", 100.0))
            for cs in vessel.parts.control_surfaces:
                if "Grid Fin" in cs.part.name:
                    cs.authority_limiter = sign * auth
            self.log("grid fins: mounting %s, authority %+.0f%% (engine-first flight reverses control surfaces)" % (
                "inward" if self.fin_sign_detected < 0 else "outward", sign * auth))
        # every sign tuned while the fins fought the autopilot absorbed that reversal: with working fins
        # the nose follows the command, the glide's side force points WITH the tilt and the roll damper
        # (fin-dominated torque) changes sign as well
        fins_ok = self.has_fins and float(os.environ.get("KSP_FINSIGN", -1.0)) < 0     # fins act the right way
        # The sign is a property of the configuration AND the flight regime, not of the fins working:
        # flown fits a_n = -c q beta: GF0T (fins only) c > 0 from 40 km to 1 km (body lift: force AGAINST the top's
        # tilt); GF2T c > 0 above 10 km, c < 0 below (its airbrake plates dominate: force WITH the tilt). So it
        # starts from that prior and is then LEARNED in flight from the sign of the online normal-force estimate.
        fins_only_ = self.has_fins and not self.has_airbrakes
        # learning only where the sign really changes (airbrake craft: flips at ~10 km). Fins-only: the fitted sign is
        # the same from 40 to 1 km on two flights, while the online estimate at q 15-120 kPa (|c| ~ 1.5-2e-4, near
        # the hysteresis) flipped it 6 times in 27 s and turned the lift guidance against itself
        self.nf_learn = "KSP_NFSIGN" not in os.environ and not fins_only_ and os.environ.get("KSP_NFLEARN", "0") == "1"
        # (off: the proven GF baseline had a fixed +1, and the online c is biased by the frame)
        self.nf_sign = float(os.environ.get("KSP_NFSIGN", -1.0 if fins_only_ else (1.0 if fins_ok else -1.0)))
        # wheels alone act the "working" way as well (g5_term: roll +1 -> -37 deg/s), so a fin-less craft uses -1
        self.roll_sign = float(os.environ.get("KSP_ROLLSIGN", -1.0 if (fins_ok or not self.has_fins) else 1.0))
        # slower autopilot response with working fins (time-to-peak sweep from g5_term: 1/2/3/5 s ->
        # pitch/yaw rate rms 5.8/4.7/2.3/1.8 deg/s, miss 3.2/4.8/5.2/0.9 m)
        self.ttp = float(os.environ.get("KSP_TTP", 5.0 if fins_ok else 1.0))
        self._cam0, self._tl_t3, self._tl_t4 = None, 1e9, 1e9
        self._own = None
        self._a_lat, self._gl_int, self._gl_t = np.zeros(2), np.zeros(2), None
        self.aim_offset = 0.0  # learned along-track bias of the final approach (m, + = long)
        self.heading = None    # ground-track heading fixed by the deorbit plan
        self.k_drag = {True: 1.0, False: 1.0}    # measured / table, per airbrake state (online)
        self.solve_times = []
        # IDENTIFICATION flight (env KSP_IDENT=1, default OFF; DR-002 condition 2, envelope stage-3 hard gate): after
        # the planned entry burn and above ident_floor there is no belly / drag modulation / cross glide; the top is
        # held on the TRUE airflow axis plus an AoA step in a fixed azimuth (vertical plane, top up by default) while
        # q is inside a band. Straight (0 deg) flight q passes each band in only ~7-13 s (flights 200037 / 203827:
        # 5 kPa at 30 km, 20 at 23, 60 at 17, 120 at 11 km), so the holds are 5 s / gaps 3 s and the design-relevant
        # 6 deg step comes first in every band. Below the floor: normal glide / landing. Only the HEAT safety burn
        # stays (no q-safety burn, no vernier).
        self.ident = os.environ.get("KSP_IDENT", "0") == "1"
        if self.ident:
            self.ident_floor = float(os.environ.get("KSP_IDENT_FLOOR", 10000.0))
            self.ident_steps = [float(x) for x in os.environ.get("KSP_IDENT_STEPS", "6,9,3").split(",")]
            self.ident_hold = float(os.environ.get("KSP_IDENT_HOLD", 5.0))
            self.ident_gap = float(os.environ.get("KSP_IDENT_GAP", 3.0))
            self.ident_sign = -1.0 if os.environ.get("KSP_IDENT_DIR", "up") == "down" else 1.0
            # "centre:lo-hi[:s1/s2..]" kPa; the optional step list overrides KSP_IDENT_STEPS for that band
            self.ident_bands, self.ident_band_steps = [], {}
            for b in os.environ.get("KSP_IDENT_BANDS", "5:3.5-9,20:13-32,60:40-75,120:90-160").split(","):
                f_ = b.split(":")
                lo_, hi_ = f_[1].split("-")
                self.ident_bands.append((float(f_[0]), float(lo_), float(hi_)))
                if len(f_) > 2:
                    self.ident_band_steps[float(f_[0])] = [float(x) for x in f_[2].split("/")]
            # attitude-controller stiffness inside the IDENT window only (DR-004; DR-003: no control saturation with
            # time_to_peak 5 s -> the controller, not the aero, limited the AoA). Unset = unchanged (self.ttp).
            # max_angular_velocity (0.35 rad/s = 20 deg/s) does not cap a 6-10 deg step (< 0.5 s of slew): unchanged
            self.ident_ttp = float(os.environ["KSP_IDENT_TTP"]) if "KSP_IDENT_TTP" in os.environ else None
            self.vernier = False
            self.q_safety = float("inf")
            self._id_cmd, self._id_end, self._id_k, self._id_abort, self._id_open = 0.0, -1e9, {}, None, None
            log("IDENT mode: steps %s deg (hold %.0f s, gap %.0f s), top %s, bands %s kPa (band steps %s), floor %.0f m, "
                "time_to_peak in window %s; no belly / drag modulation / cross glide / vernier / q-safety burn" % (
                    self.ident_steps, self.ident_hold, self.ident_gap, "up" if self.ident_sign > 0 else "down",
                    ["%g-%g" % (lo, hi) for _, lo, hi in self.ident_bands], self.ident_band_steps or "-",
                    self.ident_floor, self.ident_ttp if self.ident_ttp else "unchanged (%.1f s)" % self.ttp))
        # A1e STAGE-3 build (DR-002 v2 design A1e, DR-005; env KSP_A1E=1, default OFF; fins-only craft only, never
        # together with IDENT). Envelope stage 3 starts from g5_mid (15 km): no entry burn / 38.5 km relight here.
        # After the entry burn, above a1e_floor and until landing-burn ignition: hold AoA = a1e_aoa_cap(q) with the
        # lift AZIMUTH steered (<= 15 deg/s) from the predictor's cross-track error (both axes after the last pulse),
        # autopilot time_to_peak 1.5 s; retro trim pulses at ~12 km (aim +150 m) and ~5 km (aim 0), shorten-only;
        # landing burn with the grid fins active and gimbal 1.0. The airbrake baseline never enters any of this.
        self._a1e_setup(log)

    def _a1e_setup(self, log):
        """A1e configuration (env), called from __init__; separate so offline checks can build it without kRPC."""
        self.a1e = (os.environ.get("KSP_A1E", "0") == "1" and self.has_fins and not self.has_airbrakes
                    and not self.ident)
        if os.environ.get("KSP_A1E", "0") == "1" and not self.a1e:
            log("KSP_A1E ignored: A1e needs a fins-only craft (no airbrakes) and no KSP_IDENT")
        if self.a1e:
            env = os.environ.get
            self.a1e_aoa = float(env("KSP_A1E_AOA", 5.0))          # deg, upper limit of the q cap
            self.a1e_floor = float(env("KSP_A1E_FLOOR", 1500.0))   # m: below it the normal final-coast glide
            self.a1e_ttp = float(env("KSP_A1E_TTP", 1.5))          # s, autopilot time_to_peak inside the window
            self.a1e_rate = float(env("KSP_A1E_RATE", 15.0))       # deg/s lift-azimuth slew (trade: faster drags
            self.a1e_tx = float(env("KSP_A1E_TX", 10.0))           # the nose through 0 deg); s, cross-only time const
            self.a1e_rev = float(env("KSP_A1E_REV_DV", 3.0))       # m/s vertical-lift integral before a reversal
            self.a1e_table_deg = float(env("KSP_BELLY_TABLE", 12.0))   # AoA of the tab_out (belly) drag table
            self.a1e_tilt = float(env("KSP_A1E_PTILT", 8.0))       # deg off retrograde for the pulses' cross part
            self.a1e_pthr = float(env("KSP_A1E_PTHR", 1.0))        # pulse throttle
            self.a1e_ptimeout = float(env("KSP_A1E_PTIMEOUT", 15.0))
            self.a1e_reserve = float(env("KSP_A1E_RESERVE", 2000.0))   # kg propellant never spent on pulses (~180 u)
            self.a1e_pred_burn = env("KSP_A1E_PRED_BURN", "1") == "1"
            self.a1e_pred_thr = float(env("KSP_A1E_PRED_THR", 0.3))
            self.a1e_plan = [dict(n=1, h=float(env("KSP_A1E_P1_H", 12000.0)), aim=float(env("KSP_A1E_P1_AIM", 150.0)),
                                  cap=float(env("KSP_A1E_P1_MAX", 150.0))),
                             dict(n=2, h=float(env("KSP_A1E_P2_H", 5000.0)), aim=float(env("KSP_A1E_P2_AIM", 0.0)),
                                  cap=float(env("KSP_A1E_P2_MAX", 60.0)))]
            if env("KSP_A1E_FINS_IN_BURN", "1") == "1":
                self.fins_off_in_burn = False      # instance override of the class default (True); gated to A1e
                self.gimbal_limit = 1.0
            self._a1e_k, self._a1e_kn, self._a1e_aoa_lp, self._a1e_thr_t = 1.0, 0, None, -1e9
            self._a1e_c, self._a1e_ign, self._a1e_ct, self._a1e_pt = None, None, -1e9, -1e9
            self._a1e_phi, self._a1e_vs, self._a1e_iv, self._a1e_t = 0.0, 1.0, 0.0, None
            self._a1e_pi, self._a1e_pulse, self._a1e_abort, self._a1e_open, self._a1e_closed = 0, None, None, None, False
            self._a1e_cmd, self._a1e_qmax, self._a1e_nsat, self._a1e_n, self._a1e_logt = 0.0, 0.0, 0, 0, -1e9
            self.a1e_records = []
            log("A1E mode: AoA min(%.1f, cap(q)) deg, azimuth <= %.0f deg/s, ttp %.1f s, floor %.0f m; pulses %s "
                "(throttle %.2f, <= %.0f deg off retro, reserve %.0f kg); predictor drag blend at AoA/%.0f deg, "
                "landing burn %s; fins in landing burn %s, gimbal %.1f" % (
                    self.a1e_aoa, self.a1e_rate, self.a1e_ttp, self.a1e_floor,
                    ["#%d h<=%.0f aim %+.0f cap %.0f m/s" % (p["n"], p["h"], p["aim"], p["cap"]) for p in self.a1e_plan],
                    self.a1e_pthr, self.a1e_tilt, self.a1e_reserve, self.a1e_table_deg,
                    "modelled (thr %.2f)" % self.a1e_pred_thr if self.a1e_pred_burn else "not modelled",
                    "ON" if not self.fins_off_in_burn else "off", self.gimbal_limit))

    def a1e_window(self, h):
        """A1e lift-vector / pulse window: after the entry burn, not burning it, above the floor (COAST only: the
        landing burn leaves COAST for good)."""
        return self.a1e and self.entry_done and not self.entry_burn and h > self.a1e_floor

    def _a1e_basis(self):
        """(r_bf, v_bf, retro_bf, up_bf, side_bf): the basis of deorbit.impact_correction's Jacobian (retro = -airflow,
        up = local vertical perpendicular to it, side = u x up), so its per-m/s columns map 1:1 onto these vectors.
        Converted to pad ENU with self.frame.vec (pure rotation; never take cross products in ENU)."""
        r_bf, v_bf = np.asarray(self.pos(), float), np.asarray(self.vel(), float)
        u = unit(v_bf)
        up = unit(r_bf)
        e2 = unit(up - np.dot(up, u) * u)
        return r_bf, v_bf, -u, e2, np.cross(u, e2)

    def _a1e_pred(self, m, dv_bf=None):
        """A1e predictor at the current state (optionally with an instantaneous dv, body frame)."""
        r_bf, v_bf = np.asarray(self.pos(), float), np.asarray(self.vel(), float)
        if dv_bf is not None:
            v_bf = v_bf + dv_bf
        # the future is flown at the CURRENT command (0 after an abort), the learned k carries the rest
        aoa_f = 0.0 if self._a1e_abort else (self._a1e_cmd if self._a1e_cmd > 0 else self.a1e_aoa)
        dr = BlendDrag(self.drag_in, self.drag_out, aoa_f / self.a1e_table_deg, self._a1e_k)
        isp = max((e.specific_impulse for e in self.v.parts.engines if e.active), default=300.0) or 300.0
        return a1e_predict(r_bf, v_bf, m, dr, self.target, self.heading, tmax=self.thrust() if self.a1e_pred_burn else 0.0,
                           isp=isp, g=self.g, cda=self.cda, hs=self.hs, h_com=self.h_com, burn_thr=self.a1e_pred_thr,
                           target_alt=self.target_alt)

    def a1e_close(self, h, why):
        """End of the A1e window (floor or landing-burn ignition): abandon a running pulse, summary line."""
        if not self.a1e or self._a1e_open is None or self._a1e_closed:
            return
        self._a1e_closed = True
        if self._a1e_pulse is not None:
            p = self._a1e_pulse
            self.log("A1E PULSE #%d ABANDONED (%s) at h=%.0fm: flown %.1f of %.1f m/s" % (p["n"], why, h, p["got"], p["dv"]))
            self._a1e_record(p, h, "abandoned (%s)" % why)
            self._a1e_pulse = None
        for p in self.a1e_plan[self._a1e_pi:]:
            self.log("A1E PULSE #%d not flown (%s at h=%.0fm)" % (p["n"], why, h))
        self._a1e_pi = len(self.a1e_plan)
        self.log("A1E window closed (%s) h=%.0fm: input saturation %.3f of %d ticks, max q %.1f kPa, k %.3f%s" % (
            why, h, self._a1e_nsat / max(self._a1e_n, 1), self._a1e_n, self._a1e_qmax / 1e3, self._a1e_k,
            "  (aborted: %s)" % self._a1e_abort if self._a1e_abort else ""))
        if self._a1e_qmax > 148e3:
            self.log("A1E WARNING: peak q %.1f kPa above the 148 kPa logged maximum (DR-002 grounding trigger)" % (
                self._a1e_qmax / 1e3))

    def _a1e_record(self, p, h, how):
        rn = lambda x: round(x) if x == x else None
        rec = dict(n=p["n"], h0=round(p["h0"]), q0_kpa=round(p["q0"] / 1e3, 1), dv_plan=round(p["dv"], 1),
                   dva=round(p["dva"], 1), dvc=round(p["dvc"], 1), dv_flown=round(p["got"], 1), capped=p["capped"],
                   before=[rn(x) for x in p["before"]], after_pred=[rn(x) for x in p["after"]],
                   after_flown=[rn(x) for x in p["flown_pred"]] if "flown_pred" in p else None, h_end=round(h), how=how)
        self.a1e_records.append(rec)
        self.events.append(("a1e_pulse", rec["n"], rec["h0"], rec["q0_kpa"], rec["dv_plan"], rec["dv_flown"],
                            tuple(rec["before"]), tuple(rec["after_pred"]), how))

    def _a1e_plan_pulse(self, P, ut, h, q, m):
        """Size pulse P on a fresh prediction. Logs every case (short / tiny / capped); returns the running-pulse
        state for the control loop, or None."""
        c, ign = self._a1e_pred(m)
        self._a1e_c, self._a1e_ign, self._a1e_ct = c, ign, ut
        isp = max((e.specific_impulse for e in self.v.parts.engines if e.active), default=300.0) or 300.0
        m_res = getattr(self, "dry", self.v.dry_mass) + self.a1e_reserve
        dv_fuel = isp * 9.80665 * math.log(m / m_res) if m > m_res else 0.0
        cap = min(P["cap"], dv_fuel)
        s = a1e_size_pulse(c, P["aim"], cap, self.a1e_tilt)
        head = "A1E PULSE #%d plan h=%.0fm q=%.1fkPa: predicted along %+.0f cross %+.0f (aim %+.0f), %.1f m per m/s retro, " \
               "ignition %s" % (P["n"], h, q / 1e3, c.along, c.cross, P["aim"], c.jac[0][0],
                                "%.0f m at %.0f m/s" % (ign["h"], ign["speed"]) if ign else "-")
        if s["why"]:
            self.log(head + " -> NO BURN (%s)%s" % (s["why"], ": predicted SHORT, logged only" if s["why"] == "short" else ""))
            self.events.append(("a1e_pulse", P["n"], round(h), round(q / 1e3, 1), 0.0, 0.0, (round(c.along), round(c.cross)),
                                (round(c.along), round(c.cross)), s["why"]))
            return None
        try:
            s = a1e_refine_pulse(lambda dvb: self._a1e_pred(m, dvb)[0], c, s, P["aim"], cap)
            after = s["after"]
        except Exception as e:
            self.log("A1E pulse refine error: %r (linear sizing kept)" % e)
            after = (s["lin_al"], s["lin_cr"])
        self.log(head + " -> %.1f m/s (retro %.1f, cross %+.1f%s; fuel cap %.0f) -> predicted after %+.0f / %+.0f "
                 "(linear %+.0f / %+.0f)" % (s["dv"], s["dva"], s["dvc"], ", CAPPED" if s["capped"] else "", dv_fuel,
                                             after[0], after[1], s["lin_al"], s["lin_cr"]))
        return dict(n=P["n"], dva=s["dva"], dvc=s["dvc"], dv=s["dv"], capped=s["capped"], got=0.0, t0=ut, t_last=None,
                    h0=h, q0=q, before=(c.along, c.cross), after=after, lit=False)

    def a1e_command(self, ut, h, q, m, air, nose):
        """A1e window, every tick. Returns (pointing direction ENU, throttle). Non-blocking: the predictor (5
        predictions, ~30-60 ms) runs at 1 Hz, a pulse plan once (~10 predictions); the loop keeps beating (deadman)."""
        if self._a1e_open is None:
            self._a1e_open = h
            self.log("A1E window open h=%.0fm q=%.1fkPa" % (h, q / 1e3))
        dt = min(max(ut - self._a1e_t, 0.0), 0.5) if self._a1e_t is not None else 0.0
        self._a1e_t = ut
        self._a1e_qmax = max(self._a1e_qmax, q)
        aoa_meas = getattr(self, "_aoa_meas", float("nan"))
        if self._a1e_abort is None:          # same automatic abort set as IDENT (AoA limit 3x the command)
            why = None
            if self.heat > 0.75:
                why = "heat %.2f" % self.heat
            elif getattr(self, "_parts_lost", False):
                why = "parts lost"
            elif abs(getattr(self, "_rr_last", 0.0)) > 60.0:
                why = "roll rate %.0f deg/s" % self._rr_last
            elif aoa_meas > 15.0:
                why = "AoA %.1f deg" % aoa_meas
            if why:
                self._a1e_abort = why
                self.log("A1E ABORT (%s) at h=%.0fm q=%.1fkPa: 0 deg AoA for the rest of the window (pulses still "
                         "allowed: they fly along the airflow)" % (why, h, q / 1e3))
        r_bf, v_bf, _, up_bf, side_bf = self._a1e_basis()
        up_enu, side_enu = np.asarray(self.frame.vec(up_bf), float), np.asarray(self.frame.vec(side_bf), float)
        # drag scale k = measured drag along the airflow / blended table at the MEASURED AoA (engine off >= 2 s)
        if self._a1e_pulse is None and aoa_meas == aoa_meas:
            self._a1e_aoa_lp = aoa_meas if self._a1e_aoa_lp is None else self._a1e_aoa_lp + min(dt / 1.0, 1.0) * (
                aoa_meas - self._a1e_aoa_lp)
        try:
            sp = float(np.linalg.norm(v_bf))
            alt = h + self.h_com + self.alt0
            d_air = -float(np.dot(self._dvec, v_bf)) / max(sp, 1e-6)
            w_m = min(max((self._a1e_aoa_lp or 0.0) / self.a1e_table_deg, 0.0), 1.5)
            model = BlendDrag(self.drag_in, self.drag_out, w_m)(alt, sp)
            if q > 1000.0 and model > 1.0 and ut - self._a1e_thr_t > 2.0 and self.v.control.throttle < 0.01:
                x = min(max(d_air / model, 0.3), 3.0)
                self._a1e_k = x if self._a1e_kn == 0 else self._a1e_k + 0.05 * (x - self._a1e_k)
                self._a1e_kn += 1
        except Exception:
            pass
        # retro trim pulses: plan once at their height (drag scale learned for >= 1 s first)
        if self._a1e_pulse is None and self._a1e_pi < len(self.a1e_plan) and self._a1e_kn >= 20:
            P = self.a1e_plan[self._a1e_pi]
            if h <= P["h"]:
                self._a1e_pi += 1
                try:
                    self._a1e_pulse = self._a1e_plan_pulse(P, ut, h, q, m)
                except Exception as e:
                    self.log("A1E pulse #%d plan error: %r" % (P["n"], e))
        if self._a1e_pulse is not None:
            p = self._a1e_pulse
            d = unit(p["dva"] * air + p["dvc"] * side_enu)       # engine leads: thrust along the nose (air = retro)
            ang = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(unit(nose), d))))))
            on = ang < 3.0
            if on and p["t_last"] is not None:
                p["got"] += float(self.v.thrust) / max(m, 1.0) * math.cos(math.radians(ang)) * max(ut - p["t_last"], 0.0)
            p["t_last"] = ut
            if on and not p["lit"]:
                p["lit"] = True
                self.log("A1E PULSE #%d lit at h=%.0fm (%.1f s to align, %.1f deg)" % (p["n"], h, ut - p["t0"], ang))
            self._a1e_thr_t = ut
            if p["got"] >= p["dv"] or ut - p["t0"] > self.a1e_ptimeout:
                how = "done" if p["got"] >= p["dv"] else "TIMEOUT"
                try:
                    cf, _ = self._a1e_pred(m)
                    p["flown_pred"] = (cf.along, cf.cross)
                    self._a1e_c, self._a1e_ct = cf, ut
                except Exception:
                    p["flown_pred"] = (float("nan"), float("nan"))
                short = p["got"] < 0.8 * p["dv"]
                self.log("A1E PULSE #%d %s at h=%.0fm: flown %.1f of %.1f m/s in %.1f s; re-predicted along %+.0f cross %+.0f "
                         "(planned after %+.0f / %+.0f)%s" % (p["n"], how, h, p["got"], p["dv"], ut - p["t0"],
                                                               p["flown_pred"][0], p["flown_pred"][1], p["after"][0],
                                                               p["after"][1], "  WARNING: dv shortfall > 20 %" if short else ""))
                self._a1e_record(p, h, how)
                self._a1e_pulse = None
                return d, 0.0
            return d, (self.a1e_pthr if on else 0.0)
        # lift-vector channel: predictor at 1 Hz
        if ut - self._a1e_pt >= 1.0:
            self._a1e_pt = ut
            try:
                self._a1e_c, self._a1e_ign = self._a1e_pred(m)
                self._a1e_ct = ut
                c = self._a1e_c
                hd = math.radians(c.heading)
                self._pred_en = (c.along * math.sin(hd) + c.cross * math.cos(hd),
                                 c.along * math.cos(hd) - c.cross * math.sin(hd), ut)
                self._pred_miss = math.hypot(c.along, c.cross)
            except Exception as e:
                self.log("A1E predictor error: %r" % e)
        aoa = a1e_aoa_cap(q, self.a1e_aoa) if self._a1e_abort is None else 0.0
        self._a1e_cmd = aoa
        a_l = self.c_hat() * q * math.sin(math.radians(aoa))       # lift acceleration at this AoA, m/s^2
        c = self._a1e_c
        both = self._a1e_pi >= len(self.a1e_plan)                  # after the last pulse: both axes
        phi_des = self._a1e_phi
        if c is not None and a_l > 1e-3:
            J = np.asarray(c.jac, float)
            if both:            # ZEM-like: a = -3 e / (sensitivity x t_go), t_go to the predicted ignition
                t_ref = self._a1e_ign["t"] if self._a1e_ign else c.impact.t
                tgo = max(t_ref - (ut - self._a1e_ct), 5.0)
                a_up = -c.along / J[0, 1] * 3.0 / tgo if abs(J[0, 1]) > 1.0 else 0.0
                a_sd = -c.cross / J[1, 2] * 3.0 / tgo if abs(J[1, 2]) > 1.0 else 0.0
                if math.hypot(a_up, a_sd) > 1e-3:
                    phi_des = math.atan2(a_sd, a_up)
            else:               # cross only; the vertical lift integral is held near 0 by reversals
                a_sd = -c.cross / J[1, 2] / self.a1e_tx if abs(J[1, 2]) > 1.0 else 0.0
                s = min(max(a_sd / a_l, -1.0), 1.0)
                self._a1e_iv += a_l * math.cos(self._a1e_phi) * dt
                if self._a1e_vs * self._a1e_iv > self.a1e_rev:
                    self._a1e_vs = -self._a1e_vs
                    self.log("A1E lift reversal: vertical lift now %s (integral %+.1f m/s, cross %+.0f m) h=%.0fm" % (
                        "UP" if self._a1e_vs > 0 else "DOWN", self._a1e_iv, c.cross, h))
                phi_des = math.atan2(s, self._a1e_vs * math.sqrt(max(1.0 - s * s, 0.0)))
        dphi = (phi_des - self._a1e_phi + math.pi) % (2 * math.pi) - math.pi
        lim = math.radians(self.a1e_rate) * dt
        self._a1e_phi = (self._a1e_phi + min(max(dphi, -lim), lim) + math.pi) % (2 * math.pi) - math.pi
        lift = math.cos(self._a1e_phi) * up_enu + math.sin(self._a1e_phi) * side_enu
        d = unit(air + self.nf_sign * math.tan(math.radians(aoa)) * lift)   # fins-only: lift AGAINST the top's tilt
        if ut - self._a1e_logt >= 5.0:
            self._a1e_logt = ut
            self.log("A1E h=%.0fm q=%.1fkPa aoa cmd %.1f meas %.1f, lift az %+.0f deg (%s), a_lift %.2f m/s^2, k %.3f, "
                     "pred along %s cross %s, ignition %s" % (
                         h, q / 1e3, aoa, aoa_meas, math.degrees(self._a1e_phi), "both axes" if both else "cross only",
                         a_l, self._a1e_k, "%+.0f" % c.along if c else "-", "%+.0f" % c.cross if c else "-",
                         "%.0f m / %.0f m/s" % (self._a1e_ign["h"], self._a1e_ign["speed"]) if self._a1e_ign else "-"))
        return d, 0.0

    def ident_window(self, h):
        """IDENT flight: after the planned entry burn, not burning, above the floor."""
        return self.ident and self.entry_done and not self.entry_burn and h > self.ident_floor

    def ident_command(self, ut, h, q, vz=-1e9):
        """IDENT schedule. Airflow axis from the TRUE velocity; a step, once started, is held to its end (a cut step
        carries no steady-state data) and is always followed by a gap at 0 deg. Automatic abort (straight flight for
        the rest of the window): heat > 0.75, parts lost, |roll rate| > 60 deg/s or measured AoA > 20 deg.
        Returns (pointing direction ENU, commanded AoA deg, q band centre kPa or 0)."""
        air = -unit(np.asarray(self.v_true_enu, float))
        if self._id_abort is None:
            why = None
            if self.heat > 0.75:
                why = "heat %.2f" % self.heat
            elif getattr(self, "_parts_lost", False):
                why = "parts lost"
            elif abs(getattr(self, "_rr_last", 0.0)) > 60.0:
                why = "roll rate %.0f deg/s" % self._rr_last
            elif getattr(self, "_aoa_meas", 0.0) > 20.0:
                why = "AoA %.1f deg" % self._aoa_meas
            if why:
                self._id_abort = why
                self.log("IDENT ABORT (%s) at h=%.0fm q=%.1fkPa: straight for the rest of the window" % (why, h, q / 1e3))
        qk = q / 1e3
        band = next((c for c, lo, hi in self.ident_bands if lo <= qk <= hi), 0.0)
        if self._id_abort is not None:
            self._id_cmd, self._id_end = 0.0, -1e9
        elif ut >= self._id_end:
            if self._id_cmd != 0.0:              # step over: back to the airflow axis for the gap
                self._id_cmd, self._id_end = 0.0, ut + self.ident_gap
            elif band and (h - self.ident_floor) / max(-vz, 1.0) > self.ident_hold:   # never start a step the floor cuts
                k = self._id_k.get(band, 0)
                self._id_k[band] = k + 1
                steps = self.ident_band_steps.get(band, self.ident_steps)
                self._id_cmd, self._id_end = steps[k % len(steps)], ut + self.ident_hold
                self.log("IDENT step %.0f deg (band %g kPa) h=%.0fm q=%.1fkPa aoa now %.1f deg" % (
                    self._id_cmd, band, h, qk, getattr(self, "_aoa_meas", float("nan"))))
        if self._id_cmd <= 0.0:
            return air, 0.0, band
        try:      # true local vertical, perpendicular to the airspeed (same basis as the belly / entry predictor)
            up_ = unit(np.asarray(self.frame.vec(self.c_basis()[0]), float))
        except Exception:
            up_ = np.array([0.0, 0.0, 1.0])
        w = unit(up_ - np.dot(up_, air) * air)
        return unit(air + self.ident_sign * math.tan(math.radians(self._id_cmd)) * w), self._id_cmd, band

    # --- sensing ---
    def state(self):
        """Pad-frame state. Height and vertical speed are radial (exact far from the pad, where the
        flat ENU frame is not); horizontal components come from the local ENU frame."""
        pb, vb = np.asarray(self.pos()), np.asarray(self.vel())
        r = self.frame.pos(pb)
        v = self.frame.vec(vb)
        self.v_true_enu = v.copy()        # undistorted velocity (pure rotation) for pointing
        up = pb / np.linalg.norm(pb)
        r[2] = float(np.linalg.norm(pb) - np.linalg.norm(self.frame.origin)) - self.h_com
        v[2] = float(np.dot(vb, up))
        # self.h_gnd: height of the leg feet above the REAL ground (CA-3). On the pad the pad is the ground;
        # off it, the radar (CoM height above terrain / sea) minus h_com. r[2] stays the guidance height:
        # min(pad level, radar), so it is never above h_gnd (lower ground beside an elevated pad: r[2] < 0
        # while h_gnd > 0 -- the legs are still in the air). nan = radar unavailable off the pad.
        self.h_gnd = float(r[2])
        if (self.offsite or np.hypot(r[0], r[1]) > 30) and r[2] < 3000:
            # off the pad the ground is not at pad level (sea, slopes): use radar altitude (an offset target has no
            # pad structure: radar right down to the target, as on the CA-3 off-pad landings)
            try:
                self.h_gnd = float(self.radar()) - self.h_com
                r[2] = min(r[2], self.h_gnd)
            except Exception:
                self.h_gnd = float("nan")
        return r, v, self.mass()

    def update_drag_scale(self, r, dmag):
        """Online model correction: k = measured drag / table drag for the current brake state."""
        if os.environ.get("KSP_ABL_NODRAGLEARN") == "1":   # EXPLORATORY ablation (d): k stays at nominal 1.0
            return
        state = bool(self._brakes())
        if self.dm_cont or self.belly:      # partial deploy / partial tilt: learn only at the ends of the range
            f = getattr(self, "_dm_f", 1.0)
            if 0.05 < f < 0.95:
                return
            state = f >= 0.95
        tab = self.drag_out if state else self.drag_in
        if tab is None:
            return
        try:
            spd = float(np.linalg.norm(self.vel()))
            alt = r[2] + self.h_com + self.alt0
            model = tab(alt, spd)
            q = 0.5 * atmo.density(alt) * spd * spd
            steady = (self._age(getattr(self, "_brake_change", 0)) > self.brake_settle_s and self._throttle() < 0.01
                      and self._age(getattr(self, "_last_thrust", 0)) > 2)
            if self._throttle() >= 0.01:
                self._last_thrust = self._stamp()
            if q > 30 and model > 1.0 and steady:      # never learn drag while the engine runs
                k = self.k_drag[state]
                self.k_drag[state] = k + 0.02 * (min(max(dmag / model, 0.2), 6.0) - k)
        except Exception:
            pass

    def scaled(self, tab, k):
        return lambda alt, v, mach=None: k * tab(alt, v, mach)

    def latlon(self):
        try:
            from . import entry
            lat, lon, _ = entry.bf_to_latlonalt(self.v.position(self.body.reference_frame) if self._tel is None
                                                else self.pos())     # rec_v2: frame.rf = body frame stream
            return lat, lon
        except Exception:
            return 0.0, 0.0

    def update_cda(self, r, vel, dmag):
        sp2 = float(np.dot(vel, vel))
        rho = atmo.density(self.alt0 + max(r[2], 0.0))
        if sp2 > 400 and rho > 1e-4 and dmag > 0:
            self.cda += 0.1 * (dmag / (0.5 * rho * sp2) - self.cda)

    def base_cfg(self):
        isp = max((e.specific_impulse for e in self.v.parts.engines if e.active), default=300.0) or 300.0
        return pdg.PDGConfig(t_max=max(self.thrust(), 1.0), isp=isp, m_dry=self.dry, g=self.g, cda=self.cda,
                             v_td=self.hs.v_td, throttle=(0.3, 1.0), tilt_max_deg=20, glideslope_deg=10, n=40)

    def sense(self, ut, vel, m, dmag):
        """Thrust-scale estimate from measured acceleration (as in the GNC closed-loop tests)."""
        if self.prev is not None:
            t0, v0, a_cmd = self.prev
            dt = ut - t0
            if dt > 1e-3:
                vm = (vel + v0) / 2
                a = (vel - v0) / dt + np.array([0, 0, self.g])
                a += dmag / m * vm / max(np.linalg.norm(vm), 1e-6)
                self.est.update(a, a_cmd, dt)
                # unmodelled acceleration (aero side force), perpendicular to the thrust axis
                th = unit(a_cmd) if np.linalg.norm(a_cmd) > 1e-6 else np.array([0, 0, 1.0])
                res = a - a_cmd * self.est.eta
                res -= np.dot(res, th) * th
                k = min(1.0, dt / 1.0)
                self.a_dist += k * (res - self.a_dist)
        return self.est.eta

    def glide_dir(self, r, vel, h, vz, cq, sp, air, h_ign=None, a_meas=None, ut=None):
        """Final-coast aerodynamic steering: tilt off the airflow so the body normal force (a = -c q beta)
        pulls the zero-effort miss to the pad. Returns the pointing direction (ENU)."""
        dirs = os.environ.get("KSP_SYSID_DIRS")     # e.g. "E,W": fixed tilt E above 1600 m, W below (to 600 m)
        if dirs and 600.0 < h < 2600.0:
            d1, d2 = dirs.split(",")
            b = {"E": (1.0, 0.0), "W": (-1.0, 0.0), "N": (0.0, 1.0), "S": (0.0, -1.0)}[d1 if h > 1600.0 else d2]
            return unit(air + math.sin(math.radians(self.sysid_deg)) * np.array([b[0], b[1], 0.0]))
        if self.sysid and 1200.0 < h < 3900.0:
            # system identification: hold a fixed tilt off the airflow (east, then north) and log the side
            # force AND the attitude error (did the tilt actually happen?)
            east = h > 2550.0
            b = np.array([1.0, 0.0]) if east else np.array([0.0, 1.0])
            if a_meas is not None and int(h) % 100 < 10:
                self.log("SYSID h %.0f beta %s %.0fdeg  a_lat %+.3f %+.3f m/s^2  c_model %.5f" % (
                    h, "E" if east else "N", self.sysid_deg, a_meas[0], a_meas[1], self.aero.c))
            return unit(air + math.sin(math.radians(self.sysid_deg)) * np.array([b[0], b[1], 0.0]))
        if self.glide_max_deg <= 0 or not (300.0 < h < self.glide_below) or abs(cq) <= 0.05 or sp <= 30:
            return air
        tau = max(float(np.linalg.norm(vel)), 1.0) / self.g
        if self.glide_zev:
            # ZEM/ZEV law to the START of the landing burn (~h_ign above the pad): arrive over the pad
            # with no horizontal velocity, so the burn (which cannot steer at high q) has nothing to fix.
            # Drag decays v_h with tau = |v| / g:  r_end = r + v tau (1 - e^-t/tau),  v_end = v e^-t/tau
            h_end = self.glide_h_ign if h_ign is None else h_ign      # predicted ignition height
            t_go = max((h - h_end) / max(-vz, 1.0), self.glide_tmin)   # floor: no gain blow-up
            k = math.exp(-t_go / tau)
            zem = r[:2] + vel[:2] * tau * (1.0 - k)
            zev = vel[:2] * k
            a_des = -6.0 * zem / (t_go * t_go) + 2.0 * zev / t_go
        else:
            t_go = h / max(-vz, 1.0)
            drift = tau * (1.0 - math.exp(-t_go / tau)) if self.glide_drag_zem else t_go
            zem = r[:2] + vel[:2] * drift                 # predicted miss, no further control
            a_des = -2.0 * zem / (t_go * t_go)
        if self.glide_ki > 0 and a_meas is not None and ut is not None:
            # acceleration loop: integrate (wanted - measured) side acceleration, so a pointing bias of the
            # autopilot or a wrong normal-force coefficient is trimmed out (both are ~0.2 m/s^2 in flight)
            dt = min(ut - self._gl_t, 0.5) if self._gl_t is not None else 0.0
            self._gl_t = ut
            self._gl_int = np.clip(self._gl_int + self.glide_ki * (a_des - a_meas) * dt, -1.5, 1.5)
            a_des = a_des + self._gl_int
        b = self.nf_sign * a_des / cq                 # side force along (+1) / against (-1) the commanded tilt
        bmax = math.sin(math.radians(self.glide_max_deg))
        nb = float(np.linalg.norm(b))
        if nb > bmax:
            b *= bmax / nb
        return unit(air + np.array([b[0], b[1], 0.0]))

    def burn_law(self, r, vel, m, tmax, dmag):
        """One continuous landing-burn law (no mode switches): vertical stopping profile from the
        hoverslam law; lateral E-guidance / ZEM-ZEV to the target with time-to-go from the vertical
        channel, shortened by the attitude lag; below the gate only the lateral velocity is damped."""
        p = self.hs
        h, vz = r[2] - self.gate[2], vel[2]
        sp = float(np.linalg.norm(vel))
        a_drag = -0.5 * dmag / m * vel / sp if sp > 1e-6 else np.zeros(3)   # drag fades as the burn slows us
        a_max = tmax / m
        if h > 1.0:
            a_lo = min(p.a_min_frac * max(a_max - self.g, 0.5), self.a_lo_max)   # high TWR: gentle floor profile
            v_lo = math.sqrt(p.v_td ** 2 + 2 * a_lo * h)
            if -vz >= v_lo:
                az = self.g + (vz * vz - p.v_td ** 2) / (2 * h)
            else:
                az = self.g + a_lo + p.kv * (-v_lo - vz)
            tgo = 2 * h / max(-vz + p.v_td, 1.0)
            # lag-aware cascade: crossover well below 1/lag; lateral speed capped so it can be
            # nulled (half the available lateral accel) before reaching the gate
            a_lat = math.tan(math.radians(p.tilt_max_deg)) * max(az, 0.5 * self.g)
            d = float(np.linalg.norm(r[:2]))
            if d > 1000.0:      # target out of reach: kill horizontal velocity, land where we are
                r = np.array([0.0, 0.0, r[2]]) * 1.0
                d = 0.0
            # lateral speed must be nullable before the commit height (60 m), where position chasing stops
            tgo_c = 2 * max(h - 60.0, 0.0) / max(-vz + p.v_td, 1.0)
            v_cap = min(math.sqrt(a_lat * d), 0.5 * a_lat * max((min(tgo, tgo_c) if self.commit_cap else tgo) - self.lag, 0.0))
            v_cmd = -r[:2] / d * min(self.kp_lat * d, v_cap) if d > 1e-6 else np.zeros(2)
            if h < self.commit_h:   # last seconds: commit, damp lateral velocity, do not chase metres
                # centring speed funnels to zero at touchdown: |v| <= min(3, 0.2 h)
                v_cmd = -self.term_kp * r[:2] if d < 40 else np.zeros(2)
                n = float(np.linalg.norm(v_cmd))
                cap = min(3.0, self.funnel_k * max(h, 0.0))
                if n > cap:
                    v_cmd *= cap / n
                ah = self.term_kv * (v_cmd - vel[:2])
            elif self.lat_law == "zemzev":
                # ZEM/ZEV to the commit height: be over the pad with no lateral speed when position chasing
                # stops. Time-to-go from the vertical profile (constant decel: speed ~ sqrt(height)).
                w = max(-vz, 1.0)
                w_c = w * math.sqrt(self.commit_h / max(h, self.commit_h))
                t_c = 2.0 * max(h - self.commit_h, 0.0) / max(w + w_c, 1.0)
                t_e = max(t_c - 0.5 * self.lag, 1.0)
                ah = -6.0 * r[:2] / (t_e * t_e) - 4.0 * vel[:2] / t_e
            else:
                n = float(np.linalg.norm(v_cmd))
                cap = 0.3 * max(h, 0.0)          # touchdown funnel, all the way down
                if n > cap:
                    v_cmd = v_cmd * (cap / n)
                ah = self.kv_lat * (v_cmd - vel[:2])
            tilt = p.tilt_max_deg if h >= self.commit_h else self.term_tilt
        else:
            # >2 m below pad level (lower ground off the pad): 2.5 m/s, back to the touchdown speed for the
            # last 5 m above the radar ground (CA-3: keep descending until the legs touch)
            hg = getattr(self, "h_gnd", float("nan"))
            az = self.g + p.kv * (-(self.v_final if (r[2] > -2.0 or hg < 5.0) else 2.5) - vz)
            ah = self.gate_kv * (-vel[:2]) - (0.02 * r[:2] if np.hypot(r[0], r[1]) < 15 else 0.0)   # touch down with ~0 drift
            tilt = self.gate_tilt if r[2] > 1.0 else 0.0   # the last metre: perfectly vertical (no tilt on the legs)
        ah_max = math.tan(math.radians(tilt)) * max(az - a_drag[2], 0.5 * self.g)
        n = float(np.linalg.norm(ah))
        if n > ah_max:
            ah *= ah_max / n
        acc = np.array([ah[0], ah[1], az]) - a_drag
        thr, d = pdg._limit(acc, m, tmax, tilt)
        if r[2] < 500:   # never free-fall low down: keep gimbal authority, avoid relighting while tipping
            # floor relative to weight: on a high-TWR stage a fixed 12 % can exceed the weight and hover
            floor = min(self.thr_floor, self.floor_g * m * self.g / max(tmax, 1.0)) if self.floor_g > 0 else self.thr_floor
            thr = max(thr, floor)
        return thr, d

    def cross_glide(self, ut, air, q, m):
        """High-altitude cross-track steering with the grid fins (glide_below .. xglide_top): every second the
        full drag predictor gives the cross-track miss and, by a 1 m/s sideways test perturbation, its
        sensitivity; the side acceleration that removes it over xglide_T seconds is commanded as a fin tilt
        (a = c q beta, measured c). Along-track is left to the drag modulation."""
        from . import deorbit
        if ut - getattr(self, "_xg_t", -1e9) >= 1.0:
            self._xg_t = ut
            rf = self.body.reference_frame if self._tel is None else self.frame.rf
            try:
                r_bf, v_bf = self._r_v_bf(rf)
                dr = self.scaled(self.drag_out, self.k_drag[True]) if self.drag_out else None
                pred = lambda vv: deorbit.impact_correction(r_bf, vv, self._mass_now(), drag=dr,
                                                            cda=None if dr else self.cda_out, target=self.target,
                                                            heading=self.heading, burns=(), **self._tkw)
                up = r_bf / np.linalg.norm(r_bf)
                side = np.cross(up, v_bf)
                side /= np.linalg.norm(side)
                c0, c1 = pred(v_bf), pred(v_bf + side)
                k = c1.cross - c0.cross
                a_side = (-c0.cross / k) / self.xglide_T if abs(k) > 1.0 else 0.0     # m/s^2 sideways
                cq = self.glide_c_per_kg / max(m, 1000.0) * q
                a_vec = a_side * side
                k_al, a_fwd = 0.0, 0.0
                if self.xglide_along:
                    # along-track too ("the belly as the airbrake"): sensitivity of the along miss to 1 m/s more
                    # horizontal speed; tilting forward / back gives the forward / backward body force
                    fwd = v_bf - np.dot(v_bf, up) * up
                    fwd /= max(np.linalg.norm(fwd), 1e-6)
                    c2 = pred(v_bf + fwd)
                    k_al = c2.along - c0.along
                    a_fwd = (-c0.along / k_al) / self.xglide_T if abs(k_al) > 1.0 else 0.0
                    a_vec = a_vec + a_fwd * fwd
                b_vec = self.nf_sign * a_vec / max(cq, 1e-3)
                bmax = math.radians(self.glide_max_deg)
                nb = float(np.linalg.norm(b_vec))
                if nb > bmax:
                    b_vec *= bmax / nb
                beta = float(np.linalg.norm(b_vec))
                self._xg_dir = np.asarray(self.frame.vec(b_vec / max(beta, 1e-9))) * math.tan(beta) if beta > 1e-6 else np.zeros(3)
                if int(ut) % 10 == 0:
                    self.log("cross glide: cross %+.0f m (%.0f m per m/s) along %+.0f m (%.0f) -> side %.3f fwd %.3f m/s^2, tilt %.1f deg" % (
                        c0.cross, k, c0.along, k_al, a_side, a_fwd, math.degrees(beta)))
            except Exception as e:
                self.log("cross glide error:", repr(e))
        return unit(air + getattr(self, "_xg_dir", np.zeros(3)))

    def ap_err(self, act):
        """Pointing error (deg): the autopilot's, or ours while the own attitude controller flies."""
        if self._own is None:
            try:
                return act.ap.error if self._tel is None else self._tel["aperr"]()
            except Exception:
                return 0.0
        d = getattr(self, "_last_cmd_dir", None)
        if d is None:
            return 0.0
        n = np.asarray(self.frame.vec(self.nose()))
        return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(n, unit(np.asarray(d, float))))))))

    def c_hat(self):
        """Body normal-force coefficient for the fins-only glide: the clean flight fit (1.3e-4, tools/aero_truth.py)
        unless KSP_CLEARN=1 (the online RLS jumped 4x mid-glide once)."""
        if os.environ.get("KSP_CLEARN", "0") == "1" and self.aero.confident:
            return min(max(abs(self.aero.c), 2e-4), 9e-4)
        return self.c_lift0

    def c_basis(self):
        """Lift basis in the body frame, same definition as entry._make_rhs: (vertical part perpendicular to
        the airspeed, unit(airspeed x up))."""
        r_bf, v_bf = np.asarray(self.pos(), float), np.asarray(self.vel(), float)
        vh, up = unit(v_bf), unit(r_bf)
        return unit(up - np.dot(up, vh) * vh), unit(np.cross(vh, up)), r_bf, v_bf

    def ctl_inputs(self, v):
        """Pitch / yaw / roll control inputs the autopilot is sending (saturation = +-1: attitude authority
        limited by the vehicle, not by the controller). Instrumentation for the replay sim (replay-simulator notes)."""
        try:
            if self._tel is not None:
                return float(self._tel["pitch"]()), float(self._tel["yaw"]()), float(self._tel["roll"]())
            c = v.control
            return float(c.pitch), float(c.yaw), float(c.roll)
        except Exception:
            return float("nan"), float("nan"), float("nan")

    def heat_ratio(self, v):
        """Max over parts of temperature / max temperature (internal and skin), from streams."""
        parts = v.parts.all if self._tel is None or getattr(self, "_parts_list", None) is None else self._parts_list
        if len(parts) != getattr(self, "_heat_n", -1):
            for st in getattr(self, "_heat_st", []):
                try:
                    st[0].remove()
                    st[1].remove()
                except Exception:
                    pass
            self._heat_st = [(self.conn.add_stream(getattr, p, "temperature"),
                              self.conn.add_stream(getattr, p, "skin_temperature"),
                              p.max_temperature, p.max_skin_temperature, p.title) for p in parts]
            self._heat_n = len(parts)
        best, who = 0.0, ""
        for t, ts, tm, tsm, name in self._heat_st:
            r = max(t() / max(tm, 1.0), ts() / max(tsm, 1.0))
            if r > best:
                best, who = r, name
        if best > getattr(self, "_heat_peak", 0.0) + 0.05:
            self._heat_peak = best
            self.log("heat: %.2f of max temperature (%s)" % (best, who))
        return best

    def _abl_setup(self):
        """EXPLORATORY ablation / dispersion switches (report Section 4.8). They are not part of the
        guidance. All default OFF; with every KSP_ABL_* variable unset
        the default path is bit-identical (tests/test_ablation.py).
          KSP_ABL_NOBRAKES=1     (a) airbrakes never deployed in the descent (set_brakes holds the Brakes group off;
                                 the grid fins stay held out by extend_grid_fins as on the baseline)
          KSP_ABL_NODRAGLEARN=1  (d) in-flight drag scale k_drag fixed at nominal 1.0 (update_drag_scale)
          KSP_ABL_NAVNOISE=sp,sv Gaussian noise on the navigation state guidance reads (position sp m, velocity
                                 sv m/s, per axis), re-drawn each control tick; KSP_ABL_SEED seeds it
          KSP_ABL_DRAGBIAS=k     the guidance drag model (drag tables, CdA out / in) scaled by k (1.2 = +20 %)
        Existing switches used by the ablation: KSP_GLIDE_MAX=0 (b, no glide), KSP_NOGUIDE=1 + KSP_NOVERNIER=1 (c)."""
        env = os.environ.get
        if env("KSP_ABL_NOBRAKES") == "1":
            self.set_brakes(False)
            self.log("EXPLORATORY KSP_ABL_NOBRAKES: airbrakes held retracted")
        if env("KSP_ABL_DRAGBIAS"):
            k = float(env("KSP_ABL_DRAGBIAS"))
            for a in ("drag_out", "drag_in"):
                if getattr(self, a, None) is not None:
                    setattr(self, a, self.scaled(getattr(self, a), k))
            for a in ("cda_out", "cda_in"):
                if getattr(self, a, None) is not None:
                    setattr(self, a, k * getattr(self, a))
            self.log("EXPLORATORY KSP_ABL_DRAGBIAS: guidance drag model x %.3f" % k)
        if env("KSP_ABL_NAVNOISE"):
            sp, sv = (float(x) for x in env("KSP_ABL_NAVNOISE").split(","))
            seed = int(env("KSP_ABL_SEED")) if env("KSP_ABL_SEED") else None
            rng = np.random.default_rng(seed)
            pos0, vel0, state0 = self.pos, self.vel, self.state
            self._abl_nav = (pos0, vel0)
            self._abl_draw = (rng.normal(0.0, sp, 3), rng.normal(0.0, sv, 3))

            def state_nn():                 # one draw per control tick: the loop reads state() once per tick
                self._abl_draw = (rng.normal(0.0, sp, 3), rng.normal(0.0, sv, 3))
                return state0()
            self.state = state_nn
            self.pos = lambda: np.asarray(pos0(), float) + self._abl_draw[0]
            self.vel = lambda: np.asarray(vel0(), float) + self._abl_draw[1]
            self.log("EXPLORATORY KSP_ABL_NAVNOISE: sigma pos %.2f m vel %.3f m/s per axis, seed %s" % (sp, sv, seed))

    def _abl_true_nav(self):
        """End of KSP_ABL_NAVNOISE: true streams again (touchdown / settle measured without noise)."""
        nav = self.__dict__.pop("_abl_nav", None)
        if nav is not None:
            self.pos, self.vel = nav
            self.__dict__.pop("state", None)

    def extend_grid_fins(self, want=True):
        """KRE grid fins deploy through ModuleAnimateGeneric bound to the Brakes group as a TOGGLE, so every
        airbrake change flips them (their state became random: 0.8 m vs 100 m flights). Hold them in the wanted
        state (out, except while they are the drag modulator of a craft without airbrakes)."""
        if self._tel is not None:
            return self._extend_grid_fins_v2(want)
        try:
            n = 0
            ev = "Extend Fins" if want else "Retract Fins"
            for p in self.v.parts.all:
                if "Grid Fin" not in p.name:
                    continue
                for m in p.modules:
                    if m.name == "ModuleAnimateGeneric" and m.has_event(ev):
                        m.trigger_event(ev)
                        n += 1
            if n:
                self.log("grid fins: %s %d" % ("re-extended" if want else "retracted", n))
        except Exception as e:
            self.log("grid fins error:", repr(e))

    def _extend_grid_fins_v2(self, want):
        """rec_v2 (REC_FULL) only: the same hold with the fins' ModuleAnimateGeneric modules found ONCE (the scan
        reads every part's name and every fin module's name: ~40 RPCs every 2 s, ~1.3 game s while recording);
        re-scanned when the part count changes. Per call: one has_event per fin (+ the triggers)."""
        try:
            npart = getattr(self, "_parts_n", None)
            if getattr(self, "_fin_mods", None) is None or npart != self._fin_mods_n:
                self._fin_mods = [m for p in self.v.parts.all if "Grid Fin" in p.name for m in p.modules
                                  if m.name == "ModuleAnimateGeneric"]
                self._fin_mods_n = npart
            n = 0
            ev = "Extend Fins" if want else "Retract Fins"
            for m in self._fin_mods:
                if m.has_event(ev):
                    m.trigger_event(ev)
                    n += 1
            if n:
                self.log("grid fins: %s %d" % ("re-extended" if want else "retracted", n))
        except Exception as e:
            self._fin_mods = None
            self.log("grid fins error:", repr(e))

    def entry_guidance(self, ut):
        """Closed-loop cross-track steering DURING the entry burn (the engine gimbal gives the authority the
        coasting stage lacks): every second predict the cross-track miss if the rest of the burn is flown
        retrograde, and tilt the burn sideways by atan(dv_side / dv_left). As dv_left shrinks the correction
        sharpens, so the burn's own pointing error is closed out too."""
        if ut - getattr(self, "_eg_t", -1e9) < 1.0:
            return
        self._eg_t = ut
        from . import deorbit
        from .entry import Burn
        rf = self.body.reference_frame if self._tel is None else self.frame.rf
        try:
            left = max(self.entry_dv - self.entry_flown, 0.0)
            if left < 30.0:                      # last moments: hold the tilt, no gain blow-up
                return
            r_bf, v_bf = self._r_v_bf(rf)
            dr = self.scaled(self.drag_out, self.k_drag[True]) if self.drag_out else None
            isp = self._isp_vac()
            burns = (Burn(t_start=0.0, thrust=max(self.thrust(), 1.0), isp=isp, mode="air_retro", dv=left),)
            pred = lambda vv: deorbit.impact_correction(r_bf, vv, self._mass_now(), drag=dr,
                                                        cda=None if dr else self.cda_out, target=self.target,
                                                        heading=self.heading, burns=burns, **self._tkw)
            up = r_bf / np.linalg.norm(r_bf)
            side = np.cross(up, v_bf)
            side /= np.linalg.norm(side)
            c0, c1 = pred(v_bf), pred(v_bf + side)
            k = c1.cross - c0.cross
            if abs(k) < 1.0:
                return
            dv_side = -c0.cross / k
            tilt = max(-3.0, min(3.0, math.degrees(math.atan2(dv_side, left))))
            side_enu = self.frame.vec(side)                # frame.rf is the body frame: same axes
            self._eb_offset = np.asarray(side_enu) * math.tan(math.radians(tilt))
            self.log("entry guidance: cross %+.0f m, %.0f m/s left -> tilt %+.2f deg" % (c0.cross, left, tilt))
        except Exception as e:
            self.log("entry guidance error:", repr(e))

    def plan_vernier(self, ut):
        """Right after the entry burn (engine still lit, gimbal authority): plan a sideways vernier that
        nulls the predicted cross-track miss. Returns the state dict for the control loop, or None."""
        from . import deorbit
        rf = self.body.reference_frame if self._tel is None else self.frame.rf
        try:
            r_bf, v_bf = self._r_v_bf(rf)
            dr = self.scaled(self.drag_out, self.k_drag[True]) if self.drag_out else None
            pred = lambda vv: deorbit.impact_correction(r_bf, vv, self._mass_now(), drag=dr,
                                                        cda=None if dr else self.cda_out, target=self.target,
                                                        heading=self.heading, burns=(), **self._tkw)
            up = r_bf / np.linalg.norm(r_bf)
            side = np.cross(up, v_bf)
            side /= np.linalg.norm(side)
            c0, c1 = pred(v_bf), pred(v_bf + side)
            k = c1.cross - c0.cross
            dv = -c0.cross / k if abs(k) > 1.0 else 0.0
            self.log("vernier plan: cross %+.0f m (along %+.0f), %.1f m per m/s -> %.2f m/s sideways" % (
                c0.cross, c0.along, k, dv))
            if abs(c0.cross) < self.trim_min_m or abs(dv) > self.trim_max_dv:
                return None
            side_enu = np.asarray(self.frame.vec(side)) * math.copysign(1.0, dv)
            return dict(side=side_enu, dv=abs(dv), got=0.0, t0=ut, t_last=None)
        except Exception as e:
            self.log("vernier plan error:", repr(e))
            return None

    def post_entry_trim(self, act):
        """After the entry burn (q ~1 kPa, attitude easy): one small burn that nulls the predicted
        CROSS-track miss. The entry burn's pointing error (0.1 deg of 590 m/s ~ 1 m/s sideways) leaves
        +-50 m cross-track scatter that nothing later can remove (drag modulation only moves along-track).
        Sensitivity measured with a test perturbation (secant), so no gain is assumed."""
        from . import deorbit
        from .mission import burn_vector
        rf = self.body.reference_frame if self._tel is None else self.frame.rf
        try:
            r_bf, v_bf = self._r_v_bf(rf)
            dr = self.scaled(self.drag_out, self.k_drag[True]) if self.drag_out else None
            pred = lambda vv: deorbit.impact_correction(r_bf, vv, self._mass_now(), drag=dr,
                                                        cda=None if dr else self.cda_out, target=self.target,
                                                        heading=self.heading, burns=(), **self._tkw)
            c0 = pred(v_bf)
            up = r_bf / np.linalg.norm(r_bf)
            side = np.cross(up, v_bf)
            side /= np.linalg.norm(side)               # horizontal, perpendicular to the track
            c1 = pred(v_bf + side * 1.0)
            k = c1.cross - c0.cross                     # metres of cross-track per m/s sideways
            if abs(k) < 1.0:
                self.log("post-entry trim: no cross sensitivity (%.2f m per m/s), skipped" % k)
                return
            dv = -c0.cross / k
            self.log("post-entry trim: predicted cross %+.0f m (along %+.0f), %.1f m per m/s -> %.2f m/s sideways" % (
                c0.cross, c0.along, k, dv))
            if abs(c0.cross) < self.trim_min_m or abs(dv) > self.trim_max_dv:
                return
            # do not swing 90 deg sideways (it cannot align in time): stay near retrograde and tilt only
            # trim_tilt_deg towards the side; the extra retro part is absorbed by the drag modulation
            retro = -v_bf / np.linalg.norm(v_bf)
            t = math.radians(self.trim_tilt_deg)
            u = retro * math.cos(t) + side * math.copysign(math.sin(t), dv)
            mag = abs(dv) / math.sin(t)
            self.log("post-entry trim: %.2f m/s at %.0f deg off retrograde" % (mag, self.trim_tilt_deg))
            self._hb_hold = True                       # an intended blocking burn: not a stall
            try:
                burn_vector(self.conn, self.v, u * mag, self.log, rf, align_timeout=5)   # never block the loop long
            finally:
                self._hb_hold = False
                self._hb_beat()
        except Exception as e:
            self.log("post-entry trim error:", repr(e))
        finally:                                        # hand the attitude back to the lander's actuator
            act.ap.reference_frame = self.frame.rf
            act.ap.time_to_peak = (self.ttp, self.ttp, self.ttp)
            act.ap.max_angular_velocity = (0.35, 0.35, 0.35)
            act.ap.engaged = True

    def airbrake_angle_fields(self):
        """PartFields of the airbrakes' deploy angle (ModuleAeroSurface.aeroDeployAngle, 0..70 deg). This, not
        the authority limiter, sets how far they open (probe 2026-10-20: CdA 13 / 24 / 26 m^2 at 0 / 35 / 70 deg)."""
        if getattr(self, "_ab_fields", None) is None:
            self._ab_fields = [f for p in self.v.parts.all if "airbrake" in p.name for m in p.modules
                               if m.name == "ModuleAeroSurface" for f in m.field_list if f.name == "aeroDeployAngle"]
        return self._ab_fields

    def set_brakes(self, state):
        """The ONLY way to change the Brakes action group (CA-2): every state change, from any path (q-safety,
        below-glide force-out, drag modulation, landing burn), stamps _brake_change so update_drag_scale skips
        the deploy/retract transient."""
        state = bool(state)
        if state and os.environ.get("KSP_ABL_NOBRAKES") == "1":   # EXPLORATORY ablation (a): never deployed
            state = False
        if bool(self._brakes()) != state:
            self._ctlw().brakes = state
            if self._tel is not None:
                self._brk_w = (self._tick, state)
            self._brake_change = self._stamp()
            return True
        return False

    def _stamp(self):
        """CA-2 settle stamp: time.time() as before; recording v2 (rec_v2, HOP_REC=full only): a game-clock mark
        (core.GameClock on the lander's UT stream: the game runs at ~0.4x while recording, decision log 11:06)."""
        return time.time() if not getattr(self, "rec_v2", False) else self._gclock().mark()

    def _age(self, stamp):
        """Seconds since a _stamp(): time.time() - stamp as before; rec_v2: game seconds (3x wall cap)."""
        return time.time() - stamp if not getattr(self, "rec_v2", False) else self._gclock().elapsed(stamp)

    def _gclock(self):
        if getattr(self, "_gc", None) is None:
            self._gc = core.GameClock(self.ut)
        return self._gc

    # --- DR-010 item 3: per-tick I/O. REC_FULL (rec_v2) reads streams; everything else is the old RPC, unchanged ---
    _tel = None            # rec_v2 only: dict of kRPC streams for the per-tick reads (None = old per-tick RPCs)

    def _io_setup(self, act):
        """REC_FULL only. While recording, kRPC serves ~1 RPC per game frame (fps ~12), so each per-tick RPC costs a
        frame: the loop paid ~40-100 per tick (0.5-1.2 game s, DEADMAN stalls; decision log 14:55). Streams are pushed
        by the server every frame at no RPC cost; writes go through one cached Control object."""
        c, v = self.conn, self.v
        ctl = v.control
        self._ctl = ctl
        self._tel = dict(
            thr=c.add_stream(getattr, ctl, "throttle"), brk=c.add_stream(getattr, ctl, "brakes"),
            pitch=c.add_stream(getattr, ctl, "pitch"), yaw=c.add_stream(getattr, ctl, "yaw"),
            roll=c.add_stream(getattr, ctl, "roll"), thrust=c.add_stream(getattr, v, "thrust"),
            pwarp=c.add_stream(getattr, self.sc, "physics_warp_factor"),
            rwarp=c.add_stream(getattr, self.sc, "rails_warp_factor"),
            aperr=c.add_stream(getattr, act.ap, "error"),
            rollang=c.add_stream(getattr, v.flight(v.surface_reference_frame), "roll"))
        act.ctl, act.thr_s = ctl, self._tel["thr"]      # flightctl.set_thrust: cached Control, skip unchanged writes
        self._tick, self._thr_w, self._brk_w = 0, None, None

    def _ctlw(self):
        """Control object for a write: rec_v2 the cached one (1 RPC per write instead of 2)."""
        return self.v.control if self._tel is None else self._ctl

    def _throttle(self):
        """Current throttle. rec_v2: the stream, or the value written earlier in THIS tick (a stream shows a write
        only a frame later; reads at the start of a tick are a few frames after the last write)."""
        if self._tel is None:
            return self.v.control.throttle
        w = self._thr_w
        return w[1] if w is not None and w[0] == self._tick else self._tel["thr"]()

    def _brakes(self):
        if self._tel is None:
            return self.v.control.brakes
        w = self._brk_w
        return w[1] if w is not None and w[0] == self._tick else self._tel["brk"]()

    def _r_v_bf(self, rf):
        """Body-frame position / velocity for the predictors (frame.rf IS body.reference_frame: rec_v2 uses the
        loop's own streams)."""
        if self._tel is None:
            return np.asarray(self.v.position(rf)), np.asarray(self.v.velocity(rf))
        return np.asarray(self.pos()), np.asarray(self.vel())

    def _mass_now(self):
        return self.v.mass if self._tel is None else self.mass()

    def _isp_vac(self):
        """Max vacuum Isp of the active engines (predictor burns). rec_v2: read once (2n+1 RPCs) and cached."""
        if self._tel is not None and getattr(self, "_isp_vac_c", None) is not None:
            return self._isp_vac_c
        isp = max((e.vacuum_specific_impulse for e in self.v.parts.engines if e.active), default=300.0)
        if self._tel is not None:
            self._isp_vac_c = isp
        return isp

    def _legs_grounded(self):
        if self._tel is None:
            return any(l.is_grounded for l in self.v.parts.legs)
        if getattr(self, "_legs_s", None) is None:
            self._legs_s = [self.conn.add_stream(getattr, l, "is_grounded") for l in self.v.parts.legs]
        return any(s() for s in self._legs_s)

    # --- DR-010 item 1: entry-burn dv from the propellant actually burned (all modes) ---
    def _isp_now(self):
        """Current (pressure-dependent) specific impulse of the active engines, read once at burn start."""
        try:
            isp = float(self.v.specific_impulse)
        except Exception:
            isp = 0.0
        if not 50.0 < isp < 2000.0:
            try:
                isp = float(max((e.vacuum_specific_impulse for e in self.v.parts.engines if e.active), default=0.0))
            except Exception:
                isp = 0.0
        return isp if 50.0 < isp < 2000.0 else 300.0

    def _eb_start(self, m):
        self._eb_m0 = self._eb_m = float(m)
        self._eb_isp = self._isp_now()
        self._eb_c = self._eb_isp * 9.80665
        self._eb_thr_sum, self._eb_capped = 0.0, False

    def _dv_mass(self, m_prev, m, dt):
        """Rocket-equation dv over one tick, Isp g0 ln(m_prev / m), from the measured mass (the ut and mass streams
        are sampled together). Unlike a thrust sample x tick length it cannot lose a tick whose sample reads 0 (the
        DEADMAN cut: flight #4 lost ~120 m/s, decision log 14:55) nor count one twice; the ln telescopes, so the sum
        is exactly Isp g0 ln(m0 / m). Capped at 1.25 x the engine's full-thrust dv for the tick (+0.5 m/s): a part
        lost mid-burn is mass, not dv."""
        if m_prev is None or m <= 0.0 or m >= m_prev:
            return 0.0
        c = getattr(self, "_eb_c", None) or self._isp_now() * 9.80665
        dv = c * math.log(m_prev / m)
        cap = 1.25 * max(float(self.thrust()), 1.0) / m_prev * max(dt, 0.0) + 0.5
        if dv > cap:
            if not getattr(self, "_eb_capped", False):
                self._eb_capped = True
                self.log("dv accounting: mass drop %.0f kg in %.2f s exceeds the engine's flow (part lost?): "
                         "%.1f m/s counted, not %.1f" % (m_prev - m, dt, cap, dv))
            dv = cap
        return dv

    def set_dm_fraction(self, f):
        """Continuous drag modulation: keep the Brakes group deployed (grid fins stay out and keep steering)
        and set the airbrakes' deploy angle to f x 70 deg."""
        f = float(min(1.0, max(0.0, f)))
        self.set_brakes(True)
        if self.belly:
            self._dm_f = self._belly_f = f
            return
        if abs(f - getattr(self, "_dm_f", -1.0)) < 0.02:
            return
        self._dm_f = f
        self._brake_change = self._stamp()
        try:
            for fld in self.airbrake_angle_fields():
                fld.float_value = 70.0 * f
        except Exception as e:
            self.log("airbrake angle error:", repr(e))

    def drag_modulation(self, ut):
        """Entry range control with the airbrakes: deploy when even full drag lands long or the
        prediction with brakes out is long, retract when brakes-out would land short."""
        if ut - self._dm_t < 1.0 or self.cda_in is None:
            return
        self._dm_t = ut
        from . import deorbit
        rf = self.body.reference_frame if self._tel is None else self.frame.rf
        try:
            r_bf, v_bf = self._r_v_bf(rf)
            def pred(tab, k, cda):
                dr = self.scaled(tab, k) if tab else None
                return deorbit.impact_correction(r_bf, v_bf, self._mass_now(), drag=dr, cda=None if dr else cda,
                                                 target=self.target, heading=self.heading, burns=burns, **self._tkw)
            burns = ()
            if not self.entry_done and self.entry_dv > 0:
                from .entry import Burn
                isp = self._isp_vac()
                burns = (Burn(t_start=0.0, thrust=max(self.thrust(), 1.0), isp=isp, mode="air_retro",
                              dv=self.entry_dv, alt_trigger=self.entry_alt),)
            c_out = pred(self.drag_out, self.k_drag[True], self.cda_out)
            c_in = pred(self.drag_in, self.k_drag[False], self.cda_in) if (self.drag_in or self.cda_in) else None
        except Exception as e:
            self.log("drag modulation error:", repr(e))
            return
        if self.draw:
            self.draw_prediction(c_out.impact)
        last = getattr(self, "_dm_last", None)
        if last is not None and abs(c_out.along - last) > 2000 and not getattr(self, "_dm_skipped", False):
            self._dm_skipped = True          # one-sample outlier guard
            return
        self._dm_skipped, self._dm_last = False, c_out.along
        try:   # HUD convergence view: predicted touchdown in pad E/N for the airbrake state now flown
            c_now = c_out if (self._brakes() or c_in is None) else c_in
            if self.dm_cont and c_in is not None:
                fr = getattr(self, "_dm_f", 1.0)
                al, cr = fr * c_out.along + (1 - fr) * c_in.along, fr * c_out.cross + (1 - fr) * c_in.cross
            else:
                al, cr = c_now.along, c_now.cross
            hd = math.radians(c_out.heading)
            self._pred_en = (al * math.sin(hd) + cr * math.cos(hd), al * math.cos(hd) - cr * math.sin(hd), ut)
        except Exception:
            pass
        self._pred_miss = math.hypot(c_out.along + self.aim_offset, c_out.cross)   # vs the aim point
        alt_now = float(np.linalg.norm(r_bf)) - self.body.equatorial_radius
        tgt = -self.aim_offset            # learned systematic bias (m along track)
        if c_out.along > tgt + 150 and alt_now > 6000 and self.entry_done:
            # long even with full drag: shorten with a short retro pulse (engine already points forward)
            self.retro_until = ut + 0.5
            self._pulses = getattr(self, "_pulses", 0) + 1
            if self._pulses % 5 == 1:
                self.log("range trim: retro pulse #%d (out %+.0fm)" % (self._pulses, c_out.along))
        if self.belly and c_in is not None and self.belly_dir == "side":
            # roll reversal: if the cross-track miss keeps growing for 6 s, put the belly lift on the other side
            self._belly_hist.append((ut, c_out.cross))
            self._belly_hist = [x for x in self._belly_hist if ut - x[0] <= 6.0]
            if (len(self._belly_hist) >= 5 and abs(c_out.cross) > 30.0
                    and abs(self._belly_hist[-1][1]) > abs(self._belly_hist[0][1]) + 10.0
                    and ut - getattr(self, "_belly_flip_t", -1e9) > 8.0):
                self._belly_sign *= -1.0
                self._belly_flip_t = ut
                self.log("belly: roll reversal (cross %+.0f m)" % c_out.cross)
        if (self.dm_cont or self.belly) and c_in is not None:
            # deploy fraction that zeroes the predicted miss, linear between the all-in and all-out predictions
            # (theory experiment tools/dm_mc.py: |miss| median 3-7 m vs 600-950 m bang-bang, drag jerk 1/4)
            e_out, e_in = c_out.along - tgt, c_in.along - tgt
            f = 1.0 if e_out >= 0 else 0.0 if e_in <= 0 else e_in / (e_in - e_out)
            old_f = getattr(self, "_dm_f", 1.0)
            self.set_dm_fraction(f)
            if abs(f - old_f) > 0.15 or int(ut) % 10 == 0:
                self.log("drag modulation: airbrakes %3.0f%% (out %+.0fm / in %+.0fm, k_out %.2f k_in %.2f)" % (
                    100 * f, c_out.along, c_in.along, self.k_drag[True], self.k_drag[False]))
            return
        if c_out.along > tgt - self.dm_margin or c_in is None:
            want = c_out.along > tgt - self.dm_margin      # long even with full drag -> brakes out
        elif c_in.along < tgt + self.dm_margin:
            want = False                                   # short even with brakes in -> brakes in
        else:
            want = False      # bracketed: retract until the full-drag prediction is back on target (bang-bang)
        want = bool(want)
        if self.set_brakes(want):
            self.log("drag modulation: brakes %s (out %+.0fm / in %s, k_out %.2f k_in %.2f)" % (
                "OUT" if want else "IN", c_out.along, "%+.0fm" % c_in.along if c_in else "-",
                self.k_drag[True], self.k_drag[False]))

    def _camera_v2(self, ut, r):
        """Recording pipeline v2 lander camera (HOP_REC=full only): showrun.DescentDirector picks the shot (S7 entry
        burn, S8 / side / front rotation, V7 three-camera rotation D1 / D2 / D3 below 15 km, S9 pad lens from the
        1 km hand-off + 3 s, V10 S10 drone camera from the first h_gnd < 60 m). Returns (shot id, shot string)."""
        if getattr(self, "_dd", None) is None:
            self._dd = showrun.DescentDirector()
            self._v2cap = None
        R = 600000.0
        lat0, lon0 = self.target
        glat = lat0 + math.degrees(-160.0 / R)
        glon = lon0 + math.degrees(-380.0 / (R * math.cos(math.radians(lat0))))
        h = float(r[2])
        hg = getattr(self, "h_gnd", float("nan"))
        try:
            vz = float(self.v_true_enu[2])
        except Exception:
            vz = 0.0
        rng = math.sqrt((float(r[0]) + 380.0) ** 2 + (float(r[1]) + 160.0) ** 2 + (h - 12.0) ** 2)
        pre = not self.entry_done and getattr(self, "_tl_t1", 1e9) < showrun.V2_PRE_S7
        txs = hud.current.transitions() if hud.current is not None else []
        done = self.entry_done or (not self.entry_burn and h < showrun.V7_START_H)   # no entry burn flown: V7 all the same
        # V7 pad lens (fix round 2): usable once the zoom can frame the stage >= 15 % of the picture height; latched
        # (the range only shrinks), the band flies the steep chase D3 before that
        self._pad_ok = getattr(self, "_pad_ok", False) or showrun.pad_lens(rng)[2]
        sid = self._dd.step(ut, h, hg, vz, rng, self.entry_burn, pre, done, getattr(self, "_handoff_ut", None), txs,
                            pad_ok=self._pad_ok)
        if sid in ("S9", "S10") and getattr(self, "_drone", None) is None:
            # plan the S10 drone camera as soon as S9 is on air (~8 s ahead of the 60 m cut); the cut itself is then a
            # plain shot.txt write. V14: the drone ORBITS the pad aimed at the vessel (plugin 'orbit' shot, animated per
            # frame by the plugin), sun-safe arc (showrun.drone_orbit_shot); the sun direction is the one the rel
            # shots cached (_sun_rel), else ~7 RPCs once
            try:
                if getattr(self, "_sun_enu", None) is None or ut - self._sun_ut > self.SUN_REFRESH:
                    self._sun_enu, self._sun_ut = core.sun_enu(self.conn, lat0, lon0), ut
                s = self._sun_enu
            except Exception as e:
                s = None                           # fallback: slow arc on the pad camera side
                self.log("camera: sun direction unavailable (%r), drone orbit on the pad camera side" % e)
            self._drone, info = showrun.drone_orbit_shot(lat0, lon0, pdg.PAD_ALT, s)
            self.log("camera: %s: %s" % (showrun.drone_orbit_text(info), self._drone))
        if sid == "S10" and getattr(self, "_sid", None) != "S10":
            self.log("camera: S10 drone view at h_gnd %.1f m (h %.1f m, vz %.1f m/s)" % (hg, h, vz))
        # fix round 3 (QA 4.1: 31 s of the S8 stock chase looked into the sun): S8 is a rel chase too, and every rel
        # shot is placed sun-aware once per shot (_sun_rel) and hard-cut ('cut': no eased swing through the sun)
        cams = {"S7": "onboard 0 3 2.2 75", "D1": "onboard 0 3 2.2 75", "S8": "rel 60 10 50 55 cut",
                "SIDE": "rel 40 0 90 55 cut", "FRONT": "rel 50 -5 160 55 cut", "D3": "rel 50 70 20 60 cut",
                "D2": "ground %.6f %.6f %.1f 14 %.1f %.2f" % (glat, glon, pdg.PAD_ALT + 12.0, showrun.PAD_LENS_FRAME,
                                                             showrun.PAD_LENS_MIN_FOV),
                "S9": "ground %.6f %.6f %.1f 14" % (glat, glon, pdg.PAD_ALT + 12.0)}
        want = self._drone if sid == "S10" else cams.get(sid, "chase")
        want = self._sun_rel(sid, want, ut, r)
        cap = ("PAD TRACKING · KSC", "RANGE %.1f KM" % (rng / 1e3)) if sid == "D2" else \
            ("ONBOARD · DOWNWARD", None) if sid == "D1" else None
        if cap != self._v2cap:
            self._v2cap = cap
            hud.report("set", cam=cap[0] if cap else None, camtel=cap[1] if cap else None, camuntil=None)
        return ("S8" if sid == "S8" else sid), want

    SUN_REFRESH = 600.0    # game s: the sun direction at the pad (~7 RPCs) is read at the first rel cut and re-read only
                           # after this long - it turns with the planet at 0.017 deg/s (< 4 deg over a whole descent),
                           # so the control loop pays for it once

    def _sun_rel(self, sid, cam, ut, r):
        """Recording v2 fix round 3 (rec_v2 only, called from _camera_v2): a rel shot string moved so its view axis
        stays >= 60 deg from the sun (showrun.sun_safe_rel), decided ONCE per shot at its cut (the string never
        changes mid-shot). Geometry: the pad-ENU position / velocity the lander already has + the sun direction at
        the pad (core.sun_enu, cached SUN_REFRESH game s). One log line per shot: the chosen sun angle."""
        if showrun.parse_rel(cam) is None:
            return cam
        if getattr(self, "_sun_sid", None) == sid and getattr(self, "_sun_base", None) == cam:
            return self._sun_out
        self._sun_sid, self._sun_base, self._sun_out = sid, cam, cam
        try:
            if getattr(self, "_sun_enu", None) is None or ut - self._sun_ut > self.SUN_REFRESH:
                lat0, lon0 = self.target
                self._sun_enu, self._sun_ut = core.sun_enu(self.conn, lat0, lon0), ut
            geo = showrun.lander_geo(r, self.v_true_enu, self._sun_enu, R0=showrun.R_KERBIN + pdg.PAD_ALT)
            self._sun_out, info = showrun.sun_safe_rel(cam, geo)
            if info is not None:
                self.log("camera: " + showrun.sun_info_text(sid, info))
        except Exception as e:
            self.log("camera: %s sun geometry unavailable (%r): nominal camera" % (sid, e))
        return self._sun_out

    def direct_camera(self, stage, t_land, r, ut=0.0):
        """Presentation camera. Level chase view during entry. Once the stage is nearly over the landing
        site (the pad would drop out of the level view: line-of-sight depression 35 -> 45 deg) the view
        blends linearly to look past the stage at the pad and holds it until 15 s before touchdown, when
        it blends back to the level view (15 -> 10 s) for the touchdown shot. The AutoLab plugin steers
        the camera closed-loop from the blend factor (cam.txt), so no camera-heading convention is assumed."""
        # director, unified show timeline (kspilot/showrun.py, show-timeline notes): S7 hull camera from ~4 s
        # BEFORE the entry burn to its end; S8 chase / pad view after it (the cut is marked: the entry funnel
        # slides in 2 s later); S9 long-lens ground camera beside the pad from the 1 km hand-off + 3 s through
        # touchdown (the single landing-burn cut), then no further cut
        if self.rec_v2:
            sid, want = self._camera_v2(ut, r)
        else:
            pre = not self.entry_done and getattr(self, "_tl_t1", 1e9) < 6.0
            sid = showrun.landing_shot(ut, self.entry_burn, pre, getattr(self, "_handoff_ut", None), False)
            if sid == "S7":
                want = "onboard 0 3 2.2 75"
            elif sid == "S9":
                R = 600000.0
                lat0, lon0 = self.target
                glat = lat0 + math.degrees(-160.0 / R)
                glon = lon0 + math.degrees(-380.0 / (R * math.cos(math.radians(lat0))))
                want = "ground %.6f %.6f %.1f 14" % (glat, glon, pdg.PAD_ALT + 12.0)
            else:
                want = "chase"
        prev = getattr(self, "_sid", None)
        if sid == "S8" and (prev == "S7" or (prev is None and self.entry_done)):
            hud.report("mark", "S8", ut)           # entry burn over (or a run without one): the funnel follows
        self._sid = sid
        if want != getattr(self, "_shot", None):
            self._shot = want
            core.camera_shot(want)
        to_pad = -np.asarray(r, float)
        dep = math.degrees(math.atan2(-to_pad[2], math.hypot(to_pad[0], to_pad[1])))
        alpha = min(max((dep - 35.0) / 10.0, 0.0), 1.0) * min(max((t_land - 10.0) / 5.0, 0.0), 1.0)
        core.camera_aim(alpha)
        if alpha > 0.0 or self._ov % 20 or getattr(self, "_shot", "chase") != "chase":
            return                          # never touch the camera while a director shot is live
        try:                                    # KSP drops back to orbital/auto camera: keep a level chase view
            cam = self.conn.space_center.camera
            want = 70.0 if r[2] > 3000 else 55.0
            if not str(cam.mode).endswith("free") or abs(cam.pitch - 10.0) > 3 or abs(cam.distance - want) > 5:
                core.set_camera(self.conn, want, 10.0)
        except Exception as e:
            if not getattr(self, "_cam_err", False):
                self._cam_err = True
                self.log("camera:", repr(e))

    def timeline(self, ut, stage, h, vz, m, tmax, margin):
        """Live time-to-go to the start of each mission phase and a continuous progress value
        (phase index + fraction of the way to the next phase) for the HUD timeline."""
        w = max(-vz, 1.0)
        a = max(tmax / max(m, 1.0), 1.0)
        alt = h + self.h_com + self.alt0
        if not self.entry_burn and not self.entry_done:
            t1 = max(alt - self.entry_alt, 0.0) / w
            t2 = t1 + self.entry_dv / a
        elif self.entry_burn:
            t1, t2 = 0.0, max(self.entry_dv - self.entry_flown, 0.0) / a
        else:
            t1 = t2 = 0.0
        if stage >= 3:
            t3 = 0.0
        elif margin is not None:
            t3 = max(margin, 0.0) / w
        else:
            t3 = max(t2, max(h - 500.0, 0.0) / w)
        t_gate = self.gate[2] / max(self.v_final, 0.5)    # final slow descent below the gate
        t4 = (2.0 * max(h - self.gate[2], 0.0) / (w + self.v_final) + t_gate if stage >= 3
              else t3 + 2.0 * min(max(h, 0.0), 500.0) / w + t_gate)
        tts = [0.0, t1, t2, t3, t4]
        self._tl_t1, self._tl_t3, self._tl_t4 = t1, t3, t4
        for i, (name, tt) in enumerate((("ENTRY BURN", t1), ("AERO GLIDE", t2), ("LANDING BURN", t3), ("TOUCHDOWN", t4))):
            if i + 1 > stage:                                # only events still ahead (a run started from a
                hud.report("plan", name, ut + tt)            # checkpoint must not show missed events as "now")
        # HUD v3 timeline arc: seconds from now to each event (past events from the recorded start times)
        starts = self.__dict__.setdefault("_tl_starts", {})
        for i in range(stage + 1):
            starts.setdefault(i, ut)
        names = ["ENTRY", "ENTRY BURN", "AERO GLIDE", "LANDING BURN", "TOUCHDOWN"]
        self._tl_events = "|".join("%s:%.1f" % (names[i], (starts[i] - ut) if i <= stage else tts[i])
                                   for i in range(1, 5))
        if getattr(self, "_tl_stage", None) != stage:
            self._tl_stage, self._tl_t0 = stage, ut
        frac = 0.0
        if stage < 4:
            el = ut - self._tl_t0
            frac = min(el / max(el + tts[stage + 1], 1e-3), 0.99)
        fmt = lambda t: "T-%d:%02d" % (int(t) // 60, int(t) % 60) if t >= 60 else "T-%ds" % int(t)
        return "|".join(fmt(t) if i > stage else "" for i, t in enumerate(tts)), stage + frac

    def hud_pred(self, ut, r, vel, phase):
        """Predicted touchdown point (pad E, N) for the HUD: the drag-modulation predictor while it runs,
        the glide law's drag-decayed drift below it, the current position once the landing burn is lit."""
        if phase != "COAST":
            return float(r[0]), float(r[1])
        pe = getattr(self, "_pred_en", None)
        if pe is not None and ut - pe[2] < 3.0:
            return pe[0], pe[1]
        sp = float(np.linalg.norm(vel))
        tau = max(sp, 1.0) / self.g
        t = max(r[2], 0.0) / max(-vel[2], 1.0)
        k = tau * (1.0 - math.exp(-t / tau))
        return float(r[0] + vel[0] * k), float(r[1] + vel[1] * k)

    def hud_clock(self, ut):
        """Mission clock for the HUD: T+ since lift-off when this run launched the rocket, else time to touchdown."""
        t0 = getattr(self, "t_launch_ut", None)
        if t0 is not None:
            t = max(ut - t0, 0.0)
            return "T+ %02d:%02d:%02d" % (int(t) // 3600, int(t) // 60 % 60, int(t) % 60)
        t = max(getattr(self, "_tl_t4", 0.0), 0.0)
        return "T− %02d:%02d" % (int(t) // 60, int(t) % 60)

    def science(self, ut, r, q, v, att=float("nan"), roll=float("nan")):
        """Live research panel for the HUD: convergence and model-learning time series + parameters."""
        if phase_burn := (getattr(self, "ignition_offset", None) is not None):
            miss = float(np.linalg.norm(r[:2]))
        else:
            miss = getattr(self, "_pred_miss", float("nan"))
        ko, ki = self.k_drag[True], self.k_drag[False]
        rows = ["t=%.2f" % ut,
                "plot.miss=%s|%.1f|1|100000|log" % ("Distance to pad (m)" if phase_burn else "Predicted miss (m)",
                                                     miss if miss == miss else 1e5),
                "plot.att=Pointing error (deg)|%.2f|0|20|lin" % (att if att == att else 0.0),
                "plot.roll=Roll rate (deg/s)|%.2f|-20|20|lin" % (roll if roll == roll else 0.0),
                "plot.k=Drag model scale k (measured/table)|%.3f|0|2|lin" % ko,
                "plot.q=Dynamic pressure (kPa)|%.2f|0|40|lin" % (q / 1e3),
                "plot.thr=Throttle|%.2f|0|1|lin" % self._throttle()]
        prm = ["aim offset (learned)  %+.0f m" % self.aim_offset,
               "drag k  out %.2f / in %.2f" % (ko, ki),
               "CdA  out %.1f / in %.1f m2" % (self.cda_out or 0, self.cda_in or 0),
               "airbrakes  %s" % ("OUT" if self._brakes() else "IN"),
               "entry burn  %.0f / %.0f m/s" % (self.entry_flown, self.entry_dv),
               "hoverslam drag credit  %.2f" % self.hs.drag_frac,
               "aero normal-force c  %.4f%s" % (self.aero.c, "" if self.aero.confident else " (prior)"),
               "lateral kp/kv  %.2f / %.2f   lag %.1f s" % (self.kp_lat, self.kv_lat, self.lag),
               "gate %.0f m  v_final %.1f m/s  thr floor %.2f" % (self.gate[2], self.v_final, self.thr_floor)]
        return "\n" + "\n".join(rows) + "\nparams=" + ";".join(prm)

    def draw_prediction(self, impact):
        """Predicted track + pad marker for the AutoLab plugin to draw in the scene (file only: no RPCs)."""
        try:
            _, _, _, lat, lon, alt = impact.history_bf()
            k = max(1, len(lat) // 80)
            rows = ["pad %.6f %.6f %.1f" % (self.target[0], self.target[1], pdg.PAD_ALT)]   # (camera target)
            if self.draw_track:
                rows += ["%.6f %.6f %.1f" % (a, o, max(h, 0.0)) for a, o, h in zip(lat[::k], lon[::k], alt[::k])]
            core.trajectory("\n".join(rows))
        except Exception as e:
            if not getattr(self, "_draw_err", False):
                self._draw_err = True
                self.log("draw error:", repr(e))

    def aero_map(self, cmd, tmax, m, cq, air):
        """Re-aim the thrust so guidance gets the lateral acceleration it asked for, given the body
        normal force: a_lat = a_T * tau - c q (tau - air_h)  =>  tau = (a_T d_h - c q air_h) / (a_T - c q)."""
        thr, d = cmd
        a_t = thr * tmax / m
        if a_t < 0.5 or abs(cq) < 1e-3:
            return cmd
        d = np.asarray(d, float)
        den = a_t - cq
        smax = math.sin(math.radians(20))
        if abs(den) < 0.25 * a_t:          # no lateral authority either way: fly straight along the airflow
            tau = air[:2] * 0.0
        else:
            tau = (a_t * d[:2] - cq * air[:2]) / den
        n = float(np.linalg.norm(tau))
        if n > smax:
            tau *= smax / n
        return thr, np.array([tau[0], tau[1], math.sqrt(max(1 - float(np.dot(tau, tau)), 0.0))])

    # --- G-FOLD worker ---
    def _planner(self):
        while not self._stop:
            t_wall = time.time()
            try:
                ut = self.ut()
                r, v, m = self.state()
                with self._lock:
                    ref, put = self.plan, self.plan_ut
                if r[2] > self.gate[2] + 20:
                    cfg = replace(self.base_cfg(), t_max=max(self.thrust(), 1.0) * self.est.eta,
                                  a_dist=tuple(self.a_dist))
                    p = pdg.solve(r - self.gate, v, m, cfg, ref=ref, t_shift=(ut - put) if ref else 0.0)
                    self.solve_times.append(p.solve_time)
                    if p.ok:
                        # the plan starts at the state sampled at `ut`
                        with self._lock:
                            self.plan, self.plan_ut = p, ut
                    # on failure keep tracking the previous plan until it expires
            except Exception as e:  # the planner must never kill the flight
                self.log("planner error:", repr(e))
            time.sleep(max(0.05, 1.0 - (time.time() - t_wall)))

    DEADMAN_S = 2.5        # s without a control-loop heartbeat while the engine burns -> throttle cut

    def _hb_game(self):
        """DR-010 item 2: REC_FULL (rec_v2) times the heartbeat in GAME seconds (the game runs ~0.4x while recording,
        a 2.5 s wall stall was a normal 1 game-s tick there); everything else keeps the wall clock."""
        return bool(getattr(self, "rec_v2", False)) and getattr(self, "_hb_mark", None) is not None

    def _hb_beat(self):
        self._hb = time.time()
        if self._hb_game():
            self._hb_mark = self._gclock().mark()          # stream reads only: no RPC

    def _hb_age(self):
        """Seconds since the last heartbeat: wall s (as before); rec_v2: game s, or wall s / 3 if that is more
        (core.GameClock cap: a paused game / unreadable UT still trips the switch after 3 x 2.5 s wall)."""
        if self._hb_game():
            return self._gclock().elapsed(self._hb_mark)
        return time.time() - self._hb

    def _deadman(self):
        """Dead-man switch with its own connection: if the control loop stops beating for 2.5 s while the
        engine burns (a blocking prediction once held full throttle for 85 s and emptied the tanks), cut it."""
        try:
            conn = core.connect("deadman")
            vv = conn.space_center.active_vessel
        except Exception as e:
            self.log("deadman: no connection (%r)" % e)
            return
        cut = False
        while not self._stop:
            stale = self._hb_age() > self.DEADMAN_S and not self._hb_hold
            try:
                if stale and not cut and vv.control.throttle > 0:
                    vv.control.throttle = 0.0
                    cut = True
                    self.log("DEADMAN: control loop stalled %.1f s%s -> throttle cut" % (
                        self._hb_age(), " (game)" if self._hb_game() else ""))
                elif not stale:
                    cut = False
            except Exception:
                pass
            time.sleep(0.25)
        try:
            conn.close()
        except Exception:
            pass

    # --- main loop ---
    def run(self, timeout=900, coast=True):
        v, log = self.v, self.log
        act = Actuator(self.conn, v, self.frame, max_tilt_deg=20)
        act.ap.time_to_peak = (self.ttp, self.ttp, self.ttp)
        v.control.throttle = 0.0
        if len(v.parts.rcs):
            v.control.rcs = True
        self.dry = v.dry_mass
        if self.freeze_leading:
            # flying engine-first, surfaces below the CoM lead the airflow and act reversed
            rfv = v.reference_frame
            n = 0
            for cs in v.parts.control_surfaces:
                if cs.part.position(rfv)[1] < 0 or self.freeze_leading == "all":
                    cs.pitch_enabled = cs.yaw_enabled = cs.roll_enabled = False
                    n += 1
            log("frozen %d leading control surfaces" % n)
        if self.draw:
            core.camera_shot("chase")
        phase = "COAST" if coast else "BURN"
        legs_out, worker, src = False, None, "-"
        t0 = time.time()
        csvf = open(getattr(self.log, 'path', core.os.path.join(core.LOG_DIR, 'landing.txt')).replace('.txt', '_landing.csv'), 'w')
        csvf.write("ut,phase,src,e,n,h,ve,vn,vu,m,tmax,eta,cda,drag,thr_cmd,dx,dy,dz,thr_act,ap_err,de,dn,du,c,q,parts,brakes,speed,drag_model,lat,lon,roll_rate,py_rate,aoa,cmd_aoa,ne,nn,nu,roll_ang,heat,ctl_p,ctl_y,ctl_r,ident_cmd,ident_band" + (",a1e_aoa_cmd,a1e_az" if self.a1e else "") + ",h_gnd\n")
        self._hb, self._hb_hold = time.time(), False
        if self.rec_v2:                     # DR-010 items 2 + 3 (REC_FULL only): game-time DEADMAN, streamed reads
            self._io_setup(act)
            self._hb_mark = self._gclock().mark()
        self._below_t0 = self._stall_t0 = None     # CA-3 below-pad-level timers (ut)
        self._abl_setup()                          # EXPLORATORY KSP_ABL_* switches: no-op when unset
        threading.Thread(target=self._deadman, name="deadman", daemon=True).start()
        while time.time() - t0 < timeout:
            self._hb_beat()
            if self._tel is not None:
                self._tick += 1
            ut = self.ut()
            try:
                r, vel, m = self.state()
            except Exception:
                m = 0.0
            if m <= 0:
                log("vessel lost (mass 0)")
                self.events.append(("lost",))
                break
            h, vz = r[2], vel[2]
            hg = self.h_gnd          # feet above the real ground (radar off the pad; nan = no radar)
            id_log = (float("nan"), float("nan"))     # CSV ident_cmd / ident_band (nan = not in the IDENT window)
            a1e_log, thr_a1e = (float("nan"), float("nan")), None     # CSV a1e_aoa_cmd / a1e_az (A1e window only)
            self._dvec = np.asarray(self.drag(), float)
            dmag = float(np.linalg.norm(self._dvec))
            self.update_cda(r, vel, dmag)
            self.update_drag_scale(r, dmag)
            eta = self.sense(ut, vel, m, dmag)
            tmax = max(self.thrust(), 1.0) * eta
            if self._pad_cleared < 2 and vz < 0 and h < (1500 if self._pad_cleared == 0 else 600):
                from .vehicle import clear_pad   # clamp debris only loads within ~2.25 km
                clear_pad(self.sc, v, log)
                self._pad_cleared += 1
            if ut - getattr(self, "_parts_t", -1e9) > 5.0:           # parts lost (burned fins went unnoticed once)
                self._parts_t = ut
                try:
                    if self._tel is None:
                        npart = len(v.parts.all)
                    else:                           # rec_v2: the list is kept for heat_ratio (no 1 Hz re-read)
                        self._parts_list = v.parts.all
                        npart = len(self._parts_list)
                    if npart < getattr(self, "_parts_n", npart):
                        self._parts_lost = True
                        log("PARTS LOST: %d -> %d at h=%.0fm v=%.0fm/s q=%.1fkPa" % (self._parts_n, npart, h,
                            float(np.linalg.norm(vel)), 0.5 * atmo.density(h + self.alt0) * float(np.dot(vel, vel)) / 1e3))
                    self._parts_n = npart
                except Exception:
                    pass
            if ut - getattr(self, "_heat_t", -1e9) > 1.0:            # heating (streams, rebuilt when parts change)
                self._heat_t = ut
                try:
                    self.heat = self.heat_ratio(v)
                except Exception:
                    pass
            if (os.environ.get("KSP_SAVE_TERM") and not getattr(self, "_mid_saved", False) and 14000.0 < h < 15000.0
                    and vz < 0 and not self.entry_burn and self._throttle() < 0.01):
                self._mid_saved = True             # envelope stage 3 checkpoint (F9R-Dev analogue, verification process)
                self.sc.save("g5_mid" + self.save_suffix)
                hud.report("checkpoint", "g5_mid" + self.save_suffix)   # the T+ clock continues from the lift-off UT
                log("checkpoint saved: g5_mid%s at %.0f m" % (self.save_suffix, h))
            if (os.environ.get("KSP_SAVE_TERM") and not getattr(self, "_term_saved", False) and 2500.0 < h < 4000.0
                    and vz < 0 and phase == "COAST" and not self.entry_burn and self._throttle() < 0.01):
                # engine off, before the landing burn: it once saved mid-burn (entry / landing burn); in BURN the
                # throttle can read 0 while the attitude interlock holds fire, hence the phase test as well
                self._term_saved = True
                self.sc.save("g5_term" + self.save_suffix)   # fast landing-test checkpoint (--from-term)
                hud.report("checkpoint", "g5_term" + self.save_suffix)
                log("checkpoint saved: g5_term%s at %.0f m" % (self.save_suffix, h))
            if (self.has_fins and self.fins_passive_below > 0 and h < self.fins_passive_below
                    and not getattr(self, "_fins_passive", False)):
                # fins + autopilot oscillate at ~0.45 Hz in the low, slow coast: let the fins act as passive
                # tail feathers (drag + weathervane stability) and stop active steering
                for cs in v.parts.control_surfaces:
                    cs.pitch_enabled = cs.yaw_enabled = False
                self._fins_passive = True
                log("grid fins passive below %.0f m" % h)
            if self.has_fins and ut - getattr(self, "_fin_chk", -1e9) > 2.0:
                self._fin_chk = ut
                # no airbrakes (T-WorksFineGF0): above the glide the fins themselves are the drag modulator
                # (probe: CdA 25 m^2 out / 11 m^2 in), following the Brakes state the modulation sets
                # Airbrake craft (T-WorksFineGF): the fins NEVER fold, they are re-extended whenever the brakes go
                # in, as on the gf-baseline reference (brakes-in CdA 8.0 m^2, logs/wd_20261020_124120.out). Folding
                # them with the airbrakes (a1e2dae) dropped brakes-in to 5.1 m^2: 500-800 m short, landing burn
                # diverted 269 m and ran dry (mishap logs/wd_20261020_231839.out, CA-1).
                fin_dm = (not self.has_airbrakes and phase == "COAST" and h > self.glide_below
                          and not self.entry_burn and not self._brakes())
                self.extend_grid_fins(want=not fin_dm)
            if self.fin_steer and not self.has_fins and not getattr(self, "_fins_on", False) and h < self.fin_alt:
                # grid-fin style: the airbrakes sit at the TRAILING end when falling engine-first ->
                # let them steer (pitch/yaw) as well as brake, for attitude authority at high q
                rfv, n_f = v.reference_frame, 0
                for cs in v.parts.control_surfaces:
                    if cs.part.position(rfv)[1] > 0:
                        cs.pitch_enabled = cs.yaw_enabled = True
                        cs.roll_enabled = False
                        n_f += 1
                self._fins_on = True
                log("fin steering: %d trailing surfaces enabled at %.0f m" % (n_f, h))
            if not legs_out and h < self.legs_alt:   # legs are draggy at the leading end: deploy late
                v.control.gear = True
                legs_out = True
            nose = self.frame.vec(self.nose())
            sp = float(np.linalg.norm(vel))
            vt = self.v_true_enu
            air = -vt / np.linalg.norm(vt) if sp > 5 else np.array([0.0, 0.0, 1.0])   # engine-first axis (true)
            if dmag > 0.05 * m:
                air_d = unit(self.frame.vec(self._dvec))
                if ut - getattr(self, "_airlog_t", -1e9) > 10.0 and sp > 5:
                    self._airlog_t = ut
                    log("airflow check: drag vector %.1f deg off the velocity axis (q %.1f kPa)" % (
                        math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(air_d, air)))))), 0.5 * atmo.density(
                            self.alt0 + max(h, 0.0)) * sp * sp / 1e3))
                # the drag vector carries the BODY LIFT of a lifting craft: on the fins-only core it sat ~4 deg off
                # the airflow and cancelled the commanded angle of attack (C-mode run 3: 4.4 deg wanted, 0.2-1
                # flown, lift reversed above 24 km; also corrupted the normal-force estimate). No wind in stock:
                # the velocity IS the airflow. Drag axis kept only for the other craft (KSP_AIRDRAG).
                if os.environ.get("KSP_AIRDRAG", "0" if (self.has_fins and not self.has_airbrakes) else "1") == "1":
                    air = air_d
            # near apogee the velocity is mostly horizontal: never let "retrograde" lay the vehicle down
            tilt = math.acos(max(-1.0, min(1.0, air[2])))
            if tilt > math.radians(15) and sp < 150:
                hz = np.array([air[0], air[1], 0.0])
                hz = hz / max(np.linalg.norm(hz), 1e-9)
                air = math.cos(math.radians(15)) * np.array([0.0, 0.0, 1.0]) + math.sin(math.radians(15)) * hz
            q = 0.5 * atmo.density(self.alt0 + max(h, 0.0)) * sp * sp
            beta = nose - np.dot(nose, air) * air
            if self._sf is not None and ut - self._sf[0] > 0.04:
                a_sp = (vel - self._sf[1]) / (ut - self._sf[0]) + np.array([0, 0, self.g])
                a_sp -= tmax * self._throttle() / m * nose
                a_n = a_sp - np.dot(a_sp, air) * air
                k_f = min((ut - self._sf[0]) / 1.0, 1.0)          # 1 s low-pass of the measured side force
                self._a_lat = self._a_lat + k_f * (a_n[:2] - self._a_lat)
                if self.ap_err(act) < 10:
                    self.aero.update(a_n, q, beta)
                    if (self.nf_learn and self._throttle() < 0.01 and self.aero.confident
                            and abs(self.aero.c) > 1.5e-4):                # hysteresis: ignore a near-zero c
                        s_ = -1.0 if self.aero.c > 0 else 1.0
                        if s_ != self.nf_sign:
                            self.nf_sign = s_
                            log("glide sign learned: side force %s the tilt (c %+.1e at h=%.0fm q=%.1fkPa)" % (
                                "WITH" if s_ > 0 else "AGAINST", self.aero.c, h, q / 1e3))
            self._sf = (ut, vel.copy())
            # use the learned normal force only once it is trusted: an unconfident estimate (polluted by fin
            # deflection forces) flipped the burn's steering away from the pad (115 m miss, 2026-10-19)
            cq = self.aero.c * q if self.aero.confident else 0.0
            cmd = (0.0, np.array([0.0, 0.0, 1.0]))
            # 2x physics warp only for the high, unpowered part of the fall; real time below 25 km
            want = 1 if (phase == "COAST" and h > 25000) else 0
            if self.ident and self.entry_done:      # IDENT: real time (clean attitude / control-input data)
                want = 0
            if self.rec_v2 and os.environ.get("KSP_REC_B2") == "1":
                want = 0        # spec C1 fallback B2 (opt-in, HOP_REC=full only): entry at 1x, needs a regression
            if ((self.sc.physics_warp_factor != want and self.sc.rails_warp_factor == 0) if self._tel is None else
                    (self._tel["pwarp"]() != want and self._tel["rwarp"]() == 0)):
                try:
                    self.sc.physics_warp_factor = want
                except Exception:
                    pass
            if phase == "COAST":
                if vz > 0:      # still climbing: hold attitude up, no flip
                    cmd = (0.0, np.array([0.0, 0.0, 1.0]))
                else:           # falling: engine first, airbrakes out, aerodynamic glide to the target
                    vtrue = float(np.linalg.norm(self.vel()))
                    q_true = 0.5 * atmo.density(h + self.h_com + self.alt0) * vtrue * vtrue
                    if q_true > 10000:                 # structural safety beats range control
                        if self.dm_cont:
                            self.set_dm_fraction(1.0)
                        else:
                            self.set_brakes(True)
                    elif (self.cda_in is not None and not self.dm_off and not self.ident_window(h)
                          and not self.a1e_window(h)
                          and h > (self.glide_below if self.has_fins else self.dm_end) and not self.entry_burn):
                        self.drag_modulation(ut)     # paused while the entry burn is in progress
                    elif self.dm_cont:
                        self.set_dm_fraction(1.0)
                    else:
                        self.set_brakes(True)
                    # entry burn (booster-recovery style): cap the dynamic pressure before the dense air
                    if self.entry_burn:   # fly exactly the dv the predictions assumed
                        # DR-010: dv from the propellant burned (Isp g0 ln(m0/m)); the old thrust-sample x tick sum
                        # lost every tick whose sample read 0 after a DEADMAN cut (flight #4: ~120 m/s, 3.8 km miss)
                        # and is kept as a logged diagnostic only
                        dt_eb = max(ut - getattr(self, "_eb_t", ut), 0.0)
                        thr_eb = v.thrust if self._tel is None else self._tel["thrust"]()
                        self._eb_thr_sum += thr_eb / max(m, 1.0) * dt_eb
                        self.entry_flown += self._dv_mass(self._eb_m, m, dt_eb)
                        self._eb_m = m
                    self._eb_t = ut
                    alt_now = h + self.h_com + self.alt0
                    planned = (not self.entry_done and (alt_now < self.entry_alt or self.entry_burn)
                               and self.entry_flown < self.entry_dv)
                    if (self.entry_done and not self.entry_burn and (self.heat > self.heat_safety or q_true > self.q_safety)
                            and ut - getattr(self, "_safe_t", -1e9) > 5.0):
                        # a hot part or absurd q: ONE bounded chunk (150 m/s, 8 s max), then re-check after 5 s
                        self._safe_vend, self._safe_t = vtrue - 150.0, ut
                    safety = (self.entry_done and h > 8000 and vtrue > 600 and vtrue > getattr(self, "_safe_vend", 1e9)
                              and ut - getattr(self, "_safe_t", -1e9) < 8.0)
                    if (planned or safety) and vtrue > 600:
                        if not self.entry_burn:
                            self.entry_burn, self.entry_flown = True, 0.0
                            self._eb_start(m)
                            hud.report("event", "ENTRY BURN", ut)
                            log("ENTRY BURN start h=%.0fm v=%.0fm/s q=%.1fkPa heat %.2f (m %.0f kg, Isp %.1f s)" % (
                                h, vtrue, q_true / 1e3, self.heat, m, self._eb_isp))
                    elif self.entry_burn:
                        self.entry_burn = False
                        self.entry_done = True
                        hud.report("event", "AERO GLIDE", ut)
                        log("ENTRY BURN end h=%.0fm v=%.0fm/s q=%.1fkPa dv flown %.0f m/s (propellant %.0f kg, Isp %.1f s; "
                            "thrust-sample sum %.0f m/s)" % (h, vtrue, q_true / 1e3, self.entry_flown, self._eb_m0 - m,
                                                           self._eb_isp, self._eb_thr_sum))
                        if self.post_trim:
                            self._ctlw().throttle = 0.0
                            self.post_entry_trim(act)
                        if self.vernier and not getattr(self, "_vern_planned", False):
                            self._vern_planned = True      # after the planned entry burn only: a vernier after a
                            self._vern = self.plan_vernier(ut)   # low safety burn fired 121 m/s sideways at 7 km
                    d = air
                    # glide uses the flight-measured side-force gain (grid-fin sysid: 5 deg -> ~0.5 m/s^2 at
                    # q 5 kPa, c*m ~ 15), not the online estimate (fin deflection forces corrupt it)
                    cq_glide = self.glide_c_per_kg / max(m, 1000.0) * q if self.glide_c_per_kg > 0 else cq
                    fins_only_glide = self.has_fins and not self.has_airbrakes and "KSP_GLIDE_C" not in os.environ
                    if fins_only_glide:   # clean fit c ~ 1.3e-4 (c*m ~ 2): the default 15 asked 8x too little tilt
                        cq_glide = self.c_hat() * q
                    d = self.glide_dir(r, vel, h, vz, cq_glide, sp, air, getattr(self, "_h_ign_est", None),
                                       self._a_lat, ut)
                    if (self.has_fins and self.xglide and self.entry_done and not self.entry_burn and self._brakes()
                            and self.glide_below <= h < self.xglide_top and not self.ident_window(h)
                            and not self.a1e_window(h)):
                        d = self.cross_glide(ut, air, q, m)
                    if (self.belly and self.entry_done and not self.entry_burn and h > self.glide_below
                            and self.cda_in is not None and not self.ident_window(h) and not self.a1e_window(h)):
                        b_ = math.radians(self.belly_deg * self._belly_f)
                        if self.belly_dir == "up":
                            # lift in the vertical plane, upward: the top tilts the other way (nf_sign < 0)
                            up_ = np.array([0.0, 0.0, 1.0])
                            try:
                                up_ = unit(np.asarray(self.frame.vec(self.c_basis()[0]), float))
                            except Exception:
                                pass
                            w_ = unit(up_ - np.dot(up_, air) * air)
                            d = unit(air + self.nf_sign * math.tan(b_) * w_)
                        else:
                            vh_ = np.array([vel[0], vel[1], 0.0])
                            side_ = unit(np.array([-vh_[1], vh_[0], 0.0])) if np.linalg.norm(vh_) > 1.0 else np.array([1.0, 0.0, 0.0])
                            d = unit(air + self._belly_sign * math.tan(b_) * side_)
                    if self.ident_window(h):
                        if self._id_open is None:
                            self._id_open = h
                            log("IDENT window open h=%.0fm q=%.1fkPa" % (h, q_true / 1e3))
                        d, id_c, id_b = self.ident_command(ut, h, q_true, vz)
                        id_log = (id_c, id_b)
                    elif self.ident and self._id_open is not None and not getattr(self, "_id_closed", False):
                        self._id_closed = True
                        log("IDENT window closed h=%.0fm q=%.1fkPa%s: normal glide / landing" % (
                            h, q_true / 1e3, " (aborted: %s)" % self._id_abort if self._id_abort else ""))
                    if self.a1e_window(h) and self.drag_in is not None and self.drag_out is not None:
                        d, thr_a1e = self.a1e_command(ut, h, q_true, m, air, nose)
                        a1e_log = (self._a1e_cmd, math.degrees(self._a1e_phi))
                    elif self.a1e and self._a1e_open is not None and not self._a1e_closed:
                        self.a1e_close(h, "floor")
                    elif self.a1e and self.a1e_window(h) and not getattr(self, "_a1e_notab", False):
                        self._a1e_notab = True
                        log("A1E: no drag tables (drag_in / drag_out) -> A1e inactive, normal glide")
                    if self.entry_burn and self.entry_guide and not self.entry_done:   # planned burn only: on a
                        # safety burn it predicted "800 m/s left" in dense air and blocked the loop 85 s, full throttle
                        self.entry_guidance(ut)
                        d = unit(air + getattr(self, "_eb_offset", np.zeros(3)))
                    vern, cmd_vern = getattr(self, "_vern", None), None
                    if vern is not None:
                        t = math.radians(self.vernier_tilt_deg)
                        d = unit(air * math.cos(t) + vern["side"] * math.sin(t))
                        on = float(np.dot(nose, d)) > math.cos(math.radians(3.0))   # fire only once pointed
                        if on and vern["t_last"] is not None:      # DR-010: from the propellant burned
                            vern["got"] += self._dv_mass(vern.get("m_last"), m, ut - vern["t_last"]) * math.sin(t)
                        vern["t_last"], vern["m_last"] = ut, m
                        if vern["got"] >= vern["dv"] or ut - vern["t0"] > 8.0:
                            log("vernier done: %.2f / %.2f m/s sideways in %.1f s" % (vern["got"], vern["dv"], ut - vern["t0"]))
                            self._vern = None
                        else:
                            cmd_vern = (self.vernier_throttle if on else 0.0, d)
                    cmd = (1.0 if self.entry_burn else (0.3 if ut < self.retro_until else 0.0), d)
                    if thr_a1e is not None:
                        cmd = (thr_a1e, d)
                    if cmd_vern is not None:
                        cmd = cmd_vern
                far = float(np.linalg.norm(r[:2])) > 1000.0   # unreachable target: just land safely where we are
                go, margin = (pdg.hoverslam_burn_now(h, vz, m, tmax, self.g, self.cda, None, self.hs,
                                                     None if far else r[:2], None if far else vel[:2])
                              if h < 8000 and vz < 0 else (False, h))
                if h < 8000 and vz < 0:
                    self._h_ign_est = max(h - margin, 0.0)     # height at which the burn will light
                if go:
                    phase = "BURN"
                    hud.report("event", "LANDING BURN", ut)
                    if self.retract_brakes_on_burn:
                        self.set_brakes(False)     # more lateral authority (but less aero stability)
                    self.aero = self.aero_burn          # (same learned model as in the coast)
                    if self.a1e:
                        self.a1e_close(h, "landing burn")
                    if self.fins_off_in_burn:
                        # fins + airbrakes + gimbal together over-control at burn speeds (attitude error to 40 deg,
                        # 20 m/s sideways into the ground): the burn is flown on the gimbal alone
                        for cs in v.parts.control_surfaces:
                            cs.pitch_enabled = cs.yaw_enabled = cs.roll_enabled = False
                        log("landing burn: control surfaces released, gimbal only")
                    for e in v.parts.engines:      # big gimbals + autopilot at full thrust oscillate
                        if e.active and e.gimballed:
                            e.gimbal_limit = self.gimbal_limit
                    self.events.append(("ignition", round(h), round(vz, 1), round(float(np.linalg.norm(r[:2])))))
                    self.ignition_offset = (float(r[0]), float(r[1]))
                    log("IGNITION h=%.0fm vz=%.1f vh=%.1f offset=%.0fm cda=%.2f" % (
                        h, vz, np.linalg.norm(vel[:2]), np.linalg.norm(r[:2]), self.cda))
                    if self.use_gfold and self.law == "gfold":
                        worker = threading.Thread(target=self._planner, daemon=True)
                        worker.start()
            if phase == "BURN":
                with self._lock:
                    plan, put = self.plan, self.plan_ut
                tp = ut - put
                if self.law == "zem":
                    cmd = self.burn_law(r, vel, m, tmax, dmag)
                    src = "ZEM" if h > self.gate[2] + 1 else "TERMINAL"
                elif plan is not None and tp < plan.tf and h > self.h_gfold_min:
                    cfg = replace(self.base_cfg(), t_max=tmax)
                    cmd = pdg.track(plan, tp, r - self.gate, vel, m, cfg)
                    src = "G-FOLD"
                else:
                    p_t = self.terminal if h < self.gate[2] + 30 else self.hs
                    cmd = pdg.hoverslam_command(r, vel, m, tmax, self.g, dmag, p_t)
                    src = "HOVERSLAM" if h > self.gate[2] + 1 else "TERMINAL"
                # ground tests use hg (real ground), not h: h is height above PAD level, and beside the elevated
                # pad it goes negative with the legs still in the air (CA-3)
                if str(self.sit()).endswith(("landed", "splashed")) or (hg < 0.3 and abs(vz) < 0.5):
                    break
                if legs_out and h < 3.0 and self._legs_grounded():
                    break      # first foot on the ground: cut thrust now (thrust on legs = tipping torque)
                if h < -1.0:
                    # CA-3: lower ground beside the elevated pad. The old rule (h < -1 and |vz| < 3: cut) fired
                    # with the legs still ~ metres up -> free fall, toppled (T-WorksFineGF, 2026-10-20, x2). Keep
                    # descending at the touchdown speed (burn_law) until a leg touches (break above); guards so it
                    # can never hover the tank dry (2026-10-19): settled on the radar ground, a hover/stall, or a cap.
                    if self._below_t0 is None:
                        self._below_t0 = ut
                        log("below pad level (lower ground off the pad): radar ground %.1f m, descending to touchdown" % hg)
                    slow = abs(vz) < 0.5
                    self._stall_t0 = (ut if self._stall_t0 is None else self._stall_t0) if slow else None
                    if hg < 0.5 and slow:
                        log("on the radar ground (%.2f m, vz %.2f) without leg contact: cut thrust" % (hg, vz))
                        break
                    if self._stall_t0 is not None and ut - self._stall_t0 > self.below_stall_s:
                        log("hovering below pad level for %.0f s (radar ground %.1f m): cut thrust" % (self.below_stall_s, hg))
                        break
                    if ut - self._below_t0 > self.below_max_s:
                        log("below pad level for %.0f s without touchdown (radar ground %.1f m): cut thrust" % (self.below_max_s, hg))
                        break
                cmd = self.aero_map(cmd, tmax, m, cq, air)
                if self.kick_thr > 0 and h > 250.0 and float(np.linalg.norm(r[:2])) > 25.0 and self.ap_err(act) > 8.0:
                    # the stage is not turning (high q, weak gimbal torque at low throttle): more thrust =
                    # more gimbal torque, so it gets over early and builds the sideways speed up high
                    cmd = (max(cmd[0], self.kick_thr), cmd[1])
            if self.ident and self.ident_ttp:
                # IDENT window only (COAST: a landing burn lit inside the window gets the normal controller back)
                ttp_ = self.ident_ttp if (phase == "COAST" and self.ident_window(h)) else self.ttp
                if ttp_ != getattr(self, "_ttp_now", self.ttp):
                    act.ap.time_to_peak = (ttp_, ttp_, ttp_)
                    self._ttp_now = ttp_
                    log("IDENT: autopilot time_to_peak %.1f s at h=%.0fm" % (ttp_, h))
            if self.a1e:
                # A1e window only (DR-004: ttp 1.5 s holds 6 deg at 116 kPa, 5 s gave 4.3); restored afterwards
                ttp_ = self.a1e_ttp if (phase == "COAST" and self.a1e_window(h)) else self.ttp
                if ttp_ != getattr(self, "_ttp_now", self.ttp):
                    act.ap.time_to_peak = (ttp_, ttp_, ttp_)
                    self._ttp_now = ttp_
                    log("A1E: autopilot time_to_peak %.1f s at h=%.0fm" % (ttp_, h))
            if self.own_att and phase == "COAST" and h < self.own_att_top:
                if self._own is None:
                    from .flightctl import OwnAttitude
                    act.ap.engaged = False
                    self._own = OwnAttitude(self.conn, v, self.frame, log, kp=self.own_kp, kd=self.own_kd)
                    log("own attitude controller engaged at %.0f m" % h)
                self._own.step(cmd[1], ut)
                self._last_cmd_dir = cmd[1]
                self._ctlw().throttle = float(max(0.0, min(1.0, cmd[0])))
                if self._tel is not None:
                    self._thr_w = (self._tick, float(max(0.0, min(1.0, cmd[0]))))
            else:
                if self._own is not None:                  # hand back to the autopilot (landing burn)
                    self._own.stop()
                    self._own = None
                    act.ap.reference_frame = self.frame.rf
                    act.ap.engaged = True
                set_thrust(act, cmd[0], cmd[1])
                if self._tel is not None:
                    self._thr_w = (self._tick, act.thr_cmd)
            # estimator input: thrust actually commanded (0 while the attitude interlock holds fire)
            thr_out = self._throttle()
            self.prev = (ut, np.array(vel), tmax * thr_out * np.asarray(cmd[1]) / m) if thr_out > 0.05 else None
            d = np.asarray(cmd[1])
            try:   # attitude smoothness: roll rate about the long axis, pitch/yaw rate (deg/s)
                w_b = np.asarray(self.angvel())
                nb = np.asarray(self.nose())
                roll_w = float(np.dot(w_b, nb))
                roll_rate, py_rate = math.degrees(roll_w), math.degrees(float(np.linalg.norm(w_b - roll_w * nb)))
                self._rr_last = roll_rate
                if self.roll_damp > 0:
                    # PI on roll rate: a P-only damper left a steady 8 deg/s spin (constant fin/brake roll
                    # torque); the spin rotated the autopilot's pitch/yaw axes -> side-to-side coning wobble
                    dt_r = min(0.5, ut - getattr(self, "_roll_ut", ut))
                    self._roll_ut = ut
                    self._roll_i = max(-0.9, min(0.9, getattr(self, "_roll_i", 0.0) - self.roll_ki * roll_w * dt_r))
                    self._ctlw().roll = float(max(-1.0, min(1.0, self.roll_sign * (-self.roll_damp * roll_w + self._roll_i))))
            except Exception:
                roll_rate = py_rate = float("nan")
            try:   # nose and command vs the oncoming air (retrograde surface velocity), deg
                # TRUE velocity (pure rotation): `vel` mixes pad-ENU horizontal with the radial vertical speed and
                # is ~4.5 deg off at 47 km from the pad -> the logged aoa/cmd were wrong far out
                rv = -unit(np.asarray(self.v_true_enu, float))
                n_enu = unit(np.asarray(self.frame.vec(self.nose()), float))
                aoa_n = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(n_enu, rv))))))
                aoa_c = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(unit(d), rv))))))
                self._aoa_meas = aoa_n
                n_log = tuple(float(x) for x in n_enu)
            except Exception:
                aoa_n = aoa_c = float("nan")
                n_log = (float("nan"),) * 3
            try:
                roll_ang = float(v.flight(v.surface_reference_frame).roll if self._tel is None else self._tel["rollang"]())
            except Exception:
                roll_ang = float("nan")
            ctl_ = self.ctl_inputs(v)
            if a1e_log[0] == a1e_log[0]:          # A1e window: control-input saturation (trade abort: > 0.05)
                self._a1e_n += 1
                self._a1e_nsat += int(max(abs(ctl_[0]), abs(ctl_[1])) >= 0.99)
            csvf.write("%.2f,%s,%s,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.0f,%.0f,%.3f,%.2f,%.0f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f" % (
                ut, phase, src, r[0], r[1], h, vel[0], vel[1], vel[2], m, tmax, eta, self.cda, dmag, cmd[0],
                d[0], d[1], d[2], self._throttle(), self.ap_err(act)) + ",%.2f,%.2f,%.2f,%.6f,%.0f" % (tuple(self.a_dist) + (self.aero.c, q))
                       + ",%d,%d,%.1f,%.0f,%.5f,%.5f" % ((len(v.parts.all) if self._tel is None else getattr(self, "_parts_n", -1)) if int(ut) % 5 == 0 else -1,
                                                       int(self._brakes()), sp,
                                                       self.drag_out(h + self.alt0, sp) if self.drag_out else -1,
                                                       *self.latlon())
                       + ",%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.1f,%.3f,%.3f,%.3f,%.3f,%.1f,%g" % (roll_rate, py_rate, aoa_n, aoa_c, *n_log,
                                                                  roll_ang, self.heat, *ctl_, *id_log)
                       + (",%.2f,%.1f" % a1e_log if self.a1e else "") + ",%.2f\n" % hg)
            if int((time.time() - t0) * 4) != getattr(self, "_ov", -1):
                self._ov = int((time.time() - t0) * 4)
                vt = float(np.linalg.norm(self.v_true_enu))
                if phase != "COAST":
                    stage = 4 if h < 15 else 3
                else:
                    stage = 1 if self.entry_burn else (2 if self.entry_done else 0)
                etas, prog = self.timeline(ut, stage, h, vz, m, tmax, margin if phase == "COAST" and h < 8000 else None)
                if hud.current is not None:      # HUD v7: the whole-mission feed composes the overlay
                    pe_, pn_ = self.hud_pred(ut, r, self.v_true_enu, phase)
                    hud.current.set(pos="%.2f,%.2f,%.1f" % (r[0], r[1], max(h, 0.0)), pred="%.2f,%.2f" % (pe_, pn_),
                                    dist="%.1f" % math.hypot(r[0], r[1]), thr="%.3f" % self._throttle(), t="%.2f" % ut,
                                    pip="1" if phase == "BURN" else "0")
                else:
                    core.overlay("HUD\nspeed=%.0f\nalt=%.1f\nphases=ENTRY|ENTRY BURN|AERO GLIDE|LANDING BURN|TOUCHDOWN\nphase=%d\neta=%s\nprogress=%.3f\ninfo=%s" % (
                        vt * 3.6, max(h, 0.0) / 1000.0, stage, etas, prog,
                        "target offset E %+.0f m  N %+.0f m   throttle %3.0f%%" % (r[0], r[1], self._throttle() * 100))
                        + "\nmission=ORBITAL RETURN\ntland=%.1f\nvs=%.1f\ndist=%.0f\nthr=%.3f\nevents=%s\nclock=%s\npos=%.1f,%.1f,%.1f\npred=%.1f,%.1f" % (
                            self._tl_t4, vz, math.hypot(r[0], r[1]), self._throttle(), self._tl_events, self.hud_clock(ut),
                            r[0], r[1], max(h, 0.0), *self.hud_pred(ut, r, self.v_true_enu, phase))
                        + self.science(ut, r, q, v, att=self.ap_err(act), roll=roll_rate))
                if getattr(self, "_handoff_ut", None) is None:
                    # show timeline: announce the 1 km funnel -> convergence hand-off ~1.5 s ahead (then frozen)
                    eta = showrun.handoff_eta(h, vz)
                    if eta is not None and eta <= showrun.HANDOFF_LEAD:
                        self._handoff_ut = ut + eta
                        hud.report("mark", "HANDOFF", self._handoff_ut)
                if not self.entry_burn and ((self.entry_done and h < 40000.0) or h < 15000.0):
                    # entry burn over / none flown (checkpoint runs, no recording): the funnel comes all the same
                    # (with a recording direct_camera marks the S7 -> S8 cut itself, in this same tick)
                    hud.report("mark", "S8", ut)
                if self.draw:
                    self.direct_camera(stage, self._tl_t4, r, ut)
            time.sleep(0.04)
        self._stop = True
        csvf.close()
        self._abl_true_nav()                       # KSP_ABL_NAVNOISE: the result is measured on the true state
        if self.draw:
            core.trajectory("")
            core.camera_aim(0.0)
        try:
            r, vel, m = self.state()
        except Exception:
            m = 0.0
        if m <= 0:
            if hud.current is None:
                core.overlay("")
            return dict(situation="destroyed", miss=float('nan'), v_touchdown=float('nan'), upright=0.0,
                        fuel_left=0.0, parts=0, solves=len(self.solve_times), solve_ms=0.0)
        v_td = float(np.linalg.norm(vel))
        miss_td = math.hypot(r[0], r[1])            # at touchdown (res["miss"] is after the 13 s settle)
        hud.report("event", "TOUCHDOWN")
        v.control.throttle = 0.0
        # hold the stage vertical with the reaction wheels while the suspension settles, then let go
        act.ap.target_direction = tuple(self.frame.to_body(np.array([0.0, 0.0, 1.0])))
        time.sleep(3.0)
        act.release()
        v.control.sas = False       # SAS on a landed stage fights the leg suspension and can topple it
        v.control.rcs = False
        time.sleep(10)              # judge only after it has settled
        r, _, _ = self.state()
        upright = float(np.dot(np.asarray(v.direction(self.frame.rf)), self.frame.u))
        res = dict(situation=str(self.sit()), miss=math.hypot(r[0], r[1]), miss_td=miss_td, v_touchdown=v_td, upright=upright,
                   fuel_left=v.resources.amount('LiquidFuel'), parts=len(v.parts.all),
                   solves=len(self.solve_times),
                   solve_ms=1000 * float(np.median(self.solve_times)) if self.solve_times else 0.0)
        if self.offsite:                     # offset aim: miss is to the target; also report the pad distance
            try:
                pr = self.frame.pos(np.asarray(self.body.surface_position(*core.KSC_PAD, self.frame.rf)))
                res["pad_dist"] = math.hypot(r[0] - pr[0], r[1] - pr[1])
            except Exception:
                res["pad_dist"] = float("nan")
        if hud.current is None:              # the whole-mission HUD holds its last frame (no collapse after landing)
            core.overlay("")
        return res
