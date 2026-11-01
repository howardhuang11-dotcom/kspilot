"""Docking rung R1 (docking R1 plan, docking R0 results section 8): measure, prep, rendezvous to 1 km.

  rdv_r1.py measure    [--target NAME] [--secs 60] [--hz 10]
      READ ONLY (no write of any kind to the game): both vessels, their docking ports (state, position / direction in
      both vessel frames), RCS (channels, thrust, thrusters, thrust centre vs CoM), MonoPropellant / EC, the relative
      state in the target's LVLH frame, the next closest approach, and `secs` s of kRPC sampling: port-to-port
      position / velocity noise (cubic detrend per 30 s segment), RPC round-trip latency, the target port axis drift
      (inertial and LVLH; secs 600 = R0 step 2). -> logs/rdv_r1_measure_<ts>.json. Gates: R0 A2/A3/A4/A7 + kit.
  rdv_r1.py prep       --vessel NAME [--rename NEW] [--recover-landed] [--jettison] [--sas] [--save NAME]
      Small actions to build the R1 saves: make NAME the active vessel, rename it, recover the other vessels that are
      landed / on the pad (never an orbiting one), jettison its fairing(s), SAS stability assist on, save.
  rdv_r1.py rendezvous [--target NAME] [--aim 1000] [--rcs-trim] [--drift 600] [--save NAME]
      From a parking orbit below the target (coplanar within 0.5 deg, both near-circular): plane match at the relative
      node if needed (> 0.05 deg), Hohmann phasing plan (kspilot.rdv.orbit.plan_hohmann_rendezvous) -> burn 1,
      Lambert mid-course correction, burn 2 matching the velocity of the V-bar point `aim` m behind the target, main
      engine nulling of the relative velocity (all burns by kspilot.burn.execute_burn, re-planned from the current
      state). --rcs-trim: RCS translation trim to < 0.1 m/s after a polarity pulse (R0 A1 first touch: off on the
      first flight). --drift: passive free drift record vs the CW prediction (R0 step 6). Long coasts are power
      budgeted (kspilot.power; hibernation if needed, else REFUSED).
      Pass (R0 section 8 step 5): hold point within 300 m of the aim; relative velocity (LVLH) < 0.1 m/s with
      --rcs-trim, < 1.0 m/s main engine only (DOCKING_CONCEPT R1); main-engine dv <= 1.3 x plan.
  --dry-run: the same logic against tests/fake_krpc.OrbitWorld (two-body, impulsive burns), offline.
      Scenarios: measure -> 1 km co-orbital hold; rendezvous / prep -> 80 km parking, target 100 km.

Watchdog: every run ends with one `RESULT PASS|FAIL|REFUSED ...` line; run with --tries 1 (rendezvous burns are not
idempotent; measure is cheap to rerun by hand).
"""
import argparse
import json
import math
import os
import sys
import time
import traceback

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from kspilot import core  # noqa: E402
from kspilot.entry import kepler_propagate  # noqa: E402
from kspilot.rdv import cw  # noqa: E402
from kspilot.rdv import orbit as ob  # noqa: E402

# ---------------------------------------------------------------- gates (R0 RESULTS section 3 / 8; CONCEPT section 4)
G_NOISE_POS = 0.02        # m      A4 port relative position noise 1-sigma
G_NOISE_VEL = 0.005       # m/s    A4
G_LAT_P95 = 0.5           # s      A3 (RPC round trip only; command latency is not observable read-only)
G_TC_AXIAL = 1.5          # m      A2 RCS thrust centre - CoM along the axis (R0 MC range)
G_FORCE_TOL = 0.15        #        RCS force vs design (4 kN axial, 2 kN lateral per direction)
G_MONO_MIN = 45.0         # u      50 u loaded
G_HOLD_POS = 300.0        # m      hold point error
G_HOLD_V_RCS = 0.1        # m/s    with --rcs-trim
G_HOLD_V_MAIN = 1.0       # m/s    main engine only
G_DV_RATIO = 1.3          #        main-engine dv / plan
G_DRIFT = 10.0            # m      free drift vs CW (informational line, not part of PASS)
ECC_MAX = 0.01
REL_INC_MAX = 0.5         # deg    larger: not this tool (plane change > 20 m/s)
PLANE_TRIM = 0.05         # deg    smaller: ignored (< 2 m/s, absorbed by the Lambert correction)
NULL_TARGET = 0.3         # m/s    main-engine nulling stops below this
DESIGN_F = (2000.0, 4000.0, 2000.0)   # N RCS force per direction along vessel x / y / z (4 x RV-105 4Horn)
EC_RATE_AWAKE = 0.12      # EC/s: TWO HECS2 cores on the chaser (core + cargo), 2 x kspilot.power.EC_RATE_AWAKE
EC_RATE_HIB = 0.02        # EC/s: 2 x kspilot.power.EC_RATE_HIB (assumed; measure mode reports the awake ec_rate)


# ---------------------------------------------------------------- environment (real game or OrbitWorld)
class Env:
    """Side-effect hooks: sleep (game time passes), wall clock, burn executor, warp. Real game by default."""

    def __init__(self, conn, log, sleep=None, wall=None, burn=None, warp=None, dry=False):
        self.conn, self.sc, self.log, self.dry = conn, conn.space_center, log, dry
        self.sleep = sleep or time.sleep
        self.wall = wall or time.time
        self._burn, self._warp = burn, warp

    def burn(self, v, ut, dv):
        if self._burn is not None:
            return self._burn(self.conn, v, ut, dv, self.log)
        from kspilot import burn, vehicle
        return burn.execute_burn(self.conn, v, ut, dv, self.log, autostage_limit=vehicle.autostage_limit(v))

    def warp(self, ut):
        if self._warp is not None:
            return self._warp(ut)
        core.warp_to(self.sc, ut)


def _vec(x):
    return np.asarray(x, float)


def _fl(x):
    return [float(v) for v in x]


def _name(v):
    try:
        return str(v.name)
    except Exception:
        return "?"


def find_target(sc, chaser, name=None):
    """The target vessel: by (sub)name, else the space centre's target, else the nearest other orbiting non-debris
    vessel around the same body. None if nothing qualifies."""
    cands = [x for x in sc.vessels if _name(x) != _name(chaser)]
    if name:
        exact = [x for x in cands if _name(x) == name]
        sub = [x for x in cands if name.lower() in _name(x).lower()]
        return (exact or sub or [None])[0]
    try:
        t = sc.target_vessel
        if t is not None:
            return t
    except Exception:
        pass
    nrf = chaser.orbit.body.non_rotating_reference_frame
    rc = _vec(chaser.position(nrf))
    best, bd = None, float("inf")
    for x in cands:
        try:
            if "orbiting" not in str(x.situation) or "debris" in str(x.type).lower():
                continue
            d = float(np.linalg.norm(_vec(x.position(nrf)) - rc))
        except Exception:
            continue
        if d < bd:
            best, bd = x, d
    return best


def rel_state(chaser, target, nrf):
    """Inertial states (nrf) and the chaser's position / velocity in the target's LVLH frame (x radial, y along-track,
    z = numeric orbit normal: handedness-consistent with kRPC's left-handed frames, see kspilot.rdv.cw)."""
    rc, vc = _vec(chaser.position(nrf)), _vec(chaser.velocity(nrf))
    rt, vt = _vec(target.position(nrf)), _vec(target.velocity(nrf))
    rho, rhod = cw.eci_to_lvlh(rt, vt, rc, vc)
    return rc, vc, rt, vt, rho, rhod


def phase_angle(rc, vc, rt):
    """Target angle ahead of the chaser along its motion, in [0, 2 pi)."""
    h = np.cross(rc, vc)
    h /= np.linalg.norm(h)
    return math.atan2(float(np.dot(h, np.cross(rc, rt))), float(np.dot(rc, rt))) % (2 * math.pi)


def elements(r, v, mu):
    r, v = _vec(r), _vec(v)
    h = np.cross(r, v)
    e = np.cross(v, h) / mu - r / np.linalg.norm(r)
    a = 1.0 / (2 / np.linalg.norm(r) - float(np.dot(v, v)) / mu)
    return a, float(np.linalg.norm(e)), h / np.linalg.norm(h)


def noise_stats(t, X, seg=30.0, deg=3):
    """1-sigma white noise of samples X (n x 3) at times t: cubic fit per axis in segments of `seg` s (removes the
    relative motion: the quartic CW term over 30 s at 1 km is ~4 mm), pooled residual RMS with the dof correction.
    Returns (sigma per axis, sigma of the 3-D norm / sqrt(3) = per-axis equivalent, n segments used)."""
    t, X = _vec(t), _vec(X)
    if len(t) < 2 * (deg + 2):
        return [float("nan")] * 3, float("nan"), 0
    edges = np.arange(t[0], t[-1] + 1e-9, seg)
    if len(edges) < 2 or t[-1] - edges[-1] > seg / 2:
        edges = np.append(edges, t[-1] + 1e-9)
    ss, dof, nseg = np.zeros(3), 0, 0
    for a, b in zip(edges[:-1], edges[1:]):
        m = (t >= a) & (t < b) if b < edges[-1] else (t >= a) & (t <= b)
        if m.sum() < deg + 3:
            continue
        tt = t[m] - t[m].mean()
        for k in range(3):
            c = np.polyfit(tt, X[m, k], deg)
            ss[k] += float(np.sum((X[m, k] - np.polyval(c, tt)) ** 2))
        dof += int(m.sum()) - (deg + 1)
        nseg += 1
    if dof <= 0:
        return [float("nan")] * 3, float("nan"), 0
    sig = np.sqrt(ss / dof)
    return [float(s) for s in sig], float(math.sqrt(float(np.sum(sig ** 2)) / 3.0)), nseg


def pct(x, q):
    x = [v for v in x if v is not None and math.isfinite(v)]
    return float(np.percentile(x, q)) if x else float("nan")


def _angle(a, b):
    a, b = _vec(a), _vec(b)
    c = float(np.dot(a, b) / max(np.linalg.norm(a) * np.linalg.norm(b), 1e-12))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


_ERR = object()


def _safe(fn, default=_ERR):
    """fn() or, on any exception, `default` (by default the text 'ERR <type>' for the survey record)."""
    try:
        return fn()
    except Exception as e:  # noqa: BLE001 - read-only survey: record and go on
        return "ERR %s" % type(e).__name__ if default is _ERR else default


# ---------------------------------------------------------------- survey (read only)
def port_info(port, own, other, nrf):
    d = dict(title=_safe(lambda: port.part.title), state=_safe(lambda: str(port.state)),
             shielded=_safe(lambda: bool(port.shielded)))
    d["pos_own"] = _safe(lambda: _fl(port.position(own.reference_frame)))
    d["dir_own"] = _safe(lambda: _fl(port.direction(own.reference_frame)))
    if other is not None:
        d["pos_other"] = _safe(lambda: _fl(port.position(other.reference_frame)))
        d["dir_other"] = _safe(lambda: _fl(port.direction(other.reference_frame)))
    d["dir_nrf"] = _safe(lambda: _fl(port.direction(nrf)))
    return d


def rcs_info(v):
    rf = v.reference_frame
    out, pos = [], []
    for r in v.parts.rcs:
        d = dict(title=_safe(lambda: r.part.title))
        for f in ("enabled", "pitch_enabled", "yaw_enabled", "roll_enabled", "forward_enabled", "up_enabled",
                  "right_enabled", "has_fuel"):
            d[f] = _safe(lambda f=f: bool(getattr(r, f)))
        for f in ("max_thrust", "available_thrust", "thrust_limit"):
            d[f] = _safe(lambda f=f: float(getattr(r, f)))
        p = _safe(lambda: _fl(r.part.position(rf)))
        d["pos"] = p
        if isinstance(p, list):
            pos.append(p)
        d["thrusters"] = _safe(lambda: [dict(dir=_fl(t.thrust_direction(rf)), pos=_fl(t.thrust_position(rf)))
                                        for t in r.thrusters])
        d["max_temp"] = _safe(lambda: [float(r.part.max_temperature), float(r.part.max_skin_temperature)])
        out.append(d)
    centre = _fl(np.mean(pos, axis=0)) if pos else None
    return out, centre


def survey(sc, chaser, target, nrf):
    s = dict(ut=float(sc.ut), vessels=[])
    rc = _vec(chaser.position(nrf))
    for x in sc.vessels:
        e = dict(name=_name(x), type=_safe(lambda: str(x.type)), situation=_safe(lambda: str(x.situation)),
                 loaded=_safe(lambda: bool(x.loaded)), packed=_safe(lambda: bool(x.packed)))
        e["sma_km"] = _safe(lambda: x.orbit.semi_major_axis / 1e3)
        e["ecc"] = _safe(lambda: float(x.orbit.eccentricity))
        e["inc_deg"] = _safe(lambda: math.degrees(x.orbit.inclination))
        e["dist_m"] = _safe(lambda: float(np.linalg.norm(_vec(x.position(nrf)) - rc)))
        s["vessels"].append(e)
    for key, own, other in (("chaser", chaser, target), ("target", target, chaser)):
        if own is None:
            continue
        d = dict(name=_name(own), loaded=_safe(lambda: bool(own.loaded)), mass=_safe(lambda: float(own.mass)))
        if d["loaded"] is True:
            d["ports"] = _safe(lambda: [port_info(p, own, other, nrf) for p in own.parts.docking_ports], [])
            d["fairings"] = _safe(lambda: [bool(f.jettisoned) for f in own.parts.fairings], [])
        for res in ("MonoPropellant", "ElectricCharge", "LiquidFuel"):
            d[res] = _safe(lambda res=res: [float(own.resources.amount(res)), float(own.resources.max(res))])
        s[key] = d
    rcs, centre = rcs_info(chaser)
    s["chaser"]["rcs"], s["chaser"]["rcs_centre"] = rcs, centre
    s["chaser"]["available_rcs_force"] = _safe(lambda: [_fl(f) for f in chaser.available_rcs_force])
    if target is not None:
        rc, vc, rt, vt, rho, rhod = rel_state(chaser, target, nrf)
        s["rel"] = dict(rho=_fl(rho), rhod=_fl(rhod), dist=float(np.linalg.norm(rc - rt)),
                        phase_deg=math.degrees(phase_angle(rc, vc, rt)),
                        rel_inc_deg=_safe(lambda: math.degrees(chaser.orbit.relative_inclination(target.orbit))))
        ca = _safe(lambda: chaser.orbit.next_closest_approach(target.orbit), None)
        s["closest"] = _safe(lambda: dict(ut=float(ca.ut), distance=float(ca.distance)), None) if ca is not None \
            else None
    return s


def sample(env, chaser, target, nrf, secs, hz):
    """kRPC sampling for `secs` s at `hz`.
    Both vessels loaded (ports readable): the A4 measurement is the ROTATION-FREE port-to-port vector, chaser port minus
    target port in the body's non-rotating frame (R0 models navigation noise and the target's attitude sway
    separately); the same vector in the target PORT frame (what the docking guidance will see, sway included) is
    recorded too ("p_port"). Otherwise: chaser in the target vessel frame, else the chaser in nrf."""
    sc = env.sc
    pc = pt = None
    try:
        if target is not None and target.loaded:
            pc, pt = chaser.parts.docking_ports[0], target.parts.docking_ports[0]
    except Exception:
        pc = pt = None
    port_rf = None
    if pc is not None and pt is not None:
        src = "port->port"
        port_rf = pt.reference_frame
        rd = (lambda: _vec(pc.position(nrf)) - _vec(pt.position(nrf)),
              lambda: _vec(pc.part.velocity(nrf)) - _vec(pt.part.velocity(nrf)))
    elif target is not None:
        src = "vessel->target vessel frame"
        rf = target.reference_frame
        rd = (lambda: chaser.position(rf), lambda: chaser.velocity(rf))
    else:
        src = "chaser in nrf (no target)"
        rd = (lambda: chaser.position(nrf), lambda: chaser.velocity(nrf))
    rows, n, dt = [], max(int(secs * hz), 1), 1.0 / hz
    last_log = env.wall()
    for i in range(n):
        w0 = env.wall()
        ut = float(sc.ut)
        lat = env.wall() - w0
        try:
            p, v = _fl(rd[0]()), _fl(rd[1]())
        except Exception:
            p = v = None
        pp = tdir = None
        if port_rf is not None:
            try:
                pp = _fl(pc.position(port_rf))
                tdir = _fl(pt.direction(nrf))
            except Exception:
                pp = tdir = None
        lvlh = None
        if target is not None and (i % max(int(hz), 1) == 0):
            try:
                rho = rel_state(chaser, target, nrf)[4]
                rt, vt = _vec(target.position(nrf)), _vec(target.velocity(nrf))
                M, _ = cw.lvlh_basis(rt, vt)
                lvlh = dict(rho=_fl(rho), tdir_lvlh=_fl(M @ _vec(tdir)) if tdir is not None else None)
            except Exception:
                lvlh = None
        rows.append(dict(ut=ut, lat=lat, p=p, v=v, p_port=pp, tdir=tdir, lvlh=lvlh))
        if env.wall() - last_log > 30:
            env.log("measure: %d/%d samples" % (i + 1, n))
            last_log = env.wall()
        env.sleep(dt)
    good = [r for r in rows if r["p"] is not None]
    t = [r["ut"] for r in good]
    sp, sp3, nseg = noise_stats(t, [r["p"] for r in good])
    sv, sv3, _ = noise_stats(t, [r["v"] for r in good])
    lat = [r["lat"] for r in rows]
    uts = [r["ut"] for r in rows]
    st = dict(source=src, n=len(rows), n_good=len(good), segments=nseg, sigma_pos_axes=sp, sigma_pos=sp3,
              sigma_vel_axes=sv, sigma_vel=sv3, latency_p50=pct(lat, 50), latency_p95=pct(lat, 95),
              latency_max=max(lat) if lat else float("nan"),
              ut_span=(uts[-1] - uts[0]) if uts else 0.0,
              ut_unique=len(set(uts)))
    gp = [r for r in rows if r["p_port"] is not None]
    if gp:
        st["sigma_pos_port_frame_axes"], st["sigma_pos_port_frame"], _ = noise_stats(
            [r["ut"] for r in gp], [r["p_port"] for r in gp])
    tdirs = [r["tdir"] for r in rows if r["tdir"] is not None]
    if len(tdirs) >= 2:
        st["target_axis_drift_inertial_deg"] = max(_angle(tdirs[0], d) for d in tdirs)
        ll = [r["lvlh"]["tdir_lvlh"] for r in rows if r["lvlh"] and r["lvlh"]["tdir_lvlh"] is not None]
        if len(ll) >= 2:
            st["target_axis_drift_lvlh_deg"] = max(_angle(ll[0], d) for d in ll)
    return st, rows


def measure_gates(s, st):
    """[(name, 'OK'|'FAIL'|'NA', detail)] for the measured survey s + sampling stats st."""
    g = []
    ch = s["chaser"]
    rcs = ch.get("rcs", [])
    if not rcs:
        g.append(("rcs present", "FAIL", "no RCS on the chaser"))
    else:
        rot = [r for r in rcs if any(r.get(f) is True for f in ("pitch_enabled", "yaw_enabled", "roll_enabled"))]
        g.append(("A7 rcs rotation off", "FAIL" if rot else "OK", "%d of %d blocks with rotation on" % (len(rot),
                                                                                                         len(rcs))))
        c = ch.get("rcs_centre")
        if c is not None:
            g.append(("A2 thrust centre", "OK" if abs(c[1]) <= G_TC_AXIAL else "FAIL",
                      "axial %+.2f m, lateral %.2f m from CoM" % (c[1], math.hypot(c[0], c[2]))))
        f = ch.get("available_rcs_force")
        if isinstance(f, list) and len(f) == 2:
            dev = max(abs(abs(f[s_][k]) / DESIGN_F[k] - 1.0) for s_ in (0, 1) for k in range(3))
            g.append(("rcs force", "OK" if dev <= G_FORCE_TOL else "FAIL",
                      "+%s / -%s N, max deviation %.0f %%" % (f[0], f[1], 100 * dev)))
    mono = ch.get("MonoPropellant")
    if isinstance(mono, list):
        g.append(("monoprop", "OK" if mono[0] >= G_MONO_MIN else "FAIL", "%.1f / %.0f u" % tuple(mono)))
    ports = ch.get("ports") or []
    if ports:
        p = ports[0]
        dirok = isinstance(p.get("dir_own"), list) and p["dir_own"][1] > 0.999
        g.append(("chaser port axis = vessel +y", "OK" if dirok else "FAIL", "dir %s, pos %s, %s" % (
            p.get("dir_own"), p.get("pos_own"), p.get("state"))))
        g.append(("chaser port ready", "OK" if "ready" in str(p.get("state")).lower() else "FAIL", str(p.get("state"))))
    tp = (s.get("target") or {}).get("ports")
    if tp:
        g.append(("target port ready", "OK" if "ready" in str(tp[0].get("state")).lower() else "FAIL",
                  str(tp[0].get("state"))))
    else:
        g.append(("target port", "NA", "target not loaded or no port"))
    g.append(("A3 rpc latency p95", "OK" if st["latency_p95"] <= G_LAT_P95 else "FAIL",
              "%.3f s (p50 %.3f, max %.3f)" % (st["latency_p95"], st["latency_p50"], st["latency_max"])))
    if st["source"] == "port->port":
        ok = st["sigma_pos"] <= G_NOISE_POS and st["sigma_vel"] <= G_NOISE_VEL
        g.append(("A4 port noise", "OK" if ok else "FAIL", "sigma pos %.4f m (<= %.2f), vel %.4f m/s (<= %.3f), %d "
                  "segments" % (st["sigma_pos"], G_NOISE_POS, st["sigma_vel"], G_NOISE_VEL, st["segments"])))
    else:
        g.append(("A4 port noise", "NA", "%s: sigma pos %.4f m (not the docking measurement)" % (st["source"],
                                                                                              st["sigma_pos"])))
    return g


def do_measure(env, args):
    sc, log = env.sc, env.log
    chaser = sc.active_vessel
    target = find_target(sc, chaser, args.target)
    nrf = chaser.orbit.body.non_rotating_reference_frame
    log("measure: chaser %s, target %s (read only, %.0f s at %.0f Hz)" % (
        _name(chaser), _name(target) if target is not None else "NONE", args.secs, args.hz))
    s = survey(sc, chaser, target, nrf)
    e0 = s["chaser"].get("ElectricCharge")
    st, rows = sample(env, chaser, target, nrf, args.secs, args.hz)
    e1 = _safe(lambda: [float(chaser.resources.amount("ElectricCharge")), float(chaser.resources.max("ElectricCharge"))])
    if isinstance(e0, list) and isinstance(e1, list) and st["ut_span"] > 0:
        st["ec_rate"] = (e0[0] - e1[0]) / st["ut_span"]
    gates = measure_gates(s, st)
    out = dict(mode="measure", survey=s, stats=st, gates=gates, samples=rows, args=vars(args))
    path = os.path.join(core.LOG_DIR, "rdv_r1_measure_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(core.LOG_DIR, exist_ok=True)
    with open(path, "w") as f:
        json.dump(out, f, indent=1, default=str)
    for name, res, det in gates:
        log("gate %-30s %-4s %s" % (name, res, det))
    if "rel" in s:
        log("relative (LVLH): rho %s m, rhod %s m/s, distance %.0f m, phase %.2f deg" % (
            np.round(s["rel"]["rho"], 1), np.round(s["rel"]["rhod"], 3), s["rel"]["dist"], s["rel"]["phase_deg"]))
    for k in ("sigma_pos_port_frame", "target_axis_drift_inertial_deg", "target_axis_drift_lvlh_deg", "ec_rate"):
        if k in st:
            log("%s = %.4f" % (k, st[k]))
    log("data: %s" % path)
    fails = [n for n, r, _ in gates if r == "FAIL"]
    log("RESULT", "FAIL" if fails else "PASS", "measure gates %d OK / %d FAIL / %d NA%s" % (
        sum(r == "OK" for _, r, _ in gates), len(fails), sum(r == "NA" for _, r, _ in gates),
        (" (" + ", ".join(fails) + ")") if fails else ""))
    return out, path


# ---------------------------------------------------------------- prep (small actions)
def do_prep(env, args):
    sc, log = env.sc, env.log
    cands = [x for x in sc.vessels if args.vessel and args.vessel.lower() in _name(x).lower()]
    if len(cands) != 1:
        log("RESULT REFUSED prep: %d vessels match %r: %s" % (len(cands), args.vessel, [_name(x) for x in sc.vessels]))
        return False
    v = cands[0]
    if _name(sc.active_vessel) != _name(v):
        sc.active_vessel = v
        env.sleep(5.0)
    v = sc.active_vessel                     # re-acquire after a switch
    log("prep: active vessel %s (%s, %s)" % (_name(v), _safe(lambda: str(v.situation)), _safe(lambda: str(v.type))))
    if args.rename:
        v.name = args.rename
        log("prep: renamed to %s" % args.rename)
    if args.recover_landed:                  # e.g. the core that landed on the pad in the station flight
        for x in list(sc.vessels):
            sit = str(_safe(lambda: x.situation))
            if _name(x) != _name(v) and any(k in sit for k in ("landed", "pre_launch", "splashed")):
                if _safe(lambda: bool(x.recoverable), False):
                    x.recover()
                    log("prep: recovered %s (%s)" % (_name(x), sit))
                else:
                    log("prep: WARNING %s (%s) not recoverable" % (_name(x), sit))
    if args.jettison:
        n = 0
        for f in v.parts.fairings:
            if not f.jettisoned:
                f.jettison()
                n += 1
        log("prep: %d fairing(s) jettisoned" % n)
        env.sleep(3.0)
    if args.sas:
        v.control.sas = True
        env.sleep(0.5)
        try:
            v.control.sas_mode = env.sc.SASMode.stability_assist       # enum on the SpaceCenter service (real game)
        except Exception:
            pass
        log("prep: SAS %s, mode %s" % (_safe(lambda: v.control.sas), _safe(lambda: v.control.sas_mode)))
    if args.save:
        sc.save(args.save)
        log("prep: saved %s" % args.save)
    log("RESULT PASS prep %s" % _name(v))
    return True


# ---------------------------------------------------------------- rendezvous
class Rendezvous:
    def __init__(self, env, args):
        self.env, self.args, self.log = env, args, env.log
        self.sc = env.sc
        self.burns = []          # (label, ut, |dv| planned, |dv| left)
        self.track = []
        self.dv_plan = 0.0

    # ---- state helpers
    def vessels(self):
        v = self.sc.active_vessel                  # re-acquired every time
        return v, self.target

    def state(self):
        v, t = self.vessels()
        return rel_state(v, t, self.nrf)

    def at(self, ut):
        """Two-body prediction of both vessels at ut from the current state."""
        rc, vc, rt, vt, _, _ = self.state()
        dt = ut - float(self.sc.ut)
        return kepler_propagate(rc, vc, dt, self.mu) + kepler_propagate(rt, vt, dt, self.mu)

    def aim(self):
        return np.array([0.0, -float(self.args.aim), 0.0])

    def fly(self, label, ut, dv):
        v, _ = self.vessels()
        dv = _vec(dv)
        mag = float(np.linalg.norm(dv))
        self.log("%s: %.2f m/s at UT %.0f (in %.0f s)" % (label, mag, ut, ut - float(self.sc.ut)))
        left = self.env.burn(v, ut, dv)
        if left is None:
            raise RuntimeError("%s missed (burn executor refused a late burn)" % label)
        self.burns.append((label, float(ut), mag, float(left)))
        return left

    def coast(self, ut_burn, t_after):
        """Warp to ~10 min before ut_burn; power budgeted (hibernate in warp when the budget needs it)."""
        from kspilot import power as pwr
        v, _ = self.vessels()
        now = float(self.sc.ut)
        e, emax = pwr.ec(v)
        rates = dict(rate_hib=EC_RATE_HIB, rate_awake=EC_RATE_AWAKE)
        b = pwr.budget(e, emax, ut_burn - now, t_after, False, **rates)
        hib = False
        if not b.ok:
            if pwr.check_toggle(v, self.log):
                b = pwr.budget(e, emax, ut_burn - now, t_after, True, **rates)
                hib = b.ok
            if not b.ok:
                raise Refused("power: %s" % b.text())
        self.log("coast %.0f s to UT %.0f: %s" % (ut_burn - now, ut_burn, b.text()))
        if ut_burn - now > 900:
            if hib:
                pwr.sleep(v, self.log)
            self.env.warp(ut_burn - max(600.0, pwr.WAKE_LEAD))
            if hib:
                v, _ = self.vessels()
                if not pwr.wake(v, self.log):
                    raise RuntimeError("wake-up after the coast not verified")

    # ---- phases
    def preflight(self):
        args = self.args
        v = self.sc.active_vessel
        self.target = find_target(self.sc, v, args.target)
        if self.target is None:
            raise Refused("no target vessel")
        body = v.orbit.body
        self.nrf, self.mu = body.non_rotating_reference_frame, float(body.gravitational_parameter)
        rc, vc, rt, vt, rho, rhod = self.state()
        ac, ec_, hc = elements(rc, vc, self.mu)
        at, et, ht = elements(rt, vt, self.mu)
        ri = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(hc, ht))))))
        self.log("rendezvous: chaser %s a %.1f km e %.4f | target %s a %.1f km e %.4f | rel inc %.3f deg | dist %.1f "
                 "km" % (_name(v), ac / 1e3, ec_, _name(self.target), at / 1e3, et, ri,
                         np.linalg.norm(rc - rt) / 1e3))
        d = float(np.linalg.norm(rc - rt))
        if d < 5000.0:
            return "terminal"
        if "orbiting" not in str(v.situation):
            raise Refused("chaser not orbiting (%s)" % v.situation)
        if et > ECC_MAX:
            raise Refused("target orbit not circular (e %.4f)" % et)
        if ri > REL_INC_MAX:
            raise Refused("relative inclination %.2f deg > %.1f" % (ri, REL_INC_MAX))
        if ec_ > ECC_MAX:
            raise Refused("chaser orbit not circular (e %.4f): mid-transfer restarts are not supported" % ec_)
        if at - ac < 5000.0:
            raise Refused("target not above the chaser (%.1f km)" % ((at - ac) / 1e3))
        try:
            self.sc.target_vessel = self.target
        except Exception:
            pass
        return "plane" if ri > PLANE_TRIM else "phasing"

    def plane(self):
        """Remove the out-of-plane velocity at the next crossing of the target plane."""
        rc, vc, rt, vt, _, _ = self.state()
        ht = np.cross(rt, vt)
        ht /= np.linalg.norm(ht)
        T = 2 * math.pi * math.sqrt(elements(rc, vc, self.mu)[0] ** 3 / self.mu)
        f = lambda t: float(np.dot(kepler_propagate(rc, vc, t, self.mu)[0], ht))  # noqa: E731
        t0, f0 = 120.0, f(120.0)
        tn = None
        for t1 in np.arange(140.0, 120.0 + T, 20.0):
            f1 = f(t1)
            if f0 * f1 <= 0:
                lo, hi = t0, t1
                for _ in range(40):
                    m = 0.5 * (lo + hi)
                    lo, hi = (lo, m) if f(lo) * f(m) <= 0 else (m, hi)
                tn = 0.5 * (lo + hi)
                break
            t0, f0 = t1, f1
        if tn is None:
            raise RuntimeError("no node found")
        ut = float(self.sc.ut) + tn
        _, vn = kepler_propagate(rc, vc, tn, self.mu)
        dv = -float(np.dot(vn, ht)) * ht
        self.dv_plan += float(np.linalg.norm(dv))
        self.coast(ut, 1800.0)                  # the phasing coast after it is budgeted on its own
        self.fly("plane match", ut, dv)

    def phasing(self):
        rc, vc, rt, vt, _, _ = self.state()
        ac = elements(rc, vc, self.mu)[0]
        at = elements(rt, vt, self.mu)[0]
        lead = 300.0                                       # s: never plan a burn closer than this
        plan = ob.plan_hohmann_rendezvous(ac, at, phase_angle(rc, vc, rt), self.args.aim, self.mu)
        if plan.t_wait < lead:                             # too close to align and burn: next synodic window
            plan.t_wait += plan.synodic
        self.plan = plan
        self.dv_plan += plan.dv_total
        now = float(self.sc.ut)
        self.ut_b1, self.ut_arr = now + plan.t_wait, now + plan.t_wait + plan.tof
        self.log("plan: wait %.0f s (synodic %.1f h), dv1 %.2f m/s, transfer %.0f s, dv2 %.2f m/s, total %.1f m/s" % (
            plan.t_wait, plan.synodic / 3600, plan.dv1, plan.tof, plan.dv2, plan.dv_total))
        self.coast(self.ut_b1, plan.tof + 3600.0)
        rc1, vc1, _, _ = self.at(self.ut_b1)
        self.fly("burn 1 (Hohmann)", self.ut_b1, plan.dv1 * vc1 / np.linalg.norm(vc1))

    def predicted_miss(self, ut_arr):
        rc, vc, rt, vt = self.at(ut_arr)
        rho, _ = cw.eci_to_lvlh(rt, vt, rc, vc)
        return rho - self.aim(), rho

    def lambert_burns(self, ut_now, ut_arr):
        rc, vc, rt, vt = self.at(ut_now)
        pole = np.cross(rc, vc)
        return ob.lambert_intercept(rc, vc, rt, vt, ut_arr - ut_now, aim_lvlh=self.aim(), mu=self.mu,
                                    pole=pole / np.linalg.norm(pole))

    def transfer(self):
        tof = self.ut_arr - self.ut_b1
        ut_mc = self.ut_b1 + 0.5 * tof
        self.env.warp(ut_mc - 180.0)
        miss, _ = self.predicted_miss(self.ut_arr)
        self.log("mid-course: predicted arrival error %s m (|%.0f|)" % (np.round(miss, 0), np.linalg.norm(miss)))
        if np.linalg.norm(miss) > 50.0:
            ut_c = float(self.sc.ut) + 120.0
            dv1, _, _ = self.lambert_burns(ut_c, self.ut_arr)
            self.fly("mid-course (Lambert)", ut_c, dv1)
        self.env.warp(self.ut_arr - 240.0)
        miss, rho = self.predicted_miss(self.ut_arr)
        self.log("arrival: predicted position %s m (error %.0f m)" % (np.round(rho, 0), np.linalg.norm(miss)))
        rc, vc, rt, vt = self.at(self.ut_arr)
        _, v_aim = cw.lvlh_to_eci(rt, vt, self.aim(), np.zeros(3))
        self.fly("burn 2 (match the hold point)", self.ut_arr, v_aim - vc)

    def null(self):
        """Main-engine nulling of the LVLH relative velocity (3 tries max), each burn planned 60 s ahead by CW."""
        for i in range(3):
            rc, vc, rt, vt, rho, rhod = self.state()
            if np.linalg.norm(rhod) < NULL_TARGET:
                break
            n = cw.mean_motion_at(np.linalg.norm(rt) - ob.R_KERBIN)
            dt = 60.0
            rho2, rhod2 = cw.propagate(n, rho, rhod, dt)
            rt2, vt2 = kepler_propagate(rt, vt, dt, self.mu)
            M, _ = cw.lvlh_basis(rt2, vt2)
            self.fly("null %d" % (i + 1), float(self.sc.ut) + dt, M.T @ (-rhod2))

    def rcs_trim(self):
        """RCS translation trim of the LVLH relative velocity to < G_HOLD_V_RCS/2. Control mapping right/forward/up ->
        vessel +x/+y/-z [ASSUMED], checked by a first pulse (polarity + magnitude) before the loop."""
        v, _ = self.vessels()
        rfv = v.reference_frame
        fpos, fneg = [np.abs(_vec(f)) for f in v.available_rcs_force]
        m = float(v.mass)
        v.control.sas = True                               # hold attitude (reaction wheels: RCS rotation is off)
        v.control.rcs = True
        sign = np.array([1.0, 1.0, -1.0])                  # vessel axis -> control input sign (right, forward, up)

        def need():
            """Needed dv in the vessel axes (re-read every call: the attitude may wander) and the LVLH rhod."""
            axes = np.array([self.sc.transform_direction(tuple(e), rfv, self.nrf) for e in np.eye(3)])  # rows x,y,z
            rc, vc, rt, vt, rho, rhod = self.state()
            M, _ = cw.lvlh_basis(rt, vt)
            return axes @ (M.T @ (-rhod)), rhod

        def pulse(cmd, dur):
            c = v.control
            c.right, c.forward, c.up = (float(x) for x in cmd * sign)
            self.env.sleep(dur)
            c.right = c.forward = c.up = 0.0

        try:
            dvb, rhod = need()
            k = int(np.argmax(np.abs(dvb)))
            a = (fpos if dvb[k] > 0 else fneg)[k] / m
            dur = min(0.5, abs(dvb[k]) / max(a, 1e-6))
            cmd = np.zeros(3)
            cmd[k] = math.copysign(1.0, dvb[k])
            before = dvb.copy()
            pulse(cmd, dur)
            after, _ = need()
            got = before[k] - after[k]
            exp = a * dur * math.copysign(1.0, dvb[k])
            self.log("rcs polarity pulse axis %d: expected %+.4f m/s, measured %+.4f m/s" % (k, exp, got))
            if got * exp <= 0 or not 0.3 < got / exp < 2.0:
                raise RuntimeError("RCS polarity / magnitude check failed (axis %d: %.4f vs %.4f)" % (k, got, exp))
            for it in range(40):
                dvb, rhod = need()
                if np.linalg.norm(rhod) < G_HOLD_V_RCS / 2:
                    break
                acc = np.where(dvb > 0, fpos, fneg) / m
                T = 1.0
                cmd = np.clip(dvb / np.maximum(acc * T, 1e-6), -1.0, 1.0)
                cmd[np.abs(dvb) < 0.01] = 0.0
                pulse(cmd, T)
            self.log("rcs trim: |rhod| %.3f m/s" % float(np.linalg.norm(self.state()[5])))
        finally:
            v.control.right = v.control.forward = v.control.up = 0.0
            v.control.rcs = False

    def drift(self, secs):
        rc, vc, rt, vt, rho0, rhod0 = self.state()
        n = cw.mean_motion_at(np.linalg.norm(rt) - ob.R_KERBIN)
        t0, err, rows = float(self.sc.ut), 0.0, []
        while float(self.sc.ut) - t0 < secs:
            self.env.sleep(2.0)
            rho = self.state()[4]
            t = float(self.sc.ut) - t0
            pred, _ = cw.propagate(n, rho0, rhod0, t)
            e = float(np.linalg.norm(rho - pred))
            err = max(err, e)
            rows.append((t, _fl(rho), _fl(pred)))
            if len(rows) % 15 == 0:
                self.log("drift %.0f s: rho %s m, CW error %.2f m" % (t, np.round(rho, 1), e))
        return err, rows

    def run(self):
        args = self.args
        phase = self.preflight()
        if phase == "plane":
            self.plane()
            phase = "phasing"
        if phase == "phasing":
            self.phasing()
            self.transfer()
        self.null()
        if args.rcs_trim:
            self.rcs_trim()
        rc, vc, rt, vt, rho, rhod = self.state()
        perr = float(np.linalg.norm(rho - self.aim()))
        vrel = float(np.linalg.norm(rhod))
        used = sum(mag - left for _, _, mag, left in self.burns)
        gv = G_HOLD_V_RCS if args.rcs_trim else G_HOLD_V_MAIN
        gates = [("hold position", perr <= G_HOLD_POS, "rho %s m, error %.0f m (<= %.0f)" % (
                     np.round(rho, 1), perr, G_HOLD_POS)),
                 ("relative velocity", vrel < gv, "%.3f m/s (< %.1f, %s)" % (
                     vrel, gv, "RCS trim" if args.rcs_trim else "main engine only"))]
        if self.dv_plan > 0:
            gates.append(("main-engine dv", used <= G_DV_RATIO * self.dv_plan, "%.1f m/s used vs plan %.1f (<= x%.1f)" % (
                used, self.dv_plan, G_DV_RATIO)))
        for name, ok, det in gates:
            self.log("gate %-20s %s %s" % (name, "OK  " if ok else "FAIL", det))
        drift = None
        if args.drift > 0:
            err, rows = self.drift(args.drift)
            drift = dict(max_err=err, rows=rows)
            self.log("DRIFT %s: free drift %.0f s, max CW prediction error %.2f m (gate %.0f m, R0 step 6)" % (
                "PASS" if err < G_DRIFT else "FAIL", args.drift, err, G_DRIFT))
        ok = all(g[1] for g in gates)
        out = dict(mode="rendezvous", gates=[(n, bool(o), d) for n, o, d in gates], burns=self.burns,
                   dv_plan=self.dv_plan, dv_used=used, rho=_fl(rho), rhod=_fl(rhod), drift=drift, args=vars(args))
        if getattr(self, "plan", None) is not None:
            p = self.plan
            out["plan"] = dict(t_wait=p.t_wait, dv1=p.dv1, tof=p.tof, dv2=p.dv2, synodic=p.synodic)
        path = os.path.join(core.LOG_DIR, "rdv_r1_rdv_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(core.LOG_DIR, exist_ok=True)
        with open(path, "w") as f:
            json.dump(out, f, indent=1, default=str)
        if ok and args.save:
            self.sc.save(args.save)
            self.log("saved %s" % args.save)
        self.log("RESULT", "PASS" if ok else "FAIL", "rendezvous hold err %.0f m, |v| %.3f m/s, dv %.1f/%.1f m/s%s" % (
            perr, vrel, used, self.dv_plan, "" if ok else " (" + ", ".join(n for n, o, _ in gates if not o) + ")"))
        return ok, out, path


class Refused(Exception):
    pass


# ---------------------------------------------------------------- entry points
def parse(argv):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("mode", choices=("measure", "prep", "rendezvous"))
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--target")
    ap.add_argument("--secs", type=float, default=60.0)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--vessel")
    ap.add_argument("--rename")
    ap.add_argument("--jettison", action="store_true")
    ap.add_argument("--sas", action="store_true")
    ap.add_argument("--recover-landed", action="store_true")
    ap.add_argument("--save")
    ap.add_argument("--aim", type=float, default=1000.0)
    ap.add_argument("--rcs-trim", action="store_true")
    ap.add_argument("--drift", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=1)
    return ap.parse_args(argv)


def dry_world(mode, seed=1, **kw):
    sys.path.insert(0, os.path.join(ROOT, "tests"))
    import fake_krpc as fk
    if mode == "measure":
        d = dict(alt_c=100e3, alt_t=100e3, phase0=1000.0 / 700e3)
    else:
        d = dict(alt_c=80e3, alt_t=100e3, phase0=2.0, burn_err=0.003, hibernate_ok=True)
    d.update(kw)
    return fk.OrbitWorld(seed=seed, **d)


def run(args, conn=None, env=None, log=None):
    if log is None:                            # (an empty list-like test logger is falsy: test identity)
        log = core.Log("rdv_r1_%s%s" % (args.mode, "_dry" if args.dry_run else ""))
    world = None
    if env is None:
        if args.dry_run:
            world = dry_world(args.mode, args.seed)
            env = Env(world.conn, log, sleep=world.sleep, burn=world.burn, warp=world.advance_to, dry=True)
            log("DRY RUN against tests/fake_krpc.OrbitWorld (no game)")
        else:
            env = Env(conn or core.connect("rdv_r1"), log)
    try:
        if args.mode == "measure":
            return do_measure(env, args)
        if args.mode == "prep":
            return do_prep(env, args)
        return Rendezvous(env, args).run()
    except Refused as e:
        log("RESULT REFUSED %s" % e)
    except Exception as e:  # noqa: BLE001 - one RESULT line for the watchdog on every path
        log("".join(traceback.format_exception(type(e), e, e.__traceback__)).rstrip())
        try:
            v = env.sc.active_vessel
            v.control.right = v.control.forward = v.control.up = 0.0
            v.control.throttle = 0.0
        except Exception:
            pass
        log("RESULT FAIL exception %s: %s" % (type(e).__name__, e))
    return None


if __name__ == "__main__":
    run(parse(sys.argv[1:]))
