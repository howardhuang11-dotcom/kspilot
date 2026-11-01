"""Deorbit targeting and in-flight impact correction on top of kspilot.entry.

Burn model: finite burn, constant thrust, attitude frozen in inertial space at ignition with
components (prograde, normal, radial) = (-dv_retro, dv_normal, 0) -- i.e. exactly what executing a
KSP manoeuvre node with that burn vector does. All inputs are kRPC body-frame states (see entry.py).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Callable, Optional, Sequence

import numpy as np
from scipy.optimize import brentq

from . import entry as E
from .entry import Body, Burn, Impact, KERBIN, KSC_PAD, KSC_PAD_ALT


@dataclass
class Vehicle:
    mass: float                      # kg (at deorbit ignition)
    thrust: float                    # N (full throttle)
    isp: float                       # s (vacuum)
    drag: Optional[Callable] = None  # drag(alt, airspeed, mach) -> N
    cda: Optional[float] = None      # used when drag is None (ballistic model)
    throttle: float = 1.0

    def drag_fn(self, body: Body = KERBIN):
        return self.drag if self.drag is not None else (E.ballistic(self.cda, body) if self.cda else None)


@dataclass
class DeorbitPlan:
    t_ign: float                 # s after the input state (UT0) to ignite the deorbit burn
    t_node: float                # s after UT0 at which half its dv is delivered (KSP node time)
    burn_time: float
    dv_retro: float              # m/s (positive = retrograde)
    dv_normal: float             # m/s (+ = KSP orbit-normal, north for prograde orbits); in the
                                 # plane-change burn at t_pc, or folded into the deorbit burn if t_pc is None
    t_pc: Optional[float]        # s after UT0, plane-change ignition (None: no separate burn)
    burns: list                  # entry.Burn objects (absolute times from UT0) reproducing the plan
    nodes: list                  # [(t_node from UT0, prograde, normal, radial)] for control.add_node
    impact: Impact               # impact.t is measured from the first burn of the plan
    t_impact: float              # s after UT0
    along: float                 # m, impact relative to aim point (after bias), + = long
    cross: float                 # m, + = right of track
    heading: float               # deg, ground-track heading at the target used for along/cross
    dir_ign_bf: np.ndarray = field(repr=False)   # deorbit thrust unit vector at ignition (body axes then)
    candidates: list = field(default_factory=list, repr=False)   # [(t_ign, cross)] scanned roots

    @property
    def dv(self):
        return self.dv_retro + abs(self.dv_normal) if self.t_pc is not None else math.hypot(self.dv_retro, self.dv_normal)

    @property
    def miss(self):
        return math.hypot(self.along, self.cross)


def _retro_dv_for_periapsis(r, v, pe_alt, body):
    rn = np.linalg.norm(r)
    rp = body.radius + pe_alt
    return float(np.linalg.norm(v) - math.sqrt(body.mu * (2 / rn - 2 / (rn + rp))))


class _Problem:
    def __init__(self, r_bf, v_bf, veh: Vehicle, target, target_alt, aim_bias, cross_bias, body, entry_burns,
                 check_pe=True):
        self.r0, self.v0 = E.bf_to_inertial(r_bf, v_bf, 0.0, body)
        a, e, self.period, rp, ra = E.orbit_elements(self.r0, self.v0, body.mu)
        if check_pe and rp < body.radius + body.atm_height:
            raise ValueError("periapsis inside the atmosphere: use impact_correction instead")
        self.veh, self.drag, self.body = veh, veh.drag_fn(body), body
        self.target, self.target_alt = target, target_alt
        self.aim_bias, self.cross_bias = aim_bias, cross_bias
        self.entry_burns = list(entry_burns)
        self.heading = None
        self.t_pc = None                 # plane-change burn time (None: normal dv folded into deorbit burn)

    def state_at(self, t):
        return E.kepler_propagate(self.r0, self.v0, t, self.body.mu)

    def burns(self, t_ign, dvr, dvn):
        th, isp, thr = self.veh.thrust, self.veh.isp, self.veh.throttle
        if self.t_pc is None:                         # single burn with a normal component
            return [Burn(t_ign, th, isp, thr, mode="orbital", dv=math.hypot(dvr, dvn), components=(-dvr, dvn, 0.0))]
        out = [Burn(t_ign, th, isp, thr, mode="orbital", dv=dvr, components=(-1.0, 0.0, 0.0))]
        if dvn != 0.0:                                # separate plane-change burn at t_pc
            out.insert(0, Burn(self.t_pc, th, isp, thr, mode="orbital", dv=abs(dvn),
                               components=(0.0, math.copysign(1.0, dvn), 0.0)))
        return out

    def entry(self, t_ign):
        """Entry burns; altitude-triggered ones are armed at deorbit ignition."""
        return [replace(b, t_start=max(b.t_start, t_ign)) if b.alt_trigger is not None else b
                for b in self.entry_burns]

    def run(self, t_ign, dvr, dvn, record=True, **kw):
        ts = t_ign if self.t_pc is None or dvn == 0.0 else min(t_ign, self.t_pc)
        r, v = self.state_at(ts)
        return E.predict_inertial(r, v, self.veh.mass, self.drag, self.burns(t_ign, dvr, dvn) + self.entry(t_ign),
                                  self.body, self.target_alt, t0=ts, record=record, **kw)

    def errors(self, imp, heading=None):
        h = heading if heading is not None else (self.heading if self.heading is not None
                                                 else E.track_heading(imp, self.target))
        al, cr = E.ground_errors(imp.lat, imp.lon, self.target, h, self.body.radius)
        return al - self.aim_bias, cr - self.cross_bias, h


def plan_deorbit(r_bf, v_bf, vehicle: Vehicle, target=KSC_PAD, target_alt=KSC_PAD_ALT, aim_bias=0.0,
                 cross_bias=0.0, dv=None, pe_alt=30e3, t_min=30.0, n_orbits=3.0, cross_mode="pc",
                 wait_cost=1.0, scan_deg=4.0, entry_burns: Sequence[Burn] = (), body: Body = KERBIN,
                 tol=1.0) -> DeorbitPlan:
    """Find the deorbit ignition time (+ cross-range dv) so the predicted impact hits the aim point.

    r_bf, v_bf: kRPC body-frame position / surface-relative velocity at UT0 (orbit with Pe above the
    atmosphere). dv: retrograde dv (m/s); default lowers the vacuum periapsis to pe_alt.
    aim_bias / cross_bias: aim point offset from target (m; + = long / right of track).
    cross_mode: 'pc'     separate pure-normal burn ~270 deg of orbit before impact (efficient, default)
                'normal' normal component folded into the deorbit burn (cheap only for tiny errors)
                'none'   along-track only; residual cross-track reported.
    entry_burns: extra burns (absolute times from UT0, e.g. an 'air_retro' entry burn) in the model.
    Method: 1) scan ignition time over n_orbits (scan_deg steps), 2) bracket along-track zero
    crossings and brentq each, pick min |cross| + wait_cost * t_ign, 3) damped 2-D Newton on
    (t_ign, dv_normal) -> (along, cross) = 0, heading frozen then refreshed once.
    """
    P = _Problem(r_bf, v_bf, vehicle, target, target_alt, aim_bias, cross_bias, body, entry_burns)
    if dv is None:
        dv = _retro_dv_for_periapsis(P.r0, P.v0, pe_alt, body)
    coarse = dict(rtol=1e-7, atol_r=1.0, atol_v=1e-3)
    ts = np.arange(t_min, t_min + n_orbits * P.period, P.period * scan_deg / 360.0)
    A = np.full(len(ts), np.nan)
    Dist = np.full(len(ts), np.nan)         # impact-to-target distance (closest-approach fallback)
    for i, t in enumerate(ts):
        imp = P.run(t, dv, 0.0, **coarse)
        if imp.hit:
            A[i] = P.errors(imp)[0]
            Dist[i] = E.great_circle(imp.lat, imp.lon, target[0], target[1], body.radius)
    cands = []
    for i in range(len(ts) - 1):
        if A[i] < 0.0 <= A[i + 1] and A[i + 1] - A[i] < math.pi * body.radius / 2:
            try:
                t_root = brentq(lambda t: P.errors(P.run(t, dv, 0.0, **coarse))[0], ts[i], ts[i + 1], xtol=1e-2)
            except ValueError:
                continue
            imp = P.run(t_root, dv, 0.0)
            a, c, h = P.errors(imp)
            t_pc = t_root + imp.t - 0.75 * P.period
            if cross_mode == "pc" and t_pc < t_min:
                continue
            cands.append((abs(c) + wait_cost * t_root, t_root, c, h, t_pc))
    box = None                              # Newton trust region (closest-approach candidates only)
    if not cands:
        # No along-track sign change: every pass in the window misses the target by more than ~half the
        # 40 km-to-impact ground distance (the per-sample heading then points across the track, see
        # _parallel_heading). Bracket each pass by its closest approach instead, measured in a heading frozen
        # parallel to that pass, and let the Newton below put the plane change on it (C2, 2026-10-21).
        cands = _closest_approach_candidates(P, ts, Dist, dv, coarse, t_min, cross_mode, wait_cost)
        box = (0.25 * P.period, 1500.0)
    if not cands:
        raise RuntimeError("no impact passes the target within the scan window (dv too small / window too short?)")
    _, t_ign, c0, P.heading, t_pc = min(cands)
    dvn = 0.0
    if cross_mode == "pc":
        P.t_pc = t_pc
    if cross_mode in ("pc", "normal"):
        t_ign, dvn = _newton2(P, t_ign, dv, dvn, tol, box=box)
        h_new = E.track_heading(P.run(t_ign, dv, dvn), target)
        if abs((h_new - P.heading + 180) % 360 - 180) > 0.2:      # refresh frozen heading once
            P.heading = h_new
            t_ign, dvn = _newton2(P, t_ign, dv, dvn, tol, box=box)
    else:
        t_ign = brentq(lambda t: P.errors(P.run(t, dv, 0.0))[0], t_ign - 2, t_ign + 2, xtol=1e-4)
    return _finish(P, t_ign, dv, dvn, [(t, cc) for _, t, cc, _, _ in cands])


def _finish(P: _Problem, t_ign, dv, dvn, cands):
    """DeorbitPlan for the burn(s) (t_ign, dv retro, dvn normal) of problem P."""
    vehicle, body = P.veh, P.body
    imp = P.run(t_ign, dv, dvn)
    a, c, h = P.errors(imp)
    burns = P.burns(t_ign, dv, dvn)
    # node times (half-dv point) and frozen directions, mass carried through the burn sequence
    m, nodes, ve = vehicle.mass, [], vehicle.isp * E.G0
    for b in burns:
        nodes.append((b.t_start + m / b.mdot * (1 - math.exp(-b.dv / (2 * ve))),
                      *(b.dv * np.array(b.components) / np.linalg.norm(b.components))))
        m_ign, m = m, m * math.exp(-b.dv / ve)
    bd = burns[-1]
    r_ig, v_ig = P.state_at(t_ign) if P.t_pc is None or dvn == 0.0 else _state_after(P, t_ign, dv, dvn)
    d_i = np.array(E._orbital_dir(tuple(r_ig) + tuple(v_ig), bd.components))
    t_first = burns[0].t_start
    return DeorbitPlan(t_ign, nodes[-1][0], bd.burn_time(m_ign), dv, dvn, P.t_pc if dvn != 0.0 else None,
                       burns + P.entry(t_ign), [tuple(map(float, n)) for n in nodes], imp, t_first + imp.t,
                       a, c, h, E.rot_y(d_i, -body.omega * t_ign), cands)


def retro_plan(r_bf, v_bf, vehicle: Vehicle, target=KSC_PAD, target_alt=KSC_PAD_ALT, pe_alt=-60e3, t_ign=30.0,
               entry_burns: Sequence[Burn] = (), body: Body = KERBIN) -> DeorbitPlan:
    """Last resort (never strand the core): a pure retrograde burn t_ign s from the input state that lowers the
    periapsis to pe_alt. No targeting - the core comes down wherever this pass takes it."""
    P = _Problem(r_bf, v_bf, vehicle, target, target_alt, 0.0, 0.0, body, entry_burns, check_pe=False)
    dv = max(_retro_dv_for_periapsis(P.r0, P.v0, pe_alt, body), 1.0)
    return _finish(P, t_ign, dv, 0.0, [])


def propagate_bf(r_bf, v_bf, dt, body: Body = KERBIN):
    """Vacuum (Kepler) propagation of a kRPC body-frame state by dt seconds (body rotation included)."""
    r_i, v_i = E.bf_to_inertial(r_bf, v_bf, 0.0, body)
    r, v = E.kepler_propagate(r_i, v_i, dt, body.mu)
    return E.inertial_to_bf(r, v, dt, body)


def _parallel_heading(imp: Impact, target, ref_alt=40e3):
    """Heading (deg) at the target of the circle parallel to this trajectory's ground track (the great circle
    through the ref_alt point and the impact, moved onto the target). Along-track error in this frame is zero
    at the pass's closest approach however far the pass is (E.track_heading points from the ref_alt point to
    the target, so it swings across the track when the pass is far away and along never changes sign)."""
    t, rb, vb, lat, lon, alt = imp.history_bf()
    i = int(np.argmax(alt <= ref_alt)) if np.any(alt <= ref_alt) else 0
    p1 = rb[i] / np.linalg.norm(rb[i])
    pi_ = np.asarray(imp.r_bf, float) / np.linalg.norm(imp.r_bf)
    nrm = np.cross(p1, pi_)                       # track pole; ground motion at x is along nrm x x
    if np.linalg.norm(nrm) < 1e-6:
        return E.track_heading(imp, target)
    e, n, u = E.enu_basis(*target)
    h = np.cross(nrm, u)
    return math.degrees(math.atan2(np.dot(h, e), np.dot(h, n)))


def _closest_approach_candidates(P, ts, Dist, dv, coarse, t_min, cross_mode, wait_cost, max_dist=400e3):
    """Candidates at the local minima of impact-to-target distance over the scan (one per pass). Passes more than
    max_dist away are skipped: they would need > ~1.3 km/s of plane change (and the Newton may not converge)."""
    out = []
    for i in range(1, len(ts) - 1):
        d0, d1, d2 = Dist[i - 1], Dist[i], Dist[i + 1]
        if not (np.isfinite(d0) and np.isfinite(d1) and np.isfinite(d2)) or not (d1 <= d0 and d1 < d2):
            continue
        if d1 > max_dist:
            continue
        h = _parallel_heading(P.run(ts[i], dv, 0.0, **coarse), P.target)
        f = lambda t: P.errors(P.run(t, dv, 0.0, **coarse), heading=h)[0]
        try:
            t_root = brentq(f, ts[i - 1], ts[i + 1], xtol=1e-2)
        except ValueError:                        # no bracket: keep the sampled minimum
            t_root = ts[i]
        imp = P.run(t_root, dv, 0.0)
        if not imp.hit:
            continue
        a, c, _ = P.errors(imp, heading=h)
        t_pc = t_root + imp.t - 0.75 * P.period
        if cross_mode == "pc" and t_pc < t_min:
            continue
        out.append((abs(c) + wait_cost * t_root, t_root, c, h, t_pc))
    return out


def _state_after(P, t_ign, dvr, dvn):
    """Inertial state at deorbit ignition after the plane-change burn (vacuum)."""
    r, v = P.state_at(P.t_pc)
    pc = P.burns(t_ign, dvr, dvn)[0]
    hit, t, y, _, _ = E.simulate(np.r_[r, v, P.veh.mass], P.t_pc, None, [pc], P.body, -1e9,
                                 t_max=t_ign - P.t_pc, record=False)
    return E.kepler_propagate(y[0:3], y[3:6], t_ign - t, P.body.mu)


def _newton2(P: _Problem, t, dvr, dvn, tol, it=12, box=None):
    """Damped Newton with finite-difference Jacobian on (t_ign, dv_normal) -> (along, cross).
    box: optional (max |t_ign - t|, max |dv_normal|) trust region; a step that cannot stay inside raises
    RuntimeError (far-pass candidates: unbounded iterates reach escape orbits that take minutes to simulate)."""
    def F(x):
        a, c, _ = P.errors(P.run(x[0], dvr, x[1]))
        return np.array([a, c])
    x = np.array([t, dvn], float)
    f = F(x)
    steps = np.array([0.5, 0.2])
    for _ in range(it):
        if np.hypot(*f) < tol:
            break
        J = np.column_stack([(F(x + np.eye(2)[k] * steps[k]) - f) / steps[k] for k in range(2)])
        dx = -np.linalg.solve(J, f)
        lam = 1.0
        while lam > 1e-3:
            xn = x + lam * dx
            if box is not None and (abs(xn[0] - t) > box[0] or abs(xn[1]) > box[1]):
                lam *= 0.5
                if lam <= 1e-3:
                    raise RuntimeError("deorbit Newton left its trust region (plane change > %.0f m/s)" % box[1])
                continue
            fn = F(xn)
            if np.hypot(*fn) < np.hypot(*f):
                break
            lam *= 0.5
        x, f = xn, fn
    return x[0], x[1]


# --------------------------------------------------------------------------- window selection (C1)
PLAN_ERRORS = (RuntimeError, ValueError, ArithmeticError, np.linalg.LinAlgError)


@dataclass
class WindowChoice:
    plan: Optional[DeorbitPlan]  # None only with status 'none'
    t_base: float                # s after the survey epoch of the state the plan was made from (its time origin)
    k: int                       # revolution index of that state (t_base = k * period)
    status: str                  # 'ok' | 'no_accept' | 'warn_dvn' | 'low_power' | 'over_budget' | 'none'
    tried: list = field(default_factory=list)   # [(k, dv_normal or None, note)]


def deorbit_budget(plan_dv, mass, dry_mass, isp, reserve_dv, margin=1.08):
    """(ok, dv available, dv needed): can the core fly this deorbit and still keep reserve_dv (entry + landing
    burns)? Needed = margin * (plan_dv + reserve_dv), as kspilot.feasibility.recovery_dv budgets it."""
    avail = isp * E.G0 * math.log(mass / dry_mass) if mass > dry_mass > 0 else 0.0
    need = margin * (plan_dv + reserve_dv)
    return avail >= need, avail, need


def choose_window(plan_at: Callable, period: float, max_dvn: float = 30.0, n_revs: int = 12,
                  accept: Optional[Callable] = None, budget_ok: Optional[Callable] = None, max_miss: float = 5e3,
                  log: Callable = print, power_ok: Optional[Callable] = None) -> WindowChoice:
    """Survey deorbit windows one revolution apart and pick one. Never raises.

    plan_at(t_base) -> DeorbitPlan made from the state t_base s after the survey epoch (raises when no pass
    qualifies). Revolution k uses t_base = k * period (k = 0 .. n_revs-1); plan_at is called lazily.
    1) The FIRST plan with |dv_normal| <= max_dvn that accept(plan, t_base) passes (e.g. daylight) is returned at
       once: status 'ok' (the pre-C1 rule: least waiting; the baseline flies exactly what it flew before).
    2) Otherwise, after n_revs revolutions: the earliest plan within max_dvn that `accept` rejected
       ('no_accept': what accept=None would have flown),
    3) else the plan with the lowest |dv_normal| seen: 'warn_dvn' if budget_ok(plan) (default: True), else
       'over_budget' (the caller flies a cheaper fallback; never strands the core),
    4) else status 'none' (no plan at all).
    A plan whose predicted miss exceeds max_miss (Newton did not converge) counts as no plan.
    power_ok(plan, t_base) -> bool (electric power left at touchdown, kspilot.power): a window that fails it is
    not eligible, and the survey stops there (every later window waits longer). If no eligible window exists the
    soonest plan that failed it stands in for 'best' in rule 3 (least waiting = least drain; never strands the core).
    """
    best, first_ok, tried, low_power = None, None, [], None
    for k in range(int(n_revs)):
        tb = k * period
        try:
            plan = plan_at(tb)
        except PLAN_ERRORS as e:
            tried.append((k, None, str(e)[:60]))
            log("deorbit window rev %d: no plan (%s)" % (k, str(e)[:60]))
            continue
        if not plan.impact.hit or not math.isfinite(plan.miss) or plan.miss > max_miss:
            tried.append((k, plan.dv_normal, "miss %.0f m" % plan.miss))
            log("deorbit window rev %d: plan did not converge (miss %.0f m)" % (k, plan.miss))
            continue
        dvn = abs(plan.dv_normal)
        if power_ok is not None and not power_ok(plan, tb):
            tried.append((k, plan.dv_normal, "EC"))
            log("deorbit window rev %d: rejected - too little electric power left at touchdown; survey stops" % k)
            low_power = (plan, tb, k)
            break
        if dvn <= max_dvn:
            if accept is None or accept(plan, tb):
                tried.append((k, plan.dv_normal, "ok"))
                return WindowChoice(plan, tb, k, "ok", tried)
            tried.append((k, plan.dv_normal, "not accepted"))
            if first_ok is None:
                first_ok = (plan, tb, k)
        else:
            tried.append((k, plan.dv_normal, "normal > %.0f" % max_dvn))
            log("deorbit window rev %d: needs %.0f m/s normal (> %.0f)" % (k, plan.dv_normal, max_dvn))
        if best is None or dvn < abs(best[0].dv_normal):
            best = (plan, tb, k)
    if first_ok is not None:
        log("deorbit: no accepted window in %d revolutions; flying the first one within %.0f m/s normal (rev %d)" % (
            n_revs, max_dvn, first_ok[2]))
        return WindowChoice(*first_ok, "no_accept", tried)
    if best is None and low_power is not None:
        plan, tb, k = low_power
        if budget_ok is None or budget_ok(plan):
            log("deorbit WARNING: no window keeps the electric-power reserve; flying the soonest (rev %d, %.0f m/s "
                "normal, total %.0f m/s)" % (k, plan.dv_normal, plan.dv))
            return WindowChoice(plan, tb, k, "low_power", tried)
        best = low_power
    if best is not None:
        plan, tb, k = best
        if budget_ok is None or budget_ok(plan):
            log("deorbit WARNING: no window within %.0f m/s normal in %d revolutions; flying the cheapest "
                "(rev %d, %.0f m/s normal, total %.0f m/s)" % (max_dvn, n_revs, k, plan.dv_normal, plan.dv))
            return WindowChoice(plan, tb, k, "warn_dvn", tried)
        log("deorbit ERROR: cheapest window (rev %d, %.0f m/s normal, total %.0f m/s) exceeds the fuel budget" % (
            k, plan.dv_normal, plan.dv))
        return WindowChoice(plan, tb, k, "over_budget", tried)
    log("deorbit ERROR: no deorbit plan at all in %d revolutions" % n_revs)
    return WindowChoice(None, 0.0, 0, "none", tried)


# --------------------------------------------------------------------------- in-flight correction
@dataclass
class Correction:
    along: float                 # m (+ long), after aim bias
    cross: float                 # m (+ right of track)
    heading: float
    impact: Impact
    jac: np.ndarray              # d(along, cross)/d(dv) [2x3] per m/s along `basis` rows
    basis: np.ndarray            # 3x3 rows: retro (-airspeed), "up" (perp. to airspeed), side
    dv_bf: np.ndarray            # min-norm dv (m/s, body axes) that nulls along & cross (linearised)
    direction: np.ndarray        # unit vector of dv_bf (thrust direction to correct)
    lateral: np.ndarray          # component of dv_bf perpendicular to airspeed (aero-steering cue)

    @property
    def miss(self):
        return math.hypot(self.along, self.cross)


def impact_correction(r_bf, v_bf, mass, drag=None, cda=None, target=KSC_PAD, target_alt=KSC_PAD_ALT,
                      heading=None, aim_bias=0.0, cross_bias=0.0, burns: Sequence[Burn] = (),
                      body: Body = KERBIN, probe=2.0) -> Correction:
    """Predict impact from the current body-frame state and the dv needed to put it on the aim point.

    heading: track heading to measure along/cross in (pass DeorbitPlan.heading for consistency);
    None -> estimated from the trajectory. 4 predictions (nominal + 3 probes of `probe` m/s).
    """
    if drag is None and cda is not None:
        drag = E.ballistic(cda, body)
    r_bf, v_bf = np.asarray(r_bf, float), np.asarray(v_bf, float)
    kw = dict(drag=drag, burns=burns, body=body, target_alt=target_alt)
    imp = E.predict(r_bf, v_bf, mass, **kw)
    h = heading if heading is not None else E.track_heading(imp, target)
    err = lambda im: np.array(E.ground_errors(im.lat, im.lon, target, h, body.radius)) - (aim_bias, cross_bias)
    f0 = err(imp)
    u = v_bf / (np.linalg.norm(v_bf) or 1.0)
    up = r_bf / np.linalg.norm(r_bf)
    e2 = up - np.dot(up, u) * u
    if np.linalg.norm(e2) < 1e-3:                       # near-vertical: use north as reference
        e2 = np.array([0.0, 1.0, 0.0]) - u[1] * u
    e2 /= np.linalg.norm(e2)
    basis = np.array([-u, e2, np.cross(u, e2)])
    J = np.column_stack([(err(E.predict(r_bf, v_bf + probe * b, mass, **kw)) - f0) / probe for b in basis])
    dv_coef = -np.linalg.pinv(J) @ f0
    dv = basis.T @ dv_coef
    n = np.linalg.norm(dv)
    lat = dv - np.dot(dv, u) * u
    return Correction(f0[0], f0[1], h, imp, J, basis, dv, dv / n if n > 0 else dv, lat)
