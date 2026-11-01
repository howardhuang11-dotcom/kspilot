"""Terminal powered-descent guidance (G-FOLD style) and a hoverslam fallback law.

Frame: local flat ENU frame rotating with Kerbin, origin at the landing target (pad surface point,
or the point where the vehicle's reference point should be at touchdown): x = East, y = North,
z = Up. r in m, v in m/s relative to the surface, thrust in N, g along -z.

Convex guidance (G-FOLD family):
- lossless convexification of the thrust bounds/pointing with the log-mass change of variables
  (Acikmese & Ploen 2007), glideslope and sink-rate cones;
- minimum landing error and minimum fuel combined as exact penalties (Blackmore et al. 2010),
  plus a soft throttle ceiling so closed-loop re-plans keep a thrust reserve instead of riding
  the hard limit (min-fuel plans are bang-bang and otherwise lose feasibility on any deficit);
- quadratic drag, Coriolis and the gravity gradient linearised in the state about the previous
  solution (successive convexification, Szmuk & Acikmese);
- free final time by grid + golden-section search.
Problems are DPP-parametrised and cached, so a re-solve only updates parameter values.
"""
import functools
import math
import time
import warnings
from dataclasses import dataclass
from types import SimpleNamespace

import cvxpy as cp
import numpy as np

from . import atmo

G0 = atmo.G0
PAD_ALT = atmo.KERBIN["ksc_pad"]["surface_height"]  # m above sea level, for the density lookup
_SOLVER_OPTS = {"CLARABEL": dict(tol_gap_abs=1e-6, tol_gap_rel=1e-6, tol_feas=1e-6)}


@dataclass(frozen=True)
class PDGConfig:
    t_max: float                    # max thrust, N
    isp: float                      # s
    m_dry: float                    # mass floor (dry + reserve), kg
    throttle: tuple = (0.3, 1.0)    # hard throttle band (fractions of t_max)
    throttle_soft: float = 0.9      # soft ceiling: thrust above it is penalised (closed-loop margin)
    tilt_max_deg: float = 20.0      # max thrust angle from vertical
    glideslope_deg: float = 20.0    # min elevation of the vehicle seen from the target
    v_td: float = 1.5               # touchdown descent rate (> 0, downward)
    k_sink: float = 1.0             # sink-rate cone near the ground: -v_z <= v_td + k * h, with
                                    # k >= k_sink raised to what the min-throttle decel. needs (1/s)
    g: float = 9.81                 # m/s^2 (use effective gravity incl. centrifugal if desired)
    cda: float = 0.0                # Cd*A drag area, m^2 (0 disables drag)
    pad_alt: float = PAD_ALT        # target altitude above sea level, m
    omega_enu: tuple | None = None
    a_dist: tuple | None = None     # measured unmodelled acceleration (ENU, m/s^2), held constant  # planet rotation vector in ENU (rad/s) for Coriolis, or None
    n: int = 40                     # discretisation intervals
    w_miss: float = 1.0             # exact-penalty weights on landing position / touchdown velocity
    w_vel: float = 1.0              #   error (>> fuel duals, so exact when the target is reachable)
    w_soft: float = 0.05            # penalty per m/s of delta-v above the soft ceiling (fuel ~3e-4)
    solver: str = "CLARABEL"


@dataclass
class Plan:
    status: str
    tf: float = math.nan
    t: np.ndarray = None        # (N+1,)
    r: np.ndarray = None        # (N+1, 3)
    v: np.ndarray = None        # (N+1, 3)
    m: np.ndarray = None        # (N+1,)
    u: np.ndarray = None        # (N+1, 3) thrust acceleration T/m
    ab: np.ndarray = None       # (N+1, 3) modelled non-thrust acceleration (gravity, drag, ...)
    miss: float = math.nan      # final horizontal landing error, m
    v_err: float = math.nan     # touchdown velocity error, m/s
    cost: float = math.inf
    solve_time: float = 0.0     # wall time of the whole call, s
    n_solves: int = 0

    @property
    def ok(self):
        return self.status in ("optimal", "optimal_inaccurate")

    @property
    def exact(self):
        """True if the plan meets the landing target and touchdown velocity."""
        return self.ok and self.miss < 0.1 and self.v_err < 0.05

    @property
    def thrust(self):
        return self.u * self.m[:, None]

    @property
    def fuel(self):
        return float(self.m[0] - self.m[-1])

    def at(self, t):
        """Planned state (r, v, u) at time t since plan start (first-order hold between nodes)."""
        t = float(np.clip(t, 0.0, self.tf))
        dt = self.tf / (len(self.t) - 1)
        k = min(int(t / dt), len(self.t) - 2)
        s = t - k * dt
        a0 = self.u[k] + self.ab[k]
        da = (self.u[k + 1] + self.ab[k + 1] - a0) / dt
        v = self.v[k] + a0 * s + da * s * s / 2
        r = self.r[k] + self.v[k] * s + a0 * s * s / 2 + da * s ** 3 / 6
        return r, v, self.u[k] + (self.u[k + 1] - self.u[k]) * s / dt


# ----------------------------------------------------------------------------- convex problem

@functools.lru_cache(maxsize=8)
def _build(n, cos_tilt, tan_gs, w_miss, w_vel, w_soft, upright=True):
    N = n
    r, v, u, ad = (cp.Variable((3, N + 1)) for _ in range(4))  # ad: linearised external accel.
    z, s, q = cp.Variable(N + 1), cp.Variable(N + 1), cp.Variable(N + 1, nonneg=True)
    P = SimpleNamespace(
        r0=cp.Parameter(3), v0=cp.Parameter(3), zi=cp.Parameter(), vf=cp.Parameter(3), zmin=cp.Parameter(),
        ks=cp.Parameter(nonneg=True),
        dt=cp.Parameter(nonneg=True), dth=cp.Parameter(nonneg=True), dt26=cp.Parameter(nonneg=True),
        adth=cp.Parameter(nonneg=True), gdv=cp.Parameter(3), gdr=cp.Parameter(3),
        J=[[cp.Parameter(N + 1) for _ in range(3)] for _ in range(3)],  # d a_ext / d v per node
        E=cp.Parameter((3, N + 1)), D=cp.Parameter((3, N + 1)),          # d a_ext / d h, offset
        z0=cp.Parameter(N + 1), zmax=cp.Parameter(N + 1),
        a1=cp.Parameter(N + 1, nonneg=True), a1z0=cp.Parameter(N + 1),
        sq1=cp.Parameter(N + 1, nonneg=True), sq1z0=cp.Parameter(N + 1),
        a2=cp.Parameter(N + 1, nonneg=True), a2z0=cp.Parameter(N + 1),
        a3=cp.Parameter(N + 1, nonneg=True), a3z0=cp.Parameter(N + 1))
    ones = np.ones((1, N))
    gdv, gdr = (cp.reshape(x, (3, 1), order="F") @ ones for x in (P.gdv, P.gdr))  # gravity terms
    acc = u + ad  # thrust + linearised external (drag, Coriolis, gravity gradient) acceleration
    con = [  # first-order-hold dynamics (exact for piecewise-linear acceleration)
        v[:, 1:] == v[:, :-1] + P.dth * (acc[:, :-1] + acc[:, 1:]) + gdv,
        r[:, 1:] == r[:, :-1] + P.dt * v[:, :-1] + P.dt26 * (2 * acc[:, :-1] + acc[:, 1:]) + gdr,
        z[1:] == z[:-1] - P.adth * (s[:-1] + s[1:]),
        r[:, 0] == P.r0, v[:, 0] == P.v0, z[0] == P.zi,
        r[2, N] == 0,
        cp.norm(u, 2, axis=0) <= s,                                   # lossless convexification
        u[2, :] >= cos_tilt * s,                                      # thrust pointing (tilt)
        tan_gs * cp.norm(r[:2, 1:N], 2, axis=0) <= r[2, 1:N],         # glideslope cone
        v[2, 1:] + P.ks * r[2, 1:] >= P.vf[2],                         # gentle final approach
        # thrust bounds rho1/m <= sigma <= rho2/m, second/first-order expansion about z0(t)
        P.a1 - (cp.multiply(P.a1, z) - P.a1z0) + 0.5 * cp.square(cp.multiply(P.sq1, z) - P.sq1z0) <= s,
        s <= P.a2 - (cp.multiply(P.a2, z) - P.a2z0),
        q >= s - (P.a3 - (cp.multiply(P.a3, z) - P.a3z0)),               # excess over soft ceiling
        z >= P.z0, z <= P.zmax, z >= P.zmin,
    ]
    con += [ad[i, :] == sum(cp.multiply(P.J[i][j], v[j, :]) for j in range(3))
            + cp.multiply(P.E[i, :], r[2, :]) + P.D[i, :] for i in range(3)]
    if upright:
        con.append(u[:2, N] == 0)
    cost = -z[N] + w_miss * cp.norm(r[:2, N]) + w_vel * cp.norm(v[:, N] - P.vf) + w_soft * P.dt * cp.sum(q)
    prob = cp.Problem(cp.Minimize(cost), con)
    return prob, P, SimpleNamespace(r=r, v=v, u=u, z=z, s=s, ad=ad)


def _linearize(cfg, r, v, m):
    """First-order model a_ext ~= J v + E h + D (per node) of drag, Coriolis and the gravity
    gradient about a reference trajectory. Tangent models of |v| v and rho(h) underestimate the
    drag, so the plan is conservative. Returns J (N+1, 3, 3), E (N+1, 3), D (N+1, 3)."""
    n1 = len(v)
    J, E, D = np.zeros((n1, 3, 3)), np.zeros((n1, 3)), np.zeros((n1, 3))
    rp = atmo.R + cfg.pad_alt
    E[:, 2] = 2 * cfg.g / rp  # gravity gradient d|g|/dh (exact to O(h/R)^2)
    if cfg.cda > 0:
        h = np.maximum(r[:, 2], 0.0)
        rho = atmo.density(cfg.pad_alt + h)
        lo = np.maximum(h - 50.0, 0.0)
        drho = (atmo.density(cfg.pad_alt + h + 50.0) - atmo.density(cfg.pad_alt + lo)) / (h + 50.0 - lo)
        sp = np.maximum(np.linalg.norm(v, axis=1), 1e-3)
        k = 0.5 * cfg.cda / m
        a_d = -(k * rho * sp)[:, None] * v
        vv = v[:, :, None] * v[:, None, :] / sp[:, None, None]
        J -= (k * rho)[:, None, None] * (sp[:, None, None] * np.eye(3) + vv)  # d(-k rho |v| v)/dv
        Ed = -(k * drho * sp)[:, None] * v
        E += Ed
        D += a_d - np.einsum("kij,kj->ki", J, v) - Ed * h[:, None]
    if cfg.a_dist is not None:
        D += np.asarray(cfg.a_dist, float)
    if cfg.omega_enu is not None:
        w = np.asarray(cfg.omega_enu, float)
        J += -2 * np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    return J, E, D


def solve_fixed_tf(r0, v0, m0, tf, cfg, lin=None):
    """Solve the convexified landing problem for a fixed final time.
    lin: (J, E, D) from _linearize, or None (no drag/Coriolis/gravity-gradient terms)."""
    N = cfg.n
    prob, P, X = _build(N, math.cos(math.radians(cfg.tilt_max_deg)),
                        math.tan(math.radians(cfg.glideslope_deg)), cfg.w_miss, cfg.w_vel, cfg.w_soft)
    dt = tf / N
    t = np.arange(N + 1) * dt
    J, E, D = lin if lin is not None else (np.zeros((N + 1, 3, 3)), np.zeros((N + 1, 3)), np.zeros((N + 1, 3)))
    g = np.array([0.0, 0.0, -cfg.g])
    alpha = 1.0 / (cfg.isp * G0)
    rho1, rho2, rho3 = (f * cfg.t_max for f in (*cfg.throttle, cfg.throttle_soft))
    z0 = np.log(np.maximum(m0 - alpha * rho2 * t, cfg.m_dry))  # mass after a full-throttle burn
    a1, a2, a3 = rho1 * np.exp(-z0), rho2 * np.exp(-z0), rho3 * np.exp(-z0)
    sq1 = np.sqrt(a1)
    a_min = rho1 / cfg.m_dry - cfg.g  # strongest unavoidable deceleration at min throttle
    vals = dict(r0=r0, v0=v0, zi=math.log(m0), vf=[0.0, 0.0, -cfg.v_td], zmin=math.log(cfg.m_dry),
                ks=max(cfg.k_sink, 1.5 * a_min / cfg.v_td),
                dt=dt, dth=dt / 2, dt26=dt * dt / 6, adth=alpha * dt / 2, gdv=g * dt, gdr=g * dt * dt / 2,
                E=E.T, D=D.T, z0=z0, zmax=np.log(np.maximum(m0 - alpha * rho1 * t, 1e-3 * cfg.m_dry)),
                a1=a1, a1z0=a1 * z0, sq1=sq1, sq1z0=sq1 * z0, a2=a2, a2z0=a2 * z0, a3=a3, a3z0=a3 * z0)
    for k, val in vals.items():
        getattr(P, k).value = np.asarray(val, float) if np.ndim(val) else float(val)
    for i in range(3):
        for j in range(3):
            P.J[i][j].value = J[:, i, j]
    try:
        with warnings.catch_warnings():  # "optimal_inaccurate" is reported through the status
            warnings.simplefilter("ignore", UserWarning)
            prob.solve(solver=cfg.solver, **_SOLVER_OPTS.get(cfg.solver, {}))
        status = prob.status
    except cp.SolverError:
        status = "solver_error"
    plan = Plan(status=status, tf=tf, n_solves=1)
    if plan.ok:
        plan.t, plan.r, plan.v, plan.u = t, X.r.value.T, X.v.value.T, X.u.value.T
        plan.m, plan.ab = np.exp(X.z.value), X.ad.value.T + g
        plan.miss = float(np.linalg.norm(plan.r[-1, :2]))
        plan.v_err = float(np.linalg.norm(plan.v[-1] - vals["vf"]))
        plan.cost = float(prob.value)
    return plan


def _reference(r0, v0, m0, cfg, ref, t_shift, n):
    """Reference states at normalised time tau in [0, 1] for re-linearising drag/Coriolis."""
    tau = np.linspace(0.0, 1.0, n + 1)
    if ref is not None and ref.ok and ref.tf - t_shift > 0.5:
        ts = t_shift + tau * (ref.tf - t_shift)
        st = [ref.at(x) for x in ts]
        r = np.array([x[0] for x in st])
        v = np.array([x[1] for x in st])
        m = np.interp(ts, ref.t, ref.m) * m0 / np.interp(t_shift, ref.t, ref.m)
        return r, v, m
    vf = np.array([0.0, 0.0, -cfg.v_td])
    r = np.outer(1 - tau, r0)
    v = np.outer(1 - tau, v0) + np.outer(tau, vf)
    return r, v, np.full(n + 1, m0)


def _golden(f, a, b, tol):
    """Golden-section minimisation of f on [a, b] down to bracket width tol."""
    gr = (math.sqrt(5) - 1) / 2
    c, d = b - gr * (b - a), a + gr * (b - a)
    fc, fd = f(c), f(d)
    while b - a > tol:
        if fc <= fd:
            b, d, fd = d, c, fc
            c = b - gr * (b - a)
            fc = f(c)
        else:
            a, c, fc = c, d, fd
            d = a + gr * (b - a)
            fd = f(d)


def solve(r0, v0, m0, cfg, tf_guess=None, ref=None, t_shift=0.0, iters=None, tol=0.2):
    """Free-final-time minimum-fuel / minimum-landing-error descent.

    r0, v0, m0: initial state in the pad ENU frame. tf_guess: centre of a narrow +-12% search
    bracket (closed-loop warm start); otherwise a coarse grid over a heuristic bracket is used.
    ref/t_shift: previous Plan and time elapsed since it was made, used to re-linearise the
    external accelerations (gravity gradient, drag, Coriolis).
    iters: re-linearisation passes (default 1 with a usable ref, else 3).
    Landing position and touchdown velocity are exact penalties and thrust above
    cfg.throttle_soft is penalised, so re-plans degrade smoothly instead of becoming infeasible
    when the vehicle falls slightly behind a minimum-fuel (bang-bang) plan. plan.exact tells
    whether the target is met; plan.ok is False only if the solver found nothing feasible.
    """
    t_start = time.perf_counter()
    r0, v0 = np.asarray(r0, float), np.asarray(v0, float)
    if iters is None:
        iters = 1 if (ref is not None and ref.ok) else 3
    if tf_guess is None and ref is not None and ref.ok:
        tf_guess = ref.tf - t_shift
    alpha = 1.0 / (cfg.isp * G0)
    t_fuel = (m0 - cfg.m_dry) / (alpha * cfg.throttle[0] * cfg.t_max) if cfg.throttle[0] > 0 else math.inf
    n_solves, best = 0, Plan(status="infeasible")
    for it in range(iters):
        lin = _linearize(cfg, *_reference(r0, v0, m0, cfg, ref, t_shift, cfg.n))
        cache = {}

        def f(tf):
            nonlocal n_solves
            if tf not in cache:
                n_solves += 1
                cache[tf] = solve_fixed_tf(r0, v0, m0, tf, cfg, lin)
            p = cache[tf]
            return p.cost if p.ok else 1e6 - tf  # infeasible: slope towards longer tf

        last = it == iters - 1
        tol_it = tol if last else 4 * tol
        if tf_guess is not None:
            if it == 0 or last:
                w = 0.12 if it == 0 else 0.05
                _golden(f, (1 - w) * tf_guess, min((1 + w) * tf_guess, t_fuel), tol_it)
            else:  # intermediate pass: only re-linearise about the current best
                f(tf_guess)
        if not any(p.ok for p in cache.values()):  # cold start, or the warm bracket failed
            a_max = cfg.throttle_soft * cfg.t_max / m0
            lo = max(np.linalg.norm(v0) / a_max, math.sqrt(max(r0[2], 0.0) / a_max), 2.0)
            grid = np.linspace(lo, min(4.0 * lo, t_fuel), 6)
            i = int(np.argmin([f(x) for x in grid]))
            _golden(f, grid[max(i - 1, 0)], grid[min(i + 1, len(grid) - 1)], tol_it)
        feas = [p for p in cache.values() if p.ok]
        if not feas:
            break
        best = min(feas, key=lambda p: p.cost)
        ref, t_shift, tf_guess = best, 0.0, best.tf
    best.n_solves, best.solve_time = n_solves, time.perf_counter() - t_start
    return best


def _limit(acc, m, t_max, tilt_max_deg, thr_min=0.0):
    """Clip a thrust-acceleration command to tilt/throttle limits (vertical priority).
    Returns (throttle, unit direction)."""
    a = np.array(acc, float)
    a[2] = max(a[2], 1e-3)
    ah = np.linalg.norm(a[:2])
    ah_max = a[2] * math.tan(math.radians(tilt_max_deg))
    if ah > ah_max:
        a[:2] *= ah_max / ah
    a_max = t_max / m
    if np.linalg.norm(a) > a_max:
        a[2] = min(a[2], a_max)
        ah = np.linalg.norm(a[:2])
        room = math.sqrt(max(a_max ** 2 - a[2] ** 2, 0.0))
        if ah > room:
            a[:2] *= room / ah
    n = np.linalg.norm(a)
    return float(np.clip(n / a_max, thr_min, 1.0)), a / n


def track(plan, t, r, v, m, cfg, kp=1.0, kd=2.0, t_lead=0.25):
    """Follow a plan: feedforward thrust (read t_lead ahead to cancel engine lag) plus PD feedback
    on the planned state. Returns (throttle, unit thrust direction)."""
    rp, vp, _ = plan.at(t)
    up = plan.at(t + t_lead)[2]
    acc = up + kp * (rp - np.asarray(r)) + kd * (vp - np.asarray(v))
    return _limit(acc, m, cfg.t_max, cfg.tilt_max_deg)


class ThrustEstimator:
    """Online estimate of eta = achieved / commanded thrust from measured acceleration. Low-pass,
    updated only after the command has been steady for t_settle (engine lag). Plan with t_max * eta."""

    def __init__(self, tau=1.0, bounds=(0.7, 1.3), steady=0.03, t_settle=0.4):
        self.eta, self.tau, self.bounds, self.steady, self.t_settle = 1.0, tau, bounds, steady, t_settle
        self._last, self._t_steady = None, 0.0

    def update(self, a_thrust, a_cmd, dt):
        """a_thrust: measured acceleration minus gravity and modelled drag over the last interval;
        a_cmd: thrust acceleration commanded for that interval (t_max * throttle * dir / m)."""
        a_cmd = np.asarray(a_cmd, float)
        c = float(a_cmd @ a_cmd)
        if self._last is not None and np.linalg.norm(a_cmd - self._last) < self.steady * math.sqrt(c):
            self._t_steady += dt
        else:
            self._t_steady = 0.0
        self._last = a_cmd
        if self._t_steady >= self.t_settle and c > 1.0:
            k = min(dt / self.tau, 1.0)
            self.eta = float(np.clip(self.eta + k * (np.dot(a_thrust, a_cmd) / c - self.eta), *self.bounds))
        return self.eta


# ----------------------------------------------------------------------------- hoverslam fallback

@dataclass(frozen=True)
class HoverslamParams:
    throttle: float = 0.87          # constant throttle assumed by the burn-now decision
    h_stop: float = 6.0             # altitude where the main burn should end (m above target)
    v_td: float = 1.5               # final descent rate below h_stop, m/s
    t_delay: float = 0.1            # command latency + one control period, s
    tau_engine: float = 0.15        # engine spool-up time constant, s
    tilt_reserve_deg: float = 10.0  # thrust cone reserved for horizontal control in the decision
    drag_frac: float = 0.8          # fraction of the modelled drag credited in the decision
    kv: float = 1.5                 # vertical speed-loop gain, 1/s
    a_min_frac: float = 0.25        # slowest descent profile, fraction of max net deceleration
    tilt_max_deg: float = 20.0
    tilt_final_deg: float = 5.0     # tilt limit below h_stop
    kp_h: float = 0.25              # horizontal position gain -> velocity command, 1/s
    kv_h: float = 0.8               # horizontal velocity gain -> acceleration, 1/s
    t_null: float = 2.0             # horizontal velocity must be nulled this long before h_stop, s
    lat_eff: float = 0.8            # usable fraction of the tilted thrust for the divert (ignition check)
    t_att: float = 0.0              # dead time before the stage can tilt after ignition (high q), s


def hoverslam_stop_height(h, vz, m, t_max, g, cda=0.0, isp=None, p=HoverslamParams(), n=80):
    """Height (above target) where a vertical burn commanded now at constant p.throttle brings the
    descent rate down to p.v_td. Integrates (RK2) the command delay p.t_delay, first-order engine
    spool-up p.tau_engine, thrust within the tilt_reserve cone, drag_frac * tabulated drag, mass
    flow (if isp is given) and the gravity gradient (g given at the pad). Returns -inf if the
    throttle cannot overcome gravity."""
    w = -vz
    f = p.throttle * t_max * math.cos(math.radians(p.tilt_reserve_deg))
    if f / m <= g:
        return -math.inf
    if w <= p.v_td:
        return h
    mdot = f / (isp * G0) if isp else 0.0
    k = p.drag_frac * 0.5 * cda
    dt = ((w - p.v_td) / (f / m - g) + p.t_delay + p.tau_engine) / n
    rp = atmo.R + PAD_ALT

    def acc(t, h, w, m):  # deceleration (positive up)
        s = t - p.t_delay
        thr = 0.0 if s <= 0 else (1.0 - math.exp(-s / p.tau_engine) if p.tau_engine > 0 else 1.0)
        return (thr * f + k * atmo.density(PAD_ALT + max(h, 0.0)) * w * w) / m - g * (rp / (rp + h)) ** 2

    t = 0.0
    for _ in range(4 * n):
        a1 = acc(t, h, w, m)
        wm, hm = w - a1 * dt / 2, h - w * dt / 2
        a = acc(t + dt / 2, hm, wm, m - mdot * dt / 2)
        if a > 0 and w - a * dt <= p.v_td:  # final partial step
            return h - (w + p.v_td) / 2 * (w - p.v_td) / a
        h, w, m, t = h - wm * dt, w - a * dt, m - (mdot * dt if t >= p.t_delay else 0.0), t + dt
    return h


def hoverslam_burn_now(h, vz, m, t_max, g, cda=0.0, isp=None, p=HoverslamParams(), r_h=None, v_h=None):
    """Ignition decision. h: height above target, vz: vertical speed (< 0 falling), cda: drag-area
    estimate (m^2). Lights when the constant-throttle stop height (hoverslam_stop_height, which
    includes command delay and spool-up) reaches p.h_stop. With r_h/v_h (horizontal position and
    velocity) it also lights early enough for the horizontal correction (bang-bang estimate at
    80 % of tilt_max). Returns (burn_now, margin_m)."""
    w = max(-vz, 0.0)
    margin = hoverslam_stop_height(h, vz, m, t_max, g, cda, isp, p) - p.h_stop
    if margin > 0 and r_h is not None and w > 1.0:
        a_h = p.lat_eff * p.throttle * t_max / m * math.sin(math.radians(p.tilt_max_deg))
        r_h, v_h = np.asarray(r_h, float), np.asarray(v_h, float)
        vh = np.linalg.norm(v_h)
        d_h = np.linalg.norm(r_h + v_h * vh / (2 * a_h))  # offset left after nulling v_h
        t_h = vh / a_h + 2 * math.sqrt(d_h / a_h) + p.t_null + p.t_delay + p.t_att
        if 2 * (h - p.h_stop) / (w + p.v_td) <= t_h:  # constant-decel burn from now just fits
            return True, margin
    return margin <= 0, margin


def hoverslam_command(r, v, m, t_max, g, drag=0.0, p=HoverslamParams()):
    """Throttle/attitude law once the burn is lit. Vertical: constant-deceleration stop at h_stop
    (floored by a minimum descent-rate profile), v_td speed hold below. Horizontal: PD-to-tilt as a
    position -> velocity -> acceleration cascade; the velocity command is capped so the remaining
    horizontal speed can always be nulled with the available tilt before h_stop (0.3 m/s below).
    r, v: pad-frame state; drag: |drag| N (acts along -v). Returns (throttle, unit thrust direction)."""
    r, v = np.asarray(r, float), np.asarray(v, float)
    h, vz = r[2], v[2]
    sp = np.linalg.norm(v)
    a_drag = -drag / m * v / sp if sp > 1e-6 else np.zeros(3)
    dh = h - p.h_stop
    if dh > 1.0:
        a_lo = p.a_min_frac * max(t_max / m - g, 0.5)
        v_lo = math.sqrt(p.v_td ** 2 + 2 * a_lo * dh)
        if -vz >= v_lo:
            az = g + (vz * vz - p.v_td ** 2) / (2 * dh)
        else:
            az = g + a_lo + p.kv * (-v_lo - vz)
        tilt = p.tilt_max_deg
        tgo = 2 * dh / max(-vz + p.v_td, 1.0)  # time to h_stop
    else:
        az = g + p.kv * (-p.v_td - vz)
        tilt = p.tilt_final_deg
        tgo = None
    ah_max = math.tan(math.radians(tilt)) * max(az - a_drag[2], 0.5 * g)
    d = np.linalg.norm(r[:2])
    v_cap = math.sqrt(1.6 * ah_max * d)  # can still brake within d
    v_cap = min(v_cap, 0.8 * ah_max * max(tgo - p.t_null, 0.0) if tgo is not None else 0.3)
    v_cmd = -r[:2] / d * min(p.kp_h * d, v_cap) if d > 1e-6 else np.zeros(2)
    ah = p.kv_h * (v_cmd - v[:2])
    acc = np.array([ah[0], ah[1], az]) - a_drag
    return _limit(acc, m, t_max, tilt)
