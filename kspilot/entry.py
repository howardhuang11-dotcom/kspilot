"""Atmospheric entry / impact predictor (offline, pure Python + numpy).

Frames
------
All vectors use kRPC axis conventions (left-handed, see CelestialBody.reference_frame):
    x -> (lat 0, lon 0),  y -> north pole (rotation axis),  z -> (lat 0, lon 90 deg E).
A body-fixed position is r = (R+h) * (cos(lat)cos(lon), sin(lat), cos(lat)sin(lon)), identical to
KSP's CelestialBody.GetRelSurfaceNVector and kRPC body.surface_position(...).

``predict`` takes a *body-fixed* state as returned by kRPC
    r = vessel.position(body.reference_frame), v = vessel.velocity(body.reference_frame)
(velocity is relative to the rotating surface). Internally it integrates in the inertial frame that
coincides with the body frame at t = 0; at time t the body has turned east by omega*t, so a
body-fixed vector is the inertial one rotated by -omega*t about y. The co-rotating air moves with
v_air(r) = omega x r, which in these axes is (-omega*z, 0, omega*x).

Integrator: adaptive Dormand-Prince 5(4) with FSAL in scalar Python (fast for a 7-state system),
cubic-Hermite dense output for the terrain-crossing event, step boundaries clipped to burn edges
(MechJeb ReentrySimulation / Trajectories mod use the same "integrate the full drag model" design).
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import Callable, Optional, Sequence

import numpy as np

from . import atmo

G0 = 9.80665
DEG = math.pi / 180.0


# --------------------------------------------------------------------------- body / atmosphere
class _LinearTable:
    """O(1) scalar linear interpolation on a uniformly spaced table (0 outside the top)."""

    def __init__(self, x, y):
        self.x0, self.dx = float(x[0]), float(x[1] - x[0])
        self.n = len(x)
        self.y = [float(v) for v in y]
        assert np.allclose(np.diff(x), self.dx), "table must be uniformly spaced"

    def __call__(self, x):
        u = (x - self.x0) / self.dx
        if u <= 0.0:
            return self.y[0]
        i = int(u)
        if i >= self.n - 1:
            return 0.0
        f = u - i
        return self.y[i] + f * (self.y[i + 1] - self.y[i])


@dataclass(eq=False)
class Body:
    radius: float
    mu: float
    omega: float                  # rad/s, positive = eastward (prograde) rotation
    atm_height: float
    density: Callable[[float], float]
    sound_speed: Optional[Callable[[float], float]] = None   # m/s vs altitude; None -> mach=None

    @classmethod
    def kerbin(cls):
        k = atmo.KERBIN
        return cls(k["radius"], k["mu"], k["rotational_speed"], k["atmosphere_depth"],
                   _LinearTable(np.asarray(k["alt"], float), np.asarray(k["density"], float)))


KERBIN = Body.kerbin()
KSC_PAD = (atmo.KERBIN["ksc_pad"]["lat"], atmo.KERBIN["ksc_pad"]["lon"])
KSC_PAD_ALT = atmo.KERBIN["ksc_pad"]["surface_height"]


# --------------------------------------------------------------------------- frame helpers
def latlonalt_to_bf(lat_deg, lon_deg, alt, body: Body = KERBIN):
    """Body-fixed kRPC-frame position (m) of a point at geodetic lat/lon (deg) and altitude (m)."""
    la, lo = lat_deg * DEG, lon_deg * DEG
    r = body.radius + alt
    return np.array([r * math.cos(la) * math.cos(lo), r * math.sin(la), r * math.cos(la) * math.sin(lo)])


def bf_to_latlonalt(r, body: Body = KERBIN):
    """(lat_deg, lon_deg in [-180,180), alt_m) of a body-fixed kRPC-frame position."""
    x, y, z = float(r[0]), float(r[1]), float(r[2])
    n = math.sqrt(x * x + y * y + z * z)
    return math.asin(y / n) / DEG, math.atan2(z, x) / DEG, n - body.radius


def enu_basis(lat_deg, lon_deg):
    """Unit (east, north, up) vectors in kRPC body axes at lat/lon."""
    la, lo = lat_deg * DEG, lon_deg * DEG
    sa, ca, so, co = math.sin(la), math.cos(la), math.sin(lo), math.cos(lo)
    return (np.array([-so, 0.0, co]), np.array([-sa * co, ca, -sa * so]), np.array([ca * co, sa, ca * so]))


def rot_y(vec, ang):
    """Rotate a kRPC-axes vector eastward (longitude += ang rad) about the north axis."""
    c, s = math.cos(ang), math.sin(ang)
    x, y, z = vec
    return np.array([x * c - z * s, y, x * s + z * c])


def air_velocity(r, omega):
    """Velocity of co-rotating air / surface at r (kRPC axes)."""
    return np.array([-omega * r[2], 0.0, omega * r[0]])


def bf_to_inertial(r_bf, v_bf, t=0.0, body: Body = KERBIN):
    """Body-fixed (surface-relative velocity) state at time t -> inertial (body frame frozen at t=0)."""
    r_i = rot_y(np.asarray(r_bf, float), body.omega * t)
    return r_i, rot_y(np.asarray(v_bf, float), body.omega * t) + air_velocity(r_i, body.omega)


def inertial_to_bf(r_i, v_i, t=0.0, body: Body = KERBIN):
    """Inverse of ``bf_to_inertial``."""
    r_i, v_i = np.asarray(r_i, float), np.asarray(v_i, float)
    return rot_y(r_i, -body.omega * t), rot_y(v_i - air_velocity(r_i, body.omega), -body.omega * t)


def great_circle(lat1, lon1, lat2, lon2, radius=KERBIN.radius):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2
    return 2 * radius * math.asin(min(1.0, math.sqrt(a)))


# --------------------------------------------------------------------------- drag models
def ballistic(cda: float, body: Body = KERBIN):
    """Default drag model: D = 0.5 * rho * v^2 * CdA."""
    rho = body.density
    return lambda alt, v, mach=None: 0.5 * rho(alt) * v * v * cda


class DragTable:
    """Drag force (N) for the retrograde attitude, bilinear on a grid over (altitude, airspeed).

    normalize=True (default) interpolates the effective CdA = F / (0.5 rho v^2) instead of F, which
    is smooth in both axes, extrapolates physically (constant CdA, F ~ rho v^2) and keeps F -> 0
    above the atmosphere. Grid points where rho or v is ~0 are filled from neighbours.
    """

    def __init__(self, alts: Sequence[float], speeds: Sequence[float], force, body: Body = KERBIN,
                 normalize: bool = True):
        self.alts = [float(a) for a in alts]
        self.speeds = [float(s) for s in speeds]
        F = np.asarray(force, float).reshape(len(self.alts), len(self.speeds))
        assert all(np.diff(self.alts) > 0) and all(np.diff(self.speeds) > 0)
        self.rho, self.normalize = body.density, normalize
        if normalize:
            q = 0.5 * np.array([body.density(a) for a in self.alts])[:, None] * np.square(self.speeds)[None, :]
            ok = q > 1e-6
            C = np.where(ok, F / np.where(ok, q, 1.0), np.nan)
            for axis in (1, 0):            # fill invalid cells by nearest valid along speed, then alt
                C = _ffill(C, axis)
            F = np.nan_to_num(C, nan=0.0)
        self.tab = F.tolist()

    def __call__(self, alt, v, mach=None):
        a, s, T = self.alts, self.speeds, self.tab
        i = min(max(bisect.bisect_right(a, alt) - 1, 0), len(a) - 2)
        j = min(max(bisect.bisect_right(s, v) - 1, 0), len(s) - 2)
        fa = min(max((alt - a[i]) / (a[i + 1] - a[i]), 0.0), 1.0)
        fs = min(max((v - s[j]) / (s[j + 1] - s[j]), 0.0), 1.0)
        c = ((1 - fa) * ((1 - fs) * T[i][j] + fs * T[i][j + 1])
             + fa * ((1 - fs) * T[i + 1][j] + fs * T[i + 1][j + 1]))
        return 0.5 * self.rho(alt) * v * v * c if self.normalize else c


def _ffill(C, axis):
    C = np.moveaxis(C.copy(), axis, 0)
    for k in range(1, C.shape[0]):            # forward then backward fill along axis
        C[k] = np.where(np.isnan(C[k]), C[k - 1], C[k])
    for k in range(C.shape[0] - 2, -1, -1):
        C[k] = np.where(np.isnan(C[k]), C[k + 1], C[k])
    return np.moveaxis(C, 0, axis)


# --------------------------------------------------------------------------- burns
@dataclass
class Burn:
    """Constant-thrust segment. Times are absolute (same clock as the prediction, t=0 at the state).

    mode: 'air_retro' (along -airspeed, continuously; entry/landing burn)
          'retro'     (along -inertial velocity, continuously)
          'orbital'   (direction frozen at ignition from components (prograde, normal, radial) of the
                       inertial state, like a KSP manoeuvre node; normal = v x r in kRPC axes, i.e.
                       north for a prograde orbit)
          'inertial'  (fixed inertial unit vector `direction`)
    End: `duration` (s) or `dv` (m/s, converted with the rocket equation at ignition mass).
    alt_trigger: ignition on descending through this altitude (m) instead of at t_start
    (t_start then only arms it), e.g. an entry burn.
    """
    t_start: float
    thrust: float                          # N at full throttle
    isp: float = 300.0                     # s
    throttle: float = 1.0
    mode: str = "air_retro"
    dv: Optional[float] = None
    duration: Optional[float] = None
    components: Sequence[float] = (-1.0, 0.0, 0.0)
    direction: Optional[Sequence[float]] = None
    alt_trigger: Optional[float] = None    # ignite when descending through this altitude (after t_start)

    @property
    def mdot(self):
        return self.thrust * self.throttle / (self.isp * G0)

    def burn_time(self, m0):
        if self.duration is not None:
            return self.duration
        ve = self.isp * G0
        return m0 / self.mdot * (1.0 - math.exp(-self.dv / ve))


# --------------------------------------------------------------------------- result
@dataclass
class Impact:
    hit: bool
    t: float                     # time of impact (s after the input state)
    lat: float
    lon: float
    alt: float
    r_bf: np.ndarray             # body-fixed position at impact
    v_bf: np.ndarray             # surface-relative velocity at impact
    mass: float
    t_hist: np.ndarray = field(repr=False)
    y_hist: np.ndarray = field(repr=False)   # (N,7) inertial [r, v, m]
    t0: float = 0.0
    body: Body = field(default=KERBIN, repr=False)

    @property
    def speed(self):
        return float(np.linalg.norm(self.v_bf))

    def history_bf(self):
        """(t, r_bf (N,3), v_bf (N,3), lat, lon, alt) arrays."""
        t, Y, w = self.t_hist, self.y_hist, self.body.omega
        c, s = np.cos(-w * t), np.sin(-w * t)
        r, v = Y[:, 0:3], Y[:, 3:6]
        vr = v - np.stack([-w * r[:, 2], 0 * r[:, 0], w * r[:, 0]], 1)
        rot = lambda a: np.stack([a[:, 0] * c - a[:, 2] * s, a[:, 1], a[:, 0] * s + a[:, 2] * c], 1)
        rb, vb = rot(r), rot(vr)
        n = np.linalg.norm(rb, axis=1)
        return (t, rb, vb, np.degrees(np.arcsin(rb[:, 1] / n)), np.degrees(np.arctan2(rb[:, 2], rb[:, 0])),
                n - self.body.radius)


# --------------------------------------------------------------------------- integrator
_C = (0.0, 1 / 5, 3 / 10, 4 / 5, 8 / 9, 1.0, 1.0)
_A = ((), (1 / 5,), (3 / 40, 9 / 40), (44 / 45, -56 / 15, 32 / 9),
      (19372 / 6561, -25360 / 2187, 64448 / 6561, -212 / 729),
      (9017 / 3168, -355 / 33, 46732 / 5247, 49 / 176, -5103 / 18656),
      (35 / 384, 0.0, 500 / 1113, 125 / 192, -2187 / 6784, 11 / 84))
_E = (71 / 57600, 0.0, -71 / 16695, 71 / 1920, -17253 / 339200, 22 / 525, -1 / 40)


def _make_rhs(body: Body, drag, burns, mass_min, lift=None):
    """lift: optional callable(alt, airspeed, mach) -> (a_up, a_side) m/s^2 perpendicular to the airflow:
    a_up along the local vertical's part perpendicular to the airspeed, a_side along unit(airspeed x r)."""
    mu, R, w, H = body.mu, body.radius, body.omega, body.atm_height
    ss = body.sound_speed
    sqrt = math.sqrt

    def rhs(t, y, active):
        x, yy, z, vx, vy, vz, m = y
        r2 = x * x + yy * yy + z * z
        r = sqrt(r2)
        g = -mu / (r2 * r)
        ax, ay, az = g * x, g * yy, g * z
        alt = r - R
        ux, uy, uz = vx + w * z, vy, vz - w * x          # airspeed vector
        dm = 0.0
        if alt < H:
            u = sqrt(ux * ux + uy * uy + uz * uz)
            if u > 1e-9:
                D = drag(alt, u, u / ss(alt) if ss else None) / (m * u)
                ax -= D * ux; ay -= D * uy; az -= D * uz
                if lift is not None:
                    a_up, a_sd = lift(alt, u, u / ss(alt) if ss else None)
                    if a_up or a_sd:
                        hx, hy, hz = ux / u, uy / u, uz / u
                        upx, upy, upz = x / r, yy / r, z / r
                        k = upx * hx + upy * hy + upz * hz
                        ex, ey, ez = upx - k * hx, upy - k * hy, upz - k * hz
                        ne = sqrt(ex * ex + ey * ey + ez * ez) or 1.0
                        sx, sy, sz = hy * upz - hz * upy, hz * upx - hx * upz, hx * upy - hy * upx
                        ns = sqrt(sx * sx + sy * sy + sz * sz) or 1.0
                        ax += a_up * ex / ne + a_sd * sx / ns
                        ay += a_up * ey / ne + a_sd * sy / ns
                        az += a_up * ez / ne + a_sd * sz / ns
        if active is not None and m > mass_min:
            b, d = active
            if b.mode == "air_retro":
                u = sqrt(ux * ux + uy * uy + uz * uz) or 1.0
                d = (-ux / u, -uy / u, -uz / u)
            elif b.mode == "retro":
                v = sqrt(vx * vx + vy * vy + vz * vz)
                d = (-vx / v, -vy / v, -vz / v)
            a = b.thrust * b.throttle / m
            ax += a * d[0]; ay += a * d[1]; az += a * d[2]
            dm = -b.mdot
        return (vx, vy, vz, ax, ay, az, dm)
    return rhs


def _orbital_dir(y, comps):
    r, v = np.array(y[0:3]), np.array(y[3:6])
    pro = v / np.linalg.norm(v)
    nor = np.cross(v, r); nor /= np.linalg.norm(nor)
    rad = r - np.dot(r, pro) * pro; rad /= np.linalg.norm(rad)
    d = comps[0] * pro + comps[1] * nor + comps[2] * rad
    return tuple(d / np.linalg.norm(d))


def simulate(y0, t0=0.0, drag=None, burns: Sequence[Burn] = (), body: Body = KERBIN,
             target_alt: float | Callable = KSC_PAD_ALT, t_max=30000.0, rtol=1e-9, atol_r=0.05,
             atol_v=5e-5, mass_min=0.0, h0=1.0, record=True, lift=None):
    """Integrate an inertial state y0 = [r(3), v(3), m] from absolute time t0 until the terrain.

    target_alt: terrain height (m) or callable(lat_deg, lon_deg) -> height (body-fixed).
    Burns run in t_start order; one with alt_trigger ignites at the first downward crossing of
    that altitude after its t_start. Returns (hit, t_end, y_end, t_hist, y_hist).
    """
    drag = drag or (lambda a, v, m=None: 0.0)
    rhs = _make_rhs(body, drag, burns, mass_min, lift)
    R, w = body.radius, body.omega
    terrain = target_alt if callable(target_alt) else None
    h_flat = 0.0 if terrain else float(target_alt)
    edges = [[b.t_start, None, b, None] for b in sorted(burns, key=lambda b: b.t_start)]  # on, off, burn, dir

    def gfun(t, y):
        r = math.sqrt(y[0] ** 2 + y[1] ** 2 + y[2] ** 2)
        if terrain is None:
            return r - R - h_flat
        lat, lon, alt = bf_to_latlonalt(rot_y(y[0:3], -w * t), body)
        return alt - terrain(lat, lon)

    alt_of = lambda y: math.sqrt(y[0] ** 2 + y[1] ** 2 + y[2] ** 2) - R
    t, y = t0, tuple(float(v) for v in y0)
    k = 0                                           # index of next/current burn

    def active_at(t):
        """-> (active (burn, dir) or None, next edge time, armed altitude trigger or None)."""
        nonlocal k
        while k < len(edges):
            e = edges[k]
            if t < e[0] - 1e-9:
                return None, e[0], None
            if e[1] is None:
                trig = e[2].alt_trigger
                if trig is not None and e[3] != "fired":
                    if alt_of(y) > trig:
                        return None, math.inf, trig
                e[0] = t if trig is not None else e[0]
                e[1] = e[0] + e[2].burn_time(y[6])  # ignite: resolve end time and frozen direction
                e[3] = None
                if e[2].mode == "orbital":
                    e[3] = _orbital_dir(y, e[2].components)
                elif e[2].mode == "inertial":
                    d = np.asarray(e[2].direction, float); e[3] = tuple(d / np.linalg.norm(d))
            if t < e[1] - 1e-9:
                return (e[2], e[3]), e[1], None
            k += 1
        return None, math.inf, None

    active, t_edge, trig = active_at(t)
    f = rhs(t, y, active)
    T, Y = ([t], [y]) if record else (None, None)
    g_prev = gfun(t, y)
    if g_prev <= 0:
        return True, t, np.array(y), np.array([t]), np.array([y])
    tol = (atol_r, atol_r, atol_r, atol_v, atol_v, atol_v, 1e-3)
    h_try, n_ref, n_trig = h0, 0, 0
    A, Ee, Cc = _A, _E, _C
    while t < t0 + t_max:
        h, clip = h_try, False
        if t + h >= t_edge:
            h, clip = t_edge - t, True
        ks = [f]                                    # Dormand-Prince 5(4), FSAL
        for s in range(1, 7):
            a = A[s]
            yi = tuple(y[n] + h * sum(a[j] * ks[j][n] for j in range(s)) for n in range(7))
            ks.append(rhs(t + Cc[s] * h, yi, active))
        y_new = yi
        err = 0.0
        for n in range(7):
            e = h * sum(Ee[j] * ks[j][n] for j in range(7))
            err = max(err, abs(e) / (tol[n] + rtol * max(abs(y[n]), abs(y_new[n]))))
        if err > 1.0:
            h_try = h * max(0.2, 0.9 * err ** -0.2)
            continue
        t_new, f_new = t + h, ks[6]
        g_new = gfun(t_new, y_new)
        if g_new <= 0.0:                            # terrain crossing
            s, ys = _hroot(lambda tt, yy: gfun(tt, yy), t, y, f, y_new, f_new, h, g_prev, g_new)
            if s < 0.98 and s * h > 0.02 and n_ref < 4:   # re-step to land just past the root,
                n_ref += 1                                # where the Hermite error vanishes
                h_try = s * h * 1.002 + 0.005
                continue
            if record:
                T.append(t + s * h); Y.append(ys)
            return True, t + s * h, np.array(ys), np.array(T) if record else None, np.array(Y) if record else None
        if trig is not None and alt_of(y_new) <= trig:   # altitude-triggered ignition
            s, _ = _hroot(lambda tt, yy: alt_of(yy) - trig, t, y, f, y_new, f_new, h,
                          alt_of(y) - trig, alt_of(y_new) - trig)
            if s < 0.98 and s * h > 0.01 and n_trig < 4:
                n_trig += 1
                h_try = s * h * 1.0005 + 0.002
                continue
            edges[k][3], clip, n_trig = "fired", True, 0
        t, y, f, g_prev = t_new, y_new, f_new, g_new
        if record:
            T.append(t); Y.append(y)
        if clip:
            active, t_edge, trig = active_at(t)
            f = rhs(t, y, active)
        else:
            h_try = h * min(5.0, 0.9 * max(err, 1e-10) ** -0.2)
    return False, t, np.array(y), np.array(T) if record else None, np.array(Y) if record else None


def _hroot(g, t, y0, f0, y1, f1, h, g0, g1):
    """Illinois root of g along the cubic-Hermite interpolant of a step; returns (s, y(s))."""
    ta, tb, ga, gb, side = 0.0, 1.0, g0, g1, 0
    s, ys = 1.0, y1
    for _ in range(40):
        s = (ta * gb - tb * ga) / (gb - ga)
        ys = _hermite(y0, f0, y1, f1, h, s)
        gs = g(t + s * h, ys)
        if abs(gs) < 1e-3:
            break
        if gs > 0:
            ta, ga = s, gs
            if side == -1: gb *= 0.5
            side = -1
        else:
            tb, gb = s, gs
            if side == 1: ga *= 0.5
            side = 1
    return s, ys


def _hermite(y0, f0, y1, f1, h, s):
    """Cubic Hermite interpolation of positions/velocities using states and derivatives."""
    h00, h10, h01, h11 = 2 * s ** 3 - 3 * s ** 2 + 1, s ** 3 - 2 * s ** 2 + s, -2 * s ** 3 + 3 * s ** 2, s ** 3 - s ** 2
    return tuple(h00 * y0[n] + h10 * h * f0[n] + h01 * y1[n] + h11 * h * f1[n] for n in range(7))


def predict(r_bf, v_bf, mass, drag=None, cda=None, burns: Sequence[Burn] = (), body: Body = KERBIN,
            target_alt=KSC_PAD_ALT, t0=0.0, **kw) -> Impact:
    """Impact prediction from a body-fixed kRPC state (velocity relative to the surface).

    drag: callable(alt_m, airspeed_mps, mach_or_None) -> N; or give cda for the ballistic model.
    Burn times are relative to the state (t = 0). t0 only offsets the planet rotation clock (state
    is body-fixed at time t0 of the inertial frame); normally leave 0.
    """
    if drag is None and cda is not None:
        drag = ballistic(cda, body)
    r_i, v_i = bf_to_inertial(r_bf, v_bf, t0, body)
    return predict_inertial(r_i, v_i, mass, drag, burns, body, target_alt, t0, **kw)


def predict_inertial(r_i, v_i, mass, drag=None, burns=(), body: Body = KERBIN, target_alt=KSC_PAD_ALT,
                     t0=0.0, **kw) -> Impact:
    """As ``predict`` but with an inertial state (frame = body frame at t=0) at absolute time t0."""
    y0 = np.concatenate([np.asarray(r_i, float), np.asarray(v_i, float), [float(mass)]])
    hit, t, y, T, Y = simulate(y0, t0, drag, burns, body, target_alt, **kw)
    rb, vb = inertial_to_bf(y[0:3], y[3:6], t, body)
    lat, lon, alt = bf_to_latlonalt(rb, body)
    if T is None:
        T, Y = np.array([t]), y[None, :]
    return Impact(hit, t - t0, lat, lon, alt, rb, vb, float(y[6]), T, Y, t0, body)


# --------------------------------------------------------------------------- conics
def _stumpff(z):
    if z > 1e-6:
        s = math.sqrt(z); return (1 - math.cos(s)) / z, (s - math.sin(s)) / s ** 3
    if z < -1e-6:
        s = math.sqrt(-z); return (math.cosh(s) - 1) / -z, (math.sinh(s) - s) / s ** 3
    return 0.5 - z / 24 + z * z / 720, 1 / 6 - z / 120 + z * z / 5040


def kepler_propagate(r0, v0, dt, mu=KERBIN.mu):
    """Two-body propagation by dt (universal variables, Newton). Frame-agnostic."""
    r0, v0 = np.asarray(r0, float), np.asarray(v0, float)
    rn, vr = np.linalg.norm(r0), np.dot(r0, v0) / np.linalg.norm(r0)
    alpha = 2 / rn - np.dot(v0, v0) / mu
    sm = math.sqrt(mu)
    chi = sm * abs(alpha) * dt if alpha > 1e-12 else sm * dt / rn
    for _ in range(50):
        z = alpha * chi * chi
        C, S = _stumpff(z)
        F = rn * vr / sm * chi * chi * C + (1 - alpha * rn) * chi ** 3 * S + rn * chi - sm * dt
        dF = rn * vr / sm * chi * (1 - z * S) + (1 - alpha * rn) * chi * chi * C + rn
        d = F / dF
        chi -= d
        if abs(d) < 1e-9:
            break
    z = alpha * chi * chi
    C, S = _stumpff(z)
    f, g = 1 - chi * chi / rn * C, dt - chi ** 3 / sm * S
    r = f * r0 + g * v0
    rr = np.linalg.norm(r)
    fd, gd = sm / (rr * rn) * (alpha * chi ** 3 * S - chi), 1 - chi * chi / rr * C
    return r, fd * r0 + gd * v0


def orbit_elements(r, v, mu=KERBIN.mu):
    """(a, e, period or inf, periapsis radius, apoapsis radius)."""
    r, v = np.asarray(r, float), np.asarray(v, float)
    rn = np.linalg.norm(r)
    a = 1 / (2 / rn - np.dot(v, v) / mu)
    ev = ((np.dot(v, v) - mu / rn) * r - np.dot(r, v) * v) / mu
    e = float(np.linalg.norm(ev))
    per = 2 * math.pi * math.sqrt(a ** 3 / mu) if a > 0 else math.inf
    return a, e, per, a * (1 - e), (a * (1 + e) if e < 1 else math.inf)


def conic_impact(r0, v0, r_target, mu=KERBIN.mu):
    """Vacuum time of flight to descend through radius r_target and the inertial position there.

    Elliptic orbits only. Returns (tof, r_vec) or None if the conic never reaches r_target.
    """
    r0, v0 = np.asarray(r0, float), np.asarray(v0, float)
    a, e, per, rp, ra = orbit_elements(r0, v0, mu)
    if not (e < 1 and rp <= r_target <= ra):
        return None
    rn = np.linalg.norm(r0)
    p = a * (1 - e * e)
    nu0 = math.atan2(math.sqrt(p / mu) * np.dot(r0, v0), p - rn)      # true anomaly at r0
    cos_nut = np.clip((p / r_target - 1) / e, -1, 1)
    nut = 2 * math.pi - math.acos(cos_nut)                             # descending branch
    E = lambda nu: 2 * math.atan(math.sqrt((1 - e) / (1 + e)) * math.tan(nu / 2))
    M = lambda nu: (lambda EE: EE - e * math.sin(EE))(E(nu)) % (2 * math.pi)
    n = 2 * math.pi / per
    dM = (M(nut) - M(nu0 % (2 * math.pi))) % (2 * math.pi)
    tof = dM / n
    dnu = (nut - nu0) % (2 * math.pi)
    q = v0 - np.dot(v0, r0) / rn ** 2 * r0
    q /= np.linalg.norm(q)
    return tof, r_target * (math.cos(dnu) * r0 / rn + math.sin(dnu) * q)


# --------------------------------------------------------------------------- ground-track errors
def track_heading(imp: Impact, target, ref_alt=40e3):
    """Heading (deg from north, at the target) of the incoming ground track.

    Great circle from the ground point where the trajectory passed ref_alt (or its start) to the
    target; falls back to the last horizontal surface-velocity direction if that point is < 5 km away.
    """
    t, rb, vb, lat, lon, alt = imp.history_bf()
    i = int(np.argmax(alt <= ref_alt)) if np.any(alt <= ref_alt) else 0
    e, n, u = enu_basis(*target)
    tv = u
    p1 = rb[i] / np.linalg.norm(rb[i])
    hvec = -(p1 - np.dot(p1, tv) * tv)            # direction at target pointing away from p1
    if np.linalg.norm(hvec) * imp.body.radius < 5e3:
        hv = np.zeros(3)
        for j in range(len(vb) - 1, -1, -1):        # last sample with meaningful horizontal speed
            uj = rb[j] / np.linalg.norm(rb[j])
            hv = vb[j] - np.dot(vb[j], uj) * uj
            if np.linalg.norm(hv) > 5.0:
                break
        hvec = hv
    return math.degrees(math.atan2(np.dot(hvec, e), np.dot(hvec, n)))


def ground_errors(lat, lon, target, heading_deg, radius=KERBIN.radius):
    """(along, cross) metres of point (lat, lon) relative to target along a track of given heading.

    along > 0: beyond the target (long); cross > 0: right of track. Exact on the sphere.
    """
    e, n, t = enu_basis(*target)
    ps = math.radians(heading_deg)
    h = math.sin(ps) * e + math.cos(ps) * n
    right = math.cos(ps) * e - math.sin(ps) * n
    p = enu_basis(lat, lon)[2]
    return (radius * math.atan2(np.dot(p, h), np.dot(p, t)), radius * math.asin(np.clip(np.dot(p, right), -1, 1)))


class CalibratedDrag:
    """Drag table times a learned altitude correction k(alt) (logs/drag_calib.json, see
    tools/calibrate_drag.py). Outside the learned range k -> 1 (low air) or the edge value (high air)."""

    def __init__(self, table, alts, ks):
        self.table, self.alts, self.ks = table, list(alts), list(ks)

    def k(self, alt):
        if not self.alts:
            return 1.0
        if alt <= self.alts[0]:
            return 1.0 if self.alts[0] > 15000 else self.ks[0]
        return float(np.interp(alt, self.alts, self.ks))

    def __call__(self, alt, v, mach=None):
        return self.k(alt) * self.table(alt, v, mach)
