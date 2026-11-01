"""Orbital maneuver calculator (own replacement for MechJeb's maneuver planner).

Approach studied from MechJeb2's OrbitalManeuverCalculator (GPLv3) and re-derived here:
every maneuver is a closed-form / one-dimensional problem at a burn time chosen by the caller
(apoapsis, periapsis, a plane-crossing node...). Burns keep the radial velocity and change only the
horizontal velocity, either its magnitude (apsis changes) or its direction (plane changes).

Frames: any inertial frame centred on the body. KSP / kRPC frames are left-handed with the rotation
axis along +y, so with numpy's (right-handed) cross product a prograde orbit has h = r x v along -y:
POLE = (0, -1, 0) is the prograde pole in kRPC's body-centred non-rotating frame.
All vectors are numpy arrays in metres and m/s.
"""
import math

import numpy as np

from .entry import kepler_propagate, orbit_elements

POLE = np.array([0.0, -1.0, 0.0])


def _unit(x):
    return x / np.linalg.norm(x)


def _split(r, v):
    """Radial unit vector, radial speed, horizontal velocity vector."""
    u = _unit(r)
    vr = float(np.dot(v, u))
    return u, vr, v - vr * u


def apsides(r, v, mu):
    """(periapsis radius, apoapsis radius) of the conic through r, v (apoapsis inf if unbound)."""
    _, _, _, rp, ra = orbit_elements(r, v, mu)
    return rp, ra


def inclination(r, v, pole=POLE):
    """Inclination in degrees (0 = prograde equatorial, 180 = retrograde)."""
    h = np.cross(r, v)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(_unit(h), pole))))))


# ----------------------------------------------------------------------------- burn times
def time_to_radius_extremum(r, v, mu, apo=True):
    """Seconds until the next apoapsis (apo=True) or periapsis of an elliptic orbit."""
    a, e, per, _, _ = orbit_elements(r, v, mu)
    if e < 1e-9 or not math.isfinite(per):
        return 0.0
    rn = np.linalg.norm(r)
    p = a * (1 - e * e)
    nu = math.atan2(math.sqrt(p / mu) * float(np.dot(r, v)), p - rn)       # true anomaly now
    E = 2 * math.atan(math.sqrt((1 - e) / (1 + e)) * math.tan(nu / 2))
    M = (E - e * math.sin(E)) % (2 * math.pi)
    n = 2 * math.pi / per
    target = math.pi if apo else 0.0
    return ((target - M) % (2 * math.pi)) / n


def time_to_plane_crossing(r, v, mu, plane_normal, ascending=True, steps=72):
    """Seconds until the orbit crosses the plane with unit normal plane_normal (r . n changes sign;
    ascending = from negative to positive). Bracket by sampling one period, refine by bisection."""
    _, _, per, _, _ = orbit_elements(r, v, mu)
    span = 1.05 * per if math.isfinite(per) else 1e5     # a bit more than one revolution
    f = lambda t: float(np.dot(kepler_propagate(r, v, t, mu)[0], plane_normal))
    t_prev = 1.0                                         # the next crossing, not one we sit on now
    f_prev = f(t_prev)
    for k in range(1, steps + 1):
        t1 = 1.0 + (span - 1.0) * k / steps
        f1 = f(t1)
        if (f_prev < 0 <= f1) if ascending else (f_prev > 0 >= f1):
            lo, hi, flo = t_prev, t1, f_prev
            for _ in range(60):
                mid = 0.5 * (lo + hi)
                fm = f(mid)
                if (fm < 0) == (flo < 0):
                    lo, flo = mid, fm
                else:
                    hi = mid
            return 0.5 * (lo + hi)
        t_prev, f_prev = t1, f1
    return None


# ----------------------------------------------------------------------------- burns at a given state
def dv_circularize(r, v, mu):
    """Delta-v making the orbit circular at the current radius (horizontal, radial speed removed)."""
    u, _, vh = _split(r, v)
    v_new = math.sqrt(mu / np.linalg.norm(r)) * _unit(vh)
    return v_new - v


def _dv_horizontal_speed(r, v, mu, f, target):
    """Change only the horizontal speed so that f(r, v') == target (f monotonic in that speed)."""
    u, vr, vh = _split(r, v)
    dh = _unit(vh)
    rn = float(np.linalg.norm(r))
    v_esc = math.sqrt(2 * mu / rn)
    g = lambda s: f(r, vr * u + s * dh) - target
    lo, hi = 1e-3, math.sqrt(max(v_esc ** 2 - vr ** 2, 1.0)) * 0.999999
    glo, ghi = g(lo), g(hi)
    if glo * ghi > 0:          # unreachable from here: go to the nearer end
        s = lo if abs(glo) < abs(ghi) else hi
    else:
        for _ in range(100):
            mid = 0.5 * (lo + hi)
            gm = g(mid)
            if (gm < 0) == (glo < 0):
                lo, glo = mid, gm
            else:
                hi = mid
        s = 0.5 * (lo + hi)
    return vr * u + s * dh - v


def dv_change_periapsis(r, v, mu, new_rp):
    """Horizontal burn here that puts the periapsis at radius new_rp (clamped below the current radius)."""
    new_rp = min(new_rp, float(np.linalg.norm(r)) * 0.999999)
    return _dv_horizontal_speed(r, v, mu, lambda rr, vv: apsides(rr, vv, mu)[0], new_rp)


def dv_change_apoapsis(r, v, mu, new_ra):
    """Horizontal burn here that puts the apoapsis at radius new_ra (clamped above the current radius)."""
    new_ra = max(new_ra, float(np.linalg.norm(r)) * 1.000001)
    return _dv_horizontal_speed(r, v, mu, lambda rr, vv: apsides(rr, vv, mu)[1], new_ra)


def dv_change_inclination(r, v, target_deg, pole=POLE, n=720):
    """Rotate the horizontal velocity about the local vertical (speed kept) to reach the target
    inclination; of the two solutions the cheaper one. If the latitude makes it unreachable, the
    closest inclination is used (= |latitude|)."""
    u, vr, vh = _split(r, v)
    s = float(np.linalg.norm(vh))
    e1 = _unit(vh)
    e2 = np.cross(u, e1)
    def vel(th):
        return vr * u + s * (math.cos(th) * e1 + math.sin(th) * e2)
    best = None
    ths = np.linspace(-math.pi, math.pi, n, endpoint=False)
    errs = [inclination(r, vel(th), pole) - target_deg for th in ths]
    for k in range(n):                         # sign changes -> exact roots by bisection
        a, b = ths[k], ths[(k + 1) % n] + (2 * math.pi if k == n - 1 else 0.0)
        fa, fb = errs[k], errs[(k + 1) % n]
        if fa == 0:                             # sample lands exactly on the target
            dv = vel(a) - v
            if best is None or np.linalg.norm(dv) < np.linalg.norm(best):
                best = dv
        elif fa * fb < 0:
            for _ in range(60):
                mid = 0.5 * (a + b)
                fm = inclination(r, vel(mid), pole) - target_deg
                if (fm < 0) == (fa < 0):
                    a, fa = mid, fm
                else:
                    b = mid
            dv = vel(0.5 * (a + b)) - v
            if best is None or np.linalg.norm(dv) < np.linalg.norm(best):
                best = dv
    if best is None:
        # no sign change: the target is a tangency (0 / 180 deg, or |latitude| when unreachable) -
        # minimise |error| around the best sample (golden section)
        k = int(np.argmin(np.abs(errs)))
        step = 2 * math.pi / n
        a, b = ths[k] - step, ths[k] + step
        f = lambda th: abs(inclination(r, vel(th), pole) - target_deg)
        g = (math.sqrt(5) - 1) / 2
        c, d = b - g * (b - a), a + g * (b - a)
        for _ in range(80):
            if f(c) < f(d):
                b, d = d, c
                c = b - g * (b - a)
            else:
                a, c = c, d
                d = a + g * (b - a)
        best = vel(0.5 * (a + b)) - v
    return best


def dv_match_plane(r, v, plane_normal):
    """At a crossing of the target plane: rotate the horizontal velocity into that plane
    (speed and radial velocity kept), on the same side as the current motion."""
    u, vr, vh = _split(r, v)
    d = np.cross(_unit(plane_normal), u)       # horizontal direction lying in the target plane
    d = _unit(d) if np.dot(d, vh) >= 0 else -_unit(d)
    return vr * u + float(np.linalg.norm(vh)) * d - v


# ----------------------------------------------------------------------------- plans
def plan_hohmann(r, v, mu, new_radius):
    """Two burns to a circular orbit of new_radius from a near-circular orbit: (dt1, dv1, dt2, dv2)
    with dt from now. Burn 1 now (raise/lower the opposite apsis), burn 2 at that apsis (circularize)."""
    raise_ = new_radius > np.linalg.norm(r)
    dv1 = dv_change_apoapsis(r, v, mu, new_radius) if raise_ else dv_change_periapsis(r, v, mu, new_radius)
    v1 = v + dv1
    dt2 = time_to_radius_extremum(r, v1, mu, apo=raise_)
    r2, v2 = kepler_propagate(r, v1, dt2, mu)
    return 0.0, dv1, dt2, dv_circularize(r2, v2, mu)
