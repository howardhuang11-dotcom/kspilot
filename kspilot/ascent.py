"""Ascent to an arbitrary orbit with our own guidance (kspilot.ascent_own), followed by Ap/Pe/inclination
trim burns planned by kspilot.maneuver and flown by kspilot.burn. No MechJeb (removed 2026-10-21)."""


def trim_orbit(conn, pe_m, ap_m, log, tol_m=1000, inc_deg=None, tol_inc=0.1, accept_m=5000):
    """Bring Ap/Pe (and inclination) within tolerance with our own maneuver calculator and burn
    executor (kspilot.maneuver / kspilot.burn). Returns True within tol_m; a result within accept_m is
    accepted with a warning."""
    import math
    import numpy as np
    from . import burn, maneuver as mv
    from .entry import kepler_propagate
    sc = conn.space_center
    v = sc.active_vessel
    R = v.orbit.body.equatorial_radius
    circular = abs(ap_m - pe_m) < 2000

    def burn_at(apo, fn, what):
        r, vel, mu = burn.state(v)
        t = mv.time_to_radius_extremum(r, vel, mu, apo=apo)
        if t < 30:                                  # too close to set up: use the next revolution
            t += orbit_period(r, vel, mu)
        r1, v1 = kepler_propagate(r, vel, t, mu)
        dv = fn(r1, v1, mu)
        log("%s at %s: %.1f m/s" % (what, "apoapsis" if apo else "periapsis", float(np.linalg.norm(dv))))
        burn.execute_burn(conn, v, sc.ut + t, dv, log)

    for _ in range(3):
        o = v.orbit
        dap, dpe = o.apoapsis_altitude - ap_m, o.periapsis_altitude - pe_m
        log("orbit Ap=%.1fkm Pe=%.1fkm inc=%.3f (err Ap %+.0fm Pe %+.0fm)" % (
            o.apoapsis_altitude / 1e3, o.periapsis_altitude / 1e3, o.inclination * 57.29578, dap, dpe))
        if abs(dap) <= tol_m and abs(dpe) <= tol_m:
            break
        if circular and abs(dap) < tol_m and abs(dpe) > tol_m:
            burn_at(True, mv.dv_circularize, "circularize")
            continue
        if abs(dpe) > tol_m:
            burn_at(True, lambda r, vv, mu: mv.dv_change_periapsis(r, vv, mu, R + pe_m), "periapsis -> %.0f km" % (pe_m / 1e3))
        o = v.orbit
        if abs(o.apoapsis_altitude - ap_m) > tol_m:
            burn_at(False, lambda r, vv, mu: mv.dv_change_apoapsis(r, vv, mu, R + ap_m), "apoapsis -> %.0f km" % (ap_m / 1e3))
    o = v.orbit
    if inc_deg is not None and abs(o.inclination * 57.29578 - inc_deg) > tol_inc:
        r, vel, mu = burn.state(v)
        eq = np.array([0.0, 1.0, 0.0])              # equatorial plane normal (kRPC frames: y = pole)
        ts = [t for t in (mv.time_to_plane_crossing(r, vel, mu, eq, True),
                          mv.time_to_plane_crossing(r, vel, mu, eq, False)) if t and t > 30]
        if ts:
            t = min(ts)
            r1, v1 = kepler_propagate(r, vel, t, mu)
            dv = mv.dv_change_inclination(r1, v1, inc_deg)
            log("plane change to %.2f deg at the node: %.1f m/s" % (inc_deg, float(np.linalg.norm(dv))))
            burn.execute_burn(conn, v, sc.ut + t, dv, log)
        o = v.orbit
        if abs(o.inclination * 57.29578 - inc_deg) > tol_inc:
            return False
    err = max(abs(o.apoapsis_altitude - ap_m), abs(o.periapsis_altitude - pe_m))
    if tol_m < err <= accept_m:
        log("orbit accepted within %.1f km" % (err / 1e3))
    return err <= accept_m and o.periapsis_altitude > 70000


def orbit_period(r, v, mu):
    from .entry import orbit_elements
    return orbit_elements(r, v, mu)[2]


def ascend(conn, pe_m, ap_m, inc_deg, log, autostage_limit=0, timeout=1800):
    """Own ascent guidance to orbit, then our own trims. Returns True when the orbit is within tolerance."""
    from .ascent_own import Ascent
    if not Ascent(conn, pe_m, ap_m, inc_deg, log, autostage_limit=autostage_limit).fly(timeout=timeout):
        return False
    return trim_orbit(conn, pe_m, ap_m, log, inc_deg=inc_deg)
