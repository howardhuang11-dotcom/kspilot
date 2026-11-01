"""Own burn executor (replaces MechJeb's node executor).

execute_burn(conn, v, ut, dv): fly an impulsive delta-v `dv` (non-rotating frame of the main body),
centred on game time `ut`:
  - burn time from the rocket equation, start at ut - t_burn/2 (rails warp up to ~60 s before)
  - point at the remaining delta-v vector, measured as velocity gained minus two-body gravity
    (like a maneuver node's remaining dv), so steering errors are corrected during the burn
  - proportional throttle-down at the end for precision; tiny burns (< 5 m/s) use the timed-pulse
    path of mission.burn_vector
  - autostaging when the thrust disappears (never past autostage_limit)
Stock maneuver nodes may still be used to show / store a plan: fly them with execute_node().
"""
import math
import time

import numpy as np

from . import core

G0 = 9.80665


def burn_time(v, dv):
    """Seconds of full thrust for dv with the current stage (rocket equation)."""
    F = max(v.available_thrust, 1e-3)
    isp = max(v.specific_impulse, 1.0)
    ve = isp * G0
    return v.mass * ve / F * (1.0 - math.exp(-dv / ve))


def _aligned(v, nrf, u, deg):
    d = np.asarray(v.direction(nrf))
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(d, u)))))) < deg


def execute_burn(conn, v, ut, dv, log, autostage_limit=0, tol=0.05, align_deg=1.0, max_late=20.0):
    """Returns the delta-v left unflown (m/s); None if the burn time was missed by more than max_late
    seconds (nothing flown: a late burn lands somewhere else - the caller must re-plan)."""
    sc = conn.space_center
    nrf = v.orbit.body.non_rotating_reference_frame
    dv = np.asarray(dv, float)
    mag = float(np.linalg.norm(dv))
    if mag < 0.01:
        return 0.0
    if mag < 5.0:                                   # small: timed pulse at the burn time
        from .mission import burn_vector
        if ut - sc.ut > 40:
            core.warp_to(sc, ut - 30)       # = sc.warp_to; recording v2: waits out a 1x window first
        _point(v, nrf, dv / mag)
        core.wait_until(lambda: sc.ut >= ut - 0.5, max(ut - sc.ut + 5, 5), 0.05)
        if sc.ut - ut > max_late:
            log("burn: MISSED by %.0fs - not flown, re-plan" % (sc.ut - ut))
            v.auto_pilot.engaged = False
            return None
        got = burn_vector(conn, v, dv, log, nrf) or 0.0
        return max(mag - got, 0.0)
    tb = burn_time(v, mag)
    t_start = ut - tb / 2
    lead = t_start - sc.ut
    if lead > 75:
        log("burn: warping %.0fs (dv %.1f m/s, %.1f s burn)" % (lead - 60, mag, tb))
        core.warp_to(sc, t_start - 60)
    ap = _point(v, nrf, dv / mag)
    core.wait_until(lambda: _aligned(v, nrf, dv / mag, align_deg), 90, 0.2)
    wait = t_start - sc.ut
    if wait > 3:
        core.wait_until(lambda: sc.ut >= t_start, wait + 5, 0.05)
    elif wait < -max_late:
        log("burn: MISSED by %.0fs - not flown, re-plan" % -wait)
        ap.engaged = False
        return None
    # measure the burn by the velocity actually gained minus gravity (two-body): no thrust-stream lag,
    # spool-up and tail-off are counted exactly (the way a stock maneuver node's remaining dv works)
    mu = v.orbit.body.gravitational_parameter
    pos = conn.add_stream(v.position, nrf)
    vel = conn.add_stream(v.velocity, nrf)
    clock = conn.add_stream(getattr, sc, 'ut')
    mass = conn.add_stream(getattr, v, 'mass')
    def grav(r):
        r = np.asarray(r)
        return -mu * r / np.linalg.norm(r) ** 3
    v0, t_prev = np.asarray(vel()), clock()
    g_prev, g_int = grav(pos()), np.zeros(3)
    left = dv.copy()
    w0, last_progress, best = time.time(), time.time(), mag
    v.control.throttle = 1.0
    try:
        while True:
            t_now = clock()
            g_now = grav(pos())
            g_int += 0.5 * (g_prev + g_now) * (t_now - t_prev)
            g_prev, t_prev = g_now, t_now
            left = dv - (np.asarray(vel()) - v0 - g_int)
            n = float(np.linalg.norm(left))
            if n < best - 0.01:
                best, last_progress = n, time.time()
            if n < tol or float(np.dot(left, dv)) < 0:      # done (or overshot along the plan)
                break
            if n > 1.0:                                     # steer at the remaining vector (not at the end)
                ap.target_direction = tuple(left / n)
            a_full = max(v.available_thrust / max(mass(), 1.0), 1e-3)
            v.control.throttle = float(min(1.0, max(0.02, n / (a_full * 1.5))))   # ~1.5 s tail-off
            if v.available_thrust < 1.0:                    # stage burnt out
                if v.control.current_stage > autostage_limit:
                    log("burn: staging (stage %d burnt out, %.1f m/s left)" % (v.control.current_stage, n))
                    v.control.throttle = 0.0
                    v.control.activate_next_stage()
                    time.sleep(1.0)
                    v.control.throttle = 1.0
                else:
                    log("burn: out of thrust with %.1f m/s left" % n)
                    break
            if time.time() - last_progress > 20:
                from .mission import power
                log("burn: no progress for 20 s, %.1f m/s left - stopping (thrust %.0f N, EC %.0f/%.0f)" % (
                    (n, v.thrust) + power(v)))
                break
            if time.time() - w0 > 3 * tb + 60:
                log("burn: time cap, %.1f m/s left" % n)
                break
            time.sleep(0.02)
    finally:
        v.control.throttle = 0.0
        for s_ in (pos, vel, clock, mass):
            s_.remove()
    if core.WARP_GUARD is not None:         # recording v2 (HOP_REC=full) only: no warp before cutoff + 10 s + 2 s
        core.WARP_GUARD.note_cutoff(t_now)  # (UT of the last loop pass, read from the stream: no extra RPC)
    core.wait_until(lambda: v.thrust < 1.0, 2.0, 0.05)
    ap.engaged = False
    left_mag = float(np.linalg.norm(left))
    log("burn done: %.1f m/s planned, %.2f m/s left" % (mag, left_mag))
    return left_mag


def _point(v, nrf, u):
    ap = v.auto_pilot
    ap.reference_frame = nrf
    ap.target_direction = tuple(float(x) for x in u)
    ap.engaged = True
    return ap


def execute_node(conn, v, node, log, autostage_limit=0):
    """Fly a stock maneuver node with our executor, then remove it."""
    nrf = v.orbit.body.non_rotating_reference_frame
    dv = np.asarray(node.burn_vector(nrf))
    left = execute_burn(conn, v, node.ut, dv, log, autostage_limit)
    node.remove()
    return left          # None = missed


def state(v):
    """(r, v, mu) of the vessel in the body's non-rotating frame (input for kspilot.maneuver)."""
    nrf = v.orbit.body.non_rotating_reference_frame
    return np.asarray(v.position(nrf)), np.asarray(v.velocity(nrf)), v.orbit.body.gravitational_parameter
