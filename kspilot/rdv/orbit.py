"""Two-body helpers for rendezvous at Kerbin: Hohmann / phasing planner and a Lambert solver.

Constants come from data/kerbin.json (sampled from the game, via kspilot.atmo): mu = 3.5316e12 m^3/s^2,
R = 600 km. Frames: any body-centred inertial frame. Everything here is RIGHT-handed (numpy cross
product); the prograde pole defaults to +z. kRPC's body-centred frames are left-handed with the prograde
pole along -y (kspilot.maneuver.POLE) -- pass pole=maneuver.POLE when feeding kRPC vectors to lambert().
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .. import atmo
from ..entry import kepler_propagate

MU = float(atmo.MU)          # 3.5316e12 m^3/s^2  [verified: data/kerbin.json]
R_KERBIN = float(atmo.R)     # 600000 m           [verified: data/kerbin.json]
ZPOLE = np.array([0.0, 0.0, 1.0])


def circ_speed(r, mu=MU):
    return math.sqrt(mu / r)


def mean_motion(r, mu=MU):
    return math.sqrt(mu / r ** 3)


def period(a, mu=MU):
    return 2 * math.pi * math.sqrt(a ** 3 / mu)


def hohmann(r1, r2, mu=MU):
    """(dv1, dv2, tof): signed tangential burns (+ = prograde) between circular radii r1 -> r2."""
    a = 0.5 * (r1 + r2)
    dv1 = math.sqrt(mu / r1) * (math.sqrt(r2 / a) - 1.0)
    dv2 = math.sqrt(mu / r2) * (1.0 - math.sqrt(r1 / a))
    return dv1, dv2, math.pi * math.sqrt(a ** 3 / mu)


def plane_change_dv(v, d_inc_rad):
    """Pure plane change of a circular orbit at speed v by d_inc (at a node)."""
    return 2 * v * math.sin(abs(d_inc_rad) / 2)


@dataclass
class RendezvousPlan:
    r_chaser: float          # m, chaser circular orbit radius
    r_target: float          # m, target circular orbit radius
    phase0: float            # rad, target angle AHEAD of the chaser at t = 0 (in [0, 2pi))
    aim_behind: float        # m, along-track distance the chaser arrives behind the target (V-bar hold)
    t_wait: float            # s, coast before the first burn
    dv1: float               # m/s, prograde at t_wait
    tof: float               # s, transfer time (half ellipse)
    dv2: float               # m/s, prograde at t_wait + tof (circularise at the target radius)
    lead_angle: float        # rad, target lead required at burn 1
    synodic: float           # s, synodic period (worst-case wait)
    catch_up_per_rev: float  # rad, phase gained per chaser revolution

    @property
    def t_burn1(self):
        return self.t_wait

    @property
    def t_burn2(self):
        return self.t_wait + self.tof

    @property
    def dv_total(self):
        return abs(self.dv1) + abs(self.dv2)


def plan_hohmann_rendezvous(r_chaser, r_target, phase0, aim_behind=1000.0, mu=MU):
    """Coplanar circular -> circular Hohmann rendezvous that ends `aim_behind` metres behind the target
    on its V-bar with matched velocity (a natural CW hold point).

    The lead angle is theta_b = pi - n_t * tof + aim_behind / r_target; the phase angle drifts at
    (n_c - n_t) per second, so the wait is the first time the phase equals theta_b.
    """
    n1, n2 = mean_motion(r_chaser, mu), mean_motion(r_target, mu)
    dv1, dv2, tof = hohmann(r_chaser, r_target, mu)
    lead = (math.pi - n2 * tof + aim_behind / r_target) % (2 * math.pi)
    dn = n1 - n2
    if abs(dn) < 1e-12:
        raise ValueError("co-orbital: no phasing rate, use a phasing orbit")
    ph0 = phase0 % (2 * math.pi)
    if dn > 0:          # chaser below: the target's lead shrinks
        t_wait = ((ph0 - lead) % (2 * math.pi)) / dn
    else:
        t_wait = ((lead - ph0) % (2 * math.pi)) / -dn
    return RendezvousPlan(r_chaser, r_target, ph0, aim_behind, t_wait, dv1, tof, dv2, lead,
                          2 * math.pi / abs(dn), abs(dn) * 2 * math.pi / n1)


def circular_state(r, angle, mu=MU):
    """Equatorial (z-pole) prograde circular state at the given angle (right-handed frame)."""
    c, s = math.cos(angle), math.sin(angle)
    v = circ_speed(r, mu)
    return np.array([r * c, r * s, 0.0]), np.array([-v * s, v * c, 0.0])


def fly_plan(plan: RendezvousPlan, mu=MU):
    """Propagate chaser + target through the plan with impulsive burns (two-body).

    Returns (r_c, v_c, r_t, v_t) at the arrival time t_burn2, after burn 2.
    """
    rc, vc = circular_state(plan.r_chaser, 0.0, mu)
    rt, vt = circular_state(plan.r_target, plan.phase0, mu)
    rc, vc = kepler_propagate(rc, vc, plan.t_wait, mu)
    rt, vt = kepler_propagate(rt, vt, plan.t_wait, mu)
    vc = vc + plan.dv1 * vc / np.linalg.norm(vc)
    rc, vc = kepler_propagate(rc, vc, plan.tof, mu)
    rt, vt = kepler_propagate(rt, vt, plan.tof, mu)
    vc = vc + plan.dv2 * vc / np.linalg.norm(vc)
    return rc, vc, rt, vt


# --------------------------------------------------------------------------- Lambert (universal variables)
def _stumpff(z):
    if z > 1e-6:
        s = math.sqrt(z)
        return (1 - math.cos(s)) / z, (s - math.sin(s)) / s ** 3
    if z < -1e-6:
        s = math.sqrt(-z)
        return (math.cosh(s) - 1) / -z, (math.sinh(s) - s) / s ** 3
    return 0.5 - z / 24 + z * z / 720, 1 / 6 - z / 120 + z * z / 5040


def lambert(r1, r2, tof, mu=MU, pole=ZPOLE, prograde=True, tol=1e-10, max_iter=200):
    """Single-revolution Lambert problem (universal variables, bisection on z -- robust, no Newton divergence).

    Returns (v1, v2). The transfer direction is prograde about `pole` (short or long way chosen from the
    geometry), or retrograde if prograde=False. Singular for a transfer angle of exactly 180 deg
    (the plane is undefined): use the Hohmann formulas there.
    """
    r1, r2 = np.asarray(r1, float), np.asarray(r2, float)
    r1n, r2n = float(np.linalg.norm(r1)), float(np.linalg.norm(r2))
    cos_dnu = float(np.dot(r1, r2)) / (r1n * r2n)
    cos_dnu = max(-1.0, min(1.0, cos_dnu))
    hz = float(np.dot(np.cross(r1, r2), pole))
    short = hz >= 0 if prograde else hz < 0
    dnu = math.acos(cos_dnu) if short else 2 * math.pi - math.acos(cos_dnu)
    A = math.sin(dnu) * math.sqrt(r1n * r2n / (1 - cos_dnu))
    if abs(A) < 1e-9:
        raise ValueError("Lambert: transfer angle 0 or 180 deg is singular")
    sm = math.sqrt(mu)

    def y_of(z):
        C, S = _stumpff(z)
        return r1n + r2n + A * (z * S - 1) / math.sqrt(C), C, S

    def tof_of(z):
        y, C, S = y_of(z)
        if y < 0:
            return -math.inf, y
        x = math.sqrt(y / C)
        return (x ** 3 * S + A * math.sqrt(y)) / sm, y

    lo, hi = -4 * math.pi ** 2, 4 * math.pi ** 2 - 1e-9
    if y_of(lo)[0] < 0:
        # A > 0: y(z) rises through 0 below z = 0 (y(0) >= 0 always); start the bracket just above that root
        a, b = lo, 0.0
        for _ in range(100):
            m = 0.5 * (a + b)
            a, b = (m, b) if y_of(m)[0] < 0 else (a, m)
        lo = b
    if tof_of(lo)[0] > tof:
        raise ValueError("Lambert: tof shorter than the hyperbolic bracket")
    z = 0.0
    for _ in range(max_iter):
        z = 0.5 * (lo + hi)
        t, _ = tof_of(z)
        if t < tof:
            lo = z
        else:
            hi = z
        if hi - lo < tol:
            break
    y, C, S = y_of(z)
    f = 1 - y / r1n
    g = A * math.sqrt(y / mu)
    gdot = 1 - y / r2n
    v1 = (r2 - f * r1) / g
    v2 = (gdot * r2 - r1) / g
    return v1, v2


def lambert_intercept(rc, vc, rt, vt, tof, aim_lvlh=(0.0, 0.0, 0.0), mu=MU, pole=ZPOLE):
    """Two-burn Lambert transfer from the chaser's current state to a point fixed in the target's LVLH
    frame (x radial, y along-track, z orbit normal) after `tof`; the second burn matches the velocity of
    that LVLH point. Returns (dv1_vec, dv2_vec, r_arrive)."""
    from .cw import lvlh_to_eci   # local import: cw imports this module
    rt2, vt2 = kepler_propagate(rt, vt, tof, mu)
    r_aim, v_aim = lvlh_to_eci(rt2, vt2, np.asarray(aim_lvlh, float), np.zeros(3))
    v1, v2 = lambert(rc, r_aim, tof, mu, pole)
    return v1 - np.asarray(vc, float), v_aim - v2, r_aim
