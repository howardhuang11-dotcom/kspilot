"""Clohessy-Wiltshire (Hill) relative motion about a circular target orbit.

LVLH frame of the target (right-handed): x = radial out (R-bar, "up"), y = along-track (V-bar, prograde),
z = orbit normal (H-bar, along the angular momentum). Equations:
    x'' = 3 n^2 x + 2 n y',   y'' = -2 n x',   z'' = -n^2 z
Valid for separations << orbit radius (km scale at 700 km radius) and a near-circular target.
"""
from __future__ import annotations

import math

import numpy as np

from .orbit import MU, R_KERBIN


def stm(n, t):
    """6x6 state-transition matrix [x y z vx vy vz] over t."""
    c, s, nt = math.cos(n * t), math.sin(n * t), n * t
    P = np.zeros((6, 6))
    P[0, 0], P[0, 3], P[0, 4] = 4 - 3 * c, s / n, 2 * (1 - c) / n
    P[1, 0], P[1, 1], P[1, 3], P[1, 4] = 6 * (s - nt), 1.0, -2 * (1 - c) / n, (4 * s - 3 * nt) / n
    P[2, 2], P[2, 5] = c, s / n
    P[3, 0], P[3, 3], P[3, 4] = 3 * n * s, c, 2 * s
    P[4, 0], P[4, 3], P[4, 4] = -6 * n * (1 - c), -2 * s, 4 * c - 3
    P[5, 2], P[5, 5] = -n * s, c
    return P


def propagate(n, r, v, t):
    x = stm(n, t) @ np.concatenate([np.asarray(r, float), np.asarray(v, float)])
    return x[:3], x[3:]


def natural_accel(n, r, v):
    """Unforced CW acceleration (gravity gradient + Coriolis + centrifugal, linearised)."""
    return np.array([3 * n * n * r[0] + 2 * n * v[1], -2 * n * v[0], -n * n * r[2]])


def required_velocity(n, r0, rf, t):
    """Velocity needed now at r0 to coast to rf in t (two-impulse targeting, first burn)."""
    P = stm(n, t)
    return np.linalg.solve(P[:3, 3:], np.asarray(rf, float) - P[:3, :3] @ np.asarray(r0, float))


def two_impulse(n, r0, v0, rf, t, vf=(0.0, 0.0, 0.0)):
    """Classic CW two-impulse transfer r0 -> rf in time t, arriving with velocity vf.

    Returns (dv1, dv2) vectors in LVLH.
    """
    v_req = required_velocity(n, r0, rf, t)
    P = stm(n, t)
    v_arr = P[3:, :3] @ np.asarray(r0, float) + P[3:, 3:] @ v_req
    return v_req - np.asarray(v0, float), np.asarray(vf, float) - v_arr


def lvlh_basis(rt, vt):
    """Rows = LVLH unit vectors (x radial, y along-track, z normal) in the inertial frame, and omega."""
    rt, vt = np.asarray(rt, float), np.asarray(vt, float)
    h = np.cross(rt, vt)
    x = rt / np.linalg.norm(rt)
    z = h / np.linalg.norm(h)
    y = np.cross(z, x)
    omega = h / np.dot(rt, rt)          # frame angular velocity (inertial)
    return np.vstack([x, y, z]), omega


def eci_to_lvlh(rt, vt, rc, vc):
    """Chaser inertial state -> (rho, rho_dot) in the target LVLH frame (rotating-frame velocity)."""
    M, w = lvlh_basis(rt, vt)
    d = np.asarray(rc, float) - np.asarray(rt, float)
    dv = np.asarray(vc, float) - np.asarray(vt, float) - np.cross(w, d)
    return M @ d, M @ dv


def lvlh_to_eci(rt, vt, rho, rho_dot):
    M, w = lvlh_basis(rt, vt)
    d = M.T @ np.asarray(rho, float)
    return np.asarray(rt, float) + d, np.asarray(vt, float) + M.T @ np.asarray(rho_dot, float) + np.cross(w, d)


def mean_motion_at(alt, mu=MU, radius=R_KERBIN):
    return math.sqrt(mu / (radius + alt) ** 3)
