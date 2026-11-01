"""Clohessy-Wiltshire model (kspilot.rdv.cw) against exact two-body propagation for small separations."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot.entry import kepler_propagate  # noqa: E402
from kspilot.rdv import cw, orbit as ob  # noqa: E402

R100 = 700e3
N = ob.mean_motion(R100)


def test_stm_identity_and_composition():
    assert np.allclose(cw.stm(N, 0.0), np.eye(6))
    assert np.allclose(cw.stm(N, 700.0) @ cw.stm(N, 500.0), cw.stm(N, 1200.0), atol=1e-9)


def test_stm_satisfies_cw_equations():
    """Finite-difference derivative of the STM solution equals the CW right-hand side."""
    x0 = np.array([30.0, -400.0, 15.0, 0.2, -0.1, 0.05])
    t, h = 800.0, 1e-3
    xp, xm, x = (cw.stm(N, t + h) @ x0, cw.stm(N, t - h) @ x0, cw.stm(N, t) @ x0)
    acc = (xp[3:] - xm[3:]) / (2 * h)
    assert np.allclose(acc, cw.natural_accel(N, x[:3], x[3:]), atol=1e-8)


def test_lvlh_round_trip():
    rt, vt = ob.circular_state(R100, 0.7)
    rho, rhod = np.array([12.0, -300.0, 4.0]), np.array([0.1, 0.2, -0.05])
    rc, vc = cw.lvlh_to_eci(rt, vt, rho, rhod)
    r2, v2 = cw.eci_to_lvlh(rt, vt, rc, vc)
    assert np.allclose(r2, rho) and np.allclose(v2, rhod)


@pytest.mark.parametrize("rho0,rhod0", [
    ((20.0, -500.0, 10.0), (0.1, -0.05, 0.02)),
    ((0.0, -300.0, 0.0), (0.0, 0.05, 0.0)),
    ((-50.0, 200.0, -30.0), (-0.2, 0.1, 0.1)),
])
def test_cw_matches_two_body_small_separation(rho0, rhod0):
    """Over 1500 s (~0.26 orbit) the CW position error stays below 1.5 % of the current separation: the
    rectilinear frame puts a V-bar point 1 km out |rho|^2 / 2r = 0.7 m off the circular orbit, which drifts
    ~6 (nt - sin nt) x0 = 25 m along-track in 1500 s. Velocity likewise relative to |v| + n |rho|."""
    rt, vt = ob.circular_state(R100, 0.3)
    rc, vc = cw.lvlh_to_eci(rt, vt, rho0, rhod0)
    for t in (300.0, 900.0, 1500.0):
        rt2, vt2 = kepler_propagate(rt, vt, t, ob.MU)
        rc2, vc2 = kepler_propagate(rc, vc, t, ob.MU)
        rho_true, rhod_true = cw.eci_to_lvlh(rt2, vt2, rc2, vc2)
        rho_cw, rhod_cw = cw.propagate(N, rho0, rhod0, t)
        sep = np.linalg.norm(rho_true)
        assert np.linalg.norm(rho_cw - rho_true) < 0.015 * sep + 0.5
        assert np.linalg.norm(rhod_cw - rhod_true) < 0.01 * (np.linalg.norm(rhod_true) + N * sep) + 1e-4


def test_vbar_point_is_cw_equilibrium():
    r, v = cw.propagate(N, (0.0, -1000.0, 0.0), (0.0, 0.0, 0.0), 5000.0)
    assert np.allclose(r, [0, -1000, 0]) and np.allclose(v, 0)


def test_two_impulse_reaches_target_and_stops():
    r0, v0, rf, t = np.array([0.0, -1000.0, 0.0]), np.zeros(3), np.array([0.0, -60.0, 0.0]), 900.0
    dv1, dv2 = cw.two_impulse(N, r0, v0, rf, t)
    r, v = cw.propagate(N, r0, v0 + dv1, t)
    assert np.allclose(r, rf, atol=1e-6)
    assert np.allclose(v + dv2, 0.0, atol=1e-9)
    # open loop in two-body the same burns land within ~1 % of the 940 m transfer (linearisation +
    # rectilinear V-bar vs curved orbit); the flight guidance re-targets every tick, which removes it
    rt, vt = ob.circular_state(R100, 0.0)
    rc, vc = cw.lvlh_to_eci(rt, vt, r0, v0 + dv1)
    rt2, vt2 = kepler_propagate(rt, vt, t, ob.MU)
    rc2, vc2 = kepler_propagate(rc, vc, t, ob.MU)
    rho, _ = cw.eci_to_lvlh(rt2, vt2, rc2, vc2)
    assert np.linalg.norm(rho - rf) < 0.015 * 940
