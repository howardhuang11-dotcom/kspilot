"""Rendezvous planner + Lambert solver (kspilot.rdv.orbit) against known cases and two-body propagation."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot.entry import kepler_propagate  # noqa: E402
from kspilot.rdv import cw, orbit as ob  # noqa: E402

R80, R100 = 680e3, 700e3


def test_constants_match_game_data():
    assert abs(ob.MU - 3.5316e12) < 1e3
    assert ob.R_KERBIN == 600000.0


def test_lambert_curtis_example_5_2():
    """Curtis, Orbital Mechanics for Engineering Students, Example 5.2 (Earth, km -> m)."""
    mu = 398600e9
    v1, v2 = ob.lambert(np.array([5000e3, 10000e3, 2100e3]), np.array([-14600e3, 2500e3, 7000e3]), 3600.0, mu)
    assert np.allclose(v1 / 1e3, [-5.9925, 1.9254, 3.2456], atol=2e-4)
    assert np.allclose(v2 / 1e3, [-3.3125, -4.1966, -0.38529], atol=2e-4)


@pytest.mark.parametrize("frac", [0.25, 0.75])
def test_lambert_circular_quarter_and_three_quarter(frac):
    """r1 -> r2 on the same circle in frac of a period must return the circular velocity (short and long way)."""
    r = R100
    T = ob.period(r)
    ang = 2 * math.pi * frac
    r1 = np.array([r, 0.0, 0.0])
    r2 = r * np.array([math.cos(ang), math.sin(ang), 0.0])
    v1, v2 = ob.lambert(r1, r2, frac * T)
    vc = ob.circ_speed(r)
    assert np.allclose(v1, [0.0, vc, 0.0], atol=1e-3)
    assert np.allclose(v2, vc * np.array([-math.sin(ang), math.cos(ang), 0.0]), atol=1e-3)


def test_lambert_round_trip_two_body():
    rng = np.random.default_rng(1)
    for _ in range(20):
        a1, a2 = rng.uniform(0, 2 * math.pi, 2)
        r1 = rng.uniform(670e3, 720e3) * np.array([math.cos(a1), math.sin(a1), rng.uniform(-0.05, 0.05)])
        r2 = rng.uniform(670e3, 720e3) * np.array([math.cos(a2), math.sin(a2), rng.uniform(-0.05, 0.05)])
        if abs(math.cos(math.acos(np.dot(r1, r2) / np.linalg.norm(r1) / np.linalg.norm(r2)))) > 0.999:
            continue
        tof = rng.uniform(600, 2500)
        try:
            v1, v2 = ob.lambert(r1, r2, tof)
        except ValueError:
            continue
        r, v = kepler_propagate(r1, v1, tof, ob.MU)
        assert np.linalg.norm(r - r2) < 1.0
        assert np.linalg.norm(v - v2) < 1e-3


def test_hohmann_numbers_match_concept_doc():
    dv1, dv2, tof = ob.hohmann(R80, R100)
    assert abs(dv1 - 16.4) < 0.1 and abs(dv2 - 16.3) < 0.1 and abs(tof - 958) < 1
    p = ob.plan_hohmann_rendezvous(R80, R100, math.radians(30))
    assert abs(math.degrees(p.catch_up_per_rev) - 15.3) < 0.1
    assert abs(p.synodic / 3600 - 12.2) < 0.05


@pytest.mark.parametrize("phase_deg", [3.0, 40.0, 200.0, 359.0])
def test_planner_closes_phase(phase_deg):
    """Flying the plan (two-body, impulsive) ends on the target's V-bar `aim_behind` behind, velocity matched."""
    p = ob.plan_hohmann_rendezvous(R80, R100, math.radians(phase_deg), aim_behind=1000.0)
    assert 0 <= p.t_wait < p.synodic
    rc, vc, rt, vt = ob.fly_plan(p)
    rho, rhod = cw.eci_to_lvlh(rt, vt, rc, vc)
    # an arc of 1000 m on the circle sits 1000^2 / (2 r) below the straight V-bar line
    assert abs(rho[1] + 1000.0) < 1.0
    assert abs(rho[0] + 1000.0 ** 2 / (2 * R100)) < 0.5
    assert abs(rho[2]) < 1e-6
    assert np.linalg.norm(rhod) < 1e-3


def test_lambert_intercept_reaches_lvlh_aimpoint():
    """Lambert targeting (non-Hohmann time of flight) to a V-bar point: arrive there with the point's velocity."""
    p = ob.plan_hohmann_rendezvous(R80, R100, math.radians(40))
    rc, vc = ob.circular_state(R80, 0.0)
    rt, vt = ob.circular_state(R100, p.phase0)
    rc, vc = kepler_propagate(rc, vc, p.t_wait, ob.MU)
    rt, vt = kepler_propagate(rt, vt, p.t_wait, ob.MU)
    tof = 0.85 * p.tof
    dv1, dv2, r_aim = ob.lambert_intercept(rc, vc, rt, vt, tof, aim_lvlh=(0.0, -1000.0, 0.0))
    r, v = kepler_propagate(rc, vc + dv1, tof, ob.MU)
    assert np.linalg.norm(r - r_aim) < 1.0
    rt2, vt2 = kepler_propagate(rt, vt, tof, ob.MU)
    rho, rhod = cw.eci_to_lvlh(rt2, vt2, r, v + dv2)
    assert np.allclose(rho, [0, -1000, 0], atol=1.0) and np.linalg.norm(rhod) < 1e-3
    # a shorter-than-Hohmann transfer costs more than the Hohmann
    assert np.linalg.norm(dv1) + np.linalg.norm(dv2) > p.dv_total
