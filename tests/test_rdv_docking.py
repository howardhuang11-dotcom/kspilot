"""Terminal docking guidance + plant (kspilot.rdv.docking) and the sortie budget (kspilot.rdv.budget)."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot.rdv import budget as bd, cw, docking as dk, orbit as ob  # noqa: E402


def test_capture_criteria_are_the_stated_assumptions():
    assert (dk.CAPTURE_AXIAL_MAX, dk.CAPTURE_LATERAL_MAX, dk.CAPTURE_ANGLE_MAX) == (0.3, 0.1, 5.0)


@pytest.mark.parametrize("approach", ["vbar", "hbar"])
def test_nominal_undisturbed_docks(approach):
    r = dk.simulate(dk.DockParams(approach=approach))
    assert r.success, r
    assert 0.1 <= r.v_axial <= 0.2 and r.lateral < 0.05 and r.angle_deg < 1.0
    assert r.mono_kg < 80 and r.t < 2000


def test_pulse_modulator_mean_matches_request():
    resid = np.zeros(3)
    req = np.array([0.013, -0.004, 0.2])
    total = np.zeros(3)
    for _ in range(1000):
        out, resid = dk._modulate(req.copy(), resid, 0.05, False)
        total += out
        assert np.all((out == 0) | (np.abs(out) >= 0.05 - 1e-12))
    assert np.allclose(total / 1000, req, atol=0.05 / 1000 + 1e-9)


def test_dispersed_sample_docks():
    rng = np.random.default_rng(42)
    res = [dk.simulate(dk.sample(rng)) for _ in range(6)]
    assert all(r.success for r in res), [r.mode for r in res]


def test_transfer_guidance_on_a_two_body_plant():
    """The CW velocity-to-be-gained steering (TRANSFER) on an exact two-body plant (no CW in the truth):
    still delivers the chaser to the 50 m AIP; dv close to the CW-plant figure."""
    n = ob.mean_motion(700e3)
    rt, vt = ob.circular_state(700e3, 0.0)
    rc, vc = cw.lvlh_to_eci(rt, vt, (0.0, -1000.0, 0.0), (0.0, 0.0, 0.0))
    goal = np.array([0.0, -63.0, 0.0])
    T, h, dv, tau = 900.0, 0.5, 0.0, 4.0

    def acc2b(r):
        return -ob.MU * r / np.linalg.norm(r) ** 3

    t = 0.0
    while T - t > 30.0:
        rho, rhod = cw.eci_to_lvlh(rt, vt, rc, vc)
        dvL = cw.required_velocity(n, rho, goal, T - t) - rhod
        aL = np.where(np.abs(dvL) > 0.009, dvL / tau, 0.0)
        aL = np.clip(aL, -0.085, 0.085)
        M, _ = cw.lvlh_basis(rt, vt)
        a_th = M.T @ aL
        dv += np.linalg.norm(aL) * h
        for _ in range(1):     # RK2 on both bodies
            kc, kt = acc2b(rc) + a_th, acc2b(rt)
            rcm, vcm = rc + vc * h / 2, vc + kc * h / 2
            rtm, vtm = rt + vt * h / 2, vt + kt * h / 2
            rc, vc = rc + vcm * h, vc + (acc2b(rcm) + a_th) * h
            rt, vt = rt + vtm * h, vt + acc2b(rtm) * h
        t += h
    rho, rhod = cw.eci_to_lvlh(rt, vt, rc, vc)
    # 30 s before arrival: on course to the goal within a few metres (the HOLD law takes over from here)
    pred, _ = cw.propagate(n, rho, rhod, T - t)
    assert np.linalg.norm(pred - goal) < 3.0
    assert dv < 4.0


# --------------------------------------------------------------------------- budget
def test_flown_recovery_dv_is_consistent():
    st = bd.baseline_stats()
    assert st['n'] == 11
    assert 1010 < st['dv_eff_mean'] < 1045 and st['dv_eff_std'] < 20


def test_budget_model_predicts_held_out_flights():
    """Fitted on the 100 km / 2.48 t flights only; predicts 80 / 110 km and 4.5 / 6 t flights within 25 LF."""
    for r in bd.flown_table():
        if r['payload'] > 2.5 or r['alt'] != 100:
            assert abs(bd.predict_landed_lf(r['lf0']) - r['lf1']) < 25, r


def test_sortie_core_still_lands_with_margin():
    out = bd.sortie_budget(bd.Sortie())
    assert out['landed_lf'] > 250             # flown fleet minimum today: 228 LF (6 t payload) landed fine
    assert out['mono_left_kg'] >= 0
    assert out['recovery_margin_dv'] > 500
