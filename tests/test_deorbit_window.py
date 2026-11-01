"""Offline tests for the deorbit window choice (C1: never raise, best plan across revolutions) and the
closest-approach candidate search of plan_deorbit (C2). No game connection."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from kspilot import deorbit as D
from kspilot import entry as E

B = E.KERBIN
PAD = E.KSC_PAD
PERIOD = 1958.0


def fake_plan(dvn, t_ign=500.0, miss=1.0, hit=True):
    return SimpleNamespace(dv_normal=dvn, dv_retro=150.0, dv=150.0 + abs(dvn), t_ign=t_ign, t_pc=None, t_impact=t_ign + 900,
                           nodes=[(t_ign, -150.0, 0.0, 0.0)], miss=miss, cross=0.0, heading=96.0,
                           impact=SimpleNamespace(hit=hit))


def scripted(seq):
    """plan_at for choose_window: seq[k] is a normal dv (m/s) or None (= raises), indexed by revolution."""
    calls = []

    def plan_at(tb):
        k = int(round(tb / PERIOD))
        calls.append(k)
        if seq[k] is None:
            raise RuntimeError("no impact passes the target within the scan window")
        return fake_plan(seq[k])
    return plan_at, calls


# ----------------------------------------------------------------------------- choose_window (C1)
def test_first_acceptable_window_is_taken_at_once():
    plan_at, calls = scripted([None, 151.0, 4.7, 4.7, 3.0] + [1.0] * 7)
    ch = D.choose_window(plan_at, PERIOD, max_dvn=30.0, n_revs=12, log=lambda s: None)
    assert ch.status == "ok" and ch.k == 2 and ch.plan.dv_normal == 4.7
    assert ch.t_base == pytest.approx(2 * PERIOD)
    assert calls == [0, 1, 2]                    # lazy: no survey beyond the first acceptable window


def test_best_plan_across_revolutions_when_none_within_limit():
    # matrix case 100 km / 28 deg / lag 30: windows at T+3 h (-49) and T+6 h (+54), none <= 30 m/s
    seq = [None, 601.0, -49.0, -48.9, -49.1, -793.0, None, -596.0, 54.0, 54.0, 798.0, None]
    plan_at, calls = scripted(seq)
    logs = []
    ch = D.choose_window(plan_at, PERIOD, max_dvn=30.0, n_revs=12, budget_ok=lambda p: True, log=logs.append)
    assert ch.status == "warn_dvn" and ch.k == 3 and ch.plan.dv_normal == -48.9
    assert calls == list(range(12))              # the whole survey, then the cheapest window
    assert any("WARNING" in s for s in logs)
    assert [k for k, d, _ in ch.tried if d is None] == [0, 6, 11]


def test_never_raises_without_any_plan():
    plan_at, _ = scripted([None] * 12)
    ch = D.choose_window(plan_at, PERIOD, n_revs=12, log=lambda s: None)
    assert ch.status == "none" and ch.plan is None and len(ch.tried) == 12

    def broken(tb):                              # numerical failures of the Newton are plan failures too
        raise np.linalg.LinAlgError("Singular matrix")
    assert D.choose_window(broken, PERIOD, n_revs=3, log=lambda s: None).status == "none"


def test_over_budget_and_unconverged_plans():
    plan_at, _ = scripted([400.0, 300.0, 350.0])
    ch = D.choose_window(plan_at, PERIOD, n_revs=3, budget_ok=lambda p: abs(p.dv_normal) < 200, log=lambda s: None)
    assert ch.status == "over_budget" and ch.k == 1          # still the cheapest one, flagged for the fallback

    plans = {0: fake_plan(5.0, miss=4e4), 1: fake_plan(12.0)}  # rev 0 did not converge (40 km miss)
    ch = D.choose_window(lambda tb: plans[int(round(tb / PERIOD))], PERIOD, n_revs=2, log=lambda s: None)
    assert ch.status == "ok" and ch.k == 1


def test_accept_rejection_falls_back_to_first_window_within_limit():
    plan_at, calls = scripted([None, 20.0, 8.0, 60.0])
    seen = []
    ch = D.choose_window(plan_at, PERIOD, n_revs=4, accept=lambda p, tb: seen.append(tb) or False,
                         log=lambda s: None)
    assert ch.status == "no_accept" and ch.k == 1            # what daylight=False would have flown
    assert len(seen) == 2


def test_deorbit_budget():
    ok, avail, need = D.deorbit_budget(155.0, 21.6e3, 11.4e3, 319.3, 840.0)
    assert ok and avail == pytest.approx(2000, abs=10) and need == pytest.approx(1.08 * 995)
    assert not D.deorbit_budget(155.0, 12e3, 11.4e3, 319.3, 840.0)[0]


# ----------------------------------------------------------------------------- Mission.choose_deorbit (C1)
@pytest.fixture
def mission(monkeypatch):
    M = pytest.importorskip("kspilot.mission")
    rb0, vb0 = circular_orbit_bf(100e3, 6.0, -120.0)
    sc = SimpleNamespace(ut=1000.0)
    sc.warp_to = lambda t: setattr(sc, "ut", max(sc.ut, t))
    logs = []
    m = M.Mission(SimpleNamespace(space_center=sc), 100e3, 100e3, 6.0, log=logs.append)
    m.entry_burns = lambda v: ()
    v = SimpleNamespace(orbit=SimpleNamespace(body=SimpleNamespace(reference_frame="rf"), period=PERIOD),
                        mass=21.6e3, dry_mass=11.4e3,     # a huge battery: power is not what these tests are about
                        resources=SimpleNamespace(amount=lambda r: 1e6, max=lambda r: 1e6))
    m.power_log = None
    monkeypatch.setattr(M, "orbit_state", lambda sc_, v_, rf: D.propagate_bf(rb0, vb0, sc.ut - 1000.0))
    veh = D.Vehicle(21.6e3, 650e3, 320.0, cda=3.0)
    return SimpleNamespace(M=M, m=m, v=v, sc=sc, logs=logs, veh=veh)


def test_mission_never_raises_and_does_not_strand_the_core(mission, monkeypatch):
    def no_solution(*a, **k):
        raise RuntimeError("no impact passes the target within the scan window")
    monkeypatch.setattr(mission.M.deorbit, "plan_deorbit", no_solution)
    ut0, plan = mission.m.choose_deorbit(mission.v, mission.veh)
    assert mission.sc.ut == 1000.0                             # no revolutions wasted on a hopeless survey
    assert plan.dv_retro > 100.0 and len(plan.nodes) == 1      # plain retro burn: the core comes down
    assert plan.impact.hit
    assert any("ERROR" in s for s in mission.logs)


def test_mission_flies_cheapest_window_after_full_survey(mission, monkeypatch):
    seq = [None, 601.0, -49.0, -48.9, -49.1, -793.0, None, -596.0, 54.0, 54.0, 798.0, None]
    calls = []

    def plan_deorbit(rb, vb, veh, **kw):
        calls.append(kw.get("cross_mode", "pc"))
        k = len(calls) - 1
        if k >= len(seq):                                      # live re-plan after the warp: same window
            return fake_plan(-48.5, t_ign=PERIOD * 0.5 + 120.0)
        if seq[k] is None:
            raise RuntimeError("no impact passes the target within the scan window")
        return fake_plan(seq[k], t_ign=PERIOD * 0.5)
    monkeypatch.setattr(mission.M.deorbit, "plan_deorbit", plan_deorbit)
    ut0, plan = mission.m.choose_deorbit(mission.v, mission.veh)
    assert len(calls) == 13                                    # 12-revolution survey + live re-plan
    assert mission.sc.ut == pytest.approx(1000.0 + 3 * PERIOD - 120.0) == ut0
    assert plan.dv_normal == -48.5
    assert any("WARNING" in s for s in mission.logs)


def test_mission_over_budget_flies_along_track_only(mission, monkeypatch):
    mission.v.mass = 12.0e3                                    # ~160 m/s left: no plane change affordable
    calls = []

    def plan_deorbit(rb, vb, veh, **kw):
        calls.append(kw.get("cross_mode", "pc"))
        return fake_plan(0.0 if kw.get("cross_mode") == "none" else 80.0, t_ign=600.0)
    monkeypatch.setattr(mission.M.deorbit, "plan_deorbit", plan_deorbit)
    ut0, plan = mission.m.choose_deorbit(mission.v, mission.veh)
    assert calls[:12] == ["pc"] * 12 and calls[12] == "none"
    assert plan.dv_normal == 0.0


def test_mission_first_try_plan_is_unchanged(mission, monkeypatch):
    """Baseline: an acceptable plan on the first try is flown exactly as before C1 (same state, same call)."""
    got = []

    def plan_deorbit(rb, vb, veh, **kw):
        got.append((np.array(rb), kw))
        return fake_plan(8.0, t_ign=1500.0)
    monkeypatch.setattr(mission.M.deorbit, "plan_deorbit", plan_deorbit)
    ut0, plan = mission.m.choose_deorbit(mission.v, mission.veh)
    assert len(got) == 1 and ut0 == 1000.0 and mission.sc.ut == 1000.0 and plan.dv_normal == 8.0
    assert set(got[0][1]) == {"target", "aim_bias", "pe_alt", "entry_burns"}


# ----------------------------------------------------------------------------- plan_deorbit (C2)
def circular_orbit_bf(alt, inc_deg, lon0, arg_deg=0.0):
    """Body-fixed kRPC state on a circular orbit, ascending node at (0, lon0) at t=0."""
    R, V = B.radius + alt, math.sqrt(B.mu / (B.radius + alt))
    e, n, u = E.enu_basis(0.0, lon0)
    h = math.cos(math.radians(inc_deg)) * e + math.sin(math.radians(inc_deg)) * n
    a = math.radians(arg_deg)
    r, v = R * (math.cos(a) * u + math.sin(a) * h), V * (-math.sin(a) * u + math.cos(a) * h)
    return E.inertial_to_bf(r, v, 0.0)


def ksc_orbit_bf(alt, inc, t, lag_deg=10.0):
    """RE-Test matrix geometry: orbit launched north-east from the pad, state t s after liftoff."""
    R = B.radius + alt
    i = math.radians(max(abs(inc), abs(PAD[0])))
    beta = math.asin(max(-1, min(1, math.cos(i) / math.cos(math.radians(PAD[0])))))
    e, n, u = E.enu_basis(*PAD)
    r0, v0 = R * u, math.sqrt(B.mu / R) * (math.sin(beta) * e + math.cos(beta) * n)
    P = 2 * math.pi * math.sqrt(R ** 3 / B.mu)
    r, v = E.kepler_propagate(r0, v0, t - lag_deg / 360.0 * P, B.mu)
    return E.inertial_to_bf(r, v, t)


def test_far_pass_gets_a_closest_approach_plan(monkeypatch):
    """100 km / 6 deg, 1100 s after liftoff: every pass in the 3-orbit scan is > 50 km from the pad, so the
    along-track error never changes sign (pre-C2: RuntimeError). Now: a converged plan on the nearest pass."""
    rb, vb = ksc_orbit_bf(100e3, 6.0, 1100.0)
    veh = D.Vehicle(21.6e3, 650e3, 320.0, cda=3.0)
    used = []
    real = D._closest_approach_candidates
    monkeypatch.setattr(D, "_closest_approach_candidates", lambda *a: used.append(real(*a)) or used[-1])
    eb = (E.Burn(0.0, 650e3, 320.0, mode="air_retro", dv=590.0, alt_trigger=45e3),)   # as Mission.entry_burns
    plan = D.plan_deorbit(rb, vb, veh, pe_alt=-60e3, entry_burns=eb)
    assert used and used[0]                                   # the fallback ran and found the passes
    assert plan.impact.hit and plan.miss < 50.0
    assert abs(plan.dv_normal) > 100.0                        # a far pass: expensive, C1 will look for a better one
    imp = E.predict(rb, vb, veh.mass, cda=3.0, burns=plan.burns)
    assert E.great_circle(imp.lat, imp.lon, *PAD) < 1000.0


def test_propagate_bf_matches_kepler():
    rb, vb = circular_orbit_bf(100e3, 28.0, -100.0)
    r1, v1 = D.propagate_bf(rb, vb, 1234.5)
    ri, vi = E.bf_to_inertial(rb, vb, 0.0)
    r2, v2 = E.inertial_to_bf(*E.kepler_propagate(ri, vi, 1234.5, B.mu), 1234.5)
    assert np.allclose(r1, r2) and np.allclose(v1, v2)
    assert np.linalg.norm(r1) == pytest.approx(B.radius + 100e3, rel=1e-9)
