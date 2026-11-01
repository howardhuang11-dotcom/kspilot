"""Core power during the long coasts after payload release (kspilot.power; mishap 2026-10-21: the core's battery
ran flat in a 5.5 h wait -> no attitude, no throttle). Offline: no game connection.

Flown numbers (T-WorksFineGF core, HECS2 probe core, 1000 EC): g5_orbit UT 149591.28 EC 999.74/1000.2 ->
g5_entry UT 159858.45 EC 485.56 (awake, coasting / warping). Hibernation drain: stock cfg only (not flown)."""
import math
from types import SimpleNamespace

import pytest

from kspilot import deorbit as D
from kspilot import power as P

RATE_FLOWN = (999.74 - 485.56) / (159858.45 - 149591.28)     # 0.0501 EC/s awake
EC_MAX = 1000.2
EC_DECISION = 999.74 - RATE_FLOWN * 600.0                     # after the ~10 min drag probes that follow release
HIB_CFG = 0.05 * 0.002                                        # HECS2.cfg: rate 0.05, hibernationMultiplier 0.002
PERIOD = 1958.0
WAIT_90 = 5.3 * 3600.0                                        # 90 deg: second window, burn 5.3 h away
AFTER = 1500.0 + P.DESCENT_MARGIN                             # burn -> touchdown (+ landing margin)


def test_flown_awake_drain_is_the_probe_core():
    assert RATE_FLOWN == pytest.approx(0.05, rel=0.02)        # HECS2 ModuleCommand 0.05 EC/s: wheels idle draw ~0
    assert P.EC_RATE_AWAKE >= RATE_FLOWN and P.EC_RATE_HIB >= 50 * HIB_CFG


def test_replay_90deg_second_window_ok_only_with_hibernation():
    with_hib = P.budget(EC_DECISION, EC_MAX, WAIT_90, AFTER, hibernate=True)
    without = P.budget(EC_DECISION, EC_MAX, WAIT_90, AFTER, hibernate=False)
    assert with_hib.ok and with_hib.ec_touchdown > 0.6 * EC_MAX, with_hib.text()
    assert not without.ok and without.ec_touchdown < 0, without.text()
    # the rejection is right: at the flown awake drain ~1.5 % is left at the burn and the battery is flat before
    # touchdown (the mishap again) ...
    assert EC_DECISION - RATE_FLOWN * WAIT_90 < 0.02 * EC_MAX
    assert EC_DECISION / RATE_FLOWN < WAIT_90 + AFTER
    # ... and hibernating at the cfg rate it would keep ~85 %
    truth = EC_DECISION - HIB_CFG * (WAIT_90 - P.WAKE_LEAD) - RATE_FLOWN * (P.WAKE_LEAD + AFTER)
    assert truth > 0.85 * EC_MAX
    # margin on the unmeasured number: still OK if hibernation drained 250x the cfg value (half the awake rate)
    assert P.budget(EC_DECISION, EC_MAX, WAIT_90, AFTER, True, rate_hib=0.025).ok


def test_budget_edge_cases():
    assert P.budget(1000.0, 1000.0, 0.0, 1800.0, False).ok                 # come down now: fine
    assert not P.budget(350.0, 1000.0, 1000.0, 1800.0, True).ok            # already low: no waiting
    assert P.budget(0.0, 0.0, 1e6, 1e3, False).ok                          # no battery: nothing to protect
    assert not P.budget(float("nan"), float("nan"), 0.0, 1.0, True).ok     # unreadable: no optional wait
    b = P.budget(900.0, 1000.0, 1000.0, 500.0, True)
    assert b.t_hib == 1000.0 - P.WAKE_LEAD and b.t_awake == P.WAKE_LEAD + 500.0


# ----------------------------------------------------------------------------- window choice with power_ok
def fake_plan(dvn, t_ign=PERIOD * 0.5, t_flight=900.0):
    return SimpleNamespace(dv_normal=dvn, dv_retro=150.0, dv=150.0 + abs(dvn), t_ign=t_ign, t_pc=None,
                           t_impact=t_ign + t_flight, nodes=[(t_ign, -150.0, 0.0, 0.0)], miss=1.0, cross=0.0,
                           heading=96.0, impact=SimpleNamespace(hit=True))


def test_choose_window_power_stops_survey_and_never_strands():
    seq = [None, 700.0, 400.0, 5.0, 2.0]
    calls = []

    def plan_at(tb):
        k = int(round(tb / PERIOD))
        calls.append(k)
        if seq[k] is None:
            raise RuntimeError("no impact passes the target within the scan window")
        return fake_plan(seq[k])
    ch = D.choose_window(plan_at, PERIOD, 30.0, 5, power_ok=lambda p, tb: tb < 3 * PERIOD, log=lambda s: None)
    assert ch.status == "warn_dvn" and ch.k == 2 and calls == [0, 1, 2, 3]   # cheapest eligible; stopped at rev 3
    assert ch.tried[-1] == (3, 5.0, "EC")
    ch = D.choose_window(plan_at, PERIOD, 30.0, 5, power_ok=lambda p, tb: False, log=lambda s: None)
    assert ch.status == "low_power" and ch.k == 1                           # nothing eligible: the soonest plan


# ----------------------------------------------------------------------------- Mission replay (90 deg)
SEQ_90 = [None, 700.0, None, 350.0, 120.0, None, 400.0, None, 800.0, 4.0, 3.0, None]   # rev 9: burn 5.3 h away
T_IGN_90 = WAIT_90 - 9 * PERIOD


@pytest.fixture
def mission(monkeypatch):
    M = pytest.importorskip("kspilot.mission")
    sc = SimpleNamespace(ut=1000.0, warps=[])

    def warp_to(t):
        sc.warps.append((t, state["asleep"]))
        sc.ut = max(sc.ut, t)
    sc.warp_to = warp_to
    state = {"asleep": False, "sleeps": 0}
    logs = []
    m = M.Mission(SimpleNamespace(space_center=sc), 100e3, 100e3, 90.0, log=logs.append)
    m.entry_burns = lambda v: ()
    m.power_log = None
    v = SimpleNamespace(orbit=SimpleNamespace(body=SimpleNamespace(reference_frame="rf"), period=PERIOD),
                        mass=21.6e3, dry_mass=11.4e3,
                        resources=SimpleNamespace(amount=lambda r: EC_DECISION, max=lambda r: EC_MAX))
    monkeypatch.setattr(M, "orbit_state", lambda sc_, v_, rf: (None, None))
    monkeypatch.setattr(M.deorbit, "propagate_bf", lambda r, v_, t: (None, None))
    calls = []

    def plan_deorbit(rb, vb, veh, **kw):
        calls.append(sc.ut)
        k = len(calls) - 1
        if k >= len(SEQ_90) or sc.ut > 1000.0:                 # live re-plan after the coast: same window
            return fake_plan(4.2, t_ign=T_IGN_90 + 120.0, t_flight=1500.0)
        if SEQ_90[k] is None:
            raise RuntimeError("no impact passes the target within the scan window")
        return fake_plan(SEQ_90[k], t_ign=T_IGN_90, t_flight=1500.0)
    monkeypatch.setattr(M.deorbit, "plan_deorbit", plan_deorbit)

    def sleep(v_, log=print):
        state["asleep"] = True
        state["sleeps"] += 1
        return True

    def wake(v_, log=print):
        state["asleep"] = False
        return True
    monkeypatch.setattr(M.pwr, "sleep", sleep)
    monkeypatch.setattr(M.pwr, "wake", wake)
    veh = D.Vehicle(21.6e3, 650e3, 320.0, cda=3.0)
    return SimpleNamespace(M=M, m=m, v=v, sc=sc, logs=logs, veh=veh, calls=calls, state=state)


def test_mission_90deg_second_window_with_hibernation(mission, monkeypatch):
    monkeypatch.setattr(mission.M.pwr, "check_toggle", lambda v, log=print: True)
    ut0, plan = mission.m.choose_deorbit(mission.v, mission.veh)
    assert plan.dv_normal == 4.2                                         # the 4 m/s window 5.3 h away
    budget = [s for s in mission.logs if "EC budget (rev 9" in s]
    assert budget and "OK" in budget[0]
    # the coast: asleep up to WAKE_LEAD s before its end, then awake (verified) for the rest
    ut_end = 1000.0 + 9 * PERIOD - 120.0
    assert mission.sc.warps == [(ut_end - 120.0, True), (ut_end, False)]
    assert mission.state["sleeps"] == 1 and not mission.state["asleep"]
    assert ut0 == ut_end


def test_mission_90deg_second_window_rejected_without_hibernation(mission, monkeypatch):
    monkeypatch.setattr(mission.M.pwr, "check_toggle", lambda v, log=print: False)
    ut0, plan = mission.m.choose_deorbit(mission.v, mission.veh)
    assert not any("EC budget (rev 9" in s for s in mission.logs)      # survey stopped long before
    rejected = [s for s in mission.logs if "EC budget" in s and "REJECT" in s]
    assert rejected and "rev 6" in rejected[0]
    assert len(mission.calls) == 7 + 1                                   # revs 0-6, then the live re-plan
    assert any("WARNING" in s for s in mission.logs)                     # cheapest eligible: rev 4, 120 m/s normal
    assert mission.sc.warps[-1] == (pytest.approx(1000.0 + 4 * PERIOD - 120.0), False)   # rev 4, awake at the end


# ----------------------------------------------------------------------------- part state read-back (fake kRPC)
class Field:
    def __init__(self, name, value, sticky=False):
        self.name, self._v, self.sticky = name, value, sticky

    @property
    def bool_value(self):
        return self._v

    @bool_value.setter
    def bool_value(self, x):
        if not self.sticky:
            self._v = x


def fake_vessel(sticky=False):
    cmd = SimpleNamespace(name="ModuleCommand", field_list=[Field("hibernation", False),
                                                            Field("hibernateOnWarp", False, sticky)])
    wheel = SimpleNamespace(active=True, broken=False)
    ap = SimpleNamespace(engaged=True)
    ap.disengage = lambda: setattr(ap, "engaged", False)
    return SimpleNamespace(parts=SimpleNamespace(all=[SimpleNamespace(modules=[cmd])], reaction_wheels=[wheel]),
                           control=SimpleNamespace(sas=True), auto_pilot=ap,
                           resources=SimpleNamespace(amount=lambda r: 900.0, max=lambda r: 1000.0)), cmd, wheel


def test_sleep_and_wake_are_read_back(monkeypatch):
    monkeypatch.setattr(P.time, "sleep", lambda s: None)
    v, cmd, wheel = fake_vessel()
    logs = []
    assert P.can_hibernate(v)
    assert P.sleep(v, logs.append)
    assert cmd.field_list[1].bool_value and not wheel.active and not v.control.sas and not v.auto_pilot.engaged
    cmd.field_list[0]._v = True                                          # someone left the core fully hibernating
    assert P.wake(v, logs.append)
    assert not cmd.field_list[1].bool_value and not cmd.field_list[0].bool_value and wheel.active
    assert P.check_toggle(v, logs.append) and wheel.active and not cmd.field_list[1].bool_value
    assert any("EC 900/1000" in s for s in logs)


def test_stuck_field_is_reported_not_assumed(monkeypatch):
    monkeypatch.setattr(P.time, "sleep", lambda s: None)
    v, cmd, wheel = fake_vessel(sticky=True)                             # the game ignores the toggle
    logs = []
    assert not P.sleep(v, logs.append)
    assert not P.check_toggle(v, logs.append)                            # -> budget the coasts awake
    assert wheel.active                                                  # left awake
    assert not P.can_hibernate(SimpleNamespace())                        # no parts readable


def test_coast_wakes_even_if_the_warp_fails(monkeypatch):
    M = pytest.importorskip("kspilot.mission")
    monkeypatch.setattr(P.time, "sleep", lambda s: None)
    v, cmd, wheel = fake_vessel()
    sc = SimpleNamespace(ut=0.0)

    def warp_to(t):
        raise RuntimeError("kRPC connection lost")
    sc.warp_to = warp_to
    m = M.Mission(SimpleNamespace(space_center=sc), 100e3, 100e3, 0.0, log=lambda *a: None)
    m.power_log = None
    m.power_save = True                                                  # a long mission (choose_deorbit set it)
    with pytest.raises(RuntimeError):
        m.coast_to(v, 5000.0)
    assert wheel.active and not cmd.field_list[1].bool_value


def test_short_coast_stays_awake():
    M = pytest.importorskip("kspilot.mission")
    v, cmd, wheel = fake_vessel()
    sc = SimpleNamespace(ut=0.0, warps=[])
    sc.warp_to = lambda t: sc.warps.append((t, wheel.active)) or setattr(sc, "ut", t)
    m = M.Mission(SimpleNamespace(space_center=sc), 100e3, 100e3, 0.0, log=lambda *a: None)
    m.coast_to(v, 150.0)
    assert sc.warps == [(150.0, True)]
    assert math.isclose(sc.ut, 150.0)


# ----------------------------------------------------------------------------- baseline untouched (review gate)
# flown baseline from g5_orbit (logs 2026-10-21 01:31-03:53): plan made ~225 s after release, normal 8.4 m/s,
# t_ign +4100 s (the pre-C1 3-orbit scan finds it in the first plan), burn -> impact ~520 s
BASE_T_IGN, BASE_FALL, BASE_PROBES = 4100.0, 520.0, 225.0


def run_baseline(monkeypatch, t_ign=BASE_T_IGN):
    M = pytest.importorskip("kspilot.mission")
    calls = []
    for name in ("check_toggle", "sleep", "wake", "set_state"):
        monkeypatch.setattr(P, name, lambda *a, _n=name, **k: calls.append(_n) or True)
    sc = SimpleNamespace(ut=1000.0, warps=[])
    sc.warp_to = lambda t: sc.warps.append(t) or setattr(sc, "ut", max(sc.ut, t))
    logs = []
    m = M.Mission(SimpleNamespace(space_center=sc), 100e3, 100e3, 6.0, log=logs.append)
    m.power_log = None
    m.entry_burns = lambda v: ()
    m.ut_release = 1000.0 - BASE_PROBES
    v = SimpleNamespace(orbit=SimpleNamespace(body=SimpleNamespace(reference_frame="rf"), period=PERIOD),
                        mass=21.6e3, dry_mass=11.4e3,
                        resources=SimpleNamespace(amount=lambda r: 990.0, max=lambda r: EC_MAX))
    monkeypatch.setattr(M, "orbit_state", lambda sc_, v_, rf: (None, None))
    monkeypatch.setattr(M.deorbit, "plan_deorbit", lambda rb, vb, veh, **kw: fake_plan(8.4, t_ign=t_ign,
                                                                                         t_flight=BASE_FALL))
    ut0, plan = m.choose_deorbit(v, D.Vehicle(21.6e3, 650e3, 320.0, cda=3.0))
    t_node = ut0 + min(n[0] for n in plan.nodes)
    m.coast_to(v, t_node - 90, "node")                                   # as execute_nodes_guarded
    m.coast_to(v, ut0 + plan.t_impact - 400.0, "entry interface")       # as coast_and_correct
    return SimpleNamespace(m=m, sc=sc, calls=calls, logs=logs, ut0=ut0, plan=plan, t_node=t_node)


def test_baseline_100km_6deg_makes_no_power_mode_calls(monkeypatch):
    """The proven baseline (100 km / 6 deg, deorbit on the first plan, coast 1.2 h < 2 h): no hibernation
    toggle, no sleep / wake, no wheel / SAS / autopilot change - every coast is the plain warp it was before."""
    r = run_baseline(monkeypatch)
    assert r.ut0 == 1000.0 and r.plan.dv_normal == 8.4                   # first plan, as flown
    assert not r.m.power_save
    assert r.calls == []
    assert r.sc.warps == [r.t_node - 90, r.ut0 + r.plan.t_impact - 400.0]   # one plain warp each
    assert any("EC budget (rev 0" in s and "OK" in s for s in r.logs)   # the budget still runs (EC reads only)
    assert not any("power mode" in s or "power:" in s for s in r.logs)


def test_longest_flown_first_plan_stays_baseline(monkeypatch):
    """Longest first plan flown (t_ign +5849 s: coast 1.7 h) is under the 2 h gate (set 2026-10-21): exactly as
    before, no power-mode call. A 2.35 h wait (shortest 28/45 deg window) uses power mode."""
    r = run_baseline(monkeypatch, t_ign=5849.0)
    assert not r.m.power_save and r.calls == []
    assert r.sc.warps == [r.t_node - 90, r.ut0 + r.plan.t_impact - 400.0]
    r = run_baseline(monkeypatch, t_ign=2.35 * 3600 - BASE_PROBES)
    assert r.m.power_save and r.calls[0] == "check_toggle"
    assert "sleep" in r.calls and r.calls[-1] == "wake"
    monkeypatch.setenv("KSP_HIBERNATE_MIN_H", "3.0")                     # the gate is an env setting
    r = run_baseline(monkeypatch, t_ign=2.35 * 3600 - BASE_PROBES)
    assert not r.m.power_save and r.calls == []
