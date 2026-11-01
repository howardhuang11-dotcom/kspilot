"""Mishap 2026-10-21 (HOP_REC=full from g5_orbit): deorbit burn never happened ("could not align", "no progress").
Root cause: the daylight wait kept the core ~5.5 h in orbit; its battery went 999.7 -> 0 EC (autosave at UT 169486:
reaction wheel "not enough power"), so no attitude control and no throttle. rec.pause only correlated (dead air is
exactly when the vessel aligns). These checks would have caught it offline (no KSP needed)."""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot import director as d  # noqa: E402
from kspilot.mission import EC_KEEP, EC_RATE_PRIOR, daylight_wait_ok  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
T0 = 1000.0

# measured in the mishap run (saves: g5_orbit UT 149591 EC 999.74/1000.2; persistent UT 169486 EC 0.0) and in the
# last non-recorded pass (g5_orbit 149591 -> g5_entry 159858: 999.7 -> 485.6 EC): ~0.05 EC/s in orbit
RATE = (999.74 - 485.56) / (159858.45 - 149591.28)
PERIOD, T_REST = 1960.0, 4084.0 + 900.0        # one revolution; deorbit ignition + fall to the pad


def ec_at(ut):
    return max(999.74 - RATE * (ut - 149591.3), 0.0)


def test_mishap_daylight_wait_would_have_stopped():
    """Replay the run's daylight decisions: waiting must stop while the battery can still fly the landing."""
    waits, ut, ut0 = 0, 155557.0, 155557.0          # first daylight decision of the mishap run
    while waits < 16:
        rate = (ec_at(ut0) - ec_at(ut)) / (ut - ut0) if ut - ut0 > 600 else EC_RATE_PRIOR
        if not daylight_wait_ok(ec_at(ut), 1000.2, rate, PERIOD + T_REST):
            break
        waits += 1
        ut += PERIOD
    assert waits < 5, waits                          # the mishap waited 5 revolutions
    assert ec_at(ut + T_REST) >= EC_KEEP * 1000.2 - 1.0, ec_at(ut + T_REST)   # power left at touchdown


def test_daylight_wait_rule():
    assert daylight_wait_ok(1000.0, 1000.0, 0.05, 2000.0)
    assert not daylight_wait_ok(350.0, 1000.0, 0.05, 2000.0)
    assert daylight_wait_ok(0.0, 0.0, 0.05, 1e6)              # no battery: nothing to protect
    assert not daylight_wait_ok(float("nan"), 1000.0, 0.0, 1.0)   # unreadable power: do not wait


def test_pause_only_stops_frames():
    """AutoLab Recorder: the capture rate (game-time step per frame) is set at rec.req / rec.stop only; a pause or
    resume must never change it (nor timeScale / physics)."""
    src = open(os.path.join(ROOT, "plugins", "AutoLab", "AutoLab.cs")).read()
    rec = src[src.index("public class Recorder"):src.index("System.Collections.IEnumerator Grab()")]
    pause = rec[rec.index("bool p = dir != null"):]
    pause = pause[:pause.index("catch")]
    assert not re.search(r"captureFramerate|timeScale|fixedDeltaTime|TimeWarp", pause), pause
    # start and stop only -- plus the v2 physics-warp compensation (spec C1-B1), which runs only for a "v2" request
    # (HOP_REC=full), follows timeScale alone and never looks at the pause
    comp = rec[rec.index("static void Compensate()"):rec.index("void Update()")]
    assert len(re.findall(r"Time\.captureFramerate\s*=", rec.replace(comp, ""))) == 2
    assert len(re.findall(r"Time\.captureFramerate\s*=", comp)) == 1 and "paused" not in comp
    assert "if (dir != null && v2) Compensate();" in rec


def _dead(ut, guard):
    return d.State(ut, ut - T0, {"LIFTOFF": (T0, True), "MECO": (T0 + 200, True)}, 9e4, False, False, 0,
                   True, 4e5, 30.0, 67.0, guard)


def test_never_paused_while_aligning_or_burning():
    c = d.Cutter()
    acts = []
    for k in range(200):                             # 50 s of dead air while the autopilot aligns for a burn
        acts += c.step(_dead(T0 + 300 + 0.25 * k, True), [])
    assert ("pause",) not in acts and not c.paused
    for k in range(40):                              # real dead air: paused after REC_TAIL
        c.step(_dead(T0 + 400 + 0.25 * k, False), [])
    assert c.paused
    assert ("resume",) in c.step(_dead(T0 + 411, True), [])   # alignment starts: capture is back at once
    assert not c.paused


def test_guard_window():
    assert d.guard_window(1000.0, [1100.0], {})                         # node in 100 s
    assert not d.guard_window(1000.0, [1300.0], {})
    assert d.guard_window(1000.0, [], {"DEORBIT BURN": (1140.0, False)})
    assert not d.guard_window(1000.0, [], {"DEORBIT BURN": (1140.0, True)})


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
