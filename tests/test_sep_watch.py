"""Separation watch (DR-007, mishap inc45 2026-10-21): KSP.log explosion counter, flow angles, core-loss check.
Offline: no game connection. KSP.log excerpts are copies of the game log (lines 8878-8895: inc45, core tank hit;
lines 8045-8055: h=59.8 pass). Replayed telemetry: logs/wd_20261021_043543 / 045413 / 050841 / 052801 (passes)."""
import math
import os
from types import SimpleNamespace

import numpy as np
import pytest

from kspilot import ascent_own as A

INC45 = """[LOG 05:47:42.262] [kRPC] 10/21/2026 05:47:42 - Info - TCPServer: client disconnected (127.0.0.1)
[LOG 05:47:42.263] [kRPC] 10/21/2026 05:47:42 - Info - TCPServer: client disconnected (127.0.0.1)
[LOG 05:47:48.128] [AutoLab] screenshot -> <repo>/screenshots/20261021_054747_ascent_stage3_before.png
[LOG 05:47:48.891] Rockomax64.BW Exploded!! - blast awesomeness: 0.5
[LOG 05:47:49.025] solidBooster1-1 Exploded!! - blast awesomeness: 0.5
[LOG 05:47:49.062] noseCone Exploded!! - blast awesomeness: 0.5
[LOG 05:47:49.065] [AutoLab] perf fps=43.8 frame_ms avg=22.8 max=69.2 game/real=0.99 ut/real=0.99 warp=1 timeScale=1.00 maxDT=0.040 fixedDT=0.020 captureFramerate=0 rec=off hud_ms avg=1.16 max=1.47 gl_batches=17 gl_verts=3357
[LOG 05:47:49.286] Part engineLargeSkipper.v2 exited collision with pistonbaseCollider, but it wasn't in collision count list!
[LOG 05:47:49.286] Part engineLargeSkipper.v2 exited collision with pistonbaseCollider, but it wasn't in collision count list!
[LOG 05:47:49.302] Part engineLargeSkipper.v2 exited collision with pistonbaseCollider, but it wasn't in collision count list!
[LOG 05:47:49.319] Part engineLargeSkipper.v2 exited collision with pistonbaseCollider, but it wasn't in collision count list!
[LOG 05:47:49.854] noseCone Exploded!! - blast awesomeness: 0.5
[LOG 05:47:49.855] solidBooster1-1 Exploded!! - blast awesomeness: 0.5
[LOG 05:47:50.150] [AutoLab] screenshot -> <repo>/screenshots/20261021_054749_ascent_stage3_after.png
[LOG 05:47:52.270] [kRPC] 10/21/2026 05:47:52 - Info - Auto-accepting client connection (127.0.0.1)
[LOG 05:47:52.270] [kRPC] 10/21/2026 05:47:52 - Info - ProtocolBuffers: client connection accepted (127.0.0.1)
[LOG 05:47:52.342] [kRPC] 10/21/2026 05:47:52 - Info - TCPServer: client disconnected (127.0.0.1)
[LOG 05:47:52.345] [kRPC] 10/21/2026 05:47:52 - Info - TCPServer: client disconnected (127.0.0.1)
"""
PASS598 = """[LOG 05:29:01.942] [kRPC] 10/21/2026 05:29:01 - Info - ProtocolBuffers: client connection accepted (127.0.0.1)
[LOG 05:29:02.016] [kRPC] 10/21/2026 05:29:02 - Info - TCPServer: client disconnected (127.0.0.1)
[LOG 05:29:02.017] [kRPC] 10/21/2026 05:29:02 - Info - TCPServer: client disconnected (127.0.0.1)
[LOG 05:29:07.887] [AutoLab] screenshot -> <repo>/screenshots/20261021_052907_ascent_stage3_before.png
[LOG 05:29:08.747] [AutoLab] perf fps=41.1 frame_ms avg=24.3 max=58.8 game/real=1.00 ut/real=1.00 warp=1 timeScale=1.00 maxDT=0.040 fixedDT=0.020 captureFramerate=0 rec=off hud_ms avg=1.17 max=1.31 gl_batches=17 gl_verts=3349
[LOG 05:29:09.591] solidBooster1-1 Exploded!! - blast awesomeness: 0.5
[LOG 05:29:09.688] noseCone Exploded!! - blast awesomeness: 0.5
[LOG 05:29:09.908] [AutoLab] screenshot -> <repo>/screenshots/20261021_052909_ascent_stage3_after.png
[LOG 05:29:12.052] [kRPC] 10/21/2026 05:29:12 - Info - Auto-accepting client connection (127.0.0.1)
[LOG 05:29:12.052] [kRPC] 10/21/2026 05:29:12 - Info - ProtocolBuffers: client connection accepted (127.0.0.1)
[LOG 05:29:12.132] [kRPC] 10/21/2026 05:29:12 - Info - TCPServer: client disconnected (127.0.0.1)
"""


def _log(tmp_path, *chunks):
    p = tmp_path / "KSP.log"
    p.write_text("".join(chunks))
    return str(p)


def test_explosions_after_offset(tmp_path):
    path = _log(tmp_path, PASS598, INC45)
    allx = A.log_explosions(path, 0)
    assert allx == ["solidBooster1-1", "noseCone", "Rockomax64.BW", "solidBooster1-1", "noseCone", "noseCone",
                    "solidBooster1-1"]
    inc = A.log_explosions(path, len(PASS598.encode()))             # offset noted at the inc45 staging
    assert inc == allx[2:]
    assert A.core_hits(inc) == ["Rockomax64.BW"]
    assert A.core_hits(allx[:2]) == []                              # the h=59.8 pass: booster parts only


def test_explosions_restarted_game_and_missing_log(tmp_path):
    path = _log(tmp_path, PASS598)
    assert A.log_explosions(path, 10 ** 9) == ["solidBooster1-1", "noseCone"]   # log shorter than the offset: from 0
    assert A.log_explosions(str(tmp_path / "none.log"), 0) == []


def test_core_hits_names():
    assert A.core_hits(["solidBooster1-1", "noseCone", "radialDecoupler", "landingLeg1-2", "engineLargeSkipper.v2"]) \
        == ["landingLeg1-2", "engineLargeSkipper.v2"]


def test_flow_angles_axes():
    assert A.flow_angles((0, 100, 0))[0] == pytest.approx(0.0)
    c4, s4 = math.cos(math.radians(4)), math.sin(math.radians(4))
    assert A.flow_angles((0, 100 * c4, 100 * s4)) == pytest.approx((4.0, 4.0, 0.0, 0.0))     # flow off the bottom side
    assert A.flow_angles((100 * s4, 100 * c4, 0)) == pytest.approx((4.0, 0.0, 4.0, 90.0))    # off the right side


@pytest.mark.parametrize("roll_deg", [0.0, 41.7, 90.0, 200.0, 315.0])
def test_flow_angles_total_aoa_independent_of_roll(roll_deg):
    """Velocity 4 deg off the nose, the vessel rolled about its long axis (y): the total AoA stays 4 deg while the
    pitch-plane component (what kRPC's angle_of_attack reports) goes like atan(tan 4 cos roll); the crossflow
    azimuth turns with the roll."""
    r = math.radians(-roll_deg)
    rot_y = np.array([[math.cos(r), 0, math.sin(r)], [0, 1, 0], [-math.sin(r), 0, math.cos(r)]])
    v = np.array([0.0, math.cos(math.radians(4)), math.sin(math.radians(4))]) * 450.0
    aoa, alpha, beta, roll = A.flow_angles(tuple(rot_y @ v))
    assert aoa == pytest.approx(4.0)
    assert alpha == pytest.approx(math.degrees(math.atan(math.tan(math.radians(4)) * math.cos(math.radians(roll_deg)))))
    assert math.sin(math.radians(beta)) == pytest.approx(-math.sin(math.radians(roll_deg)) * math.sin(math.radians(4)))
    assert math.cos(math.radians(roll + roll_deg)) == pytest.approx(1.0)


def test_booster_offset_x_layout():
    x = [45.0, 135.0, 225.0, 315.0]                     # VAB X layout of the four SRBs
    assert A.booster_offset(41.7, x) == pytest.approx(3.3)      # inc45: a booster ~in the crossflow plane
    assert A.booster_offset(90.0, x) == pytest.approx(45.0)     # h=90: the flow passes between the boosters
    assert A.booster_offset(0.0, x) == pytest.approx(45.0)
    assert A.booster_offset(225.0, x) == pytest.approx(0.0)
    assert A.booster_offset(-100.0, [-45.0, 45.0, 135.0, -135.0]) == pytest.approx(35.0)
    assert math.isnan(A.booster_offset(10.0, []))


# --- core-loss check (Ascent._core_lost) on a fake vessel and a fake clock -----------------------------------------

class Clock:
    t = 1000.0

    def __call__(self):
        return self.t


def _ascent(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(A.time, "time", clock)
    a = A.Ascent.__new__(A.Ascent)
    a.sep, a.lost, a.lines = None, None, []
    a.log = a.lines.append
    return a, clock


def _vessel(thr_kn, avail_kn, throttle, parts):
    return SimpleNamespace(thrust=thr_kn * 1e3, available_thrust=avail_kn * 1e3,
                           control=SimpleNamespace(throttle=throttle), parts=SimpleNamespace(all=[None] * parts))


def _stage(a, clock, thr0_kn, parts0, dropped=12):
    """What _stage_if_needed leaves behind for a staging that lights nothing."""
    a.sep = dict(t=clock.t, thr0=thr0_kn * 1e3, parts0=parts0, expected=parts0 - dropped, parts=None, metric=None)
    a._thr_ref, a._t_weak = 0.0, None


def _tel(path):
    """[(t, phase, Ap km, stage, parts, thr kN)] from the 'ascent tel' / staging lines of a watchdog log; target Ap."""
    rows, target = [], None
    for line in open(path):
        t = float(line[1:line.index("s]")]) if line.startswith("[") and "s]" in line else None
        if "launch heading" in line:
            target = float(line.split(" Ap ")[1].split()[0])
        elif "ascent tel:" in line:
            w = line.split()
            rows.append((t, w[w.index("tel:") + 1], float(w[w.index("Ap") + 1]), int(w[w.index("stage") + 1]),
                         int(w[w.index("parts") + 1]), float(w[w.index("thr") + 1])))
        elif "ascent: staging" in line:
            rows.append((t, "STAGING", 0.0, 0, 0, 0.0))
    return rows, target


import flightdata  # noqa: E402  (bundled, sanitised watchdog logs in tests/data; KSP_REPLAY_LOGS for full logs)

LOG_DIRS = flightdata.log_dirs() + [flightdata.DATA]
LOGS = ["wd_20261021_043543.out", "wd_20261021_045413.out", "wd_20261021_050841.out", "wd_20261021_052801.out"]


@pytest.mark.parametrize("name", LOGS)
@pytest.mark.parametrize("dry_srb_counted", [False, True])    # kRPC available thrust of a dry SRB: 0, or still counted
def test_no_false_abort_on_passing_flights(monkeypatch, name, dry_srb_counted):
    path = next((os.path.join(d, name) for d in LOG_DIRS if os.path.exists(os.path.join(d, name))), None)
    if path is None:
        pytest.skip("watchdog flight log not available (tests/data or KSP_REPLAY_LOGS)")
    rows, target = _tel(path)
    a, clock = _ascent(monkeypatch)
    srb_peak, last_thr, staged, n = 0.0, 0.0, False, 0
    for t, phase, ap_km, stage, parts, thr in rows:
        if phase == "STAGING":
            clock.t = 1000.0 + t
            _stage(a, clock, last_thr, 49)
            staged = True
            continue
        throttle = 1.0 if phase != "apoapsis" else max(0.05, min(1.0, (target - ap_km) * 1e3 / 5000.0))
        if not staged:
            srb_peak = max(srb_peak, thr)
            avail = srb_peak if dry_srb_counted else thr
        else:
            avail = 652.0                     # the Skipper alone (<= 650 kN seen at full throttle)
        for dt in (0.0, 1.0, 1.9):            # one telemetry line per ~2 s of loop iterations
            clock.t = 1000.0 + t + dt
            assert a._core_lost(_vessel(thr, avail, throttle, parts)) is None, (name, t, phase, thr, throttle)
            n += 1
        last_thr = thr
    assert staged and target and n > 100
    assert a.sep["parts"] == 37 and a.sep["thr0"] == pytest.approx(640e3) and a.sep.get("done")
    assert not any("ANOMALY" in line for line in a.lines)
    assert any(r[1] == "apoapsis" for r in rows)      # the throttled-down phase was replayed too


def test_inc45_core_loss_aborts(monkeypatch):
    """wd_20261021_054641: staging at 640 kN / 49 parts; first look 31 parts, 0 kN -> abort 0.5 s later."""
    a, clock = _ascent(monkeypatch)
    _stage(a, clock, 640.0, 49)
    clock.t += 2.0
    assert a._core_lost(_vessel(0.0, 0.0, 1.0, 31)) is None          # first look: low thrust starts the 0.5 s timer
    assert any("ANOMALY" in line for line in a.lines)
    clock.t += 0.6
    assert a._core_lost(_vessel(0.0, 0.0, 1.0, 31)) == "lost core at staging (parts 31, thrust 0 kN)"


def test_part_loss_after_first_look_aborts(monkeypatch):
    a, clock = _ascent(monkeypatch)
    _stage(a, clock, 640.0, 49)
    clock.t += 2.0
    assert a._core_lost(_vessel(642.0, 652.0, 1.0, 37)) is None
    clock.t += 1.0
    assert a._core_lost(_vessel(642.0, 652.0, 1.0, 33)) == "lost core at staging (parts 33, thrust 642 kN)"


def test_generic_thrust_collapse(monkeypatch):
    """Engine lost later (no staging): throttle up, thrust < 10 % of the available thrust seen, for 2 s."""
    a, clock = _ascent(monkeypatch)
    _stage(a, clock, 640.0, 49)
    for _ in range(8):                                   # healthy, past the 5 s post-staging window
        clock.t += 1.0
        assert a._core_lost(_vessel(645.0, 652.0, 1.0, 37)) is None
    for _ in range(2):
        clock.t += 1.0
        assert a._core_lost(_vessel(0.0, 0.0, 1.0, 33)) is None
    clock.t += 1.1
    assert a._core_lost(_vessel(0.0, 0.0, 1.0, 33)).startswith("lost core: thrust collapse (parts 33, thrust 0 kN")
    b, clock = _ascent(monkeypatch)                     # throttled down to 5 % (apoapsis phase), then MECO: no abort
    for _ in range(5):
        clock.t += 1.0
        assert b._core_lost(_vessel(33.0, 660.0, 0.05, 37)) is None
    clock.t += 5.0
    assert b._core_lost(_vessel(0.0, 660.0, 0.0, 37)) is None


def test_sep_record_and_metric(monkeypatch, tmp_path):
    """_sep_record on a fake vessel at the inc45 geometry (flow 4 deg off the nose at azimuth 41.7 deg, boosters on
    the X diagonals), then the metric line once KSP.log has grown after the staging."""
    from kspilot import core
    monkeypatch.setattr(core, "KSP_DIR", str(tmp_path))
    (tmp_path / "KSP.log").write_text(PASS598)
    a, clock = _ascent(monkeypatch)
    phi, aoa = math.radians(41.7), math.radians(4.0)
    vel = (450 * math.sin(aoa) * math.sin(phi), 450 * math.cos(aoa), 450 * math.sin(aoa) * math.cos(phi))
    a.sc = SimpleNamespace(transform_direction=lambda d, f, t: d)
    a.body = SimpleNamespace(reference_frame=object())
    boosters = [SimpleNamespace(position=lambda rf, az=az: (2.0 * math.sin(math.radians(az)), 5.0,
                                                           2.0 * math.cos(math.radians(az)))) for az in (45, 135, 225, 315)]
    v = SimpleNamespace(thrust=640e3, parts=SimpleNamespace(all=[None] * 49), reference_frame=object(),
                        flight=lambda rf=None: SimpleNamespace(dynamic_pressure=19900.0, velocity=vel))
    a.sep = a._sep_record(v, set(range(12)), boosters)
    s = a.sep
    assert (s["expected"], s["q"], s["off"]) == (37, 19900.0, len(PASS598.encode()))
    assert (s["aoa"], s["roll"], s["offset"]) == pytest.approx((4.0, 41.7, 3.3))
    with open(tmp_path / "KSP.log", "a") as f:
        f.write(INC45)
    a._sep_metric()
    line = a.lines[-1]
    assert line.startswith("sep metric: q 19.9 kPa roll 41.7 deg sideslip 2.7 deg aoa 4.0 deg explosions 5 [")
    assert "solidBooster1-1 x2" in line and "CORE HIT: Rockomax64.BW" in line and "booster offset 3.3 deg" in line
