"""Recording pipeline v2, fix round 2 (video QA report 2026-10-21, decisions 11:06 / 11:30). Offline only:
  1  warp guard: no time warp inside a protected 1x window (window logic, live windows, game-clock wait), and the
     editor fallback (rails / gapped frames in a 1x window -> badged lapse + loud EDL CHECKS FAILED)
  2  V7 pad lens auto-zoom from the range (FOV math, the band goes to the steep chase beyond the lens' reach)
  3  V8 event shots split (S4 -> S4b, S5 -> S5b, long burns alternate)
  5  waits in game seconds while recording (GameClock, 3x wall cap), helpers identical without it
and the non-recording guarantee: with the hooks unset every helper is exactly the call it replaces."""
import json
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
from kspilot import core, director as d, showrun as sr  # noqa: E402
import director_replay as rp  # noqa: E402
import edit_video as ed  # noqa: E402


@pytest.fixture(autouse=True)
def no_hooks():
    """Every test starts and ends with the recording hooks unset (the non-recording state)."""
    core.GAME_CLOCK = core.WARP_GUARD = None
    yield
    core.GAME_CLOCK = core.WARP_GUARD = None


class Clock:
    """Fake wall clock + game UT: the game runs at `rate` x real time (0.4 while recording, 0 = paused)."""

    def __init__(self, ut=1000.0, rate=0.4):
        self.w, self.u, self.rate = 0.0, ut, rate

    def wall(self):
        return self.w

    def ut(self):
        return self.u

    def sleep(self, dt):
        self.w += dt
        self.u += dt * self.rate


# ------------------------------------------------------------------------------ 1. warp guard (flight side)
def test_warp_hold_until_window_logic():
    w = [(100.0, 110.0)]
    assert sr.warp_hold_until(105.0, w) == pytest.approx(110.0 + sr.WARP_PAD)
    assert sr.warp_hold_until(111.0, w) == pytest.approx(112.0)              # still inside the pad
    assert sr.warp_hold_until(112.1, w) is None
    assert sr.warp_hold_until(99.0, w) is None                               # a window not yet open never holds
    assert sr.warp_hold_until(105.0, w + [(111.0, 120.0)]) == pytest.approx(122.0)   # chained windows


def test_flight2_insertion_warp_is_held_until_cutoff_plus_12():
    """Flight #2: cutoff 103525.22, the trim warp started 1.6 s later. Live windows from the events the director
    had at that moment (ORBIT reported at cutoff, the thrust period in burns) hold it to cutoff + 10 + 2."""
    t0, meco, cut = 103121.48, 103301.16, 103525.22
    ev = {"LIFTOFF": (t0, True), "MAX-Q": (t0 + 42, True), "SRB SEP": (t0 + 60, True), "MECO": (meco, True),
          "ORBIT": (cut, True)}
    w = sr.live_windows(ev, [(103512.22, cut)], t0, cut + 1.6)
    assert (103512.22 - sr.PRE_BURN - sr.WIN_MARGIN, cut + sr.POST_ORBIT) in [(round(a, 2), round(b, 2)) for a, b in w]
    assert sr.warp_hold_until(cut + 1.6, w) == pytest.approx(cut + sr.POST_ORBIT + sr.WARP_PAD)
    # the warp after MECO (to the insertion burn) waits for MECO + 5 s (+2) only; P1 before MECO is open -> ignored
    assert sr.warp_hold_until(meco + 0.1, sr.live_windows(ev, [], t0, meco + 0.1)) == pytest.approx(meco + 7.0)
    assert sr.live_windows({"LIFTOFF": (t0, True)}, [], t0, t0 + 50.0) == []   # P0+P1 merged, open until MECO
    # release: V4 window +-10 s
    ev2 = dict(ev, **{"PAYLOAD SEP": (104990.36, True)})
    assert sr.warp_hold_until(104993.0, sr.live_windows(ev2, [(103512.22, cut)], t0, 104993.0)) == \
        pytest.approx(104990.36 + sr.SEP_WIN + sr.WIN_MARGIN + sr.WARP_PAD)     # the +1 s is held at 1x too


def test_warp_guard_waits_on_the_game_clock():
    c = Clock(ut=1001.6, rate=0.4)
    logs = []
    g = core.WarpGuard(c.ut, lambda now: [], log=logs.append, wall=c.wall, sleep=c.sleep)
    g.note_cutoff(1000.0)                                                    # executor cutoff
    assert g.hold(1100.0) is True
    assert c.u >= 1000.0 + sr.POST_ORBIT + sr.WARP_PAD                       # waited 10.4 GAME s (26 s wall)
    assert c.w == pytest.approx((1012.0 - 1001.6) / 0.4, abs=0.2)
    assert g.holds and "warp guard" in logs[0]
    # target reached while waiting: the warp is skipped
    c2 = Clock(ut=1001.6, rate=1.0)
    g2 = core.WarpGuard(c2.ut, lambda now: [], log=lambda *a: None, wall=c2.wall, sleep=c2.sleep)
    g2.note_cutoff(1000.0)
    core.WARP_GUARD = g2

    class SC:
        calls = []

        def warp_to(self, ut):
            self.calls.append(ut)
    sc = SC()
    core.warp_to(sc, 1010.0)
    assert sc.calls == []
    core.warp_to(sc, 1500.0)                                                 # no window open any more: warps
    assert sc.calls == [1500.0]
    # game paused (rate 0): the hold ends after cap x the planned game wait in wall time (never hangs)
    c3 = Clock(ut=1001.6, rate=0.0)
    g3 = core.WarpGuard(c3.ut, lambda now: [], log=lambda *a: None, wall=c3.wall, sleep=c3.sleep)
    g3.note_cutoff(1000.0)
    g3.hold(2000.0)
    assert c3.w == pytest.approx(core.REC_WAIT_CAP * (1012.0 - 1001.6) + 1.0, abs=0.15)


def test_warp_to_without_guard_is_the_plain_call():
    """Non-recording guarantee: core.warp_to(sc, ut) with WARP_GUARD None is exactly sc.warp_to(ut) - no UT read."""
    seen = []

    class SC:
        def __getattr__(self, name):
            seen.append(name)
            return (lambda *a: seen.append(a)) if name == "warp_to" else 0.0
    core.warp_to(SC(), 123.5)
    assert seen == ["warp_to", (123.5,)]


def test_burn_and_mission_warps_go_through_the_guard():
    src = open(os.path.join(ROOT, "kspilot", "burn.py")).read() + open(os.path.join(ROOT, "kspilot", "mission.py")).read()
    assert "sc.warp_to(" not in src.replace("core.warp_to(", "")              # every executor / coast warp
    assert "if core.WARP_GUARD is not None:" in open(os.path.join(ROOT, "kspilot", "burn.py")).read()


# ------------------------------------------------------------------------------ 1. editor fallback (synthetic index.csv)
def _recording(tmp_path, rails_gap):
    """T-11 .. cutoff + 1.6 s at 1x; then (flight #2) 9 s captured on rails at 0.3 s per frame inside the P4 window;
    then a rails coast (x50). rails_gap False: the same recording without the defect (1x to the end of P4)."""
    fr = tmp_path / ("frames_%s" % rails_gap)
    fr.mkdir()
    t0, cut = 1000.0, 1030.0
    rows = [(t0 - 11 + i / 30.0, "phys", 1.0) for i in range(int((cut + 1.6 - (t0 - 11)) * 30))]
    u = rows[-1][0]
    if rails_gap:
        while u < cut + 10.6:
            u += 0.3
            rows.append((u, "rails", 6.0))
    else:
        while u < cut + 12.0:
            u += 1 / 30.0
            rows.append((u, "rails", 1.0))                                   # WarpMode HIGH at rate 1: not rails
    for _ in range(200):
        u += 50 / 30.0
        rows.append((u, "rails", 50.0))
    with open(fr / "index.csv", "w") as f:
        f.write("frame,ut,timescale,warp,mode,capture,shot\n")
        for i, (u, mode, w) in enumerate(rows):
            f.write("%d,%.4f,1,%.2f,%s,30,chase\n" % (i, u, w, mode))
    json.dump(dict(ev={"LIFTOFF": [t0, True], "MECO": [t0 + 5.0, True], "ORBIT": [cut, True]},
                   burns=[[cut - 8.0, cut]], cd=t0, t0=t0, period=1958.0, marks={}), open(fr / "events.json", "w"))
    return str(fr), cut


def test_editor_badges_rails_frames_in_a_1x_window(tmp_path, capsys):
    frames, cut = _recording(tmp_path, True)
    idx, evj, m, plan, out, problems = ed.build(frames)
    lap = [s for s in plan if s.phase == "P4" and s.kind == "lapse"]
    assert len(lap) == 1 and lap[0].ut0 == pytest.approx(cut + 1.6, abs=0.1) and lap[0].ut1 == pytest.approx(cut + 10)
    assert sr.tier(lap[0].speed) == 10                                      # 8.4 s game in ~1 s video: "x10" badge
    assert ed.overlay_state(plan, plan.index(lap[0]), 0)[0] == "lapse"     # badged (never an unbadged speed-up)
    uts = [x[1] for x in idx]
    for o in out:                                                         # no rails frame is shown as 1x
        if o.src is not None and lap[0].ut0 <= uts[o.src] < lap[0].ut1:
            assert plan[o.seg].kind == "lapse"
    assert any("BADGED lapse" in p for p in problems) and any("V2 insertion burn" in p for p in problems)
    rc = ed.main([frames, "--plan-only", "--out", str(tmp_path / "x.mp4")])
    txt = capsys.readouterr().out
    assert rc == 0 and "EDL CHECKS FAILED" in txt and "checks: FAILED" in txt
    # the clean recording: no fallback, no banner (rate-1 'rails' rows are not rails)
    frames2, _ = _recording(tmp_path, False)
    idx2, _, _, plan2, _, problems2 = ed.build(frames2)
    assert problems2 == [] and not any(s.phase == "P4" and s.kind == "lapse" for s in plan2)
    ed.main([frames2, "--plan-only", "--out", str(tmp_path / "y.mp4")])
    assert "EDL CHECKS FAILED" not in capsys.readouterr().out


def test_compressed_runs_ignore_single_hitches():
    plan = [sr.Seg("P10", "real", 0.0, 10.0, 1.0, 10.0)]
    uts = [i / 30.0 for i in range(150)] + [5.2 + i / 30.0 for i in range(140)]   # one 0.2 s hitch at 5 s
    assert sr.compressed_runs(uts, None, plan) == []
    assert sr.badge_compressed(plan, []) == plan


# ------------------------------------------------------------------------------ 2. V7 pad lens auto-zoom
def test_pad_lens_fov_math():
    frame, fov, ok = sr.pad_lens(13100.0)                                     # flight #2's empty-sky shot
    assert frame == pytest.approx(66.7, abs=0.1) and fov == pytest.approx(0.29, abs=0.01) and not ok
    assert sr.stage_fill(13100.0, 1.2) == pytest.approx(40 / 1080.0, abs=0.003)    # the measured 40 px dot
    assert sr.PAD_LENS_MAX_RANGE == pytest.approx(12732, abs=5)
    for rng in (12700.0, 9000.0, 5000.0, 1500.0):
        frame, fov, ok = sr.pad_lens(rng)
        assert ok and sr.stage_fill(rng, min(fov, 14.0)) >= sr.PAD_LENS_FILL - 1e-9
    assert sr.PAD_LENS_MIN_FOV < 1.2                                          # below the old plugin clamp
    lsrc = open(os.path.join(ROOT, "kspilot", "landing.py")).read()
    assert "showrun.PAD_LENS_FRAME" in lsrc and "showrun.PAD_LENS_MIN_FOV" in lsrc and "pad_ok=self._pad_ok" in lsrc
    cs = open(os.path.join(ROOT, "plugins", "AutoLab", "AutoLab.cs")).read()
    assert "Mathf.Min(P(shot, 6, 1.2f), fmax)" in cs                          # old shot strings keep the 1.2 deg clamp


def test_pad_lens_band_goes_to_the_steep_chase_beyond_its_reach():
    def first(pad_ok):
        dd = sr.DescentDirector()
        return dd.step(0.0, 12000.0, 12000.0, -240.0, 14000.0, False, False, True, None, pad_ok=pad_ok)
    assert first(True) == "D2" and first(False) == sr.V7_PAD_SUB == "D3"


# ------------------------------------------------------------------------------ 3. V8 event shots split
def _state(ut, ev, burning=False, burn_end=None, burn_start=None):
    return d.State(ut, ut - 0.0, ev, 1e5, burning, False, 0, True, 4e5, 30.0, 90.0, True, burn_end, False,
                   burn_start)


def test_long_burn_alternates_angles():
    """A 30 s deorbit burn: S4D and S4E alternate, no piece longer than EVENT_SPLIT (+ one tick); after the cutoff
    S4E holds POST_DEORBIT, then the bridge."""
    c = d.CutterV2()
    ev = {"PAYLOAD SEP": (0.0, True), "DEORBIT BURN": (115.0, False)}
    cuts, ut = [], 90.0
    while ut < 150.0:
        burning = 100.0 + 1.0 < ut < 130.0
        for a in c.step(_state(ut, ev, burning, 130.0 if ut >= 130.0 else None), []):
            if a[0] == "cut":
                cuts.append((ut, a[2]))
        ut = round(ut + 0.25, 4)
    ids = [s for _, s in cuts]
    assert ids[0] == "BRIDGE" and ids[1] == "S4D" and ids[-1] == "BRIDGE" and "S4E" in ids
    for (u0, s), (u1, _) in zip(cuts, cuts[1:]):
        if s in ("S4D", "S4E"):
            assert u1 - u0 <= sr.EVENT_SPLIT + 0.25 + 1e-6, (s, u0, u1)
    assert cuts[-1][0] == pytest.approx(130.0 + d.POST_BURN_HOLD, abs=0.3)


def test_short_burn_split_at_cutoff_and_release_announced():
    """Insertion: S4 (pre-roll + burn) -> S4b at the cutoff (urgent) for P4's 10 s; the release, announced
    PRE_SEP_LEAD s ahead, cuts to S5 at release - 10 s and to S5b at release + S5_AFTER_SEP.
    Fix round 3: this 8 s burn (pre-roll from node - 6 s + 8 s + the 5 s ORBIT-icon hold = 15 s predicted) is split in
    two halves S4 / S4c (CutterV2.split_limit); the partner gets at least MIN_SHOT; the predicted (non-firm) ORBIT
    window does not hold burn-chain cuts."""
    c = d.CutterV2()
    cut, sep = 200.0, 400.0
    cuts, ut = [], 180.0
    while ut < 420.0:
        ev = {"MECO": (0.0, True)}
        if ut >= cut + 2.0:
            ev["ORBIT"] = (cut + 2.0, True)
        else:
            ev["ORBIT"] = (cut - 4.0, False)
        if ut >= sep:
            ev["PAYLOAD SEP"] = (sep, True)
        elif ut >= sep - sr.PRE_SEP_LEAD:
            ev["PAYLOAD SEP"] = (sep, False)
        lit = cut - 8.0 <= ut < cut
        for a in c.step(_state(ut, ev, cut - 8.0 + 1.0 < ut < cut, cut if ut >= cut else None,
                               cut - 8.0 if lit else None), sr.transitions(ev, None, v2=True)):
            if a[0] == "cut":
                cuts.append((ut, a[2]))
        ut = round(ut + 0.25, 4)
    at = {s: u for u, s in cuts}
    assert [s for _, s in cuts] == ["BRIDGE", "S4", "S4c", "S4b", "S5", "S5b", "BRIDGE"]
    assert at["S4"] == pytest.approx(cut - 4.0 - d.BURN_PREROLL, abs=0.3)
    assert at["S4c"] - at["S4"] == pytest.approx(7.5, abs=0.3)                 # half of the predicted 15 s
    assert at["S4b"] == pytest.approx(max(cut, at["S4c"] + sr.MIN_SHOT), abs=0.3)
    assert at["S5"] == pytest.approx(sep - sr.SEP_WIN, abs=0.3) and at["S5b"] == pytest.approx(sep + 2.0, abs=0.3)
    assert at["BRIDGE"] == pytest.approx(sep + sr.SEP_WIN, abs=0.3)
    cams = {k: f(None) for k, f in d.V2_CAM.items() if k in ("S4b", "S4c", "S4D", "S4E", "S5", "S5b")}
    assert all(v.endswith("cut") for v in cams.values()) and "sun" in cams["S5"]      # hard cuts, lit side
    assert cams["S4D"].startswith("rel 16 ")                    # fix round 3: engine camera (hull cam was luma 4.9)


def test_flight2_replay_shot_lengths(capsys):
    from flightdata import flight_file
    csv_path = flight_file("20261021_105006_orbit_T-WorksFineGF_landing.csv")
    if csv_path is None:
        pytest.skip("flight #2 landing CSV not available (tests/data or KSP_REPLAY_LOGS)")
    assert rp.main([csv_path]) == 0                                          # plan / frames / shot lengths PASS
    out = capsys.readouterr().out
    assert "flight 105003" in out and "S4b" in out and "S4D" in out


# ------------------------------------------------------------------------------ 5. waits in game seconds
def test_game_clock_waits_in_game_seconds_with_a_wall_cap():
    c = Clock(ut=500.0, rate=0.4)
    gc = core.GameClock(c.ut, wall=c.wall, sleep=c.sleep)
    m = gc.mark()
    gc.sleep(5.0)                                                             # DRAG_SETTLE_S: 5 GAME s
    assert c.u - 500.0 == pytest.approx(5.0, abs=0.03) and c.w == pytest.approx(12.5, abs=0.1)
    assert gc.elapsed(m) == pytest.approx(5.0, abs=0.03)
    c0 = Clock(ut=500.0, rate=0.0)                                           # paused game: 3x wall cap
    gc0 = core.GameClock(c0.ut, wall=c0.wall, sleep=c0.sleep)
    gc0.sleep(1.0)
    assert c0.w == pytest.approx(3.0, abs=0.06)
    gcx = core.GameClock(lambda: 1 / 0, wall=c.wall, sleep=c.sleep)           # UT unreadable: wall clock
    mx = gcx.mark()
    c.sleep(2.0)
    assert gcx.elapsed(mx) == pytest.approx(2.0)


def test_rec_helpers_are_the_plain_calls_without_a_game_clock(monkeypatch):
    """Non-recording guarantee: tmark / tsince / tsleep == time.time() / time.time() - m / time.sleep(d)."""
    calls = []
    monkeypatch.setattr(core.time, "time", lambda: calls.append("time") or 42.0)
    monkeypatch.setattr(core.time, "sleep", lambda dt: calls.append(("sleep", dt)))
    assert core.tmark() == 42.0 and core.tsince(40.0) == 2.0
    core.tsleep(5.0)
    assert calls == ["time", "time", ("sleep", 5.0)]


def test_lander_settle_stamps(monkeypatch):
    from kspilot.landing import Lander
    L = Lander.__new__(Lander)                                                # no kRPC: only the clock helpers
    monkeypatch.setattr("kspilot.landing.time.time", lambda: 100.0)
    L.rec_v2 = False
    assert L._stamp() == 100.0 and L._age(94.0) == 6.0                        # wall clock, as before
    c = Clock(ut=7000.0, rate=0.4)
    c.w = 1.76e9                                                              # an epoch-like wall clock
    L.rec_v2, L.ut = True, c.ut
    L._gc = core.GameClock(c.ut, wall=c.wall, sleep=c.sleep)
    s = L._stamp()
    c.sleep(10.0)                                                             # 10 s wall = 4 s game
    assert L._age(s) == pytest.approx(4.0) and L._age(s) < L.brake_settle_s   # still settling (was "settled")
    assert L._age(0) > 1e3                                                   # the default stamp: long ago


def test_separation_window_in_game_seconds():
    from kspilot.ascent_own import Ascent
    import time as _t
    assert Ascent._sep_age({"t": _t.time() - 3.5, "m": None}) == pytest.approx(3.5, abs=0.05)   # wall, as before
    c = Clock(ut=103181.0, rate=0.4)
    core.GAME_CLOCK = core.GameClock(c.ut, wall=c.wall, sleep=c.sleep)
    sep = {"t": 0.0, "m": core.GAME_CLOCK.mark()}
    c.sleep(3.9)                                                              # flight #2: metric after 3.9 s wall
    assert Ascent._sep_age(sep) == pytest.approx(1.56, abs=0.01)            # = 1.6 game s: keep watching (< 3)
    Ascent._sep_wait(sep, 3.0)
    assert Ascent._sep_age(sep) >= 3.0 - 1e-9 and c.u - 103181.0 >= 3.0 - 1e-9


def test_hooks_installed_only_under_rec_full():
    """GAME_CLOCK / WARP_GUARD are assigned in exactly one place: the `if REC_FULL:` block of the flight script."""
    src = open(os.path.join(ROOT, "tools", "orbit_recover.py")).read()
    block = src.split("\nif REC_FULL:\n", 1)[1].split("\nif FROM_ORBIT or FROM_ENTRY:", 1)[0]
    for hook in ("core.GAME_CLOCK = ", "core.WARP_GUARD = "):
        assert src.count(hook) == 1 and hook in block
    assert "if REC_FULL and m.payload_stage is not None:" in src                # the announced release
    for root, _, files in os.walk(os.path.join(ROOT, "kspilot")):
        for f in files:
            if f.endswith(".py"):
                body = open(os.path.join(root, f)).read()
                assert "GAME_CLOCK = core" not in body and "WARP_GUARD = core" not in body
