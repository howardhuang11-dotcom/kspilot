"""Recording pipeline v2 (HOP_REC=full; director v2 specification, video requirements v2 V1-V10), pure pieces:
playback (warp) policy per phase, the jump-cut EDL from the mission events, frame selection, the v2 shot schedulers
(ascent / orbit director, descent camera), the 60 m drone trigger, the overlay keys and the baseline protection
(nothing changes without v2). No KSP needed; the editor test needs ffmpeg."""
import json
import re
import math
import os
import shutil
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
from kspilot import core, director as d, hud, showrun as sr  # noqa: E402
import director_replay as rp  # noqa: E402
import edit_video as ed  # noqa: E402

F = rp.FLIGHT
T0 = F["t0"]
TD = T0 + 21812.0
ENTRY = (T0 + 21617.4, T0 + 21634.5)


def nominal(**kw):
    m = dict(rec0=T0 - sr.REC_LEAD, rec1=TD + sr.TD_TAIL, cd=T0, t0=T0, meco=F["meco"], orbit=F["orbit"], sep=F["sep"],
             deorbit=F["deorbit"], entry=ENTRY[0], td=TD, lander=F["handoff"], burns=[F["orbit"], F["deorbit"]])
    m.update(kw)
    return m


def seg_at(plan, ut):
    return next(s for s in plan if s.ut0 <= ut < s.ut1)


# ---------------------------------------------------------------- playback policy per phase (V1 V2 V4 V5 V6 V9)
def test_warp_policy_per_phase():
    m = nominal()
    plan = sr.playback_plan(m)
    assert sr.check_plan(plan, m) == []
    want = [(T0 - 10.5, "drop"), (T0 - 9.9, "real"), (T0 + 100, "real"), (F["meco"] + 4.9, "real"),  # V1 V2
            (F["meco"] + 6, "lapse"),                                                               # P3
            (F["orbit"][0] - 9.9, "real"), (F["orbit"][1] + 9.9, "real"),                            # V2 insertion
            (F["orbit"][1] + 11, "lapse"),                                                          # P5
            (F["sep"] - 9.9, "real"), (F["sep"] + 9.9, "real"),                                      # V4
            (F["sep"] + 10.9, "real"), (F["sep"] + 11.1, "lapse"), (F["sep"] + 500, "skip"), (F["deorbit"][0] - 11, "skip"),      # P7 bridge + card
            (F["deorbit"][0] - 9.9, "real"), (F["deorbit"][1] + 4.9, "real"),                        # P8
            (F["deorbit"][1] + 6, "lapse"), (ENTRY[0] - 11.1, "lapse"), (ENTRY[0] - 10.9, "real"),     # P9
            (ENTRY[0] - 9.9, "real"), (TD + 14.9, "real")]                                           # V6
    for ut, kind in want:
        assert seg_at(plan, ut).kind == kind, (ut - T0, seg_at(plan, ut))
    assert [s.phase for s in plan if s.kind == "skip"] == ["P7"]                    # V9: the only jump cut
    summ = sr.plan_summary(plan)
    assert summ["P3"] == pytest.approx(10.0) and summ["P5"] == pytest.approx(3.0) and summ["P9"] == pytest.approx(8.0)
    assert summ["P7"] == pytest.approx(sr.BRIDGE_VIDEO + sr.CARD_VIDEO)
    assert next(s for s in plan if s.phase == "P7" and s.kind == "lapse").speed == pytest.approx(sr.BRIDGE_S)
    assert 480.0 <= sum(summ.values()) <= 540.0                                      # spec target ~8:30 (8:00-9:00)


def test_badge_tiers():
    assert sr.tier(20.4) == 20 and sr.tier(481) == 500 and sr.tier(43.5) == 50 and sr.tier(100) == 100
    assert sr.tier(1.6) == 2 and sr.tier(30000) == 10000


def test_extra_burn_in_a_lapse_is_real_time_and_splits_the_span():
    b = (F["orbit"][1] + 600.0, F["orbit"][1] + 603.0)          # a 3 s trim in P5 (> 1 s): P11
    m = nominal(burns=[F["orbit"], F["deorbit"], b])
    plan = sr.playback_plan(m)
    assert sr.check_plan(plan, m) == []
    assert seg_at(plan, b[0] - 1.9).kind == "real" and seg_at(plan, b[1] + 1.9).kind == "real"
    p5 = [s for s in plan if s.phase == "P5"]
    assert len(p5) == 2 and p5[0].speed == pytest.approx(p5[1].speed)              # one speed per span
    assert sum(s.video for s in p5) == pytest.approx(3.0)
    short = (F["orbit"][1] + 700.0, F["orbit"][1] + 700.6)      # 0.6 s pulse: not a burn shot, stays in the lapse
    assert seg_at(sr.playback_plan(nominal(burns=[F["orbit"], F["deorbit"], short])), short[0]).kind == "lapse"


def test_checkpoint_recording_from_orbit_starts_in_the_coast():
    m = dict(rec0=T0 + 2000.0, rec1=TD + sr.TD_TAIL, deorbit=F["deorbit"], entry=ENTRY[0], td=TD, lander=F["handoff"],
             burns=[F["deorbit"]])
    plan = sr.playback_plan(m)
    assert sr.check_plan(plan, m) == []
    assert [s.kind for s in plan[:2]] == ["lapse", "skip"] and plan[0].phase == "P7"


def test_aborted_recording_keeps_its_tail():
    m = dict(rec0=T0 - sr.REC_LEAD, rec1=T0 + 120.0, cd=T0, t0=T0, burns=[])
    plan = sr.playback_plan(m)
    assert plan[-1].kind == "real" and plan[-1].ut1 == pytest.approx(T0 + 120.0)


def test_check_plan_catches_a_jump_cut_in_a_real_time_window():
    m = nominal()
    plan = sr.playback_plan(m)
    bad = [sr.Seg("P10", "skip", ENTRY[0], ENTRY[0] + 5, None, 2.5) if s.phase == "P10" else s for s in plan]
    assert any("outside P7" in p for p in sr.check_plan(bad, m)) and any("V6" in p for p in sr.check_plan(bad, m))


# ---------------------------------------------------------------- EDL from the logged events + frame selection
def events_json():
    ev = {"LIFTOFF": [T0, True], "MAX-Q": [F["maxq"], True], "SRB SEP": [F["srb"], True], "MECO": [F["meco"], True],
          "ORBIT": [F["orbit"][1] + 0.3, True], "PAYLOAD SEP": [F["sep"], True],
          "DEORBIT BURN": [0.5 * sum(F["deorbit"]), True], "ENTRY BURN": [ENTRY[0], True],
          "AERO GLIDE": [ENTRY[1], True], "TOUCHDOWN": [TD, True]}
    burns = [list(F["orbit"]), [F["orbit"][1] + 400, F["orbit"][1] + 400.6], list(F["deorbit"])]
    return dict(ev=ev, burns=burns, cd=T0, t0=T0, period=1958.0, lander=F["handoff"],
                marks={"S8": ENTRY[1], "HANDOFF": TD - 16.0})


def test_mission_from_events_finds_the_burns():
    m = sr.mission_from_events(events_json(), T0 - 11, TD + 15)
    assert m["orbit"] == tuple(F["orbit"]) and m["deorbit"] == tuple(F["deorbit"])
    assert m["t0"] == T0 and m["td"] == TD and m["entry"] == ENTRY[0]
    j = events_json()
    j["burns"] = []                                              # no thrust record: a window around the events
    m2 = sr.mission_from_events(j, T0 - 11, TD + 15)
    assert m2["orbit"][1] == pytest.approx(F["orbit"][1] + 0.3) and m2["deorbit"][0] < 0.5 * sum(F["deorbit"])


def synth_uts(rails, pause=None, phys_uncomp=None, t1=None):
    """Frame UTs: 1/30 s per frame, rate/30 on rails, none while paused; phys_uncomp: physics x2 WITHOUT the B1
    capture compensation (2/30 s per frame)."""
    uts, u = [], T0 - 11.0
    t1 = t1 or TD + 15.0
    while u < t1:
        if pause and pause[0] <= u < pause[1]:
            u = pause[1]
            continue
        uts.append(u)
        r = next((w for a, b, w in rails if a <= u < b), 1.0)
        if phys_uncomp and phys_uncomp[0] <= u < phys_uncomp[1]:
            r = 2.0
        u += r / 30.0
    return uts


def test_select_frames_real_time_is_frame_exact_and_lapse_is_even():
    m = nominal()
    plan = sr.playback_plan(m)
    uts = synth_uts(F["rails"], pause=(F["sep"] + 440.0, F["deorbit"][0] - 150.0))
    out = sr.select_frames(uts, plan)
    assert sr.check_frames(out, uts, plan) == []
    assert abs(len(out) / 30.0 - sum(sr.plan_summary(plan).values())) < 2.0
    p0 = [o for o in out if plan[o.seg].phase == "P0+P1"]
    assert len(p0) == pytest.approx(30 * (F["meco"] + 5 - (T0 - 10)), abs=2)        # every 1x frame, once
    assert [o.src for o in p0] == list(range(p0[0].src, p0[0].src + len(p0)))
    card = [o for o in out if plan[o.seg].kind == "skip"]
    assert len(card) == 75 and any(o.src is not None for o in card)                 # 2.5 s, real background frames


def test_uncompensated_physics_warp_is_shown_at_the_right_speed():
    """If the B1 capture compensation failed (frames 2/30 s apart), a 1x window still plays at 1x (each frame twice,
    15 fps) instead of 2x: the speed is right, the picture stutters (spec 2.1)."""
    m = nominal()
    plan = sr.playback_plan(m)
    uts = synth_uts(F["rails"], phys_uncomp=(ENTRY[0] - 30.0, ENTRY[1]))
    out = sr.select_frames(uts, plan)
    burn = [o for o in out if ENTRY[0] <= uts[o.src] < ENTRY[1]]
    assert len(burn) == pytest.approx(30 * (ENTRY[1] - ENTRY[0]), abs=3)
    assert sr.check_frames(out, uts, plan) == []


def test_video_time_mapping():
    plan = sr.playback_plan(nominal())
    assert sr.video_time(plan, T0 - 10.0) == pytest.approx(0.0)
    assert sr.video_time(plan, T0) == pytest.approx(10.0)
    a, b = sr.video_time(plan, F["sep"] + 10.0 + sr.WIN_MARGIN), sr.video_time(plan, F["deorbit"][0] - 10.0)
    assert b - a == pytest.approx(6.5)
    assert sr.video_time(plan, TD + 15.0) == pytest.approx(sum(sr.plan_summary(plan).values()))


# ---------------------------------------------------------------- v2 director: ascent / orbit shots
def test_ascent_schedule():
    assert sr.ascent_shot_v2(None) == ("S1", sr.V2_S1_END)
    assert sr.ascent_shot_v2(24.9)[0] == "S1" and sr.ascent_shot_v2(25.0) == ("S2a", 48.0)
    assert sr.ascent_shot_v2(100.0) == ("ONB", 110.0) and sr.ascent_shot_v2(170.0) == ("S2e", sr.ASCENT_CYCLE_FROM)
    assert sr.ascent_shot_v2(190.0)[0] == "S2d" and sr.ascent_shot_v2(210.0)[0] == "S2f"     # late MECO: alternate


def replay_director():
    def txs(ev):
        return sr.transitions(ev, None, v2=True)
    return rp.replay_ascent_orbit(F, txs)


def test_v2_director_shots_and_durations():
    cuts, pauses = replay_director()
    ids = [s for _, s in cuts]
    assert ids[:9] == ["S1", "S2a", "S2b", "S3", "S2c", "ONB", "S2d", "S2f", "S2e"]
    # fix round 2 (V8): insertion S4 -> S4b (cutoff hold + P5 lapse) -> S5 (announced release - 10 s) -> S5b;
    # deorbit S4D (night side) -> S4E (cutoff + 5 s). Fix round 3: the 07:53 insertion (pre-roll + 7.6 s burn + the
    # 5 s ORBIT-icon hold = 14.8 s predicted) is split in two halves S4 / S4c before the cutoff hold S4b
    assert ids[9:] == ["BRIDGE", "S4", "S4c", "S4b", "S5", "S5b", "BRIDGE", "S4D", "S4E", "BRIDGE"]
    at = {s: u for u, s in reversed(cuts)}
    assert at["S2a"] - T0 == pytest.approx(sr.V2_S1_END, abs=0.3)
    assert at["S3"] - F["srb"] == pytest.approx(-2.5, abs=0.3)                       # urgent pre-roll
    s4 = [u for u, s in cuts if s in ("S4", "S4D")]
    assert F["orbit"][0] - 10.0 < s4[0] < F["orbit"][0] and F["deorbit"][0] - 10.0 < s4[1] < F["deorbit"][0]
    assert at["S5"] == pytest.approx(F["sep"] - sr.SEP_WIN, abs=0.3)                 # the V4 window, not before
    assert at["S5b"] == pytest.approx(F["sep"] + sr.S5_AFTER_SEP, abs=0.3)
    plan = sr.playback_plan(nominal())
    for sid, u0, u1, v in sr.shot_durations(plan, cuts, F["handoff"]):
        assert v >= sr.MIN_SHOT - 1e-6, (sid, u0 - T0, v)
        assert v <= sr.MAX_HOLD[sr.SHOT_TYPE[sid]], (sid, u0 - T0, v)
        assert sr.SHOT_TYPE[sid] != "event" or v <= 15.0 + 1e-6                     # event close-up limit (V8)


def test_v2_director_never_cuts_in_a_hud_window_and_pauses_only_in_p7():
    cuts, pauses = replay_director()
    ev_all = {"LIFTOFF": (T0, True), "ORBIT": (F["orbit"][1], True), "PAYLOAD SEP": (F["sep"], True)}
    txs = sr.transitions(ev_all, None, v2=True)
    for u, s in cuts[1:]:
        assert sr.cut_ok(u, txs), (u - T0, s)
    p7 = (F["sep"] + sr.SEP_WIN, F["deorbit"][0] - sr.PRE_BURN)
    for u, a in pauses:
        assert p7[0] < u < p7[1], (u - T0, a)
    assert pauses and pauses[0][1] == "pause" and pauses[-1][1] == "resume"
    assert pauses[0][0] > F["sep"] + sr.SEP_WIN + sr.BRIDGE_S * sr.BRIDGE_VIDEO   # the bridge footage is recorded
    assert pauses[-1][0] < F["deorbit"][0] - 60.0                                  # capture back long before P8


def test_v2_cutter_never_pauses_outside_p7():
    c = d.CutterV2()
    ev = {"LIFTOFF": (T0, True), "MECO": (T0 + 180, True)}
    for k in range(400):                         # 100 s of "dead air" after MECO (drag probes, waits): no pause
        s = d.State(T0 + 300 + 0.25 * k, 300 + 0.25 * k, ev, 9e4, False, False, 0, True, 4e5, 30.0, 67.0, False,
                    None, False)
        assert ("pause",) not in c.step(s, [])


# ---------------------------------------------------------------- v2 descent camera + the 60 m drone trigger
DESC = [(-10, 47000), (0, 44856), (17, 40508), (64, 25000), (93, 15000), (113, 11000), (124, 9000), (136, 7000),
        (148, 5000), (159, 3500), (179, 1000), (189.6, 120), (191.5, 60), (196.2, 0)]
HOR = [(-10, 60000), (64, 32200), (93, 6600), (113, 2000), (124, 670), (159, 46), (179, 1), (197, 1)]


def interp(tab, t):
    for (t0, a), (t1, b) in zip(tab, tab[1:]):
        if t0 <= t <= t1:
            return a + (b - a) * (t - t0) / (t1 - t0)
    return tab[-1][1]


def replay_descent(dt=0.125, hud_win=()):
    dd = sr.DescentDirector()
    log, handoff, t = [], None, -10.0
    while t < 196.2 + 15:
        h = max(interp(DESC, t), 0.0)
        vz = (interp(DESC, t + 0.05) - interp(DESC, t)) / 0.05
        if handoff is None:
            eta = sr.handoff_eta(h, vz)
            if eta is not None and eta <= sr.HANDOFF_LEAD:
                handoff = t + eta
        rng = math.sqrt((interp(HOR, t) + 410) ** 2 + h * h)
        sid = dd.step(t, h, h, vz, rng, 0 <= t < 17, -4 <= t < 0, t >= 17, handoff, hud_win, td_done=t >= 196.2)
        if not log or log[-1][1] != sid:
            log.append((t, sid, h))
        t = round(t + dt, 4)
    return log, handoff


def test_descent_rotation_v7_and_drone():
    log, handoff = replay_descent()
    ids = [s for _, s, _ in log]
    assert ids[:3] == ["S8", "S7", "S8"]
    v7 = ids[ids.index("D2"):]
    assert v7 == ["D2", "D1", "D2", "D1", "D3", "D1", "S9", "S10"], ids
    at = {s: (t, h) for t, s, h in reversed(log)}
    assert at["S7"][0] == pytest.approx(-4.0, abs=0.13) and log[2][0] == pytest.approx(17.0, abs=0.13)
    s9 = next(t for t, s, _ in log if s == "S9")
    assert s9 == pytest.approx(handoff + sr.S9_AFTER_HANDOFF, abs=0.13)
    t10, s10, h10 = next(x for x in log if x[1] == "S10")
    assert 55.0 < h10 <= sr.DRONE_H + 0.15 * 32.0                                     # first h_gnd < 60 m (+ lead)
    assert log[-1][1] == "S10"                                                        # no cut after the drone shot
    prev = [s for _, s, _ in log][[s for _, s, _ in log].index("S9") - 1]
    assert prev != "D2"                                                               # never pad lens -> pad lens
    for (t0, s, _), (t1, _, _) in zip(log, log[1:] + [(211.2, None, 0)]):
        if s not in ("S7", "S9", "S10") and t0 > -10:
            assert t1 - t0 >= sr.MIN_SHOT, (s, t0)
        cover = (17.0 + sr.V2_PRE_S7 + 0.5 - 15.0) if s == "S7" else 0.0    # an event shot covers its whole burn
        assert t1 - t0 <= sr.MAX_HOLD[sr.SHOT_TYPE[s]] + cover, (s, t0, t1 - t0)


def test_descent_waits_for_hud_transitions():
    win = [sr._tx("FUNNEL_IN", 19.0, True)]                     # S8 + 2 s
    log, _ = replay_descent(hud_win=win + [sr._tx("HANDOFF", 112.0, True)])
    for t, s, _ in log:
        if s not in ("S7", "S9", "S10"):
            assert sr.cut_ok(t, win + [sr._tx("HANDOFF", 112.0, True)]), (t, s)


def test_drone_trigger_and_pose():
    assert not sr.drone_due(61.0, 0.0) and sr.drone_due(59.9, 0.0)
    assert sr.drone_due(62.5, -31.5)                                                  # 0.1 s lead at 31 m/s
    assert not sr.drone_due(float("nan"), -31.5)
    lat0, lon0 = core.KSC_PAD
    lat, lon, alt, tl, tn, ta = sr.drone_pose(lat0, lon0, 70.0, 90.0)              # sun due east
    e = math.radians(lon - lon0) * 600000.0 * math.cos(math.radians(lat0))
    n = math.radians(lat - lat0) * 600000.0
    assert e == pytest.approx(70.0, abs=0.1) and abs(n) < 0.1 and alt == pytest.approx(102.0)
    assert (tl, tn) == (lat0, lon0) and ta == pytest.approx(80.6)
    dep = math.degrees(math.atan2(alt - ta, 70.0))
    assert 15.0 < dep < 20.0                                                          # ~17 deg down (spec ~20)


def test_real_log_replay():
    from flightdata import flight_file
    csv_path = flight_file("20261021_075304_orbit_T-WorksFineGF_landing.csv")
    if csv_path is None:
        pytest.skip("07:53 landing CSV not available (tests/data or KSP_REPLAY_LOGS)")
    assert rp.main([csv_path]) == 0


# ---------------------------------------------------------------- overlay keys, baseline protection
def test_overlay_keys_only_with_v2():
    f = hud.HudFeed(craft="TEST")
    f._ut = T0 - 5.0
    base = f._compose({})
    assert "rec=" not in base and "cd=" not in base
    f.v2, f.cd = True, T0
    kv = dict(l.split("=", 1) for l in f._compose({}).split("\n")[1:])
    assert kv["rec"] == "full" and kv["cd"] == "%.3f" % T0


def test_caption_out_is_v2_only():
    ev = {"LIFTOFF": (T0, True)}
    co = lambda v2: next(x.ut for x in sr.transitions(ev, None, v2=v2) if x.name == "CAPTION_OUT")   # noqa: E731
    assert co(False) == T0 + sr.CAPTION_OUT == T0 + 30.0 and co(True) == T0 + 20.0


def test_record_start_request_is_unchanged_without_v2(tmp_path, monkeypatch):
    monkeypatch.setattr(core, "_AL", str(tmp_path))
    core.record_start("x")
    assert open(tmp_path / "rec.req").read().count("|") == 2
    core.record_start("x", v2=True)
    assert open(tmp_path / "rec.req").read().endswith("|v2")


def test_baseline_paths_gated():
    """Every v2 code path in the flight script hangs off REC_FULL (HOP_REC=full) and the lander's rec_v2 flag."""
    src = open(os.path.join(ROOT, "tools", "orbit_recover.py")).read()
    assert "lander.rec_v2 = REC_FULL" in src and "Director(log=log, v2=True)" in src
    assert "elif frames:\n    print(\"video:\", core.record_stop(frames, tag, fast=2.0, tail_s=25.0))" in src
    assert src.count("v2=True") == 3          # director + the two record_start calls, all inside REC_FULL blocks
    lsrc = open(os.path.join(ROOT, "kspilot", "landing.py")).read()
    assert "if self.rec_v2:\n            sid, want = self._camera_v2(ut, r)" in lsrc
    assert 'if self.rec_v2 and os.environ.get("KSP_REC_B2") == "1":' in lsrc


# ---------------------------------------------------------------- editor: overlays + encode (synthetic clip)
def test_overlay_state_badges():
    plan = sr.playback_plan(nominal())
    i3 = next(i for i, s in enumerate(plan) if s.phase == "P3")
    assert ed.overlay_state(plan, i3, 0)[:3] == ("lapse", 20, 2.0)                  # intro at 2x size
    assert ed.overlay_state(plan, i3, 90)[2] == pytest.approx(1.0)
    i4 = i3 + 1
    assert ed.overlay_state(plan, i4, 0)[0] == "real" and ed.overlay_state(plan, i4, 90) is None
    assert ed.overlay_state(plan, 1, 0) is None                                      # 1x at the start: no badge
    isk = next(i for i, s in enumerate(plan) if s.kind == "skip")
    assert ed.overlay_state(plan, isk, 10)[0] == "skip" and ed.overlay_state(plan, isk, 10)[4]
    assert ed.overlay_state(plan, isk - 1, 0)[2] == 2.0                             # bridge intro
    assert re.fullmatch(r"\d+ H \d\d MIN", ed.card_text(plan, nominal()))           # skip badge text, English (V16)


@pytest.mark.skipif(shutil.which("ffmpeg") is None or ed.python_with_pil() is None, reason="needs ffmpeg + Pillow")
def test_editor_end_to_end_on_a_synthetic_recording(tmp_path):
    """A tiny recording (320x180 frames): 36 s at 1x around a fake lift-off, a rails coast and a 40 s landing.
    The editor (run as the flight script runs it: the interpreter that has Pillow) must write an mp4 whose length
    matches the plan and keep the frames with --keep."""
    py = ed.python_with_pil()
    fr = tmp_path / "frames_t"
    fr.mkdir()
    t0 = 1000.0
    uts = [t0 - 11 + i / 30.0 for i in range(30 * 36)]                               # T-11 .. T+25 at 1x
    uts += [uts[-1] + 50 / 30.0 * (i + 1) for i in range(300)]                       # coast on rails (x50)
    uts += [uts[-1] + (i + 1) / 30.0 for i in range(30 * 40)]                        # 40 s at 1x: landing
    with open(fr / "index.csv", "w") as f:
        f.write("frame,ut,timescale,warp,mode,capture,shot\n")
        for i, u in enumerate(uts):
            f.write("%d,%.4f,1,1,phys,30,chase\n" % (i, u))
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "lavfi", "-i", "testsrc=size=320x180:rate=30",
                    "-frames:v", str(len(uts)), "-start_number", "0", "-q:v", "4", str(fr / "%06d.jpg")], check=True)
    entry = uts[-1] - 30.0
    json.dump(dict(ev={"LIFTOFF": [t0, True], "MECO": [t0 + 18.0, True], "ENTRY BURN": [entry, True],
                       "TOUCHDOWN": [uts[-1] - 15.0, True]}, burns=[], cd=t0, t0=t0, period=1958.0, marks={}),
              open(fr / "events.json", "w"))
    out = tmp_path / "v.mp4"
    r = subprocess.run([py, os.path.join(ROOT, "tools", "edit_video.py"), str(fr), "--out", str(out), "--keep"],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "checks: PASS" in r.stdout, r.stdout
    dur = float(subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0",
                                str(out)], capture_output=True, text=True).stdout)
    idx, evj, m, plan, outf, problems = ed.build(str(fr))
    assert problems == []
    assert dur == pytest.approx(len(outf) / 30.0, abs=0.1)
    assert (tmp_path / "v.edl.txt").exists() and fr.exists()
