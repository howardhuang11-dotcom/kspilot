"""Recording pipeline v3 (decision record DR-011: OBS real-time capture, HOP_REC=full): the clocks (time map
interpolation, OBS video-clock calibration), the EDL -> ffmpeg filtergraph builders, the runtime recorder (fake OBS /
fake kRPC), the baseline protection, and an end-to-end cut of a synthetic OBS-like clip (ffmpeg testsrc-style colour
+ sine audio + a synthetic time map with a rails-warp and a physics-warp stretch). No KSP, no OBS."""
import json
import math
import os
import random
import shutil
import struct
import subprocess
import sys
import threading
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
from kspilot import core, obsrec, showrun as sr, timemap as tmap  # noqa: E402
import edit_obs as eo  # noqa: E402
import edit_video as ed  # noqa: E402

T = 1.0 / 30
W0 = 1000.0
# synthetic flight: (video s, UT, rails factor, physics factor, warp rate) at the start of each stretch
STRETCH = [(0.0, 4989.0, 0, 0, 1.0),     # T-11 .. T+36 at 1x (lift-off UT 5000, MECO 5018)
           (36.0, 5025.0, 4, 0, 50.0),   # 8 s of rails warp x50: 400 s of game time
           (44.0, 5425.0, 0, 1, 2.0),    # 4 s of physics warp x2 (the entry coast)
           (48.0, 5433.0, 0, 0, 1.0)]    # 1x to the end (entry burn 5440, touchdown 5458)
VDUR = 90.0
EV = {"LIFTOFF": [5000.0, True], "MECO": [5018.0, True], "ENTRY BURN": [5440.0, True], "TOUCHDOWN": [5458.0, True]}


def ut_of_v(v):
    for i, (v0, u0, _, _, r) in enumerate(STRETCH):
        if i + 1 == len(STRETCH) or v < STRETCH[i + 1][0]:
            return u0 + r * (v - v0)


def stretch_of(v):
    return [s for s in STRETCH if s[0] <= v][-1]


def synth_rows(jitter=0.004, hz=10.0, seed=1):
    rnd = random.Random(seed)
    rows, v = [], 0.0
    while v <= VDUR + 0.5:
        _, _, rails, phys, rate = stretch_of(v)
        vv = v + rnd.uniform(-jitter, jitter)
        rows.append((W0 + vv, ut_of_v(max(vv, 0.0)), rate, rails, phys))
        v += 1.0 / hz
    return rows


def synth_samples(w0=W0, lag=0.0, period=2.0, rtt=0.002, slope=1.0, drop_at=None, drop_s=0.0, seed=2):
    """GetRecordStatus samples: duration = whole frames handed to the encoder by the request midpoint."""
    rnd = random.Random(seed)
    out = []
    ts = [0.2 + 0.1 * i for i in range(12)] + [2.0 + period * i for i in range(int((VDUR - 2) / period))]
    for t in ts:
        t += rnd.uniform(0.0, 0.05)                                          # requests at a random frame phase
        mid = w0 + lag + t + rnd.uniform(0, 0.001)
        vt = (t - (drop_s if drop_at is not None and t > drop_at else 0.0)) / slope
        dur = math.floor(vt / T) * T
        out.append((mid - rtt / 2, mid + rtt / 2, dur))
    return out


class Ctx(eo.Ctx):
    pass


def synth_ctx(evj=None):
    tm = tmap.TimeMap(synth_rows())
    vc = tmap.calibrate(synth_samples(), lag=0.0)
    evj = evj or dict(ev=EV, burns=[], cd=5000.0, t0=5000.0, period=1958.0, marks={}, planned={})
    return eo.Ctx({}, "x.mp4", VDUR, True, vc, tm, evj)


# ---------------------------------------------------------------------------------------------------------------------
# clocks
# ---------------------------------------------------------------------------------------------------------------------
def test_timemap_interpolation_and_inverse():
    tm = tmap.TimeMap(synth_rows(jitter=0.0))
    assert tm.ut(W0 + 10.0) == pytest.approx(4999.0, abs=1e-6)
    assert tm.ut(W0 + 40.0) == pytest.approx(5225.0, abs=1e-6)                   # inside the rails stretch
    assert tm.ut(W0 + 46.0) == pytest.approx(5429.0, abs=1e-6)                   # physics x2
    for u in (4995.0, 5100.0, 5430.0, 5470.0):
        assert tm.ut(tm.wall(u)) == pytest.approx(u, abs=1e-6)
    assert tm.wall(4980.0) == pytest.approx(W0 - 9.0, abs=1e-6)                  # before the map: 1x
    assert tm.wall(5480.0) > tm.w[-1]                                            # after it: last measured speed
    assert tm.rate(W0 + 37, W0 + 43) == pytest.approx(50.0, rel=1e-6)


def test_timemap_paused_stretch_maps_to_its_start_and_ut_never_decreases():
    rows = [(0.0, 100.0, 1, 0, 0), (1.0, 101.0, 1, 0, 0), (2.0, 101.0, 1, 0, 0), (3.0, 101.0, 1, 0, 0),
            (4.0, 102.0, 1, 0, 0), (4.5, 101.9, 1, 0, 0), (5.0, 103.0, 1, 0, 0)]    # pause 1-3 s, one stale reading
    tm = tmap.TimeMap(rows)
    assert tm.wall(101.0) == pytest.approx(1.0)
    assert tm.wall(101.5) == pytest.approx(3.5)
    assert all(b >= a for a, b in zip(tm.u, tm.u[1:]))
    assert tm.rate(1.0, 3.0) == pytest.approx(0.0)


def test_timemap_runs_classify_by_warp_fields_and_measure_speed():
    tm = tmap.TimeMap(synth_rows())
    runs = tm.runs(W0 + 1.0, W0 + 88.0)
    assert [r[2] for r in runs] == ["normal", "rails", "phys", "normal"]
    edges = [r[1] - W0 for r in runs[:3]]
    assert edges == pytest.approx([36.0, 44.0, 48.0], abs=0.11)                 # one 10 Hz row
    assert runs[1][3] == pytest.approx(50.0, rel=0.03)
    assert runs[2][3] == pytest.approx(2.0, rel=0.05)
    assert runs[0][3] == pytest.approx(1.0, rel=0.01)
    # a one-row blip of physics warp inside 1x is not a stretch of its own
    rows = [(i * 0.1, i * 0.1, 1.0, 0, 1 if i == 20 else 0) for i in range(50)]
    assert [r[2] for r in tmap.TimeMap(rows).runs(0.0, 4.9)] == ["normal"]


def test_video_clock_calibration_offset_within_one_frame():
    vc = tmap.calibrate(synth_samples(), lag=0.0)
    assert vc.info["mode"] == "linear" and vc.info["n"] >= 50
    assert vc.wall(0.0) == pytest.approx(W0, abs=0.006)                       # ~6 ms from 56 samples
    assert vc.wall(60.0) - vc.wall(0.0) == pytest.approx(60.0, abs=0.01)
    assert vc.video(vc.wall(37.0)) == pytest.approx(37.0, abs=1e-6)
    # an encoder queue latency L is absorbed into W0 (the remaining capture latency is LAG_S, a gate item)
    vc = tmap.calibrate(synth_samples(lag=0.05), lag=0.0)
    assert vc.wall(0.0) == pytest.approx(W0 + 0.05, abs=T / 2)
    vc = tmap.calibrate(synth_samples(lag=0.05), lag=0.05)
    assert vc.wall(0.0) == pytest.approx(W0, abs=T / 2)


def test_video_clock_drift_slow_samples_and_dropped_frames():
    vc = tmap.calibrate(synth_samples(slope=1.0005))                        # 0.05 % clock drift: linear fit
    assert vc.info["mode"] == "linear" and vc.info["slope"] == pytest.approx(1.0005, abs=3e-4)
    assert vc.wall(80.0) == pytest.approx(W0 + 80.0 * 1.0005, abs=T)
    s = synth_samples()
    s[20] = (s[20][0] - 0.3, s[20][1] + 0.3, s[20][2])                      # one slow round trip: dropped
    assert tmap.calibrate(s).info["n"] == len(s) - 1
    vc = tmap.calibrate(synth_samples(drop_at=30.0, drop_s=0.5))            # OBS dropped 15 frames at 30 s
    assert vc.info["mode"] == "piecewise"
    assert vc.wall(10.0) == pytest.approx(W0 + 10.0, abs=T)
    assert vc.wall(60.0) == pytest.approx(W0 + 60.5, abs=T)                 # after the drop: video runs short
    with pytest.raises(ValueError):
        tmap.calibrate([(1.0, 1.001, 0.0)])


# ---------------------------------------------------------------------------------------------------------------------
# EDL -> pieces -> ffmpeg
# ---------------------------------------------------------------------------------------------------------------------
def test_edl_on_the_synthetic_flight():
    ctx = synth_ctx()
    m, plan, segs, problems, notes = eo.build(ctx)
    assert problems == []
    kinds = [(plan[i].phase, plan[i].kind) for i, _, _ in segs]
    assert kinds == [("P0+P1", "real"), ("P3", "lapse"), ("P10", "real")]
    real1, lapse, p10 = segs
    assert real1[1][0].kind == "copy" and real1[2] == 33 * 30                  # T-10 .. MECO + 5 at 1x
    assert real1[1][0].a == pytest.approx(1.0, abs=0.02)                         # video s of T-10 (first s dropped)
    assert lapse[2] == 300 and sr.tier(plan[lapse[0]].speed) == 50              # 10 s at S = 407 / 10 -> badge x50
    # the video speed-up comes from the time map: x40.7 over 1x footage, x0.81 over the x50 rails footage
    ks = [eo.k_of(p) for p in lapse[1]]
    assert max(ks) > 20.0 and min(ks) == pytest.approx(40.7 / 50.0, rel=0.06)
    assert sum(p.out for p in lapse[1]) == pytest.approx(10.0, abs=1e-6)
    assert lapse[1][0].a == pytest.approx(34.0, abs=0.05) and lapse[1][-1].b == pytest.approx(46.0, abs=0.05)
    # entry coast at physics x2 inside the V6 window (from entry - 11 s): slowed to game 1x (4 game s), then 1x
    assert [p.kind for p in p10[1]] == ["slow", "copy"]
    assert p10[1][0].out == pytest.approx(4.0, abs=0.12) and eo.k_of(p10[1][0]) == pytest.approx(0.5, rel=0.06)
    assert p10[2] == pytest.approx(44 * 30, abs=4)
    assert any("slowed to game 1x" in n for n in notes)
    assert sum(n for _, _, n in segs) == pytest.approx(87 * 30, abs=4)


def test_rails_warp_inside_a_1x_window_is_badged_at_its_measured_speed():
    ctx = synth_ctx(dict(ev=dict(EV, MECO=[5100.0, True]), burns=[], cd=5000.0, period=1958.0, marks={}))
    m, plan, segs, problems, notes = eo.build(ctx)
    lap = [plan[i] for i, _, _ in segs if plan[i].kind == "lapse" and plan[i].phase.startswith("P0")]
    assert lap and lap[0].speed == pytest.approx(50.0, rel=0.05)                # x50 badge, not an unbadged speed-up
    assert any("flown on rails warp" in p for p in problems)


def test_atempo_chain_and_frame_counts():
    for k in (0.3, 0.5, 0.81, 1.0, 1.7, 2.0, 7.3, 240.0):
        ch = eo.atempo_chain(k)
        assert all(0.5 - 1e-9 <= x <= 2.0 + 1e-9 for x in ch)
        assert math.prod(ch or [1.0]) == pytest.approx(k)
    assert eo.atempo_chain(1.0) == []
    assert eo.frame_counts([1.01, 1.01, 1.01]) == [30, 31, 30]
    assert sum(eo.frame_counts([0.333] * 100)) == round(33.3 * 30)


def test_video_filtergraph_builder():
    P = eo.Piece
    pcs = [P(10.0, 12.0, 4.0, "slow"), P(12.0, 20.0, 8.0, "copy")]
    ss, t = eo.input_window(pcs)
    assert ss == 10.0 and t == pytest.approx(11.0)
    g = eo.video_graph(pcs, 360, ss)
    assert g.startswith("[0:v]split=2[s0][s1]")
    assert "[s0]trim=start=0.000000:end=2.000000,setpts=(PTS-STARTPTS)/0.500000[p0]" in g
    assert "[s1]trim=start=2.000000:end=10.000000,setpts=(PTS-STARTPTS)/1.000000[p1]" in g
    assert "[p0][p1]concat=n=2:v=1:a=0[cat]" in g and "fps=30,scale=1920:1080" in g
    assert "lutrgb" not in g and "overlay" not in g and g.endswith("[v]")
    g = eo.video_graph([P(5.0, 605.0, 2.5, "skip")], 75, 5.0, overlay=True)
    assert "split" not in g and "setpts=(PTS-STARTPTS)/240.000000" in g
    assert "lutrgb" not in g and "[1:v]fps=30,format=rgba[ov]" in g                # V16: no shadow lift
    cmd = eo.video_cmd("in.mp4", [P(5.0, 605.0, 2.5, "skip")], 75, "o.mp4", "ov.ffconcat")
    assert cmd[cmd.index("-skip_frame") + 1] == "nokey" and cmd[cmd.index("-frames:v") + 1] == "75"
    assert "ov.ffconcat" in cmd and cmd[cmd.index("-c:v") + 1] == "libx264"
    assert "-skip_frame" not in eo.video_cmd("in.mp4", pcs, 360, "o.mp4")


def test_audio_filtergraph_builder_mutes_lapses_and_crossfades_cuts():
    P = eo.Piece
    plan = [sr.Seg("P1", "real", 0, 10, 1.0, 10.0), sr.Seg("P3", "lapse", 10, 410, 40.0, 10.0),
            sr.Seg("P10", "real", 410, 440, 1.0, 30.0)]
    segs = [(0, [P(1.0, 11.0, 10.0, "copy")], 300), (1, [P(11.0, 15.0, 10.0, "lapse")], 300),
            (2, [P(15.0, 16.5, 3.0, "slow"), P(16.5, 43.5, 27.0, "copy")], 900)]
    ap = eo.audio_plan(plan, segs, lapse_audio="mute")
    assert [a[0] for a in ap] == ["keep", "mute", "keep"]
    assert [a[3] for a in ap] == [0.2, 0.2, 0.0] and ap[2][2] == pytest.approx(30.0)
    cmd = eo.audio_cmd("in.mp4", ap, "a.m4a")
    g = cmd[cmd.index("-filter_complex") + 1]
    assert cmd.count("-i") == 2                                                 # the two 1x segments only
    assert g.count("acrossfade=d=0.200000") == 2                               # every cut, none inside a segment
    assert "aevalsrc=0|0:c=stereo:s=48000:d=10.200000" in g                    # muted lapse incl. the extension
    assert "atempo=0.500000" in g                                               # slowed piece: pitch-safe
    assert "apad=whole_dur=10.200000,atrim=end=10.200000[a0]" in g             # 1x segment + 0.2 s run-on
    assert "atrim=start=0.000000:end=10.200000" in g                           # ... taken from the source
    assert "apad=whole_dur=30.000000,atrim=end=30.000000[a2]" in g
    ap = eo.audio_plan(plan, segs, lapse_audio="atempo")
    g = eo.audio_graph(ap, [(0, 0.0), (1, 11.0), (2, 15.0)])
    assert "volume=-12.0dB" in g and "atempo=0.400000" not in g                # 0.4x lapse: speed-up chain


def test_overlay_keys_badge_spans():
    ctx = synth_ctx()
    m, plan, segs, problems, notes = eo.build(ctx)
    (i0, p0, n0), (i1, p1, n1), (i2, p2, n2) = segs
    assert all(k is None for k in eo.overlay_keys(ctx, plan, i0, p0, n0))      # 1x at the start: clean
    k1 = eo.overlay_keys(ctx, plan, i1, p1, n1)
    assert all(k is not None and k[0][0] == "lapse" and k[0][1] == 50 for k in k1)
    assert k1[0][0][2] == 2.0 and k1[-1][0][2] == 1.0                           # intro at 2x size, then 1x
    k2 = eo.overlay_keys(ctx, plan, i2, p2, n2)
    assert k2[0][0][0] == "real" and k2[89] is None                             # REAL TIME 2 s + 0.5 s fade
    assert sum(1 for k in k2 if k is not None) == 75
    assert len(eo.runs_of(k1)) < 30                                             # PNG runs, not one PNG per frame


# ---------------------------------------------------------------------------------------------------------------------
# runtime recorder (fake OBS, fake kRPC) and baseline protection
# ---------------------------------------------------------------------------------------------------------------------
class _FakeSC:
    def __init__(self):
        self.t0 = time.time()

    @property
    def ut(self):
        return 5000.0 + (time.time() - self.t0)

    warp_rate, rails_warp_factor, physics_warp_factor = 1.0, 0, 0


class _FakeStream:
    def __init__(self, obj, name):
        self.obj, self.name, self.rate, self.cbs = obj, name, 0, []

    def __call__(self):
        return getattr(self.obj, self.name)

    def add_callback(self, cb):
        self.cbs.append(cb)
        cb(self())


class _FakeConn:
    def __init__(self):
        self.space_center = _FakeSC()
        self.closed = False

    def add_stream(self, fn, obj, name):
        return _FakeStream(obj, name)

    def close(self):
        self.closed = True


class _FakeObs:
    def __init__(self, path):
        self.path, self.t0, self.on = path, None, False

    def start_recording(self, name, on_sample=None):
        self.t0, self.on = time.time(), True
        on_sample((time.time(), time.time(), 0.0))
        return "client", dict(t_req=self.t0, t_active=self.t0, prefix=name)

    def status_sample(self, c):
        t = time.time()
        return t, t + 0.001, math.floor((t - self.t0) / T) * T, self.on

    def stop_recording(self, c):
        self.on = False
        return self.path, dict(t_send=time.time(), t_recv=time.time())


def test_recording_runtime_with_fakes(tmp_path, monkeypatch):
    monkeypatch.setattr(obsrec, "REC_DIR", str(tmp_path))
    monkeypatch.setattr(obsrec, "MARKER", str(tmp_path / "marker.json"))
    monkeypatch.setattr(obsrec, "CLOCK_PERIOD", 0.2)
    monkeypatch.setattr(core, "_AL", str(tmp_path))
    fake = _FakeObs(str(tmp_path / "rec.mp4"))
    r = obsrec.Recording("orbit_GF", log=lambda *a: None, obs=fake, connect=_FakeConn).start()
    assert core.REC_OBS is r and os.path.exists(obsrec.MARKER)
    core.record_pause(True)                                                     # v3: no-op (records on)
    assert not os.path.exists(tmp_path / "rec.pause")
    time.sleep(1.0)
    assert r.stop() == fake.path and core.REC_OBS is None
    assert not os.path.exists(obsrec.MARKER)
    meta = json.load(open(r.meta_path))
    assert meta["video"] == fake.path and meta["timemap_rows"] >= 4             # >= 4 Hz
    tm = tmap.TimeMap.load(r.timemap_path)
    assert tm.u[-1] - tm.u[0] == pytest.approx(tm.w[-1] - tm.w[0], abs=0.15)
    vc = tmap.calibrate(eo.load_clock(r.clock_path))
    assert vc.info["n"] >= 10
    assert r.stop() == fake.path                                                # idempotent (atexit)


def test_record_pause_without_v3_is_unchanged(tmp_path, monkeypatch):
    monkeypatch.setattr(core, "_AL", str(tmp_path))
    assert core.REC_OBS is None
    core.record_pause(True)
    assert os.path.exists(tmp_path / "rec.pause")
    core.record_pause(False)
    assert not os.path.exists(tmp_path / "rec.pause")


def test_engine_switch_and_flight_script_gating(monkeypatch):
    monkeypatch.delenv("KSP_REC_ENGINE", raising=False)
    assert obsrec.engine() == "obs"
    monkeypatch.setenv("KSP_REC_ENGINE", "frames")
    assert obsrec.engine() == "frames"
    src = open(os.path.join(ROOT, "tools", "orbit_recover.py")).read()
    assert ('REC_OBS = os.environ.get("HOP_REC") == "full" and os.environ.get("KSP_REC_ENGINE", "obs").strip().lower()'
            ' != "frames"') in src
    assert "if REC_OBS:\n    from kspilot import obsrec" in src                 # imported only for HOP_REC=full
    assert src.count("obsrec.Recording(") == 3 and src.count("core.record_start(tag, v2=True)") == 2
    assert "elif frames:\n    print(\"video:\", core.record_stop(frames, tag, fast=2.0, tail_s=25.0))" in src
    assert "edit_obs.py" in src and "edit_video.py" in src
    for f in ("director.py", "landing.py", "core.py"):
        body = open(os.path.join(ROOT, "kspilot", f)).read()
        assert "REC_OBS = " not in body or f == "core.py"


def test_gap_phase_hook_only_from_the_v3_editor():
    m = dict(rec0=0.0, rec1=100.0, t0=None, entry=60.0, td=80.0, burns=[])
    assert sr._phase_of_gap(30.0, m) == "P7"
    m["gap_phase"] = "P9"
    assert sr._phase_of_gap(30.0, m) == "P9"
    assert "gap_phase" not in sr.mission_from_events(dict(ev=EV), 0.0, 1e9)


# ---------------------------------------------------------------------------------------------------------------------
# end to end: a synthetic OBS-like clip through tools/edit_obs.py
# ---------------------------------------------------------------------------------------------------------------------
def _frame_rgb(path, t, x, y):
    r = subprocess.run(["ffmpeg", "-loglevel", "error", "-ss", "%.3f" % t, "-i", path, "-frames:v", "1", "-vf",
                        "crop=1:1:%d:%d,format=rgb24" % (x, y), "-f", "rawvideo", "-"], capture_output=True)
    return tuple(r.stdout[:3])


def _mean_db(path, a, b):
    r = subprocess.run(["ffmpeg", "-nostats", "-ss", "%.3f" % a, "-t", "%.3f" % (b - a), "-i", path, "-vn", "-af",
                        "volumedetect", "-f", "null", "-"], capture_output=True, text=True)
    line = [x for x in r.stderr.splitlines() if "mean_volume" in x]
    return float(line[0].split("mean_volume:")[1].split()[0]) if line else None


@pytest.mark.skipif(shutil.which("ffmpeg") is None or ed.python_with_pil() is None, reason="needs ffmpeg + Pillow")
def test_editor_end_to_end_on_a_synthetic_obs_clip(tmp_path):
    base = str(tmp_path / "orbit_T_20261021-160000")
    video = base + ".mp4"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "lavfi", "-i", "color=c=0x203040:s=640x360:r=30",
                    "-f", "lavfi", "-i", "sine=frequency=440:sample_rate=48000", "-t", str(VDUR), "-c:v", "libx264",
                    "-preset", "ultrafast", "-pix_fmt", "yuv420p", "-c:a", "aac", "-shortest", video], check=True)
    with open(base + ".timemap.csv", "w") as f:
        tmap.write_header(f)
        for r in synth_rows():
            f.write(tmap.format_row(*r))
    with open(base + ".clock.csv", "w") as f:
        f.write("t_send,t_recv,duration\n")
        for a, b, d in synth_samples():
            f.write("%.4f,%.4f,%.4f\n" % (a, b, d))
    json.dump(dict(ev=EV, burns=[], cd=5000.0, t0=5000.0, period=1958.0, marks={}, planned={}, start="pad"),
              open(base + ".events.json", "w"))
    json.dump(dict(engine="obs", name="orbit_T", base=base, video=video, timemap=base + ".timemap.csv",
                   clock=base + ".clock.csv", events=base + ".events.json"), open(base + ".rec.json", "w"))
    out = str(tmp_path / "cut.mp4")
    r = subprocess.run([ed.python_with_pil(), os.path.join(ROOT, "tools", "edit_obs.py"), base + ".rec.json", "--out",
                        out], capture_output=True, text=True, env=dict(os.environ, KSP_REC_SYNC_S="0"))
    assert r.returncode == 0, r.stdout + r.stderr
    assert "checks: PASS" in r.stdout and "slowed to game 1x" in r.stdout, r.stdout
    j = json.loads(subprocess.run(["ffprobe", "-v", "error", "-show_entries", "stream=codec_name,width,height,"
                                   "avg_frame_rate,nb_frames,duration:format=duration", "-of", "json", out],
                                  capture_output=True, text=True).stdout)
    v = [s for s in j["streams"] if s["codec_name"] == "h264"][0]
    a = [s for s in j["streams"] if s["codec_name"] == "aac"][0]
    assert (v["width"], v["height"], v["avg_frame_rate"]) == (1920, 1080, "30/1")
    assert float(j["format"]["duration"]) == pytest.approx(87.0, abs=0.15)      # 33 + 10 + (4 + 40): entry - 11 s
    assert int(v["nb_frames"]) == pytest.approx(2610, abs=3)
    assert float(a["duration"]) == pytest.approx(87.0, abs=0.15)
    white = lambda p: min(p) > 170 and max(p) - min(p) < 40                     # noqa: E731  V16: white, no accent
    assert white(_frame_rgb(out, 38.0, 68, 132))                                # x50 badge ▶▶ in the lapse (33-43 s)
    assert not white(_frame_rgb(out, 20.0, 68, 132))                            # 1x: no badge
    assert not white(_frame_rgb(out, 60.0, 68, 132))
    assert white(_frame_rgb(out, 44.0, 69, 132))                                # REAL TIME (white dot) after it
    one_x = _mean_db(out, 5.0, 30.0)
    assert one_x > -40.0                                                         # 1x: the game sound
    bed = _mean_db(out, 34.0, 42.5)                                              # time-lapse: low bed, not silence
    assert one_x - 21.0 < bed < one_x - 15.0, (one_x, bed)                       # -18 dB (next video round)
    assert _mean_db(out, 50.0, 80.0) > -40.0
    loud = eo.measure_loudness(out)                                              # final mix: -16 LUFS, <= -1.5 dBTP
    assert abs(loud["I"] - eo.LOUD_I) <= eo.LOUD_TOL and loud["TP"] <= eo.LOUD_TP, loud
    assert "# loudness:" in open(str(tmp_path / "cut.edl.txt")).read()
    assert os.path.exists(str(tmp_path / "cut.edl.txt")) and os.path.exists(video)   # the master is kept
