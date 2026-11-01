"""Next video round (recording pipeline v3, HOP_REC=full only; video requirements V12-V15,
video QA report 2026-10-21): V14 orbiting drone camera (angle advance, look-at, sun-safe arc, the plugin's pure
math cross-checked against the Python mirror), V15 HUD data panels hidden before lift-off, V12 no big countdown text,
loudness normalisation (-16 LUFS, <= -1.5 dBTP) and the time-lapse sound bed, +1 s on the three 10 s windows.
No KSP; the loudness test needs ffmpeg (a 12 s synthetic audio clip), the C# cross-check needs dotnet."""
import json
import math
import os
import shutil
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
from kspilot import core, showrun as sr  # noqa: E402
import edit_obs as eo  # noqa: E402

# flight #5 recording (OBS sidecar files + the video itself): not distributed; point KSP_VIDEO_DIR at a local copy
REC5 = os.path.join(os.environ.get("KSP_VIDEO_DIR", os.path.join(ROOT, "videos")), "obs",
                    "orbit_T-WorksFineGF_20261021-161222.rec.json")


def sun(el, az):
    e, a = math.radians(el), math.radians(az)
    return (math.cos(e) * math.sin(a), math.cos(e) * math.cos(a), math.sin(e))


def parse_orbit(cam):
    p = cam.split()
    assert p[0] == "orbit" and len(p) == 9
    return [float(x) for x in p[1:]]


# ------------------------------------------------------------------------------------------- V14 orbiting drone
def test_orbit_angle_advances_at_the_rate_and_wraps():
    assert sr.drone_orbit_az(330.0, 10.0, 0.0) == pytest.approx(330.0)
    assert sr.drone_orbit_az(330.0, 10.0, 3.5) == pytest.approx(5.0)                  # wraps through north
    assert sr.drone_orbit_az(10.0, -4.0, 5.0) == pytest.approx(350.0)                 # counter-clockwise
    a = [sr.drone_orbit_az(100.0, 10.0, k * 0.1) for k in range(241)]
    steps = [((b - c + 180.0) % 360.0) - 180.0 for c, b in zip(a, a[1:])]
    assert all(s == pytest.approx(1.0) for s in steps)                                 # smooth: 1 deg / 0.1 s


def test_orbit_position_is_a_circle_round_the_pad():
    lat0, lon0 = core.KSC_PAD
    for az in (0.0, 90.0, 247.0, 330.0):
        lat, lon = sr.drone_orbit_latlon(lat0, lon0, az)
        n = math.radians(lat - lat0) * 600000.0
        e = math.radians(lon - lon0) * 600000.0 * math.cos(math.radians(lat0))
        assert math.hypot(e, n) == pytest.approx(sr.DRONE_ORBIT_R, abs=0.01)
        assert math.degrees(math.atan2(e, n)) % 360.0 == pytest.approx(az, abs=0.01)
    # the old fixed drone pose at the same azimuth is the same ground point (proven geometry kept)
    lat, lon, _, _, _, _ = sr.drone_pose(lat0, lon0, 70.0, 90.0, dist=sr.DRONE_ORBIT_R)
    assert (lat, lon) == pytest.approx(sr.drone_orbit_latlon(lat0, lon0, 90.0), abs=1e-12)


def test_orbit_looks_at_the_vessel_while_it_descends():
    assert sr.drone_target_up(0.0) == pytest.approx(sr.DRONE_H + sr.DRONE_COM_UP)
    assert sr.drone_target_up(sr.DRONE_TD_S) == sr.drone_target_up(30.0) == pytest.approx(sr.DRONE_COM_UP)
    for az in (0.0, 135.0, 270.0):
        lk = sr.drone_look(az, 0.0)
        a = math.radians(az)
        cam = (sr.DRONE_ORBIT_R * math.sin(a), sr.DRONE_ORBIT_R * math.cos(a), sr.DRONE_ORBIT_UP)
        # the view axis passes through the vessel above the pad centre (horizontal part points at the centre)
        assert math.degrees(math.atan2(-lk[0], -lk[1])) % 360.0 == pytest.approx(az, abs=1e-6)
        t = sr.DRONE_ORBIT_R / math.hypot(lk[0], lk[1])
        assert cam[2] + t * lk[2] == pytest.approx(sr.DRONE_H + sr.DRONE_COM_UP)
        assert sr.drone_look(az, 0.0)[2] > 0.0 > sr.drone_look(az, 10.0)[2]           # tilts down as it lands


@pytest.mark.parametrize("el", [2, 10, 20, 30, 45, 60, 75, 88])
@pytest.mark.parametrize("az", [0, 90, 200, 247, 315])
def test_orbit_plan_is_sun_safe_and_moves(el, az):
    s = sun(el, az)
    cam, info = sr.drone_orbit_shot(core.KSC_PAD[0], core.KSC_PAD[1], 70.0, s)
    clat, clon, alt, r, up, az0, rate, fov = parse_orbit(cam)
    assert (clat, clon, alt, r, up, fov) == pytest.approx((core.KSC_PAD[0], core.KSC_PAD[1], 70.0, sr.DRONE_ORBIT_R,
                                                          sr.DRONE_ORBIT_UP, sr.DRONE_FOV))
    assert info["mode"] == "sun-safe" and rate >= 4.0                                  # it orbits (V14)
    worst = min(sr.drone_sun_angle(sr.drone_orbit_az(az0, rate, 0.1 * k), 0.1 * k, s) for k in range(241))
    assert worst >= sr.SUN_MIN_ANGLE - 0.5                                             # whole shot, 0.1 s steps
    assert info["sun_az"] == pytest.approx(az % 360, abs=1e-6) and info["sun_el"] == pytest.approx(el, abs=1e-6)


def test_orbit_plan_prefers_the_nominal_rate_and_front_light_at_touchdown():
    s = sun(30.0, 120.0)
    az0, rate, info = sr.drone_orbit_plan(s)
    assert rate == sr.DRONE_ORBIT_RATE and info["anchor"] == "touchdown"
    td = sr.drone_orbit_az(az0, rate, sr.DRONE_TD_S)
    assert abs(((td - 120.0 - info["offset"] + 180.0) % 360.0) - 180.0) < 1e-6
    assert sr.drone_sun_angle(td, sr.DRONE_TD_S, s) > 90.0                             # sun behind the camera side
    az0, rate, info = sr.drone_orbit_plan(None)                                        # sun unknown: slow, pad side
    assert rate == sr.DRONE_ORBIT_RATES[-1] and info["mode"] == "no sun"
    assert sr.drone_orbit_az(az0, rate, sr.DRONE_ORBIT_S / 2) == pytest.approx(sr.DRONE_FALLBACK_AZ)
    assert sr.drone_orbit_plan(sun(-8.0, 90.0))[2]["mode"] == "sun below the horizon"
    assert "deg/s" in sr.drone_orbit_text(sr.drone_orbit_shot(0.0, 0.0, 70.0, s)[1])


def test_plugin_orbit_shot_source():
    cs = open(os.path.join(ROOT, "plugins", "AutoLab", "AutoLab.cs")).read()
    assert '|| kind == "fixed" || kind == "orbit";' in cs
    blk = cs.split('else if (kind == "orbit")', 1)[1].split('else if (kind == "rel")', 1)[0]
    assert "if (cut) orbitUt0 = now;" in blk                                           # t = 0 at the cut
    assert "RecMath.OrbitAz(PD(shot, 6, 0), PD(shot, 7, 10), now - orbitUt0)" in blk   # per frame, game time
    assert "RecMath.OrbitLatLon(clat, clon, v.mainBody.Radius, PD(shot, 4, 70), az, out lat, out lon)" in blk
    assert "PD(shot, 3, 0) + PD(shot, 5, 30)" in blk
    assert "look = (Vector3)v.CoM - pos;" in blk                                       # always aimed at the vessel
    assert "fov = P(shot, 8, 50f);" in blk and "Ease(" not in blk                     # fixed fov: no zoom
    lsrc = open(os.path.join(ROOT, "kspilot", "landing.py")).read()
    cam = lsrc.split("    def _camera_v2(self, ut, r):", 1)[1].split("    def _sun_rel(", 1)[0]
    assert "self._drone, info = showrun.drone_orbit_shot(lat0, lon0, pdg.PAD_ALT, s)" in cam   # rec_v2 path only
    assert "drone_orbit" not in lsrc.replace(cam, "")
    assert "if self.rec_v2:\n            sid, want = self._camera_v2(ut, r)" in lsrc


# ------------------------------------------------------------------------------------------- V15 / V12 HUD
def test_panel_reveal_hidden_before_liftoff_only_in_a_recording():
    assert sr.panel_reveal(False, 100.0, None, 50.0) == 1.0                           # no recording: unchanged
    assert sr.panel_reveal(True, None, None, 50.0) == 1.0                             # recording, no countdown
    assert sr.panel_reveal(True, 100.0, None, 50.0) == 0.0                            # countdown: hidden
    assert sr.panel_reveal(True, 100.0, None, 100.5) == 0.0                           # until the lift-off is known
    assert sr.panel_reveal(True, 100.0, 100.0, 100.0) == 0.0
    assert sr.panel_reveal(True, 100.0, 100.0, 100.4) == pytest.approx(0.5)
    assert sr.panel_reveal(True, 100.0, 100.0, 100.0 + sr.PANEL_IN) == 1.0
    assert sr.panel_reveal(True, 100.0, 100.0, 5000.0) == 1.0
    v = [sr.panel_reveal(True, 0.0, 0.0, 0.01 * k) for k in range(100)]
    assert all(b >= a for a, b in zip(v, v[1:]))                                       # monotonic slide / fade in


def test_hud_source_hides_panels_keeps_arc_and_clock_no_big_countdown():
    cs = open(os.path.join(ROOT, "plugins", "AutoLab", "HudV2.cs")).read()
    draw = cs.split("public static void Draw(", 1)[1].split("static GUIStyle sCamTelS;", 1)[0]
    assert 'float pPan = RecMath.PanelReveal(v8 && S(kv, "rec") == "full", haveCdV, haveT0V, UT, t0V, PANEL_IN);' in draw
    assert "const float PANEL_IN = 0.8f;" in cs and sr.PANEL_IN == 0.8
    # timeline arc + clock drawn unconditionally, before the gated panels (V15 leftover: after pPan is known, because
    # TimelineArc reads gPan to hide its NEXT block before lift-off - the arc itself is never gated)
    i_arc = draw.index("TimelineArc(kv, paint);")
    i_gate = draw.index("if (pPan > 0f)")
    i_pan = draw.index("float pPan = RecMath.PanelReveal(")
    assert i_pan < draw.index("gPan = pPan;") < i_arc < i_gate
    assert "if (" not in draw[i_pan:i_arc] and "if (pPan" not in draw[:i_arc]
    # V15 leftover: the NEXT block is a data panel - hidden before lift-off, fades / slides in with the panels
    arc = cs.split("static void TimelineArc(", 1)[1]
    assert "static float gPan = 1f;" in cs
    assert "bool showNext = nx != null && gPan > 0f;" in arc and "if (showNext) boxes.Add(" in arc
    nb = arc[arc.index("if (showNext)\n"):]
    nb = nb[:nb.index("gA = keepA; gOff = keepO;")]
    assert "gA = keepA * gPan;" in nb and 'Sp("NEXT", 2)' in nb
    assert "if (nx != null)" not in arc
    rows = draw[i_gate:draw.index("gOff = Vector2.zero;", i_gate)]
    assert "Rows(kv, true, paint);" in rows and "Rows(kv, false, paint);" in rows and "gA = alpha * pPan;" in rows
    assert 'if (S(kv, "cam").Length > 0 && pPan > 0f)' in draw and "gA = alpha * ca * pPan;" in draw
    assert "if (v8 && pPan > 0f) { gA = alpha * pPan; Widgets(kv, paint, pOut); gA = alpha; }" in draw
    assert draw.count("Rows(kv,") == 2 and draw.count("Widgets(kv,") == 1 and draw.count("CamCaption(") == 1
    # V12: nothing draws the big 10..1 digits or the LIFTOFF word (the dead code is gone)
    assert "Countdown(" not in cs and "sBig" not in cs and '"LIFTOFF"' not in cs
    assert "LIFTOFF" not in open(os.path.join(ROOT, "plugins", "AutoLab", "AutoLab.cs")).read()
    for f in ("edit_obs.py", "edit_video.py"):                                         # nor the editors' overlays
        src = open(os.path.join(ROOT, "tools", f)).read().lower()
        assert "liftoff" not in src and "countdown" not in src


@pytest.mark.skipif(shutil.which("dotnet") is None, reason="needs dotnet")
def test_plugin_recmath_matches_the_python_mirror(tmp_path):
    proj = os.path.join(ROOT, "tests", "cs_recmath", "recmath.csproj")
    r = subprocess.run(["nice", "-n", "10", "dotnet", "run", "--project", proj, "-c", "Release"],
                       capture_output=True, text=True, timeout=300,
                       env=dict(os.environ, DOTNET_CLI_TELEMETRY_OPTOUT="1", DOTNET_NOLOGO="1"))
    assert r.returncode == 0, r.stdout + r.stderr
    lines = [l.split() for l in r.stdout.splitlines() if l[:3] in ("az ", "ll ", "pr ")]
    az = [float(l[1]) for l in lines if l[0] == "az"]
    ll = [(float(l[1]), float(l[2])) for l in lines if l[0] == "ll"]
    pr = [float(l[1]) for l in lines if l[0] == "pr"]
    for (a0, rt, t), v in zip([(330, 10, 0), (330, 10, 3.5), (350, 10, 24), (10, -4, 5), (0, 10, -1)], az):
        assert v == pytest.approx(sr.drone_orbit_az(a0, rt, t), abs=1e-9)
    assert len(az) == 5 and len(ll) == 4 and len(pr) == 7
    for a, (la, lo) in zip((0.0, 90.0, 247.0, 330.0), ll):
        assert (la, lo) == pytest.approx(sr.drone_orbit_latlon(-0.0972079, -74.557673, a), abs=1e-12)
    cases = [(False, 100.0, None, 100.0), (True, None, None, 100.0), (True, 100.0, None, 100.0),
             (True, 100.0, 100.0, 100.0), (True, 100.0, 100.0, 100.4), (True, 100.0, 100.0, 100.8),
             (True, 100.0, 100.0, 5000.0)]
    for (rec, cd, t0, ut), v in zip(cases, pr):
        assert v == pytest.approx(sr.panel_reveal(rec, cd, t0, ut), abs=1e-6)


# ------------------------------------------------------------------------------------------- audio
def test_lapse_and_skip_get_a_low_tempo_limited_bed():
    P = eo.Piece
    plan = [sr.Seg("P1", "real", 0, 10, 1.0, 10.0), sr.Seg("P3", "lapse", 10, 410, 40.0, 10.0),
            sr.Seg("P7", "skip", 410, 9000, None, 2.5), sr.Seg("P10", "real", 9000, 9030, 1.0, 30.0)]
    segs = [(0, [P(1.0, 11.0, 10.0, "copy")], 300), (1, [P(11.0, 15.0, 4.0, "lapse"), P(15.0, 71.0, 6.0, "lapse")], 300),
            (2, [P(71.0, 620.0, 2.5, "skip")], 75), (3, [P(620.0, 650.0, 30.0, "copy")], 900)]
    assert eo.LAPSE_AUDIO == "bed" or os.environ.get("KSP_REC_LAPSE_AUDIO")       # the default is the bed
    ap = eo.audio_plan(plan, segs, lapse_audio="bed")
    assert [a[0] for a in ap] == ["keep", "bed", "bed", "keep"]
    assert eo.bed_tempo(segs[1][1], 10.0) == eo.BED_TEMPO_MAX == 2.0                 # x6 source: capped at x2
    assert eo.bed_tempo([P(0.0, 5.0, 10.0, "lapse")], 10.0) == pytest.approx(0.5)     # never reads past its span
    cmd = eo.audio_cmd("in.mp4", ap, "mix.wav")
    g = cmd[cmd.index("-filter_complex") + 1]
    assert cmd.count("-i") == 4 and cmd[cmd.index("-c:a") + 1] == "pcm_f32le"
    assert g.count("volume=-18.0dB") == 2 and g.count("acrossfade=d=0.200000") == 3   # 0.2 s crossfades kept
    assert "atempo=2.000000,volume=-18.0dB" in g and "aevalsrc" not in g
    j = g.index("[1:a]atrim=start=0.000000:end=%s" % eo._f(10.2 * 2.0))               # 10.2 s out at x2 from the start
    assert "apad=whole_dur=10.200000,atrim=end=10.200000[a1]" in g[j:]
    assert eo.audio_plan(plan, segs, lapse_audio="mute")[1][0] == "mute"              # the old modes stay
    assert eo.audio_cmd("in.mp4", ap, "a.m4a")[-5:-1] == ["-c:a", "aac", "-b:a", "192k"]


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="needs ffmpeg")
def test_loudness_normalised_to_minus16_lufs_true_peak_below_minus1_5(tmp_path):
    """12 s synthetic mix (quiet engine-like noise + a sine + two full-scale bursts: QA v5 had -20.4 LUFS and
    +0.2 dBTP): -16 LUFS +- 0.5, true peak <= -1.5 dBTP, length and timing unchanged."""
    src = str(tmp_path / "mix.wav")
    expr = ("0.05*sin(2*PI*220*t)+0.04*(random(0)-0.5)+"
            "if(between(t,3,3.05)+between(t,8,8.02),0.999*sin(2*PI*90*t),0)")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "lavfi", "-i",
                    "aevalsrc='%s|%s':s=48000:d=12" % (expr, expr), "-c:a", "pcm_f32le", src], check=True)
    m0 = eo.measure_loudness(src)
    assert m0["I"] < -20.0 and m0["TP"] > -1.0                                         # quiet and peaky, like v5
    out = str(tmp_path / "audio.m4a")
    info = eo.loudness_normalise(src, out, log=lambda *a: None)
    m = eo.measure_loudness(out)
    assert info["ok"] and abs(m["I"] - eo.LOUD_I) <= eo.LOUD_TOL and m["TP"] <= eo.LOUD_TP, (m, info)
    dur = lambda p: float(subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of",  # noqa
                                         "csv=p=0", p], capture_output=True, text=True).stdout)
    assert dur(out) == pytest.approx(12.0, abs=0.05)
    assert "PASS" in eo.loudness_text(info)
    # the limiter is delay-compensated: the first burst starts where it did (AAC priming / frame slack)
    def onset(p):
        r = subprocess.run(["ffmpeg", "-loglevel", "error", "-i", p, "-ac", "1", "-ar", "48000", "-f", "f32le", "-"],
                           capture_output=True)
        import array
        a = array.array("f")
        a.frombytes(r.stdout[:4 * 48000 * 4])
        i0, i1 = int(2.8 * 48000), int(3.2 * 48000)
        top = max(abs(x) for x in a[i0:i1])
        return next(i for i in range(i0, i1) if abs(a[i]) > 0.6 * top) / 48000.0
    assert onset(out) == pytest.approx(onset(src), abs=0.025)


# ------------------------------------------------------------------------------------------- +1 s window margins
def test_three_ten_second_windows_get_one_second_of_margin():
    m = dict(rec0=0.0, rec1=30000.0, cd=1000.0, t0=1000.0, meco=1180.0, orbit=(1400.0, 1420.0), sep=1460.0,
             deorbit=(20000.0, 20010.0), entry=21000.0, td=21200.0, lander=20500.0, burns=[])
    w = {p: (a, b) for p, a, b in sr.real_windows(m)}
    p4 = next(v for k, v in w.items() if k.startswith("P4"))
    assert p4[0] == pytest.approx(1400.0 - 11.0) and w["P6"][1] == pytest.approx(1460.0 + 11.0)
    assert w["P10"][0] == pytest.approx(21000.0 - 11.0)
    assert w["P8"][0] == pytest.approx(20000.0 - sr.PRE_BURN)                          # deorbit unchanged
    m2 = dict(m, sep=1600.0)
    w2 = {p: (a, b) for p, a, b in sr.real_windows(m2)}
    assert w2["P6"][1] == pytest.approx(1600.0 + 11.0)
    assert sr.check_plan(sr.playback_plan(m), m) == []


@pytest.mark.skipif(not os.path.exists(REC5), reason="needs the flight #5 OBS recording, which is not distributed (set KSP_VIDEO_DIR)")
def test_window_margins_on_flight5_edl():
    """The EDL builder on the real flight #5 recording (logs/wd_20261021_161137.out, video 16:35): every 10 s window
    now has >= 11 s at 1x, the checks still pass and nothing inside the wider windows was flown on rails warp."""
    ctx = eo.load(REC5)
    m, plan, segs, problems, notes = eo.build(ctx)
    assert problems == []
    ign, sep, entry = m["orbit"][0], m["sep"], m["entry"]

    def real_cover(a, b):
        return all(s.kind == "real" for s in plan if s.kind != "drop" and s.ut0 < b - 1e-6 and s.ut1 > a + 1e-6)
    assert real_cover(ign - 11.0, ign) and real_cover(sep, sep + 11.0) and real_cover(entry - 11.0, entry)
    for a, b in ((ign - 11.0, ign), (sep, sep + 11.0), (entry - 11.0, entry)):
        assert all(r[2] != "rails" for r in ctx.tm.runs(ctx.tm.wall(a), ctx.tm.wall(b)))
    # the video grew by exactly the three seconds (QA v5: 15685 frames)
    assert sum(n for _, _, n in segs) == pytest.approx(15685 + 3 * 30, abs=3)
