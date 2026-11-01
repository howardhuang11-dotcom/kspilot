"""Recording pipeline v2, fix round 3 (video QA report 2026-10-21, decision 13:00). Offline only:
  1  sun-aware rel cameras (director + lander): synthetic sun vectors, handedness, shadow, 'sun' token, eased swings
  2  deorbit burn widget (V16: HUD-style line art, white only, no panel, no shadow lift)
  3  V4: 1x from 12 s before the earliest release UT (HUD stamp / commanded / announced), announced 16 s ahead
  4  V8: insertion burn shots split so none passes 15 s with the ORBIT-icon hold (flight #3 replay: S4 was 17 s)
  5  MECO close-up >= 15 % of the frame width
  6  separation metric: 15 s GAME window, KSP.log re-read after a late flush, 3 s / 15 s, booster vs core
  7  HUD NEXT label during a burn (rec=full only)
and the non-recording guarantee for every new path."""
import json
import math
import os
import subprocess
import sys
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
from kspilot import ascent_own as A, core, director as d, showrun as sr  # noqa: E402
import director_replay as rp  # noqa: E402
import edit_video as ed  # noqa: E402


def sph(az_deg, el_deg):
    """Unit vector in ENU from an azimuth (deg from north, clockwise) and an elevation."""
    a, e = math.radians(az_deg), math.radians(el_deg)
    return (math.cos(e) * math.sin(a), math.cos(e) * math.cos(a), math.sin(e))


def geo(sun, vel=(0.0, 1500.0, 0.0), alt=30000.0):
    return dict(up=(0.0, 0.0, 1.0), vsrf=vel, vobt=vel, sun=sun, alt=alt, R=600000.0, north=(0.0, 1.0, 0.0))


# ------------------------------------------------------------------------------------- 1. sun-aware rel cameras
def test_rel_frame_is_the_plugins_frame():
    """fwd = horizontal velocity, right = its PHYSICAL right (heading north -> east), az 90 = right side."""
    f, r, u = sr.rel_basis(geo((1, 0, 0)))
    assert f == pytest.approx([0, 1, 0]) and r == pytest.approx([1, 0, 0]) and u == pytest.approx([0, 0, 1])
    f, r, u = sr.rel_basis(geo((1, 0, 0), vel=(800.0, 0.0, -50.0)))      # heading east: right = south
    assert f == pytest.approx([1, 0, 0]) and r == pytest.approx([0, -1, 0])
    assert sr.cam_offset(0, 0) == pytest.approx((-1, 0, 0))               # az 0: behind
    assert sr.cam_offset(0, 90) == pytest.approx((0, 1, 0), abs=1e-12)    # az 90: right
    assert sr.sun_angle(0, 0, (1, 0, 0)) == pytest.approx(0.0, abs=1e-6)  # rear chase, sun dead ahead
    assert sr.sun_angle(0, 180, (1, 0, 0)) == pytest.approx(180.0)        # camera between the sun and the stage
    # the plugin's 'sun' token: az from the sun's horizontal direction, atan2(s.right, -s.fwd) (AutoLab.cs)
    s = sr.sun_in_rel(geo(sph(45, 0)))                                     # sun ahead-right
    assert math.degrees(math.atan2(s[1], -s[0])) == pytest.approx(135.0)


def test_qa_flight3_entry_chase_into_the_sun_is_moved():
    """QA 4.1: chase from behind, the sun ahead of the stage -> black disc. The camera moves to >= 90 deg (front-lit),
    keeps its distance / lens / tokens, and the move is logged."""
    g = geo(sph(90, 25), vel=(1700.0, 190.0, -195.0), alt=37400.0)          # entry heading ~east, sun east 25 deg up
    out, info = sr.sun_safe_rel("rel 60 10 0 55 cut", g)
    assert info["angle0"] < 40.0 and info["angle"] >= sr.SUN_PREF_ANGLE and info["changed"]   # was in the picture
    p = sr.parse_rel(out)
    assert p[0] == 60 and p[3] == 55 and p[4] == ["cut"]
    assert sr.sun_angle(p[1], p[2], sr.sun_in_rel(g)) == pytest.approx(info["angle"])
    assert "-> " in sr.sun_info_text("S8", info)


@pytest.mark.parametrize("cam", ["rel 45 6 25 55", "rel 60 8 90 55", "rel 50 -5 160 55 cut", "rel 60 10 50 55 cut",
                                 "rel 50 70 20 60 cut", "rel 24 6 70 45", "rel 16 8 125 60 cut", "rel 60 20 20 50"])
def test_every_rel_shot_keeps_60_deg_from_the_sun(cam):
    """Synthetic sun grid (every 15 deg of azimuth, -5..85 deg of elevation, three headings, sunlit): the chosen
    camera is always >= 60 deg from the sun, the nominal is kept when it already is >= 90 deg."""
    for hdg in (0.0, 90.0, 237.0):
        vel = tuple(1500.0 * x for x in sph(hdg, 0))
        for el in (-5, 5, 20, 45, 85):
            for az in range(0, 360, 15):
                g = geo(sph(az, el), vel=vel)
                out, info = sr.sun_safe_rel(cam, g, side=cam.startswith("rel 24 "))
                assert info["angle"] >= sr.SUN_MIN_ANGLE - 1e-6, (cam, hdg, az, el, info)
                if info["angle0"] >= sr.SUN_PREF_ANGLE and not cam.startswith("rel 24 "):
                    assert out == cam
                p = sr.parse_rel(out)
                assert p[0] == sr.parse_rel(cam)[0] and p[4] == sr.parse_rel(cam)[4]


def test_shadow_and_sun_token_are_kept():
    up_shadow = geo(sph(0, -80), alt=100000.0)                     # the sun below the stage: behind the planet
    assert sr.sun_occluded(up_shadow)
    out, info = sr.sun_safe_rel("rel 16 8 125 60 cut", up_shadow)
    assert out == "rel 16 8 125 60 cut" and info["mode"] == "shadow"
    assert not sr.sun_occluded(geo(sph(0, -10), alt=100000.0))     # 100 km up sees the sun 10 deg below the horizon
    out, info = sr.sun_safe_rel("rel 18 10 35 50 sun cut", geo(sph(30, 20)))
    assert out == "rel 18 10 35 50 sun cut" and info["mode"] == "sun-token" and info["angle"] > 120.0
    assert sr.sun_safe_rel("onboard 0 3 2.2 75", geo(sph(0, 0))) == ("onboard 0 3 2.2 75", None)
    assert sr.sun_safe_rel("rel 60 8 0 55", None) == ("rel 60 8 0 55", None)


def test_sun_geometry_is_handedness_free():
    """core.sun_geometry on a fake LEFT-handed body frame (kRPC / Unity: x = prime meridian, y = north pole,
    z = 90 deg E - a mirror of the physical frame) returns the physical east / north / up components of the sun and
    of the velocity, and the orbital velocity = surface velocity + the rotation eastward."""
    from types import SimpleNamespace as NS
    R, lat, lon, alt, om = 600000.0, 10.0, 30.0, 50000.0, 2 * math.pi / 21549.425

    def phys(la, lo, r):
        a, b = math.radians(la), math.radians(lo)
        return (r * math.cos(a) * math.cos(b), r * math.cos(a) * math.sin(b), r * math.sin(a))

    k = lambda p: (p[0], p[2], p[1])                                # noqa: E731  physical -> left-handed kRPC
    a, b = math.radians(lat), math.radians(lon)
    up = (math.cos(a) * math.cos(b), math.cos(a) * math.sin(b), math.sin(a))
    east = (-math.sin(b), math.cos(b), 0.0)
    north = (-math.sin(a) * math.cos(b), -math.sin(a) * math.sin(b), math.cos(a))
    comb = lambda c: tuple(c[0] * east[i] + c[1] * north[i] + c[2] * up[i] for i in range(3))   # noqa: E731
    s_enu, v_enu = sph(250, 20), (300.0, 2100.0, -40.0)
    p = phys(lat, lon, R + alt)
    sun_p = tuple(p[i] + 1.3e10 * comb(s_enu)[i] for i in range(3))
    body = NS(reference_frame="rf", rotational_speed=om, equatorial_radius=R,
              surface_position=lambda la, lo, rf: k(phys(la, lo, R)))
    v = NS(orbit=NS(body=body), position=lambda rf: k(p), velocity=lambda rf: k(comb(v_enu)))
    sc = NS(bodies={"Sun": NS(position=lambda rf: k(sun_p))})
    g = core.sun_geometry(sc, v, lat, lon)
    assert g["sun"] == pytest.approx(s_enu, abs=1e-6) and g["vsrf"] == pytest.approx(v_enu, abs=1e-6)
    rot = om * (R + alt) * math.cos(a)
    assert g["vobt"] == pytest.approx((v_enu[0] + rot, v_enu[1], v_enu[2]), abs=1e-3)
    assert g["alt"] == pytest.approx(alt)


def test_lander_geometry_far_from_the_pad():
    """Flight #3 hand-off: 393 km west of the pad, 77.6 km up, 1703 m/s east: the local up leans ~37 deg west, the
    orbital velocity adds the planet's rotation (~175 m/s east)."""
    g = sr.lander_geo((-393294.9, -45111.5, 77578.8), (1702.85, 190.93, -193.66), sph(90, 30), R0=600070.0)
    tilt = math.degrees(math.acos(g["up"][2]))
    assert tilt == pytest.approx(math.degrees(math.asin(math.hypot(393294.9, 45111.5) / (600070.0 + 77578.8))),
                                 abs=0.01)
    assert g["vobt"][0] - g["vsrf"][0] == pytest.approx(2 * math.pi / 21549.425 * 677e3 * math.cos(math.radians(tilt)),
                                                        rel=0.05)
    f, r, u = sr.rel_basis(g)
    assert abs(sum(a * b for a, b in zip(f, u))) < 1e-9 and abs(sum(a * b for a, b in zip(r, u))) < 1e-9


def test_eased_swing_through_the_sun_is_detected():
    s = (1.0, 0.0, 0.0)                                            # sun dead ahead on the horizon
    assert sr.ease_min_angle((8, -90), (8, 90), s) < 30.0          # -90 -> 90 sweeps through the rear (az 0)
    assert sr.ease_min_angle((8, 90), (8, 120), s) >= 89.0


class FakeSC:
    active_vessel = object()


def test_director_sun_camera_frozen_per_shot_and_logged(monkeypatch):
    calls, lines = [], []
    g = geo(sph(0, 5))                                            # sun ahead (north), velocity north
    monkeypatch.setattr(core, "sun_geometry", lambda sc, v, lat, lon: calls.append(1) or g)
    D = d.Director.__new__(d.Director)
    D.log, D._latlon = lines.append, (0.0, 0.0)
    a = D._sun_cam(FakeSC(), "S2a", "rel 45 6 25 55", True, 100.0)
    assert a != "rel 45 6 25 55" and len(calls) == 1 and "S2a sun angle" in lines[-1]
    for k in range(20):                                           # same shot, same string: no RPC, no change
        assert D._sun_cam(FakeSC(), "S2a", "rel 45 6 25 55", False, 100.0 + k) == a
    assert len(calls) == 1
    b = D._sun_cam(FakeSC(), "S2b", "rel 60 8 90 55", True, 130.0)
    p0, p1 = sr.parse_rel(a), sr.parse_rel(b)
    s = sr.sun_in_rel(g)
    ok = sr.ease_min_angle((p0[1], p0[2]), (p1[1], p1[2]), s) >= sr.SUN_MIN_ANGLE
    assert ("cut" in b.split()) == (not ok)                        # hard cut only when the swing would pass the sun
    assert D._sun_cam(FakeSC(), "ONB", "onboard 0 6 2.5 70", True, 140.0) == "onboard 0 6 2.5 70"
    # the orbital bridge (no 'cut' token) re-reads the geometry after SUN_REFRESH game s
    D._sun_cam(FakeSC(), "BRIDGE", "rel 60 20 20 50", True, 200.0)
    n = len(calls)
    D._sun_cam(FakeSC(), "BRIDGE", "rel 60 20 20 50", False, 200.0 + d.SUN_REFRESH + 1.0)
    assert len(calls) == n                                         # < 2 wall s since the last read (a warp)
    D._sun_wall -= 3.0
    D._sun_cam(FakeSC(), "BRIDGE", "rel 60 20 20 50", False, 200.0 + d.SUN_REFRESH + 1.0)
    assert len(calls) == n + 1
    D._sun_cam(FakeSC(), "S4b", "rel 50 15 25 50 cut", True, 300.0)
    n = len(calls)
    D._sun_cam(FakeSC(), "S4b", "rel 50 15 25 50 cut", False, 300.0 + 10 * d.SUN_REFRESH)
    assert len(calls) == n                                         # a 'cut' shot never changes mid-shot


def test_director_geometry_failure_keeps_the_nominal(monkeypatch):
    lines = []

    def boom(*a):
        raise RuntimeError("no rpc")
    monkeypatch.setattr(core, "sun_geometry", boom)
    D = d.Director.__new__(d.Director)
    D.log, D._latlon = lines.append, (0.0, 0.0)
    assert D._sun_cam(FakeSC(), "S2a", "rel 45 6 25 55", True, 1.0) == "rel 45 6 25 55"
    assert "unavailable" in lines[-1]


def test_lander_sun_rel_once_per_shot(monkeypatch):
    from kspilot.landing import Lander
    calls, lines = [], []
    monkeypatch.setattr(core, "sun_enu", lambda conn, lat, lon: calls.append(1) or sph(90, 25))
    L = Lander.__new__(Lander)
    L.log, L.conn, L.target = lines.append, None, core.KSC_PAD
    L.v_true_enu = (1700.0, 190.0, -195.0)
    r = (-60000.0, -7000.0, 37400.0)
    a = L._sun_rel("S8", "rel 60 10 50 55 cut", 1000.0, r)
    assert a.endswith(" cut") and "S8 sun angle" in lines[-1] and len(calls) == 1
    g = sr.lander_geo(r, L.v_true_enu, sph(90, 25), R0=sr.R_KERBIN + __import__("kspilot.powered_descent",
                                                                                 fromlist=["x"]).PAD_ALT)
    p = sr.parse_rel(a)
    assert sr.sun_angle(p[1], p[2], sr.sun_in_rel(g)) >= sr.SUN_MIN_ANGLE
    for k in range(10):
        assert L._sun_rel("S8", "rel 60 10 50 55 cut", 1000.0 + k, r) == a
    L._sun_rel("SIDE", "rel 40 0 90 55 cut", 1010.0, r)
    assert len(calls) == 1                                          # the sun direction is cached SUN_REFRESH s
    L._sun_rel("FRONT", "rel 50 -5 160 55 cut", 1000.0 + L.SUN_REFRESH + 1.0, r)
    assert len(calls) == 2
    assert L._sun_rel("D2", "ground 1 2 3 14", 1100.0, r) == "ground 1 2 3 14"


def test_lander_v2_cameras_are_rel_and_hard_cut():
    src = open(os.path.join(ROOT, "kspilot", "landing.py")).read()
    body = src.split("def _camera_v2(", 1)[1].split("\n    def ", 1)[0]
    assert '"S8": "rel 60 10 50 55 cut"' in body and "want = self._sun_rel(sid, want, ut, r)" in body
    assert src.count("self._sun_rel(") == 1                        # only inside _camera_v2 (rec_v2 only)


# ------------------------------------------------------------------------------------- 5. MECO close-up
def test_meco_close_up_fills_15_percent():
    """Calibration: flight #3 S2e (60 m, el 8, az 15, 55 deg lens) measured ~5 % of the width (QA qa_v3_07); a 6 m
    + 2.5 m stage model reproduces it. The new S2e, wherever the sun solver puts it (side-locked), is >= 15 %."""
    L, D = 6.0, 2.5
    assert sr.stage_frame_fraction(60, 8, 15, 55, L, D) == pytest.approx(0.05, abs=0.01)
    dist, el, az, fov, toks = sr.parse_rel(d.V2_CAM["S2e"](None))
    assert sr.stage_frame_fraction(dist, el, az, fov, L, D) >= 0.20
    worst = 1.0
    for hdg in (90.0,):
        for sel in (-5, 8, 30, 60):
            for saz in range(0, 360, 10):
                out, info = sr.sun_safe_rel(d.V2_CAM["S2e"](None), geo(sph(saz, sel), vel=(1500.0, 0, 300.0)),
                                            side=True)
                p = sr.parse_rel(out)
                worst = min(worst, sr.stage_frame_fraction(p[0], p[1], p[2], p[3], L, D))
                assert info["angle"] >= sr.SUN_MIN_ANGLE
    assert worst >= 0.15, worst


# ------------------------------------------------------------------------------------- 3. V4 pre-release window
def test_v4_window_from_the_earliest_release_ut():
    """Flight #3: announced 104987.6, HUD event 104989.0 (logged after the staging + a 1.5 s wall wait). The 1x
    window now opens 12 s before the earliest of them (was +-10 s around the HUD stamp: 8.6 s before the plan)."""
    base = dict(rec0=103000.0, rec1=125000.0, t0=103110.38, cd=103110.4, meco=103289.96,
                orbit=(103498.88, 103512.02), sep=104989.0, deorbit=(124350.58, 124360.86), entry=124724.0,
                td=124930.0, burns=[(103498.88, 103512.02), (124350.58, 124360.86)])
    w = {p: (a, b) for p, a, b in sr.real_windows(dict(base, sep_plan=104987.6))}
    assert w["P6"] == pytest.approx((104987.6 - 12.0, 104989.0 + 10.0 + sr.WIN_MARGIN))   # + 1 s (QA v5)
    for late in (0.0, 1.4, 3.0):                                    # the HUD stamp up to 3 s late: still 12 s
        m = dict(base, sep=104987.6 + late, sep_cmd=104987.6)
        a, b = next((a, b) for p, a, b in sr.real_windows(m) if p == "P6")
        assert 104987.6 - a >= 12.0 - 1e-9 and b - (104987.6 + late) >= 10.0 - 1e-9
    assert sr.PRE_SEP_LEAD >= sr.SEP_PRE + 3.0                      # announced well before the window opens
    plan = sr.playback_plan(dict(base, sep_plan=104987.6))
    assert sr.check_plan(plan, dict(base, sep_plan=104987.6)) == []
    bad = sr.badge_compressed(plan, [(104987.6 - 11.0, 104987.6 - 9.0, 10)])   # warp still on 11 s before
    assert any("V4" in x for x in sr.check_plan(bad, dict(base, sep_plan=104987.6)))


def test_mission_from_events_reads_the_commanded_and_planned_release():
    evj = dict(ev={"LIFTOFF": [100.0, True], "PAYLOAD SEP": [2001.4, True]}, burns=[],
               planned={"PAYLOAD SEP": [2000.0, 1984.0]}, sep_cmd=2000.1)
    m = sr.mission_from_events(evj, 90.0, 3000.0)
    assert m["sep_plan"] == 2000.0 and m["sep_cmd"] == 2000.1 and sr.sep_start(m) == 2000.0
    evj["planned"] = {"PAYLOAD SEP": [1500.0, 1400.0]}              # a stale plan is not the release
    del evj["sep_cmd"]
    m = sr.mission_from_events(evj, 90.0, 3000.0)
    assert "sep_plan" not in m and sr.sep_start(m) == 2001.4


def test_flight_script_release_announcement_rec_full_only():
    src = open(os.path.join(ROOT, "tools", "orbit_recover.py")).read()
    blk = src.split("if REC_FULL and m.payload_stage is not None:", 1)[1].split("v = m.release_payload()", 1)[0]
    assert "rails_warp_factor == 0" in blk and '_rec_extra["sep_cmd"] = sc.ut' in blk
    assert "showrun.PRE_SEP_LEAD" in blk
    for line in src.splitlines():                                   # every _rec_extra write sits in a REC_FULL block
        if "_rec_extra[" in line or "_rec_extra.update(" in line:
            i = src.splitlines().index(line)
            ctx = "\n".join(src.splitlines()[max(0, i - 12):i])
            assert "if REC_FULL" in ctx, line
    assert "planned={k: list(x) for k, x in _feed.planned.items()}" in src


# ------------------------------------------------------------------------------------- 4. V8 insertion split
def _ins_run(burn, oe_delay, node_offset=0.0):
    """Insertion with a `burn` s burn centred on the node (+ node_offset: the executor's tail), ORBIT event reported
    oe_delay s after the director sees the cutoff (negative: before). Returns the cut list."""
    c = d.CutterV2()
    node = 1000.0
    ign, cut = node - burn / 2.0, node + burn / 2.0 + node_offset
    cuts, ut = [], node - 40.0
    while ut < cut + 40.0:
        ev = {"MECO": (0.0, True)}
        ev["ORBIT"] = (cut + oe_delay, True) if ut >= cut + oe_delay else (node, False)
        burning = ign + 1.0 < ut < cut
        st = d.State(ut, 500.0, ev, 1e5, burning, False, 0, True, 4e5, 30.0, 90.0, True,
                     cut if ut >= cut else None, False, ign if ign <= ut < cut else None)
        for a in c.step(st, sr.transitions(ev, None, v2=True)):
            if a[0] == "cut":
                cuts.append((ut, a[2]))
        ut = round(ut + 0.25, 4)
    return cuts + [(cut + 25.0, "END")]


@pytest.mark.parametrize("burn,oe_delay,tail", [(4.0, 0.0, 0.0), (7.6, -0.6, 0.0), (11.0, -0.6, 2.0),
                                                (13.0, 0.0, 0.0), (20.0, -0.6, 0.0), (30.0, 0.5, 1.0),
                                                (9.0, 2.0, 0.0)])
def test_insertion_burn_shots_never_pass_15_s(burn, oe_delay, tail):
    cuts = _ins_run(burn, oe_delay, tail)
    for (u0, s), (u1, _) in zip(cuts, cuts[1:]):
        if s in ("S4", "S4c"):
            assert u1 - u0 <= sr.MAX_HOLD["event"] + 1e-6, (burn, s, u0, u1)
            assert u1 - u0 >= sr.MIN_SHOT - 1e-6, (burn, s, u0, u1)


def test_flight3_replay_reproduces_the_defect_and_passes(capsys):
    from flightdata import flight_file
    csv_path = flight_file("20261021_121140_orbit_T-WorksFineGF_landing.csv")
    if csv_path is None:
        pytest.skip("flight #3 landing CSV not available (tests/data or KSP_REPLAY_LOGS)")
    assert rp.main([csv_path]) == 0                                 # plan / frames / shot lengths PASS
    out = capsys.readouterr().out
    assert "flight 121138" in out and "V4: 1x from 12.0 s before" in out
    lens = {}
    for line in out.splitlines():
        w = line.split()
        if len(w) >= 5 and w[0] == "T" and w[2] in ("S4", "S4c", "S4b"):
            lens.setdefault(w[2], []).append(float(w[4]))
    assert lens["S4"] and lens["S4c"] and max(max(v) for v in lens.values()) <= 15.0


# ------------------------------------------------------------------------------------- 6. separation metric
def _ts(w):
    t = time.localtime(w)
    return "%02d:%02d:%06.3f" % (t.tm_hour, t.tm_min, t.tm_sec + (w - math.floor(w)))


class FakeGame:
    """Wall clock + game UT at `rate` x real time; KSP.log lines appear in the file only at their flush time."""

    def __init__(self, path, w0, u0=103170.0, rate=0.4):
        self.path, self.w, self.w0, self.u0, self.rate = path, w0, w0, u0, rate
        self.pending = []                  # (flush wall, line)

    def ut(self):
        return self.u0 + (self.w - self.w0) * self.rate

    def wall(self):
        return self.w

    def sleep(self, dt):
        self.w += dt
        due = [x for x in self.pending if x[0] <= self.w]
        self.pending = [x for x in self.pending if x[0] > self.w]
        with open(self.path, "a") as f:
            for _, line in sorted(due):
                f.write(line)

    def explode(self, game_s, name, flush_wall):
        w = self.w0 + game_s / self.rate
        self.pending.append((flush_wall, "[LOG %s] %s Exploded!! - blast awesomeness: 0.5\n" % (_ts(w), name)))


def test_sep_window_counts_late_flushed_lines(tmp_path):
    """Flight #3 pattern: game 0.4x, booster-side explosions ~2 game s after the staging, written to KSP.log late.
    The 15 s game window ends 37.5 wall s after the staging; the log is re-read 2 s later and sees them all."""
    log = tmp_path / "KSP.log"
    log.write_text("[LOG 12:00:00.000] earlier line\n")
    off = os.path.getsize(log)
    w0 = time.mktime((2026, 10, 21, 12, 14, 54, 0, 0, -1)) + 0.25
    g = FakeGame(str(log), w0)
    g.explode(1.0, "Rockomax64.BW", w0 + 30.0)              # a core part, 3 s window
    g.explode(2.0, "solidBooster1-1", w0 + 38.0)           # flushed only after the window closed (37.5 s wall)
    g.explode(2.4, "radialDecoupler", w0 + 38.5)
    g.explode(10.0, "noseCone", w0 + 26.0)                 # 15 s window only
    g.explode(20.0, "solidBooster1-1", w0 + 39.0)          # after the window: not counted
    sep = dict(t=w0, m=(g.ut(), w0), log=str(log), off=off)
    lines = []
    c = A.sep_window_watch(sep, lines.append, g.ut, wall=g.wall, sleep=g.sleep)
    assert (c["explosions_3s"], c["booster_3s"], c["core_3s"]) == (3, 2, 1)
    assert (c["explosions_15s"], c["booster_15s"], c["core_15s"]) == (4, 3, 1)
    assert sep["window"] is c and "explosions_3s 3 (booster 2, core 1)" in lines[-1]
    assert "explosions_15s 4 (booster 3, core 1)" in lines[-1]
    # the old 3 s read (flight #3: at 3 game s = 7.5 wall s) would have seen none of the booster lines
    assert A.log_explosions(str(log), off).count("solidBooster1-1") == 2


def test_sep_window_without_ut_and_without_a_mark(tmp_path):
    """Non-recording flight (no game-clock mark): the staging UT is extrapolated from the samples; UT unreadable:
    the wall clock is the game clock."""
    log = tmp_path / "KSP.log"
    log.write_text("")
    w0 = time.mktime((2026, 10, 21, 23, 59, 55, 0, 0, -1))          # across midnight
    g = FakeGame(str(log), w0, rate=1.0)
    g.explode(2.0, "solidBooster1-1", w0 + 3.0)
    g.explode(8.0, "landingLeg1-2", w0 + 9.0)
    g.w = w0 + 0.4                                                    # the thread starts a little after the staging
    c = A.sep_window_watch(dict(t=w0, m=None, log=str(log), off=0), lambda *a: None, g.ut, wall=g.wall,
                           sleep=g.sleep)
    assert (c["explosions_3s"], c["explosions_15s"], c["core_15s"]) == (1, 2, 1)
    g2 = FakeGame(str(log), w0 + 100.0, rate=1.0)
    c2 = A.sep_window_watch(dict(t=w0, m=None, log=str(log), off=0), lambda *a: None, lambda: 1 / 0,
                            wall=g2.wall, sleep=g2.sleep, max_wall=20.0)
    assert c2["explosions_15s"] == 2


def test_wall_at_and_midnight():
    s = [(100.0, 1000.0), (102.0, 1005.0)]
    assert A.wall_at(101.0, s) == pytest.approx(1002.5) and A.wall_at(106.0, s) == pytest.approx(1015.0)
    assert A.wall_at(99.0, s) == pytest.approx(997.5) and A.wall_at(5.0, [(0.0, 10.0)]) == pytest.approx(15.0)
    w0 = time.mktime((2026, 10, 21, 23, 59, 59, 0, 0, -1))
    c = A.sep_window_counts([(0.5, "solidBooster1-1"), (86398.0, "noseCone")], w0, w0 + 3.0, w0 + 15.0)
    assert c["explosions_15s"] == 1 and c["explosions_3s"] == 1        # 00:00:00.5 is 1.5 s after 23:59:59


def test_sep_metric_is_measurement_only():
    """The window thread reads UT and KSP.log only; the flight loop starts it and never waits for it."""
    import inspect
    for fn in (A.sep_window_watch, A.start_sep_window, A.sep_window_counts, A.log_explosions_timed):
        body = inspect.getsource(fn)
        for word in ("control", "auto_pilot", "throttle", "activate_next_stage", "active_vessel"):
            assert word not in body, (fn.__name__, word)
    st = inspect.getsource(A.Ascent._stage_if_needed)
    assert "start_sep_window(self.sep, self.log)" in st and ".join(" not in st


# ------------------------------------------------------------------------------------- 2. deorbit graphic
def test_deorbit_info_from_the_plan():
    m = dict(rec0=0.0, rec1=1000.0, deorbit=(500.0, 510.3), burns=[(500.0, 510.3)], entry=800.0, td=990.0)
    plan = sr.playback_plan(m)
    info = ed.deorbit_info(plan, m, {"pe0_km": 100.0, "pe1_km": -42.3, "dv": 150.0})
    assert info["ut0"] == pytest.approx(490.0) and info["ut1"] == pytest.approx(515.3)
    assert (info["ign"], info["cutoff"], info["pe1"], info["dv"]) == (500.0, 510.3, -42.3, 150.0)
    assert ed.deorbit_info(plan, dict(m, deorbit=None), {}) is None


def test_v16_overlays_follow_the_hud_style(tmp_path):
    """V16 (requirement 2026-10-21): every post-production graphic in HudV2 style. Run in the
    interpreter that has Pillow: badges and the deorbit widget are white only (no accent colour), small (badge one
    line <= 24 design px tall), the widget fits HudV2's picture-in-picture box and has no fill (no glass panel), and a
    dark P8 frame is not lifted (no shadow lift: outside the widget the picture is unchanged)."""
    py = ed.python_with_pil()
    if py is None:
        pytest.skip("needs Pillow")
    code = r"""
import json, os, sys
sys.path.insert(0, sys.argv[1]); sys.path.insert(0, os.path.join(sys.argv[1], "tools"))
from PIL import Image, ImageStat
import edit_video as ed
ov = ed.Overlays(1920, 1080)
def chroma(im):
    px = [p for p in im.getdata() if p[3] > 40]
    return max((max(p[:3]) - min(p[:3])) for p in px) if px else 0, len(px)
res = {}
for name, b in (("lapse", ov.badge("lapse", 50)), ("skip", ov.badge("skip", None, sub="5 H 46 MIN")),
                ("real", ov.badge("real")), ("intro", ov.badge("lapse", 50, 2.0))):
    bb = b.getbbox()
    res[name] = dict(chroma=chroma(b)[0], h=bb[3] - bb[1], w=bb[2] - bb[0])
info = dict(ut0=100.0, ut1=125.3, ign=110.0, cutoff=120.3, pe0=100.0, pe1=-60.0, dv=150.0)
w = ov.deorbit_widget(info, 115.0)
c, n = chroma(w)
res["widget"] = dict(chroma=c, cover=n / float(w.width * w.height), size=list(w.size),
                     centre_alpha=w.getpixel((int(w.width * 0.33), int(w.height * 0.55)))[3])
im = Image.new("RGB", (1920, 1080), (4, 4, 6))
o = ed.deorbit_frame(ov, im.copy(), info, 115.0)
x0, y0 = ed.DEO_BOX[0], ed.DEO_BOX[1]
res["outside"] = ImageStat.Stat(o.crop((x0 + ed.DEO_BOX[2] + 10, 0, 1920, 1080)).convert("L")).mean[0]
res["box"] = list(ed.DEO_BOX)
print(json.dumps(res))
"""
    r = subprocess.run([py, "-c", code, ROOT], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    res = json.loads(r.stdout.strip().splitlines()[-1])
    for k in ("lapse", "skip", "real", "widget"):
        assert res[k]["chroma"] <= 12, (k, res[k])                         # white / grey only, no amber
    for k in ("lapse", "skip", "real"):
        assert res[k]["h"] <= 24 and res[k]["w"] <= 420, (k, res[k])       # one small line
    assert res["intro"]["h"] <= 48                                         # the 2x intro stays HUD-value sized
    assert res["box"][2:] == [340, 330]                                    # HudV2 WW x WH
    assert res["widget"]["size"][0] <= 342 and res["widget"]["size"][1] <= 332
    assert res["widget"]["cover"] < 0.12, res["widget"]                     # line art, no filled panel
    assert res["widget"]["centre_alpha"] == 0                              # inside the planet outline: see-through
    assert res["outside"] < 6.0                                            # no shadow lift (raw 4.4)


def test_v16_editors_have_no_chinese_dimming_lift_or_accent():
    """V16 source guard: no CJK text, no dimmed background, no lutrgb shadow lift, no amber in the editors."""
    import re
    for f in ("edit_video.py", "edit_obs.py"):
        src = open(os.path.join(ROOT, "tools", f), encoding="utf-8").read()
        assert not re.search(r"[\u3000-\u303f\u4e00-\u9fff\uff00-\uffef]", src), f
        assert "lutrgb" not in src and "AMBER" not in src and "Image.blend" not in src, f
        assert "(0, 0, 0, 179)" not in src and "night_lift" not in src, f


# ------------------------------------------------------------------------------------- 7. HUD NEXT label
def test_next_label_now_during_a_burn_rec_full_only():
    from kspilot import hud
    f = hud.HudFeed.__new__(hud.HudFeed)
    src = open(os.path.join(ROOT, "kspilot", "hud.py")).read()
    blk = src.split("        if self.v2:\n            lines.append(\"rec=full\")", 1)[1].split("        if self.phase_name", 1)[0]
    assert 'lines.append("thr_now=%.2f" % tel["thr"])' in blk
    cs = open(os.path.join(ROOT, "plugins", "AutoLab", "HudV2.cs")).read()
    assert 'bool v2rec = S(kv, "rec") == "full";' in cs
    assert "if (e.dt > 0 && !(v2rec && e.done)) { nx = e; break; }" in cs
    assert 'Sp(burnNow ? "NOW" : "IN " + Dur(nx.dt))' in cs
    del f


# ------------------------------------------------------------------------------------- non-recording guarantee
def test_fix3_paths_are_rec_full_only():
    """Director: only built under REC_FULL (test_rec_v2). Lander: _sun_rel only from _camera_v2 (rec_v2). Flight
    script: _rec_extra only written in REC_FULL blocks, the release announcement block is REC_FULL. HUD: thr_now only
    with v2. Editor: runs only for REC_FULL recordings. The separation window is measurement (all modes, no control)."""
    src = open(os.path.join(ROOT, "tools", "orbit_recover.py")).read()
    assert "lander.rec_v2 = REC_FULL" in src and src.count("Director(log=log, v2=True)") == 1
    lsrc = open(os.path.join(ROOT, "kspilot", "landing.py")).read()
    assert "if self.rec_v2:\n            sid, want = self._camera_v2(ut, r)" in lsrc
    assert "else:\n            pre = not self.entry_done" in lsrc                # the non-v2 camera is unchanged
    assert 'want = "chase"' in lsrc
