"""DR-012 offset aim (kspilot/aim.py), offline: target math (ENU offset -> lat/lon, along/cross), env interface,
pre-flight site checks, the lander's target frame / CA-3 radar rule at an offset site, deorbit planning to the
site, the replay check (tools/aim_replay.py, replay_sim closed loop) and the baseline gate: without KSP_AIM the
lander is bit-identical to the pre-DR-012 code (same RPC sequence and landing CSV on the counting fake kRPC)."""
import importlib.util
import math
import os
import subprocess
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from kspilot import aim, core, entry, landing, mission  # noqa: E402
from kspilot import deorbit as D  # noqa: E402
from kspilot import powered_descent as pdg  # noqa: E402
import fake_krpc as fk  # noqa: E402

PAD = entry.KSC_PAD
R = entry.KERBIN.radius
BASE_REV = "d53a4d8"        # last commit before DR-012 (landing.py baseline for the bit-identity gate)


# ----------------------------------------------------------------------------- target math
@pytest.mark.parametrize("along,cross,az", [(-2000, 0, 90), (2000, 0, 90), (-2000, 300, 83.4), (1500, -700, 96.6),
                                            (-9000, 2500, 45.0), (600, 0, 270.0)])
def test_offset_latlon_inverts_ground_errors(along, cross, az):
    lat, lon = aim.offset_latlon(along, cross, az)
    a, c = aim.along_cross(lat, lon, az)
    assert a == pytest.approx(along, abs=1e-6) and c == pytest.approx(cross, abs=1e-6)
    assert aim.pad_distance(lat, lon) == pytest.approx(math.hypot(along, cross), rel=1e-4)


def test_enu_offset_matches_local_tangent_plane():
    """Small offsets: the point lies along/cross metres along the heading / right unit vectors of the pad's ENU."""
    e, n, u = entry.enu_basis(*PAD)
    for along, cross, az in [(-2000, 0, 90), (800, 400, 30), (-1200, -900, 200)]:
        ps = math.radians(az)
        h = math.sin(ps) * e + math.cos(ps) * n
        right = math.cos(ps) * e - math.sin(ps) * n
        lat, lon = aim.offset_latlon(along, cross, az)
        d = entry.latlonalt_to_bf(lat, lon, 0.0) - entry.latlonalt_to_bf(*PAD, 0.0)
        assert float(d @ h) == pytest.approx(along, abs=1.0)        # chord vs arc: < 0.01 m at 2 km
        assert float(d @ right) == pytest.approx(cross, abs=1.0)
        assert float(d @ u) == pytest.approx(-(along ** 2 + cross ** 2) / (2 * R), abs=0.05)   # curvature drop


def test_flat2k_site():
    t = aim.from_env({"KSP_AIM_SITE": "flat2k"})
    assert (t.along, t.cross, t.az, t.h_exp) == (-2000.0, 0.0, 90.0, 64.7)
    assert t.lat == pytest.approx(-0.0972074, abs=2e-7) and t.lon == pytest.approx(-74.7486592, abs=2e-7)
    assert aim.pad_distance(*t.latlon) == pytest.approx(2000.0, abs=0.01)
    # short of the pad on BOTH flown approach headings (inc 6 deg: 83.4 / 96.6 deg), within 230 m of either track
    for hd in (83.4, 90.0, 96.6):
        a, c = aim.along_cross(t.lat, t.lon, hd)
        assert a < -1980 and abs(c) < 235


# ----------------------------------------------------------------------------- env interface
def test_env_default_is_none():
    assert aim.from_env({}) is None
    assert aim.from_env({"KSP_AIM": "  ", "KSP_AIM_SITE": ""}) is None


def test_env_custom_offset_and_azimuth():
    t = aim.from_env({"KSP_AIM": "-1500,200", "KSP_AIM_AZ": "83.4"})
    assert t.name == "aim(-1500,+200)" and t.h_exp is None
    assert aim.along_cross(t.lat, t.lon, 83.4) == pytest.approx((-1500.0, 200.0), abs=1e-6)


@pytest.mark.parametrize("env", [{"KSP_AIM": "-2000,0", "KSP_AIM_SITE": "flat2k"}, {"KSP_AIM": "-2000"},
                                 {"KSP_AIM": "a,b"}, {"KSP_AIM": "100,0"}, {"KSP_AIM": "0,0"},
                                 {"KSP_AIM": "-20000,0"}, {"KSP_AIM_SITE": "beach"}, {"KSP_AIM": "nan,0"}])
def test_env_rejects(env):
    with pytest.raises(ValueError):
        aim.from_env(env)


# ----------------------------------------------------------------------------- pre-flight site checks
SITE = aim.from_env({"KSP_AIM_SITE": "flat2k"})


def test_check_site_flat_ok():
    ok, h, msg = aim.check_site(lambda la, lo: 64.7 + 0.3 * math.sin(la * 1e4), SITE)
    assert ok and h == pytest.approx(64.7, abs=0.3), msg


@pytest.mark.parametrize("height,why", [
    (lambda la, lo: 64.7 + (lo - SITE.lon) * 10472 * 0.05, "not flat"),       # 5 % slope: 10 m over 200 m
    (lambda la, lo: -10.0, "water"),
    (lambda la, lo: 120.0, "logs say")])                                        # wrong place
def test_check_site_refuses(height, why):
    ok, _, msg = aim.check_site(height, SITE)
    assert not ok and why in msg


def test_check_clear():
    ground = lambda la, lo, up: up                                            # every ray hits the terrain
    assert aim.check_clear(ground, SITE)[0] == "clear"
    hut = lambda la, lo, up: up - 12.0 if abs(lo - SITE.lon) * 10472 < 30 else up    # a 12 m building at the site
    assert aim.check_clear(hut, SITE)[0] == "obstacle"
    assert aim.check_clear(lambda la, lo, up: float("inf"), SITE)[0] == "unknown"
    assert aim.check_clear(lambda la, lo, up: None, SITE)[0] == "unknown"


# ----------------------------------------------------------------------------- lander at an offset site
def offsite_lander(target_alt):
    conn, vessel, _ = fk.make_world(SITE.latlon, 10, alt=2000.0)
    fk.CTR = fk.Counter()
    kw = {} if target_alt is None else dict(target_alt=target_alt)
    return landing.Lander(conn, vessel, target=SITE.latlon, log=lambda *a: None, h_com=8.0, **kw)


def test_lander_defaults_are_the_pad():
    assert landing.Lander.alt0 == pdg.PAD_ALT and landing.Lander.target_alt is None
    assert landing.Lander.offsite is False and landing.Lander._tkw == {} and landing.Lander.save_suffix == ""
    L = offsite_lander(None)
    assert L.alt0 == pdg.PAD_ALT and not L.offsite and L._tkw == {}
    m = mission.Mission.__new__(mission.Mission)
    assert m._tkw() == {}
    m.target_alt = 64.7
    assert m._tkw() == {"target_alt": 64.7}


def test_lander_offsite_state_uses_radar_down_to_the_target():
    """CA-3 at the offset site: no pad structure -> the radar ground (h_gnd) is used even within 30 m of the target,
    and r[2] = min(height above the target terrain, radar ground)."""
    L = offsite_lander(64.7)
    assert L.offsite and L.alt0 == 64.7 and L._tkw == {"target_alt": 64.7}
    o, u = L.frame.origin, L.frame.u
    L.vel = lambda: tuple(-2.0 * u)
    L.mass = lambda: 20000.0
    for above, ground_below_origin, side in [(20.0, 0.0, 10.0), (20.0, 3.0, 10.0), (6.0, -0.5, 0.0), (2500, 0.0, 300)]:
        L.pos = lambda above=above, side=side: tuple(o + u * (above + L.h_com) + L.frame.e * side)
        L.radar = lambda above=above, g=ground_below_origin: above + g + L.h_com
        r, v, m = L.state()
        assert L.h_gnd == pytest.approx(above + ground_below_origin, abs=1e-6)
        assert r[2] == pytest.approx(min(above + side ** 2 / (2 * R), above + ground_below_origin), abs=0.02)
        assert v[2] == pytest.approx(-2.0, abs=1e-5)        # radial (local up 0.03 deg off at 300 m)
    # the pad rule for comparison: within 30 m of the target the radar is NOT used (elevated pad)
    P = offsite_lander(None)
    P.vel, P.mass = L.vel, L.mass
    P.pos = lambda: tuple(P.frame.origin + P.frame.u * (20.0 + P.h_com))
    P.radar = lambda: 5.0
    r, _, _ = P.state()
    assert P.h_gnd == pytest.approx(r[2]) and r[2] == pytest.approx(20.0, abs=1e-6)


def test_ca3_below_site_level_keeps_descending():
    """Ground 3 m below the site's terrain (slope): burn_law below the gate commands 2.5 m/s down while the radar
    ground is > 5 m... and the touchdown speed for the last 5 m (CA-3, as off the pad)."""
    import aim_replay
    L = aim_replay.controller()
    m, tmax = 20000.0, 650e3
    L.h_gnd = 8.0
    thr_far, _ = L.burn_law(np.array([0.0, 0.0, -3.0]), np.array([0.0, 0.0, -2.5]), m, tmax, 0.0)
    L.h_gnd = 3.0
    thr_near, _ = L.burn_law(np.array([0.0, 0.0, -3.0]), np.array([0.0, 0.0, -2.5]), m, tmax, 0.0)
    w = m * L.g / tmax
    assert thr_far == pytest.approx(w, rel=0.02)              # holding 2.5 m/s: hover thrust, no cut
    assert thr_near > thr_far                                  # slowing to v_final 1.5 m/s for the last metres


# ----------------------------------------------------------------------------- deorbit planning to the site
def test_deorbit_plan_hits_the_offset_site():
    from test_entry import circular_orbit_bf
    rb, vb = circular_orbit_bf(80e3, 6.0, -170.0)
    veh = D.Vehicle(10e3, 200e3, 320.0, cda=2.0)
    plan = D.plan_deorbit(rb, vb, veh, target=SITE.latlon, target_alt=64.7)
    assert plan.impact.hit and plan.miss < 5.0
    assert entry.great_circle(plan.impact.lat, plan.impact.lon, *SITE.latlon) < 5.0
    assert 1990.0 < aim.pad_distance(plan.impact.lat, plan.impact.lon) < 2010.0
    a, c = entry.ground_errors(plan.impact.lat, plan.impact.lon, PAD, plan.heading)
    dh = math.radians(plan.heading - 90.0)                                # flight track vs the site's 90 deg
    assert a == pytest.approx(-2000.0 * math.cos(dh), abs=5.0) and c == pytest.approx(2000.0 * math.sin(dh), abs=5.0)
    assert a < -1950 and abs(c) < 250.0                                   # short of the pad, near its track


# ----------------------------------------------------------------------------- replay check (replay_sim)
from flightdata import flight_file  # noqa: E402

REPLAY_LOG = flight_file("20261020_203827_orbit_T-WorksFineGF0T_landing.csv")


@pytest.mark.skipif(REPLAY_LOG is None, reason="GF0T flight log not available (tests/data or KSP_REPLAY_LOGS)")
def test_replay_converges_to_offset_target():
    """tools/aim_replay.py: the same flight carried so its unpowered impact is 158 m from the target; the real
    ignition rule + burn law in the target frame land it at the offset site as precisely as on the pad."""
    import aim_replay
    pad, off = aim_replay.check(REPLAY_LOG, "flat2k", (150.0, 50.0))
    assert off["miss"] < 10.0 and abs(off["miss"] - pad["miss"]) < 2.0, (pad, off)
    assert abs(off["vz"]) < 2.0 and off["vh"] < 1.0
    assert 1990.0 < off["pad_dist"] < 2015.0


# ----------------------------------------------------------------------------- baseline gate (bit-identity)
def _baseline_landing():
    try:
        src = subprocess.run(["git", "show", "%s:kspilot/landing.py" % BASE_REV], cwd=ROOT, capture_output=True,
                             text=True, check=True).stdout
    except Exception:
        pytest.skip("baseline revision %s exists only in the development history, not in this export" % BASE_REV)
    spec = importlib.util.spec_from_loader("kspilot._landing_base", loader=None)
    mod = importlib.util.module_from_spec(spec)
    mod.__package__ = "kspilot"
    exec(compile(src, "landing@%s.py" % BASE_REV, "exec"), mod.__dict__)
    return mod


@pytest.mark.parametrize("world", [dict(), dict(alt=2000.0, speed=120.0, east_off=10.0),
                                   dict(alt=800.0, speed=90.0, east_off=400.0)])
def test_default_run_bit_identical_to_baseline(tmp_path, world):
    """No KSP_AIM: the real Lander.run on the counting fake (entry burn at 41 km; coast / landing burn at 2 km
    within 30 m of the pad; landing burn 400 m off) makes the same RPCs in the same order with the same
    arguments-derived values and writes the same landing CSV as the pre-DR-012 landing.py.
    (Update BASE_REV only when landing.py changes for another, reviewed reason.)"""
    base = _baseline_landing()
    (tmp_path / "new").mkdir(), (tmp_path / "old").mkdir()
    new = fk.run_lander(landing, core, tmp_path / "new", rec=False, ticks=15, **world)
    old = fk.run_lander(base, core, tmp_path / "old", rec=False, ticks=15, **world)
    assert not new["missing"] and not old["missing"]
    assert new["seq"] == old["seq"]
    assert new["counts"] == old["counts"]
    assert new["csv"] == old["csv"]
    assert new["log"] == old["log"]


def test_orbit_recover_default_path_adds_nothing():
    """Source gate for the flight script: every offset-aim addition is behind `AIM is not None` or an empty
    kwargs dict / suffix that the default leaves empty."""
    src = open(os.path.join(ROOT, "tools", "orbit_recover.py")).read()
    assert "_aim_kw, _aim_sfx = {}, \"\"" in src
    assert "Mission(conn, pe * 1e3, ap * 1e3, inc, recover=True, log=log, tag=tag, **_aim_kw)" in src
    assert "if AIM is not None else {}))" in src
    assert "_miss_tol = 20 if AIM is None" in src
