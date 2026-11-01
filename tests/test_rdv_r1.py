"""Docking rung R1 offline checks: tools/rdv_r1.py (measure / prep / rendezvous) against the two-vessel orbit fake
(tests/fake_krpc.OrbitWorld) and the T-WorksFineGF-DOCK craft (tools/craft_dock.py). No game, no kRPC."""
import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import fake_krpc as fk  # noqa: E402
import rdv_r1 as r1  # noqa: E402
from kspilot import core  # noqa: E402
from kspilot.rdv import cw  # noqa: E402


class Lines(list):
    def __call__(self, *msg):
        self.append(" ".join(str(m) for m in msg))

    def result(self):
        res = [l for l in self if l.startswith("RESULT")]
        assert len(res) == 1, res
        return res[0]


@pytest.fixture(autouse=True)
def _logs(tmp_path, monkeypatch):
    monkeypatch.setattr(core, "LOG_DIR", str(tmp_path))


def _run(mode, world, *argv):
    log = Lines()
    env = r1.Env(world.conn, log, sleep=world.sleep, burn=world.burn, warp=world.advance_to, dry=True)
    out = r1.run(r1.parse([mode] + list(argv)), env=env, log=log)
    return log, out


def _hold_world(**kw):
    d = dict(alt_c=100e3, alt_t=100e3, phase0=1000.0 / 700e3)
    d.update(kw)
    return fk.OrbitWorld(**d)


def _writes():
    return [s for s in fk.CTR.seq if "=" in s.split("(")[0]]


# ---------------------------------------------------------------- measure
def test_measure_recovers_noise_and_is_read_only():
    w = _hold_world(noise=0.01, noise_v=0.002)
    log, (out, path) = _run("measure", w, "--secs", "60")
    assert log.result().startswith("RESULT PASS")
    st = out["stats"]
    assert st["source"] == "port->port" and st["segments"] == 2
    # nrf difference of two noisy port readings: sqrt(2) x the per-reading noise; target port frame: 1 x
    assert 0.012 < st["sigma_pos"] < 0.017 and 0.0023 < st["sigma_vel"] < 0.0034
    assert 0.008 < st["sigma_pos_port_frame"] < 0.012
    assert os.path.exists(path) and os.path.basename(path).startswith("rdv_r1_measure_")
    # read only: no property write, no save / jettison / warp / target change
    assert _writes() == [] and w.saved == [] and w.jettisoned == [] and w.sc._peek("target_vessel") is None
    assert not fk.CTR.missing
    s = out["survey"]
    assert {v["name"] for v in s["vessels"]} == {"R1-Chaser", "R1-Station"}
    assert s["chaser"]["ports"][0]["dir_own"] == [0.0, 1.0, 0.0]
    assert "pos_other" in s["chaser"]["ports"][0] and "pos_other" in s["target"]["ports"][0]
    assert abs(s["rel"]["rho"][1] + 1000.0) < 1.0 and s["closest"]["distance"] < 1100.0
    assert abs(st["ec_rate"] - 0.1) < 0.01


def test_measure_gates_fail_on_noise_and_rcs_rotation():
    w = _hold_world(noise=0.03, rcs_rot=True)
    log, (out, _) = _run("measure", w, "--secs", "30")
    res = log.result()
    assert res.startswith("RESULT FAIL") and "A4 port noise" in res and "A7 rcs rotation off" in res


def test_measure_far_target_unloaded_is_na_not_fail():
    w = fk.OrbitWorld(alt_c=80e3, alt_t=100e3, phase0=2.0)
    log, (out, _) = _run("measure", w, "--secs", "10")
    g = {n: r for n, r, _ in out["gates"]}
    assert g["A4 port noise"] == "NA" and g["target port"] == "NA"
    assert log.result().startswith("RESULT PASS")
    assert out["stats"]["source"] == "vessel->target vessel frame"


def test_noise_stats_removes_relative_motion():
    rng = np.random.default_rng(3)
    n = cw.mean_motion_at(100e3)
    t = np.arange(0.0, 600.0, 0.1)
    X = np.array([cw.propagate(n, [5.0, -1000.0, 2.0], [0.02, 0.1, -0.01], tt)[0] for tt in t])
    sig, s3, nseg = r1.noise_stats(t, X + 0.01 * rng.standard_normal(X.shape))
    assert nseg == 20 and all(0.009 < s < 0.011 for s in sig) and 0.009 < s3 < 0.011


def test_phase_angle_in_krpc_left_handed_frame():
    w = fk.OrbitWorld(alt_c=80e3, alt_t=100e3, phase0=0.5)
    rc, vc = w.rv("c")
    rt, _ = w.rv("t")
    assert abs(r1.phase_angle(rc, vc, rt) - 0.5) < 1e-9          # target 0.5 rad AHEAD along the motion
    assert abs(r1.phase_angle(rt, vc, rc) - (2 * math.pi - 0.5)) < 1e-9


# ---------------------------------------------------------------- prep
def test_prep_renames_jettisons_and_saves():
    w = fk.OrbitWorld(landed_core=True)
    log, ok = _run("prep", w, "--vessel", "station", "--rename", "R1-Target", "--recover-landed", "--jettison",
                   "--sas", "--save", "r1_station")
    assert ok and log.result().startswith("RESULT PASS")
    assert w.recovered == ["landed_core"]                    # the landed core, never the orbiting chaser
    assert w.sc._peek("active_vessel") is w.vessels["t"] and w.vessels["t"]._peek("name") == "R1-Target"
    assert w.jettisoned == ["t"] and w.saved == ["r1_station"]


def test_prep_refuses_ambiguous_name():
    w = fk.OrbitWorld()
    log, ok = _run("prep", w, "--vessel", "R1-")
    assert not ok and log.result().startswith("RESULT REFUSED")


# ---------------------------------------------------------------- rendezvous
def test_rendezvous_hohmann_to_1km_hold_with_hibernation():
    w = fk.OrbitWorld(alt_c=80e3, alt_t=100e3, phase0=2.0, burn_err=0.003, hibernate_ok=True)
    log, (ok, out, path) = _run("rendezvous", w, "--save", "r1_hold_1km")
    assert ok and log.result().startswith("RESULT PASS"), log
    assert any("asleep for warp" in l for l in log) and any("awake" in l for l in log)
    labels = [b[0] for b in out["burns"]]
    assert labels[0].startswith("burn 1") and any(l.startswith("burn 2") for l in labels)
    assert abs(out["dv_plan"] - 32.8) < 0.2 and out["dv_used"] <= 1.3 * out["dv_plan"]
    rho = np.array(out["rho"])
    assert np.linalg.norm(rho - [0, -1000, 0]) < 300 and w.distance() < 1300
    assert w.saved == ["r1_hold_1km"] and w.sc._peek("target_vessel") is w.vessels["t"]


def test_rendezvous_refused_when_the_coast_drains_the_battery():
    w = fk.OrbitWorld(alt_c=80e3, alt_t=100e3, phase0=2.0, hibernate_ok=False)
    log, out = _run("rendezvous", w)
    assert log.result().startswith("RESULT REFUSED power") and w.burns == []


def test_rendezvous_plane_match_then_hold():
    w = fk.OrbitWorld(alt_c=80e3, alt_t=100e3, phase0=0.8, inc_t_deg=0.2, hibernate_ok=True)
    log, (ok, out, _) = _run("rendezvous", w)
    assert ok, log
    assert out["burns"][0][0] == "plane match" and 7.5 < out["burns"][0][2] < 8.5     # 2 v sin(0.1 deg) = 7.95
    assert math.degrees(w._rel_inc()) < 0.01


def test_rendezvous_midcourse_correction_on_big_burn_error():
    w = fk.OrbitWorld(alt_c=80e3, alt_t=100e3, phase0=0.8, burn_err=0.02, seed=4, hibernate_ok=True)
    log, (ok, out, _) = _run("rendezvous", w)
    assert any(b[0].startswith("mid-course") for b in out["burns"]), log
    assert ok, log


def test_rendezvous_refuses_bad_geometry():
    w = fk.OrbitWorld(alt_c=100e3, alt_t=80e3, phase0=2.0)          # target below
    log, _ = _run("rendezvous", w)
    assert log.result().startswith("RESULT REFUSED target not above")
    w = fk.OrbitWorld(alt_c=80e3, alt_t=100e3, phase0=2.0, inc_t_deg=1.0)
    log, _ = _run("rendezvous", w)
    assert log.result().startswith("RESULT REFUSED relative inclination")


def _kick(w, dv_lvlh):
    rc, vc = w.rv("c")
    rt, vt = w.rv("t")
    M, _ = cw.lvlh_basis(rt, vt)
    w._sync()
    w.state["c"] = (rc, vc + M.T @ np.asarray(dv_lvlh, float))


def test_terminal_main_engine_null_and_rcs_trim():
    w = _hold_world(noise=0.0, noise_v=0.0)
    _kick(w, [0.4, -0.5, 0.2])
    log, (ok, out, _) = _run("rendezvous", w, "--rcs-trim", "--drift", "120")
    assert ok, log
    assert any(b[0].startswith("null") for b in out["burns"])
    assert any("rcs polarity pulse" in l for l in log)
    assert np.linalg.norm(out["rhod"]) < 0.1 and w.mono < 50.0
    assert any(l.startswith("DRIFT PASS") for l in log)
    c = w.vessels["c"]._a["control"]
    assert c._peek("right") == c._peek("forward") == c._peek("up") == 0.0


def test_rcs_inverted_polarity_aborts_with_controls_zeroed():
    w = _hold_world(rcs_sign=(-1, -1, -1))
    _kick(w, [0.0, 0.2, 0.0])
    log, out = _run("rendezvous", w, "--rcs-trim")
    assert out is None and log.result().startswith("RESULT FAIL exception") and "polarity" in log.result()
    c = w.vessels["c"]._a["control"]
    assert c._peek("right") == c._peek("forward") == c._peek("up") == 0.0


def test_dry_run_cli_entry_points():
    for mode, argv in (("measure", ["--secs", "5"]), ("rendezvous", []), ("prep", ["--vessel", "chaser"])):
        log = Lines()
        r1.run(r1.parse([mode, "--dry-run"] + argv), log=log)
        assert log.result().startswith("RESULT PASS"), (mode, log)


# ---------------------------------------------------------------- craft
def test_dock_craft_is_the_committed_build_and_checks():
    import craft_dock as cd
    text = cd.build()
    assert text == open(cd.DST).read(), "craft/VAB/T-WorksFineGF-DOCK.craft is stale: run tools/craft_dock.py build"
    cb = __import__("craft_ballast")
    hdr, blocks, _ = cb.split_parts(text)
    fs = [cb.fields(b) for b in blocks]
    names = [f["part"][0].rsplit("_", 1)[0] for f in fs]
    assert names.count("RCSBlock.v2") == 4 and names.count("radialRCSTank") == 4
    assert names.count("dockingPort2") == 1 and "RelayAntenna50" not in names
    assert text.count("enablePitch = False") == 4 and text.count("enableRoll = False") == 4
    assert text.count("selectedVariant = 4Horn") == 4
    assert sum(f["_res"].get("MonoPropellant", 0.0) for f in fs) == pytest.approx(50.0)


GAME = os.path.join(os.path.dirname(__import__("craft_ballast").GAMEDATA), "GameData", "Squad", "Parts")


@pytest.mark.skipif(not os.path.isdir(GAME), reason="KSP install not present")
def test_dock_craft_parts_exist_in_the_stock_install_and_check_passes():
    import craft_dock as cd
    found = set()
    for d, _, files in os.walk(GAME):
        for f in files:
            if f.endswith(".cfg"):
                for ln in open(os.path.join(d, f), errors="replace"):
                    s = ln.strip()
                    if s.startswith("name = ") and s[7:] in ("dockingPort2", "RCSBlock_v2", "radialRCSTank"):
                        found.add(s[7:])
    assert found == {"dockingPort2", "RCSBlock_v2", "radialRCSTank"}
    probs, info = cd.check()
    assert probs == [] and info["payload"] == pytest.approx(2.23, abs=0.005)
