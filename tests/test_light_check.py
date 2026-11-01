"""tools/light_check.py (V17 lamps on the pad at night) against tests/fake_krpc.PadWorld - offline, no game."""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import fake_krpc as fk  # noqa: E402
import light_check as lc  # noqa: E402
from kspilot import core  # noqa: E402


class Lines(list):
    def __call__(self, *msg):
        self.append(" ".join(str(m) for m in msg))

    def result(self):
        res = [ln for ln in self if ln.startswith("RESULT")]
        assert len(res) == 1, res
        return res[0]


@pytest.fixture(autouse=True)
def _logs(tmp_path, monkeypatch):
    monkeypatch.setattr(core, "LOG_DIR", str(tmp_path))


def _run(world=None, argv=()):
    log = Lines()
    env = lc.dry_env(log, world)
    out = lc.run(lc.parse(list(argv)), env=env, log=log)
    return log, env, out


def test_committed_lit_craft_has_the_8_lamp_kit():
    data = lc.craft_bytes(lc.CRAFT)                      # working copy, or git HEAD if it was cloud-evicted
    assert data.startswith(b"ship = T-WorksFineGF-LIT") and lc.lamp_count(data) == 8


def test_pass_installs_lights_shoots_and_recovers():
    log, env, out = _run()
    w = env.world
    assert out is True and log.result() == "RESULT PASS 8/8 lamps on", log
    dst = os.path.join(env.ships_dir, lc.CRAFT + ".craft")
    assert open(dst, "rb").read() == lc.craft_bytes(lc.CRAFT)          # byte-identical install
    assert w.launched == [("VAB", lc.CRAFT, "LaunchPad")] and w.recovered == [lc.CRAFT]
    kinds = [c.split()[0] for c in env.cams]
    assert kinds == ["ground", "rel", "ground", "chase"]                  # side, chase, below, then back to stock
    assert sum("lamp " in ln and ": ON," in ln for ln in log) == 16       # 8 lamps x 2 reads, each with its state
    assert any("/ 2000 K" in ln for ln in log)                            # temperature / max reported
    shots = [ln for ln in log if ln.startswith("camera ")]
    assert len(shots) == 3 and all("light_check_" in ln and ln.endswith(".png") for ln in shots)
    assert not fk.CTR.missing or all(not m.startswith("vessel_") for m in fk.CTR.missing)
    _, env2, out2 = _run()                                                # second run: identical file untouched
    assert out2 is True


def test_dead_lamp_fails_but_still_cleans_up():
    w = fk.PadWorld(dead=(3,))
    log, env, out = _run(w)
    assert out is False and log.result().startswith("RESULT FAIL") and "lamp(s) [4] not on" in log.result()
    assert w.recovered == [lc.CRAFT] and env.cams[-1] == "chase"


def test_missing_lamps_fail():
    log, _, out = _run(fk.PadWorld(n_lamps=6))
    assert out is False and "6 spotLight3 parts on the vessel, craft has 8" in log.result()


def test_refuses_while_a_mission_flies():
    w = fk.PadWorld(flying=True)
    log, env, out = _run(w)
    assert out is None and log.result().startswith("RESULT REFUSED") and "flying" in log.result()
    assert w.launched == [] and env.cams == [] and os.listdir(env.ships_dir) == []


def test_keep_leaves_the_vessel_lights_off():
    w = fk.PadWorld()
    log, env, out = _run(w, ["--keep"])
    assert out is True and w.recovered == [] and w.vessel.control._peek("lights") is False


def test_dataless_working_copy_falls_back_to_git(tmp_path, monkeypatch):
    real = lc.craft_bytes(lc.CRAFT)
    (tmp_path / "craft" / "VAB").mkdir(parents=True)
    (tmp_path / "craft" / "VAB" / (lc.CRAFT + ".craft")).write_bytes(b"")   # what an evicted file reads as

    class R:
        stdout = real
    calls = []
    monkeypatch.setattr(lc.subprocess, "run", lambda cmd, **k: calls.append(cmd) or R())
    assert lc.craft_bytes(lc.CRAFT, root=str(tmp_path)) == real
    assert calls and calls[0][-1] == "HEAD:craft/VAB/%s.craft" % lc.CRAFT
    calls.clear()                                     # an evicted ('dataless') file is never opened: git directly
    monkeypatch.setattr(lc, "dataless", lambda p: True)
    monkeypatch.setattr("builtins.open", lambda *a, **k: pytest.fail("opened a dataless file"))
    assert lc.craft_bytes(lc.CRAFT) == real and len(calls) == 1


def test_shot_geometry():
    lat, lon = core.KSC_PAD
    s = dict(lc.shot_strings(lat, lon))
    side, below = s["side"].split(), s["below"].split()
    m = lc.R_KERBIN * 3.141592653589793 / 180.0
    assert (lat - float(side[1])) * m == pytest.approx(160.0, abs=0.5)
    assert float(side[3]) == pytest.approx(lc.PAD_ALT + 12.0, abs=0.1)
    assert float(below[3]) == pytest.approx(lc.PAD_ALT + 1.5, abs=0.1) and float(below[4]) == 80.0
    assert s["chase"].startswith("rel 35 ")


def test_dry_run_cli():
    log = Lines()
    assert lc.run(lc.parse(["--dry-run"]), log=log) is True and log.result().startswith("RESULT PASS")
