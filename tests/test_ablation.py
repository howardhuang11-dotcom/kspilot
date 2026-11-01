"""EXPLORATORY ablation / dispersion switches (KSP_ABL_*, kspilot/landing.py Lander._abl_setup; report Section 4.8).
These switches are not part of the guidance. Offline only (fake kRPC, tests/fake_krpc.py):
  1  unset = bit-identical: the real Lander.run on the fake kRPC produces the same RPC sequence (names and written
     values), the same CSV and the same log as the code before the switches existed (golden SHA-256 digests below,
     taken from the pre-switch landing.py)
  2  each switch does what it says (brakes held off, k_drag frozen, noise per tick and true state at the end,
     drag model scaled)."""
import hashlib
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))
from kspilot import core, landing  # noqa: E402
import fake_krpc as fk  # noqa: E402

ABL = ("KSP_ABL_NOBRAKES", "KSP_ABL_NODRAGLEARN", "KSP_ABL_NAVNOISE", "KSP_ABL_SEED", "KSP_ABL_DRAGBIAS")
# First 16 hex digits of SHA-256 over repr() of each output of the 80-tick fake-kRPC flight below, computed with the
# landing.py as it was before the KSP_ABL_* switches were added: `git show v1.0.0:kspilot/landing.py` (2962 RPC calls).
# test_golden_matches_pre_switch_source regenerates these values from that file, so they do not rest on trust.
GOLDEN = dict(n_seq=2962, seq="94b04e3c3eb19035", csv="d2ebe330b17e95d2", log="fc8ea3f519a843a3", tick=80)


def digest(x):
    return hashlib.sha256(repr(x).encode()).hexdigest()[:16]


def fly(mod, d, ticks=80):
    d.mkdir(exist_ok=True)
    out = fk.run_lander(mod, core, d, rec=False, ticks=ticks, frame_s=0.001)
    assert not out["missing"], out["missing"]
    return out



def test_unset_is_bit_identical(tmp_path, monkeypatch):
    for k in ABL:
        monkeypatch.delenv(k, raising=False)
    a, b = fly(landing, tmp_path / "a"), fly(landing, tmp_path / "b")
    assert a["seq"] == b["seq"] and a["csv"] == b["csv"]           # the harness itself is deterministic
    assert len(a["seq"]) == GOLDEN["n_seq"]
    assert digest(a["seq"]) == GOLDEN["seq"]
    assert digest(a["csv"]) == GOLDEN["csv"]
    assert digest(a["log"]) == GOLDEN["log"]
    assert a["world"]["tick"] == GOLDEN["tick"]


def test_nobrakes_holds_brakes_off(tmp_path, monkeypatch):
    monkeypatch.setenv("KSP_ABL_NOBRAKES", "1")
    out = fly(landing, tmp_path / "n", ticks=20)
    assert out["vessel"].control.brakes is False
    assert any("KSP_ABL_NOBRAKES" in x for x in out["log"])
    L = out["lander"]
    assert L.set_brakes(True) is False and out["vessel"].control.brakes is False


def test_nodraglearn_freezes_k(monkeypatch):
    L = landing.Lander.__new__(landing.Lander)
    L.k_drag = {True: 1.0, False: 1.0}
    monkeypatch.setenv("KSP_ABL_NODRAGLEARN", "1")
    L.update_drag_scale(np.zeros(3), 1e6)          # returns before any read (no vessel needed)
    assert L.k_drag == {True: 1.0, False: 1.0}


def test_navnoise_per_tick_and_true_at_end(monkeypatch):
    monkeypatch.setenv("KSP_ABL_NAVNOISE", "5,0.5")
    monkeypatch.setenv("KSP_ABL_SEED", "7")
    L = landing.Lander.__new__(landing.Lander)
    L.log = lambda *a: None
    p0, v0 = np.array([1.0, 2.0, 3.0]), np.array([4.0, 5.0, 6.0])
    L.pos, L.vel = (lambda: p0), (lambda: v0)
    L.state = lambda: (L.pos(), L.vel(), 1.0)        # stands in for Lander.state (reads pos / vel)
    L._abl_setup()
    dp, dv = [], []
    for _ in range(4000):
        r, v, _ = L.state()
        assert np.array_equal(L.pos(), r)            # the same draw for every read inside one tick
        dp.append(r - p0)
        dv.append(v - v0)
    dp, dv = np.array(dp), np.array(dv)
    assert abs(dp.std() - 5.0) < 0.2 and abs(dv.std() - 0.5) < 0.02 and abs(dp.mean()) < 0.2
    assert not np.array_equal(dp[0], dp[1])
    L._abl_true_nav()
    assert np.array_equal(L.pos(), p0) and np.array_equal(L.vel(), v0) and "state" not in L.__dict__


def test_dragbias_scales_model(monkeypatch):
    monkeypatch.setenv("KSP_ABL_DRAGBIAS", "1.2")
    L = landing.Lander.__new__(landing.Lander)
    L.log = lambda *a: None
    L.drag_out = L.drag_in = lambda alt, v, mach=None: 1000.0
    L.cda_out, L.cda_in = 10.0, 5.0
    L._abl_setup()
    assert abs(L.drag_out(1e4, 300.0) - 1200.0) < 1e-9 and abs(L.drag_in(1e4, 300.0) - 1200.0) < 1e-9
    assert abs(L.cda_out - 12.0) < 1e-9 and abs(L.cda_in - 6.0) < 1e-9


def test_golden_matches_pre_switch_source(tmp_path, monkeypatch):
    """The golden digests are those of the pre-switch landing.py (tag v1.0.0), recomputed here from git."""
    import importlib.util
    import subprocess
    for k in ABL:
        monkeypatch.delenv(k, raising=False)
    try:
        code = subprocess.run(["git", "show", "v1.0.0:kspilot/landing.py"], cwd=ROOT, capture_output=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        pytest.skip("git history with tag v1.0.0 not available")
    assert b"KSP_ABL_" not in code, "tag v1.0.0 must hold the pre-switch landing.py"
    path = tmp_path / "landing_v100.py"
    path.write_bytes(code)
    spec = importlib.util.spec_from_file_location("kspilot.landing_v100", str(path))
    old = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(old)
    a = fly(old, tmp_path / "old")
    assert (len(a["seq"]), digest(a["seq"]), digest(a["csv"]), digest(a["log"]), a["world"]["tick"]) == (
        GOLDEN["n_seq"], GOLDEN["seq"], GOLDEN["csv"], GOLDEN["log"], GOLDEN["tick"])
