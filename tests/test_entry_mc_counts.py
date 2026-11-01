"""paper/analysis/entry_mc_counts.py: the authoritative Monte Carlo counts of the report (Section 5.2, Table 7, Fig. 12b).

The expected numbers are the ones printed by the script from the committed data/raw/gnc2/entry_mc*.json; the last test
checks that the report and the README quote exactly these numbers.
"""
import importlib.util
import json
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCRIPT = os.path.join(ROOT, "paper", "analysis", "entry_mc_counts.py")


@pytest.fixture(scope="module")
def emc():
    spec = importlib.util.spec_from_file_location("entry_mc_counts", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_committed_counts(emc):
    r = emc.counts()
    assert (r["mild"]["n"], r["stress"]["n"], r["total"]["n"]) == (36, 18, 54)
    out = {(s, c): r[s][c]["stated"]["outside"] for s in ("mild", "stress") for c in ("counter", "pcc")}
    assert out == {("mild", "counter"): 3, ("mild", "pcc"): 4, ("stress", "counter"): 17, ("stress", "pcc"): 6}
    assert r["total"]["counter"]["stated"]["outside"] == 20
    assert r["total"]["pcc"]["stated"]["outside"] == 10
    # widened bands (chosen after the draws were seen)
    assert r["total"]["pcc"]["+185"]["outside"] == 8 and r["total"]["pcc"]["+190"]["outside"] == 0
    assert r["total"]["counter"]["+190"]["outside"] == 17
    # the mild-set predictor-corrector leaves the band more often than the counted cutoff
    assert r["mild"]["pcc"]["stated"]["outside"] > r["mild"]["counter"]["stated"]["outside"]


def test_direction_of_misses(emc):
    r = emc.counts()
    assert r["mild"]["counter"]["stated"]["below"] == 0 and r["mild"]["pcc"]["stated"]["below"] == 0
    assert r["stress"]["counter"]["stated"]["below"] == 17 and r["stress"]["pcc"]["stated"]["above"] == 6


def test_mission21_counter_variants(emc):
    m = emc.mission21()
    assert round(m["legacy"]) == -3360 and round(m["pcc_legacy"]) == 179
    assert round(m["counter"], 1) == 142.7 == round(m["pcc"], 1)
    assert 90 <= m["counter"] <= 175


def test_edges_are_inclusive(emc, tmp_path):
    def draw(a, b):
        return {"counter": {"E10_along": a}, "pcc": {"E10_along": b}}
    (tmp_path / "entry_mc.json").write_text(json.dumps([draw(90.0, 175.0), draw(89.99, 175.01), draw(120, 120)]))
    (tmp_path / "entry_mc_stress.json").write_text(json.dumps([draw(-500, 300)]))
    r = emc.counts(str(tmp_path))
    assert r["mild"]["counter"]["stated"] == {"outside": 1, "below": 1, "above": 0}
    assert r["mild"]["pcc"]["stated"] == {"outside": 1, "below": 0, "above": 1}
    assert r["total"]["counter"]["stated"]["outside"] == 2 and r["total"]["n"] == 4


def test_report_and_readme_quote_the_script(emc):
    text = open(os.path.join(ROOT, "paper", "kspilot_report.md"), encoding="utf-8").read()
    for s in ("3 of 36", "4 of 36", "17 of 18", "6 of 18", "10 of 54", "20 of 54"):
        assert s in text, s
    for stale in ("5 of 36", "11 of which", "(5 of 36 and 6 of 18)", "11 of 54"):
        assert stale not in text, stale
