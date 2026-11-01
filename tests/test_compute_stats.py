"""Inclusion and classification rules of paper/analysis/compute_stats.py on synthetic fixture records.

The fixtures are invented transcripts in the format of the supervisor (`tools/watchdog.py`); they exercise every
inclusion branch: an analysed complete mission, a separation-only test, a launched run without an outcome line, an
in-window pad attempt that never launched, a pre-window mission, a checkpoint flight and an unclassifiable transcript.
"""
import importlib.util
import json
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(os.path.dirname(HERE), "paper", "analysis", "compute_stats.py")


@pytest.fixture(scope="module")
def cs():
    spec = importlib.util.spec_from_file_location("compute_stats", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ASCENT = "[   13.0s] ascent (own): target Pe 100 km Ap 100 km i 6.0 deg -> launch heading 83.5 deg\n"
ORBIT = "[  290.0s] orbit Ap=100.0km Pe=100.0km inc=6.040 (err Ap -18m Pe -43m)\n[  296.0s] payload released (stage 1)\n"
TRANSCRIPTS = {
    # analysed: complete mission, 0.1 m miss in the transcript, 0.01 m in the experiment log
    "wd_20261021_020000.out": ASCENT + ORBIT +
    "[  774.0s] vacuum correction 0: predicted miss 3537m (along +3537 cross +11) spread 1m, trim 0.661 m/s\n"
    "[  789.3s] trim effect 1.00x of prediction -> gain 0.54\n"
    "[  789.3s] vacuum correction 1: predicted miss 179m (along +179 cross +1) spread 0m, trim 0.023 m/s\n"
    "[  925.8s] range trim: retro pulse #1 (out +229m)\n[  975.4s] range trim: retro pulse #6 (out +192m)\n"
    "[ 1077.7s] RESULT PASS miss=1.5m v_td=0.14m/s upright=1.000 fuel_left=360 parts=26 VesselSituation.landed\n",
    # excluded: separation-only test
    "wd_20261021_030000.out": ASCENT + "[   95.0s] RESULT SEPTEST q=9.6 parts=37 thr=649 explosions=0 core_hit=0\n",
    # excluded: no outcome line (terminated externally)
    "wd_20261021_040000.out": ASCENT + ORBIT,
    # in window, never launched: refused before the own-code ascent
    "wd_20261021_050000.out": "[   14.0s] stage 3: dV 888 m/s TWR 1.73\n[   45.9s] RESULT REFUSED (OBS recording failed)\n",
    # before the window: ascent loss
    "wd_20261021_000000.out": ASCENT + "[  195.9s] ascent ABORT: back on the surface\n[  195.9s] RESULT FAIL ascent\n",
    # checkpoint flight (4 km)
    "wd_20261021_060000.out": "[    0.1s] loading checkpoint g5_term_GF\n"
    "[   60.0s] RESULT PASS miss=3.7m v_td=0.12m/s upright=1.000 fuel_left=300 parts=26 VesselSituation.landed\n",
    # unclassifiable: no checkpoint, no pre-launch check, no flight log
    "wd_20261021_070000.out": "something else entirely\n",
}
FLIGHT_LOGS = ["20261021_020005_orbit_T-WorksFineGF.txt", "20261021_030004_orbit_T-WorksFineGF.txt",
               "20261021_040003_orbit_T-WorksFineGF.txt", "20261021_050002_orbit_T-WorksFineGF-LIT.txt",
               "20261021_000001_orbit_T-WorksFineGF.txt", "20261021_060002_orbit_T-WorksFineGF.txt"]


def _jsonl(key, start, miss, ok, time):
    return json.dumps(dict(time=time, craft="T-WorksFineGF", git="abc1234", start=start, env={"KSP_TERM_SAVE": "g5_term_GF"},
                           ignition=dict(h=500, vz=-113.0, offset=1), log="/x/logs/%s.txt" % key,
                           result=dict(ok=ok, miss=miss, v_td=0.14, upright=1.0, fuel_left=360, parts=26)))


@pytest.fixture()
def logs(tmp_path):
    for name, text in TRANSCRIPTS.items():
        (tmp_path / name).write_text(text)
    for name in FLIGHT_LOGS:
        (tmp_path / name).write_text("")
    (tmp_path / "experiments.jsonl").write_text(
        _jsonl("20261021_020005_orbit_T-WorksFineGF", "pad", 1.53, True, "2026-10-21T02:20:00") + "\n" +
        _jsonl("20261021_060002_orbit_T-WorksFineGF", "term", 3.71, True, "2026-10-21T06:05:00") + "\n" +
        # a run that started after the freeze must be ignored even though the file holds it
        _jsonl("20261022_010000_orbit_T-WorksFineGF", "pad", 9.99, True, "2026-10-22T01:20:00") + "\n")
    (tmp_path / "matrix_results.txt").write_text(
        "2026-10-21 02:20:00 base T-WorksFineGF 100/100/6  RESULT PASS miss=1.5m fuel=360 "
        "orbit=[orbit Ap=100.0km Pe=100.0km inc=6.040 (err Ap -18m Pe -43m)] wd=logs/wd_20261021_020000.out\n")
    return tmp_path


def test_rounding_half_up(cs):
    assert cs.rnd(496.5, 0) == "497"
    assert cs.rnd(42.5, 0) == "43"
    assert cs.rnd(0.115, 2) == "0.12"
    assert cs.rnd(float("nan")) == "-"


def test_interval_methods(cs):
    lo, hi = cs.wilson(26, 28)
    assert (round(100 * lo, 1), round(100 * hi, 1)) == (77.4, 98.0)
    lo, hi = cs.clopper_pearson(26, 28)
    assert (round(100 * lo, 1), round(100 * hi, 1)) == (76.5, 99.1)
    lo, hi = cs.clopper_pearson(7, 7)
    assert round(100 * lo, 1) == 59.0 and hi == 1.0
    lo, hi = cs.jeffreys(26, 28)
    assert 0.76 < lo < 0.82 and 0.97 < hi < 0.995
    assert abs(cs.betainc(2.0, 3.0, 0.4) - 0.5248) < 1e-4


def test_inclusion_rules(cs, logs, tmp_path_factory):
    out = tmp_path_factory.mktemp("out")
    import sys
    argv = sys.argv
    sys.argv = ["compute_stats.py", str(logs), "--out", str(out), "--no-figures"]
    try:
        cs.main()
    finally:
        sys.argv = argv
    S = json.load(open(out / "tables" / "stats.json"))
    inc = S["flow"]["by_inclusion"]
    assert inc["analysed: complete mission"] == 1
    assert inc["excluded: separation-only test"] == 1
    assert inc["excluded: no outcome line"] == 1
    assert inc["in window, not launched (counted, not analysed)"] == 1
    assert inc["outside the exploratory window definition"] == 3
    assert S["records"]["wd_files"] == 7
    assert S["records"]["jsonl_rows"] == 2                     # the post-freeze run is filtered on its start time
    assert S["records"]["unresolved_craft"] == ["wd_20261021_070000.out"]
    assert S["records"]["unclassified_start"] == ["wd_20261021_070000.out"]
    c = S["campaign"]
    assert (c["completed"], c["passed"]) == (1, 1)
    assert c["miss_pass"]["median"] == pytest.approx(1.53)     # 0.01 m value from the joined experiment log
    assert c["miss_pass_transcript"]["median"] == pytest.approx(1.5)
    assert c["vac"]["missions"] == 1 and c["pulse_logged_max"]["max"] == 6
    sens = {s["population"]: (s["k"], s["n"]) for s in c["sensitivity"]}
    assert sens["Window; monitor-terminated run counted as a failure"] == (1, 2)
    assert sens["Window; also the in-window pad attempts that never launched"] == (1, 3)
    assert sens["All complete GF-family missions in the records"] == (1, 2)
    manifest = open(out / "tables" / "raw_manifest.sha256").read()
    assert "wd_20261021_020000.out" in manifest and "/x/logs" not in manifest


def test_hibernation_counts_every_segment_kind(cs):
    """Mission.coast_to logs hibernated coasts as "node" and as "deorbit window" segments; both must be counted
    (revision 1 matched only "node"). Awake segments and the drain-budget lines are not hibernated segments."""
    text = ("[  400.0s] power: deorbit window 5000 s hibernating: EC 990.0 -> 989.2 (0.0002 EC/s, budget 0.0100)\n"
            "[ 5500.0s] power: node 3000 s hibernating: EC 989.0 -> 988.6 (0.0001 EC/s, budget 0.0100)\n"
            "[ 8600.0s] power: node 900 s AWAKE: EC 988.6 -> 980.0 (0.0096 EC/s, budget 0.0100)\n"
            "[ 8601.0s] power: measured hib drain 0.0200 EC/s > budget 0.0100 - budgeting it from now on\n")
    segs = [(float(a), float(b) - float(c)) for a, b, c in cs.RE_HIB.findall(text)]
    assert [t for t, _ in segs] == [5000.0, 3000.0]
    assert sum(e for _, e in segs) == pytest.approx(1.2)
