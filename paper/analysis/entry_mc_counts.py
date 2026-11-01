#!/usr/bin/env python3
"""Authoritative counts of the entry-burn Monte Carlo (report Section 5.2, Table 7, Fig. 12b).

usage:
    python paper/analysis/entry_mc_counts.py [GNC2_DIR] [--json]

Reads entry_mc.json (mild set, 36 draws), entry_mc_stress.json (stress set, 18 draws) and entry_replay.json from
GNC2_DIR (default data/raw/gnc2 of this repository; read-only) and prints, for each set and each cutoff, how many draws
leave the along-track error at 10 km (field E10_along) outside the stated nominal band +90 ... +175 m, and the total
over the 54 draws. The same counts are given for the two wider bands that were considered (+90 ... +185 m and
+90 ... +190 m), which were chosen after the draws were seen. Band edges are inclusive. The cutoffs are the keys
"counter" (counted-delta-v cutoff, the one flown) and "pcc" (predictor-corrector cutoff, not in the released code).

It also prints the replay of mission 21 (entry_replay.json, key 20261021_135358) for every counter variant that the
record holds, because the report compares them (Section 5.2): "legacy" and "pcc_legacy" are the counter and the
predictor-corrector in the form in which the failure occurred, "counter" and "pcc" the mass-based counter that has
been flown since mission 22.

Only the Python standard library is needed (Python 3.9 or later). This script is the source of every Monte Carlo count
in the report and the README; if the text and this output differ, the output is right.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DIR = os.path.join(os.path.dirname(os.path.dirname(HERE)), "data", "raw", "gnc2")

SETS = (("mild", "entry_mc.json"), ("stress", "entry_mc_stress.json"))
CUTOFFS = (("counter", "counted-dv cutoff"), ("pcc", "predictor-corrector"))
BANDS = (("stated", 90.0, 175.0), ("+185", 90.0, 185.0), ("+190", 90.0, 190.0))
MISSION_21 = "20261021_135358"


def outside(values, lo, hi):
    return sum(1 for v in values if not (lo <= v <= hi))


def counts(gnc2_dir=DEFAULT_DIR):
    """{set: {"n": int, "<cutoff>": {"min": m, "max": m, "<band>": {"outside": k, "below": b, "above": a}}}} plus totals."""
    res = {}
    totals = {c: {b: 0 for b, _, _ in BANDS} for c, _ in CUTOFFS}
    n_total = 0
    for sname, fname in SETS:
        with open(os.path.join(gnc2_dir, fname)) as fh:
            draws = json.load(fh)
        entry = {"n": len(draws)}
        n_total += len(draws)
        for c, _ in CUTOFFS:
            vals = [d[c]["E10_along"] for d in draws]
            e = {"min": min(vals), "max": max(vals)}
            for b, lo, hi in BANDS:
                k = outside(vals, lo, hi)
                e[b] = {"outside": k, "below": sum(1 for v in vals if v < lo), "above": sum(1 for v in vals if v > hi)}
                totals[c][b] += k
            entry[c] = e
        res[sname] = entry
    res["total"] = {"n": n_total, **{c: {b: {"outside": totals[c][b]} for b, _, _ in BANDS} for c, _ in CUTOFFS}}
    return res


def mission21(gnc2_dir=DEFAULT_DIR):
    with open(os.path.join(gnc2_dir, "entry_replay.json")) as fh:
        r = json.load(fh)[MISSION_21]
    return {k: r[k]["E10_along"] for k in ("log", "legacy", "pcc_legacy", "counter", "pcc") if k in r}


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    as_json = "--json" in argv
    args = [a for a in argv if not a.startswith("--")]
    d = args[0] if args else DEFAULT_DIR
    if not os.path.isfile(os.path.join(d, "entry_mc.json")):
        sys.exit("entry_mc_counts: %s/entry_mc.json not found (see data/raw/README.md)" % d)
    res, m21 = counts(d), mission21(d)
    if as_json:
        print(json.dumps({"counts": res, "mission21_E10_along_m": m21}, indent=1, sort_keys=True))
        return
    print("Draws with along-track error at 10 km outside the band (m); inclusive edges")
    for b, lo, hi in BANDS:
        print("\nband +%g ... +%g m (%s)" % (lo, hi, "stated nominal band" if b == "stated" else "widened after the draws were seen"))
        print("  %-8s %-20s %s" % ("set", "cutoff", "outside (below / above)   range of E10_along"))
        for sname, _ in SETS:
            for c, label in CUTOFFS:
                e = res[sname][c]
                print("  %-8s %-20s %2d of %2d (%2d / %2d)         %+.1f to %+.1f" % (
                    sname, label, e[b]["outside"], res[sname]["n"], e[b]["below"], e[b]["above"], e["min"], e["max"]))
        for c, label in CUTOFFS:
            print("  %-8s %-20s %2d of %2d" % ("total", label, res["total"][c][b]["outside"], res["total"]["n"]))
    print("\nMission 21 replay, E10_along (m): " + ", ".join("%s %+.1f" % (k, v) for k, v in m21.items()))


if __name__ == "__main__":
    main()
