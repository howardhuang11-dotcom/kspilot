#!/usr/bin/env python3
"""EXPLORATORY ablation and dispersion record (report Section 4.8).

usage:
    python paper/analysis/ablation_record.py [LOGS_DIR] [--out PAPER_DIR]

Reads experiments.jsonl rows that carry "exploratory" (set by KSP_ABL_TAG = the arm), plus the <key>_landing.csv[.gz] of each one. LOGS_DIR defaults to data/raw/logs of this repository. Writes PAPER_DIR/tables/ablation_dispersion.csv (one
row per arm) and ablation_dispersion_flights.csv (one row per flight). Only the base names of the logs go into the outputs.

Definitions are the ones in divert_flight_record.py / compute_stats.parse_landing_csv:
    touchdown miss       result.miss_td: hypot(e, n) at the first state after the control loop ends (touchdown)
    post-settle miss     result.miss, about 13 s after touchdown (includes any tip-over)
    offset at ignition   hypot(e, n) on the first BURN row of the landing CSV
    propellant left      result.fuel_left x compute_stats.LFOX_KG_PER_LF (LF + Ox, kg), post-settle
    crash                destroyed, a part lost (parts below the arm's save baseline), or upright <= 0.95
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import compute_stats as cs  # noqa: E402

# arm -> (label, element, saved state, craft, intact part count)
ARMS = [
    ("base_mid", "baseline", "none", "g5_mid", 26),
    ("a_nobrakes", "A1 no airbrakes", "airbrakes never deployed in the descent", "g5_mid", 26),
    ("b_noglide", "A2 no glide", "no glide / ZEM-ZEV below 10 km", "g5_mid", 26),
    ("d_nodraglearn", "A3 no drag learning", "in-flight drag scale fixed at nominal", "g5_mid", 26),
    ("n_navnoise", "D1 nav noise", "nav noise: position 5 m, velocity 0.5 m/s (1 sigma, per tick)", "g5_mid", 26),
    ("k_dragbias", "D2 drag bias +20 %", "guidance drag model x 1.2", "g5_mid", 26),
    ("base_entry", "baseline (entry)", "none", "g5_entry", 34),
    ("c_noxrange", "A4 no entry x-range", "no entry-burn cross-range steering / vernier", "g5_entry", 34),
]
KIND = {"base_mid": "baseline", "base_entry": "baseline", "n_navnoise": "dispersion", "k_dragbias": "dispersion"}
FLIGHT_FIELDS = ["arm", "kind", "save", "log", "code", "time", "miss_td_m", "miss_post_settle_m", "offset_ign_m",
                 "h_ign_m", "v_td_m_s", "upright", "parts", "propellant_left_kg", "crash", "pass"]
ARM_FIELDS = ["arm", "label", "kind", "element", "save", "n", "n_crash", "n_upright", "n_pass", "miss_td_median_m",
              "miss_td_max_m", "miss_post_settle_median_m", "miss_post_settle_max_m", "offset_ign_median_m",
              "v_td_median_m_s", "v_td_max_m_s", "propellant_left_median_kg", "propellant_left_min_kg",
              "d_miss_td_median_vs_base_m", "d_propellant_median_vs_base_kg", "status"]


def flights(logs):
    rows, _ = cs.load_jsonl(logs)
    out = []
    parts0 = {a[0]: a[4] for a in ARMS}
    save = {a[0]: a[3] for a in ARMS}
    for r in rows:
        arm = r.get("exploratory")
        if arm not in parts0:
            continue
        res = r["result"]
        lc = cs.parse_landing_csv(os.path.join(logs, r["key"] + "_landing.csv")) if r["key"] else None
        ign = (lc or {}).get("ign") or {}
        crash = ("destroyed" in res["situation"] or int(res["parts"]) < parts0[arm] or float(res["upright"]) <= 0.95)
        out.append(dict(arm=arm, kind=KIND.get(arm, "ablation"), save=save[arm], log=r["key"], code=r.get("git", "-"),
                        time=r["time"], miss_td_m=float(res.get("miss_td", float("nan"))),
                        miss_post_settle_m=float(res["miss"]), offset_ign_m=float(ign.get("offset", float("nan"))),
                        h_ign_m=float(ign.get("h", float("nan"))), v_td_m_s=float(res["v_td"]),
                        upright=float(res["upright"]), parts=int(res["parts"]),
                        propellant_left_kg=float(res["fuel_left"]) * cs.LFOX_KG_PER_LF, crash=bool(crash),
                        **{"pass": bool(res["ok"])}))
    return out


def med(x):
    x = [v for v in x if not math.isnan(v)]
    return float(np.median(x)) if x else float("nan")


def arms(fl):
    by = {}
    for f in fl:
        by.setdefault(f["arm"], []).append(f)
    out = []
    for arm, label, element, save, _ in ARMS:
        fs = by.get(arm, [])
        base = by.get("base_mid" if save == "g5_mid" else "base_entry", [])
        g = lambda k, fs=fs: [f[k] for f in fs]
        row = dict(arm=arm, label=label, kind=KIND.get(arm, "ablation"), element=element, save=save, n=len(fs),
                   n_crash=sum(g("crash")), n_upright=sum(1 for u in g("upright") if u > 0.95), n_pass=sum(g("pass")),
                   miss_td_median_m=med(g("miss_td_m")), miss_td_max_m=max(g("miss_td_m"), default=float("nan")),
                   miss_post_settle_median_m=med(g("miss_post_settle_m")),
                   miss_post_settle_max_m=max(g("miss_post_settle_m"), default=float("nan")),
                   offset_ign_median_m=med(g("offset_ign_m")), v_td_median_m_s=med(g("v_td_m_s")),
                   v_td_max_m_s=max(g("v_td_m_s"), default=float("nan")),
                   propellant_left_median_kg=med(g("propellant_left_kg")),
                   propellant_left_min_kg=min(g("propellant_left_kg"), default=float("nan")),
                   d_miss_td_median_vs_base_m=med(g("miss_td_m")) - med([f["miss_td_m"] for f in base]),
                   d_propellant_median_vs_base_kg=med(g("propellant_left_kg")) - med([f["propellant_left_kg"] for f in base]),
                   status="EXPLORATORY")
        out.append(row)
    return out


def cell(v):
    if isinstance(v, float):
        return "" if math.isnan(v) else "%.2f" % v
    return v


def write(paper, fl, ar):
    t = os.path.join(paper, "tables")
    os.makedirs(t, exist_ok=True)
    with open(os.path.join(t, "ablation_dispersion.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, ARM_FIELDS, lineterminator="\n")
        w.writeheader()
        for r in ar:
            w.writerow({k: cell(r[k]) for k in ARM_FIELDS})
    with open(os.path.join(t, "ablation_dispersion_flights.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, FLIGHT_FIELDS, lineterminator="\n")
        w.writeheader()
        for r in fl:
            w.writerow({k: cell(r[k]) for k in FLIGHT_FIELDS})
    return [os.path.join(t, n) for n in ("ablation_dispersion.csv", "ablation_dispersion_flights.csv")]


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="?", default=cs.DEFAULT_LOGS,
                    help="raw-record directory (default: data/raw/logs of this repository)")
    ap.add_argument("--out", default=os.path.dirname(HERE))
    a = ap.parse_args(argv)
    logs = os.path.abspath(a.logs)
    cs.require_logs(logs)
    fl = flights(logs)
    ar = arms(fl)
    print(write(a.out, fl, ar))
    for r in ar:
        print("%-14s n=%d crash=%d upright=%d pass=%d miss_td med %s max %s settle med %s ign %s vtd %s prop %s" % (
            r["arm"], r["n"], r["n_crash"], r["n_upright"], r["n_pass"], cell(r["miss_td_median_m"]),
            cell(r["miss_td_max_m"]), cell(r["miss_post_settle_median_m"]), cell(r["offset_ign_median_m"]),
            cell(r["v_td_median_m_s"]), cell(r["propellant_left_median_kg"])))


if __name__ == "__main__":
    main()
