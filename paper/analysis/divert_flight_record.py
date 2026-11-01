#!/usr/bin/env python3
"""Measured flight record of landing-burn diverts (Fig. 10b,c and paper/tables/divert_flight_record.*).

usage:
    python paper/analysis/divert_flight_record.py [LOGS_DIR] [--out PAPER_DIR]

Reads experiments.jsonl and the <key>_landing.csv[.gz] telemetry of the selected flights from LOGS_DIR (default
data/raw/logs of this repository; read-only) and
writes PAPER_DIR/tables/divert_flight_record.csv (one row per flight) and .md (the gate set as one summary row).
PAPER_DIR defaults to the parent of this script's directory. Only base names of the logs reach the outputs.

Definitions (identical to compute_stats.parse_landing_csv):
    offset at ignition   hypot(e, n) on the first BURN row of the landing CSV (m from the pad)
    touchdown offset     hypot(e, n) on the last row of the landing CSV
    post-settle miss     experiments.jsonl result.miss, recorded about 13 s after touchdown (includes any tip-over)
    closure              offset at ignition minus touchdown offset
    mass at ignition     CSV mass on the first BURN row
    propellant used      CSV mass on the first BURN row minus mass on the last row (the landing burn)
    propellant left      result.fuel_left x compute_stats.LFOX_KG_PER_LF, recorded post-settle; it reads 0 when the
                         vehicle broke up and lost its tanks (parts < 26), so it is not a fuel-exhaustion flag
    upright, v_td        experiments.jsonl result.upright (cosine of the tilt, post-settle) and result.v_td (m/s)

Selection (hard-coded below; rule from the plant-model note PLANT_V2.md, section 2.2, of the development repository):
    large offset  every T-WorksFineGF landing burn of set O (flown burn law) lit with an offset >= 100 m and
                  < 1,000 m (burns lit > 1,000 m from the target give up the target and are excluded)
    gate          set G: the 47 runs of the 4 km regression gate, all started from one saved state (g5_term_GF,
                  12,977 kg); this is one test repeated, not 47 independent approaches
    divert series the four flights of 22 October 2026 from saved state g5_mid (15 km, mission mass): flight 0 (no
                  offset), flight 1 (flown law, injected 50 m ignition offset) and E-50 #1 / #2 (law E, "bt",
                  default off and not frozen; #2 holds E's translate leg down to 60 m above the pad)

Only the Python standard library and numpy are required (numpy for the burn time series used by the figure).
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import compute_stats as cs  # noqa: E402

CRAFT = "T-WorksFineGF"
# log time stamps (key = <stamp>_orbit_T-WorksFineGF); experiments.jsonl 0-based rows 1, 4, 7, 10, 11, 88, 90, 91
LARGE = ["20261020_011739", "20261020_013145", "20261020_014605", "20261020_022311", "20261020_022422",
         "20261020_231841", "20261020_235036", "20261021_000635"]
# experiments.jsonl rows 74-82, 84-86, 92-97, 99-101, 110-112, 114-116, 127-129, 131-133, 135-138, 140, 143-145,
# 152-154, 158-160
GATE = ["20261020_210913", "20261020_211035", "20261020_211156", "20261020_213611", "20261020_213732",
        "20261020_213853", "20261020_224329", "20261020_224449", "20261020_224611", "20261020_225727",
        "20261020_225848", "20261020_230009", "20261021_001949", "20261021_002110", "20261021_002232",
        "20261021_003133", "20261021_003254", "20261021_003416", "20261021_010651", "20261021_010813",
        "20261021_010934", "20261021_032509", "20261021_032620", "20261021_032731", "20261021_043205",
        "20261021_043316", "20261021_043426", "20261021_104559", "20261021_104720", "20261021_104831",
        "20261021_120755", "20261021_120906", "20261021_121017", "20261021_134444", "20261021_134725",
        "20261021_134919", "20261021_135131", "20261021_151406", "20261021_154753", "20261021_154904",
        "20261021_155015", "20261021_180129", "20261021_180240", "20261021_180351", "20261022_000606",
        "20261022_000717", "20261022_000828"]
# label, law, stamp, law-E exit rule; experiments.jsonl rows 165-168
SERIES = [("flight 0 (no offset)", "flown", "20261022_022133", None),
          ("flight 1", "flown", "20261022_022355", None),
          ("E-50 #1", "E", "20261022_030742", "capture"),
          ("E-50 #2", "E", "20261022_032122", "hmin")]
G_LARGE, G_GATE, G_SERIES = "large-offset flown law", "4 km gate (one saved state)", "divert series (g5_mid)"

# kspilot/landing.py: the burn law measures height above the 4 m gate; position chasing stops below commit_h = 25 m
# above the gate, i.e. 29 m above the pad (the logged h). Law E: bt_h 150 m, h_b = bt_h + 0.6 x offset at ignition,
# translate leg from h <= h_b + 5 m; bt_hmin 60 m above the pad.
GATE_H, COMMIT_H = 4.0, 25.0
COMMIT_ABOVE_PAD = GATE_H + COMMIT_H
BT_H, BT_K, BT_HMIN = 150.0, 0.6, 60.0

FIELDS = ["label", "group", "law", "log", "code", "log_time", "mass_ign_kg", "h_ign_m", "offset_ign_m",
          "offset_td_m", "miss_post_settle_m", "closure_td_m", "upright", "v_td_m_s", "parts", "propellant_used_kg",
          "propellant_left_kg", "handover_h_m"]


def key(stamp):
    return "%s_orbit_%s" % (stamp, CRAFT)


def load_runs(logs):
    """experiments.jsonl rows by path-free key (the last row wins, as in paper/figures/src/data.py)."""
    rows, _ = cs.load_jsonl(logs)
    return {r["key"]: r for r in rows if r["key"]}


def burn_track(logs, k):
    """BURN rows of <k>_landing.csv as numpy arrays (game time from ignition, h above the pad, e, n, speeds, mass)."""
    with cs.raw_open(os.path.join(logs, k + "_landing.csv"), "rt", encoding="utf-8", errors="replace") as f:
        rows = [r for r in csv.DictReader(f) if r.get("phase") == "BURN"]
    tr = {c: np.array([float(r[c]) for r in rows]) for c in ("ut", "h", "e", "n", "ve", "vn", "vu", "m")}
    tr["t"] = tr["ut"] - tr["ut"][0]
    tr["d"] = np.hypot(tr["e"], tr["n"])
    tr["vh"] = np.hypot(tr["ve"], tr["vn"])
    return tr


def handover(tr, rule):
    """Index of the first BURN row flown by the normal (flown) law after law E. 'capture': E-50 #1 code (fea70ce), E
    exits once in its translate leg (h <= h_b + 5 m) within 4 m and 1 m/s of the pad. 'hmin': E-50 #2 code (eaaebdc),
    E is held to bt_hmin = 60 m above the pad. Replays the documented exit rule on the logged rows."""
    if rule == "capture":
        hb = BT_H + BT_K * tr["d"][0]
        ok = (tr["h"] <= hb + 5.0) & (tr["d"] < 4.0) & (tr["vh"] < 1.0)
    elif rule == "hmin":
        ok = tr["h"] < BT_HMIN
    else:
        return None
    idx = np.flatnonzero(ok)
    return int(idx[0]) if idx.size else None


def record(logs, runs, k, label, group, law, rule=None):
    js = runs[k]
    res = js["result"]
    tr = burn_track(logs, k)
    lc = cs.parse_landing_csv(os.path.join(logs, k + "_landing.csv"))
    off_ign = lc["ign"]["offset"]
    off_td = math.hypot(lc["contact_e"], lc["contact_n"])
    assert abs(off_ign - tr["d"][0]) < 1e-9 and abs(tr["h"][0] - lc["ign"]["h"]) < 1e-9
    ho = handover(tr, rule)
    st = k.split("_")
    return dict(label=label, group=group, law=law, log=k, code=js.get("git", "-"),
                log_time="%s-%s-%s %s:%s" % (st[0][:4], st[0][4:6], st[0][6:], st[1][:2], st[1][2:4]),
                mass_ign_kg=lc["ign"]["m"], h_ign_m=lc["ign"]["h"], offset_ign_m=off_ign, offset_td_m=off_td,
                miss_post_settle_m=float(res["miss"]), closure_td_m=off_ign - off_td, upright=float(res["upright"]),
                v_td_m_s=float(res["v_td"]), parts=int(res["parts"]), propellant_used_kg=tr["m"][0] - tr["m"][-1],
                propellant_left_kg=float(res["fuel_left"]) * cs.LFOX_KG_PER_LF,
                handover_h_m=float(tr["h"][ho]) if ho is not None else float("nan"))


def records(logs):
    runs = load_runs(logs)
    need = [key(s) for s in LARGE + GATE] + [key(s[2]) for s in SERIES]
    lost = [k + "_landing.csv[.gz]" for k in need if not cs.raw_exists(os.path.join(logs, k + "_landing.csv"))]
    lost += [k + " (row of experiments.jsonl)" for k in need if k not in runs]
    if lost:
        cs.die("the divert flight record needs %d records that are not in %s: %s%s\n%s" % (
            len(lost), logs, ", ".join(lost[:4]), " ..." if len(lost) > 4 else "", cs.RAW_HINT))
    out = [record(logs, runs, key(s), "L%d" % (i + 1), G_LARGE, "flown") for i, s in enumerate(LARGE)]
    out += [record(logs, runs, key(s), "gate %d" % (i + 1), G_GATE, "flown") for i, s in enumerate(GATE)]
    out += [record(logs, runs, key(s), lab, G_SERIES, law, rule) for lab, law, s, rule in SERIES]
    return out


def tracks(logs):
    """Burn time series of the three divert flights (flight 1, E-50 #1, E-50 #2) with their hand-over rows."""
    out = []
    for lab, law, s, rule in SERIES[1:]:
        tr = burn_track(logs, key(s))
        tr.update(label=lab, law=law, handover=handover(tr, rule))
        out.append(tr)
    return out


ND = dict(mass_ign_kg=0, h_ign_m=0, offset_ign_m=1, offset_td_m=1, miss_post_settle_m=2, closure_td_m=1,
          upright=3, v_td_m_s=2, propellant_used_kg=0, propellant_left_kg=0, handover_h_m=1)


def cell(r, f):
    v = r[f]
    if f not in ND:
        return str(v)
    t = cs.rnd(v, ND[f])
    return t[1:] if t.startswith("-") and not t.strip("-0.") else t     # no "-0.0"


def write(recs, paper):
    tdir = os.path.join(paper, "tables")
    os.makedirs(tdir, exist_ok=True)
    with open(os.path.join(tdir, "divert_flight_record.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(FIELDS)
        for r in recs:
            w.writerow([cell(r, c) for c in FIELDS])
    cols = [("label", "Flight"), ("law", "Law"), ("log", "Log"), ("code", "Code"), ("mass_ign_kg", "Mass at ign. (kg)"),
            ("h_ign_m", "Ign. height (m)"), ("offset_ign_m", "Offset at ign. (m)"), ("offset_td_m", "Touchdown offset (m)"),
            ("miss_post_settle_m", "Post-settle miss (m)"), ("closure_td_m", "Closure (m)"), ("upright", "Upright"),
            ("v_td_m_s", "v_td (m/s)"), ("propellant_used_kg", "Burn propellant (kg)"),
            ("propellant_left_kg", "Propellant left (kg)")]
    lines = ["# Landing-burn divert: measured flight record", "",
             "Generated by `paper/analysis/divert_flight_record.py` from `experiments.jsonl` and the landing CSVs. "
             "Offset at ignition: horizontal distance from the pad on the first BURN row; touchdown offset: on the last "
             "CSV row; post-settle miss: recorded about 13 s after touchdown; closure = offset at ignition minus "
             "touchdown offset.", ""]
    for g in (G_LARGE, G_GATE, G_SERIES):
        rs = [r for r in recs if r["group"] == g]
        lines += ["## %s (n = %d)" % (g[0].upper() + g[1:], len(rs)), "",
                  "| " + " | ".join(h for _, h in cols) + " |", "|" + "---|" * len(cols)]
        if g == G_GATE:
            def summ(f):
                xs = [r[f] for r in rs]
                return "%s (%s-%s)" % (cs.rnd(cs.quantile(xs, 0.5), ND[f]), cs.rnd(min(xs), ND[f]),
                                       cs.rnd(max(xs), ND[f]))
            codes = sorted(set(r["code"] for r in rs))
            row = ["median (range), n = %d" % len(rs), "flown", "%s ... %s" % (rs[0]["log"][:15], rs[-1]["log"][:15]),
                   "%d revisions" % len(codes)] + [summ(f) for f, _ in cols[4:]]
            lines.append("| " + " | ".join(row) + " |")
        else:
            for r in rs:
                lines.append("| " + " | ".join(cell(r, f) for f, _ in cols) + " |")
        lines.append("")
    ser = {r["label"]: r for r in recs if r["group"] == G_SERIES}
    lines += ["Law E hand-over to the flown law (replay of the documented exit rule on the logged rows): "
              "E-50 #1 at %s m above the pad (pad capture), E-50 #2 at %s m (held to 60 m). Position chasing stops "
              "%d m above the pad (commit height, 25 m above the 4 m gate)." % (
                  cs.rnd(ser["E-50 #1"]["handover_h_m"], 1), cs.rnd(ser["E-50 #2"]["handover_h_m"], 1),
                  COMMIT_ABOVE_PAD), "",
              "Propellant left is the post-settle record; it reads 0 for vehicles that broke up after touchdown "
              "(parts < 26) and lost their tanks, so it does not mean the burn ran dry. The full per-flight gate rows "
              "are in `divert_flight_record.csv`.", ""]
    with open(os.path.join(tdir, "divert_flight_record.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines))
    return [os.path.join(tdir, "divert_flight_record" + e) for e in (".csv", ".md")]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="?", default=cs.DEFAULT_LOGS,
                    help="raw-record directory (default: data/raw/logs of this repository)")
    ap.add_argument("--out", default=os.path.dirname(HERE))
    a = ap.parse_args(argv)
    logs = os.path.abspath(a.logs)
    cs.require_logs(logs)
    recs = records(logs)
    for p in write(recs, os.path.abspath(a.out)):
        print("wrote", os.path.relpath(p, os.path.dirname(os.path.abspath(a.out))))
    for r in recs:
        if r["group"] != G_GATE:
            print("%-22s %-6s off %6s td %6s miss %7s closure %6s m %6s" % (
                r["label"], r["law"], cell(r, "offset_ign_m"), cell(r, "offset_td_m"), cell(r, "miss_post_settle_m"),
                cell(r, "closure_td_m"), cell(r, "mass_ign_kg")))


if __name__ == "__main__":
    main()
