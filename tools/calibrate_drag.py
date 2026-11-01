"""Learn the drag-model correction k(alt) = measured / table from a flight's landing CSV and merge it
into logs/drag_calib_<craft>.json (the file tools/orbit_recover.py reads), separately for brakes OUT (k_out)
and brakes IN (k_in); "k" is the combined curve kept for older readers.

Samples: unpowered COAST only, no thrust in the last 2 s, no part loss, q > 30 Pa, and none within
SETTLE_S of a brake change (CA-2: the fin + airbrake animation takes 3.5-5 s). Airspeed and altitude come
from the CSV's speed / h columns (h is the leg-feet height above the pad: alt = h + h_com + pad altitude).

The tables are the craft's g5_entry_<craft>.json (CA-4 deterministic measurement). A calibration file whose
"semantics" differs from SEMANTICS was learned against other tables and is replaced, not blended.
usage: calibrate_drag.py [LANDING_CSV [CRAFT]]   (default: newest orbit landing CSV, craft from its name)"""
import csv
import glob
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from kspilot.entry import DragTable, KSC_PAD_ALT  # noqa: E402

SEMANTICS = "CA-4"           # pre-entry measurement semantics the k values are relative to (d8de707)
BANDS = list(range(0, 72000, 4000))
SETTLE_S, H_COM = 5.0, 7.4


def samples(path, entry_json):
    d = json.load(open(entry_json))
    tabs = {1: DragTable(d["tab_out"]["alts"], d["tab_out"]["speeds"], d["tab_out"]["F"]),
            0: DragTable(d["tab_in"]["alts"], d["tab_in"]["speeds"], d["tab_in"]["F"])}
    h_com = d.get("h_com", H_COM)
    out, prev_b, prev_m, t_change, t_thr = [], None, None, -1e9, -1e9
    for r in csv.DictReader(open(path)):
        try:
            t, b, m, thr = float(r["ut"]), int(r["brakes"]), float(r["m"]), float(r["thr_act"])
            drag, spd, alt = float(r["drag"]), float(r["speed"]), float(r["h"]) + h_com + KSC_PAD_ALT
        except (ValueError, KeyError):
            continue
        if prev_b is not None and b != prev_b:
            t_change = t
        if thr >= 0.01 or r["phase"] != "COAST":
            t_thr = t
        lost = prev_m is not None and abs(m - prev_m) > 50
        prev_b, prev_m = b, m
        if t - t_change < SETTLE_S or t - t_thr < 2.0 or lost or not np.isfinite(spd):
            continue
        model = tabs[b](alt, spd)
        if drag > 50 and model > 1 and 0.5 * tabs[b].rho(alt) * spd * spd > 30:
            out.append((alt, b, drag / model))
    return out


def bands(s, state=None):
    res = {}
    for lo in BANDS:
        sel = [k for (a, b, k) in s if lo <= a < lo + 4000 and (state is None or b == state)]
        if len(sel) >= 5:
            res[lo + 2000] = float(np.median(sel))
    return res


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else max(glob.glob(os.path.join(ROOT, "logs", "*orbit*_landing.csv")),
                                                     key=os.path.getmtime)
    craft = sys.argv[2] if len(sys.argv) > 2 else os.path.basename(path).split("_orbit_")[-1].replace("_landing.csv", "")
    ej = os.path.join(ROOT, "logs", "g5_entry_%s.json" % craft)
    cal_f = os.path.join(ROOT, "logs", "drag_calib_%s.json" % craft)
    s = samples(path, ej)
    new = {"k": bands(s), "k_out": bands(s, 1), "k_in": bands(s, 0)}
    cal = json.load(open(cal_f)) if os.path.exists(cal_f) else {}
    if cal.get("semantics", "").split()[:1] != [SEMANTICS]:
        cal = {"alts": [], "k": [], "flights": 0}            # learned against other tables: start over
    merged = {}
    for key, nb in new.items():
        old = dict(zip(cal["alts"], cal.get(key, cal["k"])))
        for a, k in nb.items():
            old[a] = k if a not in old else 0.6 * old[a] + 0.4 * k
        merged[key] = old
    alts = sorted(merged["k"])

    def fill(d):     # a state with no samples in a band: interpolate / clamp its own curve, else the combined one
        xs = sorted(d)
        return [float(np.interp(a, xs, [d[x] for x in xs])) if xs else merged["k"][a] for a in alts]
    cal.update({"alts": alts, "k": [merged["k"][a] for a in alts], "k_out": fill(merged["k_out"]),
                "k_in": fill(merged["k_in"]), "flights": cal.get("flights", 0) + 1,
                "semantics": SEMANTICS + " deterministic pre-entry measurement; k = measured / raw table"})
    json.dump(cal, open(cal_f, "w"), indent=1)
    print("drag calibration %s (%d flights):" % (craft, cal["flights"]),
          " ".join("%.0fkm:%.2f/%.2f" % (a / 1e3, o, i) for a, o, i in zip(alts, cal["k_out"], cal["k_in"])))


if __name__ == "__main__":
    main()
