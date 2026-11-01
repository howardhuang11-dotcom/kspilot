"""Supplementary Fig. 2 | Control-loop timing under recording workloads: game-time interval between telemetry rows during
the entry burn and the landing burn for each campaign mission, by recording mode (landing CSVs, transcripts)."""
import os
import sys

import numpy as np
from matplotlib.lines import Line2D

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import data as d  # noqa: E402
import style as s  # noqa: E402

MODES = [("", "none", s.OI["grey"]), ("screen recording", "real-time screen recording", s.OI["blue"]),
         ("in-game frame capture", "in-game frame capture", s.OI["vermillion"]),
         ("orphan frame capture during ascent (decision record 04:15)", "orphan frame capture (ascent only)",
          s.OI["orange"])]


def make(ctx):
    s.setup()
    D = d.load(ctx["logs"])
    rows = []
    for r in D["done"]:
        T = d.telemetry(ctx["logs"], r["key"])
        if T is None:
            continue
        ut = np.array(T["ut"])
        eb = d.entry_burn_segment(T)
        dt_eb = np.diff(ut[eb[0]:eb[1] + 2])
        burn = [i for i, p in enumerate(T["phase"]) if p == "BURN"]
        dt_lb = np.diff(ut[burn])
        dt_lb = dt_lb[dt_lb > 0]
        rows.append(dict(m=r["mission"], rec=r["recording"] or "", eb=dt_eb, lb=dt_lb, stalls=r["deadman"],
                         res=r["result"]))
    f = s.fig(s.W2, 62)
    axes = [f.add_axes([0.06, 0.17, 0.42, 0.76]), f.add_axes([0.565, 0.17, 0.42, 0.76])]
    for ax, key, title in ((axes[0], "eb", "entry burn"), (axes[1], "lb", "landing burn")):
        for x in rows:
            c = next(cc for k, _, cc in MODES if k == x["rec"])
            v = x[key]
            ax.plot([x["m"]] * 2, [np.percentile(v, 5), np.percentile(v, 95)], color=c, lw=0.7)
            ax.plot(x["m"], np.median(v), "o", color=c, ms=2.8)
            ax.plot(x["m"], v.max(), "_", color=c, ms=4, mew=0.8)
            if key == "eb" and x["stalls"]:
                ax.text(x["m"], v.max() * 1.15, "%d" % x["stalls"], fontsize=5, ha="center", color="k")
        ax.set_yscale("log")
        ax.set_ylim(0.02, 3)
        ax.set_xlim(0, 29)
        ax.set_xlabel("Exploratory mission")
        ax.set_ylabel("Telemetry interval, %s (game s)" % title)
    axes[0].axhline(2.5, color="0.3", lw=0.5, ls=(0, (3, 2)))
    axes[0].text(0.5, 2.6, "stall watchdog 2.5 s (wall clock)", fontsize=5, va="bottom")
    hand = [Line2D([], [], color=c, marker="o", ms=2.8, lw=0.7) for _, _, c in MODES]
    axes[1].legend(hand, [lab for _, lab, _ in MODES], loc="upper left", fontsize=5)
    axes[0].text(28.5, 0.022, "dot median, bar 5th–95th percentile, tick maximum;\nnumbers: control-loop stall events",
                 fontsize=5, ha="right", va="bottom")
    s.label(axes[0], "a", fig_coords=(0.003, 0.99))
    s.label(axes[1], "b", fig_coords=(0.51, 0.99))
    summ = {}
    for k, lab, _ in MODES:
        sub = [x for x in rows if x["rec"] == k]
        if sub:
            summ[lab] = (len(sub), float(np.median([np.median(x["eb"]) for x in sub])),
                         float(np.median([np.median(x["lb"]) for x in sub])))
    return dict(files=s.save(f, ctx["out"], "sfig2_timing"), summary=summ)


if __name__ == "__main__":
    import _main
    _main.main(make)
