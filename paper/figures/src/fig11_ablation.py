"""Fig. 11 | EXPLORATORY ablation and dispersion (report Section 4.8). Flight
data only: the flights tagged "exploratory" in experiments.jsonl, read through paper/analysis/ablation_record.py,
so the figure and paper/tables/ablation_dispersion*.csv cannot diverge. Panel a: touchdown miss per arm (dot), with a
tick at the post-settle miss. Panel b: propellant left. Panel c: touchdown speed. The g5_mid arms (15 km, mission mass)
and the g5_entry pair (entry interface, DOCK vehicle) are separate saved states, and only arms within one group are
compared. Arms: A1-A4 are the four guidance elements removed one at a time, D1-D2 the two dispersions; an arm of fewer
than three flights is drawn as markers only (no median bar)."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "analysis"))
import style as s  # noqa: E402
import ablation_record as ar  # noqa: E402

C_TAG = "0.15"
C_KIND = {"baseline": s.OI["grey"], "ablation": s.OI["blue"], "dispersion": s.OI["orange"]}


def tag(ax, text="EXPLORATORY, flight record", loc="upper right"):
    x, ha = (0.98, "right") if "right" in loc else (0.02, "left")
    y, va = (0.98, "top") if "upper" in loc else (0.02, "bottom")
    ax.text(x, y, text, transform=ax.transAxes, ha=ha, va=va, fontsize=5.5, fontstyle="italic", color=C_TAG,
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec=C_TAG, lw=0.4), zorder=10)


def make(ctx):
    s.setup()
    fl = ar.flights(ctx["logs"])
    rows = ar.arms(fl)
    rows = [r for r in rows if r["n"] > 0]
    xs = {r["arm"]: i + (0.6 if r["save"] == "g5_entry" else 0.0) for i, r in enumerate(rows)}
    f = s.fig(s.W2, 70)
    axes = [f.add_axes([0.06 + 0.33 * j, 0.30, 0.26, 0.62]) for j in range(3)]
    rng = np.random.default_rng(0)
    for r in rows:
        fs = [x for x in fl if x["arm"] == r["arm"]]
        c = C_KIND[r["kind"]]
        jit = (rng.random(len(fs)) - 0.5) * 0.3
        x0 = xs[r["arm"]]
        for j_, x in zip(jit, fs):
            mk = "x" if x["crash"] else "o"
            axes[0].plot([x0 + j_], [max(x["miss_td_m"], 0.05)], mk, ms=3, color=c, mew=0.8, zorder=3)
            axes[0].plot([x0 + j_], [max(x["miss_post_settle_m"], 0.05)], "_", ms=5, mew=0.7, color="0.2", zorder=2)
            axes[1].plot([x0 + j_], [x["propellant_left_kg"]], mk, ms=3, color=c, mew=0.8)
            axes[2].plot([x0 + j_], [x["v_td_m_s"]], mk, ms=3, color=c, mew=0.8)
        if r["n"] < 3:           # no median bar for arms of two flights (markers only)
            continue
        for ax, k in ((axes[0], "miss_td_median_m"), (axes[1], "propellant_left_median_kg"),
                      (axes[2], "v_td_median_m_s")):
            ax.plot([x0 - 0.25, x0 + 0.25], [r[k]] * 2, color=c, lw=0.9, zorder=4)
    xt = [xs[r["arm"]] for r in rows]
    lab = ["%s\nn = %d" % (r["label"], r["n"]) for r in rows]
    split = max([xs[r["arm"]] for r in rows if r["save"] == "g5_mid"], default=None)
    for ax in axes:
        ax.set_xticks(xt)
        ax.set_xticklabels(lab, rotation=55, ha="right", fontsize=5)
        ax.set_xlim(-0.6, max(xt) + 0.6)
        if split is not None and any(r["save"] == "g5_entry" for r in rows):
            ax.axvline(split + 0.8, color="0.6", lw=0.4, ls=(0, (2, 2)))
    axes[0].set_yscale("log")
    axes[0].axhline(20.0, color=s.C_FAIL, lw=0.5, ls=(0, (3, 2)))
    axes[0].axhline(5.0, color="0.5", lw=0.4, ls=(0, (1, 2)))
    axes[0].text(0.02, 20.0 * 1.08, "20 m mission criterion", transform=axes[0].get_yaxis_transform(), fontsize=5,
                 color=s.C_FAIL, va="bottom")
    axes[0].set_ylabel("Touchdown miss (m); tick: post-settle")
    axes[1].set_ylabel("Propellant left (kg)")
    axes[2].set_ylabel("Touchdown speed (m/s)")
    axes[2].set_ylim(bottom=0)
    f.text(0.5, 0.008, "EXPLORATORY; flight record, n per arm under each label",
           ha="center", va="bottom", fontsize=5.5, fontstyle="italic", color=C_TAG)
    axes[2].plot([], [], "o", ms=3, color=C_KIND["baseline"], label="baseline")
    axes[2].plot([], [], "o", ms=3, color=C_KIND["ablation"], label="ablation")
    axes[2].plot([], [], "o", ms=3, color=C_KIND["dispersion"], label="dispersion")
    axes[2].plot([], [], "x", ms=3, color="0.3", label="crash")
    axes[2].legend(loc="upper right", fontsize=5, frameon=False)
    for a_, l, x in ((axes[0], "a", 0.003), (axes[1], "b", 0.333), (axes[2], "c", 0.663)):
        s.label(a_, l, fig_coords=(x, 0.99))
    return dict(files=s.save(f, ctx["out"], "fig11_ablation"), n_flights=len(fl))


if __name__ == "__main__":
    import _main
    _main.main(make)
