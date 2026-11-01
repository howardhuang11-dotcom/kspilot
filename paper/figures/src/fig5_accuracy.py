"""Fig. 5 | Landing accuracy.
a, touchdown position (east, north) relative to the target from the last landing-CSV row, by population.
b, empirical cumulative distribution of the miss distance (experiment log, 0.01 m).
c, touchdown speed against propellant remaining for the successful campaign missions."""
import math
import os
from datetime import datetime
import sys

import numpy as np
from matplotlib.lines import Line2D

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import data as _d0  # noqa: E402,F401  (puts paper/analysis on the path)
import compute_stats as cs  # noqa: E402
import data as d  # noqa: E402
import style as s  # noqa: E402

MATRIX_MISSIONS = None    # filled from matrix rows


DOCK_END = datetime(2026, 10, 22, 12, 0, 0)


def populations(D):
    wd_by = {r["wd"]: r for r in D["wd"]}
    mx_wd = set(r["wd"] for r in D["mx"] if r["result"] == "PASS")
    camp = [r for r in D["done"] if r["result"] == "PASS"]
    matrix = [r for r in camp if r["wd"] in mx_wd]
    base = [r for r in camp if r["wd"] not in mx_wd]
    gates = d.gate_rows(D)
    # the three docking-variant landings cited in the report; the later series
    # (from 14:05) is not part of the report
    dock = [r for r in D["post"] if r["craft"] == "T-WorksFineGF-DOCK" and r["result"] == "PASS" and r["key"]
            and r["start"] < DOCK_END]

    def en(key):
        T = d.telemetry(D["logs"], key)
        return (T["e"][-1], T["n"][-1]) if T else (float("nan"), float("nan"))
    P = {}
    P["campaign"] = dict(e_n=[en(r["key"]) for r in base], miss=[r["miss2"] for r in base])
    P["matrix"] = dict(e_n=[en(r["key"]) for r in matrix], miss=[r["miss2"] for r in matrix])
    P["gates"] = dict(e_n=[en(g["key"]) for g in gates], miss=[g["result"]["miss"] for g in gates])
    P["dock"] = dict(e_n=[en(r["key"]) for r in dock], miss=[r["miss2"] for r in dock],
                     offset=["aim=" in r["detail"] for r in dock])
    P["camp_all_pass"] = camp
    return P


STY = {"campaign": dict(c=s.C_CAMP, m="o", lab="exploratory, not matrix"),
       "matrix": dict(c=s.C_MATRIX, m="D", lab="robustness matrix (exploratory)"),
       "gates": dict(c=s.C_GATE, m="s", lab="regression gates, from 4 km"),
       "dock": dict(c=s.C_DOCK, m="^", lab="docking variant, after freeze")}


def panel_a(ax, P):
    for rr in (1, 5):
        ax.add_patch(__import__("matplotlib").patches.Circle((0, 0), rr, fill=False, lw=0.5, ls=(0, (3, 2)),
                                                             color="0.5"))
        ax.text(rr * math.cos(math.radians(50)) + 0.12, rr * math.sin(math.radians(50)) + 0.12, "%d m" % rr,
                fontsize=5, color="0.4")
    ax.axhline(0, color="0.85", lw=0.4, zorder=0)
    ax.axvline(0, color="0.85", lw=0.4, zorder=0)
    for k in ("gates", "campaign", "matrix", "dock"):
        st = STY[k]
        e, n = np.array(P[k]["e_n"]).T
        ax.scatter(e, n, s=9, marker=st["m"], facecolor=st["c"], edgecolor="white", lw=0.3,
                   label="%s (n = %d)" % (st["lab"], len(e)), zorder=3, alpha=0.9)
    ax.set_xlim(-6, 6)
    ax.set_ylim(-6, 6)
    ax.set_aspect("equal")
    ax.set_xlabel("East of target (m)")
    ax.set_ylabel("North of target (m)")
    lg = ax.legend(loc="lower left", fontsize=5, handletextpad=0.2, borderaxespad=0.1, labelspacing=0.25,
                   frameon=True, framealpha=0.9, edgecolor="none", facecolor="white")
    lg.get_frame().set_linewidth(0)


def ecdf(ax, x, **kw):
    x = np.sort(np.asarray(x))
    y = np.arange(1, len(x) + 1) / len(x)
    ax.step(np.r_[x[0], x], np.r_[0, y], where="post", **kw)


def panel_b(ax, P):
    camp_all = [r["miss2"] for r in P["camp_all_pass"]]
    ecdf(ax, camp_all, color=s.C_CAMP, lw=1.0, label="exploratory successes (n = %d)" % len(camp_all))
    ecdf(ax, P["matrix"]["miss"], color=s.C_MATRIX, lw=0.8, label="matrix cases (n = %d)" % len(P["matrix"]["miss"]))
    ecdf(ax, P["gates"]["miss"], color=s.C_GATE, lw=0.8, label="gate landings (n = %d)" % len(P["gates"]["miss"]))
    ecdf(ax, P["dock"]["miss"], color=s.C_DOCK, lw=0.8, ls=(0, (3, 1.5)),
         label="docking variant (n = %d)" % len(P["dock"]["miss"]))
    for xv, txt in ((2.0, "2 m"), (5.0, "5 m (gate limit)")):
        ax.axvline(xv, color="0.6", lw=0.4, ls=(0, (2, 2)))
        ax.text(xv * 1.03, 0.30, txt, fontsize=5, color="0.4", rotation=90, va="bottom")
    ax.set_xscale("log")
    ax.set_xlim(0.9, 40)
    ax.set_xticks([1, 2, 5, 10, 20])
    ax.set_xticklabels(["1", "2", "5", "10", "20"])
    ax.set_ylim(0, 1.02)
    ax.set_xlabel("Miss distance (m)")
    ax.set_ylabel("Cumulative fraction")
    ax.legend(loc="lower right", fontsize=5, handlelength=1.2, borderaxespad=0.1)
    ax.text(38, 0.97, "not shown: mission 21,\n3,776 m (landing\ncriterion failed)", fontsize=5, ha="right", va="top",
            color=s.C_FAIL)


def panel_c(ax, P):
    camp = P["camp_all_pass"]
    groups = [("T-WorksFineGF", "2.48 t", s.C_CAMP, "o"), ("T-WorksFineGF-LIT", "2.48 t + lamps", s.OI["sky"], "o"),
              ("T-WorksFineGF-P45", "4.50 t", s.OI["orange"], "s"), ("T-WorksFineGF-P60", "6.00 t", s.OI["vermillion"], "D")]
    for craft, lab, c, m in groups:
        sub = [r for r in camp if r["craft"] == craft]
        ax.scatter([r["v_td"] for r in sub], [r["fuel_left"] * cs.LFOX_KG_PER_LF / 1e3 for r in sub], s=11, marker=m,
                   facecolor=c, edgecolor="white", lw=0.3, label="%s (n = %d)" % (lab, len(sub)), zorder=3)
    v = np.array([r["v_td"] for r in camp])
    ax.axvline(np.median(v), color="0.6", lw=0.4, ls=(0, (2, 2)))
    ax.text(np.median(v) + 0.005, 4.6, "median %.2f m s$^{-1}$" % np.median(v), fontsize=5, color="0.4", va="top")
    ax.set_xlim(0, 0.45)
    ax.set_ylim(2.0, 4.6)
    ax.set_xlabel("Touchdown speed (m s$^{-1}$)")
    ax.set_ylabel("Propellant remaining (t)")
    ax.legend(loc="lower right", fontsize=5, handletextpad=0.2, borderaxespad=0.1, labelspacing=0.25)


def make(ctx):
    s.setup()
    D = d.load(ctx["logs"])
    P = populations(D)
    f = s.fig(s.W2, 62)
    ax_a = f.add_axes([0.055, 0.15, 0.27, 0.80])
    panel_a(ax_a, P)
    ax_b = f.add_axes([0.40, 0.15, 0.25, 0.80])
    panel_b(ax_b, P)
    ax_c = f.add_axes([0.73, 0.15, 0.255, 0.80])
    panel_c(ax_c, P)
    for ax, l, x in ((ax_a, "a", 0.003), (ax_b, "b", 0.35), (ax_c, "c", 0.685)):
        s.label(ax, l, fig_coords=(x, 0.99))
    stats = {k: (len(P[k]["miss"]), float(np.median(P[k]["miss"]))) for k in ("campaign", "matrix", "gates", "dock")}
    return dict(files=s.save(f, ctx["out"], "fig5_landing_accuracy"), n_median=stats)


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import make_all
    make_all.run_one(make)
