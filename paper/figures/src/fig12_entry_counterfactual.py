"""Fig. 12 | Entry-burn termination: replay of logged burns and bookkeeping Monte Carlo (simulation; GNC2
results entry_replay.json, entry_mc.json, entry_mc_stress.json)."""
import json
import os
import sys

import numpy as np
from matplotlib.lines import Line2D

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import style as s  # noqa: E402

F4 = "20261021_135358"      # campaign mission 21


def make(ctx):
    s.setup()
    G = ctx["gnc2"]
    R = json.load(open(os.path.join(G, "entry_replay.json")))
    mild = json.load(open(os.path.join(G, "entry_mc.json")))
    stress = json.load(open(os.path.join(G, "entry_mc_stress.json")))
    nominal = sorted(k for k in R if k != F4)
    f = s.fig(s.W2, 60)
    # a: logged (counted) vs replayed true dv for the ten nominal flights flown after the accounting correction:
    # entry-gate flights from the pre-entry checkpoint (4), full missions 22-26 (5), one later gate flight (1)
    ax = f.add_axes([0.06, 0.17, 0.25, 0.76])
    for i, k in enumerate(nominal):
        lg = R[k]["log"]
        ax.plot([i + 1] * 2, [lg["dv_count"], lg["dv_true"]], color="0.8", lw=0.6)
        ax.plot(i + 1, lg["dv_count"], "_", color="k", ms=4, mew=0.9)
        ax.plot(i + 1, lg["dv_true"], "o", color=s.C_SIM, ms=2.8)
    ax.axhline(590, color="0.5", lw=0.4, ls=(0, (2, 2)))
    ax.text(len(nominal) + 0.6, 590.4, "plan 590", fontsize=5, va="bottom", ha="right", color="0.4")
    ax.set_xlim(0, len(nominal) + 1)
    ax.set_ylim(588, 626)          # head-room so that the legend does not cover the first point
    ax.set_xticks(range(1, len(nominal) + 1))
    ax.set_xlabel("Nominal flight, chronological (n = %d)" % len(nominal))
    ax.set_ylabel("Entry-burn Δv (m s$^{-1}$)")
    ax.legend([Line2D([], [], ls="", marker="_", color="k", ms=4, mew=0.9),
               Line2D([], [], ls="", marker="o", color=s.C_SIM, ms=2.8)],
              ["counted at cut-off (logged)", "true, replayed from logged mass and throttle"],
              loc="upper left", fontsize=5, handletextpad=0.3)
    s.sim_tag(ax, "simulation (replay)", loc="upper right")
    # b: Monte Carlo, along error at 10 km vs counter scale
    ax2 = f.add_axes([0.39, 0.17, 0.30, 0.76])
    band = [min(R[k]["log"]["E10_along"] for k in nominal), max(R[k]["log"]["E10_along"] for k in nominal)]
    for rows, mk in ((mild, "o"), (stress, "s")):
        sc = np.array([r["scale"] for r in rows])
        ax2.scatter(sc, [r["counter"]["E10_along"] / 1e3 for r in rows], s=9, marker=mk, facecolor="white",
                    edgecolor="k", lw=0.5, zorder=3)
        ax2.scatter(sc, [r["pcc"]["E10_along"] / 1e3 for r in rows], s=9, marker=mk, color=s.C_SIM, lw=0, zorder=4)
    ax2.axhspan(band[0] / 1e3, band[1] / 1e3, color="0.6", alpha=0.3, lw=0)
    ax2.set_ylim(-6.4, 1.1)
    ax2.set_xlabel("Counter scale factor (counted / true Δv)")
    ax2.set_ylabel("Along-track error at 10 km (km)")
    ax2.legend([Line2D([], [], ls="", marker="o", mfc="white", mec="k", ms=3),
                Line2D([], [], ls="", marker="o", color=s.C_SIM, ms=3),
                Line2D([], [], ls="", marker="s", color="0.5", ms=3)],
               ["counted cut-off", "predictor–corrector cut-off",
                "squares: stress set (n = %d); circles: mild (n = %d)" % (len(stress), len(mild))],
               loc="lower right", fontsize=5, handletextpad=0.3)
    s.sim_tag(ax2, "simulation", loc="upper left")
    # c: hand-off error at burn end, counted vs predictor-corrector targeting -2 km
    ax3 = f.add_axes([0.77, 0.17, 0.22, 0.76])
    a_c = [R[k]["counter"]["A_end"] / 1e3 for k in nominal]
    a_p = [R[k]["pcc_target"]["A_end"] / 1e3 for k in nominal]
    for i in range(len(nominal)):
        ax3.plot([0, 1], [a_c[i], a_p[i]], color="0.8", lw=0.5)
    ax3.plot(np.zeros(len(a_c)), a_c, "o", mfc="white", mec="k", ms=3, mew=0.5)
    ax3.plot(np.ones(len(a_p)), a_p, "o", color=s.C_SIM, ms=3)
    ax3.set_xticks([0, 1])
    ax3.set_xticklabels(["counted\ncut-off", "predictor–corrector,\ntarget −2 km"])
    ax3.set_xlim(-0.5, 1.5)
    ax3.set_ylabel("Along-track error at burn end (km)")
    ax3.text(0, max(a_c) + 0.15, "s.d. %.0f m" % (np.std(a_c, ddof=1) * 1e3), fontsize=5, ha="center")
    ax3.text(1, max(a_p) + 0.15, "s.d. %.0f m" % (np.std(a_p, ddof=1) * 1e3), fontsize=5, ha="center")
    s.sim_tag(ax3, "simulation (replay)", loc="lower right")
    for a_, l, x in ((ax, "a", 0.003), (ax2, "b", 0.335), (ax3, "c", 0.715)):
        s.label(a_, l, fig_coords=(x, 0.99))
    return dict(files=s.save(f, ctx["out"], "fig12_entry_counterfactual"), n_nominal=len(nominal), mild=len(mild),
                stress=len(stress), band=band)


if __name__ == "__main__":
    import _main
    _main.main(make)
