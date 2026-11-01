"""Fig. 8 | Robustness across orbits and payloads, and electric power on long coasts.
a-c, miss distance and propellant remaining against inclination, circular-orbit altitude and payload mass for the
     robustness-matrix cases (one flight each), with the baseline-configuration campaign missions for reference.
d,   electric charge consumed during hibernated time-warp segments against segment duration, with the budget rate."""
import math
import os
import sys

import numpy as np
from matplotlib.lines import Line2D

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import data as d  # noqa: E402
cs = d.cs
import style as s  # noqa: E402

BUDGET_EC_S = 0.01      # conservative hibernation rate used by the window budget (Methods)


def factor_rows(D):
    wd_by = {r["wd"]: r for r in D["done"]}
    mx = []
    for m in D["mx"]:
        w = wd_by.get(m["wd"])
        mx.append(dict(case=m["case"], craft=m["craft"], inc=m["inc"], ap=m["ap"], payload=cs.PAYLOAD_T[m["craft"]],
                       result=m["result"], miss=w["miss2"] if w else float("nan"),
                       fuel_t=w["fuel_left"] * cs.LFOX_KG_PER_LF / 1e3 if w else float("nan")))
    base = [r for r in D["done"] if r["result"] == "PASS" and r["craft"] == "T-WorksFineGF"
            and r["target"] == (100.0, 100.0, 6.0) and r["wd"] not in set(m["wd"] for m in D["mx"])]
    return mx, base


def panel(ax_m, ax_f, mx, base, key, xs_base, xlabel, xticks, sel):
    bm = [r["miss2"] for r in base]
    bf = [r["fuel_left"] * cs.LFOX_KG_PER_LF / 1e3 for r in base]
    for ax, vals in ((ax_m, bm), (ax_f, bf)):
        q1, med, q3 = np.percentile(vals, [25, 50, 75])
        ax.plot([xs_base] * 2, [min(vals), max(vals)], color="0.6", lw=0.6, zorder=1)
        ax.plot([xs_base] * 2, [q1, q3], color="0.45", lw=2.2, solid_capstyle="butt", zorder=1)
        ax.plot([xs_base], [med], "_", color="white", ms=3, mew=0.8, zorder=2)
    rows = [m for m in mx if sel(m)]
    for m in rows:
        if m["result"] != "PASS":
            ax_f.annotate("ascent loss on first\nattempt; repeated", (m[key], ax_f.get_ylim()[0] if False else 2.25),
                          fontsize=5, ha="center", va="bottom", color=s.C_FAIL)
            continue
        ax_m.plot(m[key], m["miss"], "D", color=s.C_MATRIX, ms=3.2, mec="white", mew=0.3, zorder=3)
        ax_f.plot(m[key], m["fuel_t"], "D", color=s.C_MATRIX, ms=3.2, mec="white", mew=0.3, zorder=3)
    for ax in (ax_m, ax_f):
        ax.set_xticks(xticks)
    ax_m.tick_params(labelbottom=False)
    ax_f.set_xlabel(xlabel)
    ax_m.set_ylim(0, 3)
    ax_f.set_ylim(2.0, 4.6)


def make(ctx):
    s.setup()
    D = d.load(ctx["logs"])
    mx, base = factor_rows(D)
    f = s.fig(s.W2, 72)
    L, B, H, G = 0.055, 0.15, 0.37, 0.04
    widths = [0.20, 0.14, 0.14]
    xs = [L, L + 0.245, L + 0.43]
    axes = []
    for x, w in zip(xs, widths):
        ax_f = f.add_axes([x, B, w, H])
        ax_m = f.add_axes([x, B + H + G, w, H], sharex=ax_f)
        axes.append((ax_m, ax_f))
    panel(*axes[0], mx, base, "inc", 6, "Inclination (°)", [0, 6, 28, 45, 90],
          lambda m: m["ap"] == 100 and m["payload"] == 2.48 and m["inc"] != 6)
    axes[0][0].set_xlim(-6, 96)
    panel(*axes[1], mx, base, "ap", 100, "Circular orbit (km)", [80, 100, 110],
          lambda m: m["ap"] != 100)
    axes[1][0].set_xlim(74, 116)
    panel(*axes[2], mx, base, "payload", 2.48, "Payload (t)", [2.48, 4.5, 6.0],
          lambda m: m["payload"] != 2.48)
    axes[2][0].set_xlim(1.8, 6.7)
    axes[2][1].set_xticklabels(["2.48", "4.50", "6.00"])
    axes[0][0].set_ylabel("Miss (m)")
    axes[0][1].set_ylabel("Propellant\nremaining (t)")
    for i in (1, 2):
        axes[i][0].tick_params(labelleft=False)
        axes[i][1].tick_params(labelleft=False)
    hand = [Line2D([], [], ls="", marker="D", color=s.C_MATRIX, ms=3.2),
            Line2D([], [], color="0.45", lw=2.2)]
    axes[0][0].legend(hand, ["matrix case (one flight each)",
                             "baseline missions, IQR and range (n = %d)" % len(base)],
                      loc="lower left", bbox_to_anchor=(0.0, 0.0), fontsize=5, handletextpad=0.3, labelspacing=0.2)
    # ---- d: electric charge during hibernation
    ax_d = f.add_axes([0.69, B, 0.295, 2 * H + G])
    segs = [(t, e) for r in D["done"] for t, e in r["hib"]]
    t = np.array([x[0] for x in segs]) / 3600.0
    e = np.array([x[1] for x in segs])
    e_plot = e
    assert (e > 0).all(), "zero-consumption segment would vanish on the log axis"
    ax_d.scatter(t, e_plot, s=8, color=s.OI["green"], edgecolor="white", lw=0.3, zorder=3,
                 label="hibernated segments (n = %d, %d missions)" % (len(segs), sum(1 for r in D["done"] if r["hib"])))
    tt = np.array([0.05, 6.0])
    ax_d.plot(tt, BUDGET_EC_S * tt * 3600, color="0.25", lw=0.7, ls=(0, (4, 2)),
              label="budget rate, %.2f units s$^{-1}$" % BUDGET_EC_S)
    rate = e.sum() / (t.sum() * 3600)
    ax_d.plot(tt, rate * tt * 3600, color=s.OI["green"], lw=0.7, label="pooled observed rate, %.1f $\\times$ 10$^{%d}$ units s$^{-1}$" % (
        rate / 10 ** math.floor(math.log10(rate)), math.floor(math.log10(rate))))
    ax_d.set_xscale("log")
    ax_d.set_yscale("log")
    ax_d.set_xlim(0.05, 6)   # r2: longest hibernated segment 4.3 h (deorbit-window coasts now counted)
    ax_d.set_ylim(0.03, 200)
    ax_d.set_xlabel("Hibernated segment duration (h)")
    ax_d.set_ylabel("Electric charge consumed (units)")
    ax_d.legend(loc="upper left", fontsize=5, handletextpad=0.3)
    ax_d.text(5.8, 0.035, "charge read to 0.1 units; core battery 1,000 units", fontsize=5, ha="right", color="0.4")
    s.label(axes[0][0], "a", fig_coords=(0.003, 0.99))
    s.label(axes[1][0], "b", fig_coords=(xs[1] - 0.02, 0.99))
    s.label(axes[2][0], "c", fig_coords=(xs[2] - 0.02, 0.99))
    s.label(ax_d, "d", fig_coords=(0.635, 0.99))
    return dict(files=s.save(f, ctx["out"], "fig8_robustness_power"), n_base=len(base), segs=len(segs),
                hib_s=float(t.sum() * 3600), ec=float(e.sum()), rate=rate)


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import make_all
    make_all.run_one(make)
