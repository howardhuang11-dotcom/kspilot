"""Fig. 10 | Landing-burn divert. Panel a (simulation): convex-planner envelope against ignition height (GNC2
divert_optimal.json). Panels b and c (flight data): the measured flight record of landing burns lit with a lateral
offset, read from the raw logs (experiments.jsonl, *_landing.csv) through paper/analysis/divert_flight_record.py, so
the figure and paper/tables/divert_flight_record.* cannot diverge. They replace the earlier acceptance Monte Carlo on
the identified divert plant (divert_measured_mc.json), which was superseded because flight test refuted that plant
(divert flight 1 lit at 832 m against <= 492 m in the plant and landed 1.8 m from the pad against a plant median of
8.7-20.1 m)."""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "analysis"))
import style as s  # noqa: E402
import divert_flight_record as dfr  # noqa: E402

C_FLOWN, C_E = s.OI["blue"], s.OI["orange"]
C_FLIGHT = "0.15"          # flight-record tag (simulation panels carry the vermillion s.sim_tag)
CRIT = 5.0                 # m, landing criterion


def flight_tag(ax, text="flight record", loc="upper right"):
    """Visible in-panel marker for flight-record content (counterpart of s.sim_tag)."""
    x, ha = (0.98, "right") if "right" in loc else (0.02, "left")
    y, va = (0.98, "top") if "upper" in loc else (0.02, "bottom")
    ax.text(x, y, text, transform=ax.transAxes, ha=ha, va=va, fontsize=5.5, fontstyle="italic", color=C_FLIGHT,
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec=C_FLIGHT, lw=0.4), zorder=10)


def make(ctx):
    s.setup()
    G = ctx["gnc2"]
    OP = json.load(open(os.path.join(G, "divert_optimal.json")))["optimal"]
    f = s.fig(s.W2, 60)
    # ---------------------------------------------------------------- a: convex-planner envelope (unchanged)
    ax = f.add_axes([0.06, 0.17, 0.25, 0.76])
    h = [o["h"] for o in OP]
    ax.plot(h, [o["d_max"] for o in OP], "o-", color=s.C_SIM, ms=3, lw=0.8, label="maximum divert offset")
    ax.set_xlabel("Landing-burn ignition height (m)")
    ax.set_ylabel("Maximum nullable offset (m)")
    ax.set_ylim(0, 1000)
    ax2 = ax.twinx()
    for off, ls in (("0.0", "-"), ("150.0", "--"), ("300.0", ":")):
        ax2.plot(h, [o["fuel"][off]["fuel"] if o["fuel"][off]["exact"] else np.nan for o in OP], color="0.45", lw=0.6,
                 ls=ls, label="propellant, %s m offset" % off[:-2])
    ax2.set_ylim(700, 2200)
    ax2.set_ylabel("Propellant (kg, grey)", color="0.45")
    ax2.spines["right"].set_visible(True)
    ax2.tick_params(colors="0.45")
    ax2.legend(loc="center right", fontsize=5)
    ax.legend(loc="upper left", fontsize=5)
    s.sim_tag(ax, "simulation (convex planner)", loc="lower right")

    # ---------------------------------------------------------------- b: touchdown offset vs offset at ignition
    recs = dfr.records(ctx["logs"])
    big = [r for r in recs if r["group"] == dfr.G_LARGE]
    gate = [r for r in recs if r["group"] == dfr.G_GATE]
    ser = {r["label"]: r for r in recs if r["group"] == dfr.G_SERIES}
    axb = f.add_axes([0.415, 0.17, 0.24, 0.76])
    lim = (0.7, 1500)
    axb.plot(lim, lim, color="0.55", lw=0.5, ls=(0, (4, 2)), zorder=1)
    axb.text(2.6, 2.6 * 1.45, "no closure (y = x)", rotation=45, rotation_mode="anchor", fontsize=5, color="0.4",
             ha="left", va="bottom")
    axb.axhline(CRIT, color=s.C_FAIL, lw=0.5, ls=(0, (3, 2)), zorder=1)
    axb.text(lim[1] * 0.9, CRIT * 0.84, "5 m", color=s.C_FAIL, fontsize=5, ha="right", va="top")

    def settle(rs, c):     # post-settle miss: short tick joined to the touchdown value
        for r in rs:
            x, y0, y1 = r["offset_ign_m"], r["offset_td_m"], r["miss_post_settle_m"]
            axb.plot([x, x], [y0, y1], color=c, lw=0.4, zorder=2)
            axb.plot([x], [y1], marker="_", ms=4, mew=0.7, color=c, zorder=2, ls="none")

    axb.scatter([r["offset_ign_m"] for r in gate], [r["offset_td_m"] for r in gate], s=3, color=s.C_GATE, lw=0,
                alpha=0.8, zorder=3, label="4 km gate, one saved state, n = %d" % len(gate))
    settle(big, "0.2")
    axb.scatter([r["offset_ign_m"] for r in big], [r["offset_td_m"] for r in big], s=9, color="black", lw=0,
                zorder=4, label="flown law, offset \u2265 100 m (n = %d);\nnumbers: closure (m)" % len(big))
    # closure (m) next to each large-offset burn; offsets in points chosen so the labels clear each other
    # label positions: offset points from the marker, or data coordinates (with a leader) for the 416-481 m cluster
    place = {"L1": (-4, 6), "L2": (-4, -5), "L3": (5, 0), "L6": (5, -3), "L7": (5, -5)}
    lead = {"L4": (230, 900), "L5": (230, 600), "L8": (900, 190)}
    for r in big:
        xy, txt = (r["offset_ign_m"], r["offset_td_m"]), dfr.cs.rnd(r["closure_td_m"], 0)
        if r["label"] in lead:
            axb.annotate(txt, xy, xytext=lead[r["label"]], textcoords="data", fontsize=4.5, ha="center",
                         va="center", color="0.15",
                         arrowprops=dict(arrowstyle="-", lw=0.3, color="0.5", shrinkA=2, shrinkB=1.5))
        else:
            dx, dy = place[r["label"]]
            axb.annotate(txt, xy, xytext=(dx, dy), textcoords="offset points", fontsize=4.5,
                         ha="right" if dx < 0 else "left", va="center", color="0.15")
    f1, f0 = ser["flight 1"], ser["flight 0 (no offset)"]
    settle([f0, f1], C_FLOWN)
    axb.scatter([f1["offset_ign_m"]], [f1["offset_td_m"]], s=14, color=C_FLOWN, lw=0, zorder=6,
                label="divert flight 1, flown law")
    axb.scatter([f0["offset_ign_m"]], [f0["offset_td_m"]], s=9, facecolor="white", edgecolor=C_FLOWN, lw=0.6,
                zorder=6, label="divert flight 0, no offset")
    es = [ser["E-50 #1"], ser["E-50 #2"]]
    settle(es, C_E)
    for r, mk in zip(es, ("D", "s")):
        axb.scatter([r["offset_ign_m"]], [r["offset_td_m"]], s=12, marker=mk, facecolor="white", edgecolor=C_E,
                    lw=0.7, zorder=7)
    axb.scatter([], [], s=12, marker="D", facecolor="white", edgecolor=C_E, lw=0.7,
                label="law E (default off, not frozen)")
    for r, txt, dy in ((es[0], "#1", 0), (es[1], "#2", 3)):
        axb.annotate(txt, (r["offset_ign_m"], r["offset_td_m"]), xytext=(5, dy), textcoords="offset points",
                     fontsize=4.5, color=C_E, va="center")
    axb.annotate("1", (f1["offset_ign_m"], f1["offset_td_m"]), xytext=(5, -3), textcoords="offset points",
                 fontsize=4.5, color=C_FLOWN, va="center")
    axb.set_xscale("log")
    axb.set_yscale("log")
    axb.set_xlim(*lim)
    axb.set_ylim(*lim)
    axb.set_xlabel("Offset at landing-burn ignition (m)")
    axb.set_ylabel("Offset at touchdown (m)")
    axb.plot([], [], marker="_", ms=4, mew=0.7, color="0.2", ls="none", label="post-settle miss")
    axb.legend(loc="upper left", fontsize=4.8, handletextpad=0.3, borderaxespad=0.2, labelspacing=0.3,
               bbox_to_anchor=(0.0, 0.93))
    flight_tag(axb, "flight record", loc="upper left")

    # ---------------------------------------------------------------- c: offset against height in the burn
    axc = f.add_axes([0.745, 0.17, 0.245, 0.76])
    HMIN = 0.5             # heights below 0.5 m (the last half-metre before contact) are drawn at 0.5 m
    DMIN = 0.2             # offsets below 0.2 m (e, n are logged to 0.1 m) are drawn at 0.2 m
    sty = {"flight 1": dict(color=C_FLOWN, ls="-"), "E-50 #1": dict(color=C_E, ls="-"),
           "E-50 #2": dict(color=C_E, ls=(0, (3, 1.2)))}
    for tr in dfr.tracks(ctx["logs"]):
        hh = np.maximum(tr["h"], HMIN)
        tr["d"] = np.maximum(tr["d"], DMIN)
        axc.plot(hh, tr["d"], lw=0.8, label=tr["label"] + (" (law E)" if tr["law"] == "E" else " (flown law)"),
                 zorder=4, **sty[tr["label"]])
        axc.plot(hh[-1], tr["d"][-1], marker="o", ms=2.2, color=sty[tr["label"]]["color"], zorder=5)
        i = tr["handover"]
        if i is not None:
            axc.plot(hh[i], tr["d"][i], marker="v", ms=3.5, mfc="white", mec="black", mew=0.6, zorder=6, ls="none")
            first = tr["label"] == "E-50 #1"
            axc.annotate("hand-over\n%s m" % dfr.cs.rnd(tr["h"][i], 0), (hh[i], tr["d"][i]),
                         xytext=(350, 0.3) if first else (9, 0.42), textcoords="data", fontsize=4.5,
                         ha="center", va="center",
                         arrowprops=dict(arrowstyle="-", lw=0.3, color="0.4", shrinkA=1, shrinkB=2))
    axc.axvline(dfr.COMMIT_ABOVE_PAD, color="0.5", lw=0.5, ls=":", zorder=1)
    axc.text(dfr.COMMIT_ABOVE_PAD * 1.08, 80, "commit\n%d m" % dfr.COMMIT_ABOVE_PAD, fontsize=4.5, color="0.35",
             ha="right", va="top")
    axc.axhline(CRIT, color=s.C_FAIL, lw=0.5, ls=(0, (3, 2)), zorder=1)
    axc.text(1000, CRIT * 1.1, "5 m", color=s.C_FAIL, fontsize=5, ha="left", va="bottom")
    axc.set_xscale("log")
    axc.set_yscale("log")
    axc.set_xlim(1200, 0.4)
    axc.set_ylim(0.15, 100)
    axc.set_xlabel("Height above the pad in the landing burn (m)")
    axc.set_ylabel("Horizontal offset from the pad (m)")
    axc.plot([], [], marker="v", ms=3.5, mfc="white", mec="black", mew=0.6, ls="none", label="law E hand-over")
    axc.legend(loc="upper right", fontsize=4.8, handletextpad=0.4, borderaxespad=0.2, labelspacing=0.3)
    flight_tag(axc, "flight record", loc="upper left")
    for a_, l, x in ((ax, "a", 0.003), (axb, "b", 0.36), (axc, "c", 0.69)):
        s.label(a_, l, fig_coords=(x, 0.99))
    return dict(files=s.save(f, ctx["out"], "fig10_divert"), n_flights=len(recs))


if __name__ == "__main__":
    import _main
    _main.main(make)
