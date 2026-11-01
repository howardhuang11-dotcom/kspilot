"""Fig. 7 | Terminal-phase regression gate landings (4 km checkpoint): miss in chronological order and
touchdown positions."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import data as d  # noqa: E402
cs = d.cs
import style as s  # noqa: E402


def make(ctx):
    s.setup()
    D = d.load(ctx["logs"])
    G = d.gate_rows(D)
    sessions = []
    for g in G:
        lab = g["env"].get("KSP_REVIEW", "")
        if not sessions or sessions[-1][0] != lab:
            sessions.append([lab, []])
        sessions[-1][1].append(g)
    f = s.fig(s.W2, 62)
    ax = f.add_axes([0.055, 0.17, 0.60, 0.78])
    i = 0
    for k, (lab, rows) in enumerate(sessions):
        is_gate = ("gate" in lab.lower() or "regression" in lab.lower()) and len(rows) >= 3
        x = np.arange(i, i + len(rows)) + 1
        if k % 2 == 0:
            ax.axvspan(x[0] - 0.5, x[-1] + 0.5, color="0.94", lw=0, zorder=0)
        ax.plot(x, [r["result"]["miss"] for r in rows], "s" if is_gate else "o", ms=3,
                color=s.C_GATE if is_gate else s.OI["sky"], mec="white", mew=0.3, zorder=3)
        i += len(rows)
    ax.axhline(cs.GATE_LIMIT_M, color=s.C_FAIL, lw=0.6, ls=(0, (3, 2)))
    ax.text(1, cs.GATE_LIMIT_M + 0.12, "gate limit 5 m", fontsize=5, color=s.C_FAIL)
    ax.set_xlim(0, i + 1)
    ax.set_ylim(0, 7.2)
    ax.set_xlabel("Checkpoint landing, chronological (shading alternates between sessions)")
    ax.set_ylabel("Miss (m)")
    from matplotlib.lines import Line2D
    ax.legend([Line2D([], [], ls="", marker="s", color=s.C_GATE, ms=3),
               Line2D([], [], ls="", marker="o", color=s.OI["sky"], ms=3)],
              ["gate session flight (%d sessions)" % sum(1 for l, r in sessions if ("gate" in l.lower() or "regression" in l.lower()) and len(r) >= 3),
               "single non-gate check"], loc="lower left", fontsize=5)
    hl = [n for n, (lab, rows) in enumerate(sessions)]
    ax2 = f.add_axes([0.73, 0.17, 0.25, 0.78])
    e, n = np.array([(lambda T: (T["e"][-1], T["n"][-1]))(d.telemetry(D["logs"], g["key"])) for g in G]).T
    ax2.scatter(e, n, s=8, color=s.C_GATE, edgecolor="white", lw=0.3)
    for rr in (1, 5):
        ax2.add_patch(__import__("matplotlib").patches.Circle((0, 0), rr, fill=False, lw=0.5, ls=(0, (3, 2)), color="0.5"))
    ax2.plot([0], [0], "+", color="k", ms=5)
    ax2.set_xlim(-6, 2)
    ax2.set_ylim(-4, 4)
    ax2.set_aspect("equal")
    ax2.set_xlabel("East of target (m)")
    ax2.set_ylabel("North of target (m)")
    s.label(ax, "a", fig_coords=(0.003, 0.99))
    s.label(ax2, "b", fig_coords=(0.675, 0.99))
    miss = [g["result"]["miss"] for g in G]
    return dict(files=s.save(f, ctx["out"], "fig7_gates"), n=len(G), median=float(np.median(miss)),
                under5=sum(m < 5 for m in miss), sessions=len(sessions))


if __name__ == "__main__":
    import _main
    _main.main(make)
