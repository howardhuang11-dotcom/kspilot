"""Fig. 16 | Sub-agent dispatches by workstream and role, and cumulative dispatches over time. Data:
paper/tables/management_dispatches.csv."""
import datetime as dt
import os
import sys
from collections import Counter

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mgmt_data as m  # noqa: E402
import style as s  # noqa: E402

ROLES = ["Responsible engineer", "Review, red team, QA", "Investigator", "Andon panel", "Secretary", "Direction audit"]


def make(ctx):
    s.setup()
    OI = s.OI
    R = m.rows("management_dispatches.csv")
    col = dict(zip(ROLES, [OI["blue"], OI["sky"], OI["orange"], OI["vermillion"], OI["green"], OI["purple"]]))
    ws = Counter(r["workstream"] for r in R)
    order = [w for w, _ in sorted(ws.items(), key=lambda kv: kv[1])]
    f = s.fig(s.W2, 62)
    ax = f.add_axes([0.20, 0.16, 0.30, 0.80])
    left = np.zeros(len(order))
    for ro in ROLES:
        v = np.array([sum(1 for r in R if r["workstream"] == w and r["role"] == ro) for w in order])
        ax.barh(range(len(order)), v, left=left, color=col[ro], edgecolor="white", lw=0.4, height=0.7, label=ro)
        left += v
    for i, t in enumerate(left):
        ax.text(t + 0.5, i, "%d" % t, va="center", fontsize=5.5)
    ax.set_yticks(range(len(order)))
    ax.set_yticklabels(order)
    ax.set_xlabel("Dispatches (n = %d)" % len(R))
    ax.set_xlim(0, max(left) + 5)
    ax.legend(loc="lower right", fontsize=5.0)
    ax2 = f.add_axes([0.62, 0.16, 0.35, 0.80])
    t0 = dt.datetime(2026, 10, 19, 0, 0)
    groups = [("Responsible engineer", ["Responsible engineer"], OI["blue"]),
              ("Review, red team, QA", ["Review, red team, QA"], OI["sky"]),
              ("Investigator and andon panel", ["Investigator", "Andon panel"], OI["orange"]),
              ("Secretary and direction audit", ["Secretary", "Direction audit"], OI["green"])]
    for lab, rs, c in groups:
        ts = sorted(m.when(r, "dispatch_local_acdt") for r in R if r["role"] in rs)
        x = [(t - t0).total_seconds() / 86400 for t in ts]
        ax2.step([0] + x + [3.85], [0] + list(range(1, len(x) + 1)) + [len(x)], where="post", color=c, lw=0.9, label="%s (%d)" % (lab, len(ts)))
    ax2.set_xlim(0, 3.85)
    ax2.set_xticks([0, 1, 2, 3])
    ax2.set_xticklabels(["Day 1", "Day 2", "Day 3", "Day 4"])
    ax2.set_ylabel("Cumulative dispatches")
    ax2.legend(loc="upper left", fontsize=5.0)
    s.label(ax, "a", fig_coords=(0.003, 0.99))
    s.label(ax2, "b", fig_coords=(0.55, 0.99))
    return dict(files=s.save(f, ctx["out"], "fig16_dispatches"))


if __name__ == "__main__":
    import _main
    _main.main(make)
