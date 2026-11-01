"""Fig. 15 | Lessons by category and by who or what exposed the problem (single-coder). Data:
paper/tables/management_lessons.csv."""
import os
import sys
from collections import Counter

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mgmt_data as m  # noqa: E402
import style as s  # noqa: E402

FINDERS = ["Flight or test outcome", "Orchestrator inspection", "Sub-agent or audit", "Human lead (H.H.)"]


def make(ctx):
    s.setup()
    OI = s.OI
    R = m.rows("management_lessons.csv")
    cats = Counter(r["category"] for r in R)
    order = [c for c, _ in sorted(cats.items(), key=lambda kv: kv[1])]
    col = {FINDERS[0]: OI["lightgrey"], FINDERS[1]: OI["sky"], FINDERS[2]: OI["green"], FINDERS[3]: OI["orange"]}
    f = s.fig(s.W2, 56)
    ax = f.add_axes([0.25, 0.17, 0.30, 0.78])
    left = np.zeros(len(order))
    for fd in FINDERS:
        v = np.array([sum(1 for r in R if r["category"] == c and r["finder"] == fd) for c in order])
        ax.barh(range(len(order)), v, left=left, color=col[fd], edgecolor="white", lw=0.4, height=0.7, label=fd)
        left += v
    for i, t in enumerate(left):
        ax.text(t + 0.4, i, "%d" % t, va="center", fontsize=5.5)
    ax.set_yticks(range(len(order)))
    ax.set_yticklabels(order)
    ax.set_xlabel("Lessons (n = %d)" % len(R))
    ax.set_xlim(0, max(left) + 3)
    ax.legend(loc="lower right", fontsize=5.2)
    ax2 = f.add_axes([0.68, 0.17, 0.29, 0.78])
    tot = Counter(r["finder"] for r in R)
    ax2.barh(range(4), [tot[x] for x in FINDERS], color=[col[x] for x in FINDERS], height=0.7)
    for i, x in enumerate(FINDERS):
        ax2.text(tot[x] + 1, i, "%d" % tot[x], va="center", fontsize=5.5)
    ax2.set_yticks(range(4))
    ax2.set_yticklabels(FINDERS)
    ax2.invert_yaxis()
    ax2.set_xlim(0, max(tot.values()) + 8)
    ax2.set_xlabel("Lessons by first finder")
    s.label(ax, "a", fig_coords=(0.003, 0.99))
    s.label(ax2, "b", fig_coords=(0.60, 0.99))
    return dict(files=s.save(f, ctx["out"], "fig15_lessons"))


if __name__ == "__main__":
    import _main
    _main.main(make)
