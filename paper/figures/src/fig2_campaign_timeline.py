"""Fig. 2 | Timeline of the development flights: every supervisor transcript before the data freeze, by start state,
outcome and vehicle. Data: paper/tables/flow_all_transcripts.csv (committed; the table is derived from data/raw)."""
import csv
import datetime as dt
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import style as s  # noqa: E402

TABLES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tables")
ROWS = ["pad", "orbit checkpoint", "pre-entry checkpoint", "4 km checkpoint", "15 km checkpoint"]
ROWLAB = ["Pad", "Orbit checkpoint", "Pre-entry checkpoint", "4 km checkpoint", "15 km checkpoint"]
MARK = {"T-WorksFineGF family": "o", "T-WorksFinePayload": "s", "GF variants without airbrakes": "^"}


def make(ctx):
    s.setup()
    OI = s.OI
    R = list(csv.DictReader(open(os.path.join(TABLES, "flow_all_transcripts.csv"), newline="")))
    t0 = dt.datetime(2026, 10, 19, 0, 0)
    f = s.fig(s.W2, 68)
    ax = f.add_axes([0.14, 0.17, 0.80, 0.78])
    x_win = (dt.datetime(2026, 10, 21, 1, 10) - t0).total_seconds() / 86400
    ax.axvspan(x_win, 3.0, color="0.94", lw=0, zorder=0)
    ax.axvline(x_win, color="0.4", lw=0.5, ls=(0, (3, 2)))
    for i, r in enumerate(R):
        t = dt.datetime.strptime(r["Start (local)"], "%Y-%m-%d %H:%M:%S")
        x = (t - t0).total_seconds() / 86400
        y = ROWS.index(r["Starting point"]) + ((i * 7) % 5 - 2) * 0.07
        out = r["Outcome"]
        col = OI["blue"] if out == "PASS" else OI["vermillion"] if out == "FAIL" else OI["grey"]
        fill = out in ("PASS", "FAIL")
        mk = MARK[r["Family"]]
        ax.plot([x], [y], mk, ms=2.8, mfc=col if fill else "none", mec=col, mew=0.5, alpha=0.9, zorder=3)
    # annotated events (times from Table 6 of the report and the campaign table)
    def xd(day, hh, mm):
        return (dt.datetime(2026, 10, day, hh, mm) - t0).total_seconds() / 86400
    ev = [(xd(20, 21, 4), "stop after nine failed\nfins-only flights", -0.85, "right"),
          (xd(21, 0, 5), "line stop", -0.60, "right"),
          (x_win, "window opens", -0.85, "left"),
          (xd(21, 5, 46), "mission 12", -0.60, "left"),
          (xd(21, 13, 53), "mission 21", -0.60, "left")]
    for x, txt, y, ha in ev:
        ax.plot([x, x], [y + 0.12, -0.2], color="0.3", lw=0.4)
        ax.text(x + (0.015 if ha == "left" else -0.015), y, txt, fontsize=5.0, ha=ha, va="center", color="0.15")
    ax.set_xlim(0.15, 3.0)
    ax.set_ylim(len(ROWS) - 0.4, -1.1)
    ax.set_yticks(range(len(ROWS)))
    ax.set_yticklabels(ROWLAB)
    ax.set_xticks([1, 2, 3])
    ax.set_xticklabels(["Day 2", "Day 3", "Day 4\n(data freeze)"])
    ax.set_xticks([0.5, 1.5, 2.5], minor=True)
    ax.set_xlabel("Start of the supervisor transcript (project day)")
    from matplotlib.lines import Line2D
    h = [Line2D([], [], ls="", marker="o", color=OI["blue"], ms=3), Line2D([], [], ls="", marker="o", color=OI["vermillion"], ms=3),
         Line2D([], [], ls="", marker="o", mfc="none", mec=OI["grey"], ms=3),
         Line2D([], [], ls="", marker="o", color="0.3", ms=3), Line2D([], [], ls="", marker="s", color="0.3", ms=3),
         Line2D([], [], ls="", marker="^", color="0.3", ms=3)]
    ax.legend(h, ["landed", "failed", "no outcome, refused or separation test", "T-WorksFineGF family",
                  "earlier vehicle", "variants without airbrakes"], loc="lower left", ncol=3, fontsize=5.0,
              bbox_to_anchor=(0.0, -0.01), columnspacing=1.0)
    ax.text(0.99, 0.02, "n = %d transcripts" % len(R), transform=ax.transAxes, ha="right", fontsize=5.2)
    return dict(files=s.save(f, ctx["out"], "fig2_campaign_timeline"))


if __name__ == "__main__":
    import _main
    _main.main(make)
