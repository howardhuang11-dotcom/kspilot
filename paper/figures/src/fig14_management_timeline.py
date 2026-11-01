"""Fig. 14 | Timeline of the management record, over the project: decision-record entries,
gates, andons, mishap investigations, secretary and direction audits, exit rules, with sub-agent
dispatches as a rug. Data: paper/tables/management_{decision_log,events,dispatches}.csv."""
import datetime as dt
import os
import sys

import matplotlib.dates as md

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mgmt_data as m  # noqa: E402
import style as s  # noqa: E402

T0, T1 = dt.datetime(2026, 10, 20, 12, 0), dt.datetime(2026, 10, 22, 21, 0)


def make(ctx):
    s.setup()
    OI = s.OI
    dec, ev, dis = m.rows("management_decision_log.csv"), m.rows("management_events.csv"), m.rows("management_dispatches.csv")
    f = s.fig(s.W2, 78)
    ax = f.add_axes([0.17, 0.12, 0.81, 0.85])
    lanes = ["Sub-agent dispatches", "Decision-record entries", "Gates (listed)", "Andons (line stops)",
             "Mishap investigations", "Secretary audits", "Direction audits", "Exit rule"]
    Y = {n: len(lanes) - 1 - i for i, n in enumerate(lanes)}
    # day shading
    d = dt.datetime(2026, 10, 20, 0, 0)
    for k in range(4):
        a, b = d + dt.timedelta(days=k), d + dt.timedelta(days=k + 1)
        if k % 2 == 1:
            ax.axvspan(max(a, T0), min(b, T1), color="0.95", lw=0, zorder=0)
    for n in lanes:
        ax.axhline(Y[n], color="0.85", lw=0.3, zorder=1)
    for r in dis:
        t = m.when(r, "dispatch_local_acdt")
        if T0 <= t <= T1:
            ax.plot([t, t], [Y[lanes[0]] - 0.25, Y[lanes[0]] + 0.25], color=OI["grey"], lw=0.5, zorder=2)
    for r in dec:
        t = m.when(r, "time_local_acdt")
        h = 0.38 if r["numbered_dr"] == "1" else 0.22
        ax.plot([t, t], [Y[lanes[1]] - h, Y[lanes[1]] + h], color=OI["blue"] if r["numbered_dr"] == "1" else OI["sky"], lw=0.8, zorder=2)
    spec = {"gate": (lanes[2], "s", OI["grey"], 3.0), "andon": (lanes[3], "v", OI["vermillion"], 4.2),
            "mishap": (lanes[4], "D", OI["orange"], 3.2), "secretary": (lanes[5], "o", OI["green"], 3.6),
            "audit": (lanes[6], "P", OI["purple"], 4.0), "exit_rule": (lanes[7], "*", OI["black"], 5.0)}
    for r in ev:
        t = m.when(r, "time_local_acdt")
        if not (T0 <= t <= T1):
            continue
        lane, mk, col, ms = spec[r["kind"]]
        ax.plot([t], [Y[lane]], mk, color=col, ms=ms, mec="white" if mk not in "|" else col, mew=0.3, zorder=3)
    notes = [("2026-10-21 05:50", "andon", "vehicle lost at\nseparation", 0.42), ("2026-10-21 14:35", "andon", "landed 3.8 km\nshort", 0.42),
             ("2026-10-22 02:28", "andon", "divert plant\nrefuted", 0.42), ("2026-10-22 03:02", "andon", "", 0),
             ("2026-10-21 00:05", "andon", "2nd failure,\nverification", 0.42), ("2026-10-20 21:04", "andon", "9 failed\nflights", 0.42)]
    for t, k, txt, dy in notes:
        if txt:
            tt = dt.datetime.strptime(t, m.FMT)
            ax.text(tt, Y[lanes[3]] + dy, txt, fontsize=4.4, ha="center", va="bottom", color=OI["vermillion"], linespacing=1.0)
    ax.annotate("O4 chosen,\nthen O1 fallback", xy=(dt.datetime(2026, 10, 22, 3, 15), Y[lanes[7]]), xytext=(dt.datetime(2026, 10, 21, 20, 0), Y[lanes[7]]),
                fontsize=4.4, ha="right", va="center", arrowprops=dict(arrowstyle="-", lw=0.4, color="0.4"))
    ax.set_yticks([Y[n] for n in lanes])
    ax.set_yticklabels(lanes)
    ax.set_ylim(-0.7, len(lanes) - 0.1)
    ax.set_xlim(T0, T1)
    ax.xaxis.set_major_locator(md.HourLocator(byhour=[0, 12]))
    ax.xaxis.set_major_locator(md.DayLocator())
    ax.xaxis.set_major_formatter(__import__("matplotlib.ticker",fromlist=["x"]).FuncFormatter(lambda v,p: "Day %d" % (md.num2date(v).day - 18)))
    ax.xaxis.set_minor_locator(md.HourLocator(byhour=range(0, 24, 3)))
    ax.set_xlabel("Project day")
    ax.spines["left"].set_visible(False)
    ax.tick_params(axis="y", length=0)
    return dict(files=s.save(f, ctx["out"], "fig14_management_timeline"))


if __name__ == "__main__":
    import _main
    _main.main(make)
