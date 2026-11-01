"""Fig. 13 | The management loop: roles, briefs, review gates, independent checks, standing rules and records.
Counts in the boxes are read from paper/tables/management_counts.csv."""
import os
import sys

from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mgmt_data as m  # noqa: E402
import style as s  # noqa: E402

W, H = 180.0, 96.0


def box(ax, x, y, w, h, title, lines=(), fc="#F4F4F4", ec="0.35", tc="black", fs=5.2, title_fs=5.8):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=1.6", fc=fc, ec=ec, lw=0.6))
    ax.text(x + 1.8, y + h - 1.6, title, fontsize=title_fs, fontweight="bold", va="top", ha="left", color=tc)
    ax.text(x + 1.8, y + h - 5.6, "\n".join(lines), fontsize=fs, va="top", ha="left", linespacing=1.25)


def arrow(ax, p, q, color="0.25", rad=0.0, ls="-", lw=0.7):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=6, lw=lw, color=color, ls=ls,
                                 connectionstyle="arc3,rad=%g" % rad, shrinkA=0, shrinkB=0))


def make(ctx):
    s.setup()
    c = m.counts()
    f = s.fig(s.W2, H)
    ax = f.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.axis("off")
    OI = s.OI
    # row A: three roles
    box(ax, 2, 70, 44, 25, "Human lead (H.H.)",
        ["Sets the goal and the requirements", "Owns each requirement by name", "Approves scope and decisions that",
         "need his authority", "Sets standing safety constraints"], fc="#FBEFD5")
    box(ax, 62, 70, 54, 25, "Orchestrating agent (AI)",
        ["Decomposes, briefs, decides, reports", "Reads the checklist before each decision", "Treats every report as an input",
         "Writes the decision record and lessons", "Applies the five-step method:", "question, delete, simplify, speed up, automate"],
        fc="#DCEBF7")
    box(ax, 132, 70, 46, 25, "Responsible-engineer sub-agents",
        ["%s of %s dispatches build or analyse" % (c["role:Responsible engineer"], c["dispatches_total"]),
         "Brief = goal; known facts marked", "measured or assumed; read/write scope;",
         "deliverable; acceptance criterion", "No peer talk: reports go to the orchestrator"],
        fc="#DDF1EA")
    arrow(ax, (46, 89), (62, 89), color=OI["orange"])
    arrow(ax, (62, 79), (46, 79), color=OI["orange"])
    ax.text(54, 90.5, "direction,\napprovals", fontsize=4.6, ha="center", va="bottom", color="0.3", linespacing=1.1)
    ax.text(54, 77.5, "results, reasons,\nrisks, next step", fontsize=4.6, ha="center", va="top", color="0.3", linespacing=1.1)
    arrow(ax, (116, 89), (132, 89), color=OI["blue"])
    arrow(ax, (132, 79), (116, 79), color=OI["blue"])
    ax.text(124, 90.5, "brief", fontsize=4.6, ha="center", va="bottom", color="0.3")
    ax.text(124, 77.5, "report", fontsize=4.6, ha="center", va="top", color="0.3")

    # row B: review gates
    gx = [2, 27.5, 53, 78.5, 104]
    gt = [("Design review", ["Simulation passes", "the stage-0 test"]),
          ("Readiness review", ["One variable; expected", "result; fail criterion;", "3 relevant lessons"]),
          ("Execution", ["A real run, treated", "as a real launch"]),
          ("Post-flight review", ["Expected vs measured;", "one question answered"]),
          ("Mishap investigation", ["Root cause, fix,", "regression, then", "return to flight"])]
    for x, (t, ls_) in zip(gx, gt):
        box(ax, x, 42, 23, 17, t, ls_, fc="#F4F4F4", title_fs=5.0, fs=4.7)
    for a_, b_ in zip(gx, gx[1:]):
        arrow(ax, (a_ + 23, 50), (b_, 50))
    arrow(ax, (115.5, 42), (13.5, 42), color=OI["vermillion"], rad=-0.12, ls=(0, (3, 2)))
    ax.text(64.5, 35.3, "return to flight after a written decision", fontsize=4.8, ha="center", va="center",
            color=OI["vermillion"], bbox=dict(fc="white", ec="none", pad=0.6))
    arrow(ax, (80, 70), (80, 59), color="0.25")
    ax.text(82, 64.5, "go / no-go", fontsize=4.6, ha="left", va="center", color="0.3")
    ax.text(2, 61, "Review gates", fontsize=5.6, fontweight="bold", va="bottom", color="0.2")

    # right column: independent checks
    box(ax, 132, 33, 46, 30, "Independent checks (read-only dispatches)",
        ["Secretary: %s audits of the" % c["secretary_audits_dispatched"], "orchestrator's own conduct,",
         "answered PASS or SLIP per item",
         "Direction audits: %s" % c["role:Direction audit"],
         "Red teams, reviewers, QA: %s" % c["role:Review, red team, QA"],
         "Andon panel views: %s" % c["role:Andon panel"],
         "Mishap investigators: %s" % c["role:Investigator"],
         "All of the same model family", "as the orchestrator"],
        fc="#F7E3EE")
    arrow(ax, (132, 58), (110, 70), color=OI["purple"])
    ax.text(126, 66.5, "findings", fontsize=4.6, ha="center", va="bottom", color="0.3", rotation=0)

    # standing rules
    box(ax, 2, 12.5, 125, 19, "Standing rules",
        ["Envelope expansion: simplest case first; promote one stage at a time; no skipped stage.",
         "Andon: two failed fix-and-retry cycles, or a vehicle loss, stop the line; a panel of different views;",
         "a written decision with options, weights and exit conditions. Pre-committed exit rules are not re-read after the fact.",
         "Configuration control: a tagged baseline and a regression gate before shared code is changed.",
         "Measurement before control: check frames, units and actuator state before using data to tune."],
        fc="#FFFFFF", fs=4.8)
    # records
    labels = [("Decision record", "%s entries, %s numbered DR records" % (c["decision_log_entries"], c["decision_log_distinct_dr"])),
              ("Lessons file", "%s entries, one rule each" % c["lessons_total"]),
              ("Persistent memory", "preferences, state, rules"),
              ("Checklist (hook)", "injected on every human message")]
    for i, (t, d) in enumerate(labels):
        box(ax, 2 + i * 44.5, 0.5, 42.5, 9.5, t, [d], fc="#EDEDED", fs=4.6, title_fs=5.0)
    ax.text(2, 10.4, "Records: written by the orchestrator, read by later sessions and by the secretary",
            fontsize=4.8, ha="left", va="bottom", color="0.3")
    s.label(ax, "a", fig_coords=(0.003, 0.995))
    return dict(files=s.save(f, ctx["out"], "fig13_management_loop"))


if __name__ == "__main__":
    import _main
    _main.main(make)
