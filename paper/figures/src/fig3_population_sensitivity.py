"""Fig. 3 | Landing proportion of the exploratory missions under ten population definitions (Wilson and
Clopper-Pearson 95% intervals; nominal). Data: paper/tables/sensitivity.csv (committed; derived from data/raw)."""
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import style as s  # noqa: E402

TABLES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tables")
LABELS = ["Window, complete missions", "Return phase, conditional on reaching orbit",
          "Window, terminated run counted as a failure", "Window, also the two attempts that never launched",
          "Window extended to the two preceding missions", "All complete own-code-ascent pad missions",
          "The same, terminated run counted as a failure", "Baseline configuration only",
          "Without a recording workload", "With a recording workload"]


def _ci(txt):
    a, b = txt.replace("−", "-").split("-")[0:2] if txt.count("-") == 1 else (None, None)
    return float(a), float(b)


def make(ctx):
    s.setup()
    OI = s.OI
    R = list(csv.DictReader(open(os.path.join(TABLES, "sensitivity.csv"), newline="")))[:10]
    f = s.fig(s.W2, 62)
    ax = f.add_axes([0.37, 0.15, 0.55, 0.80])
    for i, (r, lab) in enumerate(zip(R, LABELS)):
        k, n = [int(v) for v in r["Successes / missions"].split("/")]
        w = _ci(r["Wilson 95% CI (%)"])
        c = _ci(r["Clopper-Pearson 95% CI (%)"])
        hl = i == 0
        col = OI["blue"] if hl else "0.35"
        ax.plot(c, [i, i], color=col, lw=0.6, solid_capstyle="butt", alpha=0.6)
        ax.plot(w, [i, i], color=col, lw=2.6, solid_capstyle="butt")
        ax.plot([float(r["Proportion (%)"])], [i], "o", ms=3.2, mfc="white", mec="k", mew=0.6, zorder=4)
        ax.text(101.5, i, "%d / %d" % (k, n), va="center", fontsize=5.5, clip_on=False)
    ax.set_yticks(range(len(R)))
    ax.set_yticklabels(LABELS)
    ax.set_ylim(len(R) - 0.4, -0.6)
    ax.set_xlim(40, 100)
    ax.set_xlabel("Landed within the criterion (%)")
    ax.spines["left"].set_visible(False)
    ax.tick_params(axis="y", length=0)
    ax.text(0.0, 1.0, "thick bar, Wilson 95%; thin line, Clopper-Pearson 95%; all nominal", transform=ax.transAxes,
            fontsize=5.0, va="bottom", color="0.3")
    return dict(files=s.save(f, ctx["out"], "fig3_population_sensitivity"))


if __name__ == "__main__":
    import _main
    _main.main(make)
