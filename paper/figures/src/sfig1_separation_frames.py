"""Supplementary Fig. 1 | Booster separation frames from the verification tests: about 19 kPa (four explosions) and
about 9.6 kPa (none). Screenshots taken by the separation test before and about 2 s after the staging event."""
import os
import re
import sys


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import data as d  # noqa: E402
import style as s  # noqa: E402

# (before, after) screenshot pairs and the separation test they belong to (by time stamp, 2026-10-21)
PAIRS = [("20261021_061133_ascent_stage3_before.png", "20261021_061135_ascent_stage3_after.png", "wd_20261021_061000"),
         ("20261021_061328_ascent_stage3_before.png", "20261021_061330_ascent_stage3_after.png", "wd_20261021_061200")]
CROP = (480, 330, 1440, 870)   # 960 x 540 px around the vehicle
GAMMA = 0.7                     # uniform brightness lift applied to every frame (stated in the legend)


def make(ctx):
    s.setup()
    D = d.load(ctx["logs"])
    sep = sorted(D["sep"], key=lambda r: r["start"])
    f = s.fig(s.W2, 108)
    W, H = 0.485, 0.445
    for row, (b, a, _) in enumerate(PAIRS):
        # the test whose start precedes the screenshot by less than 3 min
        ts = d.cs.ts(b[:15])
        test = max((r for r in sep if r["start"] <= ts), key=lambda r: r["start"])
        for col, fn in enumerate((b, a)):
            ax = f.add_axes([0.012 + col * (W + 0.02), 0.51 - row * (H + 0.04), W, H])
            d.show_screenshot(ax, ctx["screens"], fn, CROP,
                              post=lambda im: im.point(lambda v: int(255 * (v / 255.0) ** GAMMA)))
            ax.text(0.015, 0.97, "%s separation, %s" % ("before" if col == 0 else "about 2 s after",
                                                       fn[9:15][:2] + ":" + fn[9:15][2:4] + ":" + fn[9:15][4:]),
                    transform=ax.transAxes, color="white", fontsize=5.5, va="top")
            if col == 1:
                n15 = int(re.search(r"explosions_15s=(\d+)", d.transcript(D["logs"], test["wd"])).group(1))
                ax.text(0.015, 0.03, "q = %.1f kPa; explosions within 15 s: %d; core parts hit: %d" % (
                    test["sep_q"], n15, int(test["sep_core_hit"])), transform=ax.transAxes, color="white",
                    fontsize=5.5, va="bottom")
            if col == 0:
                s.label(ax, "ab"[row], fig_coords=(0.0, 0.51 - row * (H + 0.04) + H + 0.035))
    return dict(files=s.save(f, ctx["out"], "sfig1_separation_frames"))


if __name__ == "__main__":
    import _main
    _main.main(make)
