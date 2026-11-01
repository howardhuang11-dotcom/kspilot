"""Supplementary Fig. 3 | Offset-aim landing (docking variant, first flight after the data freeze): landing on flat ground
2.0 km short of the pad (transcript wd_20261022_001621.out, its landing CSV and in-game screenshot)."""
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import data as d  # noqa: E402
import style as s  # noqa: E402

WD = "wd_20261022_001621.out"
SHOT = ("20261022_003434_orbit_T-WorksFineGF-DOCK.png", (480, 150, 1440, 900))


def make(ctx):
    s.setup()
    D = d.load(ctx["logs"])
    r = next(x for x in D["post"] if x["wd"] == WD)
    txt = d.transcript(ctx["logs"], WD)
    m = re.search(r"\(along (-?\d+) m, cross ([+-]?\d+) m of the pad", txt)
    along = float(m.group(1))
    res = re.search(r"RESULT PASS miss=([\d.]+)m .*pad_dist=(\d+)m", txt)
    T = d.telemetry(ctx["logs"], r["key"])
    e = np.array(T["e"]) + along          # CSV is target-relative; target lies `along` m east of the pad (approach east)
    n = np.array(T["n"])
    h = np.array(T["h"])
    f = s.fig(s.W2, 62)
    ax = f.add_axes([0.005, 0.03, 0.40, 0.94])
    d.show_screenshot(ax, ctx["screens"], SHOT[0], SHOT[1])
    ax.text(0.02, 0.97, "landed at the offset site (night); pad lights on the horizon at right", transform=ax.transAxes,
            color="white", fontsize=5.2, va="top")
    axb = f.add_axes([0.47, 0.17, 0.30, 0.76])
    sel = (e > -40000) & (h < 26000)
    axb.plot(e[sel] / 1e3, h[sel] / 1e3, color=s.C_DOCK, lw=1.0)
    axb.plot([0], [0], marker="*", ms=7, color="k", clip_on=False, zorder=4)
    axb.text(0, 1.0, "pad", fontsize=5, ha="center", va="bottom")
    axb.plot([along / 1e3], [0], marker="X", ms=5, color=s.C_DOCK, clip_on=False, zorder=4)
    axb.text(along / 1e3, 1.0, "offset site", fontsize=5, ha="right", va="bottom", color=s.C_DOCK)
    axb.set_xlim(-40, 1)
    axb.set_ylim(0, 26)
    axb.set_xlabel("East of pad (km)")
    axb.set_ylabel("Altitude (km)")
    axc = f.add_axes([0.85, 0.17, 0.14, 0.76])
    axc.plot(T["e"][-1], T["n"][-1], "^", color=s.C_DOCK, ms=4)
    for rr in (1, 5):
        axc.add_patch(__import__("matplotlib").patches.Circle((0, 0), rr, fill=False, lw=0.5, ls=(0, (3, 2)), color="0.5"))
    axc.plot([0], [0], "+", color="k", ms=5)
    axc.set_xlim(-5.5, 5.5)
    axc.set_ylim(-5.5, 5.5)
    axc.set_aspect("equal")
    axc.set_xlabel("East of site (m)")
    axc.set_ylabel("North of site (m)")
    axc.set_title("miss %s m; %s m from the pad" % (res.group(1), res.group(2)), fontsize=5, pad=2)
    s.label(ax, "a", fig_coords=(0.003, 0.995))
    s.label(axb, "b", fig_coords=(0.415, 0.995))
    s.label(axc, "c", fig_coords=(0.80, 0.995))
    return dict(files=s.save(f, ctx["out"], "sfig3_offset_aim"), along=along, miss=res.group(1), pad_dist=res.group(2))


if __name__ == "__main__":
    import _main
    _main.main(make)
