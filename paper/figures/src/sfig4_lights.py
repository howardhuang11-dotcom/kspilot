"""Supplementary Fig. 4 | Lamp variant (T-WorksFineGF-LIT) night check: pre-flight light check from three cameras and the
recovered core after a night landing (in-game screenshots)."""
import os
import sys


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import data as d  # noqa: E402
import style as s  # noqa: E402

SHOTS = [("light_check_20261021_205906_side.png", (400, 100, 1520, 730), "light check, side camera"),
         ("light_check_20261021_205906_chase.png", (400, 100, 1520, 730), "light check, chase camera"),
         ("light_check_20261021_205906_below.png", (400, 100, 1520, 730), "light check, camera below"),
         ("20261021_222157_orbit_T-WorksFineGF-LIT.png", (300, 80, 1740, 890), "exploratory mission 27 after landing")]


def make(ctx):
    s.setup()
    f = s.fig(s.W2, 104)
    W, H = 0.49, 0.455
    for i, (fn, box, cap) in enumerate(SHOTS):
        r, c = divmod(i, 2)
        ax = f.add_axes([0.005 + c * (W + 0.015), 0.51 - r * (H + 0.035), W, H])
        d.show_screenshot(ax, ctx["screens"], fn, box, aspect="auto")
        ax.text(0.015, 0.97, cap, transform=ax.transAxes, color="white", fontsize=5.5, va="top")
        s.label(ax, "abcd"[i], fig_coords=(c * (W + 0.015), min(0.51 - r * (H + 0.035) + H + 0.03, 0.998)))
    return dict(files=s.save(f, ctx["out"], "sfig4_lights"))


if __name__ == "__main__":
    import _main
    _main.main(make)
