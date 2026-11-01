"""Figure style shared by all figure scripts.

Layout follows common journal artwork conventions: 1-column width 88 mm, 2-column width 180 mm, maximum height about
185 mm, sans-serif text (Helvetica or Arial) between 5 and 7 pt, RGB colour, line art as vector (PDF), bitmaps >= 300
dpi. Panel labels are bold lower-case letters. Colours are the Okabe-Ito colour-blind-safe palette."""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

MM = 1.0 / 25.4
W1, W2 = 88.0, 180.0          # mm
HMAX = 185.0                  # mm

# Okabe & Ito (2008) palette
OI = dict(black="#000000", orange="#E69F00", sky="#56B4E9", green="#009E73", yellow="#F0E442",
          blue="#0072B2", vermillion="#D55E00", purple="#CC79A7", grey="#8C8C8C", lightgrey="#C8C8C8")

# fixed meaning across figures
C_CAMP, C_GATE, C_MATRIX, C_DOCK = OI["blue"], OI["grey"], OI["orange"], OI["purple"]
C_SIM = OI["vermillion"]
C_FAIL = OI["vermillion"]

PHASE_C = {
    "ascent": OI["blue"], "boosters": OI["sky"], "coast": OI["grey"], "orbit": OI["grey"],
    "deorbit": OI["purple"], "entry burn": OI["vermillion"], "drag modulation": OI["green"],
    "glide": OI["orange"], "landing burn": OI["black"],
}


def setup():
    matplotlib.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"],
        "font.size": 6.0,
        "axes.titlesize": 6.0,
        "axes.labelsize": 6.0,
        "xtick.labelsize": 5.5,
        "ytick.labelsize": 5.5,
        "legend.fontsize": 5.5,
        "legend.frameon": False,
        "legend.handlelength": 1.4,
        "legend.borderaxespad": 0.3,
        "axes.linewidth": 0.5,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.labelpad": 2.0,
        "xtick.major.width": 0.5, "ytick.major.width": 0.5,
        "xtick.minor.width": 0.4, "ytick.minor.width": 0.4,
        "xtick.major.size": 2.0, "ytick.major.size": 2.0,
        "xtick.minor.size": 1.2, "ytick.minor.size": 1.2,
        "xtick.major.pad": 1.5, "ytick.major.pad": 1.5,
        "lines.linewidth": 0.8,
        "lines.markersize": 3.0,
        "patch.linewidth": 0.5,
        "pdf.fonttype": 42, "ps.fonttype": 42,   # editable text in the vector files
        "svg.fonttype": "none",
        "savefig.dpi": 300,
        "figure.dpi": 150,
        "mathtext.default": "regular",
    })


def fig(width_mm, height_mm):
    assert height_mm <= HMAX + 1e-6, "figure taller than the maximum height"
    return plt.figure(figsize=(width_mm * MM, height_mm * MM))


def label(ax, letter, x=-0.02, y=1.0, fig_coords=None):
    """Bold lower-case panel label at the top-left corner of an axes (axes fraction offsets)."""
    if fig_coords is not None:
        ax.figure.text(fig_coords[0], fig_coords[1], letter, fontsize=7, fontweight="bold", va="top", ha="left")
        return
    ax.text(x, y, letter, transform=ax.transAxes, fontsize=7, fontweight="bold", va="bottom", ha="right")


def sim_tag(ax, text="simulation", loc="upper right"):
    """Visible in-panel marker for simulation-derived content."""
    x, ha = (0.98, "right") if "right" in loc else (0.02, "left")
    y, va = (0.98, "top") if "upper" in loc else (0.02, "bottom")
    ax.text(x, y, text, transform=ax.transAxes, ha=ha, va=va, fontsize=5.5, fontstyle="italic", color=C_SIM,
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec=C_SIM, lw=0.4))


def save(f, outdir, name):
    os.makedirs(outdir, exist_ok=True)
    f.savefig(os.path.join(outdir, name + ".pdf"), metadata={"CreationDate": None, "Creator": "matplotlib"})   # no time stamp: reproducible bytes
    f.savefig(os.path.join(outdir, name + ".png"), dpi=300)
    plt.close(f)
    return [os.path.join(outdir, name + ext) for ext in (".pdf", ".png")]
