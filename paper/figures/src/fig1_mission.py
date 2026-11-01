"""Fig. 1 | Mission and system.
a, mission profile of campaign mission 1 drawn from its telemetry (ascent transcript + descent CSV); dashed segments are
   reconstructions (two-body coast to apoapsis; schematic orbit and deorbit coast).
b, vehicle at the pad and the recovered core on the pad (in-game screenshots) with subsystems annotated.
c, software architecture block diagram."""
import math
import os
import re
import sys

import matplotlib.patheffects as pe
import numpy as np
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import data as d  # noqa: E402
import style as s  # noqa: E402

MISSION = 1                       # campaign mission drawn in panel a
IMG_STACK = ("20261020_000037_gridfin_top.png", (800, 20, 1230, 880))
IMG_LANDED = ("20261021_154109_orbit_T-WorksFineGF.png", (840, 250, 1100, 800))


def _events(text):
    ev = {}
    pats = {"maxq": r"\[\s*([\d.]+)s\] hud: MAX-Q ([\d.]+) kPa at T\+(\d+) s",
            "staging": r"\[\s*([\d.]+)s\] ascent: staging 3 -> 2",
            "meco": r"\[\s*([\d.]+)s\] ascent: MECO",
            "orbit": r"\[\s*([\d.]+)s\] orbit reached",
            "payload": r"\[\s*([\d.]+)s\] payload released",
            "launch": r"\[\s*([\d.]+)s\] ascent \(own\)"}
    for k, p in pats.items():
        m = re.search(p, text)
        if m:
            ev[k] = m.groups()
    return ev


def panel_a(ax, D):
    r = next(x for x in D["done"] if x["mission"] == MISSION)
    text = d.transcript(D["logs"], r["wd"])
    a = d.ascent(text)
    ev = _events(text)
    T = d.telemetry(D["logs"], r["key"])
    km = 1e-3
    # ---- ascent (telemetry), coloured by stage
    boost = a["stage"] == 3
    ib = np.where(boost)[0][-1]
    ax.plot(a["x"][:ib + 2] * km, a["h"][:ib + 2] * km, color=s.PHASE_C["boosters"], lw=1.4, solid_capstyle="butt")
    ax.plot(a["x"][ib + 1:] * km, a["h"][ib + 1:] * km, color=s.PHASE_C["ascent"], lw=1.4, solid_capstyle="butt")
    # coast to apoapsis (reconstruction)
    xc, hc = d.coast_to_apoapsis(a)
    ax.plot(xc * km, hc * km, color=s.PHASE_C["coast"], lw=0.8, ls=(0, (3, 1.5)))
    x_ap = xc[-1] * km
    # orbit (schematic) to the right edge, and from the left edge down to the entry interface
    XL, XR = -430, x_ap + 120
    ax.plot([x_ap, XR], [100, 100], color=s.PHASE_C["orbit"], lw=0.8, ls=(0, (1, 1.5)))
    # ---- descent (telemetry): x = -(distance to the pad), the stage approaches from the west
    rng = d.ground_range(T) * km
    h = np.array(T["h"]) * km
    lab = d.descent_phases(T)
    xs = -rng
    xe0, he0 = xs[0], h[0]
    xx = np.linspace(XL, xe0, 50)
    ax.plot([XL, XL + 25], [100, 100], color=s.PHASE_C["orbit"], lw=0.8, ls=(0, (1, 1.5)))
    u = (xx - XL) / (xe0 - XL)
    ax.plot(xx, 100 - (100 - he0) * u ** 2, color=s.PHASE_C["deorbit"], lw=0.8, ls=(0, (3, 1.5)))
    order = ["coast", "drag modulation", "entry burn", "glide", "landing burn"]
    lab = np.array(lab)
    for ph in order:
        idx = np.where(lab == ph)[0]
        if len(idx) == 0:
            continue
        # contiguous runs
        runs = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
        for run in runs:
            j = np.r_[run, min(run[-1] + 1, len(xs) - 1)]
            ax.plot(xs[j], h[j], color=s.PHASE_C[ph], lw=1.4 if ph != "coast" else 0.9, solid_capstyle="butt")
    # ---- annotations
    def note(x, y, txt, dx, dy, ha="left"):
        ax.annotate(txt, (x, y), xytext=(x + dx, y + dy), fontsize=5.5, ha=ha, va="center",
                    arrowprops=dict(arrowstyle="-", lw=0.4, color="0.35", shrinkA=0, shrinkB=1))
        ax.plot([x], [y], "o", ms=2.0, color="0.15", zorder=5)
    tl = float(ev["launch"][0])
    def at_t(tt):
        i = int(np.argmin(np.abs(a["t"] - tt)))
        return a["x"][i] * km, a["h"][i] * km
    x, y = at_t(float(ev["maxq"][2]))
    note(x, y, "max-q %s kPa" % ev["maxq"][1], 30, 2)
    x, y = at_t(float(ev["staging"][0]) - tl)
    note(x, y, "booster separation", 30, 13)
    x, y = a["x"][-1] * km, a["h"][-1] * km
    note(x, y, "main-engine cut-off", -12, 12, ha="right")
    note(x_ap, 100, "circularization, payload release", -10, 9, ha="right")
    ax.text(XR - 5, 103, "orbit, 100 km", ha="right", va="bottom", fontsize=5.5, color="0.35")
    ax.text(XL + 3, 103, "deorbit burn about\nhalf a revolution earlier", ha="left", va="bottom", fontsize=5.5,
            color=s.PHASE_C["deorbit"], linespacing=0.95)
    ieb = np.where(lab == "entry burn")[0]
    note(xs[ieb[0]], h[ieb[0]], "entry burn, %.0f km" % h[ieb[0]], 25, 10)
    ax.axhline(70, color="0.75", lw=0.4, ls=(0, (4, 2)), zorder=0)
    ax.text(XR - 5, 70.5, "atmosphere boundary, 70 km", ha="right", va="bottom", fontsize=5, color="0.5")
    ax.set_xlim(XL, XR)
    ax.set_ylim(0, 118)
    ax.set_xlabel("Ground distance from the launch pad (km; negative, approaching from the west)")
    ax.set_ylabel("Altitude (km)")
    # ---- inset: final descent
    ins = ax.inset_axes([0.665, 0.10, 0.31, 0.40])
    sel = rng < 32
    for ph in order:
        idx = np.where((lab == ph) & sel)[0]
        if len(idx) == 0:
            continue
        runs = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
        for run in runs:
            j = np.r_[run, min(run[-1] + 1, len(xs) - 1)]
            ins.plot(rng[j], h[j], color=s.PHASE_C[ph], lw=1.2, solid_capstyle="butt")
    ins.set_xlim(32, -0.5)
    ins.set_ylim(0, 25)
    ins.set_xlabel("Distance to pad (km)", fontsize=5, labelpad=1)
    ins.set_ylabel("Altitude (km)", fontsize=5, labelpad=1)
    ins.tick_params(labelsize=5)
    ib_ = np.where(lab == "landing burn")[0][0]
    ins.annotate("landing-burn ignition\n%.0f m, miss %.1f m" % (h[ib_] * 1e3, r["miss2"]), (rng[ib_], h[ib_]),
                 xytext=(13, 7), fontsize=5, ha="center", arrowprops=dict(arrowstyle="-", lw=0.4, color="0.35"))
    ins.text(20, 13.5, "glide below 10 km", fontsize=5, color=s.PHASE_C["glide"], ha="left")
    # legend of phases
    from matplotlib.lines import Line2D
    hand = [Line2D([], [], color=s.PHASE_C[k], lw=1.4) for k in
            ("boosters", "ascent", "coast", "drag modulation", "entry burn", "glide", "landing burn")]
    labs = ["ascent, boosters", "ascent, core", "coast (dashed: reconstructed)", "drag modulation", "entry burn",
            "glide", "landing burn"]
    ax.legend(hand, labs, loc="upper center", bbox_to_anchor=(0.47, 1.02), ncol=4, fontsize=5.2, columnspacing=1.2)
    return r


def _img(ax, screens, spec):
    f, box = spec
    d.show_screenshot(ax, screens, f, box, interpolation="lanczos")
    return box


def panel_b(ax1, ax2, screens):
    b1 = _img(ax1, screens, IMG_STACK)
    b2 = _img(ax2, screens, IMG_LANDED)

    def ann(ax, box, xy, txt, xytext, ha="left"):
        x, y = xy[0] - box[0], xy[1] - box[1]
        ax.annotate(txt, (x, y), xytext=(xytext[0] - box[0], xytext[1] - box[1]), fontsize=5.2, ha=ha, va="center",
                    color="white", path_effects=[pe.withStroke(linewidth=1.2, foreground="0.15")],
                    arrowprops=dict(arrowstyle="-", lw=0.5, color="white", shrinkA=0, shrinkB=0))
    ann(ax1, b1, (1000, 110), "payload\nfairing", (1110, 90))
    ann(ax1, b1, (1010, 290), "control\nsurfaces\n(stowed)", (1110, 270))
    ann(ax1, b1, (985, 420), "core\nstage", (1110, 420))
    ann(ax1, b1, (905, 560), "4 solid\nboosters", (812, 455), ha="left")
    ann(ax1, b1, (848, 760), "launch\nclamps", (885, 860), ha="left")
    ann(ax2, b2, (885, 355), "4 grid fins,\n8 airbrakes", (860, 290), ha="left")
    ann(ax2, b2, (950, 470), "core\nstage", (852, 470), ha="left")
    ann(ax2, b2, (928, 735), "4 landing\nlegs", (860, 650), ha="left")
    ann(ax2, b2, (970, 770), "engine", (1030, 780))
    ax1.set_title("At the pad (lift-off 92.7 t)", fontsize=5.5, pad=2)
    ax2.set_title("Core after landing", fontsize=5.5, pad=2)


def _box(ax, x, y, w, h, txt, fc, ec="0.3", fs=5.2, bold_first=True, lw=0.5):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=1.2", fc=fc, ec=ec, lw=lw))
    lines = txt.split("\n")
    if bold_first and len(lines) > 1:
        ax.text(x + w / 2, y + h - 2.2, lines[0], ha="center", va="top", fontsize=fs, fontweight="bold")
        ax.text(x + w / 2, y + h - 5.6, "\n".join(lines[1:]), ha="center", va="top", fontsize=fs - 0.4,
                linespacing=1.05, color="0.15")
    else:
        ax.text(x + w / 2, y + h / 2, txt, ha="center", va="center", fontsize=fs, fontweight="bold" if bold_first else None)


def _arrow(ax, p, q, txt=None, both=False, off=(0, 0), color="0.25", ls="-"):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="<|-|>" if both else "-|>", mutation_scale=5, lw=0.6, color=color,
                                 linestyle=ls, shrinkA=0, shrinkB=0))
    if txt:
        ax.text((p[0] + q[0]) / 2 + off[0], (p[1] + q[1]) / 2 + off[1], txt, fontsize=5, ha="center", va="center",
                color="0.2")


def panel_c(ax):
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis("off")
    LG, LB, LO, LY = "#EEF4FA", "#E8F5F0", "#FDF3E1", "#F2F2F2"
    # layer frames
    ax.add_patch(FancyBboxPatch((0, 76), 100, 23, boxstyle="round,pad=0,rounding_size=1.5", fc=LY, ec="none"))
    ax.add_patch(FancyBboxPatch((0, 24), 100, 49, boxstyle="round,pad=0,rounding_size=1.5", fc=LG, ec="none"))
    ax.add_patch(FancyBboxPatch((0, 0), 100, 21, boxstyle="round,pad=0,rounding_size=1.5", fc=LO, ec="none"))
    ax.text(1.5, 97.5, "Game process", fontsize=5.5, fontweight="bold", va="top", color="0.3")
    ax.text(1.5, 71.5, "Flight software: Python package kspilot (separate process)", fontsize=5.5, fontweight="bold",
            va="top", color=s.OI["blue"])
    ax.text(1.5, 19.5, "Bench: tools/", fontsize=5.5, fontweight="bold", va="top", color="#8a5a00")
    # game layer
    _box(ax, 3, 78, 37, 14, "Kerbal Space Program 1.12.5\nrigid multi-part vehicles, rotating planet,\npart-based aerodynamics", "white")
    _box(ax, 44, 78, 22, 14, "kRPC 0.6.0 server\nremote procedure calls,\nstreams", "white")
    _box(ax, 70, 78, 27, 14, "AutoLab plugin (C#)\nsave/dialog handling, camera,\noverlay; not needed by guidance,\nneeded for unattended operation", "white")
    # flight software: sequencer + phase modules
    _box(ax, 3, 55, 22, 11, "mission.py\nphase sequencer,\npayload release", "white")
    mods = [("feasibility.py", "pre-launch Δv check"), ("ascent_own.py", "ascent, booster\nseparation guard"),
            ("deorbit.py, entry.py", "window survey,\nimpact predictor"), ("power.py", "hibernation,\nEC budget")]
    x0 = 28
    for i, (m, f) in enumerate(mods):
        _box(ax, x0 + i * 17.5, 55, 16, 11, m + "\n" + f, "white", fs=4.9)
    _box(ax, 3, 27, 46, 24, "landing.py\nentry burn with mass-based Δv; drag modulation\non two impact predictions and retro pulses;\n"
         "glide (ZEM/ZEV tilt steering below 10 km);\nhoverslam ignition test and landing law", "white")
    _box(ax, 52, 40, 21, 11, "powered_descent.py\nconvex planner\n(implemented, not flown)", "white", ec="0.55")
    _box(ax, 76, 40, 21, 11, "aim.py\noffset aim (optional;\nnot in analysed flights)", "white", ec="0.55")
    _box(ax, 52, 27, 45, 10, "flightctl.py, burn.py, maneuver.py\nattitude, throttle and burn execution", "white")
    # bench
    _box(ax, 3, 2, 30, 13, "watchdog.py supervisor\ntranscript, stall restart,\ngame restart", "white")
    _box(ax, 36, 2, 28, 13, "orbit_recover.py, matrix_run.sh\nmission and matrix launchers;\nreplay_sim.py (simulation)", "white")
    _box(ax, 67, 2, 30, 13, "Raw records\nwd_*.out transcripts, landing CSV,\nexperiments.jsonl → compute_stats.py", "white")
    # arrows
    _arrow(ax, (40, 85), (44, 85), both=True)
    _arrow(ax, (55, 66.5), (55, 78), both=True, txt="state reads ↑   commands ↓\n(throttle, attitude target,\nstaging, action groups, warp)",
           off=(16, 0))
    _arrow(ax, (14, 55), (14, 51))
    _arrow(ax, (25, 60.5), (28, 60.5))
    _arrow(ax, (18, 15), (18, 27))
    ax.text(20, 22.5, "launches and supervises flights", fontsize=5, va="center", color="0.2")
    _arrow(ax, (64, 8.5), (67, 8.5))
    _arrow(ax, (33, 8.5), (36, 8.5))


def make(ctx):
    s.setup()
    D = d.load(ctx["logs"])
    f = s.fig(s.W2, 150)
    ax_a = f.add_axes([0.06, 0.585, 0.925, 0.395])
    r = panel_a(ax_a, D)
    ax_b1 = f.add_axes([0.005, 0.035, 0.16, 0.47])
    ax_b2 = f.add_axes([0.17, 0.035, 0.12, 0.47])
    panel_b(ax_b1, ax_b2, ctx["screens"])
    ax_c = f.add_axes([0.305, 0.02, 0.69, 0.49])
    panel_c(ax_c)
    s.label(ax_a, "a", fig_coords=(0.005, 0.995))
    s.label(ax_b1, "b", fig_coords=(0.005, 0.535))
    s.label(ax_c, "c", fig_coords=(0.300, 0.535))
    out = s.save(f, ctx["out"], "fig1_mission_system")
    return dict(files=out, mission=r["mission"], key=r["key"])


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import make_all
    make_all.run_one(make)
