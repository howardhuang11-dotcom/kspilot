"""Fig. 4 | Trajectories of the complete missions.
a, altitude against ground distance for every complete campaign mission (ascent from transcript telemetry to main-engine
   cut-off; return from the descent CSV, from 77 km to touchdown).
b, descent ground tracks (CSV latitude/longitude) coloured by orbit inclination; inset: reconstructed ground track of
   the first revolution after insertion for each inclination (two-body circular orbit on the rotating planet).
c, entry corridor: dynamic pressure and heating fraction against altitude with the software thresholds."""
import math
import os
import sys

import numpy as np
from matplotlib.lines import Line2D

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import data as d  # noqa: E402
import style as s  # noqa: E402

KSC_LAT, KSC_LON = -0.0972, -74.5577
INC_C = {0.0: s.OI["sky"], 6.0: s.OI["grey"], 28.0: s.OI["green"], 45.0: s.OI["orange"], 90.0: s.OI["purple"]}


def case_style(r):
    """Colour/label by robustness factor; baseline grey."""
    pe, ap, inc = r["target"]
    if r["result"] == "FAIL":
        return s.C_FAIL, "failure (missions 12, 21)", 1.0, 3
    if inc != 6.0:
        return INC_C[inc], "%g° inclination" % inc, 1.0, 2
    if ap == 80.0:
        return s.OI["blue"], "80 km orbit", 1.0, 2
    if ap == 110.0:
        return s.OI["yellow"], "110 km orbit", 1.0, 2
    if r["craft"].endswith("P45") or r["craft"].endswith("P60"):
        return s.OI["vermillion"] if False else "#7A4A00", "4.50 / 6.00 t payload", 1.0, 2
    return s.OI["lightgrey"], "baseline (100 km, 6°, 2.48 t)", 0.6, 1


def panel_a(ax, D):
    seen = {}
    for r in D["done"]:
        c, lab, lw, z = case_style(r)
        text = d.transcript(D["logs"], r["wd"])
        a = d.ascent(text)
        # in-game frame capture slows the game against the wall clock that stamps the ascent telemetry, so the
        # integrated ground distance of those ascents is invalid; they are omitted (returns are in game time)
        if a is not None and "frame capture" not in (r["recording"] or ""):
            ax.plot(a["x"] / 1e3, a["h"] / 1e3, color=c, lw=lw, zorder=z)
        T = d.telemetry(D["logs"], r["key"])
        if T is not None:
            ax.plot(-d.ground_range(T) / 1e3, np.array(T["h"]) / 1e3, color=c, lw=lw, zorder=z)
        seen.setdefault(lab, (c, lw))
    m12 = next(r for r in D["done"] if r["mission"] == 12)
    a12 = d.ascent(d.transcript(D["logs"], m12["wd"]))
    i = int(np.argmax(a12["h"]))
    ax.annotate("12: core lost at\nbooster separation", (a12["x"][i] / 1e3, a12["h"][i] / 1e3), xytext=(38, 34),
                fontsize=5, color=s.C_FAIL, arrowprops=dict(arrowstyle="-", lw=0.4, color=s.C_FAIL))
    m21 = next(r for r in D["done"] if r["mission"] == 21)
    T21 = d.telemetry(D["logs"], m21["key"])
    eb = d.entry_burn_segment(T21)
    x21 = -d.ground_range(T21)[eb[1]] / 1e3
    ax.annotate("21: entry-burn over-burn,\nlanded 3.8 km short", (x21, T21["h"][eb[1]] / 1e3), xytext=(-150, 52),
                fontsize=5, color=s.C_FAIL, ha="left", arrowprops=dict(arrowstyle="-", lw=0.4, color=s.C_FAIL))
    ax.axhline(70, color="0.8", lw=0.4, ls=(0, (4, 2)), zorder=0)
    ax.set_xlim(-480, 135)
    ax.set_ylim(0, 80)
    ax.set_xlabel("Ground distance from the pad (km); return negative")
    ax.set_ylabel("Altitude (km)")
    ax.text(-300, 79, "return (descent CSV)", fontsize=5.5, va="top")
    ax.text(125, 77, "ascent to cut-off\n(transcript)", fontsize=5.5, va="top", ha="right")
    order = ["baseline (100 km, 6°, 2.48 t)", "0° inclination", "28° inclination", "45° inclination",
             "90° inclination", "80 km orbit", "110 km orbit", "4.50 / 6.00 t payload", "failure (missions 12, 21)"]
    hand = [Line2D([], [], color=seen[k][0], lw=1.0) for k in order if k in seen]
    ax.legend(hand, [k for k in order if k in seen], loc="lower left", bbox_to_anchor=(0.0, 0.0), fontsize=5,
              ncol=1, handlelength=1.2, labelspacing=0.25)


def groundtrack(inc_deg, alt_km=100.0, t_end=None, n=600, ascending=True):
    """Reconstructed ground track of a circular orbit inserted over the launch site (two-body, rotating planet)."""
    r = d.R_KERBIN + alt_km * 1e3
    P = 2 * math.pi * math.sqrt(r ** 3 / d.GM_KERBIN)
    t = np.linspace(0, t_end or P, n)
    i = math.radians(max(inc_deg, 1e-3))
    lat0 = math.radians(KSC_LAT)
    u0 = math.asin(max(-1, min(1, math.sin(lat0) / math.sin(i))))   # argument of latitude at the launch site
    u = u0 + 2 * math.pi * t / P
    lat = np.degrees(np.arcsin(np.sin(i) * np.sin(u)))
    dlon = np.degrees(np.arctan2(np.cos(i) * np.sin(u), np.cos(u))) - np.degrees(
        math.atan2(math.cos(i) * math.sin(u0), math.cos(u0)))
    lon = KSC_LON + dlon - np.degrees(d.OMEGA_KERBIN * t)
    lon = (lon + 180) % 360 - 180
    return lat, lon


def panel_b(ax, D):
    for r in D["done"]:
        T = d.telemetry(D["logs"], r["key"])
        if T is None:
            continue
        inc = r["target"][2]
        c = INC_C[inc] if r["result"] == "PASS" else s.C_FAIL
        lw = 0.6 if inc == 6.0 else 1.0
        ax.plot(T["lon"], T["lat"], color=c, lw=lw, zorder=3 if inc != 6 else 2)
    ax.plot([KSC_LON], [KSC_LAT], marker="*", ms=6, color="k", zorder=5)
    ax.text(KSC_LON + 1.2, KSC_LAT - 1.5, "launch pad", fontsize=5.5, va="top")
    ax.set_xlim(-121, -66)
    ax.set_ylim(-9, 41)
    ax.set_aspect("equal")
    ax.set_xlabel("Longitude (°)")
    ax.set_ylabel("Latitude (°)")
    for inc, (x, y) in {0.0: (-118, 0.8), 6.0: (-118, 5.6), 28.0: (-108, 18.3), 45.0: (-101, 27.6),
                        90.0: (-70.5, 38.5)}.items():
        ax.text(x, y, "%g°" % inc, fontsize=5.5, color=INC_C[inc] if inc != 6 else "0.35", ha="left")
    ax.text(-120, -4.6, "6° (descending pass)", fontsize=5, color="0.35")
    ax.grid(color="0.92", lw=0.4)
    ax.set_axisbelow(True)
    # inset: reconstructed first revolution
    ins = ax.figure.add_axes([0.555, 0.085, 0.43, 0.25])
    for inc, c in INC_C.items():
        lat, lon = groundtrack(inc)
        brk = np.where(np.abs(np.diff(lon)) > 90)[0] + 1
        for seg_lat, seg_lon in zip(np.split(lat, brk), np.split(lon, brk)):
            ins.plot(seg_lon, seg_lat, color=c, lw=0.7)
    ins.plot([KSC_LON], [KSC_LAT], marker="*", ms=4, color="k")
    ins.set_xlim(-180, 180)
    ins.set_ylim(-90, 90)
    ins.set_xticks([-180, 0, 180])
    ins.set_yticks([-90, 0, 90])
    ins.set_xlabel("Longitude (°)")
    ins.set_ylabel("Latitude (°)")
    ins.set_aspect("equal")
    ins.grid(color="0.92", lw=0.4)
    ins.set_axisbelow(True)
    ins.text(-175, 82, "first revolution after insertion (reconstructed, two-body)", fontsize=5, va="top")


def panel_c(axq, axh, D):
    ebh = []
    for r in D["done"]:
        T = d.telemetry(D["logs"], r["key"])
        if T is None:
            continue
        h = np.array(T["h"]) / 1e3
        c = s.C_FAIL if r["result"] == "FAIL" else s.C_CAMP
        lw, al = (0.8, 1.0) if r["result"] == "FAIL" else (0.5, 0.35)
        axq.plot(np.array(T["q"]) / 1e3, h, color=c, lw=lw, alpha=al)
        ht = np.array(T["heat"])
        ok = np.isfinite(ht) & (ht > 0)
        axh.plot(ht[ok], h[ok], color=c, lw=lw, alpha=al)
        eb = d.entry_burn_segment(T)
        if eb:
            ebh.append((h[eb[0]], h[eb[1]]))
    lo, hi = min(x[1] for x in ebh), max(x[0] for x in ebh)
    for ax in (axq, axh):
        ax.axhspan(lo, hi, color=s.PHASE_C["entry burn"], alpha=0.12, lw=0)
        ax.axhline(10, color=s.PHASE_C["glide"], lw=0.5, ls=(0, (3, 2)))
        ax.set_ylim(0, 75)
    axq.axvline(25, color="0.3", lw=0.6, ls=(0, (4, 2)))
    axq.text(24.4, 60, "safety-burn\nthreshold\n25 kPa", fontsize=5, ha="right", va="center")
    axq.text(26.5, (lo + hi) / 2, "entry burn", fontsize=5, color=s.PHASE_C["entry burn"], va="center", ha="right")
    axq.text(26.5, 11, "glide", fontsize=5, color=s.PHASE_C["glide"], va="bottom", ha="right")
    axq.set_xlim(0, 27)
    axq.set_xlabel("Dynamic pressure (kPa)")
    axq.set_ylabel("Altitude (km)")
    axh.axvline(1.0, color="0.3", lw=0.6, ls=(0, (4, 2)))
    axh.text(0.98, 60, "part\ntemperature\nlimit", fontsize=5, ha="right", va="center")
    axh.set_xlim(0.3, 1.05)
    axh.set_xlabel("Peak part temperature /\nmaximum temperature")
    axh.tick_params(labelleft=False)
    axq.plot([], [], color=s.C_CAMP, lw=0.8, label="exploratory (n = %d)" % sum(
        1 for r in D["done"] if r["result"] == "PASS" and d.telemetry(D["logs"], r["key"]) is not None))
    axq.plot([], [], color=s.C_FAIL, lw=0.8, label="mission 21")
    axq.legend(loc="upper left", bbox_to_anchor=(0.05, 1.0), fontsize=5)


def make(ctx):
    s.setup()
    D = d.load(ctx["logs"])
    f = s.fig(s.W2, 118)
    ax_a = f.add_axes([0.055, 0.56, 0.42, 0.42])
    panel_a(ax_a, D)
    ax_b = f.add_axes([0.555, 0.44, 0.43, 0.545])
    panel_b(ax_b, D)
    axq = f.add_axes([0.055, 0.10, 0.205, 0.36])
    axh = f.add_axes([0.27, 0.10, 0.205, 0.36])
    panel_c(axq, axh, D)
    s.label(ax_a, "a", fig_coords=(0.003, 0.995))
    s.label(ax_b, "b", fig_coords=(0.50, 0.995))
    s.label(axq, "c", fig_coords=(0.003, 0.50))
    return dict(files=s.save(f, ctx["out"], "fig4_trajectories"))


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import make_all
    make_all.run_one(make)
