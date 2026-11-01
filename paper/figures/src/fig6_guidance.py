"""Fig. 6 | Guidance internals and the two mishaps.
a, predicted along-track impact error as the return proceeds: vacuum mid-course predictions, drag-modulation
   predictions (airbrakes deployed), glide predictions, landing-burn ignition offset and final miss (transcripts).
b, entry-burn velocity-increment accounting: counted (logged) against mass-based Δv per mission (landing CSV mass), and
   the mission-21 counterfactual replay (simulation, GNC2 entry_replay.json).
c, booster separation: explosions within 15 s of separation against dynamic pressure at separation."""
import json
import math
import os
import re
import sys

import numpy as np
from matplotlib.lines import Line2D

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import data as d  # noqa: E402
import style as s  # noqa: E402

ISP_ENTRY = 320.0          # s, logged at every entry-burn start since the correction ("Isp 320.0 s")
HIGHLIGHT = 1              # mission drawn with markers in panel a

T_ = r"\[\s*([\d.]+)s\] "
RE = dict(vac=re.compile(T_ + r"vacuum correction \d+: predicted miss \d+m \(along ([+-]\d+)"),
          dm=re.compile(T_ + r"drag modulation: brakes (?:IN|OUT) \(out ([+-]\d+)m / in ([+-]\d+)m"),
          gl=re.compile(T_ + r"cross glide: cross [+-]?\d+ m \([\d.]+ m per m/s\) along ([+-]\d+) m"),
          ign=re.compile(T_ + r"IGNITION h=\d+m vz=-?[\d.]+ vh=[\d.]+ offset=(\d+)m"),
          res=re.compile(T_ + r"RESULT PASS miss=([\d.]+)m"))


def convergence(text):
    out = {}
    for k, p in RE.items():
        out[k] = [tuple(float(x) for x in m.groups()) for m in p.finditer(text)]
    if not out["ign"]:
        return None
    t0 = out["ign"][0][0]
    return {k: [(v[0] - t0,) + v[1:] for v in vals] for k, vals in out.items()}


def panel_a(ax, D):
    n = 0
    for r in D["done"]:
        if r["result"] != "PASS" or "frame capture" in (r["recording"] or ""):
            continue          # frame capture slows the game against the transcript clock
        c = convergence(d.transcript(D["logs"], r["wd"]))
        if c is None:
            continue
        n += 1
        pts = sorted([(t, a) for t, a in c["vac"]] + [(t, o) for t, o, _ in c["dm"]] + [(t, a) for t, a in c["gl"]] +
                     [(t, o) for t, o in c["ign"]] + [(t, m) for t, m in c["res"]])
        hi = r["mission"] == HIGHLIGHT
        if not hi:
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color="0.75", lw=0.4, zorder=1)
            continue
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color="0.2", lw=0.6, zorder=3)
        ax.plot([t for t, a in c["vac"]], [a for t, a in c["vac"]], "o", color=s.PHASE_C["deorbit"], ms=3, zorder=4)
        ax.plot([t for t, o, i in c["dm"]], [o for t, o, i in c["dm"]], "o", color=s.PHASE_C["drag modulation"], ms=3,
                zorder=4)
        ax.plot([t for t, o, i in c["dm"]], [i for t, o, i in c["dm"]], "^", mfc="none",
                mec=s.PHASE_C["drag modulation"], ms=3, mew=0.6, zorder=4)
        ax.plot([t for t, a in c["gl"]], [a for t, a in c["gl"]], "s", color=s.PHASE_C["glide"], ms=2.6, zorder=4)
        ax.plot([t for t, o in c["ign"]], [o for t, o in c["ign"]], "D", color="k", ms=2.6, zorder=4)
        ax.plot([t for t, m in c["res"]], [m for t, m in c["res"]], "*", color=s.C_CAMP, ms=5, zorder=4)
    ax.set_yscale("symlog", linthresh=10, linscale=0.6)
    ax.set_ylim(-3e3, 5e5)
    ax.set_yticks([-1000, -100, -10, 0, 10, 100, 1000, 10000, 100000])
    ax.set_yticklabels(["−10$^3$", "−10$^2$", "−10", "0", "10", "10$^2$", "10$^3$", "10$^4$", "10$^5$"])
    ax.axhline(0, color="0.6", lw=0.4)
    ax.axvline(0, color="0.6", lw=0.4, ls=(0, (2, 2)))
    ax.set_xlim(-330, 40)
    ax.set_xlabel("Time from landing-burn ignition (s, supervisor clock)")
    ax.set_ylabel("Predicted along-track error (m)")
    hand = [Line2D([], [], ls="", marker="o", color=s.PHASE_C["deorbit"], ms=3),
            Line2D([], [], ls="", marker="o", color=s.PHASE_C["drag modulation"], ms=3),
            Line2D([], [], ls="", marker="^", mfc="none", mec=s.PHASE_C["drag modulation"], ms=3),
            Line2D([], [], ls="", marker="s", color=s.PHASE_C["glide"], ms=2.6),
            Line2D([], [], ls="", marker="D", color="k", ms=2.6),
            Line2D([], [], ls="", marker="*", color=s.C_CAMP, ms=5),
            Line2D([], [], color="0.75", lw=0.6)]
    labs = ["vacuum mid-course prediction", "drag modulation, airbrakes out", "drag modulation, airbrakes in",
            "glide steering prediction", "ignition offset", "miss after touchdown", "other missions (n = %d)" % (n - 1)]
    ax.legend(hand, labs, loc="upper right", fontsize=5, handletextpad=0.3, labelspacing=0.2, borderaxespad=0.1)
    hi = next(r for r in D["done"] if r["mission"] == HIGHLIGHT)
    txt = d.transcript(D["logs"], hi["wd"])
    t_ign = float(re.search(T_ + r"IGNITION", txt).group(1))
    t_eb0 = float(re.search(T_ + r"ENTRY BURN start", txt).group(1)) - t_ign
    t_eb1 = float(re.search(T_ + r"ENTRY BURN end", txt).group(1)) - t_ign
    ax.axvspan(t_eb0, t_eb1, color=s.PHASE_C["entry burn"], alpha=0.15, lw=0)
    ax.text(0.5 * (t_eb0 + t_eb1), 4e4, "entry\nburn", fontsize=5, ha="center", va="center",
            color=s.PHASE_C["entry burn"])
    ax.text(-262, 2.5e4, "vacuum coast after the deorbit burn", fontsize=5, ha="center", va="center", color="0.35")
    ax.text(-100, 2.5e5, "atmospheric return: drag modulation, cross-track glide, range-trim pulses", fontsize=5,
            ha="center", va="center", color="0.35")
    ax.text(-325, -2200, "mission %d highlighted" % HIGHLIGHT, fontsize=5, color="0.2", va="bottom")
    return n


def entry_dv(D):
    rows = []
    for r in D["done"]:
        T = d.telemetry(D["logs"], r["key"])
        if T is None or r["eb_dv"] is None:
            continue
        eb = d.entry_burn_segment(T)
        i0, i1 = max(eb[0] - 1, 0), min(eb[1] + 1, len(T["m"]) - 1)
        # r3: the engine burns for 0.2-0.5 s after the first throttle-zero row, so take the settled mass: follow
        # the mass while it still falls, within 2 s and with the throttle still commanded to zero (no vernier).
        m, ut, tc = T["m"], T["ut"], T.get("thr_cmd")
        while (i1 + 1 < len(m) and m[i1 + 1] < m[i1] - 0.5 and ut[i1 + 1] - ut[eb[1]] <= 3.0
               and (tc is None or float(tc[i1 + 1]) == 0)):
            i1 += 1
        m0, m1 = T["m"][i0], T["m"][i1]
        rows.append(dict(mission=r["mission"], counted=r["eb_dv"], mass=ISP_ENTRY * d.G0 * math.log(m0 / m1),
                         prop=m0 - m1, stalls=r["deadman"], rec=r["recording"], result=r["result"]))
    return rows


def panel_b1(ax, D):
    rows = entry_dv(D)
    for x in rows:
        c = s.C_FAIL if x["result"] == "FAIL" else ("0.35" if x["stalls"] == 0 else s.OI["orange"])
        ax.plot([x["mission"]] * 2, [x["counted"], x["mass"]], color="0.8", lw=0.6, zorder=1)
        ax.plot(x["mission"], x["counted"], "_", color="k", ms=4, mew=0.9, zorder=3)
        ax.plot(x["mission"], x["mass"], "o", color=c, ms=2.8, zorder=3)
        if x["stalls"]:
            ax.text(x["mission"], x["mass"] + 6, "%d" % x["stalls"], fontsize=5, ha="center", va="bottom", color=c)
    ax.axvspan(17.5, 21.5, color=s.OI["yellow"], alpha=0.25, lw=0)
    ax.text(19.5, 552, "frame\ncapture", fontsize=5, ha="center", va="bottom", color="0.3")
    ax.axvline(21.5, color="0.4", lw=0.5, ls=(0, (2, 2)))
    ax.text(21.8, 775, "mass-based\ncounter in service", fontsize=5, va="top", color="0.3")
    ax.set_xlim(0, 29)
    ax.set_ylim(545, 780)
    ax.set_xlabel("Exploratory mission")
    ax.set_ylabel("Entry-burn Δv (m s$^{-1}$)")
    hand = [Line2D([], [], ls="", marker="_", color="k", ms=4, mew=0.9),
            Line2D([], [], ls="", marker="o", color="0.35", ms=2.8),
            Line2D([], [], ls="", marker="o", color=s.OI["orange"], ms=2.8),
            Line2D([], [], ls="", marker="o", color=s.C_FAIL, ms=2.8)]
    ax.legend(hand, ["counted (logged)", "mass-based, I$_{sp}$ 320 s", "with control-loop stalls (number)",
                     "mission 21 (failure)"], loc="upper left", fontsize=5, handletextpad=0.2, labelspacing=0.2,
              borderaxespad=0.1)
    return rows


def panel_b2(ax, gnc2):
    R = json.load(open(os.path.join(gnc2, "entry_replay.json")))
    f4 = R["20261021_135358"]
    nominal = [v["log"]["E10_along"] for k, v in R.items() if k != "20261021_135358"]
    cases = [("as flown\n(logged throttle)", f4["log"], s.C_FAIL),
             ("old counter,\nreplayed", f4["legacy"], s.C_FAIL),
             ("predictor–corrector\ncut-off, old counter", f4["pcc_legacy"], s.C_SIM),
             ("mass-based\ncounter", f4["counter"], s.C_CAMP)]
    y = np.arange(len(cases))[::-1]
    for yi, (lab, v, c) in zip(y, cases):
        ax.barh(yi, v["E10_along"] / 1e3, color=c, height=0.6, alpha=0.85)
        ax.text(v["E10_along"] / 1e3 + (0.12 if v["E10_along"] > 0 else -0.12), yi,
                "%+.0f m\n(Δv %.0f)" % (v["E10_along"], v["dv_true"]), fontsize=5,
                ha="left" if v["E10_along"] > 0 else "right", va="center")
    ax.axvspan(min(nominal) / 1e3, max(nominal) / 1e3, color="0.6", alpha=0.35, lw=0)
    ax.text(max(nominal) / 1e3 + 0.05, len(cases) - 0.45, "nominal\nrange,\n%d flights" % len(nominal), fontsize=5,
            va="top", color="0.35")
    ax.set_yticks(y)
    ax.set_yticklabels([c[0] for c in cases], fontsize=5)
    ax.set_xlim(-6.2, 1.6)
    ax.axvline(0, color="0.3", lw=0.5)
    ax.set_xlabel("Along-track error at 10 km (km)")
    ax.tick_params(axis="y", length=0, pad=1)
    ax.spines["left"].set_visible(False)
    s.sim_tag(ax, "simulation (replay)", loc="lower left")
    return f4


def panel_c(ax, D):
    pts = []
    for r in D["sep"]:
        txt = d.transcript(D["logs"], r["wd"])
        m = re.search(r"RESULT SEPTEST .*?explosions=(\d+) core_hit=(\d+) q=([\d.]+).*?explosions_15s=(\d+)", txt)
        pts.append(("test", float(m.group(3)), int(m.group(4)), 0, int(m.group(2))))
    for r in D["done"]:
        if r["sep_win"] and r["sep_q_kPa"]:
            pts.append(("mission", r["sep_q_kPa"], r["sep_win"][1], r["sep_win"][2], 0))
    rng = np.random.default_rng(3)
    for kind, q, nb, nc, hit in pts:
        jx = rng.uniform(-0.15, 0.15)
        if kind == "test":
            ax.plot(q + jx, nb, "o", mfc="white", mec="k", ms=3.2, mew=0.6, zorder=3)
        else:
            ax.plot(q + jx, nb, "s", color=s.C_CAMP, ms=2.8, zorder=3, alpha=0.85)
    ax.axvline(20, color=s.C_FAIL, lw=0.6, ls=(0, (3, 2)))
    ax.axvline(10, color=s.OI["green"], lw=0.6, ls=(0, (3, 2)))
    ax.text(19.7, 4.6, "limit before\ncorrective action", fontsize=5, color=s.C_FAIL, ha="right", va="top")
    ax.text(10.3, 4.6, "limit after", fontsize=5, color=s.OI["green"], ha="left", va="top")
    ax.set_xlim(7, 22)
    ax.set_ylim(-0.4, 4.8)
    ax.set_yticks(range(0, 5))
    ax.set_xlabel("Dynamic pressure at separation (kPa)")
    ax.set_ylabel("Explosions within 15 s")
    hand = [Line2D([], [], ls="", marker="o", mfc="white", mec="k", ms=3.2, mew=0.6),
            Line2D([], [], ls="", marker="s", color=s.C_CAMP, ms=2.8)]
    ntest = sum(1 for p in pts if p[0] == "test")
    nmis = len(pts) - ntest
    ax.legend(hand, ["separation-only tests (n = %d)" % ntest, "exploratory missions, booster side (n = %d)" % nmis],
              loc="upper left", bbox_to_anchor=(0.22, 0.86), fontsize=5, handletextpad=0.2)
    ax.text(19.7, 2.05, "mission 12: core lost;\nq not recorded", fontsize=5, ha="right", color=s.C_FAIL)
    ax.text(21.8, 1.25, "core-side explosions: 0 of %d" % len(pts), fontsize=5, ha="right", color="0.2")
    return pts


def make(ctx):
    s.setup()
    D = d.load(ctx["logs"])
    f = s.fig(s.W2, 120)
    ax_a = f.add_axes([0.06, 0.60, 0.925, 0.375])
    n = panel_a(ax_a, D)
    ax_b1 = f.add_axes([0.06, 0.085, 0.27, 0.40])
    rows = panel_b1(ax_b1, D)
    ax_b2 = f.add_axes([0.43, 0.085, 0.17, 0.40])
    f4 = panel_b2(ax_b2, ctx["gnc2"])
    ax_c = f.add_axes([0.68, 0.085, 0.305, 0.40])
    pts = panel_c(ax_c, D)
    s.label(ax_a, "a", fig_coords=(0.003, 0.995))
    s.label(ax_b1, "b", fig_coords=(0.003, 0.52))
    s.label(ax_c, "c", fig_coords=(0.635, 0.52))
    m21 = next(x for x in rows if x["mission"] == 21)
    return dict(files=s.save(f, ctx["out"], "fig6_guidance_mishaps"), n_conv=n, mission21=m21, sep_points=len(pts))


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import make_all
    make_all.run_one(make)
