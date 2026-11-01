"""Supplementary Fig. 5 | R1 rendezvous to a 1 km V-bar hold (flight after the data freeze): manoeuvre sequence, free drift
in the target LVLH frame and Clohessy-Wiltshire prediction error (rdv_r1_rdv_*.json, wd_20261022_012428.out)."""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import style as s  # noqa: E402


RECORD = "rdv_r1_rdv_20261022_014347.json"      # the flight of wd_20261022_012428.out (later rdv_r1_rdv_* files are other runs)


def make(ctx):
    s.setup()
    R = json.load(open(os.path.join(ctx["logs"], RECORD)))
    rows = R["drift"]["rows"]
    t = np.array([r[0] for r in rows])
    meas = np.array([r[1] for r in rows])
    cw = np.array([r[2] for r in rows])
    err = np.linalg.norm(meas - cw, axis=1)
    # short excursions: samples whose error exceeds twice the running median (window 15 samples, ~30 s) plus 1 m
    pad = np.pad(err, 7, mode="edge")
    med = np.array([np.median(pad[i:i + 15]) for i in range(len(err))])
    spike = err > 2 * med + 1.0
    good = ~spike
    f = s.fig(s.W2, 58)
    # a: burns
    ax = f.add_axes([0.06, 0.2, 0.22, 0.72])
    names = [b[0].replace(" (match the hold point)", "\n(match hold)").replace(" (Hohmann)", "\n(Hohmann)")
             .replace(" (Lambert)", "\n(Lambert)") for b in R["burns"]]
    dv = [b[2] for b in R["burns"]]
    ax.bar(range(len(dv)), dv, color=[s.OI["grey"], s.C_CAMP, s.OI["orange"], s.C_CAMP], width=0.6)
    for i, v in enumerate(dv):
        ax.text(i, v + 0.3, "%.2f" % v, ha="center", fontsize=5)
    ax.set_xticks(range(len(dv)))
    ax.set_xticklabels(names, fontsize=5)
    ax.set_ylabel("Δv (m s$^{-1}$)")
    ax.set_ylim(0, 24)
    g = {x[0]: x for x in R["gates"]}
    ax.text(0.5, 0.985, "total %.1f m s$^{-1}$ (plan %.1f)\nhold error 22 m, |v$_{rel}$| 0.12 m s$^{-1}$" % (
        R["dv_used"], R["dv_plan"]), transform=ax.transAxes, fontsize=5, ha="center", va="top")
    # b: LVLH drift
    axb = f.add_axes([0.37, 0.2, 0.27, 0.72])
    axb.plot(meas[good, 1], meas[good, 0], color=s.C_CAMP, lw=1.0, label="measured")
    axb.plot(meas[spike, 1], meas[spike, 0], "x", color=s.C_CAMP, ms=2.5, mew=0.5,
             label="short excursions (n = %d samples)" % spike.sum())
    axb.plot(cw[:, 1], cw[:, 0], color=s.C_SIM, lw=0.8, ls=(0, (3, 1.5)), label="Clohessy–Wiltshire prediction")
    axb.plot(meas[0, 1], meas[0, 0], "o", color="k", ms=3)
    axb.text(meas[0, 1] - 4, meas[0, 0], "start of free drift", fontsize=5, va="center")
    axb.set_xlabel("Along-track, V-bar (m)")
    axb.set_ylabel("Radial, R-bar (m)")
    axb.legend(loc="lower right", fontsize=5)
    axb.invert_xaxis()
    # c: CW error vs time
    axc = f.add_axes([0.73, 0.2, 0.26, 0.72])
    axc.plot(t[good], err[good], color="k", lw=0.8, label="smooth samples")
    axc.plot(t[spike], err[spike], "x", color=s.C_FAIL, ms=3, mew=0.6, label="short excursions")
    axc.legend(loc="upper left", bbox_to_anchor=(0.0, 0.76), fontsize=5)
    axc.axhline(10, color=s.C_FAIL, lw=0.5, ls=(0, (3, 2)))
    axc.text(595, 11, "drift gate 10 m", fontsize=5, color=s.C_FAIL, va="bottom", ha="right")
    axc.set_yscale("log")
    axc.set_xlabel("Free-drift time (s)")
    axc.set_ylabel("|measured − predicted| (m)")
    axc.set_xlim(0, t[-1])
    s.sim_tag(axc, "model vs flight", loc="lower right")
    for a_, l, x in ((ax, "a", 0.003), (axb, "b", 0.315), (axc, "c", 0.675)):
        s.label(a_, l, fig_coords=(x, 0.99))
    i_max = int(np.argmax(err))
    return dict(files=s.save(f, ctx["out"], "sfig5_rendezvous"), n_rows=len(rows), max_err=float(err.max()),
                t_max=float(t[i_max]), err_at=[(float(tt), float(e)) for tt, e in zip(t[::60], err[::60])],
                logged_max=R["drift"]["max_err"], n_spike=int(spike.sum()), max_smooth=float(err[good].max()))


if __name__ == "__main__":
    import _main
    _main.main(make)
