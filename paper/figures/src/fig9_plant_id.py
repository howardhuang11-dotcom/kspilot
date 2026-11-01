"""Fig. 9 | Landing-burn attitude plant: commanded against achieved tilt, the identified authority envelope
and validation of the simulated plant (identification and replay: GNC2 burn_attitude_id.json, divert_measured_mc.json;
flight data: landing CSVs)."""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import data as d  # noqa: E402
import style as s  # noqa: E402

EXAMPLE = "20261021_000635"


def angles(T):
    """Per BURN row: time from ignition (s), commanded tilt from vertical, achieved tilt from vertical, achieved angle
    off the airflow axis (-v), throttle, q (kPa), height (m)."""
    idx = [i for i, p in enumerate(T["phase"]) if p == "BURN"]
    ut = np.array(T["ut"])[idx]
    dz = np.clip(np.array(T["dz"])[idx], -1, 1)
    nose = np.array([T["ne"], T["nn"], T["nu"]]).T[idx]
    v = np.array([T["ve"], T["vn"], T["vu"]]).T[idx]
    vhat = -v / np.maximum(np.linalg.norm(v, axis=1)[:, None], 1e-6)
    nhat = nose / np.linalg.norm(nose, axis=1)[:, None]
    off_air = np.degrees(np.arccos(np.clip((nhat * vhat).sum(1), -1, 1)))
    return dict(t=ut - ut[0], cmd=np.degrees(np.arccos(dz)), ach=np.degrees(np.arccos(np.clip(nhat[:, 2], -1, 1))),
                off_air=off_air, thr=np.array(T["thr_act"])[idx], q=np.array(T["q"])[idx] / 1e3,
                h=np.array(T["h"])[idx], speed=np.array(T["speed"])[idx])


def make(ctx):
    s.setup()
    G = ctx["gnc2"]
    ID = json.load(open(os.path.join(G, "burn_attitude_id.json")))
    MC = json.load(open(os.path.join(G, "divert_measured_mc.json")))
    K = ID["pooled"]["K"]
    f = s.fig(s.W2, 62)
    # a: example burn
    T = d.telemetry(ctx["logs"], EXAMPLE + "_orbit_T-WorksFineGF")
    A = angles(T)
    ax = f.add_axes([0.06, 0.17, 0.30, 0.76])
    env = np.minimum(K * A["thr"] / np.maximum(A["q"], 1e-3), 40)
    ax.plot(A["t"], A["cmd"], color="k", lw=0.8, label="commanded tilt")
    ax.plot(A["t"], A["ach"], color=s.C_CAMP, lw=0.8, label="achieved tilt")
    ax.plot(A["t"], env, color=s.C_SIM, lw=0.7, ls=(0, (3, 1.5)), label="identified envelope K·throttle/q")
    ax.set_ylim(0, 30)
    ax.set_xlim(0, A["t"][-1])
    ax.set_xlabel("Time from landing-burn ignition (s)")
    ax.set_ylabel("Angle (°)")
    ax.legend(loc="upper right", bbox_to_anchor=(1.0, 0.86), fontsize=5)
    ax2 = ax.twinx()
    ax2.plot(A["t"], A["speed"], color="0.6", lw=0.5)
    ax2.set_ylim(0, 130)
    ax2.set_ylabel("Speed (m s$^{-1}$, grey)", color="0.45")
    ax2.spines["right"].set_visible(True)
    ax2.tick_params(colors="0.45")
    ax.text(0.02, 0.97, "flight %s\nignition %.0f m, offset %.0f m" % (EXAMPLE, A["h"][0],
            math.hypot(T["e"][T["phase"].index("BURN")], T["n"][T["phase"].index("BURN")])),
            transform=ax.transAxes, fontsize=5, va="top")
    # b: all identification burns, achieved angle off the airflow vs envelope
    axb = f.add_axes([0.45, 0.17, 0.22, 0.76])
    for pf in ID["per_flight"]:
        T = d.telemetry(ctx["logs"], pf["flight"] + "_orbit_T-WorksFineGF")
        A = angles(T)
        ok = (A["q"] > 0.3) & (A["thr"] > 0.05)
        axb.scatter(np.minimum(K * A["thr"][ok] / A["q"][ok], 30), A["off_air"][ok], s=1.2, color=s.C_CAMP,
                    alpha=0.35, lw=0)
    axb.plot([0, 30], [0, 30], color=s.C_SIM, lw=0.7, ls=(0, (3, 1.5)))
    axb.set_xlim(0, 30)
    axb.set_ylim(0, 30)
    axb.set_aspect("equal")
    axb.set_xlabel("Envelope K·throttle/q (°), K = %.1f" % K)
    axb.set_ylabel("Achieved angle off the airflow axis (°)")
    axb.text(29, 1, "%d burns, rows with q > 0.3 kPa" % len(ID["per_flight"]), fontsize=5, ha="right")
    # c: validation of the simulated plant
    axc = f.add_axes([0.76, 0.17, 0.22, 0.76])
    V = MC["validation"]
    big = [v for v in V if v["off"] > 100]
    gate = [v for v in V if v["off"] <= 100]
    axc.scatter([v["closed_flight"] for v in gate], [v["closed_sim"] for v in gate], s=10, color=s.C_GATE,
                label="gate, 45 m (n = %d)" % len(gate), zorder=3)
    axc.scatter([v["closed_flight"] for v in big], [v["closed_sim"] for v in big], s=12, marker="D", color=s.C_SIM,
                label="173–481 m (n = %d)" % len(big), zorder=3)
    axc.plot([0, 140], [0, 140], color="0.5", lw=0.5, ls=(0, (2, 2)))
    axc.set_xlim(0, 140)
    axc.set_ylim(0, 140)
    axc.set_aspect("equal")
    axc.set_xlabel("Offset closed in flight (m)")
    axc.set_ylabel("Offset closed in simulation (m)")
    axc.legend(loc="upper left", fontsize=5, handletextpad=0.2)
    s.sim_tag(axc, "simulation vs flight", loc="lower right")
    for a_, l, x in ((ax, "a", 0.003), (axb, "b", 0.395), (axc, "c", 0.70)):
        s.label(a_, l, fig_coords=(x, 0.99))
    return dict(files=s.save(f, ctx["out"], "fig9_plant_id"), K=K, tau=ID["pooled"]["tau"])


if __name__ == "__main__":
    import _main
    _main.main(make)
