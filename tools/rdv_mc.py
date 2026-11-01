"""Offline Monte Carlo of the terminal rendezvous + docking (R0 of docking concept study), no KSP needed.

Each run starts at the Hohmann arrival point ~1 km behind the target (dispersed) and flies kspilot.rdv.docking
(CW transfer -> 50 m hold -> approach -> 10 m hold -> final) against the dispersed plant. Reports success rate
(Wilson 95 % interval), failure modes, RCS dv, monopropellant, time to dock and contact conditions.

usage: rdv_mc.py [N] [--campaign NAME[,NAME...]|all] [--seed S] [--procs P]
results -> logs/rdv_mc_<campaign>.json
"""
import argparse
import json
import math
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from kspilot.rdv import docking as dk  # noqa: E402


def _campaign(name, rng):
    u = rng.uniform
    if name == "vbar":            # nominal: V-bar approach, RCS ring near the stack CoM, proportional RCS
        return dk.sample(rng, approach="vbar")
    if name == "hbar":            # H-bar (orbit-normal port; target needs only inertial SAS hold)
        return dk.sample(rng, approach="hbar")
    if name == "vbar_lat05":      # worst kRPC latency: 0.5 s total, poor latency estimate
        return dk.sample(rng, approach="vbar", tau_meas=0.25, tau_cmd=0.25, tau_est_scale=u(0.5, 1.5),
                         dt_ctrl=0.25)
    if name == "vbar_nocomp":     # latency compensation off
        return dk.sample(rng, approach="vbar", latency_comp=False)
    if name == "vbar_onoff":      # on/off thrusters (no proportional input), 25 % thrust limiter
        return dk.sample(rng, approach="vbar", onoff=True, f_lat=500.0, f_ax=1000.0)
    if name == "vbar_onoff_full":  # on/off thrusters at full thrust (no limiter): expected to fail the holds
        return dk.sample(rng, approach="vbar", onoff=True)
    if name == "vbar_cargo_rcs":  # all RCS on the cargo module: thrust centre 7-10 m ahead of the CoM
        p = dk.sample(rng, approach="vbar")
        return dk.replace(p, cp_offset=(p.cp_offset[0], p.cp_offset[1], u(7.0, 10.0)))
    if name == "vbar_nav5":       # 2.5x navigation noise and 3x bias
        return dk.sample(rng, approach="vbar", nav_sig_p=0.05, nav_sig_v=0.0125,
                         nav_bias_p=tuple(rng.normal(0, 0.03, 3)))
    raise SystemExit("unknown campaign " + name)


CAMPAIGNS = ["vbar", "hbar", "vbar_lat05", "vbar_nocomp", "vbar_onoff", "vbar_onoff_full", "vbar_cargo_rcs",
             "vbar_nav5"]


def _run(p):
    r = dk.simulate(p)
    return dict(success=r.success, mode=r.mode, t=r.t, mono=r.mono_kg, dv=r.dv, v_axial=r.v_axial,
                lateral=r.lateral, angle=r.angle_deg, v_lateral=r.v_lateral, retries=r.retries,
                mass=p.mass, cp_z=p.cp_offset[2], latency=p.tau_meas + p.tau_cmd, u_min=p.u_min)


def wilson(k, n, z=1.96):
    if n == 0:
        return (math.nan, math.nan)
    ph = k / n
    d = 1 + z * z / n
    c = (ph + z * z / (2 * n)) / d
    h = z * math.sqrt(ph * (1 - ph) / n + z * z / (4 * n * n)) / d
    return (c - h, c + h)


def pct(x, q):
    x = [v for v in x if v == v]
    return float(np.percentile(x, q)) if x else math.nan


def summarise(name, rows, n):
    k = sum(r["success"] for r in rows)
    modes = {}
    for r in rows:
        modes[r["mode"]] = modes.get(r["mode"], 0) + 1
    ok = [r for r in rows if r["success"]]
    s = dict(campaign=name, n=n, success=k, rate=k / n, ci95=wilson(k, n), modes=modes,
             mono_p50=pct([r["mono"] for r in rows], 50), mono_p95=pct([r["mono"] for r in rows], 95),
             mono_p99=pct([r["mono"] for r in rows], 99), mono_max=max(r["mono"] for r in rows),
             dv_p50=pct([r["dv"] for r in rows], 50), dv_p95=pct([r["dv"] for r in rows], 95),
             t_p50=pct([r["t"] for r in ok], 50), t_p95=pct([r["t"] for r in ok], 95),
             vax_p50=pct([r["v_axial"] for r in ok], 50), vax_max=pct([r["v_axial"] for r in ok], 100),
             lat_p50=pct([r["lateral"] for r in ok], 50), lat_p95=pct([r["lateral"] for r in ok], 95),
             lat_p99=pct([r["lateral"] for r in ok], 99),
             ang_p95=pct([r["angle"] for r in ok], 95), ang_max=pct([r["angle"] for r in ok], 100),
             retries_frac=sum(r["retries"] > 0 for r in rows) / n)
    return s


def fmt(s):
    lo, hi = s["ci95"]
    return ("%-16s N=%d success %.1f %% [%.1f, %.1f]  modes %s\n"
            "                 mono kg p50 %.1f p95 %.1f p99 %.1f max %.1f | RCS dv p50 %.2f p95 %.2f m/s | "
            "time p50 %.0f p95 %.0f s\n"
            "                 contact: v_ax p50 %.3f max %.3f m/s, lateral p50 %.3f p95 %.3f p99 %.3f m, "
            "angle p95 %.2f max %.2f deg, runs with retries %.1f %%" % (
                s["campaign"], s["n"], 100 * s["rate"], 100 * lo, 100 * hi, s["modes"],
                s["mono_p50"], s["mono_p95"], s["mono_p99"], s["mono_max"], s["dv_p50"], s["dv_p95"],
                s["t_p50"], s["t_p95"], s["vax_p50"], s["vax_max"], s["lat_p50"], s["lat_p95"], s["lat_p99"],
                s["ang_p95"], s["ang_max"], 100 * s["retries_frac"]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("n", nargs="?", type=int, default=500)
    ap.add_argument("--campaign", default="vbar")
    ap.add_argument("--seed", type=int, default=20261021)
    ap.add_argument("--procs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    a = ap.parse_args()
    names = CAMPAIGNS if a.campaign == "all" else a.campaign.split(",")
    os.makedirs(os.path.join(ROOT, "logs"), exist_ok=True)
    for name in names:
        rng = np.random.default_rng([a.seed, CAMPAIGNS.index(name) if name in CAMPAIGNS else 99])
        params = [_campaign(name, rng) for _ in range(a.n)]
        t0 = time.time()
        with Pool(a.procs) as pool:
            rows = pool.map(_run, params, chunksize=4)
        s = summarise(name, rows, a.n)
        s["wall_s"] = time.time() - t0
        s["seed"] = a.seed
        print(fmt(s), flush=True)
        with open(os.path.join(ROOT, "logs", "rdv_mc_%s.json" % name), "w") as fh:
            json.dump(dict(summary=s, runs=rows), fh, indent=0)


if __name__ == "__main__":
    main()
