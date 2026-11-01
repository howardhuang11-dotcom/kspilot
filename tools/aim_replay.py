"""Offset-aim replay check (DR-012): does the landing guidance converge on an offset target as it does on the pad?

Closed loop in tools/replay_sim.py (GF0T model, validated on 12 flights, replay-simulator notes) with the REAL flight
code for the last kilometres: pdg.hoverslam_burn_now (ignition) and Lander.burn_law (landing burn), fed the state
in the TARGET frame built exactly like Lander.state() (origin = the target's terrain, radial height, radar ground)
and the commands mapped back to the simulator's pad ENU. The coast above ignition is engine-first along the airflow
(the glide is not modelled by the simulator).

Start: a logged GF0T state at H_START, carried rigidly about Kerbin's centre so that its unpowered impact point
(coast along the airflow, same model) moves to (target + disp): the guidance must divert `disp` metres. The same flight is run once to the pad and once
to the offset target; the two misses should agree.

usage: aim_replay.py [LANDING.csv] [--site flat2k] [--disp E,N]      (no game connection; one core, ~seconds)"""
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
import replay_sim as rs  # noqa: E402
from kspilot import aim, entry  # noqa: E402
from kspilot import powered_descent as pdg  # noqa: E402

H_START = 2500.0
DEFAULT_LOG = "20261020_203827_orbit_T-WorksFineGF0T_landing.csv"


def _rot(a, b):
    """Rotation matrix taking unit vector a to unit vector b (Rodrigues)."""
    a, b = rs.unit(a), rs.unit(b)
    v, c = np.cross(a, b), float(np.dot(a, b))
    s = float(np.linalg.norm(v))
    if s < 1e-15:
        return np.eye(3)
    k = v / s
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + s * K + (1 - c) * K @ K


class TargetFrame:
    """The lander's PadFrame + state() for a target at (lat, lon, alt), in the simulator's pad ENU."""

    def __init__(self, lat, lon, alt):
        Mp = np.vstack(entry.enu_basis(*entry.KSC_PAD))                     # bf -> pad ENU
        pad_bf = entry.latlonalt_to_bf(*entry.KSC_PAD, rs.PAD_ALT)
        self.O = Mp @ (entry.latlonalt_to_bf(lat, lon, alt) - pad_bf)          # target ground point, pad ENU
        self.M = np.vstack([Mp @ x for x in entry.enu_basis(lat, lon)])     # pad ENU -> target ENU
        self.r0 = float(np.linalg.norm(self.O + rs.CENTRE))

    def state(self, P, v):
        r = self.M @ (P - self.O)
        up = rs.unit(P + rs.CENTRE)
        r[2] = float(np.linalg.norm(P + rs.CENTRE)) - self.r0      # radial height above the target terrain
        vt = self.M @ v
        vt[2] = float(np.dot(v, up))
        return r, vt

    def to_pad(self, d):
        return self.M.T @ np.asarray(d, float)


def controller():
    """Lander with only the guidance state (the flight parameters of the removed terminal-descent Monte Carlo)."""
    from kspilot.landing import Lander
    from dataclasses import replace
    L = object.__new__(Lander)
    L.g = 9.81
    L.lag, L.kp_lat, L.kv_lat = 1.5, 0.2, 0.5
    L.v_final, L.thr_floor = 1.5, 0.12
    L.gate = np.array([0.0, 0.0, 4.0])
    L.hs = pdg.HoverslamParams(v_td=1.5, drag_frac=0.35, h_stop=25.0, throttle=0.65)
    L.terminal = replace(L.hs, h_stop=4.0, tilt_final_deg=6.0, kp_h=0.0, kv_h=0.5)
    return L


def fly_to(model, d, target_lla, disp=(0.0, 0.0), h_start=H_START):
    """Closed-loop landing on target_lla = (lat, lon, alt). Returns dict(miss, vz, vh, pad_dist, ign_h, fuel_kg)."""
    i = int(np.argmax(np.asarray(d["h"]) < h_start))
    s0 = rs.state_from_log(d, i)
    tf = TargetFrame(*target_lla)
    goal = tf.O + tf.M.T @ np.array([disp[0], disp[1], 0.0])                # ground point the flight is carried to
    td = rs.fly(model, s0, lambda st, t: st.air, h_stop=0.0, t_max=300.0)["final"].P   # unpowered impact point
    R = _rot(td + rs.CENTRE, goal + rs.CENTRE)
    s0 = rs.State(s0.t, R @ (s0.P + rs.CENTRE) - rs.CENTRE, R @ s0.v, R @ s0.nose, s0.m)
    L = controller()
    mem = {"burn": False, "ign_h": None, "cmd": (0.0, None)}

    def guide(st, t):
        r, vt = tf.state(st.P, st.v)
        L.h_gnd = r[2]                               # offset target: radar ground (flat site) = target height
        tmax = model.t_avail(st.alt)
        dmag = model.cda(st.mach, 0.0) * st.q
        if not mem["burn"]:
            far = float(np.linalg.norm(r[:2])) > 1000.0
            go, _ = pdg.hoverslam_burn_now(r[2], vt[2], st.m, tmax, L.g, model.cda(st.mach, 0.0), None, L.hs,
                                           None if far else r[:2], None if far else vt[:2])
            if go and vt[2] < 0:
                mem["burn"], mem["ign_h"] = True, r[2]
        if mem["burn"]:
            thr, dd = L.burn_law(r, vt, st.m, tmax, dmag)
            mem["cmd"] = (thr, tf.to_pad(dd))
        else:
            mem["cmd"] = (0.0, st.air)
        return mem["cmd"][1]

    def throttle(st, t):
        return mem["cmd"][0] if mem["burn"] else 0.0

    def stop(st):
        return tf.state(st.P, st.v)[0][2] <= 0.0

    run = rs.fly(model, s0, guide, throttle=throttle, stop=stop, h_stop=-500.0, t_max=200.0)
    f = run["final"]
    r, vt = tf.state(f.P, f.v)
    return dict(miss=float(np.hypot(r[0], r[1])), vz=float(vt[2]), vh=float(np.hypot(vt[0], vt[1])),
                pad_dist=float(np.hypot(f.P[0], f.P[1])), ign_h=mem["ign_h"], fuel_kg=float(s0.m - f.m),
                start_offset=float(np.hypot(*tf.state(s0.P, s0.v)[0][:2])))


def check(path, site="flat2k", disp=(150.0, 50.0)):
    """Two replays of one flight: to the pad and to the offset site, each with the same `disp` to divert."""
    model, d = rs.default_model(), rs.load(path)
    t = aim.from_env({"KSP_AIM_SITE": site})
    pad = fly_to(model, d, (*entry.KSC_PAD, rs.PAD_ALT), disp)
    off = fly_to(model, d, (t.lat, t.lon, aim.SITES[site]["h_exp"]), disp)
    return pad, off


def main(argv):
    import argparse
    ap = argparse.ArgumentParser(prog="aim_replay.py", description="Offset-aim replay check (DR-012); no game "
                                 "connection. Needs a GF0T landing CSV (default: %s; searched like replay_sim.py: "
                                 "--logs, KSP_REPLAY_LOGS, data/raw/logs, logs, tests/data)." % DEFAULT_LOG)
    ap.add_argument("csv", nargs="?", help="landing CSV (default: the bundled flight)")
    ap.add_argument("--site", default="flat2k")
    ap.add_argument("--disp", default="150,50", help="E,N displacement in m (default 150,50)")
    ap.add_argument("--logs", metavar="DIR", help="directory with the landing CSVs")
    a = ap.parse_args(argv[1:])
    if a.logs:
        os.environ["KSP_REPLAY_LOGS"] = a.logs
    try:
        path = rs.find_log(a.csv or DEFAULT_LOG)
    except rs.NeedData as e:
        print("aim_replay.py: %s" % e, file=sys.stderr)
        return 2
    site, disp = a.site, tuple(float(x) for x in a.disp.split(","))
    pad, off = check(path, site, disp)
    for name, r in (("pad", pad), (site, off)):
        print("%-7s start %.0f m off, ignition h %.0f m: miss %.2f m, vz %.2f m/s, vh %.2f m/s, %.0f m from the pad, "
              "burn %.0f kg" % (name, r["start_offset"], r["ign_h"] or float("nan"), r["miss"], r["vz"], r["vh"],
                               r["pad_dist"], r["fuel_kg"]))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
