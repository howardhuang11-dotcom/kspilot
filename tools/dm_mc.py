"""Theory experiment: airbrake drag modulation, bang-bang (as flown) vs continuous deploy fraction.

2-D point mass in Kerbin's vertical plane (polar, non-rotating) from the post-entry-burn state
(h 35 km, v_h 1507, v_z -321 m/s, m 16.9 t) to the end of drag modulation (1.5 km), with dispersions the
controller does not know: drag scale of each configuration, a density bias, the airbrake's deploy-fraction
-> drag curve (gamma), actuator lag, and the along-track distance to the target.
The controller is a copy of the flight logic: once a second it predicts the along-track miss with its
model (measured tables x online drag-scale estimate) and
  bang-bang : brakes OUT if the full-drag prediction is long (> -30 m), else IN  (landing.drag_modulation)
  continuous: deploy fraction f that zeroes the miss, linear between the f=0 and f=1 predictions
Metrics: along-track miss at 1.5 km, configuration switches, rms jerk of the drag deceleration.
usage: dm_mc.py [N]
"""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot import atmo  # noqa: E402
from kspilot.entry import DragTable  # noqa: E402

R, MU = 600000.0, 3.5316e12
d = json.load(open(os.path.join(os.path.dirname(__file__), "..", "logs", "g5_entry.json")))
T_OUT = DragTable(d["tab_out"]["alts"], d["tab_out"]["speeds"], d["tab_out"]["F"])
T_IN = DragTable(d["tab_in"]["alts"], d["tab_in"]["speeds"], d["tab_in"]["F"])
N_BRAKES = int(os.environ.get("DM_NBRAKES", 8))       # fewer airbrakes: out-drag interpolated from the 8-brake table
if N_BRAKES != 8:
    _T8 = T_OUT
    T_OUT = lambda h, v, mach=None: T_IN(h, v) + N_BRAKES / 8.0 * (_T8(h, v) - T_IN(h, v))   # noqa: E731
M = 16872.0
H_END = 1500.0


def step(s, a_drag_fn, dt):
    """s = [theta, r, vt, vr]; drag force callback(h, v) -> N."""
    th, r, vt, vr = s
    h = r - R
    v = math.hypot(vt, vr)
    D = a_drag_fn(h, v)
    a_t = -D / M * vt / v - vr * vt / r
    a_r = -D / M * vr / v - MU / r ** 2 + vt * vt / r
    vt += a_t * dt
    vr += a_r * dt
    r += vr * dt
    th += vt / r * dt
    return [th, r, vt, vr], D


PRED_DT = float(os.environ.get("DM_PRED_DT", 0.5))   # predictor step: 0.5 s is ~300 m short (a model error)


def predict(s, drag_fn, dt=None):
    """Downrange (m) where the model trajectory reaches H_END."""
    s = list(s)
    dt = dt or PRED_DT
    while s[1] - R > H_END:
        s, _ = step(s, drag_fn, dt)
    return s[0] * R


def run(scheme, p):
    s = [0.0, R + 35000.0, 1507.0, -321.0]
    target = p["target"]
    k_hat = {"out": 1.0, "in": 1.0, "blend": 1.0}
    f_cmd, f_act, brakes = 1.0, 1.0, True
    t, t_ctl, switches = 0.0, -1.0, 0
    acc_hist = []
    dt = 0.1

    def model(f, k):
        return lambda h, v: k * (T_IN(h, v) + f * (T_OUT(h, v) - T_IN(h, v)))

    while s[1] - R > H_END:
        h = s[1] - R
        # truth drag at the actual deploy state
        fe = f_act ** p["gamma"]
        truth = lambda hh, vv: p["rho_k"] * (p["k_in"] * T_IN(hh, vv) + fe * (p["k_out"] * T_OUT(hh, vv) - p["k_in"] * T_IN(hh, vv)))
        s_new, D = step(s, truth, dt)
        acc_hist.append(D / M)
        # online drag-scale estimate (measured drag / model drag at the commanded state)
        v = math.hypot(s[2], s[3])
        if scheme == "bang":
            key, fm = ("out", 1.0) if brakes else ("in", 0.0)
        else:
            key, fm = "blend", f_cmd
        Dm = T_IN(h, v) + fm * (T_OUT(h, v) - T_IN(h, v))
        if Dm > 500.0:
            k_hat[key] += 0.05 * (D / Dm - k_hat[key])
        s = s_new
        t += dt
        if t - t_ctl >= 1.0:
            t_ctl = t
            if scheme == "bang":
                c_out = predict(s, model(1.0, k_hat["out"])) - target
                want = c_out > -30.0
                if want != brakes:
                    switches += 1
                brakes = want
                f_cmd = 1.0 if brakes else 0.0
            else:
                k = k_hat["blend"]
                c1 = predict(s, model(1.0, k)) - target
                c0 = predict(s, model(0.0, k)) - target
                f_new = 1.0 if c0 <= c1 else min(1.0, max(0.0, c0 / (c0 - c1)))
                if abs(f_new - f_cmd) > 0.02:
                    switches += 1
                f_cmd = f_new
        # actuator: first-order lag
        f_act += (f_cmd - f_act) * min(1.0, dt / p["tau"])
    miss = s[0] * R - target
    jerk = np.diff(np.asarray(acc_hist)) / dt
    return miss, switches, float(np.sqrt(np.mean(jerk ** 2)))


def sample(rng, r_out_nom):
    return dict(k_out=rng.uniform(0.85, 1.15), k_in=rng.uniform(0.8, 1.6), rho_k=rng.uniform(0.95, 1.05),
                gamma=rng.uniform(0.6, 1.6), tau=rng.uniform(0.3, 1.5),
                target=r_out_nom + rng.uniform(1000.0, 15000.0))   # needs some brakes-in time (as planned)


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 60
    r_out_nom = predict([0.0, R + 35000.0, 1507.0, -321.0], lambda h, v: T_OUT(h, v), dt=0.1)
    rng = np.random.default_rng(1)
    ps = [sample(rng, r_out_nom) for _ in range(n)]
    res = {}
    for scheme in ("bang", "cont"):
        out = [run(scheme, p) for p in ps]
        miss = np.array([o[0] for o in out])
        res[scheme] = out
        print("%-5s  |miss| median %5.0f m  p90 %5.0f m  max %5.0f m  | switches median %3.0f | drag jerk rms %.2f m/s^3" % (
            scheme, np.median(abs(miss)), np.percentile(abs(miss), 90), abs(miss).max(),
            np.median([o[1] for o in out]), np.median([o[2] for o in out])))
    json.dump({k: [list(map(float, o)) for o in v] for k, v in res.items()},
              open(os.path.join(os.path.dirname(__file__), "..", "logs", "dm_mc.json"), "w"))


if __name__ == "__main__":
    main()
