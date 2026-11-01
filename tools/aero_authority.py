"""Theory experiment: along-track range authority of the core's descent, airbrakes vs body lift.

2-D point mass in Kerbin's vertical plane (polar, non-rotating), from the post-entry-burn state of the
2026-10-20 12:41 full mission (h 35 km, v_h 1507, v_z -321 m/s, m 16.9 t) down to 3 km.
Drag: measured tables logs/g5_entry.json (brakes out / in; 'in' also folds the grid fins).
Lift: F = CNaS * q * beta, perpendicular to the velocity (+ = up = longer range). CNaS = 9.2 m^2
measured at ~100 m/s (fixed-fin flights, tools output 2026-10-20); its value at Mach 2-4 is unknown,
so it is swept.
usage: aero_authority.py
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


def fly(drag, k_drag=1.0, cnas=0.0, beta_deg=0.0, h0=35000.0, vh=1507.0, vz=-321.0, m=16872.0,
        h_end=3000.0, dt=0.05, lift_below=1e9):
    """Returns (downrange m, speed at h_end, max q, max lateral g)."""
    th, r = 0.0, R + h0
    vt, vr = vh, vz
    qmax = 0.0
    while r - R > h_end:
        h = r - R
        v = math.hypot(vt, vr)
        rho = atmo.density(h)
        q = 0.5 * rho * v * v
        qmax = max(qmax, q)
        D = k_drag * drag(h, v)
        ut, ur = vt / v, vr / v
        a_t, a_r = -D / m * ut, -D / m * ur
        if h < lift_below:
            L = cnas * q * math.radians(beta_deg)
            a_t += L / m * (-ur)          # perpendicular, rotated "up"
            a_r += L / m * ut
        g = MU / r ** 2
        a_r += -g + vt * vt / r           # gravity + centripetal (polar)
        a_t += -vr * vt / r
        vt += a_t * dt
        vr += a_r * dt
        r += vr * dt
        th += vt / r * dt
    return th * R, math.hypot(vt, vr), qmax


def main():
    base, v_b, q_b = fly(T_OUT)
    print("brakes OUT (today's nominal):  range %.1f km  v(3 km) %.0f m/s  max q %.1f kPa" % (base / 1e3, v_b, q_b / 1e3))
    r_in, v_in, q_in = fly(T_IN)
    print("brakes IN:                      range %+.2f km vs nominal  v(3 km) %.0f m/s  max q %.1f kPa" % ((r_in - base) / 1e3, v_in, q_in / 1e3))
    print("  -> airbrake drag-modulation authority (all-in vs all-out): %.2f km" % ((r_in - base) / 1e3))
    print()
    print("fins only (no airbrakes): drag = brakes-IN table x k_f (deployed fins add drag)")
    for k_f in (1.0, 1.4):
        r0, v0, q0 = fly(T_IN, k_f)
        print(" k_f %.1f: range %+.2f km vs nominal, v(3 km) %.0f m/s, max q %.1f kPa" % (k_f, (r0 - base) / 1e3, v0, q0 / 1e3))
        for cn in (9.2, 4.6, 2.3):
            for b in (8.0, 15.0):
                rp = fly(T_IN, k_f, cn, b)[0]
                rm = fly(T_IN, k_f, cn, -b)[0]
                print("   CNaS %4.1f m^2, |beta| %2.0f deg: lift up %+6.2f km / down %+6.2f km  -> authority %.2f km" % (
                    cn, b, (rp - r0) / 1e3, (rm - r0) / 1e3, (rp - rm) / 1e3))


if __name__ == "__main__":
    main()


def entry_burn_needed():
    """Extra entry-burn dv (retrograde, at the 35 km state) for a fins-only core to keep max q down."""
    print()
    print("fins only: extra entry-burn dv (applied at 35 km) vs max dynamic pressure")
    vh, vz = 1507.0, -321.0
    v = math.hypot(vh, vz)
    for k_f in (1.0, 1.4):
        for dv in (0, 200, 400, 600, 800):
            s = (v - dv) / v
            rng, v3, qm = fly(T_IN, k_f, vh=vh * s, vz=vz * s)
            # propellant for that dv at the start mass (Skipper vacuum-ish Isp 300 s)
            prop = 16872.0 * (1 - math.exp(-dv / (300 * 9.81)))
            print("  k_f %.1f  +%3d m/s: max q %5.1f kPa  v(3 km) %3.0f m/s  extra propellant %.2f t" % (
                k_f, dv, qm / 1e3, v3, prop / 1e3))


if __name__ == "__main__":
    entry_burn_needed()
