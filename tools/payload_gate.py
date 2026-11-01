"""Offline feasibility gate for the T-WorksFineGF payload variants (2.48 / 4.5 / 6.0 t), old vs new recovery budget.

The stock stage data are the ones the gate printed before the 02:38 flight (logs/20261021_023833_orbit_T-WorksFineGF.txt):
  stage 3: dv 888 asl / 1045 vac, 92.7 -> 59.3 t, 42 s; stage 2: 4219 vac, 53.1 -> 13.8 t, 190 s
  ascent need (100x100 km, i 6) 3434 m/s + low-TWR penalty 186 = 3620; recovery need 1064 m/s; core dry 11.4 t.
A heavier payload adds the same mass to every stage mass; the stage exhaust velocities, thrusts and burn times are
unchanged (same engines, same propellant); vacuum TWRs come from the stock thrusts (Skipper 650 kN vac, 4 Thumpers
300 kN vac each), which reproduce the logged +186 m/s penalty. The core after release is identical, so need_rec is
unchanged. Flown 2.48 t reference (the gate prints 13.8 - 11.4 = 2.4) (5 missions): ~918 LF in orbit, 370-377 LF left after landing.
usage: payload_gate.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot.feasibility import LFO_T_PER_LF, budget, gravity_penalty  # noqa: E402

G = 9.81
P0 = 2480.0                 # payload above the decoupler, from the stock part cfgs (tools/craft_ballast.py check)
NEED_ASC0 = 3434.0          # m/s before the low-TWR penalty (3620 - 186 as logged)
NEED_REC = 1064.0           # m/s, as logged (deorbit + 590 entry burn + landing, x REC_MARGIN)
CORE_DRY = 11360.0          # core incl. the payload decoupler (it stays on the core), cfg masses; gate printed 11.4
T_VAC = (650e3 + 4 * 300e3, 650e3)
LOGGED = [dict(n=3, dv_asl=888.0, dv_vac=1045.0, m0=92700.0, m1=59300.0, burn=42.0),
          dict(n=2, dv_asl=3699.0, dv_vac=4219.0, m0=53100.0, m1=13800.0, burn=190.0)]
EXACT = [(92660.0, 59300.0), (53100.0, 13840.0)]      # cfg-derived where the log rounds (lift-off, stage-2 end)
FLOWN = dict(payload=2480.0, lf_orbit=918.0, lf_left=(370.0, 377.0))


def stages_for(payload):
    """Stock-calculator stage list for the same vehicle with a different payload mass (kg above the decoupler)."""
    d = payload - P0
    out = []
    for s, t, (e0, e1) in zip(LOGGED, T_VAC, EXACT):
        ln0 = math.log(s['m0'] / s['m1'])
        ve_vac, ve_asl = s['dv_vac'] / ln0, s['dv_asl'] / ln0
        m0, m1 = e0 + d, e1 + d
        dln = math.log(m0 / m1) - math.log(e0 / e1)    # baseline reproduces the logged dv exactly
        out.append(dict(s, m0=m0, m1=m1, dv_asl=s['dv_asl'] + ve_asl * dln, dv_vac=s['dv_vac'] + ve_vac * dln,
                        twr_vac=t / (m0 * G)))
    return out


def gate(payload):
    st = stages_for(payload)
    need_asc = NEED_ASC0 + gravity_penalty(st, NEED_ASC0, G)
    total = sum(s['dv_asl'] if i == 0 else s['dv_vac'] for i, s in enumerate(st))
    old = total - need_asc - NEED_REC                      # previous gate: recovery dv with the payload attached
    new = budget(st, need_asc, CORE_DRY, NEED_REC)
    return dict(payload=payload, need_asc=need_asc, total=total, old=old, new=new)


def main():
    print("payload | ascent need | old gate | new gate (ascent-side / core-side) | core prop in orbit | left after landing")
    for p in (P0, 4500.0, 6000.0):
        r = gate(p)
        b = r['new']
        print(" %.2f t |  %4.0f m/s   | %+5.0f %s | %+5.0f / %+5.0f m/s %s       | %5.2f t (%4.0f LF)  | %5.2f t (%4.0f LF)" % (
            p / 1e3, r['need_asc'], r['old'], "GO   " if r['old'] >= 0 else "NO-GO", b['margin'], b['margin_rec'],
            "GO   " if b['margin'] >= 0 else "NO-GO", b['prop_orbit'] / 1e3, b['prop_orbit'] / 1e3 / LFO_T_PER_LF,
            b['prop_spare'] / 1e3, b['prop_spare'] / 1e3 / LFO_T_PER_LF))
    print("flown 2.48 t: %.0f LF in orbit, %.0f-%.0f LF left after landing" % (
        FLOWN['lf_orbit'], *FLOWN['lf_left']))


if __name__ == "__main__":
    main()
