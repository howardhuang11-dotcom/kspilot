"""Theory experiment: payload to a 100 km orbit vs propellant kept for the core's return (T-WorksFine family).

Calibrated on the 2026-10-20 flights instead of textbook numbers:
  stage 3 (core + 4 SRBs, 42 s): 92.7 -> 59.3 t, 1045 m/s vacuum-equivalent, thrust 1573 kN
  SRB casings + decouplers dropped: 59.3 -> 53.1 t;  stage 2 (core): Skipper 648 kN vac, ve 3131 m/s
  payload 2.4 t (13.8 - 11.4 t core dry); own ascent reached orbit with 917 LF units = 10.19 t propellant
  recovery from that orbit (deorbit + entry burn + landing) used 532 LF units = 5.91 t (12:41 flight)
The ascent's vacuum-equivalent dv actually spent (incl. its gravity and drag losses) is the requirement;
a heavier vehicle pays extra gravity loss through the low-TWR term of kspilot.feasibility.
usage: payload_curve.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot.feasibility import TWR_REF  # noqa: E402

G = 9.81
M0, M3_END, M2_START = 92.7, 59.3, 53.1          # t, as flown with P0
P0, CORE_DRY = 2.4, 11.4
VE3, VE2 = 1045.0 / math.log(92.7 / 59.3), 3131.0
T3, T2 = 1573.0, 648.0                            # kN
PROP_ORBIT0 = 10.19                               # t of core propellant left in orbit (flown)
RECOVERY = 5.91                                   # t used by the whole return (flown)
LF_T = 5 * (1 + 1.222) / 1000.0                   # t of propellant per LiquidFuel unit


def ascent_dv(P, prop_orbit):
    """Vacuum-equivalent dv and the low-TWR gravity penalty for payload P reaching orbit with prop_orbit."""
    m0 = M0 - P0 + P
    m3 = M3_END - P0 + P
    m2 = M2_START - P0 + P
    m_orb = CORE_DRY + prop_orbit + P
    dv3 = VE3 * math.log(m0 / m3)
    dv2 = VE2 * math.log(m2 / m_orb)
    # low-TWR gravity penalty (vacuum TWR, burn time of the part used)
    burn2 = (m2 - m_orb) * 1000.0 / (T2 * 1000.0 / VE2)
    twr3 = T3 / (m0 * G) * 1.0
    twr2 = T2 / (0.5 * (m2 + m_orb) * G)
    pen = G * 42.0 * max(0.0, 1 / twr3 - 1 / TWR_REF) + G * burn2 * max(0.0, 1 / twr2 - 1 / TWR_REF)
    return dv3 + dv2, pen, T3 / (m0 * G), T2 / (m2 * G)


def main():
    dv_ref, pen_ref, *_ = ascent_dv(P0, PROP_ORBIT0)
    need = dv_ref - pen_ref                           # loss-free share; penalties re-added per payload
    print("calibration: flown ascent %.0f m/s vac-eq (low-TWR penalty %.0f m/s)" % (dv_ref, pen_ref))
    print()
    print("reserve kept after landing | max payload | liftoff TWR | stage-2 TWR | core prop in orbit")
    for reserve in (0.0, 0.5, 1.0, 1.5, 2.5):
        prop = RECOVERY + reserve
        lo, hi = 0.0, 30.0
        for _ in range(60):                           # bisection on payload
            P = 0.5 * (lo + hi)
            dv, pen, tw0, tw2 = ascent_dv(P, prop)
            if dv - pen >= need:
                lo = P
            else:
                hi = P
        dv, pen, tw0, tw2 = ascent_dv(lo, prop)
        print("   %4.1f t (%4.0f LF units)    |   %5.2f t   |    %.2f     |    %.2f     |  %.2f t" % (
            reserve, reserve / LF_T, lo, tw0 * 9.81 / G, tw2, prop))
    print()
    print("flown: payload %.1f t, reserve after landing %.2f t (385 LF units)" % (P0, PROP_ORBIT0 - RECOVERY))


if __name__ == "__main__":
    main()
