"""Pre-launch feasibility report for a VAB craft (rolls it out, analyses, reverts).
usage: check.py CRAFT PE_KM AP_KM INC_DEG [norecover]"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot import core, feasibility  # noqa: E402

craft, pe, ap, inc = sys.argv[1], float(sys.argv[2]) * 1e3, float(sys.argv[3]) * 1e3, float(sys.argv[4])
recover = not (len(sys.argv) > 5 and sys.argv[5] == "norecover")
conn = core.connect("check")
sc = conn.space_center
sc.launch_vessel("VAB", craft, "LaunchPad", [], True)
time.sleep(6)
v = sc.active_vessel
for _ in range(10):  # stock delta-v needs a moment to be calculated (kRPC #1075)
    try:
        v.delta_v
        break
    except Exception:
        time.sleep(1)
ok, rep, lines = feasibility.analyse(conn, v, pe, ap, inc, recover, entry_burn_dv=590.0 if recover else 0.0)
print("== %s → Pe %.0fkm Ap %.0fkm i=%.1f° recover=%s ==" % (craft, pe / 1e3, ap / 1e3, inc, recover))
print("\n".join(lines))
if "--keep" not in sys.argv:
    sc.revert_to_launch()
    time.sleep(3)

