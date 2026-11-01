"""Bench: launch a VAB craft and fly it to a target orbit (own ascent guidance + own trim burns).
usage: fly_ascent.py CRAFT PE_KM AP_KM INC_DEG [AUTOSTAGE_LIMIT]"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot import ascent, core  # noqa: E402

craft, pe, ap, inc = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
limit = int(sys.argv[5]) if len(sys.argv) > 5 else 0
log = core.Log("ascent_" + craft.replace(" ", ""))
conn = core.connect("bench")
sc = conn.space_center
sc.launch_vessel("VAB", craft, "LaunchPad", [], True)
time.sleep(6)
from kspilot import feasibility  # noqa: E402
v = sc.active_vessel
for _ in range(10):
    try:
        v.delta_v
        break
    except Exception:
        time.sleep(1)
feas, rep, lines = feasibility.analyse(conn, v, pe * 1e3, ap * 1e3, inc, recover=False)
for l in lines:
    log(l)
if not feas and '--force' not in sys.argv:
    log('RESULT REFUSED (pre-launch check failed)')
    sys.exit(0)
ok = ascent.ascend(conn, pe * 1e3, ap * 1e3, inc, log, limit)
o = sc.active_vessel.orbit
log("RESULT", "PASS" if ok else "FAIL", "Ap=%.2fkm Pe=%.2fkm inc=%.3f" % (
    o.apoapsis_altitude / 1e3, o.periapsis_altitude / 1e3, o.inclination * 57.29578))
print("shot:", core.screenshot("ascent_" + craft.replace(" ", "")))
