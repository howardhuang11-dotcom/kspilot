"""Take over the active vessel (already falling toward the target) with the landing controller.
usage: land_now.py [H_COM]"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot import core  # noqa: E402
from kspilot.landing import Lander  # noqa: E402

log = core.Log("land_now")
conn = core.connect("land_now")
v = conn.space_center.active_vessel
h_com = float(sys.argv[1]) if len(sys.argv) > 1 else None
lander = Lander(conn, v, log=log, overlay_tag="G5 land_now", h_com=h_com)
res = lander.run(timeout=1500)
ok = res['situation'].endswith('landed') and res['miss'] < 20 and res['v_touchdown'] < 4 and res['upright'] > 0.95
log("RESULT %s miss=%.1fm v_td=%.2fm/s upright=%.3f fuel_left=%.0f parts=%d %s" % (
    "PASS" if ok else "FAIL", res['miss'], res['v_touchdown'], res['upright'], res['fuel_left'], res['parts'],
    res['situation']))
print("shot:", core.screenshot("land_now"))
