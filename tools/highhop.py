"""G3 bench (F9R-Dev style): boost to an apogee, cut off, fall back and land on the pad
with the coast / hoverslam / G-FOLD landing controller.
usage: highhop.py CRAFT APOGEE_M [PITCH_DEG] [HEADING_DEG] [hslam]   (env HOP_REC=1 records video)"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot import core  # noqa: E402
from kspilot.landing import Lander  # noqa: E402

craft = sys.argv[1]
apogee = float(sys.argv[2])
pitch = float(sys.argv[3]) if len(sys.argv) > 3 else 86.0
heading = float(sys.argv[4]) if len(sys.argv) > 4 else 90.0
use_gfold = "hslam" not in sys.argv
tag = "highhop_%s_%dm" % (craft.replace(" ", ""), apogee)
log = core.Log(tag)
conn = core.connect("highhop")
sc = conn.space_center
sc.launch_vessel("VAB", craft, "LaunchPad", [], True)
time.sleep(6)
v = sc.active_vessel
if os.environ.get("HOP_REC"):
    core.set_daylight(conn, 20.0, log=log)
if os.environ.get("HOP_REC"):
    core.set_camera(conn, 70.0, 5.0)
frames = core.record_start(tag) if os.environ.get("HOP_REC") else None
from kspilot.landing import com_height  # noqa: E402
v.control.gear = True
time.sleep(1)
h_com = com_height(v)   # legs down on the pad
log("CoM height above leg feet: %.2f m" % h_com)
ap = v.auto_pilot
ap.reference_frame = v.surface_reference_frame
ap.target_pitch_and_heading(90, heading)
ap.engaged = True
v.control.throttle = 1.0
while v.available_thrust <= 0 and v.control.current_stage > 0:
    v.control.activate_next_stage()
    time.sleep(0.5)
clamps = len(v.parts.launch_clamps)
for clamp in v.parts.launch_clamps:   # release launch clamps without staging anything else
    clamp.release()
v.control.gear = False
log("boost: target apogee %.0f m, pitch %.0f, heading %.0f" % (apogee, pitch, heading))
orbit_ap = conn.add_stream(getattr, v.orbit, 'apoapsis_altitude')
alt = conn.add_stream(getattr, v.flight(), 'mean_altitude')
while orbit_ap() < apogee + 70:
    if alt() > 150:
        ap.target_pitch_and_heading(pitch, heading)
    core.overlay("kspilot  %s\nphase: BOOST\naltitude %6.0f m  apoapsis %6.0f m" % (tag, alt(), orbit_ap()))
    time.sleep(0.05)
v.control.throttle = 0.0
ap.engaged = False
if clamps:
    from kspilot.vehicle import clear_pad  # noqa: E402
    clear_pad(sc, v, log)
log("MECO alt=%.0f apoapsis=%.0f fuel=%.0f" % (alt(), orbit_ap(), v.resources.amount('LiquidFuel')))
# boostback: null the predicted impact error with a short burn (drag-aware predictor)
if "noboost" not in sys.argv:
    import numpy as np  # noqa: E402
    from kspilot import deorbit, mission  # noqa: E402
    rf = v.orbit.body.reference_frame
    for i in range(2):
        c = deorbit.impact_correction(np.asarray(v.position(rf)), np.asarray(v.velocity(rf)), v.mass,
                                      cda=float(os.environ.get("CDA", 15)), target=core.KSC_PAD)
        log("boostback %d: predicted miss %.0fm (along %+.0f cross %+.0f) dv %.1f" % (
            i, c.miss, c.along, c.cross, np.linalg.norm(c.dv_bf)))
        if c.miss < 150:
            break
        mission.burn_vector(conn, v, c.dv_bf, log, rf)
lander = Lander(conn, v, log=log, use_gfold=use_gfold, overlay_tag="G3 " + tag, freeze_leading=os.environ.get("FREEZE", True), h_com=h_com)
res = lander.run()
ok = res['situation'].endswith('landed') and res['miss'] < 20 and res['v_touchdown'] < 4 and res['upright'] > 0.95
log("RESULT %s miss=%.1fm v_td=%.2fm/s upright=%.3f fuel_left=%.0f parts=%d %s" % (
    "PASS" if ok else "FAIL", res['miss'], res['v_touchdown'], res['upright'], res['fuel_left'], res['parts'],
    res['situation']))
for e in lander.events:
    log("event", e)
print("shot:", core.screenshot(tag))
if frames:
    print("video:", core.record_stop(frames, tag))
