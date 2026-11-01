"""G2 bench (Grasshopper-style): lift off, hover, translate, return, land on the pad.
usage: hop.py CRAFT [HOVER_ALT_M] [TRANSLATE_M]"""
import csv
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot import core  # noqa: E402
from kspilot.flightctl import Actuator, PadFrame  # noqa: E402

craft = sys.argv[1]
H = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0
DX = float(sys.argv[3]) if len(sys.argv) > 3 else 50.0
tag = "hop_" + craft.replace(" ", "")
log = core.Log(tag)
conn = core.connect("hop")
sc = conn.space_center
sc.launch_vessel("VAB", craft, "LaunchPad", [], True)
time.sleep(6)
v = sc.active_vessel
body = v.orbit.body
frame = PadFrame(body, *core.KSC_PAD)
pos = conn.add_stream(v.position, frame.rf)
vel = conn.add_stream(v.velocity, frame.rf)
sit = conn.add_stream(getattr, v, 'situation')
g = body.surface_gravity
p0 = frame.pos(pos())
h0 = p0[2]
log("start ENU offset from pad: %.1f %.1f, CoM height %.1f m" % (p0[0], p0[1], h0))

act = Actuator(conn, v, frame, max_tilt_deg=20)
REC = os.environ.get("HOP_REC")
frames = core.record_start(tag) if REC else None
v.control.gear = True
v.control.throttle = 0
while v.available_thrust <= 0 and v.control.current_stage > 0:
    v.control.activate_next_stage()
    time.sleep(0.5)
clamps = len(v.parts.launch_clamps)
for clamp in v.parts.launch_clamps:   # release launch clamps without staging anything else
    clamp.release()
log("engines up, thrust %.0f kN, mass %.1f t, TWR %.2f" % (
    v.available_thrust / 1e3, v.mass / 1e3, v.available_thrust / (v.mass * g)))

# waypoints: (east, north, height, hold seconds)
plan = [(p0[0], p0[1], H, 5), (p0[0] + DX, p0[1], H, 5), (0.0, 0.0, H, 5), (0.0, 0.0, None, 0)]
KP = float(os.environ.get('HOP_KP', 0.15))
KV = float(os.environ.get('HOP_KV', 0.5))
VMAX_H, VMAX_Z, AMAX_H = 10.0, 10.0, 3.0
f = open(log.path.replace(".txt", ".csv"), "w", newline="")
w = csv.writer(f)
w.writerow(["t", "e", "n", "h", "ve", "vn", "vu", "thr", "wp"])
t0 = time.time()
shot_done = False
for i, (te, tn, th, hold) in enumerate(plan):
    t_arr = None
    log("waypoint %d -> E%.0f N%.0f H%s" % (i, te, tn, th))
    while True:
        p = frame.pos(pos()) - np.array([0, 0, h0])
        vv = frame.vec(vel())
        landing = th is None
        tz = 0.0 if landing else th
        # horizontal: velocity-limited position loop
        dh = np.array([te - p[0], tn - p[1]])
        vh_des = KP * dh
        n = np.linalg.norm(vh_des)
        if n > VMAX_H:
            vh_des *= VMAX_H / n
        ah = KV * (vh_des - vv[:2])
        n = np.linalg.norm(ah)
        if n > AMAX_H:
            ah *= AMAX_H / n
        # vertical
        if landing:
            hd = math.hypot(*dh)
            vz_des = -max(1.0, min(VMAX_Z, 0.25 * p[2])) if hd < 3 else 0.0
        else:
            vz_des = max(-VMAX_Z, min(VMAX_Z, 0.5 * (tz - p[2])))
        az = KV * (vz_des - vv[2]) + g
        thr = act.command([ah[0], ah[1], az])
        if int((time.time() - t0) * 4) != getattr(core, '_ov', -1):
            core._ov = int((time.time() - t0) * 4)
            core.overlay('kspilot  G2 hover test  %s\nphase: waypoint %d/%d\nheight %6.1f m   E %6.1f  N %6.1f m\nvel  E %5.1f  N %5.1f  U %5.1f m/s   throttle %3.0f%%' % (craft, i + 1, len(plan), p[2], p[0], p[1], vv[0], vv[1], vv[2], thr * 100))
        w.writerow([round(time.time() - t0, 2)] + [round(x, 2) for x in p] + [round(x, 2) for x in vv] + [round(thr, 3), i])
        if landing:
            if p[2] < 3 and str(sit()).endswith(("landed", "splashed")):
                break
        else:
            if np.linalg.norm([te - p[0], tn - p[1], tz - p[2]]) < 3 and np.linalg.norm(vv) < 1.5:
                t_arr = t_arr or time.time()
                if not shot_done and i == 0:
                    core.screenshot(tag + "_hover")
                    shot_done = True
                if time.time() - t_arr > hold:
                    break
        if time.time() - t0 > 600:
            log("TIMEOUT")
            break
        if int((time.time() - t0) * 20) % 20 == 0:
            f.flush()
        time.sleep(0.05)
touch_v = np.linalg.norm(frame.vec(vel()))
act.release()
time.sleep(3)
p = frame.pos(pos())
err = math.hypot(p[0], p[1])
ok = str(sit()).endswith("landed") and err < 5
log("RESULT %s landed=%s miss=%.2fm (E%.2f N%.2f) touchdown|v|=%.2fm/s parts=%d" % (
    "PASS" if ok else "FAIL", sit(), err, p[0], p[1], touch_v, len(v.parts.all)))
print("shot:", core.screenshot(tag + "_landed"))
f.close()
core.overlay("")
if frames:
    time.sleep(1)
    print("video:", core.record_stop(frames, tag))
