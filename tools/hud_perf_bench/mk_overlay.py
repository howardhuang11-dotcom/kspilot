"""Realistic v8 overlay (ascent phase + lander keys) from kspilot/hud.py's own _compose, no kRPC."""
import sys
sys.path.insert(0, sys.argv[1])
from kspilot import hud  # noqa: E402

f = hud.HudFeed(target_km=100)
f._ut = 100000.0
f.event("LIFTOFF", 99900.0)
f.event("MAX-Q", 99942.0)
for n, dt in [("MECO", 80), ("ORBIT", 300), ("PAYLOAD SEP", 400), ("DEORBIT BURN", 3000), ("ENTRY BURN", 3500),
              ("AERO GLIDE", 3600), ("LANDING BURN", 3700), ("TOUCHDOWN", 3720)]:
    f.plan(n, f._ut + dt)
f.mark("S8", f._ut + 3550)
f.set(cam="PAD TRACKING CAMERA · KSC", camtel="RANGE 12.34 KM    ELEV 23.4°    AZ 087°", camuntil="99930.000")
tel = dict(alt=23000.0, speed=700.0, vs=300.0, q=12000.0, thr=1.0, ap=45000.0, pe=-500000.0, inc=6.0, stage=2,
           warp=1.0, x=31000.0, fuel=0.6, srb=0)
txt = f._compose(tel)
open(sys.argv[2], "w").write(txt)
print(len(txt), "bytes,", txt.count("\n") + 1, "lines")
