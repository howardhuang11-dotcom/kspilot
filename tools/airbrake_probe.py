"""Pre-flight check for continuous drag modulation (KSP_DMCONT): does the airbrakes' authority limiter set
how far they deploy, and how does drag scale with it? Also measures the grid fins' own drag.

Loads the entry checkpoint (core above the dense air, engine-first), then for each configuration waits
for the drag to settle and reports the effective CdA = drag / (0.5 rho v^2) from kRPC's aero force.
Run only when no flight is in progress (it loads a save). usage: airbrake_probe.py [save]
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot import core  # noqa: E402


def main():
    save = sys.argv[1] if len(sys.argv) > 1 else "g5_term"
    conn = core.connect("probe")
    sc = conn.space_center
    sc.load(save)
    time.sleep(8)
    conn = core.connect("probe")
    sc = conn.space_center
    core.wait_until(lambda: str(conn.krpc.current_game_scene).endswith("flight"), 60, 1)
    time.sleep(1)
    v = sc.active_vessel
    rf = v.orbit.body.reference_frame
    fl = v.flight(rf)
    brakes = [cs for cs in v.parts.control_surfaces if "airbrake" in cs.part.name]
    fins = [p for p in v.parts.all if "Grid Fin" in p.name]
    print("airbrakes %d, grid fins %d" % (len(brakes), len(fins)))

    def cda(label):
        time.sleep(1.0)               # g5_term starts at ~2.7 km: the whole probe must fit in ~20 s
        vals = []
        for _ in range(3):
            F = np.asarray(fl.aerodynamic_force)
            vel = np.asarray(v.velocity(rf))
            sp = float(np.linalg.norm(vel))
            drag = -float(np.dot(F, vel / sp))
            rho = fl.atmosphere_density
            vals.append(drag / (0.5 * rho * sp * sp))
            time.sleep(0.2)
        print("%-34s CdA %6.2f m^2  (alt %.0f m, v %.0f m/s)" % (label, np.median(vals), fl.mean_altitude, fl.speed))

    def set_fins(out):
        for p in fins:
            for m in p.modules:
                ev = "Extend Fins" if out else "Retract Fins"
                if m.name == "ModuleAnimateGeneric" and m.has_event(ev):
                    m.trigger_event(ev)

    v.auto_pilot.engaged = False
    v.control.sas = True
    v.control.brakes = True
    set_fins(True)
    for a in (100.0, 50.0, 25.0, 0.0, 100.0):
        for cs in brakes:
            cs.authority_limiter = a
        cda("brakes group ON, airbrake auth %3.0f%%" % a)
    v.control.brakes = False
    set_fins(True)
    cda("airbrakes IN, fins OUT")
    set_fins(False)
    cda("airbrakes IN, fins IN")
    sc.load(save)   # leave the checkpoint as it was


if __name__ == "__main__":
    main()
