"""Pre-flight check of the core's hibernation (kspilot.power) - run ONLY when no mission is flying.
Loads a save with the released core in orbit (default g5_orbit), then:
  1. the 'Hibernate in Warp' toggle reads back (sleep + wake at 1x),
  2. one long coast asleep (warp WARP_S s) and one awake, EC logged -> measured drain per mode (logs/power.jsonl),
  3. after the wake-up: reaction wheels on, the autopilot aligns to retrograde within 60 s.
usage: power_check.py [SAVE] [WARP_S]          (prints RESULT PASS / RESULT FAIL)"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot import core, mission, power  # noqa: E402

save = sys.argv[1] if len(sys.argv) > 1 else "g5_orbit"
warp_s = float(sys.argv[2]) if len(sys.argv) > 2 else 3600.0
log = core.Log("power_check")
conn = core.connect("power_check")
sc = conn.space_center
sc.load(save)
time.sleep(8)
conn = core.connect("power_check")
sc = conn.space_center
core.wait_until(lambda: str(conn.krpc.current_game_scene).endswith("flight"), 60, 1)
time.sleep(3)
v = sc.active_vessel
m = mission.Mission(conn, 100e3, 100e3, 0.0, log=log)
log("vessel %s, EC %.1f/%.1f, can_hibernate %s" % ((v.name,) + power.ec(v) + (power.can_hibernate(v),)))
try:   # read only: the core's solar panels (inside the closed service bay, not used by kspilot.power)
    log("solar panels: %s" % [(sp.part.title, str(sp.state), round(sp.energy_flow, 3)) for sp in v.parts.solar_panels])
except Exception as e:
    log("solar panels: unreadable (%r)" % e)
ok = power.check_toggle(v, log)
m.power_save = ok                                                  # what choose_deorbit sets for a long coast
m.coast_to(v, sc.ut + warp_s + power.WAKE_LEAD, "check asleep")
e0, t0 = power.ec(v)[0], sc.ut                                   # reference: the same coast, core awake
sc.warp_to(sc.ut + min(warp_s, 1800.0))
e1, t1 = power.ec(v)[0], sc.ut
log("power: check awake %.0f s: EC %.1f -> %.1f (%.4f EC/s; flown 0.0501)" % (t1 - t0, e0, e1, (e0 - e1) / (t1 - t0)))
power.record({"ut": t1, "why": "check awake", "mode": "awake", "dt": t1 - t0, "ec0": e0, "ec1": e1,
              "rate": (e0 - e1) / (t1 - t0), "woke": True, "vessel": v.name})
wheels = all(w.active for w in v.parts.reaction_wheels)
ap = v.auto_pilot
ap.reference_frame = v.orbital_reference_frame
ap.target_direction = (0.0, -1.0, 0.0)                            # retrograde
ap.engaged = True
t_al = time.time()
aligned = core.wait_until(lambda: ap.error < 3.0, 60, 0.5)
log("re-align to retrograde: %s in %.1f s (error %.2f deg)" % (aligned, time.time() - t_al, ap.error))
ap.engaged = False                                                # kRPC AutoPilot has no disengage()
log("rates now budgeted: %s" % m.ec_rates)
fine = ok and wheels and aligned and m.ec_rates["hib"] <= power.EC_RATE_HIB and not math.isnan(power.ec(v)[0])
log("toggle %s, wheels on %s, aligned %s, EC %.1f/%.1f" % ((ok, wheels, aligned) + power.ec(v)))
log("RESULT %s" % ("PASS" if fine else "FAIL"))
