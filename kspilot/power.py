"""Electric power of the recovered core during the long coasts after payload release.

Mishap 2026-10-21: the core's 1000 EC battery ran flat during a 5.5 h wait in orbit -> reaction wheels "not enough
power", no attitude, no throttle -> no deorbit burn. Deorbit windows of C1 can be as far as ~5.3 h away (90 deg).

What we know (T-WorksFineGF core: HECS2 probe core = 1000 EC battery + reaction wheel; no other consumer):
  MEASURED  awake, coasting / warping: 0.050 EC/s (g5_orbit UT 149591 999.7 EC -> g5_entry UT 159858 485.6 EC).
            This is exactly the stock HECS2 ModuleCommand rate (0.05 EC/s); idle reaction wheels draw nothing
            (stock: 0.5 EC/s x commanded torque, only while actuated).
  ASSUMED   hibernating: stock cfg hibernationMultiplier 0.002 -> 0.0001 EC/s. Never measured in flight here, so the
            budget uses EC_RATE_HIB = 0.01 EC/s (100x the cfg value) until coast logs (logs/power.jsonl) show it.
Only the probe core sleeps ("Hibernate in Warp" = ModuleCommand.hibernateOnWarp: hibernation only while time warp
runs, KSP wakes it when warp stops); reaction wheels are switched off as well. Both are restored and read back
WAKE_LEAD s before the end of every coast, i.e. before any alignment or burn. Power mode is used only when the planned
coast in orbit (payload release -> first deorbit burn) exceeds KSP_HIBERNATE_MIN_H (2.0 h): the baseline is untouched.

Solar panels (assessed 2026-10-21, NOT used): the core carries 4 SP-L 1x6 (solarPanels2, 1.64 EC/s each at normal
incidence, no pivot = non-tracking, retractable). In a 100 km orbit the core is sunlit ~67 % of the time (shadow
half-angle asin(600/700) = 59 deg), so even at a random fixed attitude (mean cos factor ~1/4) they would give ~1 EC/s,
20x the 0.05 EC/s drain. But they sit on the probe core INSIDE the closed ServiceBay.250 (never opened, panels never
extended in any flight): using them needs the bay doors open + panels extended, then both mechanisms retracted and
read back before the deorbit burn and entry (open bay / panels in the air: drag off the calibrated tables, breakage,
heating). Clearance of the panels inside the bay is unverified (impactResistance 2 m/s). Not low-risk -> not
implemented; hibernation alone keeps ~66 % at touchdown after a 5.3 h wait.
"""
import json
import math
import os
import time
from dataclasses import dataclass

EC_RATE_AWAKE = 0.06     # EC/s budgeted awake: measured 0.050 + 20 % for wheel work (alignments, trims)
EC_RATE_HIB = 0.01       # EC/s budgeted while hibernating in warp: ASSUMED (cfg 0.0001; 100x margin)
EC_KEEP = 0.3            # fraction of the battery that must be left at touchdown
WAKE_LEAD = 120.0        # s: everything is awake (and verified) this long before the end of a coast
DESCENT_MARGIN = 300.0   # s awake after the predicted impact time (landing burn, hover, settling)
MEASURE_MIN = 1800.0     # s: a coast at least this long updates the in-flight drain rate of its mode


def ec(v):
    """(ElectricCharge amount, max) of the vessel; (nan, nan) when unreadable (never mistaken for 'no battery')."""
    try:
        r = v.resources
        return float(r.amount("ElectricCharge")), float(r.max("ElectricCharge"))
    except Exception:
        return float("nan"), float("nan")


@dataclass
class Budget:
    ec_now: float
    ec_max: float
    t_hib: float             # s hibernating in warp
    t_awake: float           # s awake (planning, wake lead, burn, coast after the burn, descent)
    rate_hib: float
    rate_awake: float

    @property
    def ec_touchdown(self):
        return self.ec_now - self.rate_hib * self.t_hib - self.rate_awake * self.t_awake

    @property
    def ok(self):
        if not (math.isfinite(self.ec_now) and math.isfinite(self.ec_max)):
            return False         # unreadable: no optional waiting (the caller then flies the soonest window)
        if self.ec_max <= 0:
            return True          # no battery on board: nothing to protect
        return self.ec_touchdown >= EC_KEEP * self.ec_max

    def text(self):
        frac = self.ec_touchdown / self.ec_max if self.ec_max > 0 else float("nan")
        return ("EC now %.0f/%.0f, %.0f s hibernating @%.4f + %.0f s awake @%.3f -> %.0f EC at touchdown (%.0f%%, "
                "keep %.0f%%): %s" % (self.ec_now, self.ec_max, self.t_hib, self.rate_hib, self.t_awake,
                                      self.rate_awake, self.ec_touchdown, 100 * frac, 100 * EC_KEEP,
                                      "OK" if self.ok else "REJECT"))


def budget(ec_now, ec_max, t_to_burn, t_after_burn, hibernate, rate_hib=EC_RATE_HIB, rate_awake=EC_RATE_AWAKE,
           wake_lead=WAKE_LEAD):
    """EC at touchdown if the core coasts t_to_burn s to the first deorbit burn (asleep, if it can hibernate, all
    but the last wake_lead s) and then t_after_burn s to touchdown (awake: counted so even where it warps)."""
    t_to_burn = max(float(t_to_burn), 0.0)
    t_hib = max(t_to_burn - wake_lead, 0.0) if hibernate else 0.0
    return Budget(ec_now, ec_max, t_hib, t_to_burn - t_hib + max(float(t_after_burn), 0.0), rate_hib, rate_awake)


# --------------------------------------------------------------------------- part state (kRPC)
HIB_WARP = "hibernateOnWarp"     # ModuleCommand field ids (C# names; the GUI names are localised)
HIB = "hibernation"


def _command_modules(v):
    return [m for p in v.parts.all for m in p.modules if m.name == "ModuleCommand"]


def _field(m, fid):
    for f in m.field_list:
        if f.name == fid:
            return f
    return None


def _get_bool(m, fid):
    f = _field(m, fid)
    if f is not None:
        return bool(f.bool_value)
    return str(m.get_field_by_id(fid)).strip().lower() in ("true", "on", "yes", "1")


def _set_bool(m, fid, value):
    f = _field(m, fid)
    if f is not None:
        f.bool_value = bool(value)
    else:
        m.set_field_bool_by_id(fid, bool(value))


def can_hibernate(v):
    """The vessel has a probe core whose 'Hibernate in Warp' field kRPC can read (read only)."""
    try:
        mods = _command_modules(v)
        for m in mods:
            _get_bool(m, HIB_WARP)
        return bool(mods)
    except Exception:
        return False


def set_state(v, asleep):
    """asleep: probe cores hibernate in warp, reaction wheels off, SAS / autopilot off. Awake: the opposite (no
    hibernation at all). Every change is read back; returns True only if the vessel is verifiably in that state."""
    ok = True
    try:
        if asleep:
            try:
                v.auto_pilot.disengage()
            except Exception:
                pass
            v.control.sas = False
        mods = _command_modules(v)
        if not mods:
            ok = False
        for m in mods:
            _set_bool(m, HIB_WARP, asleep)
            if not asleep and _get_bool(m, HIB):
                _set_bool(m, HIB, False)             # never leave a core fully hibernating
        wheels = list(v.parts.reaction_wheels)
        for w in wheels:
            if not w.broken:
                w.active = not asleep
        time.sleep(0.2)
        for m in mods:
            ok = ok and _get_bool(m, HIB_WARP) == asleep and (asleep or not _get_bool(m, HIB))
        for w in wheels:
            ok = ok and (w.broken or w.active == (not asleep))
    except Exception:
        return False
    return ok


def sleep(v, log=print):
    e = ec(v)
    ok = set_state(v, True)
    log("power: %s (EC %.0f/%.0f)" % ("asleep for warp: cores hibernate in warp, wheels off" if ok else
                                     "WARNING could not put the core to sleep (stays awake)", *e))
    return ok


def wake(v, log=print):
    ok = set_state(v, False)
    if not ok:
        time.sleep(1.0)
        ok = set_state(v, False)
    e = ec(v)
    log("power: %s (EC %.0f/%.0f)" % ("awake: hibernation off, wheels on (read back)" if ok else
                                     "ERROR wake-up not verified (hibernation / reaction wheels)", *e))
    return ok


def check_toggle(v, log=print):
    """Pre-coast verification at 1x: sleep and wake once, both read back. True only if hibernation can be used."""
    if not can_hibernate(v):
        log("power: no readable 'Hibernate in Warp' field - budgeting the coasts awake")
        return False
    ok = set_state(v, True)
    back = set_state(v, False)
    if not back:
        back = wake(v, log)
    log("power: hibernation toggle check %s" % ("OK" if ok and back else "FAILED - budgeting the coasts awake"))
    return ok and back


def record(entry, path=None):
    """Append one coast measurement to logs/power.jsonl (the hibernation drain is still an assumption)."""
    try:
        from .core import LOG_DIR
        path = path or os.path.join(LOG_DIR, "power.jsonl")
        with open(path, "a") as f:
            f.write(json.dumps(entry) + "\n")
    except Exception:
        pass
