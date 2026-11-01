"""Full mission: pre-launch check -> ascent -> payload release -> deorbit -> entry
-> correction burns -> landing burn on the target.
"""
import math
import os
import time

import numpy as np

from . import hud, ascent, core, deorbit, entry, feasibility, vehicle
from . import power as pwr
from .landing import Lander


def _wait_aligned(v, max_err=3.0, timeout=60, target=None, rf=None):
    """Wait until the nose is within max_err deg of `target` (unit vector in rf), measured directly
    (the autopilot's own error can still refer to the previous target right after a change)."""
    time.sleep(0.5)
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            if target is not None:
                d = np.asarray(v.direction(rf))
                if math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(d, target)))))) < max_err:
                    return True
            elif v.auto_pilot.error < max_err:
                return True
        except Exception:
            pass
        time.sleep(0.2)
    return False


def burn_vector(conn, v, dv_bf, log, rf=None, align_timeout=15):
    """Burn a delta-v vector given in the axes of `rf` (default: body-fixed frame of the main body).
    The burn is measured in the NON-ROTATING frame (a rotating frame adds Coriolis/centrifugal terms
    that a small burn can never 'complete'), with a hard cap on burn time."""
    sc = conn.space_center
    body = v.orbit.body
    rf = rf or body.reference_frame
    nrf = body.non_rotating_reference_frame
    dv = np.asarray(sc.transform_direction(tuple(float(x) for x in dv_bf), rf, nrf))   # keeps the magnitude
    mag = float(np.linalg.norm(dv))
    if mag < 0.01:
        return
    u = dv / mag
    fine = mag < 5.0
    engines = [e for e in v.parts.engines if e.active]
    ap = v.auto_pilot
    ap.reference_frame = nrf
    ap.target_direction = tuple(u)
    ap.engaged = True
    if not _wait_aligned(v, timeout=align_timeout, target=u, rf=nrf):
        log("burn skipped: could not align (EC %.0f/%.0f)" % power(v))
        ap.engaged = False
        return
    if fine:
        # open-loop timed pulse: set the thrust limiter so the trim takes ~0.6 s at full throttle,
        # then time it on the game clock (stream readings lag far too much for tens-of-ms pulses)
        a_full = max(v.available_thrust / v.mass, 1e-3) / max(engines[0].thrust_limit if engines else 1.0, 1e-3)
        lim = min(1.0, max(0.01, mag / (0.6 * a_full)))
        for e in engines:
            e.thrust_limit = lim
        t_burn = mag / (a_full * lim)
        ut = conn.add_stream(getattr, sc, 'ut')
        t_start, w_start = ut(), time.time()
        v.control.throttle = 1.0
        while ut() - t_start < t_burn and time.time() - w_start < 30:
            time.sleep(0.002)
        v.control.throttle = 0.0
        ut.remove()
        core.wait_until(lambda: v.thrust < 1.0, 2.0, 0.05)   # throttle cut reaches the engine a frame later
        time.sleep(0.2)
        for e in engines:
            e.thrust_limit = 1.0
        ap.engaged = False
        log("burn done: %.3f m/s timed pulse %.2fs at %.1f%% thrust" % (mag, t_burn, lim * 100))
        return mag
    thrust = conn.add_stream(getattr, v, 'thrust')     # actual thrust (N)
    mass = conn.add_stream(getattr, v, 'mass')
    ut = conn.add_stream(getattr, sc, 'ut')
    a_nom = max(v.available_thrust / v.mass, 1e-3)
    t_cap = 3.0 * mag / a_nom + 2.0
    got, t_prev = 0.0, ut()
    v.control.throttle = 1.0
    t0 = time.time()
    while time.time() - t0 < t_cap:
        t_now = ut()
        got += thrust() / max(mass(), 1.0) * (t_now - t_prev)   # integrate thrust: gravity-free, like a node
        t_prev = t_now
        left = mag - got
        if left <= (0.003 if fine else 0.1):
            break
        a = max(v.available_thrust / v.mass, 1e-3)
        v.control.throttle = float(min(1.0, max(0.02, 2.0 * left / a)))
        time.sleep(0.005)
    v.control.throttle = 0.0
    core.wait_until(lambda: v.thrust < 1.0, 2.0, 0.05)
    for st in (thrust, mass, ut):
        st.remove()
    ap.engaged = False
    log("burn done: %.3f m/s requested, %.3f m/s achieved%s" % (mag, got, "" if time.time() - t0 < t_cap else " (time cap)"))
    return got


def orbit_state(sc, v, rf):
    """Noise-free body-frame state (position, surface-relative velocity) from the orbit elements at one
    instant (separate position/velocity RPCs can come from different physics frames)."""
    o, ut = v.orbit, sc.ut
    r0 = np.asarray(o.position_at(ut, rf))
    rp, rm = np.asarray(o.position_at(ut + 0.25, rf)), np.asarray(o.position_at(ut - 0.25, rf))
    v_in = (rp - rm) / 0.5                       # inertial velocity in the (frozen) body-frame axes
    w = v.orbit.body.rotational_speed
    return r0, v_in - np.array([-w * r0[2], 0.0, w * r0[0]])   # minus omega x r (kRPC body frame)


EC_RATE_PRIOR = 0.1      # EC/s before a drain is measured (T-WorksFineGF in orbit: 0.05 EC/s measured, 2x margin)
EC_KEEP = pwr.EC_KEEP    # fraction of the battery that must be left at touchdown after an optional wait
power = pwr.ec           # (ElectricCharge amount, max) of the vessel


def daylight_wait_ok(ec, ec_max, rate, t_rest):
    """(Pre-C1 daylight loop; the deorbit window choice now uses kspilot.power.budget.) An optional extra revolution
    (daylight for the camera) only if the battery still holds EC_KEEP of its
    capacity after the wait plus the rest of the flight (t_rest s at the measured drain rate)."""
    if ec_max <= 0 or ec != ec:
        return ec_max <= 0             # no battery on board: nothing to protect; unreadable: do not wait
    return ec - max(rate, 0.0) * t_rest >= EC_KEEP * ec_max


def core_cda(conn, v):
    return feasibility.estimate_cda(conn, v) or 3.0


def active_core(sc, v):
    """After payload separation make sure we fly the vessel that has the legs."""
    if any(l for l in v.parts.legs):
        return v
    best = None
    for o in sc.vessels:
        if o == v:
            continue
        try:
            if o.parts.legs and np.linalg.norm(np.asarray(o.position(v.reference_frame))) < 500:
                best = o
        except Exception:
            pass
    if best is not None:
        sc.active_vessel = best
        time.sleep(1)
        return best
    return v


def default_entry_dv(v):
    """Entry-burn size for this vehicle. Fins-only cores (no airbrakes) decelerate little at Mach 4-6 (CdA ~4 m^2)
    and reach q 25-36 kPa below 20 km; their bigger burn is planned (deorbit aim + predictor), not left to the
    q-safety burns that cost GF0T 580 m/s and a 20.6 km miss (2026-10-20 18:22)."""
    if os.environ.get("KSP_ENTRY_DV"):
        return float(os.environ["KSP_ENTRY_DV"])
    names = [p.name for p in v.parts.all]
    fins = any("Grid Fin" in n for n in names)
    brakes = any("airbrake" in n for n in names)
    return 800.0 if (fins and not brakes) else 590.0


class Mission:
    def __init__(self, conn, pe, ap, inc, recover=True, target=core.KSC_PAD, log=print, tag="mission"):
        self.conn, self.sc = conn, conn.space_center
        self.pe, self.ap, self.inc, self.recover, self.target = pe, ap, inc, recover, target
        self.log, self.tag = log, tag
        self.target_alt = None          # offset aim (kspilot/aim.py): the target's terrain height; None = the pad
        self.entry_alt, self.entry_dv = 45000.0, 590.0   # entry burn high up (low q), as in operational booster recovery
        # m/s: deorbit windows needing a bigger plane change are only flown when no cheaper one exists (C1)
        self.max_dv_normal = float(os.environ.get("KSP_MAX_DVN", 30.0))
        # revolutions surveyed for a window within max_dv_normal: inclined orbits only pass back over the pad
        # every half sidereal day (~5.5 revolutions at 100 km), so 12 reaches the second such window
        self.deorbit_revs = int(os.environ.get("KSP_DEORBIT_REVS", 12))
        self.land_reserve_dv = float(os.environ.get("KSP_LAND_RESERVE_DV", 250.0))   # landing burn (1.3 v_t + 60)
        # electric power of the core in the long coasts after payload release (kspilot.power; mishap 2026-10-21)
        self.hibernate = os.environ.get("KSP_HIBERNATE", "1") != "0"        # 0: coasts stay awake (rollback)
        # power mode only for long missions: the proven baseline (first plans flown: release -> deorbit burn up to
        # ~1.7 h) must not change; the 28/45/90 deg window waits (2.35-5.3 h) use it. On when the planned coast in orbit, payload release -> first deorbit burn, exceeds this.
        self.hibernate_min_s = 3600.0 * float(os.environ.get("KSP_HIBERNATE_MIN_H", 2.0))
        self.power_save = False      # set by choose_deorbit: coasts sleep (hibernate in warp, wheels off)
        self.ut_release = None
        self.ec_rates = {"hib": float(os.environ.get("KSP_EC_HIB", pwr.EC_RATE_HIB)),
                         "awake": float(os.environ.get("KSP_EC_AWAKE", pwr.EC_RATE_AWAKE))}
        self.ec_mark = None          # (EC, max, UT) at the start of the current awake stretch
        self.power_log = os.path.join(core.LOG_DIR, "power.jsonl")

    def check(self):
        v = self.sc.active_vessel
        self.entry_dv = default_entry_dv(v)
        for _ in range(10):
            try:
                v.delta_v
                break
            except Exception:
                time.sleep(1)
        ok, rep, lines = feasibility.analyse(self.conn, v, self.pe, self.ap, self.inc, self.recover,
                                             entry_burn_dv=self.entry_dv if self.recover else 0.0)
        for l in lines:
            self.log(l)
        return ok, rep

    def fly_ascent(self):
        v = self.sc.active_vessel
        lim = vehicle.autostage_limit(v)
        self.payload_stage = vehicle.payload_stage(v)
        # own guidance (kspilot.ascent_own) then our own trims; KSP_OWNASCENT is no longer needed
        return ascent.ascend(self.conn, self.pe, self.ap, self.inc, self.log, autostage_limit=lim)

    def release_payload(self):
        v = self.sc.active_vessel
        if self.payload_stage is None:
            return v
        while v.control.current_stage > self.payload_stage:
            v.control.activate_next_stage()
            time.sleep(1.5)
            v = self.sc.active_vessel
        self.log("payload released (stage %d)" % self.payload_stage)
        hud.report("event", "PAYLOAD SEP")
        time.sleep(3)
        v = active_core(self.sc, self.sc.active_vessel)
        self.ec_mark = power(v) + (self.sc.ut,)
        self.ut_release = self.sc.ut
        self.log("power: core EC %.0f/%.0f at payload release" % self.ec_mark[:2])
        return v

    def awake_rate(self, v):
        """EC/s budgeted awake: the prior, or the drain measured since the last mark if that is higher."""
        if self.ec_mark is not None:
            e1, dt = power(v)[0], self.sc.ut - self.ec_mark[2]
            r = (self.ec_mark[0] - e1) / dt if dt >= 600.0 else float("nan")
            if math.isfinite(r) and r > self.ec_rates["awake"]:
                self.log("power: measured awake drain %.4f EC/s over %.0f s > budget %.4f - budgeting it" % (
                    r, dt, self.ec_rates["awake"]))
                self.ec_rates["awake"] = r
        return self.ec_rates["awake"]

    def coast_to(self, v, ut_target, why="coast"):
        """Warp to ut_target. A long coast runs with the core asleep (probe core hibernating in warp, reaction
        wheels off) up to WAKE_LEAD s before ut_target; then everything is woken AND read back, and the last
        WAKE_LEAD s are warped awake, so alignment and burns always find a live core. EC is logged before/after."""
        lead = ut_target - self.sc.ut
        if lead <= 0:
            return
        if not (self.hibernate and self.power_save) or lead < pwr.WAKE_LEAD + 60.0:
            core.warp_to(self.sc, ut_target)    # baseline / short coast: exactly the pre-power-mode warp (= sc.warp_to)
            return
        self.awake_rate(v)
        e0, ut_a = power(v)[0], self.sc.ut
        slept = pwr.sleep(v, self.log)
        e1, ut_b = e0, ut_a
        try:
            core.warp_to(self.sc, ut_target - pwr.WAKE_LEAD)
            e1, ut_b = power(v)[0], self.sc.ut
        finally:
            woke = pwr.wake(v, self.log)
        mode, dt = ("hib" if slept else "awake"), ut_b - ut_a
        rate = (e0 - e1) / dt if dt > 0 else float("nan")
        self.log("power: %s %.0f s %s: EC %.1f -> %.1f (%.4f EC/s, budget %.4f)" % (
            why, dt, "hibernating" if slept else "AWAKE", e0, e1, rate, self.ec_rates[mode]))
        if math.isfinite(rate) and self.power_log:
            pwr.record({"ut": ut_b, "why": why, "mode": mode, "dt": dt, "ec0": e0, "ec1": e1, "rate": rate,
                        "woke": woke, "vessel": getattr(v, "name", "?")}, self.power_log)
        if dt >= pwr.MEASURE_MIN and math.isfinite(rate) and rate > self.ec_rates[mode]:
            self.log("power: measured %s drain %.4f EC/s > budget %.4f - budgeting it from now on" % (
                mode, rate, self.ec_rates[mode]))
            self.ec_rates[mode] = rate
        if not woke:
            self.log("power ERROR: core not verifiably awake %.0f s before %s" % (pwr.WAKE_LEAD, why))
        self.ec_mark = power(v) + (self.sc.ut,)
        core.warp_to(self.sc, ut_target)

    def deorbit(self, v, cda=None, aim_bias=0.0, daylight=False, drag=None, pe_alt=-60e3):
        cda = cda or core_cda(self.conn, v)
        isp = max((e.vacuum_specific_impulse for e in v.parts.engines), default=300.0)
        veh = deorbit.Vehicle(mass=v.mass, thrust=v.max_vacuum_thrust or v.max_thrust, isp=isp,
                              cda=None if drag else cda, drag=drag)
        ut0, plan = self.choose_deorbit(v, veh, aim_bias, pe_alt, daylight)
        self.log("deorbit plan: miss %.0fm dv %.1f m/s (retro %.1f normal %.1f) t_ign +%.0fs CdA %.2f" % (
            plan.miss, plan.dv, plan.dv_retro, plan.dv_normal, plan.t_ign, cda))
        for attempt in range(3):
            for (t, pro, nor, rad) in plan.nodes:
                v.control.add_node(ut0 + t, prograde=pro, normal=nor, radial=rad)
            hud.report("plan", "DEORBIT BURN", ut0 + max(t for (t, pro, nor, rad) in plan.nodes))   # the retro node is last
            self.heading = plan.heading   # fixed ground-track heading for all later along/cross errors
            if self.execute_nodes_guarded(v) is not False:
                hud.report("event", "DEORBIT BURN", ut0 + max(t for (t, pro, nor, rad) in plan.nodes))
                hud.report("phase", "descent")
                break
            # a burn was missed (nothing flown since): plan again from where we are now
            self.log("deorbit: re-planning after a missed burn")
            ut0, plan = self.choose_deorbit(v, veh, aim_bias, pe_alt, daylight=False)
            self.log("deorbit plan: miss %.0fm dv %.1f m/s t_ign +%.0fs" % (plan.miss, plan.dv, plan.t_ign))
        return plan, cda

    def _tkw(self):
        """Predictor kwargs for an offset target (its terrain height); {} = the pad defaults, as before."""
        return {} if getattr(self, "target_alt", None) is None else {"target_alt": self.target_alt}

    def choose_deorbit(self, v, veh, aim_bias=0.0, pe_alt=-60e3, daylight=False, _resurvey=True):
        """Pick the deorbit window and plan it (C1, 2026-10-21). Never raises, never strands the core.
        Returns (ut0, plan): plan times are seconds after ut0.

        Windows are surveyed from one state propagated k revolutions ahead (deorbit.choose_window): the first
        with |normal dv| <= KSP_MAX_DVN (and daylight at landing if asked) is taken at once - the pre-C1 rule,
        so the baseline plan is unchanged. Inclined orbits return over the pad only every half sidereal day, so
        the survey runs KSP_DEORBIT_REVS (12) revolutions and otherwise flies the cheapest window seen (WARNING),
        or - over the fuel budget - that window along-track only, or a plain retro burn as the last resort.
        A window k > 0 revolutions away is warped to and re-planned from the live state there."""
        rf = v.orbit.body.reference_frame
        ut0 = self.sc.ut
        r0, v0 = orbit_state(self.sc, v, rf)
        period = v.orbit.period
        eb = self.entry_burns(v)
        kw = dict(target=self.target, aim_bias=aim_bias, pe_alt=pe_alt, entry_burns=eb, **self._tkw())

        def plan_at(tb, **extra):
            rb, vb = (r0, v0) if tb == 0 else deorbit.propagate_bf(r0, v0, tb)
            return deorbit.plan_deorbit(rb, vb, veh, **kw, **extra)

        accept = None
        if daylight:      # for the camera: the landing must happen in daylight
            def accept(plan, tb):
                el = core.sun_elevation_after(self.conn, ut0 + tb + plan.t_impact + 120 - self.sc.ut)
                self.log("predicted sun elevation at landing (rev %d): %.0f deg" % (round(tb / period), el))
                return el > 12

        def budget_ok(plan):
            ok, avail, need = deorbit.deorbit_budget(plan.dv, v.mass, v.dry_mass, veh.isp,
                                                     self.entry_dv + self.land_reserve_dv)
            self.log("deorbit budget: %.0f m/s available, %.0f m/s needed (deorbit %.0f + entry/landing %.0f)" % (
                avail, need, plan.dv, self.entry_dv + self.land_reserve_dv))
            return ok

        # electric power (mishap 2026-10-21): a window is only eligible if the battery still holds EC_KEEP at
        # touchdown after waiting for it. Asleep in warp is budgeted only for a long mission (planned coast from
        # release to the deorbit burn > KSP_HIBERNATE_MIN_H) whose hibernation toggle reads back; the toggle is checked
        # lazily, so the baseline (short) path makes no power-mode call at all - only EC reads.
        self.power_save = False
        rate_awake = self.awake_rate(v)
        ec_now, ec_max = power(v)
        ut_ec = self.sc.ut
        t_rel = self.ut_release if self.ut_release is not None else ut_ec
        hib = {}

        def window_times(plan, tb):
            t_first = min(plan.t_ign, plan.t_pc if plan.t_pc is not None else plan.t_ign)
            t_burn = ut0 + tb + t_first - ut_ec                       # s from now to the first deorbit burn
            t_after = plan.t_impact - t_first + pwr.DESCENT_MARGIN    # s from that burn to touchdown
            return t_burn, t_after, ut_ec + t_burn - t_rel > self.hibernate_min_s   # coast in orbit since release

        def hibernation_ok():
            if "ok" not in hib:
                hib["ok"] = self.hibernate and pwr.check_toggle(v, self.log)
            return hib["ok"]

        def power_ok(plan, tb):
            t_burn, t_after, long_ = window_times(plan, tb)
            b = pwr.budget(ec_now, ec_max, t_burn, t_after, long_ and self.hibernate and hibernation_ok(),
                           self.ec_rates["hib"], rate_awake)
            self.log("deorbit EC budget (rev %d, burn in %.1f h%s): %s" % (
                round(tb / period), t_burn / 3600, ", long: power mode" if b.t_hib > 0 else "", b.text()))
            return b.ok

        n_revs = max(self.deorbit_revs, 16) if daylight else self.deorbit_revs
        ch = deorbit.choose_window(plan_at, period, self.max_dv_normal, n_revs, accept, budget_ok, log=self.log,
                                   power_ok=power_ok)
        if ch.plan is not None and self.hibernate:
            long_ = window_times(ch.plan, ch.t_base)[2]
            self.power_save = long_ and hibernation_ok()
            if long_:
                self.log("power: planned coast > %.1f h: power mode %s (hibernate in warp, wheels off)" % (
                    self.hibernate_min_s / 3600, "ON" if self.power_save else "UNAVAILABLE - coasts stay awake"))
        extra = {}
        if ch.status == "over_budget":   # no fuel for the plane change: come down on this pass, off the pad
            extra = dict(cross_mode="none")
            try:
                ch.plan = plan_at(ch.t_base, **extra)
                self.log("deorbit ERROR: along-track only (rev %d): expect a %.1f km cross-track miss" % (
                    ch.k, ch.plan.cross / 1e3))
            except deorbit.PLAN_ERRORS as e:
                self.log("deorbit: along-track-only plan failed (%s)" % str(e)[:60])
                ch.plan = None
        if ch.plan is None:
            self.log("deorbit ERROR: no targeted plan; plain retro burn to Pe %.0f km (lands off target)" % (
                pe_alt / 1e3))
            ut1 = self.sc.ut
            r1, v1 = orbit_state(self.sc, v, rf)
            return ut1, deorbit.retro_plan(r1, v1, veh, target=self.target, pe_alt=pe_alt, t_ign=60.0, entry_burns=eb,
                                               **self._tkw())
        ut_plan = ut0 + ch.t_base
        if ch.t_base > 0:
            self.log("deorbit: window in %d revolutions (%.1f h): normal %.0f m/s, total %.0f m/s [%s]" % (
                ch.k, (ut_plan + ch.plan.t_ign - self.sc.ut) / 3600, ch.plan.dv_normal, ch.plan.dv, ch.status))
            self.coast_to(v, ut_plan - 120.0, "deorbit window")   # a little early: re-plan sees it >= 2 min ahead
            ut1 = self.sc.ut
            r1, v1 = orbit_state(self.sc, v, rf)
            try:   # fresh state, same window (planned exactly as the pre-C1 loop did after warping)
                p1 = deorbit.plan_deorbit(r1, v1, veh, **kw, **extra)
                same = abs(ut1 + p1.t_ign - (ut_plan + ch.plan.t_ign)) < 0.25 * period
                if (p1.impact.hit and p1.miss <= 5e3 and same
                        and abs(p1.dv_normal) <= max(abs(ch.plan.dv_normal) + 5.0, self.max_dv_normal)):
                    return ut1, p1
                self.log("deorbit re-plan (normal %.0f, t_ign %+.0fs vs survey) rejected; flying the survey plan" % (
                    p1.dv_normal, ut1 + p1.t_ign - (ut_plan + ch.plan.t_ign)))
            except deorbit.PLAN_ERRORS as e:
                self.log("deorbit re-plan failed (%s); flying the survey plan" % str(e)[:60])
        stale = ut_plan + min(n[0] for n in ch.plan.nodes) - self.sc.ut < 60.0
        if stale and not (ch.status == "ok" and ch.k == 0):   # first-try plan: as fresh as before C1
            # the survey itself took the time (game runs at 1x while planning): this window is gone
            self.log("deorbit: chosen window passed during the survey")
            if _resurvey:
                return self.choose_deorbit(v, veh, aim_bias, pe_alt, daylight, _resurvey=False)
            ut1 = self.sc.ut
            r1, v1 = orbit_state(self.sc, v, rf)
            return ut1, deorbit.retro_plan(r1, v1, veh, target=self.target, pe_alt=pe_alt, t_ign=60.0, entry_burns=eb,
                                               **self._tkw())
        return ut_plan, ch.plan

    def execute_nodes_guarded(self, v):
        """Warp to each node and fly it with our own burn executor (kspilot.burn). A missed burn
        invalidates the rest of the plan: all nodes are removed and False is returned (caller re-plans)."""
        from . import burn
        while v.control.nodes:
            n = v.control.nodes[0]
            lead = n.ut - self.sc.ut - 90
            if lead > 5:
                self.log("warping %.0fs to node" % lead)
                self.coast_to(v, n.ut - 90, "node")
            if burn.execute_node(self.conn, v, n, self.log) is None:
                for m in list(v.control.nodes):
                    m.remove()
                return False
            self.log("node done, remaining nodes %d" % len(v.control.nodes))
        return True

    def entry_burns(self, v):
        """Expected entry burn (fires at ~q 20 kPa, ~24 km on a steep return; ~590 m/s), as a predictor segment."""
        from .entry import Burn
        self.entry_dv = default_entry_dv(v)
        isp = max((e.vacuum_specific_impulse for e in v.parts.engines if e.active), default=300.0)
        return (Burn(t_start=0.0, thrust=v.max_thrust or v.available_thrust, isp=isp, mode="air_retro",
                     dv=self.entry_dv, alt_trigger=self.entry_alt),)

    def predict_median(self, v, cda, drag, n=3, aim_bias=0.0, cross_bias=0.0):
        rf = v.orbit.body.reference_frame
        cs = []
        for i in range(n):
            r0, v0 = orbit_state(self.sc, v, rf)
            cs.append(deorbit.impact_correction(r0, v0, v.mass, drag=drag, cda=None if drag else cda,
                                                target=self.target, aim_bias=aim_bias, cross_bias=cross_bias,
                                                heading=getattr(self, "heading", None), burns=self.entry_burns(v),
                                                **self._tkw()))
            time.sleep(1.0)
        mid = sorted(cs, key=lambda c: c.along)[len(cs) // 2]
        dev = sorted(abs(c.along - mid.along) for c in cs)
        spread = dev[1] if len(dev) > 1 else 0.0     # robust: tolerate one outlier among three
        return mid, spread

    def coast_and_correct(self, v, cda, tol=300.0, drag=None, max_dv=15.0, aim_bias=-1500.0, min_dv=0.02,
                          cross_bias=0.0):
        """Trim the impact point in vacuum (zero dynamic pressure), then warp to the atmosphere.
        Guards: only above 71 km, median of 3 predictions, distrust noisy predictions, small dv only."""
        rf = v.orbit.body.reference_frame
        gain, last = 0.6, None          # secant-adapted: observed / predicted effect of the last trim
        for i in range(6):
            if v.flight().mean_altitude < 71000:
                self.log("vacuum correction: below 71 km, stop")
                break
            c, spread = self.predict_median(v, cda, drag, aim_bias=aim_bias, cross_bias=cross_bias)
            if last is not None:
                along0, expected = last
                if abs(expected) > 1.0:
                    ratio = (c.along - along0) / expected
                    if 0.2 < ratio < 5.0:
                        gain = min(1.0, max(0.2, gain / ratio * 0.9))
                    self.log("trim effect %.2fx of prediction -> gain %.2f" % (ratio, gain))
            dvn = float(np.linalg.norm(c.dv_bf)) * gain
            self.log("vacuum correction %d: predicted miss %.0fm (along %+.0f cross %+.0f) spread %.0fm, trim %.3f m/s" % (
                i, c.miss, c.along, c.cross, spread, dvn))
            if spread > 500:
                self.log("prediction too noisy, no burn")
                last = None
                continue
            if c.miss < tol or dvn > max_dv or dvn < min_dv:
                break
            got = burn_vector(self.conn, v, np.asarray(c.dv_bf) * gain, self.log, rf, align_timeout=60) or 0.0
            time.sleep(2)
            full = float(np.linalg.norm(c.dv_bf))
            last = (c.along, -c.along * (got / full if full > 1e-6 else 0.0))   # expected effect of what was flown
        alt = self.conn.add_stream(getattr, v.flight(), 'mean_altitude')
        ut_entry = self._time_to_alt(v, 72000) if alt() > 75000 else None
        if ut_entry:
            self.log("warping to entry interface")
            self.coast_to(v, ut_entry - 30, "entry interface")
        alt.remove()

    def _time_to_alt(self, v, h):
        o = v.orbit
        try:
            r = o.body.equatorial_radius + h
            ta = o.true_anomaly_at_radius(r)
            ut = o.ut_at_true_anomaly(-abs(ta)) if False else o.ut_at_true_anomaly(2 * math.pi - abs(ta))
            return ut if ut > self.sc.ut else None
        except Exception:
            return None

    def land(self, v):
        lander = Lander(self.conn, v, target=self.target, log=self.log, overlay_tag="G5 " + self.tag,
                        **self._tkw())
        return lander.run(timeout=1200)
