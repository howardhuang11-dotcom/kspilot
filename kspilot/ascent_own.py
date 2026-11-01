"""Own ascent guidance (replaces MechJeb PVG): gravity turn in the air, apoapsis targeting above it,
then a circularisation burn with kspilot.maneuver + kspilot.burn.

Design (classic, robust; efficiency improvements later):
  1. vertical rise to v_turn, then a small pitch-over kick towards the launch azimuth
  2. gravity turn: hold the surface-prograde direction (zero angle of attack) while in the dense air
  3. above ~70% of the atmosphere: closed-loop pitch on (apoapsis, time to apoapsis): keep the apoapsis
     rising to the target while holding time-to-apoapsis near t_apo_hold; cut at target apoapsis
  4. coast to apoapsis (rails warp out of the atmosphere), circularise (or raise periapsis) with our executor
  5. staging: when an active engine runs dry / available thrust collapses (never past autostage_limit)
Launch azimuth: inertial azimuth from the target inclination, corrected for the surface rotation speed.
Separation watch (DR-007, mishap inc45): after a staging that lights nothing the core must keep its thrust and parts
(else screenshot + abort), and a separation metric is logged (q, flow angles, KSP.log explosions within 3 s; 3 GAME
seconds while recording, HOP_REC=full). Fix round 3: a measurement-only thread (own kRPC connection, never waited for)
also counts the explosions over a 15 s GAME window after a late-flush re-read: 3 s / 15 s, booster-side vs core.
"""
import math
import os
import re
import time
from collections import Counter

import numpy as np

from . import burn, hud, maneuver as mv
from .entry import kepler_propagate


def launch_azimuth(inc_deg, lat_deg, v_orbit, v_rot, northward=True):
    """Surface-relative launch heading (deg from north) for a target inclination."""
    lat, inc = math.radians(lat_deg), math.radians(max(abs(inc_deg), abs(lat_deg)))
    s = max(-1.0, min(1.0, math.cos(inc) / math.cos(lat)))
    beta_i = math.asin(s)                                # inertial azimuth (east of north)
    if not northward:
        beta_i = math.pi - beta_i
    vx = v_orbit * math.sin(beta_i) - v_rot * math.cos(lat)   # east, minus the ground's own speed
    vy = v_orbit * math.cos(beta_i)
    return math.degrees(math.atan2(vx, vy)) % 360.0


BOOSTER_PARTS = ("solidBooster", "noseCone", "radialDecoupler")    # what the booster staging drops; the rest is core


def log_explosions(path, offset):
    """Part names of the "Exploded!!" lines KSP appended to its log after byte offset (a restarted game: from 0)."""
    try:
        with open(path, "rb") as f:
            f.seek(offset if offset <= os.path.getsize(path) else 0)
            text = f.read().decode("utf-8", "replace")
    except OSError:
        return []
    return re.findall(r"\] (\S+) Exploded!!", text)


# ---------------------------------------------------------------------------------------------------------------------
# Separation window metric (fix round 3, decision log 2026-10-21 13:00). MEASUREMENT ONLY - it runs in every mode but
# never changes control: a daemon thread with its OWN kRPC connection (it only reads the game UT), started at the
# staging; the flight loop never waits for it and reads nothing from it. Flight #3 (game 0.4x while recording):
# KSP.log 'solidBooster1-1 Exploded!!' / 'radialDecoupler Exploded!!' ~2 game s after the booster staging, yet the 3 s
# metric read the log too early and reported 0. Now: a 15 s GAME window from the staging, the explosions classified by
# their KSP.log wall-clock stamps (mapped to game time through UT samples), the log re-read once the window has closed
# + 2 s wall (late flush), counts for 3 s and 15 s, booster-side and core separately.
# ---------------------------------------------------------------------------------------------------------------------
SEP_WINDOW_S = 15.0       # game s after the staging
SEP_SHORT_S = 3.0         # game s: the early count (the in-loop metric's window)
SEP_FLUSH_S = 2.0         # wall s: KSP.log is re-read this long after the window has closed
_EXPL_TIMED = re.compile(r"\[\w+ (\d\d):(\d\d):(\d\d(?:\.\d+)?)\] (\S+) Exploded!!")


def log_explosions_timed(path, offset):
    """[(seconds of the local day, part name)] of the "Exploded!!" lines after byte offset (KSP stamps its log lines
    with the local wall-clock time of day)."""
    try:
        with open(path, "rb") as f:
            f.seek(offset if offset <= os.path.getsize(path) else 0)
            text = f.read().decode("utf-8", "replace")
    except OSError:
        return []
    return [(int(h) * 3600 + int(mi) * 60 + float(s), n) for h, mi, s, n in _EXPL_TIMED.findall(text)]


def seconds_of_day(wall):
    """Local time of day (s) of an epoch wall-clock time (the clock KSP.log stamps use)."""
    t = time.localtime(wall)
    return t.tm_hour * 3600 + t.tm_min * 60 + t.tm_sec + (wall - math.floor(wall))


def wall_at(ut, samples):
    """Wall-clock time at game time ut, piecewise-linear through [(ut, wall)] samples (extrapolated at both ends with
    the nearest pair's rate; one sample: rate 1)."""
    pts = sorted(samples)
    if not pts:
        return None
    if len(pts) == 1:
        return pts[0][1] + (ut - pts[0][0])
    for (u0, w0), (u1, w1) in zip(pts, pts[1:]):
        if ut <= u1 or (u1, w1) == pts[-1]:
            if u1 - u0 < 1e-9:
                return w0 + (ut - u0)
            return w0 + (ut - u0) * (w1 - w0) / (u1 - u0)
    return pts[-1][1]


def sep_window_counts(timed, w0, w_short, w_end, tol=0.3):
    """Explosions in the separation window: timed [(seconds of day, name)], w0 / w_short / w_end = wall-clock times of
    the staging, staging + SEP_SHORT_S game s and + SEP_WINDOW_S game s. Midnight-safe; `tol` s of stamp slack."""
    s0 = seconds_of_day(w0)
    d_short, d_end = w_short - w0, w_end - w0
    in_short, in_win = [], []
    for t, name in timed:
        d = (t - s0 + 43200.0) % 86400.0 - 43200.0
        if d < -tol or d > d_end + tol:
            continue
        in_win.append((d, name))
        if d <= d_short + tol:
            in_short.append((d, name))
    core_s, core_w = core_hits([n for _, n in in_short]), core_hits([n for _, n in in_win])
    return dict(explosions_3s=len(in_short), explosions_15s=len(in_win),
                booster_3s=len(in_short) - len(core_s), core_3s=len(core_s),
                booster_15s=len(in_win) - len(core_w), core_15s=len(core_w), events=in_win)


def sep_window_text(c):
    return ("sep window (%.0f s game, KSP.log re-read %.0f s after it closed): explosions_3s %d (booster %d, core %d) "
            "explosions_15s %d (booster %d, core %d)%s" % (
                SEP_WINDOW_S, SEP_FLUSH_S, c["explosions_3s"], c["booster_3s"], c["core_3s"], c["explosions_15s"],
                c["booster_15s"], c["core_15s"],
                (" [" + ", ".join("%s @%+.1f s wall" % (n, d) for d, n in c["events"]) + "]") if c["events"] else ""))


def sep_window_watch(sep, log, ut_fn, wall=time.time, sleep=time.sleep, poll=0.1, window=SEP_WINDOW_S,
                     short=SEP_SHORT_S, flush=SEP_FLUSH_S, max_wall=None):
    """Body of the separation-window thread (pure apart from ut_fn / wall / sleep, so it is testable): sample
    (UT, wall) until the game is `window` s past the staging (or max_wall wall s: game paused / UT unreadable -> the
    rest is mapped at the last rate), wait `flush` wall s, read KSP.log from the staging offset, store the counts in
    sep["window"] and log one line. Never touches the vessel."""
    max_wall = max_wall if max_wall is not None else 3.0 * window + 10.0
    w_stage = sep["t"]
    samples = []
    if sep.get("m") is not None and sep["m"][0] is not None:
        samples.append((float(sep["m"][0]), float(sep["m"][1])))      # recording: the game-clock mark at the staging
    u0 = samples[0][0] if samples else None
    while True:
        try:
            u, w = float(ut_fn()), wall()
            samples.append((u, w))
        except Exception:
            u, w = None, wall()
        if u0 is None and len(samples) >= 2 and samples[-1][1] - samples[0][1] > 0.5:
            (ua, wa), (ub, wb) = samples[0], samples[-1]
            rate = (ub - ua) / (wb - wa) if ub > ua else 1.0
            u0 = ua - (wa - w_stage) * rate                            # the UT at the staging, extrapolated
            samples.insert(0, (u0, w_stage))
        if u0 is not None and u is not None and u >= u0 + window:
            break
        if w - w_stage > max_wall:
            break
        sleep(poll)
    if u0 is None:                                                     # UT never readable: game time = wall time
        samples, u0 = [(0.0, w_stage)], 0.0
    w_short, w_end = wall_at(u0 + short, samples), wall_at(u0 + window, samples)
    w_close = wall()
    while wall() < w_close + flush:                                     # the log is flushed late (flight #3)
        sleep(poll)
    timed = log_explosions_timed(sep["log"], sep["off"]) if sep.get("off") is not None else []
    c = sep_window_counts(timed, w_stage, w_short, w_end)
    sep["window"] = c
    log(sep_window_text(c))
    return c


def start_sep_window(sep, log, connect=None):
    """Start the measurement thread (daemon, own kRPC connection; on failure UT is unreadable and the window falls back
    to the wall clock). Returns the thread."""
    import threading

    def run():
        conn = None
        try:
            from . import core
            conn = (connect or core.connect)("sepwatch")
            ut_fn = lambda: conn.space_center.ut                      # noqa: E731
        except Exception as e:
            log("sep window: no connection (%r), wall clock only" % e)
            ut_fn = lambda: 1 / 0                                     # noqa: E731
        try:
            sep_window_watch(sep, log, ut_fn)
        except Exception as e:
            log("sep window: failed (%r)" % e)
        finally:
            try:
                conn.close()
            except Exception:
                pass

    th = threading.Thread(target=run, name="sepwatch", daemon=True)
    th.start()
    return th


def core_hits(names):
    return [n for n in names if not n.startswith(BOOSTER_PARTS)]


def flow_angles(vel):
    """Surface velocity in the vessel frame (kRPC: x right, y nose, z bottom) -> (total angle of attack, pitch-plane
    AoA, sideslip, crossflow azimuth) in deg. kRPC's angle_of_attack is only the pitch-plane component (mishap inc45).
    The crossflow azimuth is the aerodynamic roll: where the transverse flow sits around the long axis, measured
    from the vessel bottom (z, 0 deg) towards its right side (x, 90 deg)."""
    x, y, z = vel
    spd = max(math.sqrt(x * x + y * y + z * z), 1e-9)
    aoa = math.degrees(math.acos(max(-1.0, min(1.0, y / spd))))
    alpha = math.degrees(math.atan2(z, y))
    beta = math.degrees(math.asin(max(-1.0, min(1.0, x / spd))))
    return aoa, alpha, beta, math.degrees(math.atan2(x, z)) % 360.0


def booster_offset(roll, azimuths):
    """Smallest angle (0-90 deg) between the crossflow plane (azimuth roll) and a booster (azimuths, same convention):
    0 = a booster sits in the plane (pushed straight at the core), 45 with 4 boosters = the flow passes between them."""
    return min(abs((a - roll + 90.0) % 180.0 - 90.0) for a in azimuths) if azimuths else float("nan")


class Ascent:
    def __init__(self, conn, pe_m, ap_m, inc_deg, log, autostage_limit=0, v_turn=100.0, kick_deg=3.0,
                 t_apo_hold=45.0, turn_end_alt=None, profile_alt=70000.0, q_sep_max=None, sep_test_s=None):
        self.conn, self.sc = conn, conn.space_center
        self.v = self.sc.active_vessel
        self.pe, self.ap, self.inc = pe_m, ap_m, inc_deg
        self.log, self.limit = log, autostage_limit
        self.v_turn, self.kick, self.t_hold = v_turn, kick_deg, t_apo_hold
        body = self.v.orbit.body
        self.body = body
        self.atm = body.atmosphere_depth if body.has_atmosphere else 0.0
        self.turn_end = turn_end_alt or 0.7 * self.atm
        # KSP_Q_SEP_MAX (Pa) overrides the booster separation gate (DR-007 test B)
        self.profile_alt = profile_alt
        self.q_sep_max = q_sep_max if q_sep_max is not None else float(os.environ.get("KSP_Q_SEP_MAX", 10000.0))
        self.sep_test_s = sep_test_s      # separation test (DR-007): end the flight this long after the separation
        self.sep, self.lost = None, None  # record of the last staging that lit nothing; abort reason

    def _stage_if_needed(self, last_thrust):
        """Stage only when it is useful and safe: there are dry active engines that the next stage drops,
        or nothing is burning and the next stage lights engines. Never drop an engine that still has fuel
        (the first version staged straight through the core and the payload). 2 s between stagings."""
        v = self.v
        now = time.time()
        if now - getattr(self, "_t_stage", 0.0) < 2.0:
            return False
        cur = v.control.current_stage
        if cur <= self.limit or cur <= 0:
            return False
        nxt = cur - 1
        active = [e for e in v.parts.engines if e.active]
        dry = [e for e in active if not e.has_fuel]
        dropped = set(p._object_id for p in v.parts.in_decouple_stage(nxt)) if hasattr(v.parts, "in_decouple_stage") else set()
        lights = [p for p in v.parts.in_stage(nxt) if p.engine is not None]
        if any(e.part._object_id in dropped and e.has_fuel for e in active):
            return False                                   # would throw away a burning engine
        useful = (dry and any(e.part._object_id in dropped for e in dry)) or (not [e for e in active if e.has_fuel] and lights)
        if not useful:
            return False
        burning = [e for e in active if e.has_fuel]
        if burning and not lights:
            # only dropping dead weight: wait for low dynamic pressure (empty boosters released at 35 kPa, and at 20 kPa
            # on heading 41.7, tumbled into the core and destroyed it; 10 kPa: 0 collisions in 3 tests, DR-007)
            q = v.flight().dynamic_pressure
            if q > self.q_sep_max:
                return False
        if dry and not lights and not getattr(self, "_srb_sep", False):
            self._srb_sep = True
            hud.report("event", "SRB SEP", self.sc.ut)
        self.log("ascent: staging %d -> %d (%d dry engines dropped, %d engines to light)" % (
            cur, nxt, sum(e.part._object_id in dropped for e in dry), len(lights)))
        try:
            from . import core
            core.screenshot("ascent_stage%d_before" % cur, clean=False)
        except Exception:
            pass
        if not lights:
            self.sep = self._sep_record(v, dropped, [e.part for e in dry if e.part._object_id in dropped])
            start_sep_window(self.sep, self.log)    # measurement thread only (fix round 3): no control effect
        v.control.activate_next_stage()
        self._t_stage = now
        self._thr_ref, self._t_weak = 0.0, None        # thrust-collapse reference restarts with the new stage
        try:
            time.sleep(1.0)
            from . import core
            core.screenshot("ascent_stage%d_after" % cur, clean=False)
        except Exception:
            pass
        return True

    def _sep_record(self, v, dropped, boosters):
        """State just before a staging that lights nothing: reference thrust and parts for the core-loss check, and
        the separation metric inputs (q, flow angles, booster azimuths, KSP.log size)."""
        from . import core
        parts = len(v.parts.all)
        s = dict(t=time.time(), thr0=v.thrust, parts0=parts, expected=parts - len(dropped) if dropped else None,
                 q=v.flight().dynamic_pressure, aoa=float("nan"), beta=float("nan"), roll=float("nan"),
                 offset=float("nan"), log=os.path.join(core.KSP_DIR, "KSP.log"), off=None, parts=None, metric=None,
                 m=core.GAME_CLOCK.mark() if core.GAME_CLOCK is not None else None)   # recording v2: game clock
        try:
            s["off"] = os.path.getsize(s["log"])
        except OSError:
            pass
        try:
            body = self.body.reference_frame
            vel = self.sc.transform_direction(v.flight(body).velocity, body, v.reference_frame)
            s["aoa"], _, s["beta"], s["roll"] = flow_angles(vel)
            az = []
            for p in boosters:
                x, _, z = p.position(v.reference_frame)
                az.append(math.degrees(math.atan2(x, z)))
            s["offset"] = booster_offset(s["roll"], az)
        except Exception as e:
            self.log("sep metric: attitude unavailable (%r)" % e)
        return s

    @staticmethod
    def _sep_age(sep):
        """Seconds since the separation record: wall clock (as before); recording v2 (core.GAME_CLOCK, HOP_REC=full
        only): GAME seconds with a 3x wall cap. Flight #2 (game ~0.4x while recording) logged the metric 1.4 game s
        after the staging and missed a booster-booster explosion ~2 game s after it (screenshots/qa_v2_10)."""
        if sep.get("m") is None:
            return time.time() - sep["t"]
        from . import core
        return core.GAME_CLOCK.elapsed(sep["m"])

    @staticmethod
    def _sep_wait(sep, window):
        """Sleep until the separation record is `window` s old (same clock as _sep_age)."""
        if sep.get("m") is None:
            time.sleep(max(0.0, window - (time.time() - sep["t"])))
            return
        from . import core
        core.GAME_CLOCK.sleep(max(0.0, window - core.GAME_CLOCK.elapsed(sep["m"])))

    def _sep_metric(self):
        """One line per separation: q, flow angles and the parts KSP reported exploded since the staging."""
        s = self.sep
        names = log_explosions(s["log"], s["off"]) if s["off"] is not None else []
        s["metric"], s["core"] = names, core_hits(names)
        n = Counter(names)
        self.log("sep metric: q %.1f kPa roll %.1f deg sideslip %.1f deg aoa %.1f deg explosions %d [%s] "
                 "booster offset %.1f deg%s" % (
                     s["q"] / 1e3, s["roll"], s["beta"], s["aoa"], len(names),
                     ", ".join("%s x%d" % kv for kv in n.items()), s["offset"],
                     " CORE HIT: %s" % ", ".join(sorted(set(s["core"]))) if s["core"] else ""))

    def _core_lost(self, v):
        """Abort reason if the core was lost, else None (DR-007; inc45: an SRB hit the core tank at separation, the
        engine fell off and the guidance flew 130 s at 0 kN). For 5 s after the first look following a staging that
        lit nothing: thrust >= 50 % of the pre-staging thrust and no further part loss. Always: throttle up but
        thrust < 10 % of the available thrust seen since the last staging for 2 s."""
        now, thr = time.time(), v.thrust
        s = self.sep
        if s is not None and not s.get("done"):
            parts = len(v.parts.all)
            if s["parts"] is None:                 # first look after the staging (1 s pause + screenshot)
                s["parts"], s["t1"] = parts, now
                self.log("sep parts: %d before, expected %s after, now %d, thrust %.0f -> %.0f kN%s" % (
                    s["parts0"], s["expected"], parts, s["thr0"] / 1e3, thr / 1e3,
                    "  ANOMALY: unplanned part loss" if s["expected"] is not None and parts != s["expected"] else ""))
            low = thr < 0.5 * s["thr0"]
            s["t_low"] = (s.get("t_low") or now) if low else None
            if parts < s["parts"] or (low and now - s["t_low"] >= 0.5):
                return "lost core at staging (parts %d, thrust %.0f kN)" % (parts, thr / 1e3)
            s["done"] = now - s["t1"] > 5.0
        avail, throttle = v.available_thrust, v.control.throttle
        self._thr_ref = max(getattr(self, "_thr_ref", 0.0), avail)
        if throttle > 0.0 and thr < 0.1 * throttle * self._thr_ref:
            self._t_weak = getattr(self, "_t_weak", None) or now
            if now - self._t_weak >= 2.0:
                return "lost core: thrust collapse (parts %d, thrust %.0f kN, throttle %.2f, available %.0f kN)" % (
                    len(v.parts.all), thr / 1e3, throttle, self._thr_ref / 1e3)
        else:
            self._t_weak = None
        return None

    def fly(self, timeout=1800):
        v, sc = self.v, self.sc
        lat = v.flight().latitude
        R = self.body.equatorial_radius
        v_orb = math.sqrt(self.body.gravitational_parameter / (R + self.pe))
        v_rot = self.body.rotational_speed * R
        hdg = launch_azimuth(self.inc, lat, v_orb, v_rot)
        self.log("ascent (own): target Pe %.0f km Ap %.0f km i %.1f deg -> launch heading %.1f deg" % (
            self.pe / 1e3, self.ap / 1e3, self.inc, hdg))
        ap = v.auto_pilot
        ap.reference_frame = v.surface_reference_frame
        ap.target_pitch_and_heading(90.0, hdg)
        ap.target_roll = float("nan")
        ap.engaged = True
        v.control.throttle = 1.0
        if str(v.situation).endswith("pre_launch"):
            v.control.activate_next_stage()
        t0 = sc.ut
        hud.report("phase", "ascent")
        hud.report("event", "LIFTOFF", t0)
        for name, dt in (("MAX-Q", 42.0), ("SRB SEP", 51.0), ("MECO", 180.0)):   # nominal (13:43 flight) until live
            hud.report("plan", name, t0 + dt)
        srf = v.flight(self.body.reference_frame)
        last_thrust, t0, phase = v.available_thrust, time.time(), "rise"
        while time.time() - t0 < timeout:
            cur = sc.active_vessel                      # staging can hand the old vessel id to a dropped booster
            if cur._object_id != v._object_id:
                self.log("ascent: active vessel changed after staging -> following %s" % cur.name)
                v = self.v = cur
                ap = v.auto_pilot
                ap.reference_frame = v.surface_reference_frame
                ap.target_roll = float("nan")
                ap.engaged = True
                srf = v.flight(self.body.reference_frame)
            self.lost = self._core_lost(v)
            sep = self.sep
            if sep is not None and sep["metric"] is None and (self.lost or self._sep_age(sep) > 3.0):
                self._sep_wait(sep, 3.0)                    # an abort lets the explosions reach KSP.log
                self._sep_metric()
            test_over = self.sep_test_s and sep is not None and time.time() - sep["t"] > self.sep_test_s
            if self.lost or test_over:
                if sep is not None:    # end state for the separation test (whole window: late log lines, later hits)
                    names = log_explosions(sep["log"], sep["off"]) if sep["off"] is not None else []
                    sep.update(parts_end=len(v.parts.all), thr_end=v.thrust, n_window=len(names),
                               core_window=core_hits(names))
                if self.lost:
                    self.log("ascent ABORT: %s" % self.lost)
                    try:
                        from . import core
                        core.screenshot("ascent_core_lost", clean=False)
                    except Exception:
                        pass
                    return False
                v.control.throttle = 0.0
                ap.engaged = False
                self.log("ascent: separation test over %.0f s after staging - engine off, no recovery" % (
                    time.time() - sep["t"]))
                return True
            o = v.orbit
            alt = v.flight().mean_altitude
            spd = srf.speed
            if time.time() - getattr(self, "_t_tel", 0.0) > 2.0:     # ascent telemetry
                self._t_tel = time.time()
                fl = v.flight(v.surface_reference_frame)
                try:
                    aoa = v.flight().angle_of_attack
                except Exception:
                    aoa = float("nan")
                self.log("ascent tel: %s alt %.1f km v %.0f m/s pitch %.1f aoa %.1f Ap %.1f km stage %d parts %d thr %.0f kN" % (
                    phase, alt / 1e3, spd, fl.pitch, aoa, o.apoapsis_altitude / 1e3, v.control.current_stage,
                    len(v.parts.all), v.thrust / 1e3))
            if self._stage_if_needed(last_thrust):
                last_thrust = v.available_thrust
                continue
            last_thrust = max(v.available_thrust, 1.0)
            if phase == "rise":
                if spd > self.v_turn:
                    phase = "kick"
                    self.log("ascent: pitch-over at %.0f m/s, %.0f m" % (spd, alt))
                ap.target_pitch_and_heading(90.0, hdg)
            elif phase == "kick":
                ap.target_pitch_and_heading(90.0 - self.kick, hdg)
                # hand over to prograde once the velocity has followed the kick
                vel = np.asarray(v.velocity(v.surface_reference_frame))
                pitch_v = math.degrees(math.asin(max(-1.0, min(1.0, vel[0] / max(np.linalg.norm(vel), 1e-6)))))
                if pitch_v < 90.0 - self.kick:
                    phase = "turn"
            if phase == "turn":
                vel = np.asarray(v.velocity(v.surface_reference_frame))    # (up, north, east)
                pitch_v = math.degrees(math.asin(max(-1.0, min(1.0, vel[0] / max(np.linalg.norm(vel), 1e-6)))))
                # never let a low-TWR gravity turn collapse: floor the pitch with a smooth altitude profile
                floor = max(5.0, 90.0 - 85.0 * (max(alt, 0.0) / self.profile_alt) ** 0.6)
                ap.target_pitch_and_heading(max(pitch_v, floor, 0.0), hdg)
                if alt > self.turn_end:
                    phase = "apoapsis"
                    self.log("ascent: closed-loop apoapsis phase at %.0f km (Ap %.0f km)" % (alt / 1e3, o.apoapsis_altitude / 1e3))
            if phase == "apoapsis":
                t_apo = o.time_to_apoapsis if o.time_to_apoapsis < o.period / 2 else -1.0
                err_t = self.t_hold - t_apo
                pitch = max(-5.0, min(30.0, 0.5 * err_t))         # too close to apoapsis -> pitch up
                ap.target_pitch_and_heading(pitch, hdg)
                remain = self.ap - o.apoapsis_altitude
                v.control.throttle = float(max(0.05, min(1.0, remain / 5000.0)))
                # live MECO prediction from the apoapsis growth rate
                ut_now, ap_now = sc.ut, o.apoapsis_altitude
                if getattr(self, "_ap_prev", None) is not None and ut_now > self._ap_prev[0] + 0.5:
                    rate = (ap_now - self._ap_prev[1]) / (ut_now - self._ap_prev[0])
                    if rate > 1.0:
                        hud.report("plan", "MECO", ut_now + remain / rate)
                    self._ap_prev = (ut_now, ap_now)
                elif getattr(self, "_ap_prev", None) is None:
                    self._ap_prev = (ut_now, ap_now)
                if remain <= 50.0:
                    v.control.throttle = 0.0
                    hud.report("event", "MECO", sc.ut)
                    self.log("ascent: MECO, Ap %.1f km" % (o.apoapsis_altitude / 1e3))
                    break
            if str(v.situation).endswith(("landed", "splashed")) and time.time() - t0 > 30:
                self.log("ascent ABORT: back on the surface")
                return False
            time.sleep(0.1)
        v = self.v = sc.active_vessel
        v.auto_pilot.engaged = False
        # coast to apoapsis (out of the atmosphere), then shape the orbit with our executor
        r, vel, mu = burn.state(v)
        t = mv.time_to_radius_extremum(r, vel, mu, apo=True)
        r1, v1 = kepler_propagate(r, vel, t, mu)
        R = self.body.equatorial_radius
        dv = mv.dv_change_periapsis(r1, v1, mu, R + self.pe) if self.ap - self.pe > 2000 else mv.dv_circularize(r1, v1, mu)
        self.log("ascent: circularisation %.1f m/s at apoapsis in %.0f s" % (float(np.linalg.norm(dv)), t))
        hud.report("plan", "ORBIT", sc.ut + t)
        burn.execute_burn(self.conn, v, sc.ut + t, dv, self.log, autostage_limit=self.limit)
        hud.report("event", "ORBIT", sc.ut)
        hud.report("phase", "orbit")
        o = v.orbit
        self.log("ascent: orbit Ap %.1f km Pe %.1f km i %.2f deg" % (o.apoapsis_altitude / 1e3,
                                                                     o.periapsis_altitude / 1e3, math.degrees(o.inclination)))
        return o.periapsis_altitude > self.atm
