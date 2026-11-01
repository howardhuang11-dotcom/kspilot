"""Broadcast director for whole-mission recordings (shot list, storyboard and rationale: director shot list).

Inert unless started: tools/orbit_recover.py starts it only with HOP_REC=full. One daemon thread with its OWN
kRPC connection reads the mission state the flight code already reports to the HUD feed (hud.current: events and
plans) plus a few cheap vessel reads, picks the shot from SHOTS (first match wins) and hands it to the AutoLab
plugin (shot.txt) or to the stock kRPC camera. No match = dead air: frame capture pauses (rec.pause) and the
video simply cuts to the next shot. It never commands the vessel, the throttle, staging or time warp.

Unified show timeline (show-timeline notes, kspilot/showrun.py): the HUD widget transitions are planned from the same
mission events; Cutter never cuts within 2 s (ORBIT icon: 3 s) of one and waits until its animation is over (HUD
first), never pauses the capture over one, and makes no non-urgent cut at MAX-Q. All its clocks are game UT.

Landing shots S7-S9 stay with Lander.direct_camera (flight-proven, timed by showrun.landing_shot); handoff() is
called right before the lander runs and the director then stays out of the way.
"""
import atexit
import math
import threading
import time
from collections import namedtuple

from . import core, hud, showrun

R_KERBIN = 600000.0
# the landing ground camera (Lander.direct_camera): 160 m south, 380 m west of the pad, 12 m up. Proven clear
# line of sight to the pad in the landing videos, so the liftoff shot uses the same spot.
PAD_CAM_OFFSET = (-160.0, -380.0, 12.0)          # north, east, up (m)
MIN_HOLD = 3.0          # s: no cut sooner than this after the previous one (urgent shots excepted)
REC_TAIL = 3.0          # s: frames still captured after the last shot stopped matching (no clipped endings)
PERIOD = 0.25           # s: director tick

# guard: a burn or its attitude alignment is running or due (throttle/thrust up, autopilot steering, a node or a
# predicted burn within GUARD_S): the capture is never paused then (mishap review 2026-10-21)
State = namedtuple("State", "ut met ev h_pad burning rails srb ascent_over rng elev az guard burn_end p7 burn_start",
                   defaults=(False, None, False, None))   # burn_end, p7, burn_start: v2 only (UT of the last > 1 s
                                                          # burn's cutoff, P7 disk-saving pause allowed, UT the
                                                          # current thrust period started - fix round 3)
GUARD_S = 150.0         # s before a node / predicted burn from which the capture stays on (alignment takes <= 90 s)


def guard_window(ut, node_uts, ev):
    """A maneuver node or a predicted circularisation / deorbit burn within GUARD_S s (or just passed)."""
    due = list(node_uts) + [u for n, (u, done) in ev.items() if n in ("ORBIT", "DEORBIT BURN") and not done]
    return any(-30.0 < u - ut < GUARD_S for u in due)
Shot = namedtuple("Shot", "id name when cam urgent caption")


def pad_camera(target=core.KSC_PAD, pad_alt=None):
    """(lat, lon, alt) of the pad tracking camera."""
    if pad_alt is None:
        pad_alt = hud.core_pad_alt()
    lat0, lon0 = target
    n, e, u = PAD_CAM_OFFSET
    return (lat0 + math.degrees(n / R_KERBIN),
            lon0 + math.degrees(e / (R_KERBIN * math.cos(math.radians(lat0)))), pad_alt + u)


def look_angles(cam, lat, lon, alt):
    """Range (m), elevation (deg) and azimuth (deg from north) from a ground camera to the vessel (sphere)."""
    clat, clon, calt = cam
    d = core.great_circle_distance(R_KERBIN, clat, clon, lat, lon)
    dh = alt - calt
    p1, p2, dl = math.radians(clat), math.radians(lat), math.radians(lon - clon)
    az = math.degrees(math.atan2(math.sin(dl) * math.cos(p2),
                                 math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl))) % 360.0
    return math.hypot(d, dh), math.degrees(math.atan2(dh - d * d / (2 * R_KERBIN), d)), az


# ---- event helpers over the HUD feed's events {name: (ut, done)} ----
def done(s, name):
    return name in s.ev and s.ev[name][1]


def since(s, name):
    """Seconds since the event happened (None if it has not)."""
    return s.ut - s.ev[name][0] if done(s, name) else None


def until(s, name):
    """Seconds to a predicted event (None if not predicted or already happened)."""
    return s.ev[name][0] - s.ut if name in s.ev and not s.ev[name][1] else None


def _recent(s, name, hold):
    t = since(s, name)
    return t is not None and t < hold


def _booster_sep(s):
    """Pre-roll 2.5 s before the predicted SRB separation (only while boosters burn) and hold 7 s after it."""
    t = until(s, "SRB SEP")
    return _recent(s, "SRB SEP", 7.0) or (s.srb > 0 and t is not None and -8.0 < t < 2.5)


def _pad_tracking(s):
    """From the pre-roll on the pad to the fixed S1 -> S2 cut at T+35 s (unified timeline, show-timeline notes); the
    auto-zoom lens keeps the stage framed. Safety exit only if the stage is far out of the lens (15 km)."""
    return not s.ascent_over and (s.met is None or (s.met < showrun.S1_END and s.rng < 15000.0))


def _burn(s):
    """Any burn after MECO that lasts > 1 s, with a pre-roll before the predicted circularisation / deorbit burn."""
    pre = any(t is not None and -2.0 < t < 10.0 for t in (until(s, "ORBIT"), until(s, "DEORBIT BURN")))
    return s.ascent_over and (s.burning or pre)


def _payload(s):
    """Frame the stack between orbit insertion and the separation, hold 12 s after it."""
    return done(s, "ORBIT") and (not done(s, "PAYLOAD SEP") or _recent(s, "PAYLOAD SEP", 12.0))


def _ascent(s):
    return s.met is not None and (not s.ascent_over or _recent(s, "MECO", 6.0))


def _ascent_cam(s):
    """3/4 rear chase that slowly widens and swings to the side as the stack climbs (eased in the plugin)."""
    k = min(max(s.h_pad / 40000.0, 0.0), 1.0)
    q = lambda x: 5 * round(x / 5)     # noqa: E731  coarse steps, the plugin eases between them
    return "rel %d %d %d 55" % (q(45 + 75 * k), 6 + round(6 * k), q(25 + 35 * k))


def _pad_cam(s):
    return "ground %.6f %.6f %.1f 14 80" % pad_camera()


# Priority order: the first shot whose `when` matches is on air. `cam` is the AutoLab shot string, or
# ("orbital", distance, pitch) for the stock kRPC camera. Landing shots S7-S9: Lander.direct_camera.
SHOTS = [
    Shot("S3", "BOOSTER SEP · ONBOARD", _booster_sep, lambda s: "onboard 2 3 2 75", True, False),
    Shot("S1", "PAD TRACKING CAMERA", _pad_tracking, _pad_cam, True, True),
    Shot("S4", "ENGINE BURN · SIDE", _burn, lambda s: "rel 40 6 100 50", True, False),
    Shot("S6", "ORBIT · WIDE (WARP)", lambda s: s.rails, lambda s: ("orbital", 300.0, 20.0), False, False),
    Shot("S5", "PAYLOAD SEP · SIDE", _payload, lambda s: "rel 30 12 70 50", False, False),
    Shot("S2", "ASCENT CHASE", _ascent, _ascent_cam, False, False),
]


def pick(s, table=SHOTS):
    """The shot on air for state s (None = dead air: pause the recording)."""
    for shot in table:
        if shot.when(s):
            return shot
    return None


class Cutter:
    """Shot sequencer on the unified show timeline (pure, game-UT clock; tests/test_showrun.py replays it).
    step(s, txs) -> list of actions: ("cut", old_id, new_id) | ("pause",) | ("resume",).
      - the shot table decides what should be on air (pick)
      - MIN_HOLD between cuts (urgent shots excepted); no non-urgent cut within 2 s of MAX-Q
      - HUD first: no cut (urgent or not, and no resume from a pause) within the no-cut window of a HUD transition
        (showrun.cut_ok); the shot on air holds until the animation is over
      - dead air pauses the capture after REC_TAIL, but never over a HUD transition (showrun.pause_ok): capture
        resumes PAUSE_LOOK s before one, on the last camera."""

    def __init__(self, table=SHOTS):
        self.table = table
        self.cur = None            # shot on air (None: dead air / paused)
        self.t_cut = -1e9          # UT of the last cut
        self.t_live = None         # UT the table last had a shot
        self.paused = False

    def step(self, s, txs):
        ut, acts = s.ut, []
        want = pick(s, self.table)
        if self.t_live is None:
            self.t_live = ut
        if want is not None and (self.cur is None or want.id != self.cur.id):
            hold = (not showrun.cut_ok(ut, txs)
                    or (not want.urgent and self.cur is not None and ut - self.t_cut < MIN_HOLD)
                    or (not want.urgent and self.cur is not None and showrun.maxq_quiet(ut, s.ev)))
            if hold:
                want = self.cur                     # HUD first / too soon: the shot on air stays
        if want is not None:
            self.t_live = ut
            if self.cur is None or want.id != self.cur.id:
                acts.append(("cut", self.cur.id if self.cur else None, want.id))
                self.t_cut, self.cur = ut, want
            if self.paused:
                acts.append(("resume",))
                self.paused = False
        elif self.paused:
            if not showrun.pause_ok(ut, txs) or s.guard:   # a HUD transition / burn or alignment: capture (last camera)
                acts.append(("resume",))
                self.paused, self.t_live = False, ut
        elif ut - self.t_live > REC_TAIL and s.ascent_over and showrun.pause_ok(ut, txs) and not s.guard:
            acts.append(("pause",))                 # dead air (drag probes, planning, waits before burns)
            self.paused, self.cur = True, None
        return acts


# ---- v2 (HOP_REC=full, director v2 specification): fixed ascent rotation, no dead-air pauses outside P7 ----
def _pad_cam_v2(s):
    """S1: fixed 14 deg lens during the countdown (spec 5), auto-zoom on the stage after lift-off (same camera: the
    plugin eases the lens, no cut)."""
    lat, lon, alt = pad_camera()
    return "ground %.6f %.6f %.1f 14" % (lat, lon, alt) if s.met is None else "ground %.6f %.6f %.1f 14 80" % (lat, lon, alt)


V2_CAM = {
    "S1": _pad_cam_v2,
    "S2a": lambda s: "rel 45 6 25 55",          # rear chase
    "S2b": lambda s: "rel 60 8 90 55",          # side
    "S3": lambda s: "onboard 2 3 2 75",         # booster separation, hull camera (flight-proven)
    "S2c": _ascent_cam,                         # chase widening (flight-proven S2)
    "ONB": lambda s: "onboard 0 6 2.5 70",      # hull camera looking down: the planet falls away (new parameters)
    "S2d": lambda s: "rel 80 10 90 55",         # side at 40 km: curving horizon
    "S2f": lambda s: "rel 50 -5 160 55",        # from ahead, looking back (new parameters)
    "S2e": lambda s: "rel 24 6 70 45",          # MECO close-up (fix round 3, QA: 5 % of the width at 60 m from the
                                                # rear): 24 m, 3/4 side, 45 deg lens -> ~23 % (stage_frame_fraction);
                                                # the sun solver keeps it on the side (SUN_SIDE): whole stage, lit side
    "BRIDGE": lambda s: "rel 60 20 20 50",      # orbital bridge (replaces the stock orbital camera S6)
    "S4": lambda s: "rel 40 6 100 50",          # insertion burn, side
    # fix round 2 (QA 2026-10-21 defects 2/3/5; "cut" = hard cut between two rel shots, "sun" = az from the sun's
    # direction, camera on the lit side - AutoLab plugin; an older plugin ignores both tokens)
    "S4b": lambda s: "rel 50 15 25 50 cut",     # insertion: cutoff + 10 s (P4 tail) and the P5 lapse, 3/4 rear high
    "S4c": lambda s: "rel 45 -5 -115 50 cut",   # insertion burn, partner angle (fix round 3, V8): low, the other
                                                # side, slightly ahead - S4 is split before it can pass 15 s
    "S4D": lambda s: "rel 16 8 125 60 cut",     # deorbit burn, night side (fix round 3; QA: the hull camera measured
                                                # mean luma 4.9): a tight engine camera 16 m off the stage, ahead and
                                                # to the side - the retro burn points the engine prograde, so the lit
                                                # plume (the only light there) crosses the frame next to the bell; the
                                                # editor adds the deorbit graphic when the window still measures dark
    "S4E": lambda s: "rel 30 15 25 60 cut",     # deorbit cutoff + 5 s: 3/4 rear looking prograde and down at the limb
                                                # (the dawn side: sunrise ~3 min later), stars above
    "S5": lambda s: "rel 18 10 35 50 sun cut",  # payload release close-up, lit side (QA: was a dark silhouette at 30 m)
    "S5b": lambda s: "rel 35 15 -40 50 sun cut",  # payload drifting away: the other side of the sun line, wider
}
V2_NAME = {"S1": "PAD TRACKING CAMERA", "S2a": "ASCENT · REAR", "S2b": "ASCENT · SIDE", "S3": "BOOSTER SEP · ONBOARD",
           "S2c": "ASCENT CHASE", "ONB": "ONBOARD · DOWN", "S2d": "ASCENT · SIDE HIGH", "S2f": "ASCENT · FRONT",
           "S2e": "ASCENT · MECO CLOSE-UP", "BRIDGE": "ORBIT · BRIDGE", "S4": "ENGINE BURN · SIDE",
           "S5": "PAYLOAD · SIDE", "S5b": "PAYLOAD · WIDE", "S4b": "ORBIT INSERTION · CUTOFF",
           "S4D": "DEORBIT BURN · ENGINE", "S4E": "DEORBIT BURN · CUTOFF", "S4c": "ENGINE BURN · LOW"}
V2_URGENT = ("S1", "S3", "S4", "S4c", "S4b", "S4D", "S4E")
V2_SPLIT = {"S4": "S4c", "S4D": "S4E"}   # burn shot -> partner angle (long burns alternate); fix round 3: the insertion
                                         # partner is its own angle S4c, S4b only holds the cutoff (P4 tail + P5 lapse)
BURN_CHAIN = ("S4", "S4c", "S4b", "S4D", "S4E")
# V8 fix round 3 (QA flight #3: S4 17.1 s). The ORBIT event is reported at the insertion cutoff and its HUD icon
# window (no cut from ORBIT - 1 to ORBIT + 5 s) then holds the burn shot on air 5 s past the cutoff. An insertion
# burn shot therefore splits after at most INS_SPLIT s on air (earlier - at half its predicted length - when the
# cutoff predicted from the node would push it past 14 s), so even a cutoff right before the split leaves it <= 15 s.
# The predicted ORBIT window (at the burn centre) is never drawn by the plugin (non-firm): burn-chain cuts respect
# only firm HUD windows.
INS_HOLD = showrun.ORBIT_SHRINK + showrun.KIND["ORBIT_ICON"][1]          # 5 s
INS_SPLIT = showrun.MAX_HOLD["event"] - INS_HOLD - 1.5                     # 8.5 s
SUN_SIDE = ("S2e",)      # the sun solver keeps these on the side (|sin az| >= showrun.SIDE_MIN_SIN)
SUN_REFRESH = 60.0       # game s (and >= 2 wall s: warps): a long rel shot without a 'cut' token (BRIDGE) re-reads
                         # the sun geometry
POST_BURN_HOLD = 5.0     # s: S4 stays on after a burn's cutoff
BURN_PREROLL = 6.0       # s before the predicted insertion / deorbit node (= burn centre): S4. Ignition is node -
                         # burn/2, so the S4 pre-roll is ~2-4 s (event close-up <= 15 s, SHOT_LENGTH_RESEARCH) and the
                         # cut falls inside the 1x window that starts 10 s before ignition, never in a time-lapse


def _deorbit_regime(s):
    """After the payload release (or with a deorbit burn planned, a run from the g5_orbit checkpoint)."""
    return done(s, "PAYLOAD SEP") or "DEORBIT BURN" in s.ev


def pick_v2(s):
    """(shot id, urgent, UT of the next scheduled change or None) for the v2 director (spec 8.1 / 8.2).
    Fix round 2 (V8 event shots <= 15 s, QA 2026-10-21): insertion S4 (pre-roll + burn) -> S4b from the cutoff
    through cutoff + 10 s (P4 tail) and the P5 lapse -> S5 from 10 s before the announced release to release + 2 s ->
    S5b to release + 10 s; deorbit S4D (hull camera: night side) -> S4E from the cutoff to cutoff + 5 s.
    Fix round 3: S4D is an engine camera; long insertion burns alternate S4 / S4c (CutterV2.split_limit)."""
    if _booster_sep(s):
        return "S3", True, None
    in_ascent = (not s.ascent_over) or (done(s, "MECO") and since(s, "MECO") < showrun.MECO_TAIL)
    if in_ascent:
        sid, nxt = showrun.ascent_shot_v2(s.met)
        t0 = s.ut - s.met if s.met is not None else None
        nxt_ut = t0 + nxt if t0 is not None else None
        u = until(s, "SRB SEP")
        if s.srb > 0 and u is not None:                      # S3 pre-roll starts 2.5 s before the separation
            nxt_ut = min(nxt_ut, s.ut + u - 2.5) if nxt_ut is not None else s.ut + u - 2.5
        return sid, sid == "S1", nxt_ut
    pre = any(t is not None and -2.0 < t < BURN_PREROLL for t in (until(s, "ORBIT"), until(s, "DEORBIT BURN")))
    deo = _deorbit_regime(s)
    hold = POST_BURN_HOLD if deo else showrun.POST_ORBIT      # P8 cutoff + 5 s / P4 cutoff + 10 s
    held = s.burn_end is not None and s.ut - s.burn_end < hold
    if s.burning or (pre and not held):     # fix round 3: a node logged AFTER the cutoff (flight #3 deorbit: HUD node
        return ("S4D" if deo else "S4"), True, None     # 7.5 s after it) must not re-open the pre-roll in the hold
    if held:
        return ("S4E" if deo else "S4b"), True, None
    if done(s, "ORBIT") and since(s, "ORBIT") < POST_BURN_HOLD and not deo:
        return "S4b", False, None
    if done(s, "PAYLOAD SEP"):
        t = since(s, "PAYLOAD SEP")
        return ("S5" if t < showrun.S5_AFTER_SEP else "S5b" if t < showrun.SEP_WIN else "BRIDGE"), False, None
    u = until(s, "PAYLOAD SEP")
    if u is not None and u <= showrun.SEP_WIN:             # announced release (recording v2): its 1x window
        return "S5", False, None
    if done(s, "ORBIT"):
        return "S4b", False, None                           # P4 tail + P5 lapse until the release is announced
    return "BRIDGE", False, None


class CutterV2:
    """v2 shot sequencer (pure, game-UT clock). step(s, txs) -> [("cut", old, new) | ("pause",) | ("resume",)].
      - pick_v2 decides the shot; urgent shots (V2_URGENT) cut at once, others wait MIN_SHOT after the previous
        cut, never cut at MAX-Q, and are skipped when a scheduled change follows within MIN_SHOT (no 1-2 s shots)
      - HUD first: no cut in a HUD-transition window (showrun.cut_ok)
      - V9: the capture is NEVER paused outside P7 (release + 10 s -> deorbit); inside P7 (s.p7, after the 4 s
        bridge footage) it pauses only to save disk, the camera stays on the bridge shot and the editor skips the
        whole span anyway; any burn / alignment guard resumes it.
      - V8 (fix round 2): a burn shot on air longer than EVENT_SPLIT while still burning cuts to its partner angle
        (V2_SPLIT) and back, so no event close-up passes 15 s.
      - V8 (fix round 3): insertion burn shots (S4 / S4c) split after split_limit() s (<= INS_SPLIT) because the
        cutoff's HUD window holds them 5 s longer; burn-chain cuts respect only firm HUD windows."""

    def __init__(self):
        self.cur = None
        self.t_cut = -1e9
        self.paused = False
        self.alt_on = False        # the shot on air is a long burn's partner angle (V2_SPLIT)

    def split_limit(self, s, shot):
        """Seconds a burn shot may stay on air while the burn goes on. Deorbit: EVENT_SPLIT. Insertion (fix round 3):
        INS_SPLIT, or half the predicted on-air time of S4 (pre-roll + burn + INS_HOLD) when that would exceed
        MAX_HOLD event - 1 s; the cutoff is predicted as 2 x node - ignition (the executor centres the burn on the
        node, which the HUD plans as the ORBIT event)."""
        if shot not in ("S4", "S4c"):
            return showrun.EVENT_SPLIT
        node = s.ev.get("ORBIT")
        if shot != "S4" or self.alt_on or s.burn_start is None or node is None or node[1]:
            return INS_SPLIT
        tc = 2.0 * node[0] - s.burn_start                       # predicted cutoff
        total = tc + INS_HOLD - self.t_cut
        if total > showrun.MAX_HOLD["event"] - 1.0:
            return min(max(0.5 * total, showrun.MIN_SHOT), INS_SPLIT)   # split it in two even halves
        return max(INS_SPLIT, tc + 1.0 - self.t_cut)            # it fits: split only if the burn overruns 1 s

    def step(self, s, txs):
        ut, acts = s.ut, []
        want, urgent, nxt = pick_v2(s)
        alt, flip = V2_SPLIT.get(want), False
        if alt is not None and self.cur == want and ut - self.t_cut > self.split_limit(s, want):
            want, flip = alt, True                          # a long burn: the partner angle
        elif alt is not None and self.cur == alt and self.alt_on:
            if ut - self.t_cut <= self.split_limit(s, alt):
                want = alt                                  # the partner angle holds its split limit
            else:
                flip = True                                 # ... then back to the burn shot
        if want != self.cur:
            chain = want in BURN_CHAIN or self.cur in BURN_CHAIN
            hold = not showrun.cut_ok(ut, [x for x in txs if x.firm] if chain else txs)
            if self.cur is not None and not urgent:
                hold = hold or ut - self.t_cut < showrun.MIN_SHOT or showrun.maxq_quiet(ut, s.ev) or \
                    (nxt is not None and nxt - ut < showrun.MIN_SHOT)
            elif self.cur in BURN_CHAIN and want in BURN_CHAIN:      # fix round 3: no 1-2 s burn-chain shot
                hold = hold or ut - self.t_cut < showrun.MIN_SHOT
            if not hold:
                acts.append(("cut", self.cur, want))
                self.cur, self.t_cut, self.alt_on = want, ut, flip and want in V2_SPLIT.values()
        may_pause = s.p7 and not s.guard and not s.burning and showrun.pause_ok(ut, txs)
        if may_pause and not self.paused:
            acts.append(("pause",))
            self.paused = True
        elif self.paused and not may_pause:
            acts.append(("resume",))
            self.paused = False
        return acts


class Director:
    def __init__(self, log=print, v2=False):
        self.log = log
        self._stop = threading.Event()
        self.th = None
        self.cutter = Cutter()
        self.cam_str = None        # last string written to shot.txt / stock camera tuple
        self.t_liftoff = None      # own lift-off detection (ascents that do not report events)
        self.seen_rails = False
        self.t_thrust = None       # start of the current thrust period (burn debounce)
        self.handed = False
        self.cam = pad_camera()
        self.v2 = v2
        if v2:                     # recording pipeline v2 (director v2 specification)
            self.cutter = CutterV2()
        self.burns = []            # [(ignition UT, cutoff UT)] of every thrust period > 1 s (v2 editor: P4 P8 P11)
        self.burn_end = None
        self.period = None         # orbital period after the payload release (skip card: number of orbits)
        self.t_first = None        # UT of the first tick (a recording from g5_orbit starts in P7)

    # ---- lifecycle ----
    def start(self):
        atexit.register(self._atexit)
        self.th = threading.Thread(target=self._run, name="director", daemon=True)
        self.th.start()
        self.log("director: on (whole-mission recording, director shot list)")
        return self

    def handoff(self):
        """The lander owns the camera from here (S7-S9): resume capture, back to the stock chase camera."""
        self.handed = True
        self.stop()
        self.log("director: handoff to the lander camera")

    def _atexit(self):
        """The flight script ended before the landing (abort): stop the recording, keep the frames."""
        if not self.handed:
            self.stop()
            try:
                open(core.os.path.join(core._AL, "rec.stop"), "w").close()
            except OSError:
                pass

    def stop(self):
        if self._stop.is_set():
            return
        self._stop.set()
        if self.th:
            self.th.join(timeout=2)
        core.record_pause(False)
        core.camera_shot("chase")
        hud.report("set", cam=None, camtel=None, camuntil=None)

    # ---- the loop ----
    def _state(self, sc, conn):
        ut = sc.ut
        v = sc.active_vessel
        body = v.orbit.body
        fl = v.flight(body.reference_frame)
        lat, lon, alt = fl.latitude, fl.longitude, fl.mean_altitude
        if v.thrust > 1000.0:
            self.t_thrust = ut if self.t_thrust is None else self.t_thrust
        else:
            if self.t_thrust is not None and ut - self.t_thrust > 1.0:
                self.burns.append((self.t_thrust, ut))
                self.burn_end = ut
            self.t_thrust = None
        burning = self.t_thrust is not None and ut - self.t_thrust > 1.0    # trims of a few tenths: dead air
        rails = sc.rails_warp_factor > 0
        f = hud.current
        ev, tel = {}, {}
        if f is not None:
            with f.lock:
                ev = {k: (u, d) for k, (u, d) in f.ev.items()}
            tel = getattr(f, "_tel", None) or {}
        h_pad = alt - hud.core_pad_alt()
        if "LIFTOFF" in ev and ev["LIFTOFF"][1]:
            self.t_liftoff = ev["LIFTOFF"][0]
        elif self.t_liftoff is None and self.t_thrust is not None and h_pad > 5.0:
            self.t_liftoff = ut                     # ascent without HUD events (MechJeb path)
        self.seen_rails |= rails and self.t_liftoff is not None
        ascent_over = ("MECO" in ev and ev["MECO"][1]) or self.seen_rails or alt > 70000.0
        rng, elev, az = look_angles(self.cam, lat, lon, alt)
        met = ut - self.t_liftoff if self.t_liftoff is not None else None
        guard = v.thrust > 0.0 or v.control.throttle > 0.0
        if not guard:
            try:
                guard = v.auto_pilot.engaged       # attitude being steered (burn alignment, trims, probes)
            except Exception:
                guard = True                       # unknown: never pause
        if not guard:
            guard = guard_window(ut, [n.ut for n in v.control.nodes[:1]], ev)
        p7 = False
        if self.v2:
            if self.t_first is None:
                self.t_first = ut
            sep = ev.get("PAYLOAD SEP")
            if sep and sep[1] and self.period is None:
                try:
                    self.period = v.orbit.period
                except Exception:
                    pass
            start = sep[0] + showrun.SEP_WIN if sep and sep[1] else (
                self.t_first if ascent_over and "ORBIT" not in ev else None)
            de = ev.get("DEORBIT BURN")
            p7 = (start is not None and ut > start + showrun.BRIDGE_S * showrun.BRIDGE_VIDEO + 20.0
                  and not (de and (de[1] or de[0] - ut < GUARD_S)))
        self._latlon = (lat, lon)
        return State(ut, met, ev, h_pad, burning, rails, int(tel.get("srb", 0) or 0), ascent_over, rng, elev, az,
                     guard, self.burn_end, p7, self.t_thrust)

    def _apply(self, conn, cam):
        if cam == self.cam_str:
            return
        if isinstance(cam, tuple):                 # stock kRPC camera (plugin hands the camera back first)
            core.camera_shot("chase")
            time.sleep(0.3)
            try:
                c = conn.space_center.camera
                c.mode = getattr(conn.space_center.CameraMode, cam[0])
                c.distance = max(c.min_distance, min(cam[1], c.max_distance))
                c.pitch = max(c.min_pitch, min(cam[2], c.max_pitch))
            except Exception as e:
                self.log("director: stock camera %r" % e)
        else:
            core.camera_shot(cam)
        self.cam_str = cam

    def _sun_cam(self, sc, sid, base, new_shot, ut=None):
        """Fix round 3 (QA 4.1, entry chase into the sun): the rel shot string with its camera moved so the view axis
        stays >= 60 deg from the sun (showrun.sun_safe_rel). The geometry (sun, velocity: core.sun_geometry on the
        director's own connection) is read once at the cut and frozen for the shot, so a shot string never changes
        on its own mid-shot (a 'cut' token would make that a hard cut). Exception: a rel shot WITHOUT 'cut' that stays
        on air for long (the orbital BRIDGE: hours of coast) re-reads the geometry every SUN_REFRESH game s - the
        plugin eases such a change (slow camera move). A rel -> rel cut without a 'cut' token is eased by the plugin
        (2.5 s): if that swing would pass within 60 deg of the sun, 'cut' is appended (hard cut). One log line per
        shot: the sun angle."""
        p = showrun.parse_rel(base)
        if p is None:
            self._sun_last = None                  # the plugin hard-cuts on a change of shot type
            return base
        stale = (sc is not None and "cut" not in p[4] and ut is not None and
                 ut - getattr(self, "_sun_t", ut) > SUN_REFRESH and
                 time.time() - getattr(self, "_sun_wall", 0.0) > 2.0)
        if new_shot or getattr(self, "_sun_sid", None) != sid or stale:
            if not stale:
                self._sun_sid, self._sun_prev, self._sun_logged = sid, None, False
            self._sun_cache, self._sun_geo, self._sun_t, self._sun_wall = {}, None, ut, time.time()
            if sc is not None:
                try:
                    self._sun_geo = core.sun_geometry(sc, sc.active_vessel, *self._latlon)
                except Exception as e:
                    self.log("director: %s sun geometry unavailable (%r): nominal camera" % (sid, e))
        if base not in self._sun_cache:
            out, info = showrun.sun_safe_rel(base, self._sun_geo, prev_az=self._sun_prev, side=sid in SUN_SIDE)
            if info is not None:
                self._sun_prev = info["az"]
                s_rel = showrun.sun_in_rel(self._sun_geo)
                az_abs = info["az"] + (math.degrees(math.atan2(s_rel[1], -s_rel[0])) if info["mode"] == "sun-token"
                                       else 0.0)
                last = getattr(self, "_sun_last", None)
                if (new_shot and last is not None and "cut" not in out.split() and not info["occluded"] and
                        showrun.ease_min_angle(last, (info["el"], az_abs), s_rel) < showrun.SUN_MIN_ANGLE):
                    out += " cut"                  # the eased swing would sweep past the sun: hard cut instead
                    self.log("director: %s hard cut (the eased move would pass the sun)" % sid)
                self._sun_last = (info["el"], az_abs)
                if not self._sun_logged:
                    self._sun_logged = True
                    self.log("director: " + showrun.sun_info_text(sid, info))
            self._sun_cache[base] = out
        return self._sun_cache[base]

    def _tick_v2(self, s, txs, sc=None):
        acts = self.cutter.step(s, txs)
        cur = self.cutter.cur
        when = "T+%.0f s" % s.met if s.met is not None else "UT %.0f" % s.ut
        for a in acts:
            if a[0] == "cut":
                self.log("director: %s %s (%s)" % (cur, V2_NAME.get(cur, cur), when))
            elif a[0] == "pause":                  # P7 only (V9): disk saving, the editor skips this span
                core.record_pause(True)
                self.log("director: P7 coast, capture paused (%s)%s" % (
                    when, "" if core.REC_OBS is None else " - v3: OBS records on, the editor skips it"))
            elif a[0] == "resume":
                core.record_pause(False)
                self.log("director: capture resumed (%s)" % when)
        if cur is not None:
            self._apply(None, self._sun_cam(sc, cur, V2_CAM[cur](s), any(a[0] == "cut" for a in acts), s.ut))
            if cur == "S1":
                hud.report("set", cam="PAD TRACKING CAMERA · KSC",
                           camtel="RANGE %.2f KM    ELEV %.1f°    AZ %03.0f°" % (s.rng / 1e3, s.elev, s.az),
                           camuntil="%.3f" % (self.t_liftoff + showrun.V2_CAPTION_OUT) if self.t_liftoff else None)
            else:
                hud.report("set", cam=None, camtel=None, camuntil=None)

    def _tick(self, sc, conn):
        s = self._state(sc, conn)
        if self.v2:
            f = hud.current
            return self._tick_v2(s, f.transitions() if f is not None else [], sc)
        f = hud.current
        txs = f.transitions() if f is not None else []
        acts = self.cutter.step(s, txs)
        cur = self.cutter.cur
        when = "T+%.0f s" % s.met if s.met is not None else "UT %.0f" % s.ut
        for a in acts:
            if a[0] == "cut":
                self.log("director: %s %s (%s)" % (cur.id, cur.name, when))
            elif a[0] == "pause":
                core.record_pause(True)
                hud.report("set", cam=None, camtel=None, camuntil=None)
                self.log("director: dead air, recording paused (%s)" % when)
        if cur is not None:
            self._apply(conn, cur.cam(s))
            if cur.caption:                          # S1 ground-telemetry caption, fades out at T+30 (plugin)
                hud.report("set", cam="PAD TRACKING CAMERA · KSC",
                           camtel="RANGE %.2f KM    ELEV %.1f°    AZ %03.0f°" % (s.rng / 1e3, s.elev, s.az),
                           camuntil="%.3f" % (self.t_liftoff + showrun.CAPTION_OUT) if self.t_liftoff else None)
            else:
                hud.report("set", cam=None, camtel=None, camuntil=None)
        if ("resume",) in acts:                      # the shot is set: capture again from the next frame
            time.sleep(0.25)
            core.record_pause(False)
            self.log("director: capture resumed (%s)%s" % (
                when, "" if cur else " on the last camera (HUD transition or burn guard)"))

    def _run(self):
        try:
            conn = core.connect("director")
        except Exception as e:
            self.log("director: no connection (%r)" % e)
            return
        sc = conn.space_center
        err = False
        while not self._stop.is_set():
            try:
                self._tick(sc, conn)
            except Exception as e:                   # never let the camera take the flight down
                if not err:
                    err = True
                    self.log("director error: %r" % e)
            self._stop.wait(PERIOD)
        try:
            conn.close()
        except Exception:
            pass
