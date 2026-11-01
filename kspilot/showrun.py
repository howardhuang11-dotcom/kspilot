"""Unified show timeline: ONE clock for the HUD widget transitions and the camera cuts (show-timeline notes).

Single source of truth = the mission events of the HUD feed (kspilot/hud.py ORDER: LIFTOFF ... TOUCHDOWN, done or
predicted) plus three marks the flight code reports to the feed (hud.report("mark", ...)):
  S8       UT of the lander's S7 -> S8 cut (end of the entry burn; first chase shot of a run without one)
  HANDOFF  UT at which the descent passes 1 km (announced ~1.5 s ahead by the lander, then frozen)
transitions() turns them into the planned HUD transitions. The feed publishes them in the overlay (hud_tx=, the
plugin animates at exactly those UTs) and the cameras read the same list in-process:
  - no camera cut within GUARD (2 s, ORBIT_ICON 3 s) of a transition: the cut waits until the animation is over
    (cut_ok / next_cut_ut), HUD first
  - a rec.pause dead-air cut never covers a transition (pause_ok): capture resumes PAUSE_LOOK s before one
Pure Python (no kRPC): tests/test_showrun.py replays a nominal mission against every rule.
"""
from collections import namedtuple

# ---- the schedule (design decision 2026-10-21) ----
ASCENT_IN = 3.0          # s after LIFTOFF: ascent widget slides in (during S1)
CAPTION_OUT = 30.0       # T+ s: the S1 ground-telemetry caption fades out ...
S1_END = 35.0            # T+ s: ... before the S1 -> S2 cut
ORBIT_SHRINK = 2.0       # s after ORBIT: the ascent widget shrinks into the orbit icon
FUNNEL_AFTER_CUT = 2.0   # s after the S7 -> S8 cut: entry funnel slides in (camera first here)
S9_AFTER_HANDOFF = 3.0   # s after the 1 km funnel -> convergence hand-off: the single landing-burn cut S8 -> S9
TD_PLOT = 2.5            # s after TOUCHDOWN: touchdown plot slides in; no cuts after touchdown
HANDOFF_H = 1000.0       # m above the pad: funnel -> convergence hand-off
HANDOFF_LEAD = 1.5       # s: the hand-off UT is announced (and frozen) this far ahead
GUARD = 2.0              # s: no cut this close to a HUD transition
PAUSE_LOOK = 3.0         # s: capture resumes this long before a transition (> GUARD: the resume is no cut in a window)
MAXQ_QUIET = 2.0         # s: no non-urgent cut this close to MAX-Q

# name -> (animation s, guard s); ORBIT_ICON: no cut within 3 s
KIND = {"ASCENT_IN": (0.6, GUARD), "CAPTION_OUT": (0.8, GUARD), "ORBIT_ICON": (1.2, 3.0),
        "PAYLOAD_SPLIT": (0.8, GUARD), "FUNNEL_IN": (0.6, GUARD), "HANDOFF": (1.5, GUARD), "TD_PLOT": (0.6, GUARD)}

Tx = namedtuple("Tx", "name ut dur guard firm")     # firm: from a done event / mark (the plugin animates only these)


def _tx(name, ut, firm):
    dur, guard = KIND[name]
    return Tx(name, float(ut), dur, guard, bool(firm))


def transitions(ev, marks=None, v2=False):
    """Planned HUD widget transitions, sorted by UT. ev = {name: (ut, done)} (HUD events: actual or predicted),
    marks = {name: ut}. Predicted events give non-firm entries (the director plans with them).
    v2 (HOP_REC=full recordings only): the S1 caption fades out at T+20 (V2_CAPTION_OUT) instead of T+30."""
    marks = marks or {}
    out = []

    def add(name, ev_name, dt):
        if ev_name in ev:
            out.append(_tx(name, ev[ev_name][0] + dt, ev[ev_name][1]))

    add("ASCENT_IN", "LIFTOFF", ASCENT_IN)
    add("CAPTION_OUT", "LIFTOFF", V2_CAPTION_OUT if v2 else CAPTION_OUT)
    add("ORBIT_ICON", "ORBIT", ORBIT_SHRINK)
    add("PAYLOAD_SPLIT", "PAYLOAD SEP", 0.0)
    if "S8" in marks:
        out.append(_tx("FUNNEL_IN", marks["S8"] + FUNNEL_AFTER_CUT, True))
    elif "AERO GLIDE" in ev:                                       # planning only: the cut defines it
        out.append(_tx("FUNNEL_IN", ev["AERO GLIDE"][0] + FUNNEL_AFTER_CUT, False))
    if "HANDOFF" in marks:
        out.append(_tx("HANDOFF", marks["HANDOFF"], True))
    if "TOUCHDOWN" in ev and ev["TOUCHDOWN"][1]:                   # a predicted touchdown never moves the HUD
        out.append(_tx("TD_PLOT", ev["TOUCHDOWN"][0] + TD_PLOT, True))
    return sorted(out, key=lambda t: t.ut)


def window(tx):
    """(start, end) of the no-cut window around a transition: guard before, animation (at least guard) after."""
    return tx.ut - tx.guard, tx.ut + max(tx.dur, tx.guard)


def cut_ok(ut, txs, eps=1e-6):
    """True if a camera cut at ut keeps clear of every HUD transition."""
    return all(not (a + eps < ut < b - eps) for a, b in map(window, txs))


def next_cut_ut(ut, txs):
    """Earliest UT >= ut at which a cut is allowed."""
    t = ut
    for _ in range(len(txs) + 1):
        hit = [window(x)[1] for x in txs if not cut_ok(t, [x])]
        if not hit:
            return t
        t = max(hit)
    return t


def pause_ok(ut, txs):
    """True if frame capture may stay paused at ut: no transition window starts within PAUSE_LOOK s or is open."""
    return all(not (a - PAUSE_LOOK < ut < b) for a, b in map(window, txs))


def maxq_quiet(ut, ev):
    """True while a non-urgent cut must wait for MAX-Q (actual or predicted) to pass."""
    return "MAX-Q" in ev and abs(ut - ev["MAX-Q"][0]) < MAXQ_QUIET


def handoff_eta(h, vz):
    """Seconds until the descent passes HANDOFF_H at the present sink rate (None while climbing / far)."""
    if vz >= -0.5:
        return 0.0 if h <= HANDOFF_H else None
    return max(h - HANDOFF_H, 0.0) / -vz


def landing_shot(ut, entry_burn, entry_preroll, handoff_ut, td_done):
    """Lander camera (S7-S9, Lander.direct_camera) on the unified timeline: S7 hull camera from ~4 s before the
    entry burn to its end, S8 chase/pad view after it, S9 pad long lens from hand-off + 3 s through touchdown."""
    if handoff_ut is not None and (ut >= handoff_ut + S9_AFTER_HANDOFF or td_done):
        return "S9"
    if entry_burn or entry_preroll:
        return "S7"
    return "S8"


# =====================================================================================================================
# Recording pipeline v2 (HOP_REC=full ONLY; director v2 specification, video requirements V1-V10).
# Everything below is pure Python and is used only by the v2 director (kspilot/director.py), the v2 lander camera
# (Lander.rec_v2) and the post-production editor (tools/edit_video.py). Nothing above this line changed behaviour
# for non-recording / HOP_REC=1 runs.
#
# "Record, then cut": the flight decides game time warp exactly as before; the playback speed S of every video
# frame (game seconds per video second) is decided afterwards by playback_plan() from the real event UTs.
# =====================================================================================================================
from bisect import bisect_right  # noqa: E402
import math as _m  # noqa: E402

FPS = 30
V2_S1_END = 25.0         # T+ s: S1 -> S2a (spec 5: S1 one shot T-10 -> T+25)
V2_CAPTION_OUT = 20.0    # T+ s: S1 caption fades out (= S1_END - 5)
COUNTDOWN = 10.0         # V1: T-10 -> T0 at 1x
REC_LEAD = 11.0          # recording starts at T-11 (the first second is dropped by the editor)
TD_TAIL = 15.0           # V6: keep recording 15 game-s after touchdown
PRE_BURN = 10.0          # V2/V6: 1x from 10 s before an ignition
POST_ORBIT = 10.0        # V2: insertion burn cutoff + 10 s
SEP_WIN = 10.0           # V4: payload release +-10 s (the post-release half; the director cuts S5 at release - 10)
SEP_PRE = 12.0           # V4 fix round 3: 1x from 12 s before the release (QA flight #3: the +-10 s window around
                         # the release UT the HUD logged ~1.4 s late left 8.6-9 s of 1x before the decoupling)
WIN_MARGIN = 1.0         # next video round (video QA report 2026-10-21, item 5.5): the three required 10 s windows
                         # (before the insertion burn V2, after the release V4, before the entry burn V6) measured
                         # exactly 10.00 s; the editor keeps 1 s more at 1x. check_plan still checks the bare 10 s.
POST_DEORBIT = 5.0       # P8: deorbit cutoff + 5 s (inferred, spec 2.2)
MECO_TAIL = 5.0          # P2: MECO + 5 s
EXTRA_BURN_MIN = 1.0     # P11: a burn > 1 s inside a compressed span is kept at 1x ...
EXTRA_BURN_PAD = 2.0     # ... from 2 s before to 2 s after it
LAPSE_VIDEO = {"P3": 10.0, "P5": 3.0, "P9": 8.0}   # video seconds of each time-lapse span (spec 2.2)
BRIDGE_S = 100.0         # P7: 4 s time-lapse bridge at S = 100 ...
BRIDGE_VIDEO = 4.0
CARD_VIDEO = 2.5         # ... then the 2.5 s skip card (the ONLY jump cut of the video, V9)
REALTIME_BADGE = 2.0     # "REAL TIME" badge after a compressed span (then a 0.5 s fade)
TIERS = (2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000)

# V8 shot durations, hard upper limits in video seconds (shot-length study section 4 / spec section 7)
MIN_SHOT = 3.0
MAX_HOLD = {"establish": 10.0, "ground": 40.0, "chase": 45.0, "onboard": 30.0, "event": 15.0, "key": 1e9,
            "card": 4.0, "lapse": 15.0}
# shot id -> type (for MAX_HOLD); the shot strings live in kspilot/director.py / kspilot/landing.py
SHOT_TYPE = {"S1": "ground", "S2a": "chase", "S2b": "chase", "S2c": "chase", "S2d": "chase", "S2e": "chase",
             "S2f": "chase", "ONB": "onboard", "S3": "event", "BRIDGE": "chase", "S4": "event", "S5": "event",
             "S5b": "event", "S7": "event", "S8": "chase", "SIDE": "chase", "FRONT": "chase", "D1": "onboard",
             "D2": "ground", "D3": "chase", "S9": "ground", "S10": "key",
             # fix round 2 (QA 2026-10-21 defects 2/3/5): split event shots
             "S4b": "event", "S4D": "event", "S4E": "event",
             "S4c": "event"}     # fix round 3: the insertion burn's partner angle
# V8 fix round 2: an event close-up (insertion / deorbit burn, payload release) is split before it passes 15 s
EVENT_SPLIT = 12.0       # s on air: a burn shot still burning cuts to its partner angle (long burns)
S5_AFTER_SEP = 2.0       # s: S5 holds the release this long (= end of the PAYLOAD_SPLIT HUD window), then S5b
PRE_SEP_LEAD = SEP_PRE + 4.0   # recording v2: the release is announced (HUD plan) this long ahead, then flown on time
                               # (fix round 3: 16 s, was 11 s; 4 s of margin before the 1x window opens)


def tier(S):
    """Badge value: the 'round' speed tier nearest to S (log scale), spec 3.2."""
    return min(TIERS, key=lambda t: abs(_m.log(t / max(S, 1e-9))))


# ---------------------------------------------------------------------------------------------------------------------
# Playback plan (P0-P11, spec 2.2 / 4A)
# ---------------------------------------------------------------------------------------------------------------------
class Seg(namedtuple("Seg", "phase kind ut0 ut1 speed video")):
    """kind: 'real' (1x, every frame kept), 'lapse' (time-lapse at `speed`, badge), 'skip' (jump cut: the skip card
    covers ut0..ut1 in `video` seconds; P7 only), 'drop' (before T-10 / after touchdown + 15: not in the video)."""
    __slots__ = ()


def _phase_of_gap(mid, m):
    """Which compressed span a gap belongs to (spec 4A): P3 MECO -> insertion, P5 insertion -> release, P7 release ->
    deorbit (the only jump-cut span), P9 deorbit -> entry burn. A recording from the g5_orbit checkpoint (payload
    already released) starts in P7."""
    if m.get("gap_phase"):     # v3 editor only (tools/edit_obs.py): a g5_entry / term / mid checkpoint recording,
        return m["gap_phase"]  # its coast before the entry burn is P9 (never set by mission_from_events)
    ob, sep, de = m.get("orbit"), m.get("sep"), m.get("deorbit")
    if ob and mid < ob[0]:
        return "P3"
    if sep is not None and mid < sep:
        return "P5"
    if de and mid < de[0]:
        return "P7" if (sep is not None or m.get("t0") is None) else "P5"
    if de:
        return "P9"
    if sep is not None or m.get("t0") is None:
        return "P7"
    return "P5" if ob else "P3"


def sep_start(m):
    """Fix round 3 (V4): the earliest UT the payload release may have happened - the HUD event UT (logged after the
    staging and a 1.5 s wall wait), the commanded UT (sep_cmd) and the announced UT (sep_plan), whichever is first.
    The pre-release 1x window is measured from it, so a late log stamp never shortens it."""
    return min(x for x in (m.get("sep"), m.get("sep_cmd"), m.get("sep_plan")) if x is not None)


def real_windows(m):
    """The 1x windows (V1 V2 V4 V6 + deorbit burn + P11 extra burns), clipped to the recording, merged.
    m: rec0 rec1 (first / last frame UT), cd (planned T0), t0 (lift-off), meco, orbit (ign, cutoff), sep,
    deorbit (ign, cutoff), entry (entry-burn ignition), lander (lander start, fallback), td (touchdown),
    burns [(ign, cutoff)] (every thrust period > 1 s the director saw)."""
    w = []
    rec0, rec1 = m["rec0"], m["rec1"]
    if m.get("t0") is not None:
        t0 = m["t0"]
        cd = m.get("cd")
        start = (min(cd, t0) if cd is not None else t0) - COUNTDOWN
        w.append(["P0", start, t0])
        w.append(["P1", t0, m["meco"] + MECO_TAIL if m.get("meco") is not None else rec1])
    if m.get("orbit"):
        w.append(["P4", m["orbit"][0] - PRE_BURN - WIN_MARGIN, m["orbit"][1] + POST_ORBIT])
    if m.get("sep") is not None:
        w.append(["P6", sep_start(m) - SEP_PRE, m["sep"] + SEP_WIN + WIN_MARGIN])
    if m.get("deorbit"):
        w.append(["P8", m["deorbit"][0] - PRE_BURN, m["deorbit"][1] + POST_DEORBIT])
    end = m["td"] + TD_TAIL if m.get("td") is not None else rec1
    if m.get("entry") is not None:
        w.append(["P10", m["entry"] - PRE_BURN - WIN_MARGIN, end])
    elif m.get("lander") is not None:
        w.append(["P10", m["lander"], end])
    main = [(a, b) for _, a, b in w]
    for a, b in m.get("burns", ()):
        if b - a > EXTRA_BURN_MIN and not any(x - 1e-6 <= a and b <= y + 1e-6 for x, y in main):
            w.append(["P11", a - EXTRA_BURN_PAD, b + EXTRA_BURN_PAD])
    w = [[p, max(a, rec0), min(b, rec1)] for p, a, b in w if min(b, rec1) > max(a, rec0)]
    w.sort(key=lambda x: x[1])
    out = []
    for p, a, b in w:
        if out and a <= out[-1][2] + 1e-6:
            out[-1][2] = max(out[-1][2], b)
            if p not in out[-1][0].split("+"):
                out[-1][0] += "+" + p
        else:
            out.append([p, a, b])
    return [tuple(x) for x in out]


def playback_plan(m):
    """[Seg] covering the whole recording in UT order. 1x windows are kept whole; between them the compressed spans
    P3 / P5 / P9 are time-lapses with a fixed video length (S = game duration / video seconds, one S per span),
    P7 is a 4 s bridge at S = 100 plus the 2.5 s skip card. Before T-10 and after touchdown + 15 s: dropped."""
    real = real_windows(m)
    rec0, rec1 = m["rec0"], m["rec1"]
    segs, gaps = [], []
    t = rec0
    for p, a, b in real:
        if a > t + 0.5 / FPS:
            gaps.append((t, a, len(segs)))
            segs.append(None)
        segs.append(Seg(p, "real", a, b, 1.0, b - a))
        t = b
    if rec1 > t + 0.5 / FPS:
        gaps.append((t, rec1, len(segs)))
        segs.append(None)
    first_real = real[0][1] if real else rec1
    last_real = real[-1][2] if real else rec0
    by_phase = {}
    for a, b, i in gaps:
        head = a < first_real - 1e-6 and m.get("t0") is not None          # before T-10 (the dropped first second)
        tail = b > last_real + 1e-6 and m.get("td") is not None           # after touchdown + 15 s
        if head or tail:
            segs[i] = Seg("drop", "drop", a, b, None, 0.0)
        elif b > last_real + 1e-6:              # the recording ended mid-flight (abort): keep the tail as a lapse
            S = max((b - a) / 8.0, 1.0)
            segs[i] = Seg("P9", "lapse" if S > 1.0 else "real", a, b, S, (b - a) / S)
        else:                                   # one speed per compressed span (P11 windows split a span)
            by_phase.setdefault(_phase_of_gap(0.5 * (a + b), m), []).append((a, b, i))
    for ph, lst in by_phase.items():
        if ph == "P7":
            bridge_left = BRIDGE_S * BRIDGE_VIDEO
            for a, b, i in lst:
                d = b - a
                if bridge_left > 0 and d <= bridge_left + 1e-6:
                    S = max(d / BRIDGE_VIDEO, 1.0)
                    segs[i] = Seg("P7", "lapse" if S > 1.0 else "real", a, b, S, d / S)
                    bridge_left = 0.0
                elif bridge_left > 0:
                    cut = a + bridge_left
                    segs[i] = [Seg("P7", "lapse", a, cut, BRIDGE_S, BRIDGE_VIDEO),
                               Seg("P7", "skip", cut, b, None, CARD_VIDEO)]
                    bridge_left = 0.0
                else:
                    segs[i] = Seg("P7", "skip", a, b, None, CARD_VIDEO)
            continue
        D = sum(b - a for a, b, _ in lst)
        S = D / LAPSE_VIDEO.get(ph, 8.0)
        for a, b, i in lst:
            if S <= 1.0 + 1e-9:                  # shorter than its budget: just play it
                segs[i] = Seg(ph, "real", a, b, 1.0, b - a)
            else:
                segs[i] = Seg(ph, "lapse", a, b, S, (b - a) / S)
    out = []
    for x in segs:
        out.extend(x if isinstance(x, list) else [x])
    return out


def video_time(plan, ut):
    """Video second at which game time ut is shown (monotonic; a skip card maps its span linearly onto the card)."""
    t = 0.0
    for s in plan:
        if s.kind == "drop":
            if ut < s.ut1:
                return t
            continue
        if ut < s.ut1:
            return t + s.video * max(ut - s.ut0, 0.0) / max(s.ut1 - s.ut0, 1e-9)
        t += s.video
    return t


def shot_durations(plan, cuts, t_end):
    """[(shot id, ut0, ut1, video seconds)] for a cut list [(ut, shot id)] (on air until the next cut / t_end)."""
    out = []
    for i, (u, sid) in enumerate(cuts):
        u1 = cuts[i + 1][0] if i + 1 < len(cuts) else t_end
        out.append((sid, u, u1, video_time(plan, u1) - video_time(plan, u)))
    return out


def plan_summary(plan):
    """{phase: video seconds}; merged 1x windows keep their joined name (e.g. 'P8+P11')."""
    out = {}
    for s in plan:
        if s.kind != "drop":
            out[s.phase] = out.get(s.phase, 0.0) + s.video
    return out


def check_plan(plan, m, txs=()):
    """V9 / V1-V6 assertions on a plan (spec 9.2 a, e, g): returns a list of violations (empty = pass)."""
    bad = []
    for s in plan:
        if s.kind == "skip" and s.phase != "P7":
            bad.append("jump cut outside P7: %s" % (s,))
    req = []
    if m.get("t0") is not None:
        req.append(("V1 countdown", m["t0"] - COUNTDOWN, m["t0"]))
        if m.get("meco") is not None:
            req.append(("V2 ascent to MECO", m["t0"], m["meco"]))
    if m.get("orbit"):
        req.append(("V2 insertion burn", m["orbit"][0] - PRE_BURN, m["orbit"][1] + POST_ORBIT))
    if m.get("sep") is not None:
        req.append(("V4 payload release", sep_start(m) - SEP_PRE, m["sep"] + SEP_WIN))
    if m.get("deorbit"):
        req.append(("P8 deorbit burn", m["deorbit"][0] - PRE_BURN, m["deorbit"][1] + POST_DEORBIT))
    if m.get("entry") is not None and m.get("td") is not None:
        req.append(("V6 entry burn -> touchdown + 15", m["entry"] - PRE_BURN, m["td"] + TD_TAIL))
    for a, b in m.get("burns", ()):
        if b - a > EXTRA_BURN_MIN:
            req.append(("P11 burn", a, b))
    for name, a, b in req:
        a, b = max(a, m["rec0"]), min(b, m["rec1"])
        for s in plan:
            if s.kind != "real" and s.ut0 < b - 1e-6 and s.ut1 > a + 1e-6:
                bad.append("%s %.1f-%.1f overlaps %s %s %.1f-%.1f" % (name, a, b, s.kind, s.phase, s.ut0, s.ut1))
    for x in txs:
        for s in plan:
            if s.kind != "real" and s.ut0 < x.ut < s.ut1:
                bad.append("HUD transition %s at %.1f in a %s span" % (x.name, x.ut, s.kind))
    return bad


# ---------------------------------------------------------------------------------------------------------------------
# Frame selection (EDL): which recorded frame goes into each video frame
# ---------------------------------------------------------------------------------------------------------------------
Out = namedtuple("Out", "src seg k")      # src: index into the recorded frames (None = black), seg index, k in seg


def select_frames(uts, plan, fps=FPS, dup_max=4):
    """uts: UT of every recorded frame (index.csv order, increasing). 1x spans: every recorded frame, repeated by
    its own game duration (a frame recorded at physics x2 without compensation covers 2/30 s -> shown twice:
    correct speed, 15 fps). Lapse: the frame at or before ut0 + k * S / fps. Skip card: recorded frames of the
    span sampled evenly (the card background), black if none were recorded (capture paused)."""
    out = []
    n = len(uts)
    for si, s in enumerate(plan):
        if s.kind == "drop" or n == 0:
            continue
        if s.kind == "real":
            i = bisect_right(uts, s.ut0 - 0.5 / fps)
            k = 0
            while i < n and uts[i] < s.ut1 - 0.5 / fps:
                nxt = uts[i + 1] if i + 1 < n else uts[i] + 1.0 / fps
                dup = min(max(int(round((nxt - uts[i]) * fps)), 1), dup_max)
                for _ in range(dup):
                    out.append(Out(i, si, k))
                    k += 1
                i += 1
        elif s.kind == "lapse":
            nf = max(int(round(s.video * fps)), 1)
            for k in range(nf):
                u = s.ut0 + k * s.speed / fps
                i = bisect_right(uts, u + 1e-6) - 1
                out.append(Out(max(i, 0), si, k))
        else:
            nf = max(int(round(s.video * fps)), 1)
            a, b = bisect_right(uts, s.ut0), bisect_right(uts, s.ut1)
            for k in range(nf):
                out.append(Out(a + (b - a) * k // nf if b > a else None, si, k))
    return out


def check_frames(out, uts, plan, fps=FPS):
    """V9 (spec 9.2 g): outside the skip card consecutive video frames never jump in game time: 1x spans use every
    recorded frame in order (no frame lost), lapses advance by S / fps (plus the recorded frame spacing).
    Returns violations (empty = pass)."""
    bad = []
    prev = None
    for o in out:
        s = plan[o.seg]
        if prev is not None and prev.seg == o.seg and o.src is not None and prev.src is not None:
            if s.kind == "real" and o.src - prev.src not in (0, 1):
                bad.append("1x frame lost at UT %.2f (src %d -> %d)" % (uts[o.src], prev.src, o.src))
            if s.kind == "real" and o.src == prev.src + 1 and uts[o.src] - uts[prev.src] > 4.5 / fps:
                # spec C5: rails warp / a paused capture inside a 1x window cannot be repaired by the cut -> report
                bad.append("1x window has a %.1f s frame gap at UT %.2f (rails warp or paused capture)" % (
                    uts[o.src] - uts[prev.src], uts[prev.src]))
            if s.kind == "lapse":
                du = uts[o.src] - uts[prev.src]
                gap = uts[prev.src + 1] - uts[prev.src] if prev.src + 1 < len(uts) else 1.0 / fps
                if du > s.speed / fps + gap + 1e-3:
                    bad.append("lapse jump %.1f s at UT %.1f (S %.0f)" % (du, uts[o.src], s.speed))
        prev = o
    return bad


# ---------------------------------------------------------------------------------------------------------------------
# Fix round 2 (QA 2026-10-21 defect 4): no time warp inside a 1x window (flight side), and an honest badge if one
# happens anyway (editor side)
# ---------------------------------------------------------------------------------------------------------------------
WARP_PAD = 2.0           # s (game): a warp waits until a protected 1x window is over + this


def warp_hold_until(now, windows, pad=WARP_PAD):
    """UT until which a time warp must wait (None = warp now): the end + pad of every 1x window [(ut0, ut1)] open
    at `now` (a window ending inside the next one's span is chained). Pure; core.WarpGuard waits on it."""
    t, moved = now, True
    while moved:
        moved = False
        for a, b in windows:
            if a - 1e-6 <= t < b + pad - 1e-9:
                t, moved = b + pad, True
    return t if t > now else None


def live_windows(ev, burns, cd, now, lander=None):
    """The editor's 1x windows (real_windows) as far as they are known at `now` from the live HUD events
    {name: (ut, done)} and the director's thrust periods [(ign, cutoff)]; open-ended windows (P1 before MECO, P10
    before touchdown) are left out: they never hold a warp."""
    far = 1e7
    evj = dict(ev={k: list(v) for k, v in ev.items()}, burns=[tuple(b) for b in burns], cd=cd, lander=lander)
    m = mission_from_events(evj, now - far, now + far)
    return [(a, b) for _, a, b in real_windows(m) if b < now + far - 1.0]


def compressed_runs(uts, rails, plan, fps=FPS, dup_max=4, min_game=0.5, join=0.5):
    """Editor fallback: [(ut0, ut1, n frames)] spans of the 1x segments that cannot be shown at 1x - frames captured
    on rails (index.csv mode 'rails' at a warp rate > 1: `rails[i]` True) or recorded more than dup_max / fps apart
    (select_frames would compress them silently). Spans closer than `join` s are merged; spans shorter than
    `min_game` s of game time are left to check_frames (reported, not badged)."""
    n = len(uts)
    runs = []
    for s in plan:
        if s.kind != "real":
            continue
        i = bisect_right(uts, s.ut0 - 0.5 / fps)
        cur = None
        while i < n and uts[i] < s.ut1 - 0.5 / fps:
            nxt = uts[i + 1] if i + 1 < n else uts[i] + 1.0 / fps
            bad = bool(rails[i]) if rails is not None and i < len(rails) else False
            bad = bad or (nxt - uts[i]) * fps > dup_max + 0.5
            if bad:
                a, b = uts[i], min(nxt, s.ut1)
                if cur is not None and a - cur[1] <= join:
                    cur[1], cur[2] = b, cur[2] + 1
                else:
                    if cur is not None:
                        runs.append(cur)
                    cur = [a, b, 1]
            i += 1
        if cur is not None:
            runs.append(cur)
    return [tuple(r) for r in runs if r[1] - r[0] >= min_game]


def badge_compressed(plan, runs, fps=FPS):
    """Split the 1x segments around each compressed run: the run becomes a badged time-lapse (kind 'lapse', same
    phase name, speed = game s / (frames / fps): every recorded frame shown once) - never an unbadged speed-up.
    check_plan then still reports it (a lapse inside a protected 1x window): the run prints EDL CHECKS FAILED."""
    if not runs:
        return list(plan)
    out = []
    for s in plan:
        mine = sorted(r for r in runs if s.kind == "real" and s.ut0 - 1e-6 <= r[0] < s.ut1)
        if not mine:
            out.append(s)
            continue
        t = s.ut0
        for a, b, nf in mine:
            b = min(b, s.ut1)
            if a > t + 1e-9:
                out.append(Seg(s.phase, "real", t, a, 1.0, a - t))
            video = max(nf / float(fps), 1.0 / fps)
            S = max((b - a) / video, 1.0)
            out.append(Seg(s.phase, "lapse" if S > 1.0 + 1e-9 else "real", a, b, S, (b - a) / S))
            t = b
        if s.ut1 > t + 1e-9:
            out.append(Seg(s.phase, "real", t, s.ut1, 1.0, s.ut1 - t))
    return out


# ---------------------------------------------------------------------------------------------------------------------
# v2 ascent schedule (spec 8.1): MET -> shot id; S3 (booster separation) is urgent and handled by the director
# ---------------------------------------------------------------------------------------------------------------------
ASCENT_V2 = [(V2_S1_END, "S2a"), (48.0, "S2b"), (66.0, "S2c"), (90.0, "ONB"), (110.0, "S2d"), (140.0, "S2f"),
             (165.0, "S2e")]
ASCENT_CYCLE = [("S2d", 20.0), ("S2f", 20.0), ("S2e", 20.0)]     # MECO later than ~T+187: keep alternating
ASCENT_CYCLE_FROM = 187.0


def ascent_shot_v2(met):
    """(shot id, MET of the next scheduled change) for the ascent at mission time met (None = on the pad)."""
    if met is None or met < V2_S1_END:
        return "S1", V2_S1_END
    if met < ASCENT_CYCLE_FROM:
        cur, nxt = "S1", V2_S1_END
        for i, (t, sid) in enumerate(ASCENT_V2):
            if met >= t:
                cur = sid
                nxt = ASCENT_V2[i + 1][0] if i + 1 < len(ASCENT_V2) else ASCENT_CYCLE_FROM
        return cur, nxt
    period = sum(d for _, d in ASCENT_CYCLE)
    x = (met - ASCENT_CYCLE_FROM) % period
    t = met - x
    for sid, d in ASCENT_CYCLE:
        if x < d:
            return sid, t + d
        x -= d
        t += d
    return ASCENT_CYCLE[-1][0], met + 1.0


# ---------------------------------------------------------------------------------------------------------------------
# v2 descent camera (spec 8.3, section 6 V7, section 11 V10): entry burn -> touchdown + 15 s, all 1x
# ---------------------------------------------------------------------------------------------------------------------
V2_PRE_S7 = 4.0          # S7 hull camera from 4 s before the entry burn (spec 8.3)
POST_BURN_CYCLE = [("S8", 21.0), ("SIDE", 12.0), ("FRONT", 14.0), ("S8", 16.0), ("SIDE", 13.0)]
POST_BURN_REPEAT = [("FRONT", 14.0), ("S8", 16.0), ("SIDE", 13.0)]
V7_START_H, V7_START_RANGE = 15000.0, 16000.0     # 3-camera rotation from 15 km once the pad lens sees < 16 km
V7_BANDS = [(11000.0, "D2"), (9000.0, "D1"), (7000.0, "D2"), (5000.0, "D1"), (3500.0, "D3"), (-1e9, "D1")]
V7_ALT = {"D1": "D3", "D3": "D1", "D2": "D3"}     # over-long band (slow descent): alternate, never back to D2
DRONE_H = 60.0           # V10: S10 drone camera from the first h_gnd < 60 m
DRONE_LEAD = 0.1         # s (game): mean latency of the 4 Hz camera tick + the plugin's 0.2 s shot.txt poll while
                         # recording (game ~0.5x real); ~3 m at the 31 m/s of the last 60 m
V7_LOOKAHEAD = 8.0       # s: no new rotation shot this close (at the present sink rate) to the start of the V7 bands
MIN_BAND_LEFT = 6.0      # s: an over-long band shot is not swapped when the band ends sooner than this
# V7 pad long lens, fix round 2 (QA defect 1: at 10-13 km the 1.2 deg lens clamp left a ~40 px dark dot in an empty
# sky, screenshots/qa_v2_07). The lens zooms from the range so the stage spans >= PAD_LENS_FILL of the picture height.
# FOV limits: KSP FlightCamera.SetFoV writes Unity's Camera.fieldOfView on every flight camera with NO clamp
# (Assembly-CSharp IL, 2026-10-21; FlightCamera.fovMin 20 / fovMax 160 are only initialised, never applied there) and
# the 1.2 deg lens is proven in flight #2; the narrowest lens we ask for is PAD_LENS_MIN_FOV (plugin [min_fov]).
# Farther than PAD_LENS_MAX_RANGE the pad lens is not used: its V7 band goes to the steep chase D3 (then D1).
STAGE_APPARENT_M = 10.0  # m: stage extent seen from the pad camera (measured: 40 px of 1080 at 13.1 km, 1.2 deg lens)
PAD_LENS_FILL = 0.15     # V7: the stage spans >= 15 % of the picture height
PAD_LENS_MIN_FOV = 0.3   # deg: narrowest pad lens (checked in flight #3: below the flown 1.2 deg)
PAD_LENS_FRAME = STAGE_APPARENT_M / PAD_LENS_FILL                                      # 66.7 m fill the height
PAD_LENS_MAX_RANGE = 0.5 * PAD_LENS_FRAME / _m.tan(_m.radians(0.5 * PAD_LENS_MIN_FOV))  # ~12.7 km
V7_PAD_SUB = "D3"        # the pad lens band beyond PAD_LENS_MAX_RANGE


def pad_lens(rng):
    """(frame_m, fov_deg, usable) of the V7 pad long lens at slant range rng (m): frame_m metres fill the picture
    height, fov = 2 atan(frame_m / 2 rng); usable = the fov is not narrower than PAD_LENS_MIN_FOV."""
    fov = 2.0 * _m.degrees(_m.atan(0.5 * PAD_LENS_FRAME / max(rng, 1.0)))
    return PAD_LENS_FRAME, fov, fov >= PAD_LENS_MIN_FOV - 1e-9


def stage_fill(rng, fov_deg):
    """Fraction of the picture height the stage spans at range rng with a vertical fov (deg)."""
    return STAGE_APPARENT_M / (2.0 * rng * _m.tan(_m.radians(0.5 * fov_deg)))


def drone_due(h_gnd, vz, lead=DRONE_LEAD):
    """V10 trigger: the feet first below DRONE_H above the ground (h_gnd extrapolated `lead` s ahead)."""
    return h_gnd == h_gnd and h_gnd + min(vz, 0.0) * lead < DRONE_H


def cycle_shot(t, first, repeat):
    """Shot of a duration cycle at t s after its start: `first` once, then `repeat` forever."""
    for sid, d in first:
        if t < d:
            return sid
        t -= d
    per = sum(d for _, d in repeat)
    t %= per
    for sid, d in repeat:
        if t < d:
            return sid
        t -= d
    return repeat[-1][0]


class DescentDirector:
    """Lander camera v2 (pure, game-UT clock). step(...) -> shot id. S7 from 4 s before the entry burn to its end
    (urgent); S8 chase after it, then a chase / side / front rotation above 15 km; V7 three-camera rotation by
    altitude band (D1 down-looking hull camera, D2 pad long lens, D3 steep chase) down to the 1 km hand-off; S9 pad
    long lens from the hand-off + 3 s (urgent); S10 drone from the first h_gnd < 60 m (urgent, latched: no cut
    after it). Non-urgent cuts: >= MIN_SHOT after the previous cut and never in a HUD-transition window (cut_ok).
    pad_ok False (fix round 2): the pad lens cannot frame the stage >= 15 % this far out -> its band flies D3."""

    URGENT = ("S7", "S9", "S10")

    def __init__(self):
        self.cur, self.t_cut = None, -1e9
        self.t_burn_end = None
        self.v7 = False
        self.drone = False
        self.band, self.alt_on = None, False

    def want(self, ut, h, h_gnd, vz, rng, entry_burn, pre, entry_done, handoff_ut, td_done=False, pad_ok=True):
        if self.drone or ((handoff_ut is not None or td_done) and drone_due(h_gnd if h_gnd == h_gnd else h, vz)):
            self.drone = True
            return "S10"
        if handoff_ut is not None and (ut >= handoff_ut + S9_AFTER_HANDOFF or td_done):
            return "S9"
        if entry_burn or pre:
            return "S7"
        if not entry_done:
            return "S8"
        if self.t_burn_end is None:
            self.t_burn_end = ut
        if not self.v7 and h <= V7_START_H and rng <= V7_START_RANGE:
            self.v7 = True
        if not self.v7:
            sid = cycle_shot(ut - self.t_burn_end, POST_BURN_CYCLE, POST_BURN_REPEAT)
            if self.cur is not None and sid != self.cur and h + min(vz, 0.0) * V7_LOOKAHEAD < V7_START_H:
                return self.cur                  # the V7 rotation starts within seconds: no 3-s rotation shot
            return sid
        i = next(j for j, (hb, _) in enumerate(V7_BANDS) if h > hb)
        band, floor = V7_BANDS[i][1], V7_BANDS[i][0]
        if band == "D2" and not pad_ok:
            band = V7_PAD_SUB                    # the pad lens cannot frame the stage this far out (fix round 2)
        if band != self.band:
            self.band, self.alt_on = band, False
        elif self.cur == (V7_ALT[band] if self.alt_on else band) and \
                ut - self.t_cut > MAX_HOLD[SHOT_TYPE[self.cur]] - 10.0:      # stay well under the V8 maximum
            left = (h - max(floor, HANDOFF_H)) / max(-vz, 1.0) + (S9_AFTER_HANDOFF if floor < 0 else 0.0)
            if handoff_ut is None and left >= MIN_BAND_LEFT:
                self.alt_on = not self.alt_on
        return V7_ALT[band] if self.alt_on else band

    def step(self, ut, h, h_gnd, vz, rng, entry_burn, pre, entry_done, handoff_ut, txs=(), td_done=False,
             pad_ok=True):
        w = self.want(ut, h, h_gnd, vz, rng, entry_burn, pre, entry_done, handoff_ut, td_done, pad_ok)
        if w != self.cur and self.cur != "S10":                  # no cut after the drone shot
            urgent = w in self.URGENT
            if urgent or self.cur is None or (ut - self.t_cut >= MIN_SHOT and cut_ok(ut, txs)):
                self.cur, self.t_cut = w, ut
        return self.cur


def drone_pose(pad_lat, pad_lon, pad_alt, sun_az_deg, dist=70.0, up=32.0, aim_up=10.6, R=600000.0):
    """S10 'drone' camera (spec 11): fixed in the world, `dist` m from the pad towards the sun (front light), `up` m
    above the pad, aimed at a point `aim_up` m above the pad centre (~17 deg down: the pad sits in the lower third
    of a 50 deg picture, the stage enters from the top). Returns (lat, lon, alt, tlat, tlon, talt)."""
    a = _m.radians(sun_az_deg)
    n, e = dist * _m.cos(a), dist * _m.sin(a)
    lat = pad_lat + _m.degrees(n / R)
    lon = pad_lon + _m.degrees(e / (R * _m.cos(_m.radians(pad_lat))))
    return lat, lon, pad_alt + up, pad_lat, pad_lon, pad_alt + aim_up


# ---------------------------------------------------------------------------------------------------------------------
# V14 (requirement 2026-10-21: the drone view circles the vehicle; no zoom, but it keeps tracking the rocket):
# the S10 drone ORBITS the landing
# point on a horizontal circle (radius DRONE_ORBIT_R, DRONE_ORBIT_UP above the pad), its azimuth advancing at a constant
# rate from the cut, always aimed at the vessel (look-at = the vessel's CoM, every frame), fixed fov (no zoom). The
# plugin animates it per frame (AutoLab.cs "orbit" shot, RecMath.cs = these formulas); kspilot only writes the string
# once. Sun-safe: the arc is placed so the view axis (camera -> vessel) stays >= SUN_MIN_ANGLE from the sun for the
# whole planned shot (60 m -> touchdown -> +15 s), the camera on the sun side at touchdown when possible.
# ---------------------------------------------------------------------------------------------------------------------
DRONE_ORBIT_R = 70.0         # m from the landing point (the S10 distance flown in flights #4 / #5)
DRONE_ORBIT_UP = 30.0        # m above the pad
DRONE_ORBIT_RATE = 10.0      # deg/s (game): nominal orbit rate
DRONE_ORBIT_RATES = (10.0, 9.0, 8.0, 7.0, 6.0, 5.0, 4.0)    # tried in this order; 4 deg/s = the slowest "orbit"
DRONE_ORBIT_S = 24.0         # s (game) planned on air: 60 m -> touchdown (~6.5 s, flight #5) + 15 s + 2.5 s margin
DRONE_TD_S = 6.5             # s from the 60 m cut to touchdown (flight #5: v501.3 -> v507.8)
DRONE_COM_UP = 8.0           # m: the stage's CoM above the pad after touchdown (look-at model only)
DRONE_FOV = 50.0             # deg, fixed (V14: no zoom)
DRONE_FALLBACK_AZ = 247.0    # deg: the pad camera side (proven clear line of sight) when the sun is unknown


def drone_orbit_az(az0, rate, t):
    """Camera azimuth (deg from north, clockwise, as seen from the landing point) t game s after the cut."""
    return (az0 + rate * t) % 360.0


def drone_orbit_latlon(clat, clon, az, radius=DRONE_ORBIT_R, R=600000.0):
    """(lat, lon) of the camera `radius` m from the centre at azimuth az (the plugin's RecMath.OrbitLatLon)."""
    a = _m.radians(az)
    n, e = radius * _m.cos(a), radius * _m.sin(a)
    return clat + _m.degrees(n / R), clon + _m.degrees(e / (R * _m.cos(_m.radians(clat))))


def drone_target_up(t, h0=DRONE_H + DRONE_COM_UP, td=DRONE_TD_S, rest=DRONE_COM_UP):
    """Look-at model: the vessel's CoM height above the pad t s after the 60 m cut (straight down, linear to rest)."""
    return rest if t >= td else h0 + (rest - h0) * max(t, 0.0) / td


def drone_look(az, t, radius=DRONE_ORBIT_R, up=DRONE_ORBIT_UP):
    """Unit view axis camera -> vessel in pad ENU (e, n, u) at azimuth az, t s after the cut."""
    a = _m.radians(az)
    return _unit([-radius * _m.sin(a), -radius * _m.cos(a), drone_target_up(t) - up])


def drone_sun_angle(az, t, sun_enu, radius=DRONE_ORBIT_R, up=DRONE_ORBIT_UP):
    """Angle (deg) between the drone's view axis and the sun direction (pad ENU)."""
    s = _unit(_v(sun_enu))
    c = _dot(drone_look(az, t, radius, up), s)
    return _m.degrees(_m.acos(max(-1.0, min(1.0, c))))


def drone_orbit_min_angle(az0, rate, sun_enu, dur=DRONE_ORBIT_S, step=0.5, radius=DRONE_ORBIT_R, up=DRONE_ORBIT_UP):
    n = int(round(dur / step))
    return min(drone_sun_angle(drone_orbit_az(az0, rate, k * step), k * step, sun_enu, radius, up)
               for k in range(n + 1))


def drone_orbit_plan(sun_enu, dur=DRONE_ORBIT_S, rates=DRONE_ORBIT_RATES, min_angle=None,
                     radius=DRONE_ORBIT_R, up=DRONE_ORBIT_UP):
    """(az0, rate, info) of the S10 orbit. The fastest rate in `rates` whose whole arc keeps the view >= min_angle
    from the sun; the arc is anchored on the sun's azimuth (camera between the sun and the pad: front light) at
    touchdown first, else at mid-shot, shifted by the smallest offset that passes. Sun below the horizon: no glare,
    nominal rate. Sun unknown (None): the slowest rate centred on the pad camera side. Nothing passes: the slowest
    rate with the largest minimum angle."""
    min_angle = SUN_MIN_ANGLE if min_angle is None else min_angle
    if sun_enu is None:
        r = rates[-1]
        az0 = DRONE_FALLBACK_AZ - r * dur / 2.0
        return az0 % 360.0, r, dict(mode="no sun", sun_az=None, sun_el=None, min_angle=None)
    s = _unit(_v(sun_enu))
    saz = _m.degrees(_m.atan2(s[0], s[1])) % 360.0
    sel = _m.degrees(_m.asin(max(-1.0, min(1.0, s[2]))))
    info = dict(sun_az=saz, sun_el=sel)
    if s[2] < 0.0:
        az0 = saz - rates[0] * DRONE_TD_S
        return az0 % 360.0, rates[0], dict(info, mode="sun below the horizon",
                                            min_angle=drone_orbit_min_angle(az0, rates[0], s, dur, radius=radius, up=up))
    offs = [0.0] + [sg * k for k in range(10, 181, 10) for sg in (1.0, -1.0)]
    best = None
    for r in rates:
        for tc, anchor in ((DRONE_TD_S, "touchdown"), (dur / 2.0, "mid-shot")):
            for off in offs:
                az0 = saz + off - r * tc
                a = drone_orbit_min_angle(az0, r, s, dur, radius=radius, up=up)
                if a >= min_angle:
                    return az0 % 360.0, r, dict(info, mode="sun-safe", anchor=anchor, offset=off, min_angle=a)
                if r == rates[-1] and (best is None or a > best[2]):
                    best = (az0, r, a, anchor, off)
    az0, r, a, anchor, off = best
    return az0 % 360.0, r, dict(info, mode="best effort", anchor=anchor, offset=off, min_angle=a)


def drone_orbit_shot(clat, clon, pad_alt, sun_enu, radius=DRONE_ORBIT_R, up=DRONE_ORBIT_UP, fov=DRONE_FOV):
    """(AutoLab 'orbit' shot string, info) for the V14 S10 drone:
    orbit <clat> <clon> <alt> <radius> <up> <az0> <rate> <fov> (alt = pad altitude; camera at alt + up)."""
    az0, rate, info = drone_orbit_plan(sun_enu, radius=radius, up=up)
    cam = "orbit %.7f %.7f %.1f %.1f %.1f %.2f %.2f %d" % (clat, clon, pad_alt, radius, up, az0, rate, round(fov))
    return cam, dict(info, az0=az0, rate=rate)


def drone_orbit_text(info):
    """One log line for the planned orbit."""
    f = lambda x: "-" if x is None else "%.0f" % x                                   # noqa: E731
    return "S10 drone orbit: az0 %.0f deg, %.0f deg/s, r %.0f m, up %.0f m (sun az %s el %s, min sun angle %s deg, %s%s)" % (
        info["az0"], info["rate"], DRONE_ORBIT_R, DRONE_ORBIT_UP, f(info.get("sun_az")), f(info.get("sun_el")),
        f(info.get("min_angle")), info["mode"], (", lit at " + info["anchor"]) if info.get("anchor") else "")


# V15 (requirement 2026-10-21: before launch, hide the data panels and keep only the central arc and the clock):
# in a recording (rec=full with a
# planned lift-off cd=) the data panels (left / right readout rows, the side widget, the camera caption) are hidden
# before lift-off and slide / fade in over PANEL_IN s from the lift-off UT. Mirror of HudV2.cs RecMath.PanelReveal.
PANEL_IN = 0.8               # s (game)


def panel_reveal(rec_full, cd, t0, ut, dur=PANEL_IN):
    """0 (hidden) .. 1 (shown) for the HUD data panels; 1 for every HUD that is not a recording with a countdown."""
    if not rec_full or cd is None:
        return 1.0
    if t0 is None:
        return 0.0
    x = min(max((ut - t0) / dur, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


def mission_from_events(evj, rec0, rec1):
    """Mission dict for playback_plan() from the recording's events.json (tools/orbit_recover.py, HOP_REC=full):
    ev {name: [ut, done]} (HUD feed), burns [(ign, cutoff)] (director thrust periods > 1 s), cd (planned lift-off),
    lander (lander start UT). The insertion / deorbit burns are the measured thrust periods next to the ORBIT event
    (reported at cutoff) / the DEORBIT BURN node (burn centre); without them a few seconds around the event."""
    ev = {k: tuple(v) for k, v in evj.get("ev", {}).items()}
    burns = sorted(tuple(b) for b in evj.get("burns", []))

    def at(name):
        return ev[name][0] if name in ev and ev[name][1] else None

    m = dict(rec0=rec0, rec1=rec1, cd=evj.get("cd"), t0=at("LIFTOFF"), meco=at("MECO"), sep=at("PAYLOAD SEP"),
             entry=at("ENTRY BURN"), td=at("TOUCHDOWN"), lander=evj.get("lander"), burns=burns)
    if m["sep"] is not None:            # fix round 3 (V4): commanded / announced release UT (events.json, REC_FULL)
        pl = (evj.get("planned") or {}).get("PAYLOAD SEP")
        for k, x in (("sep_cmd", evj.get("sep_cmd")), ("sep_plan", pl[0] if pl else None)):
            if x is not None and abs(float(x) - m["sep"]) < 30.0:      # a stale plan is not the release
                m[k] = float(x)
    o = at("ORBIT")
    if o is not None:
        near = [b for b in burns if o - 8.0 <= b[1] <= o + 3.0 and (m["meco"] is None or b[0] > m["meco"])]
        m["orbit"] = max(near, key=lambda b: b[1] - b[0]) if near else (o - 8.0, o)
    d = at("DEORBIT BURN")
    if d is not None:
        near = [b for b in burns if b[0] - 30.0 <= d <= b[1] + 30.0]
        m["deorbit"] = max(near, key=lambda b: b[1] - b[0]) if near else (d - 3.0, d + 3.0)
    return m



# ---------------------------------------------------------------------------------------------------------------------
# Fix round 3 (video QA report 2026-10-21, HOP_REC=full ONLY): sun-aware relative cameras
# QA 4.1: two entry chase shots (31 s) looked straight into the sun - the stage was a black disc under it. Every
# AutoLab "rel <dist> <elev> <az> [fov] [tokens]" camera (director and lander) is now placed so the angle between its
# view axis and the sun is >= SUN_MIN_ANGLE, preferring >= SUN_PREF_ANGLE (sun beside / behind the camera: front-lit
# subject). The camera frame is the plugin's: up = radial, fwd = horizontal velocity (surface velocity below 36 km,
# orbital above), right = the physical right of fwd; az 0 = behind, 90 = right, 180 = ahead; elev above the horizon.
# All vectors here are in ONE right-handed Cartesian frame (local East-North-Up), so right = fwd x up (N x U = E).
# kRPC / Unity frames are left-handed: the callers express everything in ENU components first (core.sun_geometry,
# Lander._camera_v2), never cross raw kRPC vectors.
# ---------------------------------------------------------------------------------------------------------------------
SUN_MIN_ANGLE = 60.0     # deg between the view axis and the sun: never less (sun far outside a 45-60 deg picture)
SUN_PREF_ANGLE = 90.0    # deg: preferred (sun beside / behind the camera: the subject is lit from the camera side)
SUN_PREF_COST = 45.0     # deg of az change worth paying to get the preferred angle
REL_ALT_SWITCH = 36000.0  # m: the plugin's rel camera uses the surface velocity below, the orbital velocity above
R_KERBIN = 600000.0
SIDE_MIN_SIN = 0.75      # side-locked shots (MECO close-up): |sin az| >= this (the whole stage length in frame)


def _v(a):
    return [float(a[0]), float(a[1]), float(a[2])]


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _unit(a):
    n = _m.sqrt(_dot(a, a))
    return [x / n for x in a] if n > 1e-12 else None


def rel_basis(geo):
    """(fwd, right, up) unit vectors of the plugin's rel camera frame from a geometry dict (up, vsrf, vobt, alt) in a
    right-handed frame. fwd = the horizontal part of the surface (below 36 km) / orbital velocity; with no
    horizontal motion: north (the plugin's own fallback)."""
    up = _unit(_v(geo["up"]))
    vel = _v(geo["vsrf"] if geo.get("alt", 0.0) < REL_ALT_SWITCH or geo.get("vobt") is None else geo["vobt"])
    h = [vel[i] - _dot(vel, up) * up[i] for i in range(3)]
    fwd = _unit(h) if _m.sqrt(_dot(h, h)) > 3.0 else None
    if fwd is None:
        north = _v(geo.get("north", (0.0, 1.0, 0.0)))
        fwd = _unit([north[i] - _dot(north, up) * up[i] for i in range(3)])
    return fwd, _cross(fwd, up), up


def sun_in_rel(geo):
    """Unit sun direction in the rel frame: (along fwd, along right, along up)."""
    f, r, u = rel_basis(geo)
    s = _unit(_v(geo["sun"]))
    return (_dot(s, f), _dot(s, r), _dot(s, u))


def sun_occluded(geo):
    """True when the stage is in the planet's shadow (the sun behind the planet): no glare, no sunlight."""
    up = _unit(_v(geo["up"]))
    s = _unit(_v(geo["sun"]))
    su = _dot(s, up)
    r = geo.get("R", R_KERBIN) + geo.get("alt", 0.0)
    return su < 0.0 and r * _m.sqrt(max(1.0 - su * su, 0.0)) < geo.get("R", R_KERBIN)


def cam_offset(el, az):
    """Unit vector stage -> camera in the rel frame (fwd, right, up) for elev / az in deg (the plugin's formula)."""
    e, a = _m.radians(el), _m.radians(az)
    return (-_m.cos(e) * _m.cos(a), _m.cos(e) * _m.sin(a), _m.sin(e))


def sun_angle(el, az, s_rel):
    """Angle (deg) between the camera's view axis (camera -> stage) and the sun direction s_rel (rel frame)."""
    o = cam_offset(el, az)
    c = -(o[0] * s_rel[0] + o[1] * s_rel[1] + o[2] * s_rel[2])
    return _m.degrees(_m.acos(max(-1.0, min(1.0, c))))


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def sun_safe(el, az, s_rel, prev_az=None, side=False, el_lim=(-10.0, 45.0), min_angle=SUN_MIN_ANGLE,
             pref_angle=SUN_PREF_ANGLE):
    """(el, az, angle) closest to the nominal (el, az) with a view-to-sun angle >= min_angle, preferring
    >= pref_angle (cost: |d az| + 1.5 |d el| + SUN_PREF_COST below pref_angle, + 0.5 |az - prev_az| for continuity
    within one shot). The nominal is kept when it already reaches pref_angle. side: only |sin az| >= SIDE_MIN_SIN.
    If nothing reaches min_angle, the largest angle found wins."""
    a0 = sun_angle(el, az, s_rel)
    ok_side = (lambda x: abs(_m.sin(_m.radians(x))) >= SIDE_MIN_SIN - 1e-9) if side else (lambda x: True)
    if a0 >= pref_angle and ok_side(az):
        return el, az, a0
    best, best_any = None, None
    els = sorted({min(max(el + d, el_lim[0]), el_lim[1]) for d in (0, 10, -10, 20, -20, 30, 40)} |
                 ({el} if el_lim[0] <= el <= el_lim[1] else set()))
    for e in els:
        for k in range(-36, 37):
            a = az + 5.0 * k
            if not ok_side(a):
                continue
            ang = sun_angle(e, a, s_rel)
            if best_any is None or ang > best_any[2]:
                best_any = (e, a, ang)
            if ang < min_angle:
                continue
            cost = abs(5.0 * k) + 1.5 * abs(e - el) + (SUN_PREF_COST if ang < pref_angle else 0.0)
            if prev_az is not None:
                cost += 0.5 * abs(_wrap(a - prev_az))
            if best is None or cost < best[0] - 1e-9:
                best = (cost, e, a, ang)
    if best is None:
        e, a, ang = best_any
    else:
        _, e, a, ang = best
    return e, _wrap(a), ang


def ease_min_angle(a, b, s_rel, n=36):
    """Smallest view-to-sun angle along the plugin's eased move from camera a = (el, az) to b (el and az eased
    linearly in their numeric values, as CameraDirector.Ease does without the 'sun' token)."""
    return min(sun_angle(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n, s_rel) for k in range(n + 1))


def parse_rel(cam):
    """'rel d el az [fov] [tokens]' -> (d, el, az, fov, [tokens]); None for any other shot string."""
    if not isinstance(cam, str) or not cam.startswith("rel "):
        return None
    nums, toks = [], []
    for x in cam.split()[1:]:
        try:
            nums.append(float(x))
        except ValueError:
            toks.append(x)
    while len(nums) < 4:
        nums.append([60.0, 8.0, 20.0, 60.0][len(nums)])    # the plugin's defaults
    return nums[0], nums[1], nums[2], nums[3], toks


def _num(x):
    return ("%d" % round(x)) if abs(x - round(x)) < 1e-6 else ("%.1f" % x)


def sun_safe_rel(cam, geo, prev_az=None, side=False):
    """Sun-aware version of an AutoLab rel shot string. Returns (shot string, info dict: angle0, angle, el0, az0, el,
    az, occluded, changed, mode). Strings with the plugin's 'sun' token (az counted from the sun: S5 / S5b) and any
    stage in the planet's shadow are kept; only the angle is reported. Non-rel strings / no geometry: (cam, None)."""
    p = parse_rel(cam)
    if p is None or geo is None:
        return cam, None
    d, el, az, fov, toks = p
    s = sun_in_rel(geo)
    occ = sun_occluded(geo)
    if "sun" in toks:                      # the plugin adds the sun's azimuth (horizontal projection) to az
        az_abs = az + _m.degrees(_m.atan2(s[1], -s[0]))
        ang = sun_angle(el, az_abs, s)
        return cam, dict(angle0=ang, angle=ang, el0=el, az0=az, el=el, az=az, occluded=occ, changed=False,
                         mode="sun-token")
    a0 = sun_angle(el, az, s)
    if occ:
        return cam, dict(angle0=a0, angle=a0, el0=el, az0=az, el=el, az=az, occluded=True, changed=False,
                         mode="shadow")
    e, a, ang = sun_safe(el, az, s, prev_az=prev_az, side=side)
    changed = abs(e - el) > 1e-6 or abs(_wrap(a - az)) > 1e-6
    out = " ".join(["rel", _num(d), _num(e), _num(a), _num(fov)] + toks) if changed else cam
    return out, dict(angle0=a0, angle=ang, el0=el, az0=az, el=e, az=a, occluded=False, changed=changed,
                     mode="moved" if changed else "kept")


def sun_info_text(sid, info):
    """One log line per shot (fix round 3): the chosen sun angle."""
    if info is None:
        return None
    if info["mode"] == "shadow":
        return "%s sun angle %.0f deg (stage in the planet's shadow: nominal kept)" % (sid, info["angle"])
    if info["changed"]:
        return "%s sun angle %.0f -> %.0f deg (el %.0f -> %.0f, az %.0f -> %.0f)" % (
            sid, info["angle0"], info["angle"], info["el0"], info["el"], info["az0"], info["az"])
    return "%s sun angle %.0f deg (%s)" % (sid, info["angle"], "az from the sun" if info["mode"] == "sun-token"
                                          else "nominal kept")


def lander_geo(r, v_enu, sun_enu, R0=R_KERBIN + 70.0, pad_lat=-0.0972079, omega=2 * _m.pi / 21549.425):
    """Rel-camera geometry for the lander in the pad ENU frame (e, n, u). r: the lander's position (e, n, height -
    Lander.state() replaces the Cartesian up component by the radial height), v_enu: surface velocity
    (Lander.v_true_enu), sun_enu: sun direction at the pad (core.sun_enu). The local up at the stage is radial (it is
    hundreds of km from the pad at entry); the orbital velocity adds the planet's rotation (pole axis in pad ENU)."""
    e, n, h = float(r[0]), float(r[1]), float(r[2])
    rad = R0 + h
    u = _m.sqrt(max(rad * rad - e * e - n * n, 1.0))
    p = [e, n, u]
    pole = [0.0, _m.cos(_m.radians(pad_lat)), _m.sin(_m.radians(pad_lat))]
    rot = [omega * x for x in _cross(pole, p)]
    vs = _v(v_enu)
    return dict(up=_unit(p), vsrf=vs, vobt=[vs[i] + rot[i] for i in range(3)], sun=_v(sun_enu), alt=h, R=R0,
                north=(0.0, 1.0, 0.0))


def stage_frame_fraction(dist, el, az, fov_v, length, diam, pitch=20.0, aspect=16.0 / 9.0):
    """Fraction of the picture WIDTH spanned by a stage (length along fwd pitched `pitch` deg up, diameter diam)
    seen from a rel camera (dist, el, az, vertical fov): projected length + diameter over the frame width at dist."""
    p = _m.radians(pitch)
    axis = (_m.cos(p), 0.0, _m.sin(p))
    o = cam_offset(el, az)
    c = -(o[0] * axis[0] + o[1] * axis[1] + o[2] * axis[2])
    extent = length * _m.sqrt(max(1.0 - c * c, 0.0)) + diam
    width = 2.0 * dist * aspect * _m.tan(_m.radians(0.5 * fov_v))
    return extent / width
