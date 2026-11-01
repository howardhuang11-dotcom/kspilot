"""G5 bench: full mission — pre-launch check, ascent to orbit, payload release, deorbit,
entry, correction burns and a hoverslam landing back on the KSC launch pad.
usage: orbit_recover.py CRAFT [PE_KM AP_KM INC_DEG]   (env HOP_REC=1 records the landing,
HOP_REC=full the whole mission cut by the broadcast director, director shot list;
KSP_SEP_TEST=1 flies only to 15 s after the booster separation and prints RESULT SEPTEST, DR-007;
KSP_Q_SEP_MAX=Pa overrides the separation gate, default 10000;
KSP_AIM="along_m,cross_m" or KSP_AIM_SITE=flat2k lands on an offset target instead of the pad, DR-012 / kspilot/aim.py)"""
import os
import sys
import json
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot import aim, core, feasibility, showrun  # noqa: E402
from kspilot.landing import Lander  # noqa: E402
from kspilot.mission import Mission  # noqa: E402
from kspilot.vehicle import clear_pad  # noqa: E402

craft = sys.argv[1]
ENTRY_JSON = os.path.join(core.LOG_DIR, "g5_entry_%s.json" % craft)   # per craft; legacy shared file as fallback
if not os.path.exists(ENTRY_JSON):
    ENTRY_JSON = os.path.join(core.LOG_DIR, "g5_entry.json")
args = [a for a in sys.argv[2:] if not a.startswith("--")]
pe, ap, inc = (float(x) for x in (args[:3] if len(args) >= 3 else (80, 80, 0)))
tag = "orbit_%s" % craft.replace(" ", "")
log = core.Log(tag)
# offset aim (DR-012, verification process stage 4): only with KSP_AIM / KSP_AIM_SITE; without them AIM is None and
# every call below is exactly the pad run (no extra kwargs, names or RPCs: tests/test_aim.py)
try:
    AIM = aim.from_env()
except ValueError as _e:
    log("RESULT REFUSED (offset aim: %s)" % _e)
    sys.exit(0)
conn = core.connect("orbit")
sc = conn.space_center
_aim_kw, _aim_sfx = {}, ""
if AIM is not None:
    # pre-flight terrain + obstacle check with the game's own data (reads only, before anything flies)
    _kb = sc.bodies["Kerbin"]
    try:
        _ok, AIM_ALT, _msg = aim.check_site(_kb.surface_height, AIM)
    except Exception as _e:
        _ok, AIM_ALT, _msg = False, float("nan"), "terrain height unreadable: %r" % (_e,)
    log("offset aim (DR-012): %s, %.0f m from the pad; %s" % (AIM.describe(), aim.pad_distance(*AIM.latlon), _msg))

    def _ray_down(la, lo, up):
        _rf = _kb.reference_frame
        _p = np.asarray(_kb.surface_position(la, lo, _rf))
        _u = _p / np.linalg.norm(_p)
        return float(sc.raycast_distance(tuple(_p + up * _u), tuple(-_u), _rf))
    _cs, _cmsg = aim.check_clear(_ray_down, AIM)
    log("offset aim obstacle check: %s (%s)" % (_cs, _cmsg))
    if not _ok or _cs == "obstacle":
        log("RESULT REFUSED (offset aim site check failed: %s)" % (_msg if not _ok else _cmsg))
        sys.exit(0)
    _aim_kw, _aim_sfx = {"target": AIM.latlon}, "_aim"     # checkpoints of an offset run never replace the pad ones
m = Mission(conn, pe * 1e3, ap * 1e3, inc, recover=True, log=log, tag=tag, **_aim_kw)
if AIM is not None:
    m.target_alt = AIM_ALT


def load_checkpoint(name, pause=False):
    """pause=True (high-speed checkpoint, --from-mid): freeze the game as soon as the flight scene is back and keep it
    paused until the control loop starts. Unpaused, the ~15-20 s of sleeps / leg probe below cost 1-2 km from g5_term
    (saved 2.5-4 km, loop started at 2015 m) and would cost 5-8 km from g5_mid (480 m/s): the IDENT window lost."""
    global conn, sc
    log("loading checkpoint %s" % name)
    sc.load(name)
    if pause:
        t_ = time.time()
        while time.time() - t_ < 90:
            time.sleep(0.5)
            try:
                c_ = core.connect("orbit")
                v_ = c_.space_center.active_vessel if str(c_.krpc.current_game_scene).endswith("flight") else None
                # the NEW scene: the vessel of the previous run sits on the ground / sea, g5_mid is at ~15 km
                if v_ is not None and v_.flight().mean_altitude > 8000.0:
                    c_.krpc.paused = True
                    conn, sc = c_, c_.space_center
                    m.conn, m.sc = conn, sc
                    log("checkpoint %s loaded and PAUSED %.1f s after load: h=%.0f m (unpaused right before the control "
                        "loop)" % (name, time.time() - t_, v_.flight().mean_altitude))
                    return v_
                c_.close()                     # not there yet: do not pile up connections
            except Exception:
                pass
        log("WARNING: could not pause after loading %s: continuing unpaused" % name)
    time.sleep(8)
    conn = core.connect("orbit")
    sc = conn.space_center
    m.conn, m.sc = conn, sc
    core.wait_until(lambda: str(conn.krpc.current_game_scene).endswith("flight"), 60, 1)
    time.sleep(3)
    return sc.active_vessel


from kspilot import hud  # noqa: E402

DRAG_SETTLE_S = 5.0     # CA-4: >= airbrake travel (70 deg at ~20 deg/s = 3.5 s) + grid-fin animation, after EVERY change
DRAG_REPEAT_TOL = 0.02  # CA-4: two CdA@10km samples further apart than this -> re-measure once
ATT_ERR_DEG, ATT_RATE_DEG_S = 1.0, 0.3   # CA-4: sampling attitude gate (autopilot error, body rate)
ATT_TIMEOUT_S = (40.0, 10.0)            # CA-4: settle timeout, first (initial slew) / later (re-point < 1 deg)
MEAS_BUDGET_S = 75.0                    # CA-4: --from-entry starts at ~76 km falling ~230 m/s: re-measures and long
                                        # settles stop when this is spent (the old measurement took ~42 s)


def measure_drag(v):
    """Drag tables in the configurations the lander flies: OUT = Brakes group on, grid fins explicitly extended
    (the Brakes toggle can leave them folded: the 17:13 GF2T calibration read 27.6 m^2 instead of 38.6),
    airbrakes at 70 deg; IN = same with the airbrakes at 0 deg (continuous) or retracted (bang-bang), grid fins
    still OUT on airbrake craft (CA-1, as the gf-baseline flew); fins folded only on fins-only craft.
    CA-4 (deterministic): the old measurement sampled once after fixed sleeps with only ap.error < 3 deg, and was not
    repeatable (T-WorksFineGF CdA_out@10km 29.3-29.7 on good runs vs 27.1-27.5 on two failures; GF0T 'straight'
    11.6-13.9 in one configuration). Now: after EVERY brake / fin / airbrake-angle change wait DRAG_SETTLE_S and
    verify the state read back (control.brakes, fin deploy events, aeroDeployAngle); before sampling re-point at the
    CURRENT radial up (it drifts ~0.2 deg/s along the orbit) and wait for error < 1 deg AND body rate < 0.3 deg/s
    held 1 s (timeout: logged warning); sample twice, re-measure once if > 2 % apart; OUT is checked again after
    the IN cycle (re-deploy repeatability) and re-measured once if > 2 % off."""
    fin_mods = [mm for p in v.parts.all if "Grid Fin" in p.name for mm in p.modules if mm.name == "ModuleAnimateGeneric"]

    def fins(want):
        ev = "Extend Fins" if want else "Retract Fins"
        for mm in fin_mods:
            if mm.has_event(ev):
                mm.trigger_event(ev)

    def fin_state():
        """'out' / 'in' read back from the deploy events (a module offering 'Retract Fins' is extended), 'mixed',
        'moving' (status field says so), or 'unknown' (nothing readable)."""
        st = set()
        for mm in fin_mods:
            try:
                if any("moving" in str(x).lower() for x in mm.fields.values()):
                    st.add("moving")
                elif mm.has_event("Retract Fins"):
                    st.add("out")
                elif mm.has_event("Extend Fins"):
                    st.add("in")
            except Exception:
                pass
        return st.pop() if len(st) == 1 else "mixed" if st else "unknown"

    ab = [f for p in v.parts.all if "airbrake" in p.name for mm in p.modules if mm.name == "ModuleAeroSurface"
          for f in mm.field_list if f.name == "aeroDeployAngle"]
    has_ab = any("airbrake" in p.name for p in v.parts.all)       # same test as Lander.has_airbrakes
    cont = bool(ab) and os.environ.get("KSP_DMCONT", "0") == "1"   # baseline: bang-bang tables
    has_fins = bool(fin_mods)
    belly = (not ab) and has_fins and os.environ.get("KSP_BELLY", "1") == "1"
    ap = v.auto_pilot                               # the tables assume the engine-first attitude (nose radial up)
    rfb = v.orbit.body.reference_frame
    ap.reference_frame = rfb
    # recording v2 (HOP_REC=full): every wait below is in GAME seconds (core.GAME_CLOCK, 3x wall cap); otherwise
    # core.tmark / tsince / tsleep are exactly time.time() / time.time() - t / time.sleep() (decision log 11:06)
    t_start = core.tmark()

    def left():
        return MEAS_BUDGET_S - core.tsince(t_start)

    def point_up():
        up = np.asarray(v.position(rfb))
        ap.target_direction = tuple(up / np.linalg.norm(up))

    def settle_attitude(label):
        """Re-point at the current radial up; wait for error < 1 deg AND body rate < 0.3 deg/s, held 1 s."""
        point_up()
        tmo = max(3.0, min(ATT_TIMEOUT_S[0 if not settle_attitude.n else 1], left()))
        settle_attitude.n += 1
        t0, ok_t, err, w = core.tmark(), None, float("nan"), float("nan")
        while core.tsince(t0) < tmo:
            try:
                err = float(ap.error)
                w = float(np.degrees(np.linalg.norm(v.angular_velocity(rfb))))
            except Exception:
                err = w = float("nan")
            if err < ATT_ERR_DEG and w < ATT_RATE_DEG_S:
                ok_t = ok_t or core.tmark()
                if core.tsince(ok_t) >= 1.0:
                    return True
            else:
                ok_t = None
            time.sleep(0.25)
        log("WARNING drag table %s: attitude not settled after %.0f s (error %.2f deg, rate %.2f deg/s): sampling "
            "anyway" % (label, tmo, err, w))
        return False
    settle_attitude.n = 0

    def set_config(label, brakes, fins_out, ab_deg):
        """Command a configuration, wait DRAG_SETTLE_S after the last change, verify the read-back (re-command up to
        twice on a mismatch)."""
        for attempt in range(3):
            if attempt == 0 or bool(v.control.brakes) != brakes:
                v.control.brakes = brakes
                core.tsleep(1)                      # the Brakes toggle flips the fins: re-command them after it
            if has_fins:
                fins(fins_out)
            for f in ab:
                f.float_value = ab_deg
            core.tsleep(DRAG_SETTLE_S)
            fs = fin_state() if has_fins else "none"
            angs = [float(f.float_value) for f in ab]
            bad = []
            if bool(v.control.brakes) != brakes:
                bad.append("brakes %s" % v.control.brakes)
            if has_fins and fs != ("out" if fins_out else "in"):
                bad.append("fins %s" % fs)
            if any(abs(a - ab_deg) > 0.5 for a in angs):
                bad.append("airbrake angle %s" % ",".join("%.0f" % a for a in angs))
            if not bad or bad == ["fins unknown"]:   # unreadable fin state: rely on the settle time
                log("drag table %s: brakes %s, fins %s, airbrakes %s deg (read back after %.0f s)" % (
                    label, v.control.brakes, fs, ",".join("%.0f" % a for a in angs) or "-", DRAG_SETTLE_S))
                return True
            log("drag table %s: read-back mismatch (%s): re-commanding" % (label, ", ".join(bad)))
        log("WARNING drag table %s: configuration still not as commanded (%s): sampling anyway" % (label, ", ".join(bad)))
        return False

    def take(label, meas):
        """meas() -> (CdA list, payload). Settle, sample twice; > DRAG_REPEAT_TOL apart: re-settle, re-measure once.
        Returns (CdA list, payload, repeatability fraction)."""
        for attempt in (1, 2):
            settle_attitude(label)
            k1, pay = meas()
            core.tsleep(1.0)
            k2, _ = meas()
            rep = max(abs(a - b) / max(abs(a), 1e-6) for a, b in zip(k1, k2))
            log("drag table %s: CdA@10km %s / %s m^2 (repeatability %.1f %%)" % (
                label, "/".join("%.2f" % a for a in k1), "/".join("%.2f" % b for b in k2), 100 * rep))
            if rep <= DRAG_REPEAT_TOL:
                break
            if attempt == 1 and left() < 15.0:
                log("WARNING drag table %s: samples %.1f %% apart but the time budget is spent: no re-measure" % (
                    label, 100 * rep))
                break
            if attempt == 1:
                log("drag table %s: samples %.1f %% apart (> %.0f %%): re-measuring once" % (
                    label, 100 * rep, 100 * DRAG_REPEAT_TOL))
            else:
                log("WARNING drag table %s: still %.1f %% apart after the re-measure" % (label, 100 * rep))
        return k1, pay, rep

    ap.engaged = True
    set_config("out", True, True, 70.0)
    if belly:
        # no airbrakes: the drag modulator is the angle of attack ("belly"), OUT = 45 deg, IN = 0 deg, fins out
        # the table uses the angle the craft ACHIEVES, not the 45 deg command: the tail fins weathervane it to
        # 10-12 deg above q 2 kPa (flight 20261020_182747: table at 45 deg -> learned k_out 0.28)
        bdeg = float(os.environ.get("KSP_BELLY_TABLE", 12.0))
        rho10 = v.orbit.body.density_at(10000.0)

        def meas_belly():
            t_out = feasibility.drag_table(conn, v, tilt_deg=bdeg)
            t_in = feasibility.drag_table(conn, v)
            i10, j150 = t_out["alts"].index(10000), t_out["speeds"].index(150) if 150 in t_out["speeds"] else 3
            q10 = 0.5 * rho10 * t_out["speeds"][j150] ** 2
            return [t_out["F"][i10][j150] / q10, t_in["F"][i10][j150] / q10], (t_out, t_in)
        (c_out, c_in), (t_out, t_in), rep = take("belly", meas_belly)
        ap.engaged = False
        measure_drag.info = {"rep": rep, "s": round(core.tsince(t_start), 1)}
        log("drag tables measured: CdA@10km belly %.0f deg %.1f / straight %.1f m^2 (repeatability %.1f %%)" % (
            bdeg, c_out, c_in, 100 * rep))
        return c_out, t_out, c_in, t_in

    def meas():
        return [feasibility.estimate_cda_at(conn, v, 10000.0)], feasibility.drag_table(conn, v)
    (c_out,), t_out, rep_out = take("out", meas)
    if cont:
        set_config("in", True, True, 0.0)
    else:
        # airbrake craft: IN = airbrakes retracted, grid fins OUT (the lander re-extends them, CA-1; the Brakes
        # toggle may have folded them); fins-only craft: the fins are the modulator, IN = folded
        set_config("in", False, has_ab, 70.0)
    (c_in,), t_in, rep_in = take("in", meas)
    set_config("out (restored)", True, True, 70.0)
    # re-deploy repeatability: OUT again after the IN cycle (the failures read 27.1-27.5 vs 29.3-29.7)
    settle_attitude("out (restored)")
    c_out2 = feasibility.estimate_cda_at(conn, v, 10000.0)
    rep_cyc = abs(c_out2 - c_out) / max(c_out, 1e-6)
    log("drag table out: CdA@10km %.2f before / %.2f after the IN cycle (re-deploy repeatability %.1f %%)" % (
        c_out, c_out2, 100 * rep_cyc))
    if rep_cyc > DRAG_REPEAT_TOL and left() < 15.0:
        log("WARNING drag table out: re-deploy differs %.1f %% but the time budget is spent: no re-measure" % (
            100 * rep_cyc))
    elif rep_cyc > DRAG_REPEAT_TOL:
        log("drag table out: re-deploy differs %.1f %% (> %.0f %%): re-measuring OUT once" % (
            100 * rep_cyc, 100 * DRAG_REPEAT_TOL))
        c1 = c_out
        (c_out,), t_out, rep_out = take("out (re-measure)", meas)
        log("drag table out: using the re-measure %.2f (first %.2f, after the IN cycle %.2f)" % (c_out, c1, c_out2))
    ap.engaged = False
    rep = max(rep_out, rep_in, rep_cyc)
    measure_drag.info = {"rep": rep, "s": round(core.tsince(t_start), 1), "rep_out": rep_out, "rep_in": rep_in,
                         "rep_cycle": rep_cyc, "cda_out_cycle": c_out2}
    log("drag tables measured: CdA@10km out %.1f / in %.1f m^2 (%s; repeatability %.1f %%: out %.1f / in %.1f / "
        "re-deploy %.1f %%)" % (c_out, c_in, "airbrake angle" if cont else "brakes", 100 * rep, 100 * rep_out,
                               100 * rep_in, 100 * rep_cyc))
    return c_out, t_out, c_in, t_in


FROM_ORBIT, FROM_ENTRY = "--from-orbit" in sys.argv, "--from-entry" in sys.argv
# whole-mission HUD (v7): runs from lift-off (or from the loaded checkpoint) to touchdown
_feed = hud.HudFeed(mission="ORBITAL RETURN", log=log, craft=craft, target_km=ap)
if AIM is not None:
    _feed.dist_label = "TGT"            # the distance readout is to the offset target, not the pad
_feed.phase("orbit" if FROM_ORBIT else ("descent" if FROM_ENTRY or "--from-term" in sys.argv or "--from-mid" in sys.argv else "ascent"))
_feed.start()
FROM_MID = "--from-mid" in sys.argv            # envelope stage 3: checkpoint at ~15 km (g5_mid)
FROM_TERM = "--from-term" in sys.argv          # 1-minute landing tests: checkpoint at ~4 km (KSP_SAVE_TERM)
FROM_ENTRY = FROM_ENTRY or FROM_TERM or FROM_MID
# HOP_REC=full: whole-mission video cut by the broadcast director (director shot list, kspilot/director.py);
# HOP_REC=1 (or any other value): unchanged, the landing only. The director never commands the vessel.
REC_FULL = os.environ.get("HOP_REC") == "full" and not FROM_ENTRY
# recording pipeline v3 (decision record DR-011): HOP_REC=full records with OBS in real time (kspilot/obsrec.py,
# tools/edit_obs.py); KSP_REC_ENGINE=frames keeps the v2 plugin frame capture (fallback). REC_GATE: the DR-011 gate,
# an OBS recording of a checkpoint entry + landing (--from-entry / --from-term / --from-mid with HOP_REC=full).
REC_OBS = os.environ.get("HOP_REC") == "full" and os.environ.get("KSP_REC_ENGINE", "obs").strip().lower() != "frames"
REC_GATE = REC_OBS and FROM_ENTRY
_director, frames = None, None
_obs = None           # REC_OBS only: the running kspilot.obsrec.Recording
if REC_OBS:
    from kspilot import obsrec  # noqa: E402
_rec_extra = {}       # REC_FULL only (fix round 3): commanded release UT, deorbit Pe / dv for the editor (events.json)
if REC_FULL:
    # recording pipeline v2 (director v2 specification, video requirements V1-V10): v2 director, countdown, frame index +
    # physics-warp-compensated capture, lander v2 cameras, post-production cut by tools/edit_video.py. ONLY here:
    # without HOP_REC=full none of these code paths runs (baseline protection).
    from kspilot import showrun  # noqa: E402
    from kspilot.director import Director  # noqa: E402
    _director = Director(log=log, v2=True)
    _feed.v2 = True
    # V17 (video requirements v2): the deorbit burn is on the night side -> the core's lamps (craft *-LIT,
    # tools/craft_lights.py: 8 x Spotlight Mk1, Light action group) are on for every node burn and off again after
    # it. Not for the whole recording: 0.16 EC/s for a 3-5 h coast would empty the 1000 EC battery (kspilot/power.py,
    # the budget does not know the lamps). A craft without lamps: no effect. REC_FULL only (baseline protection).
    from kspilot import burn as _burn_mod  # noqa: E402
    _exec_node_unlit = _burn_mod.execute_node

    def _set_lights(vessel, on):
        try:
            vessel.control.lights = on
            log("lights %s (V17)" % ("on" if on else "off"))
        except Exception as _e:
            log("lights %s failed: %r" % ("on" if on else "off", _e))

    def _exec_node_lit(conn_, vessel, node, log_, *a, **k):
        _set_lights(vessel, True)
        try:
            return _exec_node_unlit(conn_, vessel, node, log_, *a, **k)
        finally:
            _set_lights(vessel, False)
    _burn_mod.execute_node = _exec_node_lit
    # fix round 2 (decision log 11:06, QA 2026-10-21 defect 4), REC_FULL ONLY: settle / hold waits in game
    # seconds (3x wall cap), and no time warp inside a protected 1x window of the editor (core.warp_to waits it out)
    core.GAME_CLOCK = core.GameClock(lambda: sc.ut)

    def _rec_windows(now):
        with _feed.lock:
            ev = {k: tuple(x) for k, x in _feed.ev.items()}
        return showrun.live_windows(ev, list(_director.burns), _feed.cd, now)
    core.WARP_GUARD = core.WarpGuard(lambda: sc.ut, _rec_windows, log=log)
if FROM_ORBIT or FROM_ENTRY:
    pass
elif "--resume" in sys.argv:          # vehicle already in orbit with the payload still attached
    from kspilot import vehicle  # noqa: E402
    m.payload_stage = vehicle.payload_stage(sc.active_vessel)
    log("resuming in orbit, payload stage %s" % m.payload_stage)
else:
    sc.launch_vessel("VAB", craft, "LaunchPad", [], True)
    time.sleep(6)
    if os.environ.get("HOP_REC"):
        # the landing comes ~half a Kerbin day after launch (the pad re-enters the orbit plane), so the
        # landing light is decided at launch: lift off before dawn -> land in the afternoon
        # (a whole-mission video also shows the liftoff: launch just after sunrise; the deorbit still waits for
        # a daylight landing pass)
        core.set_daylight(conn, float(os.environ.get("KSP_LAUNCH_SUN", 8.0 if REC_FULL else -40.0)), afternoon=False, log=log)
        time.sleep(3)
    ok, rep = m.check()
    if not ok:
        log("RESULT REFUSED (pre-launch check)")
        sys.exit(0)
    if REC_FULL:
        # V1 countdown (spec 5, C3): the S1 pad camera is set first, the recording starts at T-11 (the editor drops
        # the first second), the HUD counts T-10 -> 0 against the planned lift-off, ignition waits for T0 on the
        # GAME clock. Only a 10 s wait on the pad (no dynamics); the daylight warp above is already done.
        _director.start()
        time.sleep(2.0)
        if REC_OBS:                     # v3: OBS in real time (blocks until OBS records; T-11 counts from then)
            try:
                _obs = obsrec.Recording(tag, log=log).start()
            except Exception as _e:
                log("RESULT REFUSED (OBS recording failed: %r)" % (_e,))
                sys.exit(0)
            frames = _obs.base
        else:
            frames = core.record_start(tag, v2=True)
        _cd = sc.ut + showrun.REC_LEAD
        _feed.cd = _cd
        _feed.plan("LIFTOFF", _cd)
        log("countdown: recording, lift-off planned at UT %.1f (T-%.0f s)" % (_cd, showrun.REC_LEAD))
        _w0 = time.time()
        while sc.ut < _cd and time.time() - _w0 < 120.0:
            time.sleep(0.05)
    t_launch = time.time()
    t_launch_ut = sc.ut                     # HUD mission clock (T+)
    if os.environ.get("KSP_SEP_TEST") == "1":
        # separation test (DR-007): the ascent only until 15 s after the booster staging, then the engine off and
        # the result; no recovery. The heading follows INC_DEG as in a mission (45 -> 41.7 deg)
        from kspilot import vehicle  # noqa: E402
        from kspilot.ascent_own import Ascent  # noqa: E402
        a = Ascent(conn, pe * 1e3, ap * 1e3, inc, log, autostage_limit=vehicle.autostage_limit(sc.active_vessel),
                   sep_test_s=15.0)
        a.fly()
        s = a.sep
        if s is None or "parts_end" not in s:
            log("RESULT FAIL ascent (separation test: no booster staging; %s)" % (a.lost or "no abort"))
            sys.exit(0)
        hit = bool(a.lost or s["core"] or s["core_window"])
        log("RESULT SEPTEST parts=%d thr=%.0f explosions=%d core_hit=%d q=%.1f roll=%.1f sideslip=%.1f aoa=%.1f "
            "offset=%.1f explosions_15s=%d q_sep_max=%.0f" % (
                s["parts_end"], s["thr_end"] / 1e3, len(s["metric"]), hit, s["q"] / 1e3, s["roll"], s["beta"],
                s["aoa"], s["offset"], s["n_window"], a.q_sep_max))
        sys.exit(0)
    if not m.fly_ascent():
        log("RESULT FAIL ascent")
        sys.exit(0)
    log("orbit reached in %.0fs" % (time.time() - t_launch))
if REC_GATE:
    # DR-011 gate: OBS starts BEFORE the checkpoint loads (OBS can take seconds, once ~30 s, to start; the stage
    # must not fall while the script waits for it); the time map starts once the flight scene is back
    try:
        _obs = obsrec.Recording(tag, log=log).start_obs()
    except Exception as _e:
        log("RESULT REFUSED (OBS recording failed: %r)" % (_e,))
        sys.exit(0)
    frames = _obs.base
    _feed.v2 = True                     # the recording's HUD (rec=full: no game-warp tag, the editor badges)
if FROM_ENTRY:
    # per-craft checkpoint / tables (configuration management, verification process: a GF run once paired the GF save
    # with GF0T tables); KSP_TERM_SAVE / KSP_ENTRY_SAVE override the save, ENTRY_JSON picks the craft's file
    _save = (os.environ.get("KSP_TERM_SAVE", "g5_term" + _aim_sfx) if FROM_TERM else
             os.environ.get("KSP_MID_SAVE", "g5_mid" + _aim_sfx) if FROM_MID else
             os.environ.get("KSP_ENTRY_SAVE", "g5_entry" + _aim_sfx))
    v = load_checkpoint(_save, pause=FROM_MID and os.environ.get("KSP_MID_PAUSE", "1") == "1")
    if REC_GATE:
        _obs.start_timemap()
    _feed.resume(_save)                 # T+ continues from the lift-off UT stored with the save (else the MET)
    import json  # noqa: E402
    _dm = json.load(open(ENTRY_JSON))
    if not (FROM_TERM or FROM_MID):      # tables as flown (older checkpoints were calibrated with folded fins)
        _dm["cda"], _dm["tab_out"], _dm["cda_in"], _dm["tab_in"] = measure_drag(v)
        _dm["drag_meas"] = getattr(measure_drag, "info", None)     # CA-4 repeatability
        json.dump(_dm, open(ENTRY_JSON, "w"))
    cda = _dm["cda"]
    if "cda_in" not in _dm:   # older checkpoint: probe the brakes-in drag area now (still above the air)
        v.control.brakes = False
        time.sleep(2)
        _dm["cda_in"] = feasibility.estimate_cda_at(conn, v, 10000.0)
        v.control.brakes = True
        json.dump(_dm, open(ENTRY_JSON, "w"))
        log("core CdA brakes in = %.2f m^2" % _dm["cda_in"])
else:
    if FROM_ORBIT:
        v = load_checkpoint("g5_orbit")
        _feed.resume("g5_orbit")
    if REC_FULL and frames is None:     # from a checkpoint / --resume: the video starts in orbit
        if REC_OBS:
            try:
                _obs = obsrec.Recording(tag, log=log).start()
            except Exception as _e:
                log("RESULT REFUSED (OBS recording failed: %r)" % (_e,))
                sys.exit(0)
            frames = _obs.base
        else:
            frames = core.record_start(tag, v2=True)
        _director.start()
    if not FROM_ORBIT:
        clear_pad(sc, sc.active_vessel, log)
        core.screenshot(tag + "_orbit", clean=not REC_OBS)   # v3: a "clean" shot re-shows the stock UI in the video
        if REC_FULL and m.payload_stage is not None:
            # V4/V8 fix round 2 (REC_FULL only): announce the release PRE_SEP_LEAD s ahead (HUD plan) and fly it on
            # time (game clock), so the director cuts to the release close-up S5 at release - 10 s (V4 1x window).
            # Fix round 3: lead 16 s (editor 1x window from release - 12 s), announced only once no warp is running
            core.wait_until(lambda: sc.rails_warp_factor == 0 and sc.physics_warp_factor == 0, 10.0)
            _sep = sc.ut + showrun.PRE_SEP_LEAD
            _feed.plan("PAYLOAD SEP", _sep)
            log("payload release planned at UT %.1f (in %.0f s, recording)" % (_sep, showrun.PRE_SEP_LEAD))
            core.tsleep(max(_sep - sc.ut, 0.0))
            _rec_extra["sep_cmd"] = sc.ut          # the staging command UT (the HUD event is logged after a wait)
        v = m.release_payload()
        sc.save("g5_orbit")
        _feed.checkpoint("g5_orbit")    # lift-off UT stored with the save (T+ clock of later checkpoint runs)
        log("checkpoint saved: g5_orbit")
if not FROM_ENTRY:
  log("flying core: %s mass %.1ft fuel %.0f" % (v.name, v.mass / 1e3, v.resources.amount('LiquidFuel')))
  v.control.brakes = True
  rf = v.orbit.body.reference_frame
  ap_ = v.auto_pilot
  ap_.reference_frame = rf
  up = np.asarray(v.position(rf))
  ap_.target_direction = tuple(up / np.linalg.norm(up))   # nose up = engine-first attitude for the CdA probe
  ap_.engaged = True
  core.wait_until(lambda: ap_.error < 3, 60, 0.5)
  cda, tab_out, cda_in, tab_in = measure_drag(v)
  from kspilot.entry import DragTable  # noqa: E402
  drag_out = DragTable(tab_out["alts"], tab_out["speeds"], tab_out["F"])
  _cal = os.path.join(core.LOG_DIR, "drag_calib_%s.json" % craft)   # per craft (GF2T is not GF)
  if os.path.exists(_cal):   # learned correction from previous flights
      from kspilot.entry import CalibratedDrag  # noqa: E402
      _c = json.load(open(_cal)) if "json" in dir() else __import__("json").load(open(_cal))
      drag_out = CalibratedDrag(drag_out, _c["alts"], _c.get("k_out", _c["k"]))
      log("drag calibration applied (%d flights, %s)" % (_c.get("flights", 0), _c.get("semantics", "pre-CA-4")))
  log("drag table (brakes out) CdA@100m/s,10km=%.1f  @2000m/s,40km=%.1f" % (
      drag_out(10000, 100) / (0.5 * 0.41 * 1e4) if False else tab_out["F"][3][2] / (0.5 * v.orbit.body.density_at(10000) * 1e4),
      tab_out["F"][7][12] / (0.5 * v.orbit.body.density_at(40000) * 2000 ** 2)))
  log("core CdA brakes in = %.2f m^2" % cda_in)
  ap_.engaged = False   # hand attitude back (the burn executor engages it per node)
  log("core CdA (brakes out, 10 km) = %.2f m^2" % cda)
  if REC_FULL:          # fix round 3: periapsis before / after the deorbit burn for the editor's deorbit graphic
      try:
          _rec_extra["pe0_km"] = v.orbit.periapsis_altitude / 1e3
      except Exception:
          pass
  plan, _ = m.deorbit(v, cda=cda, aim_bias=0.0, daylight=bool(os.environ.get("HOP_REC")), drag=drag_out)
  if REC_FULL:
      try:
          _rec_extra.update(pe1_km=v.orbit.periapsis_altitude / 1e3, dv=float(plan.dv))
      except Exception:
          pass
  _bf = os.path.join(core.LOG_DIR, "aim_bias.json")
  _cb = json.load(open(_bf)).get("cross", 0.0) if os.path.exists(_bf) else 0.0
  log("learned cross-track offset: %+.0f m" % _cb)
  m.coast_and_correct(v, cda, drag=drag_out, cross_bias=-_cb)
  sc.save("g5_entry" + _aim_sfx)
  _feed.checkpoint("g5_entry" + _aim_sfx)
  import json  # noqa: E402
  _ej = {"craft": craft, "cda": cda, "cda_in": cda_in, "tab_out": tab_out, "tab_in": tab_in, "heading": m.heading,
         "drag_meas": getattr(measure_drag, "info", None)}     # CA-4 repeatability
  json.dump(_ej, open(os.path.join(core.LOG_DIR, "g5_entry.json"), "w"))
  json.dump(_ej, open(os.path.join(core.LOG_DIR, "g5_entry_%s.json" % craft), "w"))
  log("checkpoint saved: g5_entry" + _aim_sfx)
if os.environ.get("HOP_REC"):
    log("sun elevation at KSC: %.0f deg" % core.sun_elevation(conn))
if _director is not None:
    _director.handoff()                 # S7-S9 belong to the lander (Lander.direct_camera)
if os.environ.get("HOP_REC"):
    core.set_camera(conn, 70.0, 10.0)
if os.environ.get("HOP_REC") and frames is None:
    frames = core.record_start(tag)
# CoM height above the leg feet: probed with the legs down, which needs thin air and a running game. From g5_mid
# (q ~75 kPa, paused) it must come from the from-entry probe stored per craft (or KSP_HCOM; GF0T measured 7.48 m)
_hcom = float(os.environ["KSP_HCOM"]) if "KSP_HCOM" in os.environ else (_dm.get("h_com") if FROM_MID else None)
if FROM_MID and _hcom is None:
    log("RESULT REFUSED (--from-mid needs the CoM height: KSP_HCOM or h_com in %s)" % ENTRY_JSON)
    sys.exit(0)
lander = Lander(conn, v, log=log, overlay_tag="KSPILOT · ORBITAL RETURN TO PAD", h_com=_hcom,
                **(dict(target=AIM.latlon, target_alt=AIM_ALT) if AIM is not None else {}))
if AIM is not None:
    lander.save_suffix = _aim_sfx
if FROM_ENTRY and not (FROM_MID or FROM_TERM) and "KSP_HCOM" not in os.environ:
    _dm["h_com"] = lander.h_com          # for later --from-mid runs of this craft (probed above the air)
    json.dump(_dm, open(ENTRY_JSON, "w"))
lander.draw = bool(os.environ.get("HOP_REC"))
lander.rec_v2 = REC_FULL or REC_GATE   # v2 lander cameras (V7 rotation, V10 drone); False = unchanged
lander.t_launch_ut = globals().get("t_launch_ut")
_bias_f = os.path.join(core.LOG_DIR, "aim_bias.json")
_bias = json.load(open(_bias_f)) if os.path.exists(_bias_f) else {"along": 0.0, "flights": 0}
lander.aim_offset = _bias["along"]
if FROM_TERM or FROM_MID:
    lander.entry_done = True       # the entry burn already happened before this checkpoint
if FROM_MID:
    lander._mid_saved = True       # never overwrite the reference g5_mid with a state from its own replay
log("learned aim offset: %+.0f m (%d flights)" % (_bias["along"], _bias["flights"]))
try:
    import json  # noqa: E402
    dm = json.load(open(ENTRY_JSON))
    if dm.get("craft", craft) != craft:
        log("WARNING: drag tables belong to %s, not %s" % (dm.get("craft"), craft))
    lander.cda_out, lander.cda_in = dm["cda"], dm.get("cda_in")
    lander.heading = dm.get("heading")
    if "tab_out" in dm:
        from kspilot.entry import DragTable  # noqa: E402
        lander.drag_out = DragTable(dm["tab_out"]["alts"], dm["tab_out"]["speeds"], dm["tab_out"]["F"])
        lander.drag_in = DragTable(dm["tab_in"]["alts"], dm["tab_in"]["speeds"], dm["tab_in"]["F"])
        _cal = os.path.join(core.LOG_DIR, "drag_calib_%s.json" % craft)   # per craft (GF2T is not GF)
        if os.path.exists(_cal):
            from kspilot.entry import CalibratedDrag  # noqa: E402
            _c = json.load(open(_cal))
            lander.drag_out = CalibratedDrag(lander.drag_out, _c["alts"], _c.get("k_out", _c["k"]))
            lander.drag_in = CalibratedDrag(lander.drag_in, _c["alts"], _c.get("k_in", _c["k"]))   # per state
    log("drag modulation: CdA out %.1f in %s" % (lander.cda_out, lander.cda_in))
except Exception:
    pass
_feed.phase("descent")
try:
    if conn.krpc.paused:
        conn.krpc.paused = False
        log("unpaused: control loop starting at h=%.0f m" % v.flight().mean_altitude)
except Exception as _e:
    log("unpause failed:", repr(_e))
_lander_ut = sc.ut if (REC_FULL or REC_GATE) else None     # v2/v3 editor: lander start (fallback start of P10)
res = lander.run(timeout=1500)
_feed.stop(hold=True)          # no collapse after the landing: the final HUD frame holds until the video ends
# offset aim: miss is to the offset target; the rung's criterion is verification process stage 4 (< 200 m, KSP_AIM_TOL)
_miss_tol = 20 if AIM is None else float(os.environ.get("KSP_AIM_TOL", 200.0))
ok = res['situation'].endswith('landed') and res['miss'] < _miss_tol and res['v_touchdown'] < 4 and res['upright'] > 0.95
log("RESULT %s miss=%.1fm v_td=%.2fm/s upright=%.3f fuel_left=%.0f parts=%d %s" % (
    "PASS" if ok else "FAIL", res['miss'], res['v_touchdown'], res['upright'], res['fuel_left'], res['parts'],
    res['situation']) + ("" if AIM is None else " aim=%s miss_tol=%.0fm pad_dist=%.0fm" % (
        AIM.name, _miss_tol, res.get("pad_dist", float("nan")))))
for e in lander.events:
    log("event", e)
def _smooth():
    """Attitude smoothness of the last 3 km: RMS attitude error, RMS pitch/yaw rate, max |roll rate| (deg, deg/s)."""
    import csv as _csv, glob as _glob
    try:
        f = max(_glob.glob(os.path.join(core.LOG_DIR, "*_landing.csv")), key=os.path.getmtime)
        rows = [r for r in _csv.DictReader(open(f)) if r.get("roll_rate") not in (None, "", "nan") and 0 < float(r["h"]) < 3000]
        if not rows:
            return None
        ae = [float(r["ap_err"]) for r in rows]
        py = [float(r["py_rate"]) for r in rows]
        rr = [abs(float(r["roll_rate"])) for r in rows]
        rms = lambda x: (sum(v * v for v in x) / len(x)) ** 0.5
        return dict(ap_err_rms=round(rms(ae), 2), py_rate_rms=round(rms(py), 2), roll_rate_max=round(max(rr), 1),
                    roll_rate_rms=round(rms(rr), 2))
    except Exception:
        return None


def _ident_summary():
    """KSP_IDENT flights: one row per AoA step from this flight's landing CSV. Settled values = mean over the last
    40 % of the hold; baseline = last 1.5 s at 0 deg before the step. CdA = drag / q (m^2); sat = fraction of the
    settled samples with |ctl_p| or |ctl_y| > 0.95 (attitude authority limit). Normal force: tools/aero_truth.py."""
    import csv as _csv
    try:
        rows = [r for r in _csv.DictReader(open(log.path.replace(".txt", "_landing.csv")))
                if r.get("ident_cmd") not in (None, "", "nan")]
        f = lambda r, k: float(r[k])
        mean = lambda x: sum(x) / len(x) if x else float("nan")
        segs, cur = [], None
        for r in rows:
            c = f(r, "ident_cmd")
            if cur is None or c != cur[0] or f(r, "ut") - f(cur[1][-1], "ut") > 1.0:
                cur = (c, [])
                segs.append(cur)
            cur[1].append(r)
        out = []
        for i, (c, seg) in enumerate(segs):
            if c <= 0:
                continue
            t0, t1 = f(seg[0], "ut"), f(seg[-1], "ut")
            st = [r for r in seg if f(r, "ut") >= t1 - 0.4 * (t1 - t0)]
            base = [r for r in (segs[i - 1][1] if i > 0 and segs[i - 1][0] == 0 else [])
                    if f(r, "ut") >= f(segs[i - 1][1][-1], "ut") - 1.5]
            cda = lambda rs: mean([f(r, "drag") / max(f(r, "q"), 1.0) for r in rs])
            a0, a_set = mean([f(r, "aoa") for r in base]) if base else f(seg[0], "aoa"), mean([f(r, "aoa") for r in st])
            def t90(target):        # time from the step start to 90 % of the change target (None: never reached)
                if abs(target) < 0.2:
                    return None
                hit = next((f(r, "ut") - t0 for r in seg if (f(r, "aoa") - a0) / target >= 0.9), None)
                return None if hit is None else round(hit, 1)
            out.append(dict(cmd=c, t=round(t1 - t0, 1), q_kpa=round(mean([f(r, "q") for r in st]) / 1e3, 1),
                            h_km=round(mean([f(r, "h") for r in st]) / 1e3, 2),
                            aoa=round(mean([f(r, "aoa") for r in st]), 2),
                            aoa_max=round(max(f(r, "aoa") for r in seg), 2),
                            aoa_base=round(mean([f(r, "aoa") for r in base]), 2),
                            cda=round(cda(st), 2), cda_base=round(cda(base), 2),
                            sat=round(mean([1.0 if max(abs(f(r, "ctl_p")), abs(f(r, "ctl_y"))) > 0.95 else 0.0
                                            for r in st]), 2),
                            t90_cmd=t90(c), t90=t90(a_set - a0),
                            ctl_max=round(max(max(abs(f(r, "ctl_p")), abs(f(r, "ctl_y"))) for r in seg), 3),
                            ctl_r_max=round(max(abs(f(r, "ctl_r")) for r in seg), 3)))
        for s_ in out:
            log("IDENT step %(cmd).0f deg (%(t).1f s) q %(q_kpa).1f kPa h %(h_km).1f km: aoa %(aoa).2f (max %(aoa_max).2f, "
                "base %(aoa_base).2f) deg  CdA %(cda).2f (base %(cda_base).2f) m^2  ctl sat %(sat).2f  max |ctl p/y| %(ctl_max).2f "
                "|r| %(ctl_r_max).2f  t90 %(t90)s s (of achieved), %(t90_cmd)s s (of command)" % s_)
        return out
    except Exception as e_:
        log("ident summary failed:", repr(e_))
        return None


_ident = _ident_summary() if lander.ident else None
# experiment record (for the write-up): one JSON line per flight with config, code version and outcome
try:
    import subprocess as _sp, datetime as _dt
    _rev = _sp.run(["git", "rev-parse", "--short", "HEAD"], cwd=core.ROOT, capture_output=True, text=True).stdout.strip()
    _ign = next((e for e in lander.events if e and e[0] == "ignition"), None)
    _rec = dict(time=_dt.datetime.now().isoformat(timespec="seconds"), craft=craft, git=_rev,
                start="term" if FROM_TERM else "mid" if FROM_MID else "entry" if FROM_ENTRY else "orbit" if FROM_ORBIT else "pad",
                env={k: v for k, v in os.environ.items() if k.startswith("KSP_")},
                params=dict(glide_tmin=lander.glide_tmin, glide_max_deg=lander.glide_max_deg,
                            glide_below=lander.glide_below, glide_c=lander.glide_c_per_kg,
                            fin_steer=lander.fin_steer, has_fins=lander.has_fins, kp_lat=lander.kp_lat,
                            aim_offset=lander.aim_offset),
                ignition=dict(h=_ign[1], vz=float(_ign[2]), offset=_ign[3]) if _ign else None,
                result=dict(ok=bool(ok), miss=round(float(res["miss"]), 2), v_td=round(float(res["v_touchdown"]), 2),
                            upright=round(float(res["upright"]), 3), fuel_left=round(float(res["fuel_left"]), 1),
                            parts=res["parts"], situation=str(res["situation"])),
                log=getattr(log, "path", ""), smooth=_smooth())
    if os.environ.get("KSP_ABL_TAG"):   # EXPLORATORY ablation / dispersion record (report Section 4.8)
        _rec["exploratory"] = os.environ["KSP_ABL_TAG"]
        _rec["result"]["miss_td"] = round(float(res.get("miss_td", float("nan"))), 2)
    if AIM is not None:
        _rec["aim"] = dict(name=AIM.name, lat=AIM.lat, lon=AIM.lon, along=AIM.along, cross=AIM.cross, az=AIM.az,
                           terrain=AIM_ALT, miss_tol=_miss_tol, pad_dist=res.get("pad_dist"))
    if lander.ident:
        _rec["ident"] = dict(steps=_ident, abort=lander._id_abort, window_open_h=lander._id_open)
    with open(os.path.join(core.LOG_DIR, "experiments.jsonl"), "a") as _f:
        _f.write(json.dumps(_rec) + "\n")
    log("smoothness (last 3 km):", _rec.get("smooth"))
except Exception as _e:
    log("experiment record failed:", repr(_e))
if getattr(lander, "ignition_offset", None) and lander.heading is not None and res['situation'] != "destroyed":
    import math  # noqa: E402
    hd = math.radians(lander.heading)
    along = lander.ignition_offset[0] * math.sin(hd) + lander.ignition_offset[1] * math.cos(hd)
    cross = lander.ignition_offset[0] * math.cos(hd) - lander.ignition_offset[1] * math.sin(hd)   # + = right of track
    # aim-offset learning is OFF by default: with fin steering + early ignition the ignition offset is no longer
    # a systematic error (the burn removes it) and its +-40 m scatter made the aim drift (0.3 -> 1.9 -> 10.6 m)
    if (abs(along) < 2000 and ok and os.environ.get("KSP_AIMLEARN") and not os.environ.get("KSP_NOLEARN")
            and AIM is None):           # the pad's learned bias is not trained on offset-aim flights
        # slow, capped learning (0.3 x, at most 30 m per flight): one noisy flight must not swing the aim
        _step = lambda e: max(-30.0, min(30.0, 0.3 * e))
        _bias = {"along": _bias["along"] + _step(along), "cross": _bias.get("cross", 0.0) + _step(cross),
                 "flights": _bias["flights"] + 1}
        json.dump(_bias, open(_bias_f, "w"))
        log("aim offset updated: ignition along %+.0f m cross %+.0f m -> offsets %+.0f / %+.0f m" % (
            along, cross, _bias["along"], _bias["cross"]))
print("shot:", core.screenshot(tag, clean=not REC_OBS))   # v3 (OBS films the window): never re-show the stock UI
if frames and (REC_FULL or REC_GATE):
    # V6 / C4: keep recording until touchdown + 15 s of GAME time, then stop without encoding; tools/edit_video.py
    # (v2 frames) / tools/edit_obs.py (v3 OBS) cut by UT (no global speed-up: 1x windows whole, badges on every
    # compressed span, V9)
    _td = _feed.ev.get("TOUCHDOWN")
    if _td and _td[1]:
        _w0 = time.time()
        while sc.ut < _td[0] + showrun.TD_TAIL and time.time() - _w0 < 120.0:
            time.sleep(0.2)
        log("recording: touchdown + %.1f s (game)" % (sc.ut - _td[0]))
    if _obs is not None:
        _obs.stop()
    else:
        core.record_halt()
    with open(_obs.events_path if _obs is not None else os.path.join(frames, "events.json"), "w") as _f:
        json.dump(dict(ev={k: list(x) for k, x in _feed.ev.items()}, marks=dict(_feed.marks), cd=_feed.cd, t0=_feed.t0,
                       burns=_director.burns if _director else [], period=_director.period if _director else None,
                       lander=_lander_ut, tag=tag, planned={k: list(x) for k, x in _feed.planned.items()},
                       start="term" if FROM_TERM else "mid" if FROM_MID else "entry" if FROM_ENTRY else
                       "orbit" if FROM_ORBIT else "pad", **_rec_extra), _f, indent=1)
    import subprocess as _sp2  # noqa: E402
    sys.path.insert(0, os.path.join(core.ROOT, "tools"))
    from edit_video import python_with_pil  # noqa: E402
    _py = python_with_pil()             # the editor needs Pillow, not kRPC (the venv has no Pillow)
    _ed, _src = ("edit_obs.py", _obs.meta_path) if _obs is not None else ("edit_video.py", frames)
    if _py is None:
        log("video: no python with Pillow found: sources kept in %s (run tools/%s by hand)" % (_src, _ed))
    else:
        _r = _sp2.run([_py, os.path.join(core.ROOT, "tools", _ed), _src, "--name", tag])
        if _r.returncode != 0:
            log("video: %s failed (rc %d): sources kept in %s" % (_ed, _r.returncode, _src))
elif frames:
    print("video:", core.record_stop(frames, tag, fast=2.0, tail_s=25.0))
_feed.clear()                  # end of the held HUD (after the screenshot and the video)
core.camera_shot("chase")      # hand the camera back after the landing shot
import subprocess  # noqa: E402
if not os.environ.get("KSP_NOLEARN"):     # controlled experiments: identical parameters every flight
    subprocess.run([sys.executable, os.path.join(core.ROOT, "tools", "calibrate_drag.py")])   # learn from this flight
