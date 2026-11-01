"""Connection, logging and small helpers shared by all kspilot modules."""
import math
import os
import time

import krpc

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = os.path.join(ROOT, "logs")

# KSC launch pad (MechJeb LandingSites.cfg / KSP)
KSC_PAD = (-0.0972079, -74.557673)


def connect(name="kspilot"):
    return krpc.connect(name=name)


class Log:
    """Event log written to logs/<stamp>_<tag>.txt and echoed to stdout."""

    def __init__(self, tag):
        os.makedirs(LOG_DIR, exist_ok=True)
        self.t0 = time.time()
        self.path = os.path.join(LOG_DIR, time.strftime("%Y%m%d_%H%M%S") + "_" + tag + ".txt")
        self.f = open(self.path, "w")

    def __call__(self, *msg):
        line = "[%7.1fs] %s" % (time.time() - self.t0, " ".join(str(m) for m in msg))
        print(line, flush=True)
        self.f.write(line + "\n")
        self.f.flush()


def wait_until(pred, timeout, period=0.2):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if pred():
            return True
        time.sleep(period)
    return False


# ---------------------------------------------------------------------------------------------------------------------
# Recording pipeline v2 (HOP_REC=full ONLY, tools/orbit_recover.py REC_FULL): game-clock waits and the 1x warp guard.
# Both hooks stay None unless the flight script installs them for a whole-mission recording. With None every helper
# below IS the call it replaces (time.time() / time.time() - m / time.sleep(d) / sc.warp_to(ut)): no extra RPC, no
# extra sleep, same order of calls -> non-recording missions run bit-identical code paths (tests/test_rec_fix2.py).
# ---------------------------------------------------------------------------------------------------------------------
GAME_CLOCK = None        # GameClock while recording: settle / hold waits are measured in game seconds
WARP_GUARD = None        # WarpGuard while recording: no time warp starts inside a protected 1x window of the editor
REC_WAIT_CAP = 3.0       # a game-clock wait of D s ends after D game s, or at the latest after 3 x D wall s
REC_OBS = None           # recording pipeline v3 (DR-011): the running kspilot.obsrec.Recording (OBS records in real
                         # time; nothing is paused: record_pause is a no-op). None = no v3 recording (plugin semantics)


class GameClock:
    """Waits in GAME seconds (decision log 2026-10-21 11:06: while frames are captured the game runs at ~0.35-0.5x real
    time, so a 10 s wall-clock settle was only ~4 s of game time). mark() -> (ut, wall); elapsed(m) =
    max(game s, wall s / cap): a wait of D game s also ends after cap x D wall s (game paused / UT unreadable)."""

    def __init__(self, ut_fn, cap=REC_WAIT_CAP, wall=None, sleep=None, poll=0.05):
        self.ut_fn, self.cap, self.poll = ut_fn, cap, poll
        self.wall = wall or time.time
        self._sleep = sleep or time.sleep

    def _ut(self):
        try:
            return float(self.ut_fn())
        except Exception:
            return None

    def mark(self):
        return (self._ut(), self.wall())

    def elapsed(self, m):
        if not isinstance(m, tuple):              # a wall-clock stamp taken before the clock was installed
            return self.wall() - m
        u0, w0 = m
        dw = self.wall() - w0
        u = self._ut()
        if u is None or u0 is None:
            return dw
        return max(u - u0, dw / self.cap)

    def sleep(self, d):
        if d <= 0:
            return
        m = self.mark()
        while self.elapsed(m) < d:
            self._sleep(self.poll)


def tmark():
    """Start stamp of a timed wait: time.time(); recording v2: a GameClock mark."""
    return time.time() if GAME_CLOCK is None else GAME_CLOCK.mark()


def tsince(m):
    """Seconds since tmark(): time.time() - m; recording v2: game seconds (3x wall cap)."""
    return time.time() - m if GAME_CLOCK is None else GAME_CLOCK.elapsed(m)


def tsleep(d):
    """time.sleep(d); recording v2: d game seconds (3x wall cap)."""
    if GAME_CLOCK is None:
        time.sleep(d)
    else:
        GAME_CLOCK.sleep(d)


def warp_to(sc, ut):
    """sc.warp_to(ut); recording v2: first wait (game clock) until no protected 1x window is open (WarpGuard), and
    skip the warp if its target is reached meanwhile."""
    if WARP_GUARD is not None and not WARP_GUARD.hold(ut):
        return
    sc.warp_to(ut)


class WarpGuard:
    """Recording v2 only (fix round 2, QA defect 4): after the insertion burn the trim executor warped 1.6 s after
    cutoff (KSP.log 'Packing ... for orbit', warp 6 -> 50) inside the editor's 1x window cutoff + 10 s: 9 game s
    were captured at 0.2-0.4 s per frame and played ~2x without a badge. Every time warp now first waits (game
    clock) until the 1x windows open at the present UT are over + showrun.WARP_PAD: the editor's own windows from
    the live events (windows_fn(now) -> [(ut0, ut1)]) and cutoff + showrun.POST_ORBIT after every executor burn."""

    def __init__(self, ut_fn, windows_fn, log=print, cap=REC_WAIT_CAP, max_wait=120.0, wall=None, sleep=None):
        self.ut_fn, self.windows_fn, self.log = ut_fn, windows_fn, log
        self.cap, self.max_wait = cap, max_wait
        self.wall = wall or time.time
        self._sleep = sleep or time.sleep
        self.cutoffs = []
        self.holds = []                           # (now, until) of every hold (log / tests)

    def note_cutoff(self, ut):
        try:
            self.cutoffs.append(float(ut))
        except (TypeError, ValueError):
            pass

    def windows(self, now):
        from . import showrun
        try:
            w = list(self.windows_fn(now))
        except Exception as e:
            self.log("warp guard: live windows unavailable (%r)" % e)
            w = []
        return w + [(c, c + showrun.POST_ORBIT) for c in self.cutoffs]

    def hold(self, target):
        """Wait out the open 1x windows; True if the warp to `target` is still worth starting (> 1 s ahead)."""
        from . import showrun
        try:
            now = float(self.ut_fn())
        except Exception:
            return True
        until = showrun.warp_hold_until(now, self.windows(now))
        if until is None:
            return True
        self.holds.append((now, until))
        self.log("warp guard: 1x window open, warp held %.1f s (game) until UT %.1f (recording v2)" % (until - now, until))
        w0, budget = self.wall(), min(self.cap * (until - now) + 1.0, self.max_wait)
        u = now
        while u < until and self.wall() - w0 < budget:
            self._sleep(0.1)
            try:
                u = float(self.ut_fn())
            except Exception:
                pass
        return target - u > 1.0


def great_circle_distance(body_radius, lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    a = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * body_radius * math.asin(min(1.0, math.sqrt(a)))


from .paths import KSP_DIR  # noqa: E402  (game root: $KSP_DIR or the platform's Steam default, see paths.py)
SHOT_DIR = os.path.join(ROOT, "screenshots")


def screenshot(name, timeout=10, clean=True):
    """Ask the AutoLab plugin for an in-game screenshot (UI hidden if clean); returns the png path or None."""
    os.makedirs(SHOT_DIR, exist_ok=True)
    path = os.path.join(SHOT_DIR, time.strftime("%Y%m%d_%H%M%S_") + name + ".png")
    with open(os.path.join(KSP_DIR, "GameData/AutoLab/shot.req"), "w") as f:
        f.write(("clean:" if clean else "") + path)
    return path if wait_until(lambda: os.path.exists(path) and os.path.getsize(path) > 0, timeout) else None


VIDEO_DIR = os.path.join(ROOT, "videos")
_AL = os.path.join(KSP_DIR, "GameData/AutoLab")


def overlay(text):
    """Telemetry text drawn by AutoLab in the lower-left corner ('' clears)."""
    try:   # atomic replace: the plugin polls the file and must never read half of it
        path = os.path.join(_AL, "overlay.txt")
        with open(path + ".tmp", "w") as f:
            f.write(text)
        os.replace(path + ".tmp", path)
    except OSError:
        pass


def trajectory(text):
    """Predicted track for the AutoLab plugin ("pad lat lon alt" + "lat lon alt" lines; "" clears)."""
    path = os.path.join(_AL, "traj.txt")
    try:
        if text:
            with open(path + ".tmp", "w") as f:
                f.write(text)
            os.replace(path + ".tmp", path)
        elif os.path.exists(path):
            os.remove(path)
    except OSError:
        pass


def camera_shot(text):
    """Director shot for the AutoLab plugin: 'chase' | 'ground lat lon alt [fov]' | 'onboard right up out [fov]'."""
    try:   # atomic replace: a half-written file would read as "" = one frame of the stock camera
        path = os.path.join(_AL, "shot.txt")
        with open(path + ".tmp", "w") as f:
            f.write(text)
        os.replace(path + ".tmp", path)
    except OSError:
        pass


def camera_aim(alpha):
    """Blend factor for the AutoLab presentation camera (0 = level chase view, 1 = look at the pad)."""
    try:
        with open(os.path.join(_AL, "cam.txt"), "w") as f:
            f.write("%.3f" % alpha)
    except OSError:
        pass


def record_start(name, fps=30, v2=False):
    """v2 (HOP_REC=full only, director v2 specification): the plugin also writes frames/index.csv (UT of every frame)
    and compensates physics warp in the capture rate (C1-B1); without it the request is byte-identical to before."""
    frames = os.path.join(ROOT, "logs", "frames_" + name)
    with open(os.path.join(_AL, "rec.req"), "w") as f:
        # our pid: the plugin stops the recording when this process is gone (a killed run never writes rec.stop, and
        # an orphaned recording keeps captureFramerate fixed -> the next flight runs in slow motion, 2026-10-21)
        f.write("%s|%d|%d" % (frames, fps, os.getpid()) + ("|v2" if v2 else ""))
    return frames


def record_halt():
    """Stop a recording without encoding (v2: tools/edit_video.py cuts the frames afterwards)."""
    open(os.path.join(_AL, "rec.stop"), "w").close()
    time.sleep(1.5)


def record_pause(on):
    """Pause (True) / resume (False) frame capture of a running recording (AutoLab rec.pause); while paused the
    game runs at its own pace and no frames are written, so dead air is cut out of the video.
    Recording v3 (REC_OBS set, OBS in real time): a no-op, the editor cuts the dead air (tools/edit_obs.py)."""
    if REC_OBS is not None:
        return
    path = os.path.join(_AL, "rec.pause")
    try:
        if on:
            open(path, "w").close()
        elif os.path.exists(path):
            os.remove(path)
    except OSError:
        pass


def record_stop(frames, name, fps=30, fast=1.0, tail_s=25.0):
    """Stop recording and encode frames to videos/<stamp>_<name>.mp4 (frames deleted).
    fast > 1 speeds up everything except the last tail_s seconds (the landing stays real-time)."""
    import glob
    import shutil
    import subprocess
    open(os.path.join(_AL, "rec.stop"), "w").close()
    time.sleep(1.5)
    os.makedirs(VIDEO_DIR, exist_ok=True)
    out = os.path.join(VIDEO_DIR, time.strftime("%Y%m%d_%H%M%S_") + name + ".mp4")
    n = len(glob.glob(os.path.join(frames, "*.jpg")))
    cut = max(n / fps - tail_s, 0.0)
    src = ["-framerate", str(fps), "-i", os.path.join(frames, "%06d.jpg")]
    enc = ["-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "26", "-movflags", "+faststart", out]
    if fast > 1.0 and cut > 5:
        fc = ("[0:v]scale=1920:-2,split[a][b];[a]trim=0:%.3f,setpts=(PTS-STARTPTS)/%.2f[f];"
              "[b]trim=%.3f,setpts=PTS-STARTPTS[s];[f][s]concat=n=2:v=1[v]") % (cut, fast, cut)
        cmd = ["ffmpeg", "-y", "-loglevel", "error"] + src + ["-filter_complex", fc, "-map", "[v]", "-r", str(fps)] + enc
    else:
        cmd = ["ffmpeg", "-y", "-loglevel", "error"] + src + ["-vf", "scale=1920:-2"] + enc
    r = subprocess.run(cmd)
    if r.returncode == 0:
        shutil.rmtree(frames, ignore_errors=True)
        return out
    return None


def sun_elevation(conn, lat=KSC_PAD[0], lon=KSC_PAD[1]):
    """Current sun elevation (deg) at a surface point of Kerbin."""
    import numpy as np
    sc = conn.space_center
    body = sc.bodies['Kerbin']
    rf = body.reference_frame
    u = np.asarray(body.surface_position(lat, lon, rf))
    u /= np.linalg.norm(u)
    s = np.asarray(sc.bodies['Sun'].position(rf))
    s /= np.linalg.norm(s)
    return math.degrees(math.asin(max(-1.0, min(1.0, float(np.dot(u, s))))))


def set_daylight(conn, target_el=20.0, afternoon=True, log=print):
    """Warp game time until the sun stands at target_el over KSC (afternoon side by default).
    The sun's apparent rotation sign in the body frame is measured, not assumed."""
    import numpy as np
    sc = conn.space_center
    body = sc.bodies['Kerbin']
    rf = body.reference_frame
    w = body.rotational_speed
    lat, lon = KSC_PAD
    u = np.asarray(body.surface_position(lat, lon, rf)); u /= np.linalg.norm(u)

    def sun():
        s = np.asarray(sc.bodies['Sun'].position(rf))
        return s / np.linalg.norm(s)

    def rot(s, a):
        return np.array([s[0] * math.cos(a) - s[2] * math.sin(a), s[1], s[0] * math.sin(a) + s[2] * math.cos(a)])

    s1, t1 = sun(), sc.ut
    time.sleep(3)
    s2, t2 = sun(), sc.ut
    sgn = 1.0 if np.dot(rot(s1, w * (t2 - t1)) - s2, rot(s1, w * (t2 - t1)) - s2) < \
        np.dot(rot(s1, -w * (t2 - t1)) - s2, rot(s1, -w * (t2 - t1)) - s2) else -1.0
    period = 2 * math.pi / w
    for k in range(1, 1441):
        dt = k * period / 1440
        e = math.degrees(math.asin(float(np.dot(u, rot(s2, sgn * w * dt)))))
        e2 = math.degrees(math.asin(float(np.dot(u, rot(s2, sgn * w * (dt + 60))))))
        if abs(e - target_el) < 0.5 and ((e2 < e) == afternoon):
            sc.warp_to(t2 + dt)
            el = sun_elevation(conn)
            log("daylight: sun elevation %.1f deg (target %.0f)" % (el, target_el))
            return el
    return sun_elevation(conn)


def sun_elevation_after(conn, dt, lat=KSC_PAD[0], lon=KSC_PAD[1]):
    """Predicted sun elevation (deg) at a surface point dt seconds from now (planet rotation only)."""
    import numpy as np
    sc = conn.space_center
    body = sc.bodies['Kerbin']
    rf = body.reference_frame
    w = body.rotational_speed
    u = np.asarray(body.surface_position(lat, lon, rf)); u /= np.linalg.norm(u)

    def sun():
        s = np.asarray(sc.bodies['Sun'].position(rf))
        return s / np.linalg.norm(s)

    def rot(s, a):
        return np.array([s[0] * math.cos(a) - s[2] * math.sin(a), s[1], s[0] * math.sin(a) + s[2] * math.cos(a)])

    s1, t1 = sun(), sc.ut
    time.sleep(2)
    s2, t2 = sun(), sc.ut
    d = w * max(t2 - t1, 1e-3)
    sgn = 1.0 if np.linalg.norm(rot(s1, d) - s2) < np.linalg.norm(rot(s1, -d) - s2) else -1.0
    return math.degrees(math.asin(float(np.dot(u, rot(s2, sgn * w * dt)))))


def sun_azimuth(conn, lat=KSC_PAD[0], lon=KSC_PAD[1]):
    """Sun azimuth (deg from north, clockwise) at a surface point now. The local north / east axes are measured
    from surface positions (no assumption about the handedness of the body frame). 4 RPCs, no sleep."""
    import numpy as np
    sc = conn.space_center
    body = sc.bodies['Kerbin']
    rf = body.reference_frame
    p = np.asarray(body.surface_position(lat, lon, rf))
    n = np.asarray(body.surface_position(lat + 0.01, lon, rf)) - p
    e = np.asarray(body.surface_position(lat, lon + 0.01, rf)) - p
    s = np.asarray(sc.bodies['Sun'].position(rf)) - p
    return math.degrees(math.atan2(float(np.dot(s, e)) / np.linalg.norm(e),
                                   float(np.dot(s, n)) / np.linalg.norm(n))) % 360.0


def _enu_basis(body, rf, lat, lon):
    """Local (east, north, up) unit vectors at a surface point, in the body frame's coordinates, measured from
    surface positions (east = increasing longitude, north = increasing latitude; central differences: exactly
    tangent): no handedness assumption. 5 RPCs."""
    import numpy as np
    p = np.asarray(body.surface_position(lat, lon, rf))
    up = p / np.linalg.norm(p)
    e = np.asarray(body.surface_position(lat, lon + 0.01, rf)) - np.asarray(body.surface_position(lat, lon - 0.01, rf))
    n = np.asarray(body.surface_position(lat + 0.01, lon, rf)) - np.asarray(body.surface_position(lat - 0.01, lon, rf))
    e = e - np.dot(e, up) * up
    n = n - np.dot(n, up) * up
    return e / np.linalg.norm(e), n / np.linalg.norm(n), up


def sun_enu(conn, lat=KSC_PAD[0], lon=KSC_PAD[1]):
    """Sun direction (unit; east / north / up components) at a surface point now (recording v2 fix round 3: the
    lander's sun-aware rel cameras). ~7 RPCs, no sleep."""
    import numpy as np
    sc = conn.space_center
    body = sc.bodies['Kerbin']
    rf = body.reference_frame
    e, n, u = _enu_basis(body, rf, lat, lon)
    s = np.asarray(sc.bodies['Sun'].position(rf)) - np.asarray(body.surface_position(lat, lon, rf))
    s = s / np.linalg.norm(s)
    return (float(np.dot(s, e)), float(np.dot(s, n)), float(np.dot(s, u)))


def sun_geometry(sc, v, lat, lon):
    """Rel-camera geometry of vessel v for kspilot.showrun.sun_safe_rel (recording v2 fix round 3; called on the
    director's own connection): surface / orbital velocity and the sun direction as local east / north / up
    components (a right-handed frame built from surface positions). ~12 RPCs, no sleep, commands nothing."""
    import numpy as np
    body = v.orbit.body
    rf = body.reference_frame
    p = np.asarray(v.position(rf))
    vs = np.asarray(v.velocity(rf))
    sun = np.asarray(sc.bodies['Sun'].position(rf)) - p
    e, n, _ = _enu_basis(body, rf, lat, lon)
    up = p / np.linalg.norm(p)
    e, n = e - np.dot(e, up) * up, n - np.dot(n, up) * up       # the vessel's own horizon
    e, n = e / np.linalg.norm(e), n / np.linalg.norm(n)
    R = body.equatorial_radius
    # orbital velocity = surface velocity + the planet's rotation (eastward; omega x distance from the spin axis, the
    # body frame's y axis being the north pole - kRPC CelestialBody.reference_frame)
    vo = vs + body.rotational_speed * math.hypot(float(p[0]), float(p[2])) * e
    enu = lambda x: (float(np.dot(x, e)), float(np.dot(x, n)), float(np.dot(x, up)))   # noqa: E731
    return dict(up=(0.0, 0.0, 1.0), vsrf=enu(vs), vobt=enu(vo), sun=enu(sun / np.linalg.norm(sun)),
                alt=float(np.linalg.norm(p)) - R, R=R, north=(0.0, 1.0, 0.0))


def set_camera(conn, distance=70.0, pitch=5.0, heading=None):
    """Presentation camera: chase/automatic view at a fixed distance and a low pitch."""
    try:
        cam = conn.space_center.camera
        try:
            cam.mode = conn.space_center.CameraMode.free     # horizon stays level through flips
        except Exception:
            pass
        cam.distance = max(cam.min_distance, min(distance, cam.max_distance))
        cam.pitch = max(cam.min_pitch, min(pitch, cam.max_pitch))
        if heading is not None:
            cam.heading = heading
    except Exception as e:
        print("camera:", e)
