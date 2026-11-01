"""Recording pipeline v3 runtime (decision record DR-011): the whole-mission video is recorded in REAL TIME by
OBS (tools/obs_rec.py: KSP window + KSP app audio, 1080p30); the in-game director cameras / HUD are unchanged; nothing
is paused, nothing is captured by the plugin (no captureFramerate, no physics-warp capture compensation). The cut is
made afterwards by tools/edit_obs.py from three sidecars next to the OBS file in videos/obs/:

  <base>.timemap.csv  wall, UT, warp rate, rails factor, physics factor at TIMEMAP_HZ (10 Hz) from a kRPC stream on
                      the logger's OWN connection, in its own daemon thread (never in the control loop)
  <base>.clock.csv    t_send, t_recv, duration: obs-websocket GetRecordStatus samples (video clock -> wall clock,
                      kspilot/timemap.calibrate), a burst at the start, then every CLOCK_PERIOD s, a burst at the end
  <base>.rec.json     paths, OBS start / stop info and frame statistics (rewritten at the stop)
  <base>.events.json  the flight's HUD events / burns (written by the flight script after the stop)

Engine switch (HOP_REC=full only): KSP_REC_ENGINE=obs (default, v3) | frames (v2 plugin frame capture, fallback).
While a v3 recording runs core.REC_OBS is set: core.record_pause() is a no-op (record continuously, cut in post).
A killed flight cannot stop OBS (SIGKILL skips atexit): MARKER names the running recording and tools/watchdog.py stops
it. Without HOP_REC=full nothing here is imported.
"""
import atexit
import importlib.util
import json
import os
import re
import threading
import time

from kspilot import core, timemap

TIMEMAP_HZ = 10.0         # rows per second (requirement: >= 4 Hz)
UT_STREAM_HZ = 50.0       # the UT stream itself (one physics tick = 0.02 s)
CLOCK_PERIOD = 2.0        # s between OBS clock samples during the recording
CLOCK_BURST = 12          # samples at the start and at the end (0.1 s apart)
REC_DIR = os.path.join(core.ROOT, "videos", "obs")
MARKER = os.path.join(core.LOG_DIR, "obs_recording.json")


def engine():
    """'obs' (v3, default) or 'frames' (v2 plugin capture: KSP_REC_ENGINE=frames)."""
    return "frames" if os.environ.get("KSP_REC_ENGINE", "obs").strip().lower() == "frames" else "obs"


def obs_tool():
    """tools/obs_rec.py as a module (OBS only: never touches the game)."""
    spec = importlib.util.spec_from_file_location("obs_rec", os.path.join(core.ROOT, "tools", "obs_rec.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class TimeMapLogger:
    """Daemon thread: (wall, UT, warp rate, rails factor, physics factor) rows at `hz` to `path`. The UT comes from a
    stream callback (wall = arrival of the update, ~1 ms after the game tick on localhost); while the UT does not
    change (game paused, kRPC sends no update) the row carries the present wall time and the same UT. The other
    fields are streamed at `hz`. Errors never reach the flight: the thread reconnects after a failure."""

    def __init__(self, path, connect=None, hz=TIMEMAP_HZ, log=print):
        self.path, self.hz, self.log = path, hz, log
        self.connect = connect or (lambda: core.connect("timemap"))
        self._stop = threading.Event()
        self.th = None
        self.rows = 0
        self.first = threading.Event()

    def start(self):
        self.th = threading.Thread(target=self._run, name="timemap", daemon=True)
        self.th.start()
        return self

    def stop(self):
        self._stop.set()
        if self.th is not None:
            self.th.join(timeout=3.0)

    def _session(self, f):
        conn = self.connect()
        try:
            sc = conn.space_center
            last = [None]

            def cb(v):
                last[0] = (time.time(), float(v))
            s_ut = conn.add_stream(getattr, sc, "ut")
            try:
                s_ut.rate = UT_STREAM_HZ
            except Exception:
                pass
            s_ut.add_callback(cb)
            st = []
            for name in ("warp_rate", "rails_warp_factor", "physics_warp_factor"):
                s = conn.add_stream(getattr, sc, name)
                try:
                    s.rate = self.hz
                except Exception:
                    pass
                st.append(s)
            prev = None
            period = 1.0 / self.hz
            t_flush = time.time()
            while not self._stop.is_set():
                x = last[0]
                cur = float(s_ut())
                if x is None or abs(cur - x[1]) > 1e-9:  # no callback (yet) for the cached value: it holds now
                    x = (time.time(), cur)
                w, u = x
                if prev is not None and w <= prev[0] + 1e-6:
                    w = time.time()                 # no new UT update: the same UT holds now (paused game)
                row = (w, u, float(st[0]()), int(st[1]()), int(st[2]()))
                if prev is None or row[0] > prev[0] + 1e-6:
                    f.write(timemap.format_row(*row))
                    self.rows += 1
                    prev = row
                    self.first.set()
                if time.time() - t_flush > 1.0:
                    f.flush()
                    t_flush = time.time()
                self._stop.wait(period)
        finally:
            try:
                conn.close()
            except Exception:
                pass

    def _run(self):
        new = not os.path.exists(self.path)
        with open(self.path, "a") as f:
            if new:
                timemap.write_header(f)
            err = 0
            while not self._stop.is_set():
                try:
                    self._session(f)
                except Exception as e:              # scene change / kRPC hiccup: reconnect, never raise
                    err += 1
                    if err <= 3:
                        self.log("timemap: %r (reconnecting)" % e)
                    f.flush()
                    self._stop.wait(1.0)
            f.flush()


class Recording:
    """One v3 recording. start() = start_timemap() + start_obs(); a checkpoint flight may start OBS first (before the
    save loads: OBS can take seconds to start, the falling stage must not wait for it) and the time map after."""

    def __init__(self, name, log=print, obs=None, connect=None):
        self.name, self.log = name, log
        self.safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("_") or "rec"
        os.makedirs(REC_DIR, exist_ok=True)
        self.base = os.path.join(REC_DIR, "%s_%s" % (self.safe, time.strftime("%Y%m%d-%H%M%S")))
        self.timemap_path = self.base + ".timemap.csv"
        self.clock_path = self.base + ".clock.csv"
        self.meta_path = self.base + ".rec.json"
        self.events_path = self.base + ".events.json"
        self._obs_mod = obs
        self._connect = connect
        self.c = None
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self.tm = None
        self.sampler = None
        self.meta = dict(engine="obs", name=name, base=self.base, timemap=self.timemap_path, clock=self.clock_path,
                         events=self.events_path, video=None, fps=timemap.FPS)
        self.active = False

    # ---- start ----
    def start(self):
        self.start_timemap()
        return self.start_obs()

    def start_timemap(self, wait=5.0):
        self.tm = TimeMapLogger(self.timemap_path, connect=self._connect, log=self.log).start()
        if not self.tm.first.wait(wait):
            self.log("obsrec: WARNING no time-map row after %.0f s (the thread keeps trying)" % wait)
        else:
            self.log("obsrec: time map -> %s" % self.timemap_path)
        return self

    def _sample(self, smp):
        with open(self.clock_path, "a") as f:
            f.write("%.4f,%.4f,%s\n" % (smp[0], smp[1], "" if smp[2] is None else "%.4f" % smp[2]))

    def start_obs(self):
        mod = self._obs_mod or obs_tool()
        self._obs_mod = mod
        with open(self.clock_path, "w") as f:
            f.write("t_send,t_recv,duration\n")
        c, info = mod.start_recording(self.safe, on_sample=self._sample)
        self.c = c
        self.meta["start"] = info
        self.active = True
        core.REC_OBS = self
        try:
            with open(MARKER, "w") as f:
                json.dump(dict(base=self.base, pid=os.getpid(), t=time.time()), f)
        except OSError:
            pass
        atexit.register(self.abort)
        self._write_meta()
        self.sampler = threading.Thread(target=self._sample_loop, name="obsclock", daemon=True)
        self.sampler.start()
        self.log("obsrec: OBS recording (v3, real time) %s" % self.base)
        return self

    def _status(self):
        with self.lock:
            return self._obs_mod.status_sample(self.c)

    def _burst(self, n=CLOCK_BURST, gap=0.1):
        for _ in range(n):
            try:
                s = self._status()
                if s[3]:
                    self._sample(s[:3])
            except Exception:
                return
            time.sleep(gap)

    def _sample_loop(self):
        self._burst()
        while not self._stop.wait(CLOCK_PERIOD):
            try:
                s = self._status()
                if s[3]:
                    self._sample(s[:3])
            except Exception:
                pass

    def _write_meta(self):
        try:
            with open(self.meta_path + ".tmp", "w") as f:
                json.dump(self.meta, f, indent=1)
            os.replace(self.meta_path + ".tmp", self.meta_path)
        except OSError:
            pass

    # ---- stop ----
    def stop(self):
        """Stop OBS (after a last clock burst) and the time map; returns the video path (None if OBS failed)."""
        if not self.active:
            return self.meta.get("video")
        self.active = False
        self._stop.set()
        if self.sampler is not None:
            self.sampler.join(timeout=CLOCK_PERIOD + 6.0)
        self._burst()
        path = None
        try:
            with self.lock:
                path, info = self._obs_mod.stop_recording(self.c)
            self.meta["stop"] = info
        except Exception as e:
            self.log("obsrec: OBS stop failed: %r" % e)
            self.meta["stop_error"] = repr(e)
        if self.tm is not None:
            self.tm.stop()
            self.meta["timemap_rows"] = self.tm.rows
        self.meta["video"] = path
        self._write_meta()
        try:
            os.remove(MARKER)
        except OSError:
            pass
        if core.REC_OBS is self:
            core.REC_OBS = None
        self.log("obsrec: stopped -> %s (time map %s rows)" % (path, self.meta.get("timemap_rows")))
        return path

    def abort(self):
        """atexit: the flight script ended while recording (abort / exception): stop OBS, keep the sidecars."""
        if self.active:
            self.log("obsrec: flight ended while recording: stopping OBS")
            self.stop()


def stop_orphan(log=print):
    """tools/watchdog.py after killing a flight: stop the OBS recording that flight left running (MARKER), so it
    does not record for hours; the sidecars stay for a manual cut. Talks to OBS only."""
    if not os.path.exists(MARKER):
        return None
    try:
        meta = json.load(open(MARKER))
        try:
            os.kill(int(meta.get("pid", 0)), 0)          # the flight is still alive: not an orphan
            if int(meta.get("pid", 0)) > 0:
                return None
        except (OSError, ValueError):
            pass
        mod = obs_tool()
        c = mod.connect(launch=False)
        path = None
        if c.get_record_status().output_active:
            path, _ = mod.stop_recording(c)
            log("obsrec: stopped the orphaned OBS recording -> %s" % path)
        mp = meta.get("base", "") + ".rec.json"
        if path and os.path.exists(mp):
            m = json.load(open(mp))
            m["video"], m["orphan_stop"] = path, time.time()
            json.dump(m, open(mp, "w"), indent=1)
        os.remove(MARKER)
        return path
    except Exception as e:
        log("obsrec: orphan stop failed: %r" % e)
        return None
