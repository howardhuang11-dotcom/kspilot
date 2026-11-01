"""Whole-mission HUD feed for the AutoLab plugin (HUD v8;
show timeline: kspilot/showrun.py).

One background thread with its OWN read-only kRPC connection writes GameData/AutoLab/overlay.txt four times a
second from lift-off to touchdown:
  v=8        layout version (the plugin falls back to the v7 drawing without it)
  t0 / ut    lift-off UT and the UT of this write (absolute, %.3f: the plugin parses them as doubles); t0_src.
             The plugin computes the T+ clock from t0 and the game's own Planetarium UT every frame (smooth,
             monotonic, correct under time warp); "T+ --:--:--" while t0 is unknown. Checkpoint runs take the
             lift-off UT stored with the save (logs/mission_t0_<craft>.json, checkpoint()/resume()), else the
             vessel's own MET. clock= is the same string for older plugins
  warp       game warp rate -> "WARP xN" tag next to the clock (the plugin prefers TimeWarp.CurrentRate)
  events     NAME:dt|...  legacy relative form of the events (v7 plugins)
  evx        NAME;ut;planned_ut;done;level;dev|...  every mission event in absolute UT (the plugin moves the nodes
             with its own clock), the PLANNED time (= first reported prediction) and the deviation label
             (level 0 none, 1 amber, 2 red, 3 past = actual vs planned in the past style), see deviation()
  left/right three readout rows per table, chosen by phase: short labels (ALT VS SPD THR PAD FUEL AP PE INC Q
             STG NEXT), drawn left-aligned with thin separators
  hud_tx     NAME@ut@dur|...  the firm HUD widget transitions of the unified show timeline (kspilot/showrun.py):
             the plugin animates exactly at these UTs; the director reads the same list (transitions())
  asc        alt_km,downrange_km,v_horizontal,v_vertical,ap_km,pe_km  (ascent widget); target = target Ap km
  orb        u_deg,node_deg,period_s  vessel / deorbit-node angle in the orbit plane (minimal orbit icon)
  + whatever the flight code adds with set() (the lander: pos, pred, dist, thr, t; the director: cam, camtel,
    camuntil)
  end=1      only from stop(collapse=True) (aborted runs): after a touchdown the HUD holds (no collapse)
The flight code reports with event() (it happened now / at ut), plan() (predicted ut), mark() (show timeline
marks: S8 cut, HANDOFF) and phase(). MAX-Q is detected here from the dynamic pressure. Never sends commands.
"""
import json
import math
import os
import threading
import time

ORDER = ["LIFTOFF", "MAX-Q", "SRB SEP", "MECO", "ORBIT", "PAYLOAD SEP", "DEORBIT BURN",
         "ENTRY BURN", "AERO GLIDE", "LANDING BURN", "TOUCHDOWN"]
current = None      # the running feed (flight code calls hud.current.event(...) when it exists)


def _core():
    from . import core      # lazy: the deviation / format helpers stay importable without krpc
    return core


def deviation(pred, planned, lead, done):
    """Deviation label of one event: (seconds, text, level). planned = first reported prediction (or nominal plan),
    lead = how far ahead that plan was made. Shown when |pred - planned| > max(3 s, 5 % of lead); amber, red above
    max(10 s, 10 % of lead); an event that happened shows actual vs planned in the past style (level 3)."""
    if planned is None:
        return 0.0, "", 0
    d = pred - planned
    if abs(d) <= max(3.0, 0.05 * max(lead, 0.0)):
        return d, "", 0
    lvl = 3 if done else (2 if abs(d) > max(10.0, 0.10 * max(lead, 0.0)) else 1)
    return d, dev_text(d), lvl


def dev_text(d):
    n = int(round(abs(d)))
    sign = "+" if d >= 0 else "−"
    return sign + ("%dm%02ds" % (n // 60, n % 60) if n >= 60 else "%ds" % n)


def report(fn, *a, **k):
    """Call a method of the running feed if there is one (flight code never fails because of the HUD)."""
    if current is not None:
        try:
            getattr(current, fn)(*a, **k)
        except Exception:
            pass


class HudFeed:
    def __init__(self, mission="ORBITAL RETURN", log=print, craft=None, target_km=None):
        self.mission, self.log, self.craft, self.target_km = mission, log, craft, target_km
        self.ev = {}                # name -> [ut, done]
        self.planned = {}           # name -> (first predicted ut, ut when it was predicted)
        self.marks = {}             # show timeline marks (kspilot/showrun.py): S8, HANDOFF -> ut
        self.extra = {}             # key -> value string (from the flight code)
        self.phase_name = "ascent"
        self.t0 = None              # lift-off UT
        self.t0_src = ""
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._q = (0.0, None)       # (max q, ut) during the ascent
        self.th = None
        # recording pipeline v2 (HOP_REC=full only, director v2 specification): adds "rec=full" (the plugin hides its
        # game-warp tag, the editor draws the playback-speed badge) and "cd=<UT>" (planned lift-off: T- countdown)
        self.v2 = False
        self.cd = None

    # ---- reporting API (thread-safe) ----
    def event(self, name, ut=None):
        with self.lock:
            u = ut if ut is not None else getattr(self, "_ut", None)
            if u is None:
                return
            self.ev[name] = [u, True]
            if name == "LIFTOFF":
                self.t0, self.t0_src = u, "liftoff"
        if name == "LIFTOFF":
            self._t0_store(None)

    def plan(self, name, ut):
        with self.lock:
            if not (name in self.ev and self.ev[name][1]):
                self.ev[name] = [ut, False]
                if name not in self.planned:
                    self.planned[name] = (ut, getattr(self, "_ut", None) or ut)

    def unplan(self, name):
        with self.lock:
            if name in self.ev and not self.ev[name][1]:
                del self.ev[name]

    def mark(self, name, ut):
        """Show timeline mark (S8 cut, HANDOFF): the first report wins (the plugin animates on it)."""
        with self.lock:
            self.marks.setdefault(name, float(ut))

    def phase(self, name):
        self.phase_name = name

    def set(self, **kv):
        with self.lock:
            for k, v in kv.items():
                if v is None:
                    self.extra.pop(k, None)
                else:
                    self.extra[k.replace("__", ".")] = str(v)

    def transitions(self):
        """Planned HUD widget transitions of the unified show timeline (firm and predicted), kspilot/showrun.py."""
        from . import showrun
        with self.lock:
            ev = {k: (u, d) for k, (u, d) in self.ev.items()}
            marks = dict(self.marks)
        return showrun.transitions(ev, marks, v2=self.v2)

    # ---- lift-off UT with the checkpoints (the T+ clock of a checkpoint run continues the mission's) ----
    def _t0_file(self):
        return os.path.join(_core().LOG_DIR, "mission_t0_%s.json" % (self.craft or "default").replace(" ", ""))

    def _t0_store(self, save):
        if self.t0 is None:
            return
        try:
            path = self._t0_file()
            d = json.load(open(path)) if os.path.exists(path) else {}
            d["t0"] = self.t0
            if save:
                d.setdefault("saves", {})[save] = self.t0
            json.dump(d, open(path, "w"))
        except Exception as e:
            self.log("hud: lift-off UT not stored (%r)" % e)

    def checkpoint(self, save):
        """Call right after sc.save(save): the lift-off UT is stored with that save."""
        self._t0_store(save)

    def resume(self, save):
        """Call right after loading save: T+ continues from the lift-off UT stored with it (else the MET)."""
        try:
            t0 = json.load(open(self._t0_file())).get("saves", {}).get(save)
        except Exception:
            t0 = None
        with self.lock:
            self.t0, self.t0_src = (t0, "save:" + save) if t0 is not None else (None, "")
        self._met_tried = t0 is not None
        self.__dict__.pop("_met_after", None)

    # ---- lifecycle ----
    def start(self):
        global current
        current = self
        import atexit
        atexit.register(self._atexit)            # an aborted mission must not leave a frozen HUD on screen
        self.th = threading.Thread(target=self._run, name="hud", daemon=True)
        self.th.start()
        return self

    def _atexit(self):
        if getattr(self, "_held", False):
            self.clear()
        elif not getattr(self, "_stopped", False):
            self.stop(collapse=False)

    def stop(self, collapse=True, hold=False):
        """hold=True (after a touchdown): the last frame stays on screen, no collapse (clear() once the video is
        stopped); collapse=True: collapse animation (plugin), then the overlay is cleared; neither: cleared at once."""
        global current
        if getattr(self, "_stopped", False):
            return
        self._stopped = True
        self._stop.set()
        if self.th:
            self.th.join(timeout=3)
        if hold:
            self._held = True
            try:
                _core().overlay(self._compose(getattr(self, "_tel", None)))
            except Exception:
                pass
            return
        if collapse:
            try:
                _core().overlay(self._compose(getattr(self, "_tel", None)) + "\nend=1")
            except Exception:
                pass
            time.sleep(2.0)
        _core().overlay("")
        if current is self:
            current = None

    def clear(self):
        """End of a held HUD (call after the recording has stopped)."""
        global current
        self._held = False
        try:
            _core().overlay("")
        except Exception:
            pass
        if current is self:
            current = None

    # ---- the loop ----
    def _run(self):
        try:
            conn = _core().connect("hud")
        except Exception as e:
            self.log("hud: no connection (%r)" % e)
            return
        sc = conn.space_center
        ut_s = conn.add_stream(getattr, sc, "ut")
        tel, k = {}, 0
        while not self._stop.is_set():
            try:
                self._ut = ut_s()
                if k % 2 == 0:                      # telemetry at 2 Hz keeps the RPC load small
                    tel = self._telemetry(sc)
                    self._tel = tel
                k += 1
                self._t0_fallback(sc)
                self._detect_maxq(tel)
                _core().overlay(self._compose(tel))
            except Exception as e:                  # never let the HUD take the flight down
                if not getattr(self, "_err", False):
                    self._err = True
                    self.log("hud error: %r" % e)
            time.sleep(0.25)
        try:
            conn.close()
        except Exception:
            pass

    def _t0_fallback(self, sc):
        """No lift-off in this run and none stored with the save: the vessel's own mission elapsed time."""
        if self.t0 is not None or self.phase_name == "ascent" or getattr(self, "_met_tried", False):
            return
        if "_met_after" not in self.__dict__:
            self._met_after = time.time() + 15.0          # leave time for resume() after a checkpoint load
        if time.time() < self._met_after:
            return
        self._met_tried = True
        try:
            met = sc.active_vessel.met
            if met > 1.0:
                with self.lock:
                    if self.t0 is None:
                        self.t0, self.t0_src = self._ut - met, "met"
        except Exception:
            pass

    def _telemetry(self, sc):
        v = sc.active_vessel
        body = v.orbit.body
        fl = v.flight(body.reference_frame)
        t = dict(alt=fl.mean_altitude, speed=fl.speed, vs=fl.vertical_speed, q=v.flight().dynamic_pressure,
                 thr=v.control.throttle if v.thrust > 0 else 0.0, ap=v.orbit.apoapsis_altitude,
                 pe=v.orbit.periapsis_altitude, inc=math.degrees(v.orbit.inclination), stage=v.control.current_stage)
        try:
            t["warp"] = sc.warp_rate
        except Exception:
            pass
        if self.phase_name == "ascent":
            try:
                c = _core()
                t["x"] = c.great_circle_distance(body.equatorial_radius, c.KSC_PAD[0], c.KSC_PAD[1],
                                                 fl.latitude, fl.longitude)
            except Exception:
                pass
        elif "pos" not in self.extra:               # orbit icon until the lander reports its own position
            try:
                o = v.orbit
                t["u"] = math.degrees(o.argument_of_periapsis + o.true_anomaly) % 360.0
                t["period"] = o.period
            except Exception:
                pass
        if time.time() - getattr(self, "_slow_t", 0.0) > 2.0:     # part-level reads only every 2 s (RPC load)
            self._slow_t = time.time()
            try:
                res = v.resources
                lf, lfm = res.amount("LiquidFuel"), res.max("LiquidFuel")
                self._slow = dict(fuel=lf / lfm if lfm > 0 else float("nan"),
                                  srb=sum(1 for e in v.parts.engines
                                          if e.active and e.has_fuel and "SolidFuel" in e.propellant_names))
            except Exception:
                self._slow = dict(fuel=float("nan"), srb=0)
        t.update(getattr(self, "_slow", {}))
        return t

    def _detect_maxq(self, tel):
        if self.t0 is None or "q" not in tel or ("MAX-Q" in self.ev and self.ev["MAX-Q"][1]):
            return
        q = tel["q"]
        if q > self._q[0]:
            self._q = (q, self._ut)
        elif self._q[0] > 2000 and q < 0.9 * self._q[0]:
            self.event("MAX-Q", self._q[1])
            self.log("hud: MAX-Q %.1f kPa at T+%.0f s" % (self._q[0] / 1e3, self._q[1] - self.t0))

    def _rows(self, tel):
        """Readout tables (short labels, design v8)."""
        g = lambda key, d=float("nan"): tel.get(key, d)   # noqa: E731
        f0 = lambda x: "—" if x != x else "%.0f" % x       # noqa: E731
        sg = lambda x: "—" if x != x else ("−" if x < -0.5 else "") + "%.0f" % abs(x)   # noqa: E731
        km = lambda m: "—" if m != m else ("%.1f" % (m / 1e3) if abs(m) < 1e5 else "%.0f" % (m / 1e3))   # noqa: E731
        if self.phase_name == "ascent":
            srb = g("srb", 0)
            left = [("ALT", km(g("alt")), "KM"), ("SPD", f0(g("speed")), "M/S"), ("AP", km(g("ap")), "KM")]
            right = [("THR", f0(100 * g("thr", 0)), "%"), ("Q", "%.1f" % (g("q", 0) / 1e3), "KPA"),
                     ("STG", "1", ("+%d SRB" % srb) if srb else "")]   # the core is stage 1 (KSP's index counts down)
        elif self.phase_name == "orbit":
            with self.lock:
                evs = list(self.ev.items())
            nb = [(n, u) for n, (u, done) in evs if not done and u > getattr(self, "_ut", 0.0)]
            nb = min(nb, key=lambda x: x[1]) if nb else None
            left = [("AP", km(g("ap")), "KM"), ("PE", km(g("pe")), "KM"), ("INC", "%.2f" % g("inc"), "DEG")]
            right = [("THR", f0(100 * g("thr", 0)), "%"), ("SPD", f0(g("speed")), "M/S"),
                     ("NEXT", _dur(nb[1] - self._ut) if nb else "—", "")]
        else:
            h = g("alt") - core_pad_alt()
            try:                                         # the lander's own height above the pad when it runs
                h = float(self.extra["pos"].split(",")[2])
            except (KeyError, IndexError, ValueError):
                pass
            left = [("ALT", km(h) if h >= 1000 else f0(h), "KM" if h >= 1000 else "M"),
                    ("VS", sg(g("vs")), "M/S"), ("SPD", f0(g("speed")), "M/S")]
            try:
                dd = float(self.extra.get("dist", "nan"))
            except ValueError:
                dd = float("nan")
            dist = ("—", "M") if dd != dd else (("%.1f" % (dd / 1e3), "KM") if dd >= 1000 else ("%.1f" % dd, "M"))
            # "PAD" = distance to the landing target; an offset-aim run (kspilot/aim.py) labels it "TGT"
            right = [("THR", f0(100 * g("thr", 0)), "%"), (getattr(self, "dist_label", "PAD"),) + dist,
                     ("FUEL", f0(100 * g("fuel")), "%")]
        return left, right

    def _compose(self, tel=None):
        tel = tel or {}
        ut = getattr(self, "_ut", 0.0)
        with self.lock:
            # a prediction that never happened (no SRBs, no entry burn...) must not age into a "past" event
            names = [n for n in ORDER if n in self.ev and (self.ev[n][1] or self.ev[n][0] > ut - 5.0)]
            evs = "|".join("%s:%.1f" % (n, self.ev[n][0] - ut) for n in names)
            evx = []
            for n in names:
                u, done = self.ev[n]
                pl = self.planned.get(n)
                _, txt, lvl = deviation(u, pl[0], pl[0] - pl[1], done) if pl else (0.0, "", 0)
                evx.append("%s;%.3f;%s;%d;%d;%s" % (n, u, "%.3f" % pl[0] if pl else "", int(done), lvl, txt))
            node = self.ev.get("DEORBIT BURN")
            extra = dict(self.extra)
            t0, t0_src = self.t0, self.t0_src
        if "t" in extra:                               # plugin parses floats: send game time relative to a base
            try:
                tb = self.__dict__.setdefault("_t_base", float(extra["t"]))
                extra["t"] = "%.2f" % (float(extra["t"]) - tb)
            except ValueError:
                pass
        if t0 is not None:
            m = max(ut - t0, 0.0)
            clock = "T+ %02d:%02d:%02d" % (int(m) // 3600, int(m) // 60 % 60, int(m) % 60)
            met = "%.1f" % m
        else:
            clock, met = "T+ --:--:--", ""
        left, right = self._rows(tel) if tel else ([], [])
        tx = "|".join("%s@%.3f@%.2f" % (x.name, x.ut, x.dur) for x in self.transitions() if x.firm)
        lines = ["HUD", "v=8", "mission=" + self.mission, "t0=" + ("%.3f" % t0 if t0 is not None else ""),
                 "t0_src=" + t0_src, "ut=%.3f" % ut, "warp=%g" % tel.get("warp", 1.0), "clock=" + clock,
                 "met=" + met, "events=" + evs, "evx=" + "|".join(evx),
                 "left=" + "|".join(";".join(r) for r in left), "right=" + "|".join(";".join(r) for r in right),
                 "hud_tx=" + tx]
        if self.target_km:
            lines.append("target=%.0f" % self.target_km)
        if self.v2:
            lines.append("rec=full")
            if self.cd is not None:
                lines.append("cd=%.3f" % self.cd)
            if "thr" in tel:                   # fix round 3: the NEXT label reads "NOW" during a burn (HudV2)
                lines.append("thr_now=%.2f" % tel["thr"])
        if self.phase_name == "ascent" and "alt" in tel:
            vs, sp = tel.get("vs", 0.0), tel.get("speed", 0.0)
            lines.append("asc=%.3f,%.3f,%.0f,%.0f,%.2f,%.1f" % (
                tel["alt"] / 1e3, tel.get("x", 0.0) / 1e3, math.sqrt(max(sp * sp - vs * vs, 0.0)), vs,
                tel.get("ap", 0.0) / 1e3, tel.get("pe", 0.0) / 1e3))
        if "u" in tel:
            nd = ""
            if node and not node[1] and tel.get("period", 0) > 0:
                nd = "%.1f" % ((tel["u"] + 360.0 * (node[0] - ut) / tel["period"]) % 360.0)
            lines.append("orb=%.2f,%s,%.1f" % (tel["u"], nd, tel.get("period", 0.0)))
        lines += ["%s=%s" % kv for kv in extra.items()]
        return "\n".join(lines)


def _dur(s):
    s = max(int(s), 0)
    return "%dH %02dM" % (s // 3600, s // 60 % 60) if s >= 3600 else "%d:%02d" % (s // 60, s % 60)


def core_pad_alt():
    from . import powered_descent as pdg
    return pdg.PAD_ALT
