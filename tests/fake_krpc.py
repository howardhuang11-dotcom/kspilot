"""Counting fake of the kRPC client for offline RPC-budget checks of the lander loop (DR-010 item 3).

Model (decision log 2026-10-21 14:55): while recording, kRPC serves ~1 RPC per game frame, so a control tick lasts
(RPCs in the tick) x (game seconds per frame). Every property get / set and every remote method call on a Remote
counts one RPC on the calling thread; a stream (conn.add_stream) reads the value for free, as the real stream
server pushes it every frame. Game time advances FRAME_S per RPC of any thread (the UT stream reads it).
A missing attribute raises AttributeError and is recorded (the lander swallows many exceptions: a test must assert
`missing` is empty, or the count would silently skip the code behind it)."""
import math
import threading

import numpy as np

FRAME_S = 1.0 / 30.0       # game s per frame while recording (flight #4: 38 frames at fps 12 ~ 1.2 game s)
R_BODY = 600000.0


class Counter:
    def __init__(self):
        self.lock = threading.Lock()
        self.by_thread, self.names, self.total, self.missing, self.seq = {}, {}, 0, [], []
        self.main = threading.get_ident()

    def hit(self, name, val=None):
        with self.lock:
            tid = threading.get_ident()
            self.by_thread[tid] = self.by_thread.get(tid, 0) + 1
            self.total += 1
            if tid == self.main:
                self.names[name] = self.names.get(name, 0) + 1
                self.seq.append(name if val is None else "%s%r" % (name, val))   # main-thread RPC order (+ writes)

    def main_count(self):
        return self.by_thread.get(self.main, 0)


CTR = Counter()


class Prop:
    """A dynamic property value (evaluated on every read, RPC or stream)."""

    def __init__(self, fn):
        self.fn = fn


class Method:
    def __init__(self, fn):
        self.fn = fn


class _Bound:
    def __init__(self, owner, name, fn):
        self.owner, self.name, self.fn = owner, name, fn

    def __call__(self, *a, **k):
        CTR.hit("%s.%s()" % (self.owner, self.name), tuple(x for x in a if isinstance(x, (str, float, int))) or None)
        return self.fn(*a, **k)


class Remote:
    def __init__(self, _label, **attrs):
        object.__setattr__(self, "_n", _label)
        object.__setattr__(self, "_a", dict(attrs))

    def __getattr__(self, k):
        if k.startswith("__"):
            raise AttributeError(k)
        a = object.__getattribute__(self, "_a")
        n = object.__getattribute__(self, "_n")
        if k not in a:
            CTR.missing.append("%s.%s" % (n, k))
            raise AttributeError("%s.%s" % (n, k))
        val = a[k]
        if isinstance(val, Method):
            return _Bound(n, k, val.fn)          # fetching a method is local; the call is the RPC
        CTR.hit("%s.%s" % (n, k))
        return val.fn() if isinstance(val, Prop) else val

    def __setattr__(self, k, v):
        CTR.hit("%s.%s=" % (object.__getattribute__(self, "_n"), k), _rnd(v))
        object.__getattribute__(self, "_a")[k] = v

    def _peek(self, k):
        val = object.__getattribute__(self, "_a")[k]
        return val.fn() if isinstance(val, Prop) else val

    def __repr__(self):
        return "<Remote %s>" % object.__getattribute__(self, "_n")


def _rnd(v):
    """Written values, rounded for the sequence log (tuples / floats to 9 digits)."""
    if isinstance(v, float):
        return round(v, 9)
    if isinstance(v, (tuple, list)):
        return tuple(_rnd(x) for x in v)
    return v if isinstance(v, (bool, int, str, type(None))) else type(v).__name__


class Stream:
    def __init__(self, fn):
        self.fn = fn

    def __call__(self):
        return self.fn()

    def remove(self):
        pass


class FakeConn:
    def __init__(self, space_center):
        self.space_center = space_center
        self.krpc = Remote("krpc", paused=False)
        self.streams = 0

    def add_stream(self, fn, *args):
        self.streams += 1
        if fn is getattr:
            obj, name = args
            return Stream(lambda: obj._peek(name))
        if isinstance(fn, _Bound):
            return Stream(lambda: fn.fn(*args))
        raise TypeError("add_stream(%r)" % (fn,))

    def close(self):
        pass


def surface_point(lat, lon, alt=0.0):
    la, lo = math.radians(lat), math.radians(lon)
    r = R_BODY + alt
    return (r * math.cos(la) * math.cos(lo), r * math.sin(la), r * math.cos(la) * math.sin(lo))


ISP, F_VAC = 319.99, 649971.0     # T-WorksFineGF Skipper at 40-45 km (logs/20261021_075304: 3480 kg / 590 m/s)


def make_world(target, ticks, alt=41000.0, speed=2000.0, n_parts=28, ut0=200000.0, burn=False, gaps=None,
               zero_thrust=(), east_off=60000.0):
    """A falling T-WorksFineGF-like core (grid fins + airbrakes, one engine) `alt` m above the pad, descending at
    30 deg below the horizon towards it. The mass stream reads 0 after `ticks` control ticks (loop ends: "vessel
    lost"). Returns (conn, vessel, state) with state['tick'] advanced by the test's tick hook.
    burn=True: the engine really burns propellant (mass flow F_VAC / (ISP g0) x the game's throttle, integrated in
    game time); gaps {tick: s}: that tick stalls s extra game seconds (engine still burning); zero_thrust: ticks
    whose vessel.thrust sample reads 0 (the sample after a DEADMAN cut, flight #4)."""
    st = {"tick": 0, "ticks": ticks, "mass": 20308.0, "throttle": 0.0, "extra": 0.0, "last": None,
          "gaps": dict(gaps or {}), "zero": set(zero_thrust)}
    ut = Prop(lambda: ut0 + CTR.total * FRAME_S + st["extra"])
    lat, lon = target
    pad = np.array(surface_point(lat, lon))
    up = pad / np.linalg.norm(pad)
    east = np.cross(np.array([0.0, 1.0, 0.0]), up)
    east /= np.linalg.norm(east)
    north = np.cross(up, east)
    p0 = pad + up * alt - east * east_off
    v0 = speed * (math.cos(math.radians(30)) * east - math.sin(math.radians(30)) * up)

    def pos(rf=None):
        return tuple(p0 + v0 * (CTR.total * FRAME_S) * 0.0)     # frozen geometry: the counts, not the dynamics

    def vel(rf=None):
        return tuple(v0)

    def mass():
        if burn:
            u = ut.fn()
            if st["last"] is not None:
                st["mass"] -= F_VAC / (ISP * 9.80665) * control._peek("throttle") * max(u - st["last"], 0.0)
            st["last"] = u
        return 0.0 if st["tick"] >= st["ticks"] else st["mass"]

    body_rf = Remote("body_rf")
    body = Remote("body", reference_frame=body_rf, surface_gravity=9.81, radius=R_BODY, equatorial_radius=R_BODY,
                  surface_position=Method(lambda la, lo, rf: surface_point(la, lo)))
    rfv = Remote("vessel_rf")
    srf = Remote("surface_rf")

    def part(name, i, y=1.0):
        mods = []
        if "Grid Fin" in name:
            mods = [Remote("mod%d_%d" % (i, j), name=nm, has_event=Method(lambda ev: ev == "Extend Fins"),
                           trigger_event=Method(lambda ev: None))
                    for j, nm in enumerate(("ModuleControlSurface", "ModuleAnimateGeneric", "ModuleTest"))]
        return Remote("part%d" % i, name=name, title=name, modules=mods, reference_frame=Remote("prf%d" % i),
                      position=Method(lambda rf, y=y: (1.0, y, 0.0)), temperature=Prop(lambda: 500.0),
                      skin_temperature=Prop(lambda: 600.0), max_temperature=2000.0, max_skin_temperature=2400.0)

    names = ["Grid Fin M"] * 4 + ["airbrake1"] * 8 + ["engineLargeSkipper.v2", "Rockomax64.BW", "landingLeg1-2"]
    names += ["filler%d" % i for i in range(n_parts - len(names))]
    parts = [part(nm, i, y=(2.0 if "Grid Fin" in nm else -1.0)) for i, nm in enumerate(names)]
    css = [Remote("cs%d" % i, part=p, pitch_enabled=True, yaw_enabled=True, roll_enabled=True, authority_limiter=100.0)
           for i, p in enumerate(p for p in parts if p._peek("name") in ("Grid Fin M", "airbrake1"))]
    engine = Remote("engine", active=True, gimballed=True, gimbal_limit=1.0, specific_impulse=319.99,
                    vacuum_specific_impulse=320.0)
    legs = [Remote("leg%d" % i, is_grounded=False) for i in range(4)]
    parts_obj = Remote("parts", all=Prop(lambda: list(parts)), control_surfaces=css, engines=[engine], rcs=[],
                       legs=legs)
    control = Remote("control", throttle=0.0, brakes=True, gear=False, rcs=False, sas=False, pitch=0.0, yaw=0.0,
                     roll=0.0, current_stage=1)
    ap = Remote("autopilot", reference_frame=None, time_to_peak=(1, 1, 1), max_angular_velocity=(1, 1, 1),
                engaged=False, target_direction=(0, 1, 0), error=Prop(lambda: 1.0))
    flight_body = Remote("flight_body", drag=Prop(lambda: (0.0, 50.0, 0.0)), surface_altitude=Prop(lambda: alt + 50.0),
                         mean_altitude=alt)
    flight_srf = Remote("flight_srf", roll=Prop(lambda: 3.0))
    nose_dir = tuple(-np.asarray(v0) / speed)
    vessel = Remote("vessel", orbit=Remote("orbit", body=body), position=Method(pos), velocity=Method(vel),
                    mass=Prop(mass), dry_mass=12000.0, available_thrust=Prop(lambda: F_VAC),
                    thrust=Prop(lambda: 0.0 if st["tick"] in st["zero"] else F_VAC * control._peek("throttle")),
                    specific_impulse=ISP,
                    vacuum_specific_impulse=320.0, situation=Prop(lambda: "VesselSituation.flying"),
                    flight=Method(lambda rf=None: flight_srf if rf is srf else flight_body),
                    direction=Method(lambda rf: nose_dir), angular_velocity=Method(lambda rf: (0.0, 0.01, 0.0)),
                    parts=parts_obj, control=control, auto_pilot=ap, reference_frame=rfv,
                    surface_reference_frame=srf, resources=Remote("res", amount=Method(lambda n: 300.0)))
    sc = Remote("space_center", ut=ut, physics_warp_factor=1, rails_warp_factor=0, active_vessel=vessel,
                transform_direction=Method(lambda d, a, b: (1.0, 0.0, 0.0)), camera=Remote("camera"),
                save=Method(lambda n: None))
    conn = FakeConn(sc)
    return conn, vessel, st


class _Patch:
    def __init__(self):
        self.undo = []

    def setattr(self, obj, name, val):
        self.undo.append((obj, name, getattr(obj, name)))
        setattr(obj, name, val)

    def restore(self):
        for obj, name, val in reversed(self.undo):
            setattr(obj, name, val)


def run_lander(landing, core, tmp_dir, rec, ticks=40, frame_s=None, entry_dv=1e5, **world):
    """Fly `ticks` control ticks of the REAL Lander.run (module `landing`) in the entry burn (41 km, 2000 m/s) on
    the counting fake. The predictors (pure Python, no RPC) are stubbed; the DEADMAN thread (its own connection)
    is not started. Returns dict(counts=[main-thread RPCs per tick], game=[game s per tick], names=[per-tick RPC
    name counts], seq=[main-thread RPC sequence], csv=text, log=[lines], lander)."""
    global CTR, FRAME_S
    import os
    import types
    import time as _time
    from kspilot import deorbit
    CTR = Counter()
    old_frame = FRAME_S
    if frame_s is not None:
        FRAME_S = frame_s
    conn, vessel, st = make_world(core.KSC_PAD, ticks, **world)
    P = _Patch()
    out = dict(counts=[], game=[], names=[], log=[])
    mark = {"n": None, "ut": None, "names": {}}

    def tick_end():
        n, u = CTR.main_count(), conn.space_center._peek("ut")
        if mark["n"] is not None:
            out["counts"].append(n - mark["n"])
            out["game"].append(u - mark["ut"])
            out["names"].append({k: v - mark["names"].get(k, 0) for k, v in CTR.names.items()
                                 if v - mark["names"].get(k, 0)})
        mark["n"], mark["ut"], mark["names"] = n, u, dict(CTR.names)
        st["extra"] += st["gaps"].get(st["tick"], 0.0)     # a stall: game time passes, the engine keeps burning
        st["tick"] += 1

    try:
        P.setattr(core, "_AL", str(tmp_dir))
        P.setattr(landing.Lander, "_deadman", lambda self: None)
        P.setattr(deorbit, "impact_correction", lambda *a, **k: types.SimpleNamespace(
            cross=5.0 + 0.1 * a[1][0], along=100.0, heading=90.0, impact=None))
        P.setattr(landing, "time", types.SimpleNamespace(
            time=_time.time, sleep=lambda d: tick_end() if abs(d - 0.04) < 1e-9 else None))

        def log(*a):
            out["log"].append(" ".join(str(x) for x in a))
        log.path = os.path.join(str(tmp_dir), "landing.txt")
        lander = landing.Lander(conn, vessel, log=log, h_com=8.0)
        lander.draw = True
        lander.rec_v2 = rec
        lander.cda_out, lander.cda_in, lander.heading = 13.0, 5.0, 90.0
        lander.drag_out = lander.drag_in = lambda alt, v, mach=None: 1000.0
        lander.entry_dv = entry_dv                # default: never reached, the burn lasts the whole count
        lander.run(timeout=60)
    finally:
        P.restore()
        FRAME_S = old_frame
    out["missing"] = sorted(set(CTR.missing))
    out["seq"] = list(CTR.seq)
    out["csv"] = open(os.path.join(str(tmp_dir), "landing_landing.csv")).read()
    out["lander"], out["world"], out["vessel"] = lander, st, vessel
    return out


# =====================================================================================================================
# Two vessels in Kerbin orbit (docking rung R1: tools/rdv_r1.py --dry-run and tests/test_rdv_r1.py)
# =====================================================================================================================
MU_KERBIN = 3.5316e12


class Frame:
    """A reference frame of the orbit fake: nrf axes (vessel attitudes are not simulated: every vessel / port frame
    is the body's non-rotating frame translated to that object); origin() -> (pos, vel) in nrf."""

    def __init__(self, label, origin=None):
        self.label = label
        self.origin = origin or (lambda: (np.zeros(3), np.zeros(3)))

    def __repr__(self):
        return "<Frame %s>" % self.label


class OrbitWorld:
    """Two-body Kerbin world: chaser (active vessel, T-WorksFineGF-DOCK-like core + cargo) and a passive target.

    kRPC frames are left-handed with the prograde pole along -y (kspilot.maneuver.POLE): an equatorial prograde orbit
    lies in the x-z plane, r = a (cos th, 0, sin th), v = V (-sin th, 0, cos th). Game time advances only through
    warp_to, the tool's sleep hook (advance) and the impulsive burn hook (burn). RCS translation (control.right /
    forward / up with control.rcs on) pushes the chaser along nrf +x / +y / -z (attitude = nrf axes) times rcs_sign.
    noise: 1-sigma white noise (m) on every docking-port position reading, noise_v (m/s) on part velocities.
    target_loaded: None = loaded inside 2.25 km (KSP physics range), else forced. landed_core: a third vessel, the
    core landed on the pad after the station flight (recoverable)."""

    def __init__(self, alt_c=80e3, alt_t=100e3, phase0=1.0, ut0=500000.0, inc_t_deg=0.0, ecc_c=0.0, noise=0.01,
                 noise_v=0.002, burn_err=0.0, seed=1, ec=(2000.0, 2000.0), mono=50.0, rcs_force=1000.0,
                 rcs_rot=False, target_loaded=None, hibernate_ok=False, rcs_sign=(1, 1, 1), rcs_scale=1.0,
                 landed_core=False):
        global CTR
        CTR = Counter()
        self.rng = np.random.default_rng(seed)
        self.ut = ut0
        self.noise, self.noise_v, self.burn_err = noise, noise_v, burn_err
        self.burns, self.saved, self.jettisoned = [], [], []
        self.mass_c, self.mono = 23800.0, mono
        self.ec, self.target_loaded = list(ec), target_loaded
        self.rcs_force, self.rcs_sign, self.rcs_scale = rcs_force, np.asarray(rcs_sign, float), rcs_scale
        R = 600000.0
        self.state = {"c": self._kepler_state(R + alt_c, 0.0, ecc=ecc_c),
                      "t": self._kepler_state(R + alt_t, phase0, inc=math.radians(inc_t_deg))}
        self.t_state = ut0
        self.nrf = Frame("nrf")
        self.body = Remote("Kerbin", name="Kerbin", gravitational_parameter=MU_KERBIN, equatorial_radius=R,
                           non_rotating_reference_frame=self.nrf, reference_frame=Frame("body_rotating"))
        self.vessels = {k: self._vessel(k, rcs_rot, hibernate_ok) for k in ("c", "t")}
        self.vessels["c"]._a["name"] = "R1-Chaser"
        self.vessels["t"]._a["name"] = "R1-Station"
        self.recovered = []
        self.landed = Remote("landed_core", name="T-WorksFineGF-DOCK", situation="VesselSituation.landed",
                             recoverable=True, recover=Method(lambda: self.recovered.append("landed_core")))
        self.sc = Remote("space_center", ut=Prop(lambda: self.ut), active_vessel=self.vessels["c"],
                         vessels=Prop(lambda: [self.vessels["c"], self.vessels["t"]] +
                                      ([self.landed] if landed_core and not self.recovered else [])),
                         target_vessel=None,
                         warp_to=Method(lambda ut, *a: self.advance_to(ut)),
                         transform_direction=Method(lambda d, a, b: tuple(d)),
                         save=Method(lambda n: self.saved.append(n)), load=Method(lambda n: None),
                         rails_warp_factor=0, physics_warp_factor=0)
        self.conn = FakeConn(self.sc)

    # ---------------------------------------------------------------- dynamics
    @staticmethod
    def _kepler_state(a, th, inc=0.0, ecc=0.0):
        """State at true anomaly th (periapsis at th = 0), orbit tilted by inc about the x axis."""
        p = a * (1 - ecc * ecc)
        r = p / (1 + ecc * math.cos(th))
        h = math.sqrt(MU_KERBIN * p)
        pos = np.array([r * math.cos(th), 0.0, r * math.sin(th)])
        vel = np.array([-MU_KERBIN / h * math.sin(th), 0.0, MU_KERBIN / h * (ecc + math.cos(th))])
        c, s = math.cos(inc), math.sin(inc)
        rot = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        return rot @ pos, rot @ vel

    def rv(self, k):
        from kspilot.entry import kepler_propagate
        r, v = self.state[k]
        dt = self.ut - self.t_state
        if abs(dt) < 1e-9:
            return np.array(r), np.array(v)
        return kepler_propagate(r, v, dt, MU_KERBIN)

    def _sync(self):
        self.state = {k: self.rv(k) for k in ("c", "t")}
        self.t_state = self.ut

    def advance(self, dt):
        """Advance game time by dt s at 1x; RCS translation thrust acts on the chaser (sub-steps <= 0.1 s)."""
        n = max(1, int(math.ceil(dt / 0.1)))
        for _ in range(n):
            self._sync()
            self.ut += dt / n
            self._sync()
            ctl = self.vessels["c"]._a["control"]
            if ctl._peek("rcs") and self.mono > 0:
                cmd = np.clip(np.array([ctl._peek("right"), ctl._peek("forward"), -ctl._peek("up")], float), -1, 1)
                if np.any(cmd):
                    f = self.rcs_force * self.rcs_scale * np.array([2.0, 4.0, 2.0]) * cmd * self.rcs_sign
                    r, v = self.state["c"]
                    self.state["c"] = (r, v + f / self.mass_c * (dt / n))
                    self.mono = max(self.mono - float(np.sum(np.abs(f))) * (dt / n) / (240.0 * 9.80665) / 4.0, 0.0)
        self.ec[0] = max(self.ec[0] - 0.1 * dt, 0.0)

    def advance_to(self, ut):
        if ut > self.ut:
            dt = ut - self.ut
            self._sync()
            self.ut = ut                       # on rails: analytic jump, no thrust
            self._sync()
            self.ec[0] = max(self.ec[0] - 0.1 * dt, 0.0)

    def burn(self, conn, v, ut, dv, log=print, **kw):
        """Impulsive stand-in for kspilot.burn.execute_burn: dv (nrf) applied at ut with burn_err relative scatter."""
        if ut < self.ut - 20:
            log("burn: MISSED (fake)")
            return None
        self.advance_to(ut)
        dv = np.asarray(dv, float)
        err = self.burn_err * float(np.linalg.norm(dv)) * self.rng.standard_normal(3)
        r, vv = self.state["c"]
        self.state["c"] = (r, vv + dv + err)
        self.mass_c *= math.exp(-float(np.linalg.norm(dv)) / (320.0 * 9.80665))
        self.burns.append((self.ut, dv.copy(), err))
        log("burn (fake impulse): %.2f m/s at UT %.0f" % (float(np.linalg.norm(dv)), self.ut))
        return 0.0

    def sleep(self, dt):
        self.advance(dt)

    def distance(self):
        return float(np.linalg.norm(self.rv("c")[0] - self.rv("t")[0]))

    # ---------------------------------------------------------------- objects
    def _vessel(self, k, rcs_rot, hibernate_ok):
        w = self

        def pos(rf, extra=np.zeros(3), noise=0.0):
            o, _ = rf.origin()
            p = w.rv(k)[0] + extra - o
            return tuple(p + noise * w.rng.standard_normal(3)) if noise else tuple(p)

        def vel(rf, noise=0.0):
            _, ov = rf.origin()
            vv = w.rv(k)[1] - ov
            return tuple(vv + noise * w.rng.standard_normal(3)) if noise else tuple(vv)

        def loaded():
            if k == "c":
                return True
            if w.target_loaded is not None:
                return bool(w.target_loaded)
            return w.distance() < 2250.0

        vframe = Frame("vessel_%s" % k, lambda: w.rv(k))
        port_off = np.array([0.0, 10.5 if k == "c" else 1.9, 0.0])     # port ahead of the CoM along vessel +y
        pframe = Frame("port_%s" % k, lambda: (w.rv(k)[0] + port_off, w.rv(k)[1]))
        ppart = Remote("port_part_%s" % k, title="Clamp-O-Tron Docking Port", name="dockingPort2",
                       position=Method(lambda rf: pos(rf, port_off, w.noise)),
                       velocity=Method(lambda rf: vel(rf, w.noise_v)), max_temperature=2000.0,
                       max_skin_temperature=2000.0)
        port = Remote("port_%s" % k, part=ppart, state="DockingPortState.ready", shielded=False, docked_part=None,
                      reference_frame=pframe, position=Method(lambda rf: pos(rf, port_off, w.noise)),
                      direction=Method(lambda rf: (0.0, 1.0, 0.0)), rotation=Method(lambda rf: (0.0, 0.0, 0.0, 1.0)))
        rcs = []
        if k == "c":
            for i, az in enumerate((0, 90, 180, 270)):
                a = math.radians(az)
                rp = np.array([-1.3 * math.cos(a), 0.4, 1.3 * math.sin(a)])     # ring 0.4 m ahead of the CoM
                tang = np.array([math.sin(a), 0.0, math.cos(a)])
                dirs = [np.array([0.0, 1.0, 0.0]), np.array([0.0, -1.0, 0.0]), tang, -tang]
                thr = [Remote("thr%d_%d" % (i, j), thrust_direction=Method(lambda rf, d=d: tuple(d)),
                              thrust_position=Method(lambda rf, rp=rp: tuple(rp))) for j, d in enumerate(dirs)]
                part = Remote("rcs_part%d" % i, title="RV-105 RCS Thruster Block", name="RCSBlock.v2",
                              position=Method(lambda rf, rp=rp: pos(rf, rp)), max_temperature=1500.0,
                              max_skin_temperature=1500.0)
                rcs.append(Remote("rcs%d" % i, part=part, enabled=True, active=Prop(lambda: True),
                                  pitch_enabled=rcs_rot, yaw_enabled=rcs_rot, roll_enabled=rcs_rot,
                                  forward_enabled=True, up_enabled=True, right_enabled=True,
                                  max_thrust=Prop(lambda: 4.0 * w.rcs_force),
                                  available_thrust=Prop(lambda: 4.0 * w.rcs_force), thrust_limit=1.0,
                                  has_fuel=Prop(lambda: w.mono > 0), thrusters=thr))
        fair = [Remote("fairing_%s" % k, part=Remote("fairing_part_%s" % k, title="AE-FF2 Fairing"), jettisoned=False,
                       jettison=Method(lambda: w.jettisoned.append(k)))]
        mods = []
        if hibernate_ok:
            mods = [Remote("cmd_%s" % k, name="ModuleCommand", field_list=[
                Remote("f_hw_%s" % k, name="hibernateOnWarp", bool_value=False),
                Remote("f_h_%s" % k, name="hibernation", bool_value=False)])]
        probe = Remote("probe_%s" % k, title="HECS2", modules=mods)
        parts = Remote("parts_%s" % k, docking_ports=[port], rcs=rcs, fairings=fair, all=[probe], reaction_wheels=[],
                       controlling=probe)

        def amount(name):
            return {"MonoPropellant": w.mono if k == "c" else 0.0, "ElectricCharge": w.ec[0] if k == "c" else 1000.0,
                    "LiquidFuel": 875.0 if k == "c" else 27.0}.get(name, 0.0)

        def maxa(name):
            return {"MonoPropellant": 50.0 if k == "c" else 0.0, "ElectricCharge": w.ec[1] if k == "c" else 1000.0,
                    "LiquidFuel": 4320.0 if k == "c" else 27.0}.get(name, 0.0)

        def elements():
            r, vv = w.rv(k)
            h = np.cross(r, vv)
            e = np.cross(vv, h) / MU_KERBIN - r / np.linalg.norm(r)
            a = 1.0 / (2 / np.linalg.norm(r) - float(np.dot(vv, vv)) / MU_KERBIN)
            inc = math.acos(max(-1.0, min(1.0, -h[1] / np.linalg.norm(h))))
            return a, float(np.linalg.norm(e)), inc

        orbit = Remote("orbit_%s" % k, body=w.body, semi_major_axis=Prop(lambda: elements()[0]),
                       eccentricity=Prop(lambda: elements()[1]), inclination=Prop(lambda: elements()[2]),
                       period=Prop(lambda: 2 * math.pi * math.sqrt(elements()[0] ** 3 / MU_KERBIN)),
                       next_closest_approach=Method(lambda o: w._closest()),
                       relative_inclination=Method(lambda o: w._rel_inc()))
        control = Remote("control_%s" % k, rcs=False, sas=False, sas_mode=None, forward=0.0, right=0.0, up=0.0,
                         throttle=0.0, current_stage=1)
        ap = Remote("ap_%s" % k, engaged=False, reference_frame=None, target_direction=(0, 1, 0),
                    disengage=Method(lambda: None), error=Prop(lambda: 0.5))
        f = w.rcs_force
        return Remote("vessel_%s" % k, name="v%s" % k, type="VesselType.ship", recoverable=False,
                      situation=Prop(lambda: "VesselSituation.orbiting"), loaded=Prop(loaded), packed=False,
                      mass=Prop(lambda: w.mass_c if k == "c" else 2230.0), available_thrust=Prop(lambda: 650000.0),
                      specific_impulse=320.0, orbit=orbit, reference_frame=vframe,
                      position=Method(lambda rf: pos(rf)), velocity=Method(lambda rf: vel(rf)),
                      direction=Method(lambda rf: (0.0, 1.0, 0.0)), parts=parts,
                      resources=Remote("res_%s" % k, amount=Method(amount), max=Method(maxa)),
                      control=control, auto_pilot=ap,
                      available_rcs_force=Prop(lambda: ((2 * f, 4 * f, 2 * f), (-2 * f, -4 * f, -2 * f))))

    def _rel_inc(self):
        hc, ht = np.cross(*self.rv("c")), np.cross(*self.rv("t"))
        return math.acos(max(-1.0, min(1.0, float(np.dot(hc, ht) / np.linalg.norm(hc) / np.linalg.norm(ht)))))

    def _closest(self):
        """Next closest approach within one chaser period (10 s grid)."""
        from kspilot.entry import kepler_propagate
        rc, vc = self.rv("c")
        rt, vt = self.rv("t")
        a = 1.0 / (2 / np.linalg.norm(rc) - float(np.dot(vc, vc)) / MU_KERBIN)
        best = (float("inf"), 0.0)
        for t in np.arange(0.0, 2 * math.pi * math.sqrt(a ** 3 / MU_KERBIN), 10.0):
            d = float(np.linalg.norm(kepler_propagate(rc, vc, t, MU_KERBIN)[0] -
                                     kepler_propagate(rt, vt, t, MU_KERBIN)[0]))
            if d < best[0]:
                best = (d, t)
        return Remote("ca", ut=self.ut + best[1], distance=best[0], relative_speed=0.0)


# =====================================================================================================================
# KSC pad with the V17 light kit (tools/light_check.py --dry-run and tests/test_light_check.py)
# =====================================================================================================================
class PadWorld:
    """Space centre; sc.launch_vessel puts a LIT core on the pad (pre_launch) with n_lamps Spotlight Mk1 parts on the
    Light action group (vessel.control.lights). dead: lamp indices that never switch on. flying: the game starts in the
    flight scene with a mission in orbit (the tool must refuse). Game time moves only through warp_to / daylight."""

    def __init__(self, n_lamps=8, dead=(), flying=False, ut0=100000.0, recoverable=True):
        global CTR
        CTR = Counter()
        self.ut, self.n_lamps, self.dead, self.recoverable = ut0, n_lamps, set(dead), recoverable
        self.launched, self.recovered = [], []
        self.scene = "GameScene.flight" if flying else "GameScene.space_center"
        self.vessel = self._vessel("mission core", "VesselSituation.orbiting", 0) if flying else None
        w = self

        def active():
            if w.vessel is None:
                raise RuntimeError("no active vessel")          # kRPC outside the flight scene
            return w.vessel

        def launch(directory, name, site, *rest):
            w.launched.append((directory, name, site))
            w.vessel = w._vessel(name, "VesselSituation.pre_launch", w.n_lamps)
            w.scene = "GameScene.flight"

        def warp_to(ut):
            w.ut = max(w.ut, ut)
        self.sc = Remote("sc", ut=Prop(lambda: w.ut), active_vessel=Prop(active), launch_vessel=Method(launch),
                         warp_to=Method(warp_to))
        self.conn = FakeConn(self.sc)
        self.conn.krpc = Remote("krpc", paused=False, current_game_scene=Prop(lambda: w.scene))

    def daylight(self, conn, target_el, log=print):
        """core.set_daylight stand-in: warps to the requested sun elevation."""
        self.ut += 3000.0
        log("daylight: sun elevation %.1f deg (target %.0f) [fake]" % (target_el, target_el))
        return float(target_el)

    def _vessel(self, name, sit, n):
        w = self
        control = Remote("control", lights=False)

        def lamp(i):
            def on():
                return bool(control._peek("lights")) and i not in w.dead
            part = Remote("lamp%d" % i, name="spotLight3", title="Spotlight Mk1", tag="",
                          temperature=Prop(lambda: 290.0 + (4.0 if on() else 0.0)), max_temperature=2000.0)
            return Remote("light%d" % i, part=part, active=Prop(on), power_usage=Prop(lambda: 0.02 if on() else 0.0))

        def recover():
            w.recovered.append(name)
            w.vessel = None
            w.scene = "GameScene.space_center"
        flight = Remote("flight_" + name, latitude=-0.0972079, longitude=-74.557673)
        return Remote("vessel_" + name, name=name, situation=Prop(lambda: sit), recoverable=w.recoverable,
                      recover=Method(recover), control=control, flight=Method(lambda *a: flight),
                      parts=Remote("parts_" + name, lights=[lamp(i) for i in range(n)]),
                      resources=Remote("res_" + name, amount=Method(lambda r: 1000.0), max=Method(lambda r: 1000.0)))
