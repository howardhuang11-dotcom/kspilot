"""Offline replay simulator for the kspilot booster descent (no game connection). See replay-simulator notes.

3-D point mass in the ROTATING pad-ENU frame (origin at the pad, axes E,N,U; Kerbin's centre at (0,0,-RP)),
with Coriolis + centrifugal terms and inverse-square gravity along the LOCAL vertical. Aerodynamics and the
attitude response are identified from the landing CSVs in the correct frame:
  drag    a_D = -[CdA(Mach, aoa) + sin^2(aoa)(d1 cos4phi + d2 sin4phi)] q / m along the velocity (2-D table
          fitted to the logged kRPC drag; phi = tilt-plane angle about the stage axis minus the logged roll:
          the cruciform grid fins)
  normal  F_N = q [a1(M) s + a2(M) s^2 + s^2 (f1 sin4phi + f2 cos4phi)] along the top's tilt off the airflow
          (s = sin aoa; a2 < 0: towards the leading ENGINE end above ~2 deg) + q [a3 s + s^2 (f3 sin4phi +
          f4 cos4phi)] perpendicular to it
  attitude: the top-of-stage direction relaxes (tau, slew-limited wmax/(1+q/qw)) towards the command, whose
          offset from the airflow is scaled by G(q) and clipped to the envelope s_max(q); thrusting -> the
          command itself. Saturated commands (beyond the envelope) are NOT validated: see sat_frac.
  thrust  thr x T_avail along the top direction; T_avail = T_vac - k_p p; mdot = thr x 206.9 kg/s

usage:
  replay_sim.py [--logs DIR] validate [out.json]   leave-one-out identification + replay of every GF0T flight
  replay_sim.py [--logs DIR] identify              identify on all flights, print the model (JSON)
  replay_sim.py [--logs DIR] replay LANDING.csv    replay one flight (model identified on the other flights)
Data: the landing CSVs *_orbit_T-WorksFineGF0T_landing.csv (plain or .gz). Searched in --logs DIR, else in
KSP_REPLAY_LOGS (an explicit directory is the only place searched), else data/raw/logs, logs/ and tests/data of
the repository. 'validate' and 'identify' need at least two flights, 'replay' at least one other; when the data
are missing the tool prints one line saying what is needed and exits with status 2.
API (closed loop): see fly() and the example in replay-simulator notes.
"""
import copy
import csv
import glob
import gzip
import json
import math
import os
import re
import shutil
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from kspilot import atmo  # noqa: E402

MU, R = atmo.MU, atmo.R
PAD_ALT = atmo.KERBIN["ksc_pad"]["surface_height"]
RP = R + PAD_ALT                               # radius of the pad-frame origin (Kerbin's centre at (0,0,-RP))
LAT = math.radians(atmo.KERBIN["ksc_pad"]["lat"])
W = 2 * math.pi / atmo.KERBIN["rotational_period"]
OMEGA = W * np.array([0.0, math.cos(LAT), math.sin(LAT)])   # Kerbin spin axis in pad ENU (KSC ~ equator)
CENTRE = np.array([0.0, 0.0, RP])
G0 = 9.80665
_ALT = np.asarray(atmo.KERBIN["alt"], float)
_RHO = np.asarray(atmo.KERBIN["density"], float)
_PRES = np.asarray(atmo.KERBIN["pressure"], float)
_SND = np.sqrt(1.4 * _PRES / np.maximum(_RHO, 1e-12))     # ideal-gas sound speed sqrt(gamma p / rho)
_SND[_RHO < 1e-9] = _SND[np.where(_RHO >= 1e-9)[0][-1]]

GF0T_SUFFIX = "_orbit_T-WorksFineGF0T_landing.csv"
LOG_GLOB = "2026102*" + GF0T_SUFFIX
_CACHE = os.path.join(tempfile.gettempdir(), "kspilot-flightdata")


class NeedData(Exception):
    """The flight logs a tool needs are not available; the tool's main() prints one line and exits with status 2."""


def log_dirs():
    """(directories to search, explicit). An explicit directory (--logs, which sets KSP_REPLAY_LOGS, or
    KSP_REPLAY_LOGS itself) is the only place searched; otherwise data/raw/logs, logs and tests/data of the repo."""
    p = os.environ.get("KSP_REPLAY_LOGS")
    if p:
        return [p], True
    return [d for d in (os.path.join(ROOT, "data", "raw", "logs"), os.path.join(ROOT, "logs"),
                        os.path.join(ROOT, "tests", "data")) if os.path.isdir(d)], False


def need_data(what):
    dirs, explicit = log_dirs()
    where = ("--logs / KSP_REPLAY_LOGS = %s" % dirs[0]) if explicit else \
        "data/raw/logs, logs and tests/data of the repository"
    return NeedData("needs %s (searched: %s). Point --logs (or KSP_REPLAY_LOGS) at a directory holding the "
                    "GF0T landing CSVs; they are released in data/raw/logs/ of this repository." % (what, where))


def _unpack(gz):
    """A .csv.gz is unpacked once into a cache directory (with its .txt console log when present)."""
    out = os.path.join(_CACHE, os.path.basename(gz)[:-3])
    if not os.path.exists(out) or os.path.getmtime(out) < os.path.getmtime(gz):
        os.makedirs(_CACHE, exist_ok=True)
        tmp = out + ".%d.tmp" % os.getpid()
        with gzip.open(gz, "rb") as src, open(tmp, "wb") as dst:
            shutil.copyfileobj(src, dst)
        os.replace(tmp, out)
    side = gz[:-len("_landing.csv.gz")] + ".txt"
    if os.path.exists(side):
        shutil.copyfile(side, out[:-len("_landing.csv")] + ".txt")
    return out


def find_log(name):
    """Path of the landing CSV `name` (given with .csv; a .csv.gz copy is accepted), else NeedData."""
    if os.path.isfile(name):
        return name
    for d in log_dirs()[0]:
        for cand in (name, name + ".gz"):
            p = os.path.join(d, cand)
            if os.path.isfile(p):
                return _unpack(p) if p.endswith(".gz") else p
    raise need_data(os.path.basename(name))


def rho_of(alt):
    return np.interp(alt, _ALT, _RHO, right=0.0)


def sound_of(alt):
    return np.interp(alt, _ALT, _SND)


def pres_of(alt):
    return np.interp(alt, _ALT, _PRES, right=0.0)


def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def fin_phi(nose, air, up, roll_deg):
    """Angle (rad) of the tilt plane (nose off the airflow) about the stage axis, measured from the roll
    reference (local up projected normal to the axis) minus the logged roll: the cruciform grid fins make
    the aero forces depend on 4*phi (identified: R^2 0.5-0.6 of the normal-force residual at aoa > 4 deg)."""
    b = nose - np.dot(nose, air) * air
    nb = np.linalg.norm(b)
    ref = up - np.dot(up, nose) * nose
    nr = np.linalg.norm(ref)
    if nb < 1e-9 or nr < 1e-9 or roll_deg != roll_deg:
        return 0.0
    ref = ref / nr
    bn = b - np.dot(b, nose) * nose
    return math.atan2(float(np.dot(bn, np.cross(nose, ref))), float(np.dot(bn, ref))) - math.radians(roll_deg)


# ----------------------------------------------------------------------------------------------- logs
FIELDS = ("ut", "e", "n", "h", "ve", "vn", "vu", "m", "tmax", "eta", "drag", "dx", "dy", "dz", "thr_act",
          "ne", "nn", "nu", "roll_ang")
OPT_FIELDS = ("ctl_p", "ctl_y", "ctl_r", "ident_cmd")      # newer logs only (nan when absent)
TTP_DEFAULT = 5.0      # kRPC autopilot time_to_peak of the fin craft (landing.py) in every flight before DR-004


def load(path):
    """Landing CSV -> dict of arrays in the TRUE rotating pad-ENU frame.

    The CSV's e,n are pad-ENU, h is the RADIAL height above the pad and vu the RADIAL vertical speed at the
    vehicle. Rebuilt: P (position from the pad origin, ENU, with u the true ENU up coordinate) and
    v (true ENU velocity). h_com is ignored consistently (|P - CENTRE| = RP + h)."""
    rows, phase, opt = [], [], []
    for r in csv.DictReader(open(path)):
        try:
            rows.append([float(r[k]) for k in FIELDS])
            phase.append(r["phase"])
        except (ValueError, KeyError, TypeError):
            continue
        o = []
        for k in OPT_FIELDS:
            try:
                o.append(float(r.get(k, "nan")))
            except (ValueError, TypeError):
                o.append(float("nan"))
        opt.append(o)
    A = np.array(rows)
    d = {k: A[:, i] for i, k in enumerate(FIELDS)}
    O = np.array(opt)
    d.update({k: O[:, i] for i, k in enumerate(OPT_FIELDS)})
    d["phase"] = np.array(phase)
    # autopilot time_to_peak schedule: TTP_DEFAULT except inside an IDENT window flown with KSP_IDENT_TTP
    ttp_win = None
    try:
        for line in open(path.replace("_landing.csv", ".txt")):
            mt = re.search(r"time_to_peak in window ([0-9.]+)", line)
            if mt:
                ttp_win = float(mt.group(1))
                break
    except OSError:
        pass
    d["ttp"] = np.full(len(A), TTP_DEFAULT)
    if ttp_win is not None:
        d["ttp"][~np.isnan(d["ident_cmd"]) & (d["phase"] == "COAST")] = ttp_win
    e, n, h = d["e"], d["n"], d["h"]
    rad = RP + h
    u = np.sqrt(rad ** 2 - e ** 2 - n ** 2) - RP
    vz = (d["vu"] * rad - d["ve"] * e - d["vn"] * n) / (RP + u)
    d["P"] = np.stack([e, n, u], 1)
    d["v"] = np.stack([d["ve"], d["vn"], vz], 1)
    for k, cols in (("cmd", ("dx", "dy", "dz")), ("nose", ("ne", "nn", "nu"))):
        x = np.stack([d[c] for c in cols], 1)
        d[k] = x / np.linalg.norm(x, axis=1)[:, None]
    d["roll"] = d["roll_ang"]                                # kRPC surface-frame roll, deg (fin orientation)
    d["avail"] = d["tmax"] / np.maximum(d["eta"], 1e-3)      # available thrust (thrust = thr_act x avail)
    d["name"] = os.path.basename(path)[:15]
    d["path"] = path
    _derive(d)
    _window(d)
    return d


def _derive(d, k=6):
    P, v = d["P"], d["v"]
    rad = np.linalg.norm(P + CENTRE, axis=1)
    d["alt"] = alt = rad - R                                 # above sea level
    sp = np.linalg.norm(v, axis=1)
    d["q"] = 0.5 * rho_of(alt) * sp ** 2
    d["mach"] = sp / sound_of(alt)
    d["air"] = -v / np.maximum(sp, 1e-6)[:, None]            # where the top points when exactly engine-first
    d["aoa"] = np.degrees(np.arccos(np.clip((d["nose"] * d["air"]).sum(1), -1, 1)))
    d["cmd_aoa"] = np.degrees(np.arccos(np.clip((d["cmd"] * d["air"]).sum(1), -1, 1)))
    up = (P + CENTRE) / rad[:, None]
    d["phi"] = np.array([fin_phi(d["nose"][i], d["air"][i], up[i], d["roll"][i]) for i in range(len(rad))])
    # specific non-gravitational force from central differences (inertial: + Coriolis + centrifugal)
    t, n = d["ut"], len(d["ut"])
    f = np.full((n, 3), np.nan)
    i = np.arange(k, n - k)
    dt = t[i + k] - t[i - k]
    ok = (dt > 0.4) & (dt < 2.5)
    i, dt = i[ok], dt[ok]
    a = (v[i + k] - v[i - k]) / dt[:, None]
    Pc = P[i] + CENTRE
    g = -MU * Pc / np.linalg.norm(Pc, axis=1)[:, None] ** 3
    f[i] = a - g + 2 * np.cross(OMEGA, v[i]) + np.cross(OMEGA, np.cross(OMEGA, Pc))
    d["f"] = f
    # thrust-free mask (no throttle within +-10 samples)
    th = d["thr_act"] > 0.01
    near = np.zeros(n, bool)
    for s in range(-10, 11):
        near |= np.roll(th, s)
    d["coast"] = ~near


def _window(d):
    """Replay window: end of the entry burn (thrust back to 0 above 35 km) -> landing-burn ignition (first
    BURN-phase row, i.e. the lander's landing burn; the q-guard/entry pulses in COAST are replayed as thrust).
    i_gap: first logging gap > 2 s inside the window (inputs unknown inside; ignition inside one = no data)."""
    th, alt = d["thr_act"], d["alt"]
    on = np.where((th > 0.05) & (d["h"] > 35000))[0]
    if len(on):
        seg_end = on[0]
        while seg_end + 1 < len(th) and th[seg_end + 1] > 0.05:
            seg_end += 1
        d["i0"] = i0 = seg_end + 1
    else:                              # started below the entry burn (g5_mid checkpoint): first coasting row
        d["i0"] = i0 = int(np.where(th < 0.05)[0][0])
    b = np.where(d["phase"] == "BURN")[0]
    d["i1"] = int(b[0]) if len(b) else len(th) - 1
    # replay end: ignition, or (ignition above 4 km) on through the landing burn with the logged throttle
    below = np.where(d["h"][d["i1"]:] < 3900.0)[0]
    d["i_end"] = int(d["i1"] + below[0]) if (d["h"][d["i1"]] > 4000.0 and len(below)) else d["i1"]
    gaps = np.where(np.diff(d["ut"][i0:d["i_end"] + 1]) > 2.0)[0]
    d["ign_ok"] = True
    for g in gaps:                     # an unlogged burn (mass drop) inside a gap: the replay must stop there
        if d["m"][i0 + g + 1] < 0.99 * d["m"][i0 + g]:
            d["i_end"] = d["i1"] = i0 + g
            d["ign_ok"] = False
            break
    d["gaps"] = [(float(d["ut"][i0 + g]), float(d["ut"][i0 + g + 1])) for g in gaps if i0 + g < d["i_end"]]
    del alt


def flights(pattern=LOG_GLOB, min_rows=800):
    """Completed GF0T flights (with a landing burn or a ground contact), from the first directories that have them."""
    out, seen = [], set()
    paths = []
    for d in log_dirs()[0]:
        for p in sorted(glob.glob(os.path.join(d, pattern)) + glob.glob(os.path.join(d, pattern + ".gz"))):
            base = os.path.basename(p)[:-3] if p.endswith(".gz") else os.path.basename(p)
            if base not in seen:
                seen.add(base)
                paths.append(_unpack(p) if p.endswith(".gz") else p)
    for p in sorted(paths, key=os.path.basename):
        try:
            d = load(p)
        except Exception:
            continue
        if len(d["ut"]) >= min_rows and (d["phase"] == "BURN").any():
            out.append(d)
    return out


# ----------------------------------------------------------------------------------------------- model
def _hat(x, nodes):
    """Linear-interpolation (hat) basis weights, shape (len(x), len(nodes))."""
    x = np.clip(np.atleast_1d(np.asarray(x, float)), nodes[0], nodes[-1])
    W_ = np.zeros((len(x), len(nodes)))
    j = np.clip(np.searchsorted(nodes, x, side="right") - 1, 0, len(nodes) - 2)
    fr = (x - nodes[j]) / (nodes[j + 1] - nodes[j])
    W_[np.arange(len(x)), j] = 1 - fr
    W_[np.arange(len(x)), j + 1] = fr
    return W_


def _d2(n):
    D = np.zeros((max(n - 2, 0), n))
    for i in range(n - 2):
        D[i, i:i + 3] = (1, -2, 1)
    return D


class Model:
    MACH = np.array([0.3, 0.6, 0.85, 1.0, 1.15, 1.35, 1.6, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.2])
    AOA = np.array([0.0, 1.5, 3.0, 5.0, 8.0, 11.0, 15.0, 20.0, 30.0])        # deg
    LMACH = np.array([0.5, 1.0, 1.5, 2.5, 3.5, 4.5])

    def __init__(self, cda=None, lift=None, att=None, thrust=None, fin=None, dfin=None, roll=96.0, att15=None):
        self.cda_tab = np.asarray(cda) if cda is not None else np.full((len(self.MACH), len(self.AOA)), 5.0)
        self.lift = np.asarray(lift) if lift is not None else np.zeros((3, len(self.LMACH)))   # a1, a2, a3 rows
        self.fin = np.asarray(fin if fin is not None else np.zeros(4), float)    # along sin4p, cos4p; perp sin, cos
        self.dfin = np.asarray(dfin if dfin is not None else np.zeros(2), float)  # CdA: cos4p, sin4p (x sin^2 aoa)
        self.att = dict(att or dict(tau=1.63, G0=1.03, qg=261.0, s_inf=0.16, s0=0.39, wmax=4.1, qw=48.0))
        self.thrust = dict(thrust or dict(T_vac=650131.0, k_p=0.7916, mdot=206.9))
        self.roll = float(roll)                   # default roll angle (deg) for closed-loop runs
        # stiff autopilot (time_to_peak 1.5 s, DR-004 flight 230136; measured only at q 86-121 kPa, Mach 1.8-3.9):
        # mean-tilt gain, lag, and a coning limit cycle (inputs bang-bang saturated) of amplitude `cone` deg,
        # period `period` s, ramped in over q cone_q0..cone_q1 kPa (below 86 kPa: ASSUMPTION, never flown)
        self.att15 = dict(att15 or dict(ttp=1.5, G=0.52, bias=0.5, tau=0.7, wmax=20.0, cone=5.0, period=1.6,
                                        cone_q0=40.0, cone_q1=86.0, q_lo=86.0, q_hi=121.0))
        self.ttp = TTP_DEFAULT                    # current autopilot time_to_peak (set per step by simulate)
        self._up = np.array([0.0, 0.0, 1.0])     # local vertical of the current step (set by simulate)
        self.k_drag, self.k_lift = 1.0, 1.0       # Monte-Carlo scale factors

    # --- aero
    def cda(self, mach, aoa_deg, sin_a=0.0, phi=0.0):
        """Drag area m^2 (drag = CdA q along the airflow)."""
        wm = _hat(mach, self.MACH)[0]
        wa = _hat(aoa_deg, self.AOA)[0]
        s2 = sin_a * sin_a
        return self.k_drag * (float(wm @ self.cda_tab @ wa)
                              + s2 * (self.dfin[0] * math.cos(4 * phi) + self.dfin[1] * math.sin(4 * phi)))

    def normal(self, mach, sin_a, phi=0.0):
        """(along the top's tilt, perpendicular = air x tilt) normal force per unit q, m^2."""
        w = _hat(mach, self.LMACH)[0]
        a1, a2, a3 = self.lift @ w
        s2 = sin_a * sin_a
        s4, c4 = math.sin(4 * phi), math.cos(4 * phi)
        f = self.fin
        return (self.k_lift * (a1 * sin_a + a2 * s2 + s2 * (f[0] * s4 + f[1] * c4)),
                self.k_lift * (a3 * sin_a + s2 * (f[2] * s4 + f[3] * c4)))

    # --- attitude
    def stiff(self):
        """Weight of the stiff-autopilot regime: 0 at time_to_peak >= 5 s (all flights before DR-004), 1 at
        <= 1.5 s; linear in between (INTERPOLATION, no data between 1.5 and 5 s)."""
        t1 = self.att15["ttp"]
        return min(max((TTP_DEFAULT - self.ttp) / (TTP_DEFAULT - t1), 0.0), 1.0)

    def gain(self, q):
        """Steady mean-tilt gain (achieved offset / commanded offset) at the current time_to_peak."""
        w = self.stiff()
        g5 = self.att["G0"] / (1.0 + q / 1e3 / self.att["qg"])
        return (1 - w) * g5 + w * self.att15["G"]

    def trim(self, q):
        """Uncommanded tilt (sin) towards the upper side of the airflow that the soft autopilot (time_to_peak
        5 s, no integral action) leaves standing; the stiff one nulls it (DR-004 bias 0.3 deg)."""
        p = self.att
        qk = q / 1e3
        return (1 - self.stiff()) * math.sin(math.radians(p.get("bmax", 0.0) * qk / (qk + p.get("qb", 50.0))))

    def cone(self, q):
        """Coning limit-cycle amplitude (rad) at the current time_to_peak and dynamic pressure q (Pa)."""
        w = self.stiff()
        if w <= 0:
            return 0.0
        a = self.att15
        r = min(max((q / 1e3 - a["cone_q0"]) / (a["cone_q1"] - a["cone_q0"]), 0.0), 1.0)
        return math.radians(a["cone"]) * r * w

    def att_target(self, cmd, air, q, thrusting):
        if thrusting:
            return cmd
        p = self.att
        b = cmd - np.dot(cmd, air) * air
        b = b * self.gain(q) + self.trim(q) * _upper(air, self._up)
        nb = np.linalg.norm(b)
        smax = min(p["s_inf"] + p["s0"] / max(q / 1e3, 0.05), 0.999)
        if nb > smax:
            b = b * (smax / nb)
        return air * math.sqrt(max(1.0 - float(np.dot(b, b)), 0.0)) + b

    def saturated(self, cmd, air, q):
        """True when the unpowered command asks for more angle of attack than the identified envelope: the
        replay validation is poor there (the achieved tilt azimuth skews, fin forces differ; see the doc)."""
        p = self.att
        b = cmd - np.dot(cmd, air) * air
        g = np.linalg.norm(b) * self.gain(q)
        return g > min(p["s_inf"] + p["s0"] / max(q / 1e3, 0.05), 0.999) + 0.02

    def att_step(self, nose, target, dt, q=0.0, thrusting=False):
        """First-order lag (tau) towards the target, slew-rate limited (wmax deg/s, divided by 1 + q/qw when
        unpowered: slews are slower against the aerodynamic stiffness)."""
        w = self.stiff()
        tau = (1 - w) * self.att["tau"] + w * self.att15["tau"]
        k = 1.0 - math.exp(-dt / tau)
        dn = (target - nose) * k
        w_ = self.att.get("wmax", 99.0) / (1.0 if thrusting else 1.0 + q / 1e3 / self.att.get("qw", 1e9))
        w_ = (1 - w) * w_ + w * self.att15["wmax"]
        lim = math.radians(w_) * dt
        n_ = float(np.linalg.norm(dn))
        if n_ > lim:
            dn *= lim / n_
        return unit(nose + dn)

    # --- thrust
    def t_avail(self, alt):
        return self.thrust["T_vac"] - self.thrust["k_p"] * float(pres_of(alt))

    # --- io
    def to_dict(self):
        return dict(mach=self.MACH.tolist(), aoa=self.AOA.tolist(), cda=np.round(self.cda_tab, 3).tolist(),
                    dfin=np.round(self.dfin, 3).tolist(), lift_mach=self.LMACH.tolist(),
                    lift=np.round(self.lift, 3).tolist(), fin=np.round(self.fin, 3).tolist(), att=self.att,
                    att15=self.att15, thrust=self.thrust, roll=self.roll)

    @classmethod
    def from_dict(cls, D):
        return cls(D["cda"], D["lift"], D["att"], D.get("thrust"), D.get("fin"), D.get("dfin"), D.get("roll", 96.0),
                   D.get("att15"))

    def scaled(self, drag=1.0, lift=1.0, tau=1.0, gain=1.0, smax=1.0):
        """Copy with Monte-Carlo dispersions."""
        m = copy.deepcopy(self)
        m.k_drag *= drag
        m.k_lift *= lift
        m.att["tau"] *= tau
        m.att15["tau"] *= tau
        m.att["G0"] *= gain
        m.att15["G"] *= gain
        m.att["s_inf"] *= smax
        m.att["s0"] *= smax
        return m


# ----------------------------------------------------------------------------------------------- identification
def _id_samples(fl):
    S = []
    for d in fl:
        i0, i1 = d["i0"], d["i1"]
        sel = np.zeros(len(d["ut"]), bool)
        sel[i0:i1] = True
        sel &= d["coast"] & (d["q"] > 500) & (d["phase"] != "BURN")
        S.append((d, sel))
    return S


def identify_drag(fl, lam=0.01):
    """CdA(Mach, aoa) on a bilinear grid + sin^2(aoa) (d1 cos 4phi + d2 sin 4phi): least squares on the logged
    drag / q (kRPC drag, frame-free), 2nd-difference smoothing of the table."""
    M, Aa, Y, S2, PH = [], [], [], [], []
    for d, sel in _id_samples(fl):
        M.append(d["mach"][sel])
        Aa.append(d["aoa"][sel])
        Y.append(d["drag"][sel] / d["q"][sel])
        S2.append(np.sin(np.radians(d["aoa"][sel])) ** 2)
        PH.append(d["phi"][sel])
    M, Aa, Y, S2, PH = map(np.concatenate, (M, Aa, Y, S2, PH))
    nm, na = len(Model.MACH), len(Model.AOA)
    Wm, Wa = _hat(M, Model.MACH), _hat(Aa, Model.AOA)
    X = np.hstack([(Wm[:, :, None] * Wa[:, None, :]).reshape(len(Y), nm * na),
                   (S2 * np.cos(4 * PH))[:, None], (S2 * np.sin(4 * PH))[:, None]])
    pad = np.zeros((1, 2))
    Dm = np.kron(_d2(nm), np.eye(na))
    Da = np.kron(np.eye(nm), _d2(na))
    s = math.sqrt(len(Y)) * lam
    R_ = np.vstack([s * Dm, s * Da, 1e-3 * s * np.eye(nm * na)])
    Xa = np.vstack([X, np.hstack([R_, np.repeat(pad, R_.shape[0], 0)])])
    Ya = np.concatenate([Y, np.zeros(Dm.shape[0] + Da.shape[0]), 1e-3 * s * 5.0 * np.ones(nm * na)])
    c = np.linalg.lstsq(Xa, Ya, rcond=None)[0]
    res = Y - X @ c
    return c[:-2].reshape(nm, na), c[-2:], float(np.sqrt(np.mean((res / Y) ** 2)))


def identify_lift(fl, lam=0.05):
    """Normal force (robust IRLS, both components jointly):
      along the tilt:  m a = q [a1(M) s + a2(M) s^2 + s^2 (f1 sin4phi + f2 cos4phi)]
      perpendicular:   m a = q [a3(M) s + s^2 (f3 sin4phi + f4 cos4phi)]      (s = sin aoa, Mach hat basis)"""
    rows = []
    for d, sel in _id_samples(fl):
        sel = sel & ~np.isnan(d["f"][:, 0]) & (d["q"] > 1500)
        for i in np.where(sel)[0]:
            a, f, nose = d["air"][i], d["f"][i], d["nose"][i]
            fn = f - np.dot(f, a) * a
            b = nose - np.dot(nose, a) * a
            nb = np.linalg.norm(b)
            if nb < math.sin(math.radians(0.3)):
                continue
            bh = b / nb
            rows.append((d["mach"][i], d["q"][i], nb, d["m"][i] * np.dot(fn, bh),
                         d["m"][i] * np.dot(fn, np.cross(a, bh)), d["phi"][i]))
    A = np.array(rows)
    w = _hat(A[:, 0], Model.LMACH)
    nl = len(Model.LMACH)
    q, s, ph = A[:, 1], A[:, 2], A[:, 5]
    z, zo = 0 * w, np.zeros((len(A), 1))
    S4, C4 = (q * s * s * np.sin(4 * ph))[:, None], (q * s * s * np.cos(4 * ph))[:, None]
    Xa = np.hstack([w * (q * s)[:, None], w * (q * s * s)[:, None], z, S4, C4, zo, zo])
    Xp = np.hstack([z, z, w * (q * s)[:, None], zo, zo, S4, C4])
    X, y = np.vstack([Xa, Xp]), np.concatenate([A[:, 3], A[:, 4]])
    D = np.hstack([np.kron(np.eye(3), _d2(nl)), np.zeros((3 * (nl - 2), 4))]) * lam * np.sqrt(np.mean((q * s) ** 2)) \
        * math.sqrt(len(y))
    wt = np.ones(len(y))
    for _ in range(6):                                    # Huber IRLS
        c = np.linalg.lstsq(np.vstack([X * wt[:, None], D]), np.concatenate([y * wt, np.zeros(D.shape[0])]),
                            rcond=None)[0]
        r = y - X @ c
        sc = 1.4826 * np.median(np.abs(r)) + 1e-9
        wt = np.sqrt(np.minimum(1.0, 1.5 * sc / np.maximum(np.abs(r), 1e-9)))
    lift = np.vstack([c[:nl], c[nl:2 * nl], c[2 * nl:3 * nl]])
    return lift, c[3 * nl:], float(sc)


def _att_grid(fl, step=0.2):
    """Flights resampled on a common relative-time grid for a vectorized attitude fit."""
    T = []
    for d in fl:
        i0, i1 = d["i0"], d["i1"]
        t = np.arange(d["ut"][i0], d["ut"][i1], step)
        idx = np.clip(np.searchsorted(d["ut"], t, side="right") - 1, i0, i1)
        up = (d["P"][idx] + CENTRE) / np.linalg.norm(d["P"][idx] + CENTRE, axis=1)[:, None]
        T.append(dict(t=t, air=d["air"][idx], cmd=d["cmd"][idx], nose=d["nose"][idx], q=d["q"][idx], up=up,
                      thr=d["thr_act"][idx] > 0.05,
                      valid=(np.abs(d["ut"][idx] - t) < 0.5) & (np.abs(d["ttp"][idx] - TTP_DEFAULT) < 1e-6)))
    L = max(len(x["t"]) for x in T)
    F = len(T)

    def pad(k, shape, fill):
        A = np.full((L, F) + shape, fill, dtype=float)
        for j, x in enumerate(T):
            A[:len(x["t"]), j] = x[k]
        return A
    G = dict(air=pad("air", (3,), 0.0), cmd=pad("cmd", (3,), 0.0), nose=pad("nose", (3,), 0.0),
             up=pad("up", (3,), 0.0), q=pad("q", (), 0.0), thr=pad("thr", (), 0.0) > 0.5, valid=pad("valid", (), 0.0) > 0.5, step=step)
    G["air"][np.linalg.norm(G["air"], axis=2) == 0] = (0, 0, 1)
    G["up"][np.linalg.norm(G["up"], axis=2) == 0] = (1, 0, 0)
    return G


def _upper(air, up):
    """Unit vector normal to the airflow in the vertical plane, on the upper (sky) side."""
    e1 = np.cross(up, air)
    e1 = e1 / np.maximum(np.linalg.norm(e1, axis=-1, keepdims=True), 1e-12)
    return np.cross(air, e1)


def _att_sim_grid(G, p):
    tau, G0, qg, s_inf, s0, wmax, qw, bmax, qb = p
    L = G["q"].shape[0]
    n = G["nose"][0].copy()
    out = np.empty_like(G["nose"])
    out[0] = n
    k = 1.0 - math.exp(-G["step"] / tau)
    for i in range(1, L):
        a, c, q = G["air"][i], G["cmd"][i], G["q"][i] / 1e3
        b = c - (c * a).sum(1)[:, None] * a
        b = b * (G0 / (1.0 + q / qg))[:, None]
        b = b + np.sin(np.radians(bmax * q / (q + qb)))[:, None] * _upper(a, G["up"][i])
        nb = np.linalg.norm(b, axis=1)
        smax = np.minimum(s_inf + s0 / np.maximum(q, 0.05), 0.999)
        b = b * np.where(nb > smax, smax / np.maximum(nb, 1e-12), 1.0)[:, None]
        t = a * np.sqrt(np.maximum(1 - (b * b).sum(1), 0))[:, None] + b
        t = np.where(G["thr"][i][:, None], c, t)
        dn = (t - n) * k
        dmax = np.radians(np.where(G["thr"][i], wmax, wmax / (1.0 + q / qw))) * G["step"]
        dn *= np.minimum(1.0, dmax / np.maximum(np.linalg.norm(dn, axis=1), 1e-12))[:, None]
        n = n + dn
        n /= np.linalg.norm(n, axis=1)[:, None]
        out[i] = n
    return out


def identify_attitude(fl, p0=(1.7, 1.0, 270.0, 0.16, 0.4, 4.0, 50.0, 2.0, 50.0)):
    """Fit tau, G0, qg, s_inf, s0, wmax, qw, bmax, qb (time_to_peak 5 s samples only) by replaying the attitude alone along each logged airflow/command
    history (0.2 s grid, all flights at once) against the logged top direction (soft-L1 loss, degrees)."""
    from scipy.optimize import least_squares
    G = _att_grid(fl)
    m = G["valid"]

    def res(p):
        o = _att_sim_grid(G, p)
        e = np.degrees(np.linalg.norm(o - G["nose"], axis=2))
        return e[m]
    sol = least_squares(res, p0, bounds=([0.2, 0.2, 5.0, 0.0, 0.0, 0.5, 1.0, -5.0, 1.0],
                                         [10.0, 1.5, 1e5, 1.0, 5.0, 99.0, 1e5, 8.0, 1e4]),
                        loss="soft_l1", f_scale=1.0, diff_step=1e-2, x_scale="jac")
    r = res(sol.x)
    names = ("tau", "G0", "qg", "s_inf", "s0", "wmax", "qw", "bmax", "qb")
    return {k: float(v) for k, v in zip(names, sol.x)}, float(np.median(r)), float(np.percentile(r, 90))


def identify_fast(fl, default=None):
    """Stiff-autopilot regime (time_to_peak < 5 s) from IDENT windows: mean-tilt gain on the commanded axis
    (settled part of each step, averaged over whole limit-cycle periods), residual bias at zero command, coning
    amplitude (rms of the tilt vector about its 1-period running mean), period (zero crossings of the
    cross-axis tilt), lag (t90 of the running mean). Returns the att15 dict and the measured q range."""
    out = dict(default or Model().att15)
    rows = []
    for d in fl:
        sel = d["ttp"] < TTP_DEFAULT - 1e-6
        if sel.sum() < 30:
            continue
        idx = np.where(sel)[0]
        up = (d["P"] + CENTRE) / np.linalg.norm(d["P"] + CENTRE, axis=1)[:, None]
        e1 = np.cross(up, d["air"])
        e1 /= np.linalg.norm(e1, axis=1)[:, None]
        e2 = np.cross(d["air"], e1)
        b1 = np.degrees((d["nose"] * e1).sum(1))
        b2 = np.degrees((d["nose"] * e2).sum(1))
        c2 = np.degrees((d["cmd"] * e2).sum(1))
        t = d["ut"]
        # period from upward zero crossings of the cross-axis tilt
        x = b1[idx]
        zc = t[idx][1:][(x[:-1] < 0) & (x[1:] >= 0)]
        per = float(np.median(np.diff(zc))) if len(zc) > 3 else out["period"]
        # running mean over one period
        mb1 = np.array([b1[idx][np.abs(t[idx] - tt) <= per / 2].mean() for tt in t[idx]])
        mb2 = np.array([b2[idx][np.abs(t[idx] - tt) <= per / 2].mean() for tt in t[idx]])
        cone = float(np.sqrt(np.mean((b1[idx] - mb1) ** 2 + (b2[idx] - mb2) ** 2)))
        ic = d["ident_cmd"][idx]
        for val in np.unique(ic[~np.isnan(ic)]):
            k = idx[ic == val]
            tk = t[k] - t[k[0]]
            if tk[-1] < 2 * per:
                continue
            n_per = math.floor((tk[-1] - per / 2) / per)          # whole periods after a half-period settle
            m = (tk >= tk[-1] - n_per * per)
            g = float(np.mean(b2[k][m]) / np.mean(c2[k][m])) if val > 0 else float("nan")
            bias = float(np.hypot(np.mean(b1[k][m]), np.mean(b2[k][m]))) if val == 0 else float("nan")
            # t90 of the running mean towards its settled value
            mm = mb2[np.isin(idx, k)]
            t90 = float("nan")
            if val > 0:
                tgt = np.mean(b2[k][m])
                j = np.where(mm - mm[0] >= 0.9 * (tgt - mm[0]))[0]
                t90 = float(tk[j[0]]) if len(j) else float("nan")
            rows.append(dict(flight=d["name"], cmd=float(val), gain=g, bias=bias, t90=t90, cone=cone, period=per,
                             q_lo=float(d["q"][k].min() / 1e3), q_hi=float(d["q"][k].max() / 1e3),
                             mach=(float(d["mach"][k].min()), float(d["mach"][k].max())),
                             sat=float(np.nanmean(np.maximum(np.abs(d["ctl_p"][k]), np.abs(d["ctl_y"][k])) >= 0.99)),
                             ttp=float(d["ttp"][k[0]])))
    if rows:
        g = [r["gain"] for r in rows if r["gain"] == r["gain"]]
        b = [r["bias"] for r in rows if r["bias"] == r["bias"]]
        t9 = [r["t90"] for r in rows if r["t90"] == r["t90"]]
        out.update(ttp=rows[0]["ttp"], G=float(np.mean(g)) if g else out["G"], bias=float(np.mean(b)) if b else out["bias"],
                   tau=float(np.mean(t9)) / 2.3 if t9 else out["tau"], cone=float(np.mean([r["cone"] for r in rows])),
                   period=float(np.mean([r["period"] for r in rows])), q_lo=min(r["q_lo"] for r in rows),
                   q_hi=max(r["q_hi"] for r in rows))
        out["cone_q1"] = out["q_lo"]
    return out, rows


def identify_thrust(fl):
    X, Y = [], []
    for d in fl:
        ok = d["thr_act"] > 0.05
        X.append(pres_of(d["alt"][ok]))
        Y.append(d["avail"][ok])
    X, Y = np.concatenate(X), np.concatenate(Y)
    c = np.linalg.lstsq(np.vstack([np.ones_like(X), -X]).T, Y, rcond=None)[0]
    return dict(T_vac=float(c[0]), k_p=float(c[1]), mdot=206.9)


def identify(fl, verbose=False):
    cda, dfin, cda_rms = identify_drag(fl)
    lift, fin, lift_sc = identify_lift(fl)
    att, att_med, att_p90 = identify_attitude(fl)
    rolls = np.concatenate([d["roll"][d["i0"]:d["i1"]] for d in fl])
    att15, _ = identify_fast(fl)
    m = Model(cda, lift, att, identify_thrust(fl), fin, dfin, float(np.nanmedian(rolls)), att15)
    m.fit_stats = dict(cda_rel_rms=cda_rms, lift_resid_N_per_q=lift_sc, att_err_deg_median=att_med,
                       att_err_deg_p90=att_p90, flights=[d["name"] for d in fl])
    if verbose:
        print(json.dumps(m.fit_stats))
    return m


# ----------------------------------------------------------------------------------------------- simulation
class State:
    """Vehicle state handed to a guidance law. Pad ENU (true, rotating with Kerbin), SI units."""
    __slots__ = ("t", "P", "v", "nose", "m")

    def __init__(self, t, P, v, nose, m):
        self.t, self.P, self.v, self.nose, self.m = t, np.asarray(P, float), np.asarray(v, float), unit(nose), m

    @property
    def h(self):                      # radial height above the pad (same definition as the CSV's h)
        return float(np.linalg.norm(self.P + CENTRE) - RP)

    @property
    def alt(self):
        return self.h + PAD_ALT

    @property
    def up(self):
        return unit(self.P + CENTRE)

    @property
    def speed(self):
        return float(np.linalg.norm(self.v))

    @property
    def air(self):
        return -unit(self.v)

    @property
    def q(self):
        return 0.5 * float(rho_of(self.alt)) * self.speed ** 2

    @property
    def mach(self):
        return self.speed / float(sound_of(self.alt))

    @property
    def aoa(self):
        return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(self.nose, self.air))))))

    @property
    def vu_radial(self):              # the CSV's vu
        return float(np.dot(self.v, self.up))


def state_from_log(d, i):
    return State(float(d["ut"][i]), d["P"][i], d["v"][i], d["nose"][i], float(d["m"][i]))


def _accel(model, P, v, nose, m, thrust, roll):
    Pc = P + CENTRE
    rad = math.sqrt(float(Pc @ Pc))
    a = -MU * Pc / rad ** 3 - 2 * np.cross(OMEGA, v) - np.cross(OMEGA, np.cross(OMEGA, Pc))
    alt = rad - R
    sp = math.sqrt(float(v @ v))
    rho = float(rho_of(alt))
    if rho > 0 and sp > 1.0:
        q = 0.5 * rho * sp * sp
        air = -v / sp
        ca = max(-1.0, min(1.0, float(nose @ air)))
        aoa = math.degrees(math.acos(ca))
        mach = sp / float(sound_of(alt))
        b = nose - ca * air
        sa = math.sqrt(float(b @ b))
        phi = fin_phi(nose, air, Pc / rad, roll) if sa > 1e-6 else 0.0
        a = a + air * (model.cda(mach, aoa, sa, phi) * q / m)
        if sa > 1e-6:
            bh = b / sa
            fa, fp = model.normal(mach, sa, phi)
            a = a + (fa * bh + fp * np.cross(air, bh)) * (q / m)
    if thrust > 0:
        a = a + nose * (thrust / m)
    return a


def simulate(model, s0, t_end, command, throttle=None, avail=None, mass=None, nose_fn=None, roll=None, dt=0.05,
             stop=None, record_every=1, ttp=None):
    """Integrate from State s0 to t_end (RK4, fixed dt; inputs held over a step).

    command(state, t)  -> commanded top-of-stage direction, TRUE pad ENU (like the CSV's dx,dy,dz)
    throttle(state, t) -> 0..1 (None: unpowered)
    avail(t)           -> available thrust N (None: model T_avail(pressure))
    mass(t)            -> mass kg (None: integrate mdot)
    nose_fn(t)         -> override the attitude model with a given top direction (diagnostics)
    roll(t)            -> roll angle deg (kRPC surface roll; sets the grid-fin orientation). None: model.roll
    ttp(state, t)      -> autopilot time_to_peak s (None: 5 s, the fin-craft default). 1.5 s adds the coning
                          limit cycle (model.cone) on top of the mean attitude
    stop(state)        -> True ends the run (e.g. ignition logic, h < 0)
    Returns dict of arrays t, P, v, nose, m, h and 'final' State."""
    t, P, v, nose, m = s0.t, s0.P.copy(), s0.v.copy(), s0.nose.copy(), s0.m
    rec = {k: [] for k in ("t", "P", "v", "nose", "m", "thr")}
    k, n_sat, n_air = 0, 0, 0
    nose_m = nose.copy()                          # mean attitude (the coning limit cycle rides on it)
    model.ttp = TTP_DEFAULT
    while t < t_end - 1e-9:
        h_ = min(dt, t_end - t)
        st = State(t, P, v, nose, m)
        thr = float(throttle(st, t)) if throttle else 0.0
        T = thr * (avail(t) if avail else model.t_avail(st.alt)) if thr > 0 else 0.0
        if k % record_every == 0:
            for key, val in (("t", t), ("P", P.copy()), ("v", v.copy()), ("nose", nose.copy()), ("m", m), ("thr", thr)):
                rec[key].append(val)
        if stop is not None and stop(st):
            break
        # attitude
        if nose_fn is not None:
            nose = unit(nose_fn(t))
        else:
            cmd = unit(np.asarray(command(st, t), float))
            q_ = st.q
            model.ttp = float(ttp(st, t)) if ttp else TTP_DEFAULT
            model._up = st.up
            if thr <= 0.05 and q_ > 2000.0:
                n_air += 1
                n_sat += model.saturated(cmd, st.air, q_)
            tgt = model.att_target(cmd, st.air, q_, thr > 0.05)
            nose_m = model.att_step(nose_m, tgt, h_, q_, thr > 0.05)
            nose = nose_m
            ca = model.cone(q_) if thr <= 0.05 else 0.0
            if ca > 0:
                air = st.air
                u1 = unit(np.cross(air, st.up))
                u2 = np.cross(air, u1)
                ph = 2 * math.pi * (t - s0.t) / model.att15["period"]
                nose = unit(nose_m + math.tan(ca) * (math.cos(ph) * u1 + math.sin(ph) * u2))
        mm = mass(t) if mass else m
        rl = roll(t) if roll else model.roll
        k1v = _accel(model, P, v, nose, mm, T, rl)
        k1p = v
        k2v = _accel(model, P + 0.5 * h_ * k1p, v + 0.5 * h_ * k1v, nose, mm, T, rl)
        k2p = v + 0.5 * h_ * k1v
        k3v = _accel(model, P + 0.5 * h_ * k2p, v + 0.5 * h_ * k2v, nose, mm, T, rl)
        k3p = v + 0.5 * h_ * k2v
        k4v = _accel(model, P + h_ * k3p, v + h_ * k3v, nose, mm, T, rl)
        k4p = v + h_ * k3v
        P = P + h_ / 6 * (k1p + 2 * k2p + 2 * k3p + k4p)
        v = v + h_ / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)
        t += h_
        m = mass(t) if mass else m - thr * model.thrust["mdot"] * h_
        k += 1
    for key, val in (("t", t), ("P", P.copy()), ("v", v.copy()), ("nose", nose.copy()), ("m", m), ("thr", 0.0)):
        rec[key].append(val)
    out = {k_: np.array(v_) for k_, v_ in rec.items()}
    out["h"] = np.linalg.norm(out["P"] + CENTRE, axis=1) - RP
    out["final"] = State(t, P, v, nose, m)
    out["sat_frac"] = n_sat / max(n_air, 1)      # share of the unpowered q > 2 kPa time with a saturated command
    return out


def fly(model, s0, guidance, throttle=None, t_max=600.0, h_stop=0.0, stop=None, dt=0.05, ttp=None):
    """Closed loop: guidance(state, t) -> commanded direction; stops at h <= h_stop, stop(state) or t_max.
    ttp(state, t) -> autopilot time_to_peak (None: 5 s)."""
    def _stop(st):
        return st.h <= h_stop or (stop is not None and stop(st))
    return simulate(model, s0, s0.t + t_max, guidance, throttle=throttle, stop=_stop, dt=dt, ttp=ttp)


def _zoh(t_log, x):
    def f(t):
        i = int(np.clip(np.searchsorted(t_log, t, side="right") - 1, 0, len(t_log) - 1))
        return x[i]
    return f


def replay(model, d, attitude="model", dt=0.05):
    """Open-loop replay of a logged flight from the end of the entry burn to the landing-burn ignition (to 3.9 km
    through the landing burn when it lit above 4 km), driven by the logged command (dx,dy,dz), throttle, available
    thrust, mass and roll angle. attitude='logged' uses the logged top direction instead of the attitude model."""
    i0, i1 = d["i0"], d["i_end"]
    t_log = d["ut"]
    cmd_f, thr_f, av_f = _zoh(t_log, d["cmd"]), _zoh(t_log, d["thr_act"]), _zoh(t_log, d["avail"])
    mass_f = lambda t: float(np.interp(t, t_log, d["m"]))
    nose_f = None
    if attitude == "logged":
        nose_f = lambda t: d["nose"][int(np.clip(np.searchsorted(t_log, t) , 0, len(t_log) - 1))]
    return simulate(model, state_from_log(d, i0), float(t_log[i1]), lambda s, t: cmd_f(t),
                    throttle=lambda s, t: thr_f(t), avail=av_f, mass=mass_f, nose_fn=nose_f, roll=_zoh(t_log, d["roll"]),
                    dt=dt, ttp=(lambda s, t, f=_zoh(t_log, d["ttp"]): f(t)))


# ----------------------------------------------------------------------------------------------- validation
CHECK_H = (20000.0, 10000.0, 4000.0)


def _cross_time(t, h, level):
    j = np.where((h[:-1] >= level) & (h[1:] < level))[0]
    if not len(j):
        return None
    j = j[0]
    return float(t[j] + (t[j + 1] - t[j]) * (h[j] - level) / (h[j] - h[j + 1]))


def _interp_vec(t, T, X):
    return np.array([np.interp(t, T, X[:, k]) for k in range(X.shape[1])])


def compare(d, sim):
    """Errors sim - log. For each check height: at the time the LOG crosses it (de, dn, dh, dspeed) and the
    horizontal miss at equal altitude (sim crossing vs log crossing). At ignition: same time."""
    i0, i1, ie = d["i0"], d["i1"], d["i_end"]
    tl, hl, Pl, vl = d["ut"][i0:ie + 1], d["h"][i0:ie + 1], d["P"][i0:ie + 1], d["v"][i0:ie + 1]
    out = {}
    for lev in CHECK_H + ("ign",):
        if lev == "ign":
            if not d["ign_ok"]:
                continue
            tk = float(d["ut"][i1])
        else:
            tk = _cross_time(tl, hl, lev)
            if tk is None or tk > sim["t"][-1] + 1e-6:
                continue
        Pl_k, vl_k = _interp_vec(tk, tl, Pl), _interp_vec(tk, tl, vl)
        Ps, vs = _interp_vec(tk, sim["t"], sim["P"]), _interp_vec(tk, sim["t"], sim["v"])
        hs = float(np.linalg.norm(Ps + CENTRE) - RP)
        hlk = float(np.linalg.norm(Pl_k + CENTRE) - RP)
        r = dict(de=Ps[0] - Pl_k[0], dn=Ps[1] - Pl_k[1], dh=hs - hlk,
                 dv=float(np.linalg.norm(vs) - np.linalg.norm(vl_k)))
        if lev != "ign":
            ts = _cross_time(sim["t"], sim["h"], lev)
            if ts is not None:
                Pa = _interp_vec(ts, sim["t"], sim["P"])
                r["horiz_at_alt"] = float(np.hypot(Pa[0] - Pl_k[0], Pa[1] - Pl_k[1]))
                r["dt_alt"] = ts - tk
        r["after_gap"] = any(g0 < tk for g0, g1 in d["gaps"])          # inputs unknown inside a logging gap
        r["powered"] = tk > d["ut"][i1] + 1e-6 or (lev == "ign")
        out[lev] = r
    return out


def validate(fl, modes=("logged", "model"), verbose=True, dt=0.05):
    """Leave-one-out: identify on the other flights, replay the held-out one. Returns (results, models)."""
    res = {m_: {} for m_ in modes}
    models = []
    for k, d in enumerate(fl):
        model = identify([x for j, x in enumerate(fl) if j != k])
        models.append(model)
        for att in modes:
            res[att][d["name"]] = compare(d, replay(model, d, attitude=att, dt=dt))
            if verbose:
                print("[%s] " % att[:3], end="")
                _print_row(d, res[att][d["name"]])
    return res, models


def param_spread(models):
    """Mean and std over the leave-one-out models of the headline parameters (identification uncertainty)."""
    def stat(f):
        x = np.array([f(m_) for m_ in models], float)
        return float(np.mean(x)), float(np.std(x))
    out = {}
    for M in (0.6, 1.0, 1.6, 2.5, 3.5, 4.4):
        for A in (0.0, 3.0, 10.0):
            out["CdA M%.1f a%02d" % (M, A)] = stat(lambda m_: m_.cda(M, A))
    for M in (1.0, 2.5, 4.4):
        for A in (3.0, 10.0):
            sa = math.sin(math.radians(A))
            out["CN.A along M%.1f a%02d (m^2, per q sin)" % (M, A)] = stat(lambda m_: m_.normal(M, sa)[0] / sa)
    for i, k in enumerate(("fin_along_sin4", "fin_along_cos4", "fin_perp_sin4", "fin_perp_cos4")):
        out[k] = stat(lambda m_: m_.fin[i])
    for i, k in enumerate(("dCdA_cos4", "dCdA_sin4")):
        out[k] = stat(lambda m_: m_.dfin[i])
    for k in models[0].att:
        out["att_" + k] = stat(lambda m_: m_.att[k])
    return out


def _print_row(d, r, model=None):
    s = "%s  start h %5.0f m  ign h %5.0f m%s |" % (d["name"], d["h"][d["i0"]], d["h"][d["i1"]],
                                                  "" if not d["gaps"] else " gap")
    for lev in CHECK_H + ("ign",):
        x = r.get(lev)
        if x is None:
            s += "  %5s   n/a              |" % (("%dk" % (lev / 1000)) if lev != "ign" else "ign")
            continue
        hz = x.get("horiz_at_alt", float("nan"))
        flag = ("g" if x["after_gap"] else "") + ("*" if x["powered"] and lev != "ign" else "")
        s += "  %5s dE%+6.0f dN%+5.0f dh%+5.0f dV%+5.1f H@alt %5.0f%-2s|" % (
            ("%dk" % (lev / 1000)) if lev != "ign" else "ign", x["de"], x["dn"], x["dh"], x["dv"], hz, flag)
    print(s, flush=True)


# identified on the 12 GF0T flights of 2026-10-20 incl. the DR-003/004 identification flights 224739 / 230136
# (tools/replay_sim.py identify); refresh after new flights
DEFAULT_MODEL = json.loads("""
{"mach": [0.3, 0.6, 0.85, 1.0, 1.15, 1.35, 1.6, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.2], "aoa": [0.0, 1.5, 3.0, 5.0,
8.0, 11.0, 15.0, 20.0, 30.0], "cda": [[14.832, 14.911, 15.114, 15.573, 16.123, 16.145, 16.018, 16.186, 16.444],
[13.689, 13.879, 13.973, 14.033, 14.631, 15.211, 15.643, 15.978, 16.328], [12.422, 12.84, 13.13, 12.947, 12.597,
13.591, 14.927, 15.656, 16.122], [11.003, 11.624, 12.374, 13.489, 14.411, 14.294, 14.693, 15.227, 15.723],
[9.534, 10.281, 11.039, 11.915, 13.806, 14.368, 14.459, 14.781, 15.289], [7.983, 8.754, 9.22, 10.253, 12.158,
13.096, 13.54, 14.193, 15.02], [6.311, 6.971, 7.037, 7.914, 9.273, 10.492, 11.991, 13.484, 14.925], [4.879,
5.273, 4.981, 5.579, 6.565, 8.164, 10.603, 12.814, 14.843], [3.698, 4.048, 4.577, 5.199, 5.929, 7.821, 9.93,
12.245, 14.662], [3.194, 3.713, 4.224, 4.726, 5.716, 7.904, 9.41, 11.706, 14.452], [3.244, 3.403, 3.75, 4.577,
6.49, 7.712, 8.574, 11.12, 14.18], [3.859, 3.888, 4.503, 5.244, 6.618, 7.569, 8.764, 10.523, 13.366], [3.6,
4.079, 4.499, 5.275, 6.41, 7.395, 8.339, 9.465, 10.97], [3.456, 4.098, 4.687, 5.405, 6.249, 7.105, 7.813, 8.329,
8.709]], "dfin": [-8.195, 5.033], "lift_mach": [0.5, 1.0, 1.5, 2.5, 3.5, 4.5], "lift": [[4.94, 3.774, 2.584,
2.267, 2.344, 2.007], [-89.086, -79.977, -70.894, -61.762, -52.746, -43.955], [0.004, -0.057, 0.078, 0.116,
0.104, 0.038]], "fin": [0.231, -14.683, 15.691, -0.471], "att": {"tau": 1.903, "G0": 1.044, "qg": 153.6655,
"s_inf": 0.161, "s0": 0.3924, "wmax": 4.154, "qw": 46.6846, "bmax": 3.0722, "qb": 177.8062}, "att15": {"ttp":
1.5, "G": 0.5366, "bias": 0.2653, "tau": 0.9391, "wmax": 20.0, "cone": 5.0784, "period": 1.6, "cone_q0": 40.0,
"cone_q1": 87.6198, "q_lo": 87.6198, "q_hi": 119.7204}, "thrust": {"T_vac": 650295.1131, "k_p": 0.7919, "mdot":
206.9}, "roll": 96.6}
""")


def default_model():
    return Model.from_dict(DEFAULT_MODEL)


def main(argv):
    import argparse
    ap = argparse.ArgumentParser(prog="replay_sim.py", description="Offline replay simulator for the kspilot booster "
                                 "descent (no game connection); needs the GF0T landing CSVs (see the module docstring).")
    ap.add_argument("command", nargs="?", default="validate", choices=("validate", "identify", "replay"),
                    help="validate (default): leave-one-out identification and replay of every flight; identify: "
                         "print the model identified on all flights; replay LANDING.csv: replay one flight")
    ap.add_argument("arg", nargs="?", help="validate: output JSON file; replay: the landing CSV")
    ap.add_argument("--logs", metavar="DIR", help="directory with the landing CSVs (else KSP_REPLAY_LOGS, else the "
                    "repository's data/raw/logs, logs and tests/data)")
    a = ap.parse_args(argv[1:])
    if a.logs:
        os.environ["KSP_REPLAY_LOGS"] = a.logs        # also seen by multiprocessing workers
    try:
        if a.command == "replay":
            if not a.arg:
                ap.error("replay needs LANDING.csv")
            d = load(find_log(a.arg))
            others = [x for x in flights() if x["name"] != d["name"]]
            if not others:
                raise need_data("at least one more GF0T landing CSV besides %s (the model is identified on the "
                                "other flights)" % os.path.basename(a.arg))
            m = identify(others)
            _print_row(d, compare(d, replay(m, d)))
            return 0
        fl = flights()
        if len(fl) < (1 if a.command == "identify" else 2):
            raise need_data("at least %d GF0T landing CSV%s (*%s), %d found" % (
                1 if a.command == "identify" else 2, "" if a.command == "identify" else "s", GF0T_SUFFIX,
                len(fl)))
        if a.command == "identify":
            m = identify(fl, verbose=True)
            print(json.dumps(m.to_dict(), indent=1))
        else:
            res, models = validate(fl)
            print("== identification spread over the leave-one-out fits (mean, std)")
            for k, (mu, sd) in param_spread(models).items():
                print("  %-42s %9.4g  +- %.2g" % (k, mu, sd))
            if a.arg:
                json.dump(res, open(a.arg, "w"), indent=1, default=float)
    except NeedData as e:
        print("replay_sim.py: %s" % e, file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
