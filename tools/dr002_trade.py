#!/usr/bin/env python3
"""DR-002 trade study (offline, no game contact): recovery architecture for the fins-only GF0T core.

Designs (identical dispersions per case index = common random numbers; see DESIGNS):
  A1   4 fins, pulses at 20 + 12 km (as briefed), aimed long so pulses only shorten
  A1e  4 fins, vectored pulses at 38.5 + 12 + 5 km (first one where 1 m/s buys ~4x the range of 20 km and
       the envelope allows a 30 deg tilt for cross-track; the 5 km trim cuts the 12 km prediction error)
  A2   4 fins, fin-fold drag modulation (PWM, 4 s period) above 15 km + one early vectored pulse (38.5 km)
  A3   4 fins, aoa drag modulation 1-8 deg above 5 km (fins always out) + one early pulse   [RE proposal]
  B1   8 fins (CdA x1.64, +1.6 t), pulses at 20 + 12 km (as briefed)
  B1e  8 fins, pulses as A1e
  B2   8 fins, fin-fold modulation above 15 km, no pulse (as briefed: the airbrake craft's scheme)
  B2p  8 fins, fin-fold modulation + one early pulse
Shared by all: the aero channel (fins out only) flies a fixed aoa (5 deg; A3: the modulated aoa) and steers
the LIFT DIRECTION (tilt azimuth) like a bank angle: cross-track while a pulse or the modulator still owns the
along-track axis, both axes after that. The azimuth is picked with the identified normal-force model including
the 4phi grid-fin terms at the measured roll (below ~4 deg they, not the tilt, set the force direction).
Why a fixed aoa: lift/extra-drag of a tilt is only ~0.2 (5 deg, M 3.5: normal 0.27 m^2 vs +1.5 m^2 CdA), so
an on/off "tilt when needed" cross channel shortens the range ~4-5 m per m of cross-track and wrecks the
along-track prediction; a constant aoa makes the drag predictable and the azimuth costs nothing.
Same ignition rule and landing-burn law for all.

Truth = tools/replay_sim.py identified model (4 fins out; validated 85-335 m at 4 km unsaturated) plus:
  fin configuration factor on CdA and normal force: folded 0.36+-0.03 (AERO_THEORY s7 static), 8 fins
  1.64+-0.05; fins extend/fold at 1/s; folded = no steering and tau x2 (assumption, never flown).
  dispersions: drag N(1,5%), Mach-shape drag error 1+s(M-2.5)/2.5, s N(0,4%); lift N(1,15%); tau N(1,20%);
  roll U(0,360) (known to the controller, not controlled); aim error U(+-3 km along, +-1 km cross) at 39 km;
  30 m / 1 m/s state noise.
On-board predictor (what the flight code would run): fast point mass with the nominal CdA(M) table at the
commanded aoa, scaled by the drag ratio MEASURED in flight (actual drag / table at the command, 2 s low-pass,
+N(0,2%) bias) separately for fins-out and fins-folded; it does not know lift, tau or the Mach-shape error.

Miss = touchdown distance of an UNDIVERTED retrograde landing burn from the actual ignition state = the error
the landing-burn divert (~17 m/deg) must remove. Landing-burn fuel x1.5 = calibrated to the real burns
(184942: 131 vs 92 u, 190909: 136 vs 82 u from the same ignition states).

Data: the GF0T landing CSVs named in BASES (run*) and, for 'check', also the two flights in CHECK_FLIGHTS; looked up
in --logs DIR, else KSP_REPLAY_LOGS (an explicit directory is the only place searched), else data/raw/logs, logs/
and tests/data of the repository. When one is missing the tool prints one line naming it and exits with status 2.

usage: dr002_trade.py [--logs DIR] COMMAND ...
  dr002_trade.py check                 predictor vs truth, landing-burn fuel vs logged flights
  dr002_trade.py run [N] [out.json] [A1,A1e,...]   v1 Monte Carlo (default N=200, all v1 designs)
  dr002_trade.py run2 [N] [out.json]                v2 rerun (measured high-q limit, realistic dispersions): paired
                                                    ranking vs LITE5 with bootstrap 95 % intervals
  dr002_trade.py run3 [N] [out.json]                v3 (DR-005): refit time_to_peak-dependent attitude truth, A1e and
                                                    LITE5L with time_to_peak 1.5 s in the IDENT-style window (+ A1e
                                                    at 5 s), paired vs LITE5L15
"""
import json
import math
import multiprocessing as mp
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import replay_sim as rs  # noqa: E402

KG_PER_LF = 11.11            # LF + Ox per LiquidFuel unit (5 kg/u, 0.9:1.1); dry 11.765 t on 184942 and 203827
DRY4 = 11765.0               # GF0T core dry mass, kg (m_end - fuel_left x 11.11; two flights agree to 3 kg)
M_REC0 = 21720.0             # core mass at the start of the recovery (deorbit), fuel 896 units
FIN_KG = 400.0
FOLD_NOM, EIGHT_NOM = 0.36, 1.64
PAYLOAD_NOW = 2.4            # t (PAYLOAD_THEORY.md)
BASES = ("20261020_203827", "20261020_194753")
CHECK_FLIGHTS = ("20261020_203827", "20261020_184942", "20261020_190909")   # check(): logged ignition states
PWM = 4.0                    # s, modulation decision period (fold duty cycle)
ALPHA = 5.0                  # deg, aero-channel aoa (identified envelope ~9 deg at high q)
# v2 (red team review): the reachable aoa collapses at high q (6 deg at 100 kPa, 2.3 deg at 145 kPa, measured on
# one C-mode flight; REPLAY_SIM failure #2). Truth: this limit x N(1, 15 %) on top of the identified envelope, and
# sat_frac counts commands beyond it. The controller knows the nominal limit and commands at most 90 % of it.
HQ_Q = [60.0, 100.0, 145.0, 200.0]          # kPa
HQ_A = [9.6, 6.0, 2.3, 1.0]                 # deg (60 kPa = the identified envelope; 200 kPa extrapolated)
HQ_MARGIN = 0.9
FIN_RATE = 1.0               # fin extension per s (1 s full travel; assumption)
BURN_CAL = 1.5               # real landing burn / our idealised retro burn from the same ignition state
AZ_RATE = math.radians(15.0)  # lift-direction change per 1 s update
Q_LOGGED = 148.0             # kPa, highest q in the 10 logged GF0T flights (model data range)

# pulses: (height, along-track target m). aim: along-track touchdown (m, + = long) of the undispersed nominal
# flight; "auto" = 3000 - authority/2 + 300 m at f0, so a -3 km aim error stays inside the modulator's reach and
# the early pulse removes everything long.
# pulse = (trigger height m, along-track target m, max tilt off retrograde deg, "raise"|"clip"). Pulses are
# vectored (retro + lateral): with 5 deg of aoa the lift moves the touchdown only ~0.3-1 km sideways over the
# whole descent (direction set by the uncontrolled roll), so a +-1 km cross-track error needs a burn. The tilt
# cap keeps the turn inside the identified envelope (sin aoa_max = 0.16 + 0.39/q[kPa]: 33 deg at 1 kPa, 39 km;
# ~10 deg at 20 km). "raise": the along part is raised to reach the cross demand (early pulse, aimed long);
# "clip": the cross part is clipped.
DESIGNS = {
    "A1": dict(fins=4, mod=None, pulses=((20000.0, 200.0, 8.0, "clip"), (12000.0, 0.0, 8.0, "clip")), aim=3400.0),
    "A1e": dict(fins=4, mod=None, pulses=((38500.0, 300.0, 30.0, "raise"), (12000.0, 150.0, 8.0, "clip"),
                                          (5000.0, 0.0, 8.0, "clip")), aim=3600.0),
    "A2": dict(fins=4, mod="fold", floor=15000.0, pulses=((38500.0, 0.0, 30.0, "raise"),), aim="auto"),
    "A3": dict(fins=4, mod="aoa", floor=5000.0, amin=1.0, amax=8.0, pulses=((38500.0, 0.0, 30.0, "raise"),),
               aim="auto"),
    "B1": dict(fins=8, mod=None, pulses=((20000.0, 200.0, 8.0, "clip"), (12000.0, 0.0, 8.0, "clip")), aim=3400.0),
    "B1e": dict(fins=8, mod=None, pulses=((38500.0, 300.0, 30.0, "raise"), (12000.0, 150.0, 8.0, "clip"),
                                          (5000.0, 0.0, 8.0, "clip")), aim=3600.0),
    "B2": dict(fins=8, mod="fold", floor=15000.0, pulses=(), aim=0.0),
    "B2p": dict(fins=8, mod="fold", floor=15000.0, pulses=((38500.0, 0.0, 30.0, "raise"),), aim="auto"),
    # ---- v2 (DR-002 rerun after the red team): realistic dispersions, measured high-q limit in truth and sat_frac.
    # LITE*: no relight and no 12 km pulse. The entry-burn cutoff is closed on the predictor (prior drag table, aim
    # +400 m long): modelled as a signed retro delta-v at the logged cutoff state (cut later = shorter); one 5 km
    # vectored trim (<= 8 deg, "clip") then landing-burn divert. A1eR = A1e unchanged, under the v2 rules.
    "A1eR": dict(fins=4, mod=None, v2=True, pulses=((38500.0, 300.0, 30.0, "raise"), (12000.0, 150.0, 8.0, "clip"),
                                                    (5000.0, 0.0, 8.0, "clip")), aim=3600.0),
    "LITE5": dict(fins=4, mod=None, v2=True, alpha=5.0, cutoff=400.0, pulses=((5000.0, 0.0, 8.0, "clip"),), aim=400.0),
    "LITE3": dict(fins=4, mod=None, v2=True, alpha=3.0, cutoff=400.0, pulses=((5000.0, 0.0, 8.0, "clip"),), aim=400.0),
    "LITE0": dict(fins=4, mod=None, v2=True, alpha=0.0, cutoff=400.0, pulses=((5000.0, 0.0, 8.0, "clip"),), aim=400.0),
}
# RE additions after the first v2 run (the 5 km trim alone cannot absorb the prior-drag cutoff error):
DESIGNS["LITE5m"] = dict(fins=4, mod=None, v2=True, alpha=5.0, cutoff=400.0, aim=400.0,
                         pulses=((12000.0, 150.0, 8.0, "clip"), (5000.0, 0.0, 8.0, "clip")))
DESIGNS["LITE5L"] = dict(fins=4, mod=None, v2=True, alpha=5.0, cutoff=1200.0, aim=1200.0,
                         pulses=((12000.0, 150.0, 8.0, "clip"), (5000.0, 0.0, 8.0, "clip")))
# ---- v3 (DR-005, after the DR-003/004 identification flights): truth = refit attitude model as a function of the
# autopilot time_to_peak (REPLAY_SIM "time_to_peak"): 5 s = soft (gain, trim tilt, lag); 1.5 s = stiff (mean-tilt
# gain 0.54 + a 5 deg coning limit cycle, measured at 88-120 kPa only). No measured high-q limit (DR-003: a controller
# artefact). ttp15: time_to_peak 1.5 s in the IDENT-style window (coasting, above TTP_FLOOR), 5 s elsewhere.
TTP_FLOOR = 6000.0
DESIGNS["A1e15"] = dict(DESIGNS["A1eR"], v3=True, ttp15=True)
DESIGNS["LITE5L15"] = dict(DESIGNS["LITE5L"], v3=True, ttp15=True)
DESIGNS["A1e5"] = dict(DESIGNS["A1eR"], v3=True, ttp15=False)
V3 = ("A1e15", "LITE5L15", "A1e5")
V1 = ("A1", "A1e", "A2", "A3", "B1", "B1e", "B2", "B2p")
V2 = ("LITE5", "A1eR", "LITE0", "LITE3", "LITE5m", "LITE5L")

# ------------------------------------------------------------------------------------- fast predictor
_NOM = rs.default_model()
_TM = [float(x) for x in _NOM.MACH]
_AL = np.arange(0.0, 10.01, 0.5)
_TABS = [[_NOM.cda(M, a, math.sin(math.radians(a))) for M in _TM] for a in _AL]   # CdA(M) per aoa, 4 fins out
_DA = 50.0
_AG = np.arange(0.0, 72000.0, _DA)
_RHO = [float(x) for x in rs.rho_of(_AG)]
_SND = [float(x) for x in rs.sound_of(_AG)]
_PRS = [float(x) for x in rs.pres_of(_AG)]
MU, RP, R = rs.MU, rs.RP, rs.R
OY, OZ = float(rs.OMEGA[1]), float(rs.OMEGA[2])
T_VAC, K_P, MDOT = _NOM.thrust["T_vac"], _NOM.thrust["k_p"], _NOM.thrust["mdot"]


_G0, _QG = _NOM.att["G0"], _NOM.att["qg"]


def hq_lim(q):
    """Measured reachable aoa (deg) at dynamic pressure q (Pa); 90 below 60 kPa (the envelope rules there)."""
    qk = q / 1e3
    return 90.0 if qk < HQ_Q[0] else float(np.interp(qk, HQ_Q, HQ_A))


def att_gain(q):
    return _G0 / (1.0 + q / 1e3 / _QG)


_PRED = dict(v3=False, ttp15=False)       # predictor/controller mode of the case being flown (set per case)


def alpha_ach(a_cmd, q, h=None):
    """Achieved aoa for a commanded aoa (identified gain G0/(1+q/qg), REPLAY_SIM attitude response), capped by
    the controller's own high-q limit (HQ_MARGIN x the measured limit).
    v3: the drag-effective aoa of the refit model at the time_to_peak flown at that height: |mean tilt (gain x
    command, plus the soft autopilot's trim tilt)| combined with the coning amplitude (stiff autopilot)."""
    if _PRED["v3"]:
        _NOM.ttp = 1.5 if (_PRED["ttp15"] and h is not None and h > TTP_FLOOR) else rs.TTP_DEFAULT
        g, c = _NOM.gain(q), math.degrees(_NOM.cone(q))
        tr = math.degrees(math.asin(min(_NOM.trim(q), 0.99)))
        return math.sqrt((a_cmd * g) ** 2 + tr * tr + c * c)
    return min(a_cmd * att_gain(q), HQ_MARGIN * hq_lim(q))


def cda_nom(M, a):
    """Nominal 4-fin-out CdA at Mach M and aoa a (deg), from the aoa-indexed tables."""
    x = min(max(a, 0.0), 9.99) / 0.5
    i = int(x)
    f = x - i
    return _cda(_TABS[i], M) * (1 - f) + _cda(_TABS[i + 1], M) * f


def cda_cfg(cfg, M, q, h=None):
    """Predictor CdA for a configuration (a_cmd deg, k fins-out, k folded, fins-out fraction)."""
    a_cmd, ka, k0, w = cfg
    c = w * ka * cda_nom(M, alpha_ach(a_cmd, q, h))
    if w < 1.0:
        c += (1.0 - w) * k0 * _cda(_TABS[0], M)
    return c


def _lk(t_, alt):
    x = alt / _DA
    if x <= 0:
        return t_[0]
    i = int(x)
    if i >= len(t_) - 1:
        return t_[-1]
    f = x - i
    return t_[i] * (1 - f) + t_[i + 1] * f


def _cda(t_, M):
    if M <= _TM[0]:
        return t_[0]
    if M >= _TM[-1]:
        return t_[-1]
    i = 0
    while _TM[i + 1] < M:
        i += 1
    f = (M - _TM[i]) / (_TM[i + 1] - _TM[i])
    return t_[i] * (1 - f) + t_[i + 1] * f


def ign_rule(h, sp, sin_g, m, T, a_drag, g):
    """Shared ignition rule: light when the along-path stopping drop at 85 % thrust (half the current drag
    credited) reaches the height, with 60 m margin."""
    if sin_g <= 0.05:
        return False
    a_dec = 0.85 * T / m + 0.5 * a_drag - g * sin_g
    return h <= sp * sp * sin_g / (2 * max(a_dec, 1.0)) + 60.0


def burn_throttle(h, sp, sin_g, m, T, a_drag, g):
    """Shared landing-burn law (retrograde, gravity turn): constant along-path decel to stop at the ground."""
    a_req = sp * sp * max(sin_g, 0.05) / (2 * max(h - 3.0, 1.0))
    return min(max((a_req + g * sin_g - a_drag) / (T / m), 0.0), 1.0)


def predict(P, v, m, cfg_hi, cfg_lo, h_floor=0.0, dv=0.0, dt=0.5, dv_side=None):
    """Point mass to touchdown (coast -> ignition rule -> retro burn). cfg_hi/cfg_lo: drag configuration
    (see cda_cfg) above/below h_floor. dv: impulsive retro pulse now (m/s). Returns (e, n, t_ign, burn_kg, pulse_kg)."""
    px, py, pz = float(P[0]), float(P[1]), float(P[2])
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    pulse_kg = 0.0
    side = 0.0
    if dv_side is not None:                       # lateral impulse (3-vector, m/s)
        vx, vy, vz = vx + dv_side[0], vy + dv_side[1], vz + dv_side[2]
        side = float(np.linalg.norm(dv_side))
    if dv > 0:
        sp = math.sqrt(vx * vx + vy * vy + vz * vz)
        k = max(sp - dv, 1.0) / sp
        vx, vy, vz = vx * k, vy * k, vz * k
    if dv > 0 or side > 0:
        pulse_kg = m * (1 - math.exp(-math.hypot(dv, side) / (T_VAC / MDOT)))
        m -= pulse_kg
    t, burning, t_ign, m_ign = 0.0, False, None, m

    def acc(px, py, pz, vx, vy, vz, m, burning):
        cz = pz + RP
        r2 = px * px + py * py + cz * cz
        r = math.sqrt(r2)
        k3 = MU / (r2 * r)
        ax = -px * k3 - 2 * (OY * vz - OZ * vy) + (OY * OY + OZ * OZ) * px
        ay = -py * k3 - 2 * (OZ * vx) - OZ * (OY * cz - OZ * py)
        az = -cz * k3 + 2 * (OY * vx) + OY * (OY * cz - OZ * py)
        alt = r - R
        sp = math.sqrt(vx * vx + vy * vy + vz * vz)
        rho = _lk(_RHO, alt)
        ad = 0.0
        if rho > 0 and sp > 1:
            q = 0.5 * rho * sp * sp
            ad = cda_cfg(cfg_hi if (r - RP) > h_floor else cfg_lo, sp / _lk(_SND, alt), q, r - RP) * q / m
        at = thr = 0.0
        if burning:
            T = T_VAC - K_P * _lk(_PRS, alt)
            sg = -(px * vx + py * vy + cz * vz) / (r * sp)
            thr = burn_throttle(r - RP, sp, sg, m, T, ad, MU / r2)
            at = thr * T / m
        a = (ad + at) / max(sp, 1e-6)
        return ax - a * vx, ay - a * vy, az - a * vz, thr

    while t < 900:
        cz = pz + RP
        r = math.sqrt(px * px + py * py + cz * cz)
        h = r - RP
        sp = math.sqrt(vx * vx + vy * vy + vz * vz)
        if burning and (sp < 3.0 or h <= 1.0):
            break
        if not burning and h < 0:
            break
        if not burning and h < 9000:
            alt = r - R
            sg = -(px * vx + py * vy + cz * vz) / (r * sp)
            T = T_VAC - K_P * _lk(_PRS, alt)
            q = 0.5 * _lk(_RHO, alt) * sp * sp
            ad = cda_cfg(cfg_hi if h > h_floor else cfg_lo, sp / _lk(_SND, alt), q, h) * q / m
            if ign_rule(h, sp, sg, m, T, ad, MU / (r * r)):
                burning, t_ign, m_ign = True, t, m
        h_ = dt if not burning else 0.1
        a1x, a1y, a1z, _ = acc(px, py, pz, vx, vy, vz, m, burning)
        mx, my, mz = px + 0.5 * h_ * vx, py + 0.5 * h_ * vy, pz + 0.5 * h_ * vz
        wx, wy, wz = vx + 0.5 * h_ * a1x, vy + 0.5 * h_ * a1y, vz + 0.5 * h_ * a1z
        a2x, a2y, a2z, thr2 = acc(mx, my, mz, wx, wy, wz, m, burning)
        px, py, pz = px + h_ * wx, py + h_ * wy, pz + h_ * wz
        vx, vy, vz = vx + h_ * a2x, vy + h_ * a2y, vz + h_ * a2z
        if burning:
            m -= thr2 * MDOT * h_                      # mdot = throttle x 206.9 kg/s (as the truth)
        t += h_
    return px, py, (t_ign if t_ign is not None else t), (m_ign - m) if burning else 0.0, pulse_kg


# ------------------------------------------------------------------------------------- truth model
class Vehicle(rs.Model):
    """Identified 4-fin model + fin count / extension / Mach-shape drag error for one MC case."""

    def setup(self, fins, fold, eight, mslope, hq=None):
        self.fins, self.fold, self.eight, self.mslope, self.ext = fins, fold, eight, mslope, 1.0
        self.tau0 = self.att["tau"]
        self.hq = hq                        # None: v1 truth; else scale on the measured high-q aoa limit
        return self

    def _smax(self, q):
        p = self.att
        s_ = min(p["s_inf"] + p["s0"] / max(q / 1e3, 0.05), 0.999)
        if self.hq:
            s_ = min(s_, math.sin(math.radians(min(hq_lim(q) * self.hq, 89.0))))
        return s_

    def att_target(self, cmd, air, q, thrusting):
        if not self.hq:
            return rs.Model.att_target(self, cmd, air, q, thrusting)
        if thrusting:
            return cmd
        p = self.att
        b = cmd - np.dot(cmd, air) * air
        b = b * p["G0"] / (1.0 + q / 1e3 / p["qg"])
        nb = np.linalg.norm(b)
        smax = self._smax(q)
        if nb > smax:
            b = b * (smax / nb)
        return air * math.sqrt(max(1.0 - float(np.dot(b, b)), 0.0)) + b

    def saturated(self, cmd, air, q):
        if not self.hq:
            return rs.Model.saturated(self, cmd, air, q)
        p = self.att
        b = cmd - np.dot(cmd, air) * air
        g = np.linalg.norm(b) * p["G0"] / (1.0 + q / 1e3 / p["qg"])
        env = min(p["s_inf"] + p["s0"] / max(q / 1e3, 0.05), 0.999)
        if g > env + 0.02:
            return True
        return bool(self.hq) and g > math.sin(math.radians(min(hq_lim(q) * self.hq, 89.0))) + 0.01

    def cfg(self, ext=None):
        e = self.ext if ext is None else ext
        out = 1.0 if self.fins == 4 else self.eight
        return self.fold + (out - self.fold) * e

    def mfac(self, mach):
        return 1.0 + self.mslope * (mach - 2.5) / 2.5

    def cda(self, mach, aoa_deg, sin_a=0.0, phi=0.0):
        return rs.Model.cda(self, mach, aoa_deg, sin_a, phi) * self.cfg() * self.mfac(mach)

    def normal(self, mach, sin_a, phi=0.0):
        fa, fp = rs.Model.normal(self, mach, sin_a, phi)
        c = self.cfg()                      # assumption: normal force scales with the fin configuration like drag
        return fa * c, fp * c


def rotate_state(s, d):
    """Rotate a State about Kerbin's centre so that points at the pad move by the horizontal vector d (m)."""
    dn = math.hypot(d[0], d[1])
    if dn < 1e-6:
        return rs.State(s.t, s.P.copy(), s.v.copy(), s.nose.copy(), s.m)
    k = np.array([-d[1], d[0], 0.0]) / dn          # z x d_hat
    a = dn / RP

    def rot(x):
        return x * math.cos(a) + np.cross(k, x) * math.sin(a) + k * np.dot(k, x) * (1 - math.cos(a))
    return rs.State(s.t, rot(s.P + rs.CENTRE) - rs.CENTRE, rot(s.v), rot(s.nose), s.m)


def unit3(x):
    return x / np.linalg.norm(x)


def unit_h(v):
    a = np.array([v[0], v[1]])
    return a / np.linalg.norm(a)


# ------------------------------------------------------------------------------------- controller
class Controller:
    def __init__(self, design, veh, eps, a_hat, f0=None, freeze=False):
        self.D, self.veh, self.eps, self.a_hat, self.f0, self.freeze = design, veh, eps, a_hat, f0, freeze
        self.out_nom = 1.0 if design["fins"] == 4 else EIGHT_NOM
        self.k_out, self.k_fold = self.out_nom, FOLD_NOM     # measured drag ratios (fins out / folded)
        self.mod, self.floor = design.get("mod"), design.get("floor", 0.0)
        self.alpha0 = design.get("alpha", ALPHA)
        self.phase, self.cmd = "COAST", None
        self.t_last, self.t_pred, self.pred = None, -1e9, None
        self.f, self.t_pwm, self.ext_cmd = (1.0 if f0 is None else f0), -1e9, 1.0
        self.pulses = [] if freeze else list(design["pulses"])
        self.dv_goal = self.dv_done = 0.0
        self.p_tilt, self.p_side, self.p_t0, self.p_wait = 0.0, np.zeros(3), 0.0, True
        self.switches, self.ign = 0, None
        self.t_x, self.az, self.f_in, self.i_in = -1e9, 0.0, 0.0, 0.0
        self.log, self.hist = [], []

    # --- modulation variable f -> aoa / tables
    def alpha(self, st_h, f=None):
        if self.mod == "aoa" and st_h > self.floor:
            f = self.f if f is None else f
            return self.D["amin"] + (self.D["amax"] - self.D["amin"]) * f
        return self.alpha0

    def tables(self, f):
        lo = (self.alpha0, self.k_out, self.k_fold, 1.0)
        if self.mod is None or f is None:
            return lo, lo
        if self.mod == "fold":
            return (self.alpha0, self.k_out, self.k_fold, f), lo
        return (self.D["amin"] + (self.D["amax"] - self.D["amin"]) * f, self.k_out, self.k_fold, 1.0), lo

    def _p(self, st, f=None, dv=0.0, dv_side=None):
        hi, lo = self.tables(f)
        return predict(st.P, st.v, st.m, hi, lo, self.floor, dv=dv, dv_side=dv_side)

    def _along(self, r):
        return r[0] * self.a_hat[0] + r[1] * self.a_hat[1]

    def _f_now(self, st):
        return self.f if (self.mod and st.h > self.floor) else None

    # --- main loop (called first in every simulator step; guidance() returns the command set here)
    def throttle(self, st, t):
        dt = 0.0 if self.t_last is None else t - self.t_last
        self.t_last = t
        veh = self.veh
        old = veh.ext
        veh.ext += max(-FIN_RATE * dt, min(FIN_RATE * dt, self.ext_cmd - veh.ext))
        if (old >= 0.5) != (veh.ext >= 0.5):
            self.switches += 1
        veh.att["tau"] = veh.tau0 * (2.0 - veh.ext)          # folded: weaker weathervane (assumption)
        sp, air, up = st.speed, st.air, st.up
        if self.phase == "DONE":
            self.cmd = air
            return 0.0
        a_cmd = self.alpha(st.h) if veh.ext > 0.9 else 0.0
        if not self.D.get("v3"):
            a_cmd = min(a_cmd, HQ_MARGIN * hq_lim(st.q) / att_gain(st.q))     # never ask past the limit
        # drag-ratio measurement: actual drag / (q x nominal CdA at the actual aoa and fin phase); the flight code
        # knows both (attitude vs velocity, roll). The predictor applies it at the aoa it expects to fly.
        if sp > 50 and st.q > 500 and dt > 0 and self.phase == "COAST":
            sa = math.sin(math.radians(st.aoa))
            phi = rs.fin_phi(st.nose, air, up, veh.roll) if sa > 1e-6 else 0.0
            w = 1 - math.exp(-dt / 2.0)
            if veh.ext > 0.999:
                k = veh.cda(st.mach, st.aoa, sa, phi) / _NOM.cda(st.mach, st.aoa, sa, phi) * (1 + self.eps)
                self.k_out += w * (k - self.k_out)
            elif veh.ext < 0.001:
                k = veh.cda(st.mach, st.aoa, sa, phi) / _NOM.cda(st.mach, st.aoa, sa, phi) * (1 + self.eps)
                self.k_fold += w * (k - self.k_fold)
        T = veh.t_avail(st.alt)
        r = float(np.linalg.norm(st.P + rs.CENTRE))
        sg = -float(np.dot(st.v, up)) / sp
        ad = veh.cda(st.mach, st.aoa) * st.q / st.m
        if self.phase == "BURN":
            self.cmd = air
            if sp < 3.0 or st.h <= 1.0:
                self.phase = "DONE"
                return 0.0
            return burn_throttle(st.h, sp, sg, st.m, T, ad, MU / r ** 2)
        if self.phase == "PULSE":
            # vectored pulse: point the nose (retro + lateral), light within 3 deg (like the vernier burn in
            # landing.py) or after 8 s, throttle 1 until the delta-v is in
            th = self.p_tilt
            self.cmd = air * math.cos(th) + self.p_side * math.sin(th)
            on = (float(np.dot(st.nose, self.cmd)) > math.cos(math.radians(3.0)) or t - self.p_t0 > 8.0
                  or self.dv_done > 0 or not self.p_wait)   # light within 3 deg (or after 8 s); then burn to the end
            if self.dv_done >= self.dv_goal or t - self.p_t0 > 40.0:
                self.phase = "COAST"
                self.log.append(("pulse_done", round(st.h), round(self.dv_done, 1)))
                return 0.0
            if on:
                self.dv_done += T / st.m * dt
                return 1.0
            return 0.0
        # ---- COAST: modulation (fold duty or aoa), decided every PWM s by bisection on the predicted miss
        if self.mod and st.h > self.floor and not self.freeze:
            if t - self.t_pwm >= PWM:
                self.t_pwm = t
                if self._along(self._p(st, 0.0)) <= 0.0:
                    self.f = 0.0
                elif self._along(self._p(st, 1.0)) >= 0.0:
                    self.f = 1.0
                else:
                    lo, hi = 0.0, 1.0
                    for _ in range(6):
                        mid = 0.5 * (lo + hi)
                        if self._along(self._p(st, mid)) > 0.0:
                            lo = mid
                        else:
                            hi = mid
                    self.f = 0.5 * (lo + hi)
                self.log.append((round(st.h), round(self.f, 2)))
        if self.mod == "fold" and st.h > self.floor:
            self.ext_cmd = 1.0 if (t - self.t_pwm) % PWM < self.f * PWM else 0.0
            if self.freeze:
                self.ext_cmd = 1.0 if t % PWM < self.f * PWM else 0.0
        else:
            self.ext_cmd = 1.0
        if t - self.t_pred >= 2.0:
            self.t_pred = t
            self.pred = self._p(st, self._f_now(st))
            self.hist.append((st.h, self.pred[0], self.pred[1]))
        # vectored retro pulses, predictor-sized in 2-D (along: only ever shorten; cross: to zero). The tilt off
        # retrograde is capped (cap >= 45 deg: high, low-q pulse, the along part is raised to reach the cross
        # demand; smaller caps: the cross part is clipped)
        if self.pulses and st.h <= self.pulses[0][0]:
            hp, tgt, cap, mode = self.pulses.pop(0)
            f_sz = self.f0 if (self.mod and st.h > self.floor) else None     # centre the modulator
            c_hat = unit3(np.cross(air, up))
            cr = lambda r_: r_[0] * c_hat[0] + r_[1] * c_hat[1]               # noqa: E731
            sol = np.zeros(2)
            for _ in range(2):
                r0 = self._p(st, f_sz, dv=sol[0], dv_side=sol[1] * c_hat)
                ra = self._p(st, f_sz, dv=sol[0] + 20.0, dv_side=sol[1] * c_hat)
                rc = self._p(st, f_sz, dv=sol[0], dv_side=(sol[1] + 20.0) * c_hat)
                J = np.array([[self._along(ra) - self._along(r0), self._along(rc) - self._along(r0)],
                              [cr(ra) - cr(r0), cr(rc) - cr(r0)]]) / 20.0
                res = np.array([tgt - self._along(r0), -cr(r0)])
                try:
                    sol = sol + np.linalg.solve(J, res)
                except np.linalg.LinAlgError:
                    break
                sol[0] = max(sol[0], 0.0)              # retro only
            dva, dvc = float(sol[0]), float(sol[1])
            tc = math.tan(math.radians(cap))
            if abs(dvc) > dva * tc:
                if mode == "raise":                    # cross-track wins: accept the extra shortening
                    dva = abs(dvc) / tc
                else:
                    dvc = math.copysign(dva * tc, dvc)
            dva, dvc = min(dva, 400.0), max(-150.0, min(150.0, dvc))
            if math.hypot(dva, dvc) > 1.0:
                self.dv_goal, self.dv_done, self.p_t0 = math.hypot(dva, dvc), 0.0, t
                self.p_tilt = math.atan2(abs(dvc), dva)
                self.p_wait = mode == "raise"          # low pulses (high q, <= 8 deg) light at once
                self.p_side = c_hat * math.copysign(1.0, dvc)
                self.phase = "PULSE"
                self.log.append(("pulse", round(st.h), round(dva, 1), round(dvc, 1)))
                self.cmd = air * math.cos(self.p_tilt) + self.p_side * math.sin(self.p_tilt)
                self.t_pwm = -1e9
                return 0.0
        # ignition
        if st.h < 9000 and ign_rule(st.h, sp, sg, st.m, T, ad, MU / r ** 2):
            self.phase = "BURN"
            self.ign = rs.State(t, st.P.copy(), st.v.copy(), st.nose.copy(), st.m)
            self.cmd = air
            return burn_throttle(st.h, sp, sg, st.m, T, ad, MU / r ** 2)
        # aero channel: fixed aoa, lift direction (azimuth) steered; folded or moving fins -> straight
        if veh.ext <= 0.9:
            self.cmd = air
            return 0.0
        c_hat = unit3(np.cross(air, up))
        e2 = np.cross(air, c_hat)
        s_ = math.sin(math.radians(a_cmd))
        if s_ < 1e-6:                                # 0 deg design: straight, no aero steering
            self.cmd = air
            return 0.0
        if t - self.t_x >= 1.0 and self.pred is not None and st.q > 300:
            self.t_x = t
            tgo = max(self.pred[2] - (t - self.t_pred) + 8.0, 10.0)
            rv = np.array([self.pred[0], self.pred[1], 0.0])
            cross_only = bool(self.pulses) or (self.mod is not None and st.h > self.floor) or self.freeze
            a_des = -3.0 * rv / tgo ** 2
            a_des = a_des - np.dot(a_des, air) * air
            az_new = self._azimuth(a_des, cross_only, s_, st, air, up, c_hat, e2)
            d_az = (az_new - self.az + math.pi) % (2 * math.pi) - math.pi
            # azimuth slew limit: a fast swing drags the nose through ~0 aoa (slew 4.2 deg/s / (1 + q/44 kPa))
            self.az += max(-AZ_RATE, min(AZ_RATE, d_az))
        b = c_hat * math.cos(self.az) + e2 * math.sin(self.az)
        self.cmd = air * math.sqrt(1 - s_ * s_) + b * s_
        self.i_in += self.f_in * dt
        return 0.0

    def _azimuth(self, a_des, cross_only, s_, st, air, up, c_hat, e2):
        """Lift-direction steering at fixed aoa: the lateral component tracks the ZEM demand (proportional:
        the azimuth sets how much of |F| is lateral); the in-plane component goes to the along-track ZEM demand
        in the terminal phase, else it is reversed to keep its integral ~0 (bank-reversal logic), so the
        along-track loops (pulses / modulation) are not fighting an unmodelled lift."""
        k = st.q / st.m * self.out_nom
        a_c = float(np.dot(a_des, c_hat))
        a_in = float(np.dot(a_des, e2)) if not cross_only else -self.i_in / 10.0
        best, best_cost, best_fin = self.az, 1e9, 0.0
        for az in np.radians(np.arange(0.0, 360.0, 10.0)):
            b = c_hat * math.cos(az) + e2 * math.sin(az)
            nose = air * math.sqrt(1 - s_ * s_) + b * s_
            fa, fp = _NOM.normal(st.mach, s_, rs.fin_phi(nose, air, up, self.veh.roll))
            F = (fa * b + fp * np.cross(air, b)) * k
            f_lat, f_in = float(np.dot(F, c_hat)), float(np.dot(F, e2))
            d_az = abs((az - self.az + math.pi) % (2 * math.pi) - math.pi)
            cost = abs(f_lat - a_c) + 0.5 * abs(f_in - a_in) + 0.002 * d_az
            if cost < best_cost:
                best, best_cost, best_fin = az, cost, f_in
        self.f_in = best_fin
        return best

    def guidance(self, st, t):
        return self.cmd if self.cmd is not None else st.air


# ------------------------------------------------------------------------------------- cases
_BASE_CACHE = {}
STARTS = {}


def base_state(name):
    if name not in _BASE_CACHE:
        d = rs.load(rs.find_log(name + rs.GF0T_SUFFIX))
        _BASE_CACHE[name] = rs.state_from_log(d, d["i0"])
    return _BASE_CACHE[name]


def _fly(D, s, veh, eps, a_hat, f0, freeze=False):
    _PRED.update(v3=bool(D.get("v3")), ttp15=bool(D.get("ttp15")))
    ctl = Controller(D, veh, eps, a_hat, f0, freeze)

    def ttp(st, t):                       # IDENT-style window: stiff autopilot while coasting above the floor
        return 1.5 if (D.get("ttp15") and st.h > TTP_FLOOR and ctl.phase != "BURN") else rs.TTP_DEFAULT
    run = rs.fly(veh, s, ctl.guidance, throttle=ctl.throttle, h_stop=-50.0, t_max=900.0,
                 stop=lambda st: ctl.phase == "DONE", ttp=ttp)
    return ctl, run


def design_start(key):
    """Design-nominal start state: mass for the fin count (same mass ratio from the recovery start), rotated so
    the undispersed TRUTH flight (no pulses, modulation frozen at f0, cross-track steering on) touches down at
    the design's aim point. f0 centres the modulator's predicted range authority."""
    dname, base = key
    D = DESIGNS[dname]
    s = base_state(base)
    s = rs.State(s.t, s.P, s.v, s.nose, s.m)
    a_hat = unit_h(s.v)
    dry = DRY4 + (D["fins"] - 4) * FIN_KG
    if D["fins"] == 8:
        s.m = (M_REC0 + 4 * FIN_KG) / (M_REC0 / s.m)
    _PRED.update(v3=bool(D.get("v3")), ttp15=bool(D.get("ttp15")))
    proto = Controller(D, None, 0.0, a_hat)
    f0, auth = None, None
    if D.get("mod"):
        r0, r1 = proto._along(proto._p(s, 0.0)), proto._along(proto._p(s, 1.0))
        auth = r0 - r1
        lo, hi = 0.0, 1.0
        for _ in range(20):
            f = 0.5 * (lo + hi)
            if proto._along(proto._p(s, f)) > 0.5 * (r0 + r1):
                lo = f
            else:
                hi = f
        f0 = 0.5 * (lo + hi)
    aim = D["aim"] if D["aim"] != "auto" else 3000.0 - auth / 2 + 300.0
    for _ in range(3):
        veh = Vehicle.from_dict(rs.DEFAULT_MODEL).setup(D["fins"], FOLD_NOM, EIGHT_NOM, 0.0)
        veh.roll = 0.0
        ctl, run = _fly(D, s, veh, 0.0, a_hat, f0, freeze=True)
        err = run["final"].P[:2] - aim * a_hat
        s = rotate_state(s, -err)
    return dict(s=s, a_hat=a_hat, dry=dry, f0=f0, auth=auth, aim=aim)


def draws(i):
    rng = np.random.default_rng(1000 + i)
    return dict(base=BASES[i % len(BASES)], along=rng.uniform(-3000, 3000), cross=rng.uniform(-1000, 1000),
                dP=rng.normal(0, 30, 3), dV=rng.normal(0, 1, 3), drag=rng.normal(1, 0.05),
                lift=rng.normal(1, 0.15), tau=max(rng.normal(1, 0.2), 0.4), roll=rng.uniform(0, 360),
                mslope=rng.normal(0, 0.04), fold=rng.normal(FOLD_NOM, 0.03), eight=rng.normal(EIGHT_NOM, 0.05),
                eps=rng.normal(0, 0.02))


def draws2(i):
    """v2 dispersions (red team): aim N(0,700 m) along / N(0,250 m) cross after the vacuum correction, drag prior
    N(1,5.5 %), Mach shape 3 %, high-q aoa limit N(1,15 %); others as v1. Same seed per case for every design."""
    rng = np.random.default_rng(5000 + i)
    return dict(base=BASES[i % len(BASES)], along=rng.normal(0, 700), cross=rng.normal(0, 250),
                dP=rng.normal(0, 30, 3), dV=rng.normal(0, 1, 3), drag=rng.normal(1, 0.055),
                lift=rng.normal(1, 0.15), tau=max(rng.normal(1, 0.2), 0.4), roll=rng.uniform(0, 360),
                mslope=rng.normal(0, 0.03), fold=FOLD_NOM, eight=EIGHT_NOM, eps=rng.normal(0, 0.02),
                hq=max(rng.normal(1, 0.15), 0.5))


def draws3(i):
    """v3: the v2 draws (same seeds) without the measured high-q limit, plus the stiff-autopilot uncertainties: coning
    amplitude N(1, 20 %), mean-tilt gain N(1, 30 %) (one 8 s step), and where the coning starts below the
    measured 88 kPa: cone_q0 U(0, 80) kPa (0 = cones at every q; 80 = only near the measured band)."""
    r = draws2(i)
    r.pop("hq")
    rng = np.random.default_rng(9000 + i)
    r.update(cone=max(rng.normal(1, 0.2), 0.2), g15=max(rng.normal(1, 0.3), 0.2), cone_q0=rng.uniform(0, 80))
    return r


def apply_dv(s, dv):
    """Signed retro delta-v along the velocity (entry-burn cutoff later/earlier), with its propellant."""
    k = (s.speed - dv) / s.speed
    return rs.State(s.t, s.P.copy(), s.v * k, s.nose.copy(), s.m * math.exp(-dv / (T_VAC / MDOT)))


def cutoff_dv(D, s, a_hat, tgt):
    """Entry-burn cutoff closed on the predictor with the PRIOR drag table (no in-flight drag ratio yet at
    39 km): secant on the signed retro delta-v so the predicted along-track touchdown = tgt."""
    out = 1.0 if D["fins"] == 4 else EIGHT_NOM
    cfg = (D.get("alpha", ALPHA), out, FOLD_NOM, 1.0)

    def al(dv):
        s_ = apply_dv(s, dv)
        r_ = predict(s_.P, s_.v, s_.m, cfg, cfg)
        return r_[0] * a_hat[0] + r_[1] * a_hat[1]
    a0, a1 = al(0.0), al(10.0)
    slope = (a1 - a0) / 10.0
    dv = (tgt - a0) / slope
    dv += (tgt - al(dv)) / slope
    return max(-60.0, min(60.0, dv))


def _init(starts):
    STARTS.update(starts)


def run_case(args):
    dname, i = args
    D = DESIGNS[dname]
    _PRED.update(v3=bool(D.get("v3")), ttp15=bool(D.get("ttp15")))
    r = draws3(i) if D.get("v3") else (draws2(i) if D.get("v2") else draws(i))
    S = STARTS.get((dname, r["base"])) or design_start((dname, r["base"]))
    s0, a_hat, dry, f0 = S["s"], S["a_hat"], S["dry"], S["f0"]
    c_hat = np.array([-a_hat[1], a_hat[0]])
    s = rotate_state(s0, r["along"] * a_hat + r["cross"] * c_hat)
    s = rs.State(s.t, s.P + r["dP"], s.v + r["dV"], s.nose, s0.m)
    veh = Vehicle.from_dict(rs.DEFAULT_MODEL).scaled(drag=r["drag"], lift=r["lift"], tau=r["tau"])
    veh.roll = r["roll"]
    veh.setup(D["fins"], r["fold"], r["eight"], r["mslope"], r.get("hq"))
    if D.get("v3"):
        veh.att15["cone"] *= r["cone"]
        veh.att15["G"] *= r["g15"]
        veh.att15["cone_q0"] = min(r["cone_q0"], veh.att15["cone_q1"] - 1.0)
    cut_kg = cut_dv = 0.0
    if D.get("cutoff") is not None:
        cut_dv = cutoff_dv(D, s, a_hat, D["cutoff"])
        s1 = apply_dv(s, cut_dv)
        cut_kg = s.m - s1.m
        s = s1
    ctl, run = _fly(D, s, veh, r["eps"], a_hat, f0)
    fin = run["final"]
    h, v = run["h"], np.linalg.norm(run["v"], axis=1)
    q = 0.5 * rs.rho_of(h + rs.PAD_ALT) * v ** 2
    thr = run["thr"]
    k35 = np.where(h <= 3500.0)[0]
    fuel_avail = (s0.m - dry) / KG_PER_LF
    dm = -np.diff(run["m"])
    ign_t = ctl.ign.t if ctl.ign is not None else np.inf
    tt = run["t"][:-1]
    pulse_kg, burn_kg = float(dm[tt < ign_t].sum()) + cut_kg, float(dm[tt >= ign_t].sum())
    miss_v = fin.P[:2]
    return dict(design=dname, i=i, base=r["base"], miss=float(np.linalg.norm(miss_v)),
                along=float(miss_v @ a_hat), cross=float(miss_v @ c_hat),
                ign_h=float(ctl.ign.h) if ctl.ign else float("nan"),
                v_ign=float(ctl.ign.speed) if ctl.ign else float("nan"),
                v35=float(v[k35[0]]) if len(k35) else float("nan"), v_td=float(fin.speed),
                q_peak=float(q[thr < 0.05].max()) / 1e3, pulse_u=pulse_kg / KG_PER_LF,
                burn_u=BURN_CAL * burn_kg / KG_PER_LF, fuel_avail=fuel_avail,
                margin=fuel_avail - (pulse_kg + BURN_CAL * burn_kg) / KG_PER_LF,
                sat=float(run["sat_frac"]), switches=ctl.switches, log=[str(x) for x in ctl.log[:8]],
                aim_along=r["along"], aim_cross=r["cross"], cut_dv=cut_dv, cut_u=cut_kg / KG_PER_LF)


# ------------------------------------------------------------------------------------- reporting
CONF = {"A1": (8, "validated envelope: fins out, aoa <= 5 deg, thrust identified"),
        "A1e": (8, "as A1"),
        "A2": (4, "folded-fin flight never flown: CdA ratio static only; no-fin attitude, deploy transients unknown"),
        "A3": (6, "fins out, aoa 1-8 deg inside the envelope, but drag/lift at 6-8 deg come from saturated flights"),
        "B1": (6, "8-fin aero = static x1.64; normal force and attitude authority extrapolated"),
        "B1e": (6, "as B1"),
        "B2": (3, "both extrapolations (8 fins + folded flight); logic proven on the airbrake craft"),
        "B2p": (3, "as B2")}


def pct(x, p):
    return float(np.nanpercentile(x, p))


def summarize(res, starts=None):
    rows = {}
    for dn in DESIGNS:
        R_ = [r for r in res if r["design"] == dn]
        if not R_:
            continue

        def a(k):
            return np.array([r[k] for r in R_], float)
        S = (starts or {}).get((dn, BASES[0]), {})
        rows[dn] = dict(
            N=len(R_), miss_med=pct(a("miss"), 50), miss_90=pct(a("miss"), 90), p100=float(np.mean(a("miss") <= 100)),
            along_med=pct(a("along"), 50), v35=pct(a("v35"), 50), v_ign=pct(a("v_ign"), 50), ign_h=pct(a("ign_h"), 50),
            burn_u=pct(a("burn_u"), 50), pulse_u=pct(a("pulse_u"), 50), pulse_u90=pct(a("pulse_u"), 90),
            tot_u=pct(a("burn_u") + a("pulse_u"), 50), avail=pct(a("fuel_avail"), 50),
            margin_med=pct(a("margin"), 50), margin_10=pct(a("margin"), 10), p_dry=float(np.mean(a("margin") < 0)),
            q_peak=pct(a("q_peak"), 50), q_peak90=pct(a("q_peak"), 90), sat_med=pct(a("sat"), 50),
            sat_max=float(np.nanmax(a("sat"))), p_sat=float(np.mean(a("sat") > 0.05)), switches=pct(a("switches"), 50),
            payload_t=(DESIGNS[dn]["fins"] - 4) * FIN_KG / 1e3, f0=S.get("f0"), auth=S.get("auth"), aim=S.get("aim"))
    return rows


def scores(rows):
    out = {}
    for dn, r in rows.items():
        acc = 10 * r["p100"]
        fuel = 10 * min(max(r["margin_10"] / 100.0, 0.0), 1.0)
        pay = 10 * max(1 - r["payload_t"] / PAYLOAD_NOW, 0.0)
        conf = CONF[dn][0]
        rej = r["p_sat"] > 0.05 or r["sat_med"] > 0.05
        out[dn] = dict(acc=acc, fuel=fuel, pay=pay, conf=conf, rejected=rej,
                       total=0.40 * acc + 0.25 * fuel + 0.20 * pay + 0.15 * conf)
    return out


def print_tables(rows, sc):
    print("| design | miss med / p90 m | P(<=100 m) | v 3.5 km | v ign | ign h | burn u | pulse u med/p90 | total u |"
          " avail u | margin med/p10 | P(dry) | q peak med/p90 kPa | sat med/max | switches | auth m | payload |")
    print("|---" * 17 + "|")
    for dn, r in rows.items():
        print("| %s | %.0f / %.0f | %.2f | %.0f | %.0f | %.0f | %.0f | %.0f / %.0f | %.0f | %.0f | %.0f / %.0f | %.2f |"
              " %.0f / %.0f | %.3f / %.3f | %.0f | %s | -%.1f t |" % (
                  dn, r["miss_med"], r["miss_90"], r["p100"], r["v35"], r["v_ign"], r["ign_h"], r["burn_u"],
                  r["pulse_u"], r["pulse_u90"], r["tot_u"], r["avail"], r["margin_med"], r["margin_10"], r["p_dry"],
                  r["q_peak"], r["q_peak90"], r["sat_med"], r["sat_max"], r["switches"],
                  "-" if r["auth"] is None else "%.0f" % r["auth"], r["payload_t"]))
    print()
    print("| design | accuracy 40% | fuel margin 25% | payload 20% | model conf. 15% | weighted |")
    print("|---|---|---|---|---|---|")
    for dn, s in sorted(sc.items(), key=lambda kv: -kv[1]["total"]):
        print("| %s | %.1f | %.1f | %.1f | %d | **%.2f**%s |" % (dn, s["acc"], s["fuel"], s["pay"], s["conf"], s["total"],
                                                              " (rejected: sat)" if s["rejected"] else ""))


# ------------------------------------------------------------------------------------- checks
def check():
    """1) predictor vs truth (undispersed, aoa 5 deg fixed azimuth, fins out); 2) landing-burn fuel vs logs."""
    for b in BASES:
        s = base_state(b)
        a_hat = unit_h(s.v)
        D = dict(fins=4, mod=None, pulses=())
        veh = Vehicle.from_dict(rs.DEFAULT_MODEL).setup(4, FOLD_NOM, EIGHT_NOM, 0.0)
        ctl, run = _fly(D, s, veh, 0.0, a_hat, None, freeze=True)
        f = run["final"]
        for hq in (35000, 20000, 10000, 5000):
            p = next(x for x in ctl.hist if x[0] <= hq)[1:]
            print("%s from %5.0f m: predictor td (%.0f, %.0f) truth (%.0f, %.0f) diff %.0f m" % (
                b, hq, p[0], p[1], f.P[0], f.P[1], math.hypot(p[0] - f.P[0], p[1] - f.P[1])))
        print("   ign h %.0f v %.0f; idealised burn %.0f u" % (ctl.ign.h, ctl.ign.speed, (ctl.ign.m - f.m) / KG_PER_LF))
    for name in CHECK_FLIGHTS:
        d = rs.load(rs.find_log(name + rs.GF0T_SUFFIX))
        i = d["i1"]
        s = rs.state_from_log(d, i)
        p = predict(s.P, s.v, s.m, (1.0, 1.0, 0.0, 1.0), (1.0, 1.0, 0.0, 1.0))
        print("%s logged ignition h %.0f v %.0f: logged landing burn %.0f u; idealised rule+law from there %.0f u"
              % (name, s.h, s.speed, (d["m"][i] - d["m"][-1]) / KG_PER_LF, p[3] / KG_PER_LF))


def boot(x, stat, n=2000, seed=7):
    rng = np.random.default_rng(seed)
    x = np.asarray(x, float)
    b = [stat(x[rng.integers(0, len(x), len(x))]) for _ in range(n)]
    return float(np.percentile(b, 2.5)), float(np.percentile(b, 97.5))


def report_v2(res, ref="LITE5", designs=V2):
    """Paired ranking: every design against the reference on the same cases, with bootstrap 95 % intervals."""
    by = {}
    for r in res:
        by.setdefault(r["design"], {})[r["i"]] = r
    names = [d for d in designs if d in by]
    ids = sorted(set.intersection(*[set(by[d]) for d in names]))
    rows = {}
    for d in names:
        mi = np.array([by[d][i]["miss"] for i in ids])
        diff = mi - np.array([by[ref][i]["miss"] for i in ids])
        win = (diff < 0).astype(float)
        a = lambda k: np.array([by[d][i][k] for i in ids], float)  # noqa: E731
        rows[d] = dict(med=np.median(mi), med_ci=boot(mi, np.median), p90=np.percentile(mi, 90),
                       p90_ci=boot(mi, lambda x: np.percentile(x, 90)), p100=np.mean(mi <= 100),
                       dmed=np.median(diff), dmed_ci=boot(diff, np.median), win=win.mean(), win_ci=boot(win, np.mean),
                       margin10=np.percentile(a("margin"), 10), pulse=np.median(a("pulse_u")),
                       cut=np.median(a("cut_u")), qmed=np.median(a("q_peak")), q90=np.percentile(a("q_peak"), 90),
                       sat_med=np.median(a("sat")), sat_max=a("sat").max(), p_sat=np.mean(a("sat") > 0.05),
                       along=np.median(a("along")), cross=np.median(np.abs(a("cross"))), v_ign=np.median(a("v_ign")),
                       v_ign90=np.percentile(a("v_ign"), 90), ign_h=np.median(a("ign_h")))
    order = sorted(names, key=lambda d: rows[d]["med"])
    print("N = %d paired cases; reference %s" % (len(ids), ref))
    print("| rank | design | miss median [95%] | p90 [95%] | P(<=100) | paired median diff vs ref [95%] | "
          "P(better than ref) [95%] | margin p10 u | pulses+cutoff u | peak q med/p90 kPa | sat med / max / P(>0.05) |")
    print("|---" * 11 + "|")
    for k, d in enumerate(order, 1):
        r = rows[d]
        print("| %d | %s | %.0f [%.0f, %.0f] | %.0f [%.0f, %.0f] | %.2f | %+.0f [%+.0f, %+.0f] | %.2f [%.2f, %.2f] |"
              " %.0f | %.0f | %.0f / %.0f | %.3f / %.3f / %.2f |" % (
                  k, d, r["med"], *r["med_ci"], r["p90"], *r["p90_ci"], r["p100"], r["dmed"], *r["dmed_ci"],
                  r["win"], *r["win_ci"], r["margin10"], r["pulse"], r["qmed"], r["q90"], r["sat_med"],
                  r["sat_max"], r["p_sat"]))
    for d in order:
        r = rows[d]
        print("%s: along median %+.0f m, |cross| median %.0f m, cutoff %+.1f u, v_ign median %.0f / p90 %.0f m/s at "
              "h %.0f m" % (d, r["along"], r["cross"], r["cut"], r["v_ign"], r["v_ign90"], r["ign_h"]))
    return rows


def main(argv):
    import argparse
    ap = argparse.ArgumentParser(prog="dr002_trade.py", description="DR-002 trade study: recovery architecture for the "
                                 "fins-only GF0T core (offline Monte Carlo on the identified replay model; see the "
                                 "module docstring for the designs). Needs the GF0T landing CSVs.")
    ap.add_argument("command", nargs="?", default="run", choices=("check", "run", "run2", "run3"),
                    help="check: predictor vs truth and landing-burn fuel vs the logged flights; run: v1 Monte Carlo; "
                         "run2 / run3: the v2 / v3 reruns")
    ap.add_argument("n", nargs="?", type=int, default=200, help="cases per design (default 200)")
    ap.add_argument("out", nargs="?", help="output JSON file")
    ap.add_argument("designs", nargs="?", help="comma-separated design names, e.g. A1,A1e (default: all of the version)")
    ap.add_argument("--logs", metavar="DIR", help="directory with the landing CSVs (else KSP_REPLAY_LOGS, else the "
                    "repository's data/raw/logs, logs and tests/data)")
    a = ap.parse_args(argv[1:])
    if a.logs:
        os.environ["KSP_REPLAY_LOGS"] = a.logs        # also seen by the multiprocessing workers
    cmd = a.command
    try:       # fail early and readably, before any worker starts
        for name in (BASES + CHECK_FLIGHTS if cmd == "check" else BASES):
            rs.find_log(name + rs.GF0T_SUFFIX)
    except rs.NeedData as e:
        print("dr002_trade.py: %s" % e, file=sys.stderr)
        return 2
    if cmd == "check":
        check()
        return 0
    N, out_path = a.n, a.out
    names = a.designs.split(",") if a.designs else list({"run2": V2, "run3": V3}.get(cmd, V1))
    nproc = max(1, (os.cpu_count() or 2) - 2)
    keys = [(dn, b) for dn in names for b in BASES]
    with mp.Pool(nproc) as pool:
        starts = dict(zip(keys, pool.map(design_start, keys)))
    for k, S in starts.items():
        print(k, "aim %.0f m" % S["aim"], "f0", None if S["f0"] is None else round(S["f0"], 3),
              "authority", None if S["auth"] is None else round(S["auth"]))
    jobs = [(dn, i) for dn in names for i in range(N)]
    with mp.Pool(nproc, initializer=_init, initargs=(starts,)) as pool:
        res = pool.map(run_case, jobs, chunksize=2)
    if cmd == "run2":
        rows, sc = report_v2(res), {}
    elif cmd == "run3":
        rows, sc = report_v2(res, ref="LITE5L15", designs=V3), {}
    else:
        rows = summarize(res, starts)
        sc = scores(rows)
        print_tables(rows, sc)
    if out_path:
        json.dump(dict(rows=rows, scores=sc, cases=res), open(out_path, "w"), indent=1, default=float)
        print("wrote", out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
