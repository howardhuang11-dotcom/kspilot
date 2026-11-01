"""Theory experiment: high-altitude aerodynamic guidance for the fins-only core (T-WorksFineGF0T), engine first.

Vectorised Monte-Carlo, 3-D point mass around a spherical, non-rotating Kerbin, from the post-entry-burn state
(h 39 km, v_h 1312, v_z -237 m/s) down to h = 1.5 km. Truth model (dispersed, unknown to the controller):
  * drag    CdA(AoA, Mach) = s0 * CdA0(M) + s1 * kM(M) * dA(AoA)   (flight logs + the 40 km static table)
  * lift    a = c q sin(AoA) toward the side the ENGINE end leans to (sign known), c = 4.5e-4 m^2/kg x U(0.6, 1.4)
            x (1 +- 30 % linear Mach trend); achieved tilt direction rotated about the airflow by U(+-15 deg)
  * AoA     achieved <= amax(q): ~36 deg at q <= 1.3 kPa blending to ~11 deg at q >= 2.5 kPa (tail fins
            weathervane the stage); first-order attitude lag tau 1-2 s, rate limit 10 deg/s, 1 deg wobble
  * density atmo.density x U(0.9, 1.1) x (1 +- 4 % altitude ripple); mass 15.3 t +- 0.5 t
  * aim     target along / cross offset U(+-3 km) / U(+-1 km) from the law's reference prediction
Controller side: nominal tables, online drag scale k (measured / model), online lift coefficient (RLS),
predictor = the same point mass integrated with the controller's model (lift ignored unless the law says so).

Laws (once a second above h_sw, then the flight's ZEM/ZEV glide below h_sw unless noted):
  F   belly as flown 2026-10-20: AoA cmd 45 deg x f (f from straight / 12-deg drag predictions), lift
      sideways, bang-bang roll reversal when the cross miss grows
  A   straight + predictor lift steering (cross_glide with xglide_along): finite-difference sensitivities of the
      predicted along / cross miss to 1 m/s, T = 15 s, tilt <= 12 deg
  B   Shuttle-like: AoA = f x amax (f zeroes the drag-only along prediction), continuous bank: side lift for the
      cross miss, the rest lift-up (lift-down only when long at full drag)
  BL  B with the lift included in its predictions
  C   joint lift+drag: control u = (u_up, u_side) in the unit disc, AoA = |u| amax_nom(q), lift along u;
      predictor WITH lift (online c, online drag scale); one damped Newton step per second on (along, cross)
      C10 / C3 / C15: C down to 10 / 3 / 1.5 km (ZEM/ZEV glide below)
usage: belly_mc.py [N] [--laws F,A,B,BL,C10,C3,C15] [--seed S] [--c0 4.5e-4] [--aim law|straight|mid]
"""
import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot import atmo  # noqa: E402

R, MU = 600000.0, 3.5316e12
G0 = MU / R ** 2
D2R = math.pi / 180.0

# ---------------------------------------------------------------- atmosphere
_ALT = np.asarray(atmo.KERBIN["alt"], float)
_RHO = np.asarray(atmo.KERBIN["density"], float)
_P = np.asarray(atmo.KERBIN["pressure"], float)
_SS = np.sqrt(1.4 * _P / np.maximum(_RHO, 1e-9))
_SS[_RHO < 1e-6] = 300.0


def rho_nom(h):
    return np.interp(h, _ALT, _RHO, right=0.0)


def sound(h):
    return np.interp(h, _ALT, _SS)


# ---------------------------------------------------------------- aero model (flight-measured numbers)
_M = np.array([0.0, 0.8, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 20.0])
_CDA0 = np.array([12.0, 12.0, 12.0, 7.0, 5.3, 4.6, 4.2, 3.7, 3.7])      # straight, logs (AoA < 4 deg)
_ATAB = np.array([0.0, 15.0, 30.0, 45.0, 60.0, 90.0]) * D2R
_DA = np.array([3.7, 9.1, 14.2, 19.1, 23.2, 25.2]) - 3.7                # 40 km / 2000 m/s static table


def cda(alpha, mach, s0=1.0, s1=1.0):
    km = np.interp(mach, [1.0, 3.0], [1.25, 1.0])      # subsonic: 12 deg -> ~16-18 m^2 (logs / user)
    return s0 * np.interp(mach, _M, _CDA0) + s1 * km * np.interp(alpha, _ATAB, _DA)


def amax(q, lo=36.0, hi=11.0, q1=1300.0, q2=2500.0):
    """Achievable angle of attack (rad) vs dynamic pressure: fins weathervane the stage at high q."""
    return D2R * (lo + (hi - lo) * np.clip((q - q1) / (q2 - q1), 0.0, 1.0))


C_NOM = 4.5e-4            # m^2/kg, flight fit 3e-4 .. 6e-4 below 30 km
GLIDE_C_PER_KG = 15.0     # landing.glide_c_per_kg (flight constant, = c ~ 9.8e-4 at 15.3 t)
GLIDE_MAX = 12.0          # deg
H_GLIDE_IGN = 450.0


# ---------------------------------------------------------------- vector helpers (rows = runs)
def nrm(a):
    return np.sqrt(np.einsum("ij,ij->i", a, a))


def dot(a, b):
    return np.einsum("ij,ij->i", a, b)


def unit(a):
    return a / np.maximum(nrm(a), 1e-12)[:, None]


def basis(p, v):
    """e_upp: local up perpendicular to v (lift-up); e_side: p x v (left of the track, = +cross)."""
    vh = unit(v)
    up = unit(p)
    e_upp = unit(up - dot(up, vh)[:, None] * vh)
    e_side = unit(np.cross(p, v))
    return e_upp, e_side


def track_err(p, phi_t, lam_t):
    phi = np.arctan2(p[:, 1], p[:, 0])
    lam = np.arcsin(np.clip(p[:, 2] / nrm(p), -1, 1))
    return R * (phi - phi_t), R * (lam - lam_t)


def track_dirs(p):
    phi = np.arctan2(p[:, 1], p[:, 0])
    lam = np.arcsin(np.clip(p[:, 2] / nrm(p), -1, 1))
    e_al = np.stack([-np.sin(phi), np.cos(phi), np.zeros_like(phi)], 1)
    e_cr = np.stack([-np.sin(lam) * np.cos(phi), -np.sin(lam) * np.sin(phi), np.cos(lam)], 1)
    return e_al, e_cr


def from_offset(v, o):
    """Flight-style pointing: nose (top) = unit(air + o), air = -v. Body force a = -c q beta (against the
    top's tilt) -> returns (AoA, unit lift direction)."""
    air = -unit(v)
    nose = unit(air + o)
    perp = nose - dot(nose, air)[:, None] * air
    alpha = np.arccos(np.clip(dot(nose, air), -1, 1))
    return alpha, -unit(perp)


# ---------------------------------------------------------------- controller-side predictor
def predict(p, v, m, kD, cl, afix, f, u_up, u_side, h_lo, a_lo, h_end, phi_t, lam_t, dt=0.5, tmax=500.0):
    """Integrate the controller's model to h_end. Policy: AoA = afix (if not nan) else f * amax_nom(q) while
    h > h_lo, a_lo below; lift cl * q * sin(AoA) along (u_up e_upp + u_side e_side) (only while h > h_lo).
    Returns along, cross (m) of the h_end crossing."""
    p, v = p.copy(), v.copy()
    K = len(p)
    un = np.hypot(u_up, u_side)
    has_l = (un > 1e-6) & (cl != 0)
    wu, ws = np.where(has_l, u_up / np.maximum(un, 1e-9), 0.0), np.where(has_l, u_side / np.maximum(un, 1e-9), 0.0)
    fixed = ~np.isnan(afix)
    afx = np.where(fixed, afix, 0.0)

    if np.ndim(h_lo) == 0:
        h_lo = np.full(K, float(h_lo))
    if np.ndim(a_lo) == 0:
        a_lo = np.full(K, float(a_lo))

    def acc(p, v, s):
        r = nrm(p)
        h = r - R
        sp = nrm(v)
        q = 0.5 * rho_nom(h) * sp * sp
        hi = h > h_lo[s]
        al = np.where(hi, np.where(fixed[s], afx[s], f[s] * amax(q)), a_lo[s])
        a = -MU * p / (r ** 3)[:, None] - (kD[s] * q * cda(al, sp / sound(h)) / m[s] / sp)[:, None] * v
        lmag = np.where(hi & has_l[s], cl[s] * q * np.sin(al), 0.0)
        if np.any(lmag != 0):
            e_upp, e_side = basis(p, v)
            a = a + lmag[:, None] * (wu[s][:, None] * e_upp + ws[s][:, None] * e_side)
        return a

    out = np.full((K, 3), np.nan)
    done = np.zeros(K, bool)
    hprev = nrm(p) - R
    done |= hprev <= h_end
    out[done] = p[done]
    t = 0.0
    while not done.all() and t < tmax:
        idx = ~done
        s = np.where(idx)[0]
        pp, vv = p[idx], v[idx]
        a1 = acc(pp, vv, s)
        pm, vm = pp + 0.5 * dt * vv, vv + 0.5 * dt * a1
        a2 = acc(pm, vm, s)
        pn, vn = pp + dt * vm, vv + dt * a2
        hn = nrm(pn) - R
        ho = nrm(pp) - R
        hit = hn <= h_end
        if hit.any():
            w = ((ho - h_end) / np.maximum(ho - hn, 1e-9))[hit][:, None]
            sub = np.where(idx)[0][hit]
            out[sub] = pp[hit] + w * (pn[hit] - pp[hit])
            done[sub] = True
        p[idx], v[idx] = pn, vn
        t += dt
    left = ~done
    out[left] = p[left]
    return track_err(out, phi_t, lam_t)


# ---------------------------------------------------------------- Monte-Carlo
H0, VH0, VU0, M0 = 39000.0, 1312.0, -237.0, 15300.0


def draw(N, seed, along=3000.0, lag=1.0):
    rng = np.random.default_rng(seed)
    U = lambda a, b: rng.uniform(a, b, N)  # noqa: E731
    return dict(
        d_al=U(-along, along), d_cr=U(-1000, 1000),
        s0=U(0.9, 1.1), s1=U(0.85, 1.15), rho_b=U(0.9, 1.1), rho_ph=U(0, 2 * math.pi),
        c=C_NOM * U(0.6, 1.4), m=M0 + U(-500, 500),
        a_lo=U(30, 40), a_hi=U(9, 13), q1=1300 * U(0.8, 1.2), q2=2500 * U(0.8, 1.2),
        tau=lag * U(1.0, 2.0), rot=D2R * U(-15, 15), c_mslope=U(-0.3, 0.3), dvh=U(-5, 5), dvu=U(-5, 5), dvc=U(-2, 2),
        wob_seed=rng.integers(1 << 30),
    )


class Sim:
    def __init__(self, law, D, c0=C_NOM, aim="law", dt=0.1, dt_g=1.0, T=15.0):
        self.law, self.D, self.c0, self.dt, self.dt_g, self.T = law, D, c0, dt, dt_g, T
        N = len(D["c"])
        self.N = N
        self.h_sw = {"C3": 3000.0, "C15": 1500.0}.get(law, 10000.0)
        # nominal start and the law's reference aim (what the deorbit plan would aim with)
        p0 = np.array([[R + H0, 0.0, 0.0]])
        v0 = np.array([[VU0, VH0, 0.0]])
        ref = aim if aim != "law" else "center"
        one, zero = np.ones(1), np.zeros(1)
        pr = lambda afix, f, uu, cl: predict(p0, v0, np.array([M0]), one, cl * one, afix * one, f * one,  # noqa: E731
                                             uu * one, zero, H_GLIDE_IGN, 0.0, H_GLIDE_IGN, 0.0, 0.0)[0]
        if ref == "straight":       # max range (lift can only add drag): the deorbit aim of the flights
            al0 = pr(0.0, 0.0, 0.0, 0.0)
        elif ref == "mid":          # drag-only middle: AoA = 0.5 x achievable, no lift
            al0 = pr(np.nan, 0.5, 0.0, 0.0)
        else:                       # centre of the reachable window [full AoA lift-down, straight]
            al0 = 0.5 * (pr(0.0, 0.0, 0.0, 0.0) + pr(np.nan, 1.0, -1.0, C_NOM))
        self.al_ref = float(al0[0])
        self.phi_t = (al0[0] + D["d_al"]) / R
        self.lam_t = D["d_cr"] / R
        self.p = np.tile(p0, (N, 1))
        self.v = np.stack([VU0 + D["dvu"], VH0 + D["dvh"], D["dvc"]], 1)
        self.m = D["m"]
        self.aach = np.zeros((N, 3))                    # achieved AoA vector (rad, along the lift direction)
        self.cmd_a, self.cmd_u, self.cmd_s = np.zeros(N), np.zeros(N), np.zeros(N)   # held command (local frame)
        self.wob = np.zeros((N, 2))
        self.rng = np.random.default_rng(int(D["wob_seed"]))
        self.kD = np.ones(N)                            # online drag scale
        self.rls_n, self.rls_d = np.zeros(N), np.zeros(N)
        self.active = np.ones(N, bool)
        self.res = {k + t: np.full(N, np.nan) for k in ("along", "cross", "v", "vh") for t in ("", "_ign")}
        self.qmax = np.zeros(N)
        self.t = 0.0
        # law state
        self.f = np.full(N, 0.5)
        self.u = np.stack([np.zeros(N), np.full(N, 0.3)], 1)
        self.sign = np.ones(N)
        self.flip_t = np.full(N, -1e9)
        self.hist = []
        self.n_rev = np.zeros(N)

    # --- truth
    def truth_acc(self, p, v, aach, idx):
        D = self.D
        r = nrm(p)
        h = r - R
        sp = nrm(v)
        rho = rho_nom(h) * D["rho_b"][idx] * (1 + 0.04 * np.sin(2 * math.pi * h / 9000.0 + D["rho_ph"][idx]))
        q = 0.5 * rho * sp * sp
        al = nrm(aach)
        ldir = aach / np.maximum(al, 1e-12)[:, None]
        drag = q * cda(al, sp / sound(h), D["s0"][idx], D["s1"][idx]) / self.m[idx]
        mach = sp / sound(h)
        lift = D["c"][idx] * (1 + D["c_mslope"][idx] * np.clip((mach - 2.0) / 3.0, -1, 1)) * q * np.sin(al)
        a = -MU * p / (r ** 3)[:, None] - (drag / sp)[:, None] * v + lift[:, None] * ldir
        return a, q, drag, lift

    @property
    def c_hat(self):
        w = 2e7
        return (self.rls_n + w * self.c0) / (self.rls_d + w)

    # --- glide (flight ZEM/ZEV, landing.glide_dir) -> (alpha, ldir)
    def glide(self, idx):
        p, v, m = self.p[idx], self.v[idx], self.m[idx]
        h = nrm(p) - R
        up = unit(p)
        vz = dot(v, up)
        e_al, e_cr = track_dirs(p)
        al, cr = track_err(p, self.phi_t[idx], self.lam_t[idx])
        vh = np.stack([dot(v, e_al), dot(v, e_cr)], 1)
        rh = np.stack([al, cr], 1)
        sp = nrm(v)
        tau = np.maximum(sp, 1.0) / G0
        t_go = np.maximum((h - H_GLIDE_IGN) / np.maximum(-vz, 1.0), 15.0)
        k = np.exp(-t_go / tau)
        zem = rh + vh * (tau * (1 - k))[:, None]
        zev = vh * k[:, None]
        a_des = -6 * zem / (t_go ** 2)[:, None] + 2 * zev / t_go[:, None]
        q = 0.5 * rho_nom(h) * sp * sp
        if self.law.startswith("C"):
            cq = self.c_hat[idx] * q     # online c (sign known)
        else:
            cq = GLIDE_C_PER_KG / np.maximum(m, 1000.0) * q
        b = -a_des / np.maximum(cq, 1e-3)[:, None]                          # nf_sign = -1
        bmax = math.sin(GLIDE_MAX * D2R)
        nb = np.linalg.norm(b, axis=1)
        b *= np.minimum(1.0, bmax / np.maximum(nb, 1e-12))[:, None]
        o = b[:, :1] * e_al + b[:, 1:] * e_cr
        return from_offset(v, o)

    # --- guidance laws, once per dt_g, for rows idx above h_sw
    def pred(self, idx, rep, dv=None, **kw):
        """Run the predictor for rows idx, `rep` stacked variants; kw arrays already of length rep*len(idx)."""
        n = len(idx)
        P = np.tile(self.p[idx], (rep, 1))
        V = np.tile(self.v[idx], (rep, 1))
        if dv is not None:
            V = V + dv
        tile = lambda a: np.tile(a[idx], rep)  # noqa: E731
        z = np.zeros(rep * n)
        args = dict(cl=z, afix=np.full(rep * n, np.nan), f=z, u_up=z, u_side=z, h_lo=10000.0, a_lo=5 * D2R, h_end=0.0)
        args.update(kw)
        al, cr = predict(P, V, tile(self.m), tile(self.kD), args["cl"], args["afix"], args["f"], args["u_up"],
                         args["u_side"], args["h_lo"], args["a_lo"], args["h_end"], tile(self.phi_t),
                         tile(self.lam_t))
        return al.reshape(rep, n), cr.reshape(rep, n)

    def guide(self, idx):
        law = self.law
        p, v, m = self.p[idx], self.v[idx], self.m[idx]
        n = len(idx)
        h = nrm(p) - R
        sp = nrm(v)
        q = 0.5 * rho_nom(h) * sp * sp
        up = unit(p)
        e_upp, e_side = basis(p, v)
        vhor = v - dot(v, up)[:, None] * up
        fwd = unit(vhor)
        side = unit(np.cross(up, v))
        if law == "A":
            dv = np.concatenate([np.zeros((n, 3)), side, fwd])
            al, cr = self.pred(idx, 3, dv, afix=np.zeros(3 * n), f=np.zeros(3 * n), h_lo=0.0, a_lo=0.0)
            k = cr[1] - cr[0]
            a_side = np.where(np.abs(k) > 1, -cr[0] / np.where(np.abs(k) > 1, k, 1) / self.T, 0.0)
            k_al = al[2] - al[0]
            a_fwd = np.where(np.abs(k_al) > 1, -al[0] / np.where(np.abs(k_al) > 1, k_al, 1) / self.T, 0.0)
            a_vec = a_side[:, None] * side + a_fwd[:, None] * fwd
            cq = GLIDE_C_PER_KG / np.maximum(m, 1000.0) * q
            b = -a_vec / np.maximum(cq, 1e-3)[:, None]
            nb = nrm(b)
            beta = np.minimum(nb, GLIDE_MAX * D2R)
            o = unit(b) * np.tan(beta)[:, None]
            return from_offset(v, o)
        if law == "F":
            n2 = 2 * n
            afix = np.concatenate([np.full(n, 12 * D2R), np.zeros(n)])
            al, cr = self.pred(idx, 2, afix=afix, f=np.zeros(n2), h_lo=0.0, a_lo=0.0)
            e_out, e_in = al[0], al[1]
            f = np.where(e_out >= 0, 1.0, np.where(e_in <= 0, 0.0, e_in / np.where(e_in - e_out == 0, 1, e_in - e_out)))
            f = np.clip(f, 0, 1)
            self.hist.append((self.t, idx, cr[0]))
            self.hist = [x for x in self.hist if self.t - x[0] <= 6.0 + 1e-6]
            if len(self.hist) >= 5:
                first = np.full(self.N, np.nan)
                first[self.hist[0][1]] = self.hist[0][2]
                c_first = first[idx]
                grow = (np.abs(cr[0]) > 30) & (np.abs(cr[0]) > np.abs(c_first) + 10) & (self.t - self.flip_t[idx] > 8)
                grow &= ~np.isnan(c_first)
                gi = idx[grow]
                self.sign[gi] *= -1
                self.flip_t[gi] = self.t
                self.n_rev[gi] += 1
            b = 45 * D2R * f
            side_ = unit(np.cross(up, vhor))          # left of the horizontal velocity (code: (-vh_y, vh_x))
            o = (self.sign[idx] * np.tan(b))[:, None] * side_
            return from_offset(v, o)
        if law in ("B", "BL"):
            ch = self.c_hat[idx]
            cu, cs = self.cmd_u[idx], self.cmd_s[idx]
            cl = np.tile(ch, 4) if law == "BL" else np.zeros(4 * n)
            fv = np.concatenate([np.zeros(n), np.ones(n), self.f[idx], self.f[idx]])
            dv = np.concatenate([np.zeros((3 * n, 3)), e_side])
            al, cr = self.pred(idx, 4, dv, f=fv, cl=cl, u_up=np.tile(cu, 4), u_side=np.tile(cs, 4))
            e_in, e_out = al[0], al[1]
            f = np.where(e_out >= 0, 1.0, np.where(e_in <= 0, 0.0, e_in / np.where(e_in - e_out == 0, 1, e_in - e_out)))
            f = np.clip(f, 0, 1)
            self.f[idx] = f
            k = cr[3] - cr[2]
            a_side = np.where(np.abs(k) > 1, -cr[2] / np.where(np.abs(k) > 1, k, 1) / self.T, 0.0)
            am = amax(q)
            alpha = f * am
            # cross priority: enough AoA for the side force (Shuttle keeps lift for the reversals too)
            a_req = np.arcsin(np.clip(np.abs(a_side) / np.maximum(ch * q, 1e-6), 0, 1))
            alpha = np.minimum(np.maximum(alpha, a_req), am)
            L = ch * q * np.sin(alpha)
            s = np.clip(a_side, -L, L)
            up_sign = np.where((f >= 1) & (e_out > 0), -1.0, 1.0)
            vert = up_sign * np.sqrt(np.maximum(L * L - s * s, 0))
            ldir = unit(s[:, None] * e_side + vert[:, None] * e_upp + 1e-9 * e_upp)
            return alpha, ldir
        if law.startswith("C"):
            ch = self.c_hat[idx]
            u = self.u[idx]
            dl = 0.05
            U1 = np.concatenate([u[:, 0], u[:, 0] + dl, u[:, 0]])
            U2 = np.concatenate([u[:, 1], u[:, 1], u[:, 1] + dl])
            fm = np.minimum(np.hypot(U1, U2), 1.0)
            h_end = H_GLIDE_IGN
            al, cr = self.pred(idx, 3, f=fm, cl=np.tile(ch, 3), u_up=U1, u_side=U2, h_lo=self.h_sw, h_end=h_end)
            e = np.stack([al[0], cr[0]], 1)
            J = np.stack([np.stack([(al[1] - al[0]) / dl, (al[2] - al[0]) / dl], 1),
                          np.stack([(cr[1] - cr[0]) / dl, (cr[2] - cr[0]) / dl], 1)], 1)    # (n, 2, 2)
            JT = np.transpose(J, (0, 2, 1))
            A = JT @ J + (300.0 ** 2) * np.eye(2)[None]
            du = -np.linalg.solve(A, (JT @ e[:, :, None]))[:, :, 0] * 0.7
            dn = np.linalg.norm(du, axis=1)
            du *= np.minimum(1.0, 0.3 / np.maximum(dn, 1e-12))[:, None]
            u = u + du
            un = np.linalg.norm(u, axis=1)
            u *= np.minimum(1.0, 1.0 / np.maximum(un, 1e-12))[:, None]
            self.u[idx] = u
            alpha = np.minimum(np.linalg.norm(u, axis=1), 1.0) * amax(q)
            ldir = unit(u[:, :1] * e_upp + u[:, 1:] * e_side + 1e-9 * e_upp)
            return alpha, ldir
        raise ValueError(law)

    def run(self):
        dt, D = self.dt, self.D
        next_g = 0.0
        while self.active.any() and self.t < 600:
            idx = np.where(self.active)[0]
            h = nrm(self.p[idx]) - R
            # guidance
            hi = h >= self.h_sw
            if self.t >= next_g - 1e-9:
                next_g += self.dt_g
                gi = idx[hi]
                if len(gi):
                    a, ld = self.guide(gi)
                    e_upp, e_side = basis(self.p[gi], self.v[gi])
                    self.cmd_a[gi], self.cmd_u[gi], self.cmd_s[gi] = a, dot(ld, e_upp), dot(ld, e_side)
            lo = idx[~hi]
            e_upp, e_side = basis(self.p[idx], self.v[idx])
            tgt = (self.cmd_a[idx])[:, None] * (self.cmd_u[idx][:, None] * e_upp + self.cmd_s[idx][:, None] * e_side)
            if len(lo):
                a, ld = self.glide(lo)
                tgt[~hi] = a[:, None] * ld
            # wobble 1 deg rms, 3 s correlation
            self.wob[idx] += -self.wob[idx] * dt / 3.0 + D2R * math.sqrt(2 * dt / 3.0) * self.rng.standard_normal((len(idx), 2))
            tgt = tgt + self.wob[idx, :1] * e_upp + self.wob[idx, 1:] * e_side
            # pointing bias: the achieved tilt direction is rotated about the airflow axis by rot (+-15 deg)
            tu, ts = dot(tgt, e_upp), dot(tgt, e_side)
            cr_, sr_ = np.cos(D["rot"][idx]), np.sin(D["rot"][idx])
            tgt = (cr_ * tu - sr_ * ts)[:, None] * e_upp + (sr_ * tu + cr_ * ts)[:, None] * e_side
            # attitude lag + rate limit
            ach = self.aach[idx]
            d = (tgt - ach) * (dt / D["tau"][idx])[:, None]
            dn = nrm(d)
            d *= np.minimum(1.0, 10 * D2R * dt / np.maximum(dn, 1e-12))[:, None]
            ach = ach + d
            vh_ = unit(self.v[idx])
            ach = ach - dot(ach, vh_)[:, None] * vh_
            # achievable AoA (truth)
            sp = nrm(self.v[idx])
            q_t = 0.5 * rho_nom(h) * D["rho_b"][idx] * sp * sp
            am = amax(q_t, D["a_lo"][idx], D["a_hi"][idx], D["q1"][idx], D["q2"][idx])
            an = nrm(ach)
            ach *= np.minimum(1.0, am / np.maximum(an, 1e-12))[:, None]
            self.aach[idx] = ach
            # integrate (RK2, control held)
            p, v = self.p[idx], self.v[idx]
            a1, q, drag, lift = self.truth_acc(p, v, ach, idx)
            pm, vm = p + 0.5 * dt * v, v + 0.5 * dt * a1
            a2, _, _, _ = self.truth_acc(pm, vm, ach, idx)
            pn, vn = p + dt * vm, v + dt * a2
            self.qmax[idx] = np.maximum(self.qmax[idx], q)
            # sensing: drag scale and lift coefficient (controller's nominal density)
            qm = 0.5 * rho_nom(h) * sp * sp
            al_ = nrm(ach)
            ok = qm > 300
            dmod = qm * cda(al_, sp / sound(h)) / self.m[idx]
            dmeas = drag * (1 + 0.01 * self.rng.standard_normal(len(idx)))
            self.kD[idx] = np.where(ok, self.kD[idx] + dt / 5.0 * (dmeas / np.maximum(dmod, 1e-9) - self.kD[idx]), self.kD[idx])
            x = qm * np.sin(al_)
            y = lift + 0.1 * self.rng.standard_normal(len(idx))
            lam = 0.998
            use = ok & (al_ > 2 * D2R)
            self.rls_n[idx] = np.where(use, lam * self.rls_n[idx] + x * y, self.rls_n[idx])
            self.rls_d[idx] = np.where(use, lam * self.rls_d[idx] + x * x, self.rls_d[idx])
            # record at 1.5 km (task metric) and at the glide end / landing-burn ignition (450 m): stop there
            hn = nrm(pn) - R
            for tag, hx in (("", 1500.0), ("_ign", H_GLIDE_IGN)):
                hit = (hn <= hx) & (h > hx)
                if not hit.any():
                    continue
                w = ((h - hx) / np.maximum(h - hn, 1e-9))[hit][:, None]
                pe = p[hit] + w * (pn[hit] - p[hit])
                ve = v[hit] + w * (vn[hit] - v[hit])
                ii = idx[hit]
                al, cr = track_err(pe, self.phi_t[ii], self.lam_t[ii])
                vz = dot(ve, unit(pe))
                self.res["along" + tag][ii], self.res["cross" + tag][ii] = al, cr
                self.res["v" + tag][ii] = nrm(ve)
                self.res["vh" + tag][ii] = np.sqrt(np.maximum(nrm(ve) ** 2 - vz ** 2, 0))
                if tag:
                    self.active[ii] = False
            self.p[idx], self.v[idx] = pn, vn
            self.t += dt
        return self.res


def summary(law, res, qmax, extra=""):
    pc = lambda a, k: np.nanpercentile(np.abs(a), k)  # noqa: E731
    al, cr = res["along"], res["cross"]
    tot = np.hypot(al, cr)
    ti = np.hypot(res["along_ign"], res["cross_ign"])
    return ("%-4s| along %5.0f %5.0f | cross %5.0f %5.0f | miss %5.0f %5.0f | v %4.0f vh %3.0f || 450 m: miss %5.0f %5.0f"
            " <=100m %3.0f%% vh %3.0f | qmax %4.0f kPa%s" % (
                law, pc(al, 50), pc(al, 90), pc(cr, 50), pc(cr, 90), pc(tot, 50), pc(tot, 90), np.nanmedian(res["v"]),
                np.nanmedian(res["vh"]), pc(ti, 50), pc(ti, 90), 100 * np.mean(ti <= 100), np.nanmedian(res["vh_ign"]),
                np.nanmedian(qmax) / 1e3, extra))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("N", nargs="?", type=int, default=200)
    ap.add_argument("--laws", default="F,A,B,BL,C10,C3,C15")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--c0", type=float, default=C_NOM, help="controller prior for c (sign known)")
    ap.add_argument("--aim", default="law", choices=["law", "straight", "mid", "center"])
    ap.add_argument("--T", type=float, default=15.0)
    ap.add_argument("--along", type=float, default=3000.0, help="initial along-track error half-width (m)")
    ap.add_argument("--lag", type=float, default=1.0, help="attitude lag multiplier (tau = lag x U(1, 2) s)")
    a = ap.parse_args()
    D = draw(a.N, a.seed, a.along, a.lag)
    print("N=%d seed=%d   at 1.5 km: |along| |cross| |miss| median / 90th pct (m), speed / horizontal speed median"
          " (m/s); at 450 m (glide end = burn ignition): miss median / 90th, share <= 100 m, vh median" % (a.N, a.seed))
    for law in a.laws.split(","):
        t0 = time.time()
        s = Sim(law, D, c0=a.c0, aim=a.aim, T=a.T)
        res = s.run()
        ex = "  rev %.1f" % np.mean(s.n_rev) if law == "F" else ""
        print(summary(law, res, s.qmax, ex + "  [%.0fs]" % (time.time() - t0)), flush=True)


if __name__ == "__main__":
    main()
