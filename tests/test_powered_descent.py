"""Offline tests for kspilot.powered_descent: open-loop G-FOLD plans, closed-loop 3-DOF sims, fallback."""
import math
import os
import sys
from dataclasses import replace

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot import atmo, powered_descent as pd  # noqa: E402

G = atmo.gravity(atmo.R + pd.PAD_ALT)
CDA = 2.0  # guidance drag model, m^2; the "true" vehicle below has +10 %

# name, m0, m_dry, TWR, r0, v0
CASES = [
    ("15t_twr3", 15e3, 11e3, 3.0, (300, 0, 3000), (30, 0, -250)),
    ("25t_twr2.5", 25e3, 19e3, 2.5, (-200, 250, 3000), (10, -30, -230)),
    ("10t_twr4", 10e3, 7e3, 4.0, (0, -300, 3000), (-20, 25, -250)),
    ("30t_twr2", 30e3, 22e3, 2.0, (250, 100, 3000), (-25, 0, -200)),
]


def make_cfg(m0, m_dry, twr, **kw):
    kw = dict(t_max=twr * m0 * G, isp=300.0, m_dry=m_dry, throttle=(0.3, 1.0), throttle_soft=0.9, g=G,
              cda=CDA) | kw
    return pd.PDGConfig(**kw)


def simulate(ctrl, r0, v0, m0, cfg, thrust_err=0.0, cda=1.1 * CDA, tau_thr=0.15, tau_dir=0.25,
             f_ctrl=20.0, dt=0.005, t_end=120.0):
    """3-DOF point mass over a flat pad frame: gravity(h), tabulated density drag, first-order
    throttle and attitude lag, constant thrust scale error. ctrl(t, r, v, m) -> (throttle, dir)."""
    r, v, m = np.array(r0, float), np.array(v0, float), float(m0)
    thr, d = 0.0, np.array([0.0, 0.0, 1.0])
    cmd_thr, cmd_dir = 0.0, d.copy()
    alpha = 1.0 / (cfg.isp * atmo.G0)
    every = int(round(1.0 / (f_ctrl * dt)))

    def deriv(r, v, m):
        T = cfg.t_max * thr * (1 + thrust_err) if m > cfg.m_dry else 0.0
        rho = atmo.density(pd.PAD_ALT + max(r[2], 0.0))
        a = T / m * d - 0.5 * rho * cda * np.linalg.norm(v) * v / m
        a[2] -= atmo.gravity(atmo.R + pd.PAD_ALT + r[2])
        return v, a, -alpha * T

    t, k = 0.0, 0
    while t < t_end:
        if k % every == 0:
            cmd_thr, cmd_dir = ctrl(t, r, v, m)
        thr += (cmd_thr - thr) * min(dt / tau_thr, 1.0)
        d += (np.asarray(cmd_dir) - d) * min(dt / tau_dir, 1.0)
        d /= np.linalg.norm(d)
        k1 = deriv(r, v, m)
        k2 = deriv(r + dt / 2 * k1[0], v + dt / 2 * k1[1], m + dt / 2 * k1[2])
        k3 = deriv(r + dt / 2 * k2[0], v + dt / 2 * k2[1], m + dt / 2 * k2[2])
        k4 = deriv(r + dt * k3[0], v + dt * k3[1], m + dt * k3[2])
        rn = r + dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        vn = v + dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        m += dt / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2])
        if rn[2] <= 0:  # touchdown: interpolate to the ground plane
            s = r[2] / (r[2] - rn[2])
            r, v = r + s * (rn - r), v + s * (vn - v)
            return dict(t=t + s * dt, landed=True, r=r, v=v, miss=float(np.linalg.norm(r[:2])),
                        speed=float(np.linalg.norm(v)), vz=float(v[2]), vh=float(np.linalg.norm(v[:2])),
                        fuel=m0 - m, tilt=math.degrees(math.acos(min(d[2], 1.0))))
        r, v, t, k = rn, vn, t + dt, k + 1
    return dict(t=t, r=r, v=v, landed=False)


class _Controller:
    """Shared sensing: drag model and online thrust-scale estimate from successive velocity samples."""

    def __init__(self, cfg):
        self.cfg, self.est, self.prev = cfg, pd.ThrustEstimator(), None

    def drag(self, r, v):
        return 0.5 * atmo.density(pd.PAD_ALT + max(r[2], 0.0)) * self.cfg.cda * float(np.dot(v, v))

    def sense(self, t, r, v, m):
        """Update the estimator; returns the config with the estimated thrust."""
        if self.prev is not None:
            t0, v0, a_cmd = self.prev
            dt = t - t0
            vm = (v + v0) / 2
            a = (v - v0) / dt + np.array([0, 0, atmo.gravity(atmo.R + pd.PAD_ALT + r[2])])
            a += self.drag(r, vm) / m * vm / max(np.linalg.norm(vm), 1e-6)
            self.est.update(a, a_cmd, dt)
        return replace(self.cfg, t_max=self.cfg.t_max * self.est.eta)

    def out(self, t, v, m, cmd):
        self.prev = (t, np.array(v), self.cfg.t_max * cmd[0] * np.asarray(cmd[1]) / m)
        return cmd


class GfoldController(_Controller):
    """Replan at 1 Hz (warm-started) to a gate h_gate above the pad, track the plan at 20 Hz, then
    descend at v_td with the terminal law (which may throttle below the planning floor)."""

    def __init__(self, cfg, t_replan=1.0, t_freeze=2.0, h_gate=4.0):
        super().__init__(cfg)
        self.t_replan, self.t_freeze, self.gate = t_replan, t_freeze, np.array([0.0, 0.0, h_gate])
        self.plan, self.t_plan, self.t_try = None, 0.0, -math.inf
        self.solve_times, self.failures, self.over = [], 0, 0
        self.terminal = pd.HoverslamParams(v_td=cfg.v_td, h_stop=h_gate)

    def __call__(self, t, r, v, m):
        cfg = self.sense(t, r, v, m)
        tp, rg = t - self.t_plan, r - self.gate
        due = self.plan is None or self.plan.tf - tp > self.t_freeze
        if due and t - self.t_try >= self.t_replan - 1e-9:
            self.t_try = t
            p = pd.solve(rg, v, m, cfg, ref=self.plan, t_shift=tp)
            self.solve_times.append(p.solve_time)
            self.over += p.ok and np.linalg.norm(p.thrust, axis=1).max() > (cfg.throttle_soft + .01) * cfg.t_max
            if p.ok:
                self.plan, self.t_plan, tp = p, t, 0.0
            else:
                self.failures += 1
        if self.plan is not None and tp < self.plan.tf:
            cmd = pd.track(self.plan, tp, rg, v, m, cfg)
        else:  # no plan (solver failure) or plan finished: hoverslam / terminal descent law
            cmd = pd.hoverslam_command(r, v, m, cfg.t_max, cfg.g, self.drag(r, v), self.terminal)
        return self.out(t, v, m, cmd)


class HoverslamController(_Controller):
    """Fallback only: coast until burn-now, then the hoverslam throttle/tilt law."""

    def __init__(self, cfg, p=pd.HoverslamParams()):
        super().__init__(cfg)
        self.p, self.lit = p, False

    def __call__(self, t, r, v, m):
        cfg, drag = self.sense(t, r, v, m), self.drag(r, v)
        if not self.lit:
            self.lit = pd.hoverslam_burn_now(r[2], v[2], m, cfg.t_max, cfg.g, cfg.cda, cfg.isp, self.p,
                                             r[:2], v[:2])[0]
        if not self.lit:
            return self.out(t, v, m, (0.0, np.array([0.0, 0.0, 1.0])))
        return self.out(t, v, m, pd.hoverslam_command(r, v, m, cfg.t_max, cfg.g, drag, self.p))


# ----------------------------------------------------------------------------- tests

def test_atmo():
    assert atmo.density(0) == pytest.approx(atmo.KERBIN["density"][0])
    assert atmo.density(70000) == 0.0 and atmo.density(90000) == 0.0
    assert 0 < atmo.density(4250) < atmo.density(4000)
    assert atmo.gravity(atmo.R) == pytest.approx(atmo.KERBIN["surface_gravity"], rel=1e-3)


@pytest.mark.parametrize("name,m0,m_dry,twr,r0,v0", CASES, ids=[c[0] for c in CASES])
def test_open_loop_plan(name, m0, m_dry, twr, r0, v0):
    cfg = make_cfg(m0, m_dry, twr)
    p = pd.solve(r0, v0, m0, cfg)
    print(f"\n{name}: tf={p.tf:.2f}s fuel={p.fuel:.0f}kg miss={p.miss:.1e}m v_err={p.v_err:.1e}m/s "
          f"solves={p.n_solves} time={p.solve_time * 1e3:.0f}ms")
    assert p.exact and p.miss < 1.0 and np.linalg.norm(p.v[-1]) < 2.0 and abs(p.r[-1, 2]) < 1e-6
    thr = np.linalg.norm(p.thrust, axis=1) / cfg.t_max
    assert thr.min() > cfg.throttle[0] - 1e-3 and thr.max() < cfg.throttle_soft + 1e-3  # lossless
    tilt = np.degrees(np.arccos(p.u[:, 2] / np.linalg.norm(p.u, axis=1)))
    assert tilt.max() < cfg.tilt_max_deg + 0.1
    gs = math.tan(math.radians(cfg.glideslope_deg)) * np.linalg.norm(p.r[1:-1, :2], axis=1)
    assert np.all(p.r[1:-1, 2] >= gs - 1e-3) and p.m[-1] >= m_dry


def test_plan_matches_true_dynamics():
    """Replaying the plan's thrust open-loop (ideal engine, quadratic drag) reproduces the plan."""
    name, m0, m_dry, twr, r0, v0 = CASES[0]
    cfg = make_cfg(m0, m_dry, twr)
    p = pd.solve(r0, v0, m0, cfg)

    def replay(t, r, v, m):
        u = p.at(t)[2]
        return np.linalg.norm(u) * m / cfg.t_max, u / np.linalg.norm(u)

    res = simulate(replay, r0, v0, m0, cfg, cda=cfg.cda, tau_thr=1e-9, tau_dir=1e-9, f_ctrl=200.0,
                   t_end=p.tf - 1e-6)
    err_r, err_v = np.linalg.norm(res["r"] - p.r[-1]), np.linalg.norm(res["v"] - p.v[-1])
    print(f"\nopen-loop replay at tf: |dr|={err_r:.2f}m |dv|={err_v:.2f}m/s")
    assert err_r < 2.0 and err_v < 1.0


def test_solve_timing():
    name, m0, m_dry, twr, r0, v0 = CASES[0]
    cfg = make_cfg(m0, m_dry, twr)
    p = pd.solve(r0, v0, m0, cfg)
    r1, v1, _ = p.at(1.0)
    m1 = float(np.interp(1.0, p.t, p.m))
    warm = [pd.solve(r1, v1, m1, cfg, ref=p, t_shift=1.0) for _ in range(5)]
    per = np.median([w.solve_time / w.n_solves for w in warm])
    print(f"\ncold {p.solve_time * 1e3:.0f}ms ({p.n_solves} solves), warm median "
          f"{np.median([w.solve_time for w in warm]) * 1e3:.0f}ms ({warm[0].n_solves} solves), "
          f"{per * 1e3:.1f}ms per convex solve (N={cfg.n})")
    assert all(w.exact for w in warm) and np.median([w.solve_time for w in warm]) < 0.3


def test_unreachable_state_reported():
    """Below the glideslope cone: no feasible plan, caller must use the fallback."""
    name, m0, m_dry, twr, _, _ = CASES[0]
    p = pd.solve((600, 0, 50), (0, 0, -20), m0, make_cfg(m0, m_dry, twr))
    assert not p.ok and not p.exact


@pytest.mark.parametrize("name,m0,m_dry,twr,r0,v0", CASES, ids=[c[0] for c in CASES])
def test_burn_now_constant_throttle_stop(name, m0, m_dry, twr, r0, v0):
    """Igniting on hoverslam_burn_now (20 Hz) and holding 87 % throttle stops between h_stop and
    h_stop + one control period of travel (tilt/drag margins off, drag model = truth)."""
    cfg, hp = make_cfg(m0, m_dry, twr), pd.HoverslamParams(tilt_reserve_deg=0.0, drag_frac=1.0, t_delay=0.05)
    state = dict(lit=False, h_stop=None)

    def ctrl(t, r, v, m):
        if not state["lit"]:
            state["lit"] = pd.hoverslam_burn_now(r[2], v[2], m, cfg.t_max, cfg.g, cfg.cda, cfg.isp, hp)[0]
        if state["lit"] and v[2] >= 0 and state["h_stop"] is None:
            state["h_stop"] = r[2]
        return (hp.throttle if state["lit"] and state["h_stop"] is None else 0.0), np.array([0.0, 0.0, 1.0])

    simulate(ctrl, (0, 0, 3000), (0, 0, v0[2]), m0, cfg, cda=cfg.cda)
    print(f"\n{name}: stopped at h={state['h_stop']:.1f}m")
    assert hp.h_stop - 2.0 < state["h_stop"] < hp.h_stop + 0.05 * abs(v0[2]) * 1.2 + 2.0


@pytest.mark.parametrize("err", [-0.05, 0.05])
@pytest.mark.parametrize("name,m0,m_dry,twr,r0,v0", CASES, ids=[c[0] for c in CASES])
def test_closed_loop_gfold(name, m0, m_dry, twr, r0, v0, err):
    cfg = make_cfg(m0, m_dry, twr)
    ctrl = GfoldController(cfg)
    res = simulate(ctrl, r0, v0, m0, cfg, thrust_err=err)
    print(f"\nG-FOLD {name} err={err:+.0%}: miss={res['miss']:.2f}m vz={res['vz']:.2f} vh={res['vh']:.2f}m/s "
          f"fuel={res['fuel']:.0f}kg t={res['t']:.1f}s replans={len(ctrl.solve_times)} fails={ctrl.failures} "
          f"above_soft={ctrl.over} solve max={max(ctrl.solve_times[1:]) * 1e3:.0f}ms eta={ctrl.est.eta:.3f}")
    assert res["landed"] and res["miss"] < 1.0 and res["speed"] < 2.0 and res["tilt"] < 5.0


@pytest.mark.parametrize("err", [-0.05, 0.05])
@pytest.mark.parametrize("name,m0,m_dry,twr,r0,v0", CASES, ids=[c[0] for c in CASES])
def test_closed_loop_fallback(name, m0, m_dry, twr, r0, v0, err):
    cfg = make_cfg(m0, m_dry, twr)
    res = simulate(HoverslamController(cfg), r0, v0, m0, cfg, thrust_err=err)
    print(f"\nhoverslam {name} err={err:+.0%}: miss={res['miss']:.2f}m vz={res['vz']:.2f} "
          f"vh={res['vh']:.2f}m/s fuel={res['fuel']:.0f}kg t={res['t']:.1f}s")
    assert res["landed"] and res["speed"] < 2.0 and res["miss"] < 10.0 and res["tilt"] < 5.0
