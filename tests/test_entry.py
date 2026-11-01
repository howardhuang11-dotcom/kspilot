"""Offline tests for kspilot.entry / kspilot.deorbit (no game connection)."""
import math
import time

import numpy as np
import pytest
from scipy.integrate import solve_ivp

from kspilot import deorbit as D
from kspilot import entry as E

B = E.KERBIN
PAD = E.KSC_PAD


def circular_orbit_bf(alt, inc_deg, lon0, arg_deg=0.0):
    """Body-fixed kRPC state on a circular orbit, ascending node at (0, lon0) at t=0."""
    R, V = B.radius + alt, math.sqrt(B.mu / (B.radius + alt))
    e, n, u = E.enu_basis(0.0, lon0)
    h = math.cos(math.radians(inc_deg)) * e + math.sin(math.radians(inc_deg)) * n
    a = math.radians(arg_deg)
    r, v = R * (math.cos(a) * u + math.sin(a) * h), V * (-math.sin(a) * u + math.cos(a) * h)
    return E.inertial_to_bf(r, v, 0.0)


def entry_state(alt=70e3, speed=2250.0, fpa_deg=-5.0, lon=-90.0):
    r = E.latlonalt_to_bf(0.0, lon, alt)
    e, n, u = E.enu_basis(0.0, lon)
    g = math.radians(fpa_deg)
    return r, speed * (math.cos(g) * e + math.sin(g) * u)          # inertial velocity


# ----------------------------------------------------------------------------- frames
def test_latlonalt_roundtrip():
    rng = np.random.default_rng(1)
    for lat, lon, alt in zip(rng.uniform(-89, 89, 200), rng.uniform(-180, 180, 200), rng.uniform(-1e3, 1e5, 200)):
        la, lo, al = E.bf_to_latlonalt(E.latlonalt_to_bf(lat, lon, alt))
        assert la == pytest.approx(lat, abs=1e-9) and al == pytest.approx(alt, abs=1e-6)
        assert (lo - lon + 180) % 360 - 180 == pytest.approx(0, abs=1e-9)


def test_krpc_axes():
    """kRPC body frame: x -> (0,0), y -> north pole, z -> (0, 90E)."""
    R = B.radius
    np.testing.assert_allclose(E.latlonalt_to_bf(0, 0, 0), [R, 0, 0], atol=1e-6)
    np.testing.assert_allclose(E.latlonalt_to_bf(90, 0, 0), [0, R, 0], atol=1e-6)
    np.testing.assert_allclose(E.latlonalt_to_bf(0, 90, 0), [0, 0, R], atol=1e-6)
    p = E.latlonalt_to_bf(*PAD, 0)
    assert p[0] > 0 and p[2] < 0 and p[1] < 0                  # KSC: 74.6 deg W, just south of equator
    e, n, u = E.enu_basis(0, 0)
    np.testing.assert_allclose(e, [0, 0, 1], atol=1e-12)
    np.testing.assert_allclose(n, [0, 1, 0], atol=1e-12)
    # surface/air moves east; a fixed surface point's inertial track has that velocity
    np.testing.assert_allclose(E.air_velocity([R, 0, 0], B.omega), [0, 0, B.omega * R])
    r0, t, dt = E.latlonalt_to_bf(20, -40, 0), 1000.0, 1e-3
    fd = (E.rot_y(r0, B.omega * (t + dt)) - E.rot_y(r0, B.omega * (t - dt))) / (2 * dt)
    np.testing.assert_allclose(fd, E.air_velocity(E.rot_y(r0, B.omega * t), B.omega), atol=1e-6)


def test_bf_inertial_roundtrip_and_normal():
    r, v = np.array([5e5, 2e5, -3e5]), np.array([100.0, -50.0, 2000.0])
    ri, vi = E.bf_to_inertial(r, v, 1234.5)
    r2, v2 = E.inertial_to_bf(ri, vi, 1234.5)
    np.testing.assert_allclose(r2, r, atol=1e-6); np.testing.assert_allclose(v2, v, atol=1e-9)
    # a body-fixed point on the pad is stationary in the body frame for all t
    p = E.latlonalt_to_bf(*PAD, 0)
    pi, vpi = E.bf_to_inertial(p, np.zeros(3), 500.0)
    np.testing.assert_allclose(np.linalg.norm(vpi), B.omega * B.radius * math.cos(math.radians(PAD[0])), rtol=1e-12)
    # KSP "normal" of a prograde equatorial orbit points north (+y)
    rb, vb = circular_orbit_bf(80e3, 0, 0)
    ri, vi = E.bf_to_inertial(rb, vb)
    np.testing.assert_allclose(E._orbital_dir(tuple(ri) + tuple(vi), (0, 1, 0)), [0, 1, 0], atol=1e-12)


def test_ground_errors_signs():
    lat0, lon0 = PAD
    km = 1000.0 / B.radius / E.DEG
    a, c = E.ground_errors(lat0, lon0 + km, PAD, 90.0)          # 1 km east, heading east -> long
    assert a == pytest.approx(1000, rel=1e-3) and abs(c) < 1
    a, c = E.ground_errors(lat0 - km, lon0, PAD, 90.0)          # 1 km south, heading east -> right
    assert c == pytest.approx(1000, rel=1e-3) and abs(a) < 1


# ----------------------------------------------------------------------------- dynamics
def test_kepler_propagate_matches_integrator():
    rb, vb = circular_orbit_bf(80e3, 6, -100)
    ri, vi = E.bf_to_inertial(rb, vb)
    vi = vi * 1.05                                             # make it eccentric
    for dt in (100.0, 1500.0, -700.0):
        rk, vk = E.kepler_propagate(ri, vi, dt)
        sol = solve_ivp(lambda t, y: np.r_[y[3:], -B.mu * y[:3] / np.linalg.norm(y[:3]) ** 3],
                        (0, dt), np.r_[ri, vi], method="DOP853", rtol=1e-12, atol=1e-6)
        np.testing.assert_allclose(rk, sol.y[:3, -1], atol=0.05)
        np.testing.assert_allclose(vk, sol.y[3:, -1], atol=1e-4)


def test_vacuum_impact_matches_conic():
    """No drag: integrated impact == analytic conic + planet rotation."""
    for inc, dv in ((0, 150.0), (6, 300.0), (60, 250.0)):
        rb, vb = circular_orbit_bf(80e3, inc, -150, arg_deg=30)
        ri, vi = E.bf_to_inertial(rb, vb)
        vi = vi * (1 - dv / np.linalg.norm(vi))                # impulsive retro burn
        imp = E.predict_inertial(ri, vi, 1e4, drag=None)
        rt = B.radius + E.KSC_PAD_ALT
        tof, r_imp = E.conic_impact(ri, vi, rt)
        lat, lon, _ = E.bf_to_latlonalt(E.rot_y(r_imp, -B.omega * tof))
        assert imp.hit and imp.t == pytest.approx(tof, abs=0.01)
        assert E.great_circle(lat, lon, imp.lat, imp.lon) < 5.0


@pytest.mark.parametrize("cda", [1.0, 3.0])
def test_drag_matches_reference_integrator(cda):
    ri, vi = entry_state()
    drag, w = E.ballistic(cda), B.omega

    def f(t, y):
        r, v = y[:3], y[3:6]
        rn = np.linalg.norm(r)
        u = v - np.array([-w * r[2], 0, w * r[0]])
        a = -B.mu / rn ** 3 * r - drag(rn - B.radius, np.linalg.norm(u)) / y[6] * u / np.linalg.norm(u)
        return np.r_[v, a, 0.0]
    ev = lambda t, y: np.linalg.norm(y[:3]) - B.radius - E.KSC_PAD_ALT
    ev.terminal = True
    s = solve_ivp(f, (0, 3000), np.r_[ri, vi, 1e4], method="DOP853", rtol=1e-12, atol=1e-9, events=ev, max_step=5)
    t_ref, y_ref = s.t_events[0][0], s.y_events[0][0]
    rb, _ = E.inertial_to_bf(y_ref[:3], y_ref[3:6], t_ref)
    lat, lon, _ = E.bf_to_latlonalt(rb)
    imp = E.predict_inertial(ri, vi, 1e4, drag)
    assert imp.t == pytest.approx(t_ref, abs=0.01)
    assert E.great_circle(lat, lon, imp.lat, imp.lon) < 3.0


def test_drag_table():
    alts = np.arange(0, 70001, 2500.0)
    speeds = np.array([0, 50, 100, 200, 300, 400, 600, 800, 1200, 1600, 2400, 3000.0])
    cda = lambda v: 2.0 * (1.0 + 0.5 * np.exp(-((v - 340) / 150) ** 2))     # transonic bump
    F = np.array([[0.5 * B.density(a) * s * s * cda(s) for s in speeds] for a in alts])
    tab = E.DragTable(alts, speeds, F)
    for a, s in ((1234.0, 333.0), (40000.0, 1000.0), (69990.0, 2300.0)):    # CdA is interpolated
        assert tab(a, s) == pytest.approx(0.5 * B.density(a) * s * s * np.interp(s, speeds, cda(speeds)), rel=1e-9)
    assert tab(80e3, 2000) == 0.0
    flat = E.DragTable(alts, speeds, np.array([[0.5 * B.density(a) * s * s * 2.0 for s in speeds] for a in alts]))
    ri, vi = entry_state()
    a, b = E.predict_inertial(ri, vi, 1e4, flat), E.predict_inertial(ri, vi, 1e4, E.ballistic(2.0))
    assert E.great_circle(a.lat, a.lon, b.lat, b.lon) < 1.0
    raw = E.DragTable(alts, speeds, F, normalize=False)
    assert raw(alts[3], speeds[4]) == pytest.approx(F[3, 4])


def test_altitude_triggered_entry_burn():
    ri, vi = entry_state()
    m0, isp, dv = 1e4, 300.0, 300.0
    imp = E.predict_inertial(ri, vi, m0, E.ballistic(2.0),
                             [E.Burn(0.0, 200e3, isp, mode="air_retro", dv=dv, alt_trigger=40e3)])
    _, _, _, _, _, alt = imp.history_bf()
    i = int(np.argmax(imp.y_hist[:, 6] < m0 - 1e-9))
    assert abs(alt[i - 1] - 40e3) < 5.0
    assert imp.mass == pytest.approx(m0 * math.exp(-dv / (isp * E.G0)), rel=1e-6)


# ----------------------------------------------------------------------------- deorbit
CASES = [  # (inclination, CdA, mass)
    (0.0, 2.0, 10e3), (0.0, 1.0, 20e3), (6.0, 1.0, 5e3), (6.0, 3.0, 20e3)]


@pytest.mark.parametrize("inc,cda,mass", CASES)
def test_deorbit_hits_pad(inc, cda, mass):
    rb, vb = circular_orbit_bf(80e3, inc, -170.0)
    veh = D.Vehicle(mass, 200e3, 320.0, cda=cda)
    plan = D.plan_deorbit(rb, vb, veh)
    assert plan.impact.hit and plan.miss < 5.0
    # independent check: re-integrate the plan's burns from the original state (numerical coast)
    imp = E.predict(rb, vb, mass, cda=cda, burns=plan.burns)
    assert E.great_circle(imp.lat, imp.lon, *PAD) < 500.0
    assert imp.t == pytest.approx(plan.t_impact, abs=1.0)


def test_deorbit_aim_bias_and_entry_burn():
    rb, vb = circular_orbit_bf(80e3, 6.0, -120.0)
    veh = D.Vehicle(15e3, 250e3, 320.0, cda=2.0)
    eb = [E.Burn(0.0, 250e3, 320.0, mode="air_retro", dv=250.0, alt_trigger=35e3)]
    plan = D.plan_deorbit(rb, vb, veh, aim_bias=2000.0, entry_burns=eb)
    a, c = E.ground_errors(plan.impact.lat, plan.impact.lon, PAD, plan.heading)
    assert a == pytest.approx(2000.0, abs=5.0) and abs(c) < 5.0
    assert plan.impact.mass < veh.mass * math.exp(-plan.dv / (320 * E.G0)) * 0.95   # entry burn fired
    imp = E.predict(rb, vb, veh.mass, cda=2.0, burns=plan.burns)
    a2, c2 = E.ground_errors(imp.lat, imp.lon, PAD, plan.heading)
    assert math.hypot(a2 - 2000.0, c2) < 500.0


def test_impact_correction_reduces_miss():
    rb, vb = circular_orbit_bf(80e3, 0.0, -170.0)
    veh = D.Vehicle(10e3, 200e3, 320.0, cda=2.0)
    plan = D.plan_deorbit(rb, vb, veh)
    t, rh, vh, _, _, alt = plan.impact.history_bf()
    i = int(np.argmax(alt < 72e3))                       # state near the entry interface
    r, v = rh[i], vh[i] + np.array([3.0, -2.0, 4.0])     # dispersion
    m = plan.impact.y_hist[i, 6]
    corr = D.impact_correction(r, v, m, cda=2.0, heading=plan.heading)
    assert corr.miss > 1000.0
    after = D.impact_correction(r, v + corr.dv_bf, m, cda=2.0, heading=plan.heading)
    assert after.miss < 0.2 * corr.miss                  # linearised step
    dv = corr.dv_bf
    for _ in range(3):                                   # closed-loop style iteration converges
        dv = dv + after.dv_bf
        after = D.impact_correction(r, v + dv, m, cda=2.0, heading=plan.heading)
    assert after.miss < 50.0


# ----------------------------------------------------------------------------- timing
def _best_of(fn, n):
    ts = []
    for _ in range(n):
        t0 = time.perf_counter(); fn(); ts.append(time.perf_counter() - t0)
    return min(ts), float(np.median(ts))


def test_timing_entry_prediction():
    ri, vi = entry_state()
    tab = E.DragTable(np.arange(0, 70001, 2500.0), [0, 100, 300, 1000, 3000],
                      [[0.5 * B.density(a) * s * s * 2 for s in [0, 100, 300, 1000, 3000]] for a in np.arange(0, 70001, 2500.0)])
    for drag in (E.ballistic(2.0), tab):
        best, med = _best_of(lambda: E.predict_inertial(ri, vi, 1e4, drag), 10)
        print(f"\n70 km -> ground prediction: best {best*1e3:.1f} ms, median {med*1e3:.1f} ms")
        assert med < 0.05


def test_timing_deorbit_and_correction():
    rb, vb = circular_orbit_bf(80e3, 6.0, -170.0)
    veh = D.Vehicle(10e3, 200e3, 320.0, cda=2.0)
    t0 = time.perf_counter(); plan = D.plan_deorbit(rb, vb, veh); t_plan = time.perf_counter() - t0
    _, rh, vh, _, _, alt = plan.impact.history_bf()
    i = int(np.argmax(alt < 70e3))
    best, med = _best_of(lambda: D.impact_correction(rh[i], vh[i], 1e4, cda=2.0, heading=plan.heading), 5)
    print(f"\nplan_deorbit {t_plan:.2f} s; impact_correction median {med*1e3:.1f} ms")
    assert t_plan < 10.0 and med < 0.2
