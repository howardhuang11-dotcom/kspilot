"""Terminal rendezvous + docking: guidance law for an RCS chaser and a 6-DOF-lite plant with disturbances.

Scenario (R0, docking R0 results): the chaser is the recoverable core + cargo stack, flown from the
core's probe; the docking port is on the cargo module's nose. It starts at the natural V-bar hold point
~1 km behind the passive target (end of the Hohmann plan in orbit.py) and must dock with
    axial closing speed < 0.3 m/s, lateral miss < 0.1 m, misalignment < 5 deg   [ASSUMPTION: kRPC does not
    expose the stock capture limits; values chosen in docking concept study D8 / the R0 brief].

Frames. LVLH of the target (cw.py): x radial, y along-track, z orbit normal. The target port's outward
axis `a` is fixed in LVLH (target attitude held in the orbital frame for V-bar; inertial hold is enough
for H-bar since the orbit normal is inertially fixed). Approach frame A = (e1, e2, a); the chaser's body
frame nominally (x_b, y_b, z_b) = (e1, e2, -a), so its port (on +z_b) faces the target port.
q = chaser port - target port in A components: q[2] = s = axial separation, q[:2] = lateral offset.

Plant (truth): CW translational dynamics of the chaser CoM + RCS force; small-angle pitch/yaw attitude
dynamics with an in-game attitude controller (kRPC AutoPilot runs server-side each physics tick, so it has
no client latency) on reaction wheels with a torque limit; RCS translation torque from the thrust-centre /
CoM offset; port position = CoM + rotated arm (attitude errors move the port: L * psi).
Disturbances: per-direction thrust scale error, per-firing thrust noise, minimum input (deadzone) or on/off
thrusters, measurement age + command transport delay (kRPC round trips), navigation noise + bias,
target attitude tilt / wobble, unmodelled body torque. Monopropellant mass flow F / (Isp g0 eta).

Guidance (client side, runs every dt_ctrl on delayed, noisy telemetry, with optional latency compensation):
    TRANSFER  CW velocity-to-be-gained steering to the approach initiation point (AIP, s = s_aip on the
              port axis), re-solved every control tick (closed-loop two-impulse CW targeting)
    HOLD      velocity-command PD hold at the AIP until settled (go gate)
    APPROACH  axial closing-speed profile v = clip(s / tau_gs, v_final, v_max) inside a cone corridor,
              lateral PD to the axis; stop and settle at s_hold2 (10 m)
    FINAL     constant v_final along the axis; no-go gate at s_gate: lateral > gate_lat -> back off and retry
CW natural acceleration is cancelled by feed-forward from the measured CoM state.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field, replace

import numpy as np

from . import cw

G0 = 9.80665
N_100KM = cw.mean_motion_at(100e3)        # 3.21e-3 rad/s

# capture criteria [ASSUMPTION: not exposed by kRPC; stock defaults unverified -> calibrate before R3]
CAPTURE_AXIAL_MAX = 0.3       # m/s
CAPTURE_LATERAL_MAX = 0.1     # m
CAPTURE_ANGLE_MAX = 5.0       # deg

TRANSFER, HOLD, APPROACH, HOLD2, FINAL, BACKOFF, DONE = "transfer", "hold", "approach", "hold2", "final", "backoff", "done"


@dataclass
class DockParams:
    approach: str = "vbar"           # "vbar": port axis -V-bar (chaser behind); "hbar": +orbit normal
    n: float = N_100KM
    # vehicle (core + 2.48 t cargo + RCS kit, after the Hohmann)  [ASSUMPTION: masses from logs, inertia estimated]
    mass: float = 23.6e3             # kg
    inertia: float = 6.5e5           # kg m^2, transverse (stack ~18 m)
    f_lat: float = 2000.0            # N per lateral body axis per direction (2 RV-105 nozzles) [cfg: 1 kN/nozzle]
    f_ax: float = 4000.0             # N along the docking axis per direction (4 nozzles)
    thrust_scale: tuple = (1.0,) * 6  # true/nominal for (+x, -x, +y, -y, +z, -z) body directions
    thrust_noise: float = 0.0        # per-tick 1-sigma fractional thrust noise
    isp: float = 240.0               # s, RV-105 vacuum [cfg]
    eta: float = 0.9                 # geometric efficiency (cosine losses of canted nozzles) [ASSUMPTION]
    mono_kg: float = 400.0           # loaded monopropellant (large: the MC measures the need)
    cp_offset: tuple = (0.0, 0.0, 0.0)   # m, RCS thrust centre - CoM, body axes [ASSUMPTION]
    port_arm: tuple = (0.0, 0.0, 11.0)   # m, chaser port - CoM, body axes (port on the cargo nose)
    wheel_torque: float = 10e3       # N m per axis (one HECS2 core) [cfg: HECS2 10 kN m]
    att_wn: float = 1.0              # rad/s, in-game attitude loop natural frequency [ASSUMPTION]
    att_zeta: float = 0.8
    att_ki: float = 0.05             # integral gain fraction (x wn^3)
    att_cmd_tau: float = 30.0        # s, the client low-passes the target port axis before commanding it
                                     # (do not chase the target's SAS wobble: it swings our port by L * angle)
    torque_bias: tuple = (0.0, 0.0)  # N m unmodelled body torque
    # client loop (kRPC)
    dt_ctrl: float = 0.1             # s, guidance period
    tau_meas: float = 0.1            # s, telemetry age when the guidance reads it
    tau_cmd: float = 0.1             # s, command transport delay
    latency_comp: bool = True        # predict the state forward by the estimated latency
    tau_est_scale: float = 1.0       # estimated / true total latency
    u_min: float = 0.05              # |input| below this does not fire (deadzone / minimum impulse bit)
    onoff: bool = False              # thrusters only full on / off (worst case minimum impulse)
    nav_sig_p: float = 0.0           # m, white position noise per axis
    nav_sig_v: float = 0.0           # m/s
    nav_bias_p: tuple = (0.0, 0.0, 0.0)   # m, constant bias
    nav_sig_att: float = 0.0         # rad, in-game attitude sensing jitter
    nav_tau_p: float = 0.5           # s, navigation filter position time constant
    nav_tau_v: float = 1.0           # s, navigation filter velocity time constant
    # target
    target_arm: float = 2.0          # m, target port from target CoM along a
    target_tilt: tuple = (0.0, 0.0)  # rad, static tilt of the target port axis
    wobble_amp: float = 0.0          # rad, target attitude wobble (SAS limit cycle)
    wobble_period: float = 60.0      # s
    wobble_phase: float = 0.0
    # initial chaser CoM state, LVLH (m, m/s); default = Hohmann arrival 1 km behind
    r0: tuple = (0.0, -1000.0, 0.0)
    v0: tuple = (0.0, 0.0, 0.0)
    # guidance tuning
    s_aip: float = 50.0              # m, approach initiation point on the axis
    t_transfer: float = 900.0        # s, CW transfer time to the AIP
    s_hold2: float = 10.0
    tau_gs: float = 60.0             # s, closing-speed profile (s - s_hold2) / tau
    v_max: float = 0.5               # m/s, max closing speed inside the corridor
    v_final: float = 0.15            # m/s, contact speed command
    tau_lat: float = 16.0            # s, lateral position -> velocity
    tau_vel: float = 4.0             # s, velocity -> acceleration
    v_lat_max: float = 0.2
    db_v: float = 0.003              # m/s, velocity deadband
    db_p: float = 0.005              # m, lateral position deadband
    corridor_half: float = math.radians(8.0)
    corridor_min: float = 0.3
    s_gate: float = 2.0
    gate_lat: float = 0.06           # m, no-go if lateral exceeds this at s_gate
    gate_tau: float = 5.0            # s, averaging of the state the go/no-go gates judge
    max_retries: int = 2
    t_max: float = 4000.0
    dt: float = 0.05                 # s, plant step
    seed: int = 0


@dataclass
class DockResult:
    success: bool
    mode: str                        # "docked" or the failure mode
    t: float
    mono_kg: float
    dv: float                        # m/s delivered by RCS (|F|/m integrated)
    v_axial: float = math.nan
    lateral: float = math.nan
    angle_deg: float = math.nan
    v_lateral: float = math.nan
    retries: int = 0
    phase_times: dict = field(default_factory=dict)
    min_dist_off_axis: float = math.inf


def _cross(a, b):
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


def _frame(approach):
    if approach == "vbar":
        a = np.array([0.0, -1.0, 0.0])
        e1 = np.array([1.0, 0.0, 0.0])
    elif approach == "hbar":
        a = np.array([0.0, 0.0, 1.0])
        e1 = np.array([1.0, 0.0, 0.0])
    else:
        raise ValueError(approach)
    e2 = np.cross(a, e1)           # e1 x e2 = a  (A frame right-handed: (e1, e2, a))
    return e1, e2, a


def simulate(p: DockParams, trace=False):
    rng = np.random.default_rng(p.seed)
    e1, e2, a = _frame(p.approach)
    f = -a
    A = np.vstack([e1, e2, a])                 # LVLH -> A components
    B = np.vstack([e1, -e2, f])                # body axes (right-handed: x_b = e1, y_b = -e2, z_b = -a)
    n = p.n
    dt = p.dt
    m = p.mass
    mono = p.mono_kg
    r = np.array(p.r0, float)
    v = np.array(p.v0, float)
    psi = np.array(p.target_tilt, float)       # chaser attitude (rotation about e1, e2) from nominal
    phi_cmd = psi.copy()                       # attitude command: low-passed target port axis
    w = np.zeros(2)
    integ = np.zeros(2)
    arm_b = np.array(p.port_arm, float)
    arm_L0 = B.T @ arm_b
    cp_L = B.T @ np.array(p.cp_offset, float)
    scale = np.array(p.thrust_scale, float)
    f_nom = np.array([p.f_lat, p.f_lat, p.f_ax])
    ve = p.isp * G0 * p.eta
    tilt = np.array(p.target_tilt, float)
    bias = np.array(p.nav_bias_p, float)

    k_meas = max(0, int(round(p.tau_meas / dt)))
    hist = deque(maxlen=k_meas + 1)
    cmd_q = deque()
    u = np.zeros(3)
    resid = np.zeros(3)
    nav = None
    gate = None
    a_cmd_A = np.zeros(3)
    k_ctrl = max(1, int(round(p.dt_ctrl / dt)))
    tau_hat = (p.tau_meas + p.tau_cmd) * p.tau_est_scale

    phase, t_phase, settle = TRANSFER, 0.0, 0.0
    hold_s = p.s_aip
    retries = 0
    dv_acc = 0.0
    phase_times = {}
    min_off = math.inf
    out = []
    t = 0.0
    # chaser CoM goal for the transfer: port at s_aip on the axis
    def com_goal(s):
        return (p.target_arm + s) * a - arm_L0

    def target_att(tt):
        wob = p.wobble_amp * math.sin(2 * math.pi * tt / p.wobble_period + p.wobble_phase)
        wobd = p.wobble_amp * 2 * math.pi / p.wobble_period * math.cos(2 * math.pi * tt / p.wobble_period + p.wobble_phase)
        return tilt + np.array([wob, 0.7 * wob]), np.array([wobd, 0.7 * wobd])

    step = 0
    result = None
    while t < p.t_max:
        # ---------------- truth geometry
        phi, phid = target_att(t)
        phi_v = phi[0] * e1 + phi[1] * e2
        a_act = a + _cross(phi_v, a)
        p_t = p.target_arm * a_act
        vp_t = p.target_arm * _cross(phid[0] * e1 + phid[1] * e2, a)
        psi_v = psi[0] * e1 + psi[1] * e2
        arm_L = arm_L0 + _cross(psi_v, arm_L0)
        p_c = r + arm_L
        vp_c = v + _cross(w[0] * e1 + w[1] * e2, arm_L0)
        d = p_c - p_t
        q = A @ d
        qd = A @ (vp_c - vp_t)
        s_true = float(np.dot(d, a_act))
        theta = psi - phi                       # pointing error w.r.t. the target port axis (capture check)
        phi_cmd = phi_cmd + (phi - phi_cmd) * min(1.0, dt / p.att_cmd_tau)   # slow attitude command
        lat = math.hypot(q[0], q[1])
        dist_com = float(np.linalg.norm(r))
        if lat > 2.0 and dist_com < 15.0:
            min_off = min(min_off, dist_com)
        if s_true <= 0.0 and lat < 1.0 and phase in (APPROACH, HOLD2, FINAL, BACKOFF):
            ang = math.degrees(math.hypot(theta[0], theta[1]))
            res = dict(v_axial=-float(qd[2]), lateral=lat, angle_deg=ang, v_lateral=math.hypot(qd[0], qd[1]))
            ok = (res['v_axial'] < CAPTURE_AXIAL_MAX and lat < CAPTURE_LATERAL_MAX and ang < CAPTURE_ANGLE_MAX)
            if ok:
                mode = "docked"
            elif lat >= CAPTURE_LATERAL_MAX:
                mode = "lateral_miss"
            elif res['v_axial'] >= CAPTURE_AXIAL_MAX:
                mode = "axial_speed"
            else:
                mode = "misalignment"
            result = DockResult(ok, mode, t, p.mono_kg - mono, dv_acc, retries=retries,
                                phase_times=phase_times, min_dist_off_axis=min_off, **res)
            break
        if min_off < 10.0:
            result = DockResult(False, "keep_out", t, p.mono_kg - mono, dv_acc, retries=retries,
                                phase_times=phase_times, min_dist_off_axis=min_off)
            break
        hist.append((q.copy(), qd.copy(), r.copy(), v.copy(), psi.copy()))

        # ---------------- guidance (client) at its own rate, on delayed telemetry
        if step % k_ctrl == 0:
            qm, qdm, rm, vm, psim = hist[0]
            qm = qm + bias + rng.normal(0, p.nav_sig_p, 3) if p.nav_sig_p > 0 else qm + bias
            rm = rm + (A.T @ bias) + (rng.normal(0, p.nav_sig_p, 3) if p.nav_sig_p > 0 else 0.0)
            if p.nav_sig_v > 0:
                qdm = qdm + rng.normal(0, p.nav_sig_v, 3)
                vm = vm + rng.normal(0, p.nav_sig_v, 3)
            # navigation filter: model prediction (last commanded acceleration) blended with the measurement
            if nav is None:
                nav = [qm, qdm, rm, vm]
            else:
                h = p.dt_ctrl
                aL = A.T @ a_cmd_A
                pred = [nav[0] + nav[1] * h + 0.5 * a_cmd_A * h * h, nav[1] + a_cmd_A * h,
                        nav[2] + nav[3] * h + 0.5 * aL * h * h, nav[3] + aL * h]
                ka, kb = min(1.0, h / p.nav_tau_p), min(1.0, h / p.nav_tau_v)
                nav = [pred[0] + ka * (qm - pred[0]), pred[1] + kb * (qdm - pred[1]),
                       pred[2] + ka * (rm - pred[2]), pred[3] + kb * (vm - pred[3])]
            qm, qdm, rm, vm = nav
            if p.latency_comp:
                qm = qm + qdm * tau_hat + 0.5 * a_cmd_A * tau_hat ** 2
                qdm = qdm + a_cmd_A * tau_hat
                aL = A.T @ a_cmd_A
                rm = rm + vm * tau_hat + 0.5 * aL * tau_hat ** 2
                vm = vm + aL * tau_hat
            a_ff = A @ cw.natural_accel(n, rm, vm)
            # go/no-go gates judge a time-averaged state (noise must not reset a settle timer)
            kg = min(1.0, p.dt_ctrl / p.gate_tau)
            gate = (qm.copy(), qdm.copy()) if gate is None else (gate[0] + kg * (qm - gate[0]),
                                                                 gate[1] + kg * (qdm - gate[1]))
            t_phase += p.dt_ctrl
            a_g, phase, t_phase, settle, hold_s, retries, failed = _guide(
                p, phase, t_phase, settle, hold_s, retries, t, qm, qdm, rm, vm, com_goal, A, gate)
            if failed:
                result = DockResult(False, failed, t, p.mono_kg - mono, dv_acc, retries=retries,
                                    phase_times=phase_times, min_dist_off_axis=min_off)
                break
            if phase not in phase_times:
                phase_times[phase] = t
            # TRANSFER steers onto the natural CW arc (no cancellation); proximity phases cancel CW drift
            a_tot = a_g if phase == TRANSFER else a_g - a_ff
            fb = m * (B @ (A.T @ a_tot))           # body force request
            uc = np.clip(fb / f_nom, -1.0, 1.0)
            uc, resid = _modulate(uc, resid, 1.0 if p.onoff else p.u_min, p.onoff)
            cmd_q.append((t + p.tau_cmd, uc))
            a_cmd_A = A @ (B.T @ (uc * f_nom)) / m + a_ff
        while cmd_q and cmd_q[0][0] <= t + 1e-9:
            u = cmd_q.popleft()[1]

        # ---------------- plant
        if mono > 0:
            sc = np.where(u >= 0, scale[[0, 2, 4]], scale[[1, 3, 5]])
            fbody = u * f_nom * sc
            if p.thrust_noise > 0:
                fbody = fbody * (1 + rng.normal(0, p.thrust_noise, 3))
        else:
            fbody = np.zeros(3)
        fmag = float(np.sum(np.abs(fbody)))
        mdot = fmag / ve
        mono -= mdot * dt
        m -= mdot * dt
        dv_acc += fmag / m * dt
        F_L = B.T @ fbody
        F_L = F_L + _cross(psi_v, F_L)
        acc = cw.natural_accel(n, r, v) + F_L / m
        v = v + acc * dt
        r = r + v * dt
        # attitude: torque from the thrust-centre offset, wheels track the target port axis
        tq = _cross(cp_L, B.T @ fbody)
        tau_d = np.array([np.dot(tq, e1), np.dot(tq, e2)]) + np.array(p.torque_bias)
        th_m = psi - phi_cmd + (rng.normal(0, p.nav_sig_att, 2) if p.nav_sig_att > 0 else 0.0)
        integ = integ + th_m * dt
        I = p.inertia
        wn, z = p.att_wn, p.att_zeta
        tau_c = -I * (wn * wn * th_m + 2 * z * wn * w + p.att_ki * wn ** 3 * integ)
        tau_c = np.clip(tau_c, -p.wheel_torque, p.wheel_torque)
        w = w + (tau_c + tau_d) / I * dt
        psi = psi + w * dt
        if trace and step % 20 == 0:
            out.append((t, phase, q[0], q[1], q[2], qd[2], mono, *psi))
        t += dt
        step += 1
    if result is None:
        result = DockResult(False, "timeout", t, p.mono_kg - mono, dv_acc, retries=retries,
                            phase_times=phase_times, min_dist_off_axis=min_off)
    if mono <= 0 and not result.success:
        result.mode = "propellant"
    if trace:
        return result, out
    return result


def _modulate(uc, resid, u_min, onoff):
    """Pulse modulator for the minimum impulse bit: inputs >= u_min pass (proportional thrusters; on/off ones
    fire full), smaller requests accumulate and fire one u_min pulse per control tick when the accumulated
    impulse reaches it (sigma-delta), so the mean thrust matches the request."""
    out = np.zeros(3)
    for i in range(3):
        x = uc[i]
        if abs(x) >= u_min or (onoff and abs(x) >= 0.5):
            out[i] = math.copysign(1.0, x) if onoff else x
            resid[i] = 0.0
        else:
            resid[i] += x
            if abs(resid[i]) >= u_min:
                out[i] = math.copysign(u_min, resid[i])
                resid[i] -= out[i]
            elif x == 0.0:
                resid[i] = 0.0          # inside the guidance deadband: forget the residual
    return out, resid


def _vel_cmd(err, tau, vmax):
    return -max(-vmax, min(vmax, err / tau))


def _guide(p, phase, t_phase, settle, hold_s, retries, t, q, qd, r, v, com_goal, A, gate):
    """One guidance tick. Returns (a_cmd in A components, phase, t_phase, settle, hold_s, retries, fail)."""
    fail = None
    acc = np.zeros(3)
    gq, gqd = gate
    lat = math.hypot(gq[0], gq[1])
    vlat = math.hypot(gqd[0], gqd[1])
    if phase == TRANSFER:
        t_go = p.t_transfer - t_phase
        if t_go < 30.0:
            phase, t_phase, settle = HOLD, 0.0, 0.0
        else:
            v_req = cw.required_velocity(p.n, r, com_goal(p.s_aip), t_go)
            dvL = v_req - v
            dvA = A @ dvL
            acc = np.where(np.abs(dvA) > p.db_v * 3, dvA / p.tau_vel, 0.0)
            return acc, phase, t_phase, settle, hold_s, retries, fail
    lat_v = np.array([_vel_cmd(q[0], p.tau_lat, p.v_lat_max) if abs(q[0]) > p.db_p else 0.0,
                      _vel_cmd(q[1], p.tau_lat, p.v_lat_max) if abs(q[1]) > p.db_p else 0.0])
    if phase in (HOLD, HOLD2, BACKOFF):
        tgt = hold_s if phase != HOLD2 else p.s_hold2
        v_ax = _vel_cmd(q[2] - tgt, p.tau_lat, p.v_max if phase != BACKOFF else 0.3)
        vtol = 0.02 if phase == HOLD else 0.01
        good = (lat < (0.2 if phase == HOLD else 0.04) and abs(gq[2] - tgt) < (1.0 if phase == HOLD else 0.3)
                and vlat < vtol and abs(gqd[2]) < vtol)
        settle = settle + p.dt_ctrl if good else 0.0
        if settle >= 20.0:
            phase, t_phase, settle = (FINAL if phase == HOLD2 else APPROACH), 0.0, 0.0
        elif t_phase > 900.0:
            fail = "hold_timeout"
    elif phase == APPROACH:
        s = q[2]
        if s <= p.s_hold2 + 0.2:
            phase, t_phase, settle = HOLD2, 0.0, 0.0
        if lat > p.corridor_min + math.tan(p.corridor_half) * max(s, 0.0):
            phase, t_phase, settle, retries, hold_s = _backoff(p, s, retries)
            if retries > p.max_retries:
                fail = "corridor"
        v_ax = -min(p.v_max, max(0.05, (s - p.s_hold2) / p.tau_gs))
    elif phase == FINAL:
        s = q[2]
        v_ax = -p.v_final
        if s < p.s_gate and math.hypot(q[0], q[1]) > p.gate_lat and qd[2] < 0:
            phase, t_phase, settle, retries, hold_s = _backoff(p, s, retries)
            phase = HOLD2 if retries <= p.max_retries else phase
            if retries > p.max_retries:
                fail = "gate_no_go"
            v_ax = 0.0
    else:
        v_ax = 0.0
    vcmd = np.array([lat_v[0], lat_v[1], v_ax])
    err = vcmd - qd
    acc = np.where(np.abs(err) > p.db_v, err / p.tau_vel, 0.0)
    return acc, phase, t_phase, settle, hold_s, retries, fail


def _backoff(p, s, retries):
    return BACKOFF, 0.0, 0.0, retries + 1, max(p.s_hold2, min(p.s_aip, s + 10.0))


# --------------------------------------------------------------------------- Monte Carlo dispersions
def sample(rng, base: DockParams = None, **over):
    """Draw one dispersed case. Ranges are ASSUMPTIONS (see docking R0 results section 2)."""
    b = base or DockParams()
    u = rng.uniform
    m = u(22.8e3, 24.4e3)
    L = u(16.0, 22.0)
    tau_m, tau_c = u(0.05, 0.25), u(0.05, 0.25)
    kw = dict(
        mass=m, inertia=m * L * L / 12.0,
        thrust_scale=tuple(u(0.9, 1.1, 6)), thrust_noise=0.02,
        eta=u(0.85, 0.95),
        cp_offset=(u(-0.15, 0.15), u(-0.15, 0.15), u(-1.5, 1.5)),
        port_arm=(u(-0.05, 0.05), u(-0.05, 0.05), u(9.0, 14.0)),
        att_wn=u(0.5, 1.5), att_zeta=u(0.6, 1.0),
        torque_bias=tuple(rng.normal(0, 200.0, 2)),
        dt_ctrl=u(0.1, 0.25), tau_meas=tau_m, tau_cmd=tau_c, tau_est_scale=u(0.7, 1.3),
        u_min=u(0.02, 0.1),
        nav_sig_p=0.02, nav_sig_v=0.005, nav_bias_p=tuple(rng.normal(0, 0.01, 3)),
        nav_sig_att=math.radians(0.05),
        target_arm=u(1.0, 3.0), target_tilt=tuple(rng.normal(0, math.radians(0.5), 2)),
        wobble_amp=u(0, math.radians(0.3)), wobble_period=u(20, 120), wobble_phase=u(0, 2 * math.pi),
        r0=(rng.normal(0, 20.0), -1000.0 + rng.normal(0, 50.0), rng.normal(0, 10.0)),
        v0=(rng.normal(0, 0.1), rng.normal(0, 0.1), rng.normal(0, 0.05)),
        seed=int(rng.integers(0, 2 ** 31)),
    )
    kw.update(over)
    return replace(b, **kw)
