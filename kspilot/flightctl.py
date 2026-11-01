"""Low-level flight control shared by hover tests and powered descent.

PadFrame: local East-North-Up frame at a ground target (built numerically from
kRPC surface positions, so it is independent of kRPC's frame handedness).
Actuator: turns a commanded acceleration vector (ENU, m/s^2, gravity
included by the caller) into throttle + autopilot pointing.
"""
import math

import numpy as np


def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


class PadFrame:
    def __init__(self, body, lat, lon):
        self.body = body
        self.rf = body.reference_frame
        p = np.array(body.surface_position(lat, lon, self.rf))
        d = 1e-4
        e = np.array(body.surface_position(lat, lon + d, self.rf)) - p
        n = np.array(body.surface_position(lat + d, lon, self.rf)) - p
        self.origin = p
        self.u = unit(p)
        self.e = unit(e - np.dot(e, self.u) * self.u)
        self.n = unit(n - np.dot(n, self.u) * self.u - np.dot(n, self.e) * self.e)
        self.M = np.vstack([self.e, self.n, self.u])   # body -> ENU rows

    def pos(self, p_body):
        return self.M @ (np.asarray(p_body) - self.origin)

    def vec(self, v_body):
        return self.M @ np.asarray(v_body)

    def to_body(self, v_enu):
        return self.M.T @ np.asarray(v_enu)


class Actuator:
    """Acceleration command -> throttle and attitude (kRPC autopilot)."""

    def __init__(self, conn, vessel, frame, max_tilt_deg=25.0):
        self.v = vessel
        self.frame = frame
        self.max_tilt = math.radians(max_tilt_deg)
        ap = vessel.auto_pilot
        ap.reference_frame = frame.rf
        import os as _os
        _t = float(_os.environ.get("KSP_TTP", 1.0))
        ap.time_to_peak = (_t, _t, _t)
        ap.max_angular_velocity = (0.35, 0.35, 0.35)   # rad/s: no violent slews under full gimbal authority
        ap.engaged = True
        self.ap = ap
        self.mass = conn.add_stream(getattr, vessel, 'mass')
        self.thrust = conn.add_stream(getattr, vessel, 'available_thrust')
        self.nose = conn.add_stream(vessel.direction, frame.rf)

    def command(self, a_enu, roll=None):
        """a_enu: desired thrust acceleration (m/s^2) in ENU. Returns throttle."""
        a = np.asarray(a_enu, float)
        if a[2] < 0.1:
            a[2] = 0.1
        tilt = math.atan2(math.hypot(a[0], a[1]), a[2])
        if tilt > self.max_tilt:
            h = math.hypot(a[0], a[1])
            s = a[2] * math.tan(self.max_tilt) / h
            a[0] *= s
            a[1] *= s
        d = unit(self.frame.to_body(a))
        self.ap.target_direction = tuple(d)
        tmax = self.thrust()
        if tmax <= 0:
            return 0.0
        # thrust only acts along the current nose: project the command onto it
        along = float(np.dot(np.asarray(self.nose()), d))
        thr = self.mass() * np.linalg.norm(a) / tmax / max(along, 0.5)
        thr = min(max(thr, 0.0), 1.0)
        self.v.control.throttle = thr
        return thr

    def release(self):
        self.v.control.throttle = 0.0
        self.ap.engaged = False


def set_thrust(act, throttle, dir_enu, max_rate_deg=20.0):
    """Point along dir_enu (ENU unit vector) and set throttle.

    The pointing command is rate-limited (max_rate_deg/s) so the autopilot is never
    asked to chase a jittery guidance output; throttle compensates the cosine loss
    of the current alignment (capped) and is cut when alignment is poor.
    """
    import time as _t
    tgt = unit(np.asarray(dir_enu, float))
    now = _t.time()
    prev = getattr(act, "_dir", None)
    if prev is None:
        cur = tgt
    else:
        dt = min(max(now - act._t, 0.0), 0.2)
        ang = math.acos(max(-1.0, min(1.0, float(np.dot(prev, tgt)))))
        step = math.radians(max_rate_deg) * dt
        cur = tgt if ang <= step else unit(prev * math.sin(ang - step) + tgt * math.sin(step))
    act._dir, act._t = cur, now
    d = unit(act.frame.to_body(cur))
    act.ap.target_direction = tuple(d)
    along = float(np.dot(np.asarray(act.nose()), d))
    thr = 0.0 if (throttle <= 0 or along < math.cos(math.radians(45))) else \
        float(min(max(throttle / max(along, 0.9), 0.0), 1.0))
    act.thr_cmd = thr                       # (no RPC) what this call commanded
    ctl = getattr(act, "ctl", None)
    if ctl is None:                         # everything but REC_FULL: unchanged
        act.v.control.throttle = thr
    elif abs(float(act.thr_s()) - thr) > 1e-4:
        # DR-010 (REC_FULL only): cached Control (1 RPC, not 2) and no write when the game already holds this value
        # (the stream shows the real throttle, so a DEADMAN cut on the other connection is always re-commanded)
        ctl.throttle = thr


def _qrot(q, v):
    """Rotate vector v by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    u = np.array([x, y, z])
    v = np.asarray(v, float)
    return 2.0 * np.dot(u, v) * u + (w * w - np.dot(u, u)) * v + 2.0 * w * np.cross(u, v)


class OwnAttitude:
    """Own pitch/yaw attitude controller for the unpowered final coast (fins + wheels): PD on the pointing
    error with strong angular-rate damping, which the kRPC autopilot lacked (0.45 Hz weathervane wobble).
    The input->axis mapping and signs are identified in flight with two short control pulses, so no
    assumption about KSP's control conventions is needed."""

    def __init__(self, conn, vessel, frame, log, kp=1.5, kd=2.5):
        self.v, self.frame, self.log = vessel, frame, log
        self.kp, self.kd = kp, kd
        self.rot = conn.add_stream(vessel.rotation, frame.rf)          # vessel -> frame.rf
        self.w = conn.add_stream(vessel.angular_velocity, frame.rf)
        self.dirs = conn.add_stream(vessel.direction, frame.rf)
        # vessel +x axis in frame.rf (sensitive to the quaternion convention, unlike the nose when vertical)
        self.x_true = np.asarray(conn.space_center.transform_direction((1.0, 0.0, 0.0), vessel.reference_frame, frame.rf))
        self.conj = None           # quaternion convention, checked against the measured nose direction
        self.map = None            # {'pitch': (axis, sign), 'yaw': (axis, sign)}
        self._id = None

    def _q(self):
        x, y, z, w = self.rot()
        q = (x, y, z, w)
        if self.conj is None:      # pick the convention whose +y matches vessel.direction
            nose = np.asarray(self.dirs())
            err = [float(np.dot(_qrot(c, (0.0, 1.0, 0.0)), nose)) + float(np.dot(_qrot(c, (1.0, 0.0, 0.0)), self.x_true))
                   for c in (q, (-x, -y, -z, w))]
            self.conj = err[1] > err[0]
            self.log("own attitude: quaternion check %.3f / %.3f -> conj=%s" % (err[0], err[1], self.conj))
        return (-x, -y, -z, w) if self.conj else q

    def _wv(self):
        x, y, z, w = self._q()
        return _qrot((-x, -y, -z, w), self.w())                    # angular velocity in vessel axes

    def identify(self, t):
        """Two 0.4 s pulses; returns True once the mapping is known."""
        c = self.v.control
        st = self._id
        if st is None:
            self._id = st = dict(stage=0, t0=t, w0=self._wv(), res={})
        name = ("pitch", "yaw")[st["stage"]]
        if t - st["t0"] < 0.4:
            setattr(c, name, 1.0)
            return False
        setattr(c, name, 0.0)
        dw = self._wv() - st["w0"]
        ax = int(np.argmax(np.abs(dw)))
        st["res"][name] = (ax, 1.0 if dw[ax] > 0 else -1.0)
        st["stage"] += 1
        st["t0"], st["w0"] = t, self._wv()
        if st["stage"] < 2:
            return False
        self.map = st["res"]
        self.log("own attitude: control map identified %s" % self.map)
        return True

    def step(self, d_enu, t):
        if self.map is None:
            self.identify(t)
            return
        x, y, z, w = self._q()
        q_inv = (-x, -y, -z, w)
        n_rf = _qrot((x, y, z, w), (0.0, 1.0, 0.0))                  # nose (vessel +y)
        d_rf = unit(self.frame.to_body(np.asarray(d_enu, float)))
        e_v = _qrot(q_inv, np.cross(n_rf, d_rf))                     # rotation needed, vessel axes
        w_v = _qrot(q_inv, self.w())
        for name in ("pitch", "yaw"):
            ax, sg = self.map[name]
            u = sg * (self.kp * e_v[ax] - self.kd * w_v[ax])
            setattr(self.v.control, name, float(max(-1.0, min(1.0, u))))

    def stop(self):
        self.v.control.pitch = 0.0
        self.v.control.yaw = 0.0
        # (streams are shared with the lander - kRPC de-duplicates identical streams - so keep them)
