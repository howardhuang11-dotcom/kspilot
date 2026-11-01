"""Checks of kspilot.maneuver against orbital-mechanics identities (no KSP needed)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot import maneuver as mv  # noqa: E402
from kspilot.entry import kepler_propagate  # noqa: E402

MU, R = 3.5316e12, 600000.0


def circular(alt, inc_deg=0.0, phase=0.0):
    """State on a circular orbit in kRPC's frame (y = pole, prograde h along -y)."""
    r = R + alt
    s = math.sqrt(MU / r)
    pos = r * np.array([math.cos(phase), 0.0, math.sin(phase)])
    east = np.array([-math.sin(phase), 0.0, math.cos(phase)])      # prograde direction at the equator
    i = math.radians(inc_deg)
    vel = s * (math.cos(i) * east + math.sin(i) * np.array([0.0, 1.0, 0.0]))
    return pos, vel


def test_frame_convention():
    r, v = circular(100e3)
    assert abs(mv.inclination(r, v)) < 1e-6          # prograde equatorial = 0 deg
    assert abs(mv.inclination(r, -v) - 180) < 1e-6
    r, v = circular(100e3, inc_deg=6.0)
    assert abs(mv.inclination(r, v) - 6.0) < 1e-6


def test_circularize_at_apoapsis():
    r, v = circular(80e3)
    v = v * 1.05                                         # raise the apoapsis
    t = mv.time_to_radius_extremum(r, v, MU, apo=True)
    ra = mv.apsides(r, v, MU)[1]
    r2, v2 = kepler_propagate(r, v, t, MU)
    assert abs(np.linalg.norm(r2) - ra) < 1.0
    v3 = v2 + mv.dv_circularize(r2, v2, MU)
    rp, ra3 = mv.apsides(r2, v3, MU)
    assert abs(ra3 - rp) < 1.0


def test_change_apsides():
    r, v = circular(100e3)
    for alt in (150e3, 400e3):
        dv = mv.dv_change_apoapsis(r, v, MU, R + alt)
        assert abs(mv.apsides(r, v + dv, MU)[1] - (R + alt)) < 1.0
        assert abs(np.dot(dv, r)) < 1e-6 * np.linalg.norm(r)        # horizontal burn
    dv = mv.dv_change_periapsis(r, v, MU, R - 60e3)                   # the deorbit burn
    assert abs(mv.apsides(r, v + dv, MU)[0] - (R - 60e3)) < 1.0
    assert 140 < np.linalg.norm(dv) < 160                             # ~150 m/s as flown


def test_hohmann():
    r, v = circular(100e3)
    _, dv1, dt2, dv2 = mv.plan_hohmann(r, v, MU, R + 250e3)
    v1 = v + dv1
    r2, v2 = kepler_propagate(r, v1, dt2, MU)
    rp, ra = mv.apsides(r2, v2 + dv2, MU)
    assert abs(rp - (R + 250e3)) < 5 and abs(ra - (R + 250e3)) < 5
    # textbook Hohmann total
    r1, r2_ = R + 100e3, R + 250e3
    a = (r1 + r2_) / 2
    tot = (math.sqrt(MU * (2 / r1 - 1 / a)) - math.sqrt(MU / r1)) + (math.sqrt(MU / r2_) - math.sqrt(MU * (2 / r2_ - 1 / a)))
    assert abs(np.linalg.norm(dv1) + np.linalg.norm(dv2) - tot) < 0.5


def test_inclination_change_at_node():
    r, v = circular(100e3, inc_deg=6.0, phase=0.0)       # at the node (equator)
    dv = mv.dv_change_inclination(r, v, 0.0)
    assert abs(mv.inclination(r, v + dv)) < 1e-3
    expected = 2 * np.linalg.norm(v) * math.sin(math.radians(3.0))   # 2 v sin(di/2)
    assert abs(np.linalg.norm(dv) - expected) < 0.5


def test_plane_crossing_time_and_match():
    r, v = circular(100e3, inc_deg=10.0, phase=0.3)
    n = np.array([0.0, 1.0, 0.0])                         # equatorial plane
    t = mv.time_to_plane_crossing(r, v, MU, n, ascending=True)
    r2, v2 = kepler_propagate(r, v, t, MU)
    assert abs(np.dot(r2, n)) < 1.0
    v3 = v2 + mv.dv_match_plane(r2, v2, n)
    assert mv.inclination(r2, v3) < 1e-3


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
