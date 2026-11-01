"""Kerbin constants and atmosphere model sampled from the game (data/kerbin.json)."""
import json
import os

import numpy as np

_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", "kerbin.json")

with open(_PATH) as _f:
    KERBIN = json.load(_f)

R = KERBIN["radius"]                    # m
MU = KERBIN["mu"]                       # m^3/s^2
OMEGA = KERBIN["rotational_speed"]      # rad/s
ATMO_DEPTH = KERBIN["atmosphere_depth"] # m
G0 = 9.80665                            # Isp reference gravity used by KSP, m/s^2
_ALT = np.asarray(KERBIN["alt"], float)
_RHO = np.asarray(KERBIN["density"], float)


def density(alt):
    """Air density (kg/m^3) at altitude above sea level (500 m table, linear interpolation; 0 above 70 km)."""
    rho = np.interp(alt, _ALT, _RHO, right=0.0)
    return float(rho) if np.ndim(rho) == 0 else rho


def gravity(r):
    """Gravitational acceleration magnitude (m/s^2) at radius r from Kerbin's centre."""
    return MU / np.square(r)
