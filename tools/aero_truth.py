"""Achieved angle of attack and normal-force coefficient from a landing CSV, with the TRUE velocity.

The CSV's ve/vn are pad-ENU horizontal components but vu is the RADIAL vertical speed at the vehicle (state()),
so far from the pad (47 km -> 4.5 deg) the logged aoa/cmd are distorted. Rebuild the ENU vertical
from the radial one and the position, then fit a_n = c * q * sin(aoa) (lift toward the leading ENGINE end,
i.e. against the top's tilt) per altitude band. Commanded direction (dx,dy,dz) is in true ENU already.
usage: aero_truth.py landing.csv
"""
import csv
import sys

import numpy as np

R = 600000.0
OMEGA = np.array([0.0, 2 * np.pi / 21549.425, 0.0])    # Kerbin sidereal rotation, pad ENU (KSC on the equator)


def load(path):
    out = []
    for r in csv.DictReader(open(path)):
        try:
            out.append({k: float(r[k]) for k in ("ut", "h", "e", "n", "ve", "vn", "vu", "ne", "nn", "nu", "dx", "dy", "dz",
                                                 "q", "thr_act", "m")})
        except (ValueError, KeyError, TypeError):
            pass
    return out


def true_v(r):
    uf = r["h"] - (r["e"] ** 2 + r["n"] ** 2) / (2 * R)
    P = np.array([r["e"], r["n"], R + uf])
    up = P / np.linalg.norm(P)
    vz = (r["vu"] * np.linalg.norm(P) - r["ve"] * r["e"] - r["vn"] * r["n"]) / (R + uf)
    return np.array([r["ve"], r["vn"], vz]), up


def main(path):
    rows = [r for r in load(path) if r["h"] == r["h"]]
    print("   h      q kPa   cmd   aoa  | lift toward engine end m/s2 | c=a/(q sin aoa) [m2/kg]")
    bands = {}
    for i in range(0, len(rows) - 20):
        a, b = rows[i], rows[i + 20]
        dt = b["ut"] - a["ut"]
        if not (0.3 < dt < 3.0) or a["thr_act"] > 0.01 or b["thr_act"] > 0.01 or a["h"] > 45000:
            continue
        va, up = true_v(a)
        vb, _ = true_v(b)
        air = -va / np.linalg.norm(va)
        nose = np.array([a["ne"], a["nn"], a["nu"]])
        nose /= np.linalg.norm(nose)
        d = np.array([a["dx"], a["dy"], a["dz"]])
        d /= np.linalg.norm(d)
        aoa = np.degrees(np.arccos(np.clip(np.dot(nose, air), -1, 1)))
        cmd = np.degrees(np.arccos(np.clip(np.dot(d, air), -1, 1)))
        acc = (vb - va) / dt + 9.81 * up * (R / (R + a["h"])) ** 2      # remove gravity (local vertical)
        # the pad ENU frame turns with Kerbin (axis = north at KSC, lat ~0): add back Coriolis + centrifugal
        # (2 W x v ~ 0.76 m/s^2 at 1300 m/s - it looked like a q-independent "lift")
        P = np.array([a["e"], a["n"], R + a["h"]])
        acc = acc + 2 * np.cross(OMEGA, va) + np.cross(OMEGA, np.cross(OMEGA, P))
        an = acc - np.dot(acc, air) * air
        beta = nose - np.dot(nose, air) * air                         # top's tilt off the airflow
        nb = np.linalg.norm(beta)
        lift_eng = float(np.dot(an, -beta / nb)) if nb > 1e-4 else float("nan")   # toward the engine end
        key = int(a["h"] // 5000) * 5
        bands.setdefault(key, []).append((a["q"], aoa, cmd, lift_eng, nb))
    for k in sorted(bands, reverse=True):
        B = np.array(bands[k])
        sel = B[:, 4] > np.sin(np.radians(1.0))
        c = (np.sum(B[sel, 3] * B[sel, 0] * B[sel, 4]) / np.sum((B[sel, 0] * B[sel, 4]) ** 2)) if sel.sum() > 5 else float("nan")
        print("%3d-%2d km  %6.1f  %5.1f %5.1f  | %+6.2f (n %3d)            | %+.1e" % (
            k, k + 5, np.median(B[:, 0]) / 1e3, np.median(B[:, 2]), np.median(B[:, 1]), np.median(B[:, 3]), len(B), c))


if __name__ == "__main__":
    main(sys.argv[1])
