"""Offset-aim landing target (DR-012, verification process stage 4 "offset-aim entry"): land on flat ground a few km
from the KSC pad instead of on the pad. Selected only by the environment; without it nothing here runs.

  KSP_AIM="along_m,cross_m"   target = the pad moved along_m along / cross_m across the reference azimuth
  KSP_AIM_SITE=flat2k         a named, terrain-checked site (SITES below)
  KSP_AIM_AZ=deg              reference azimuth for KSP_AIM (default 90 = the eastbound KSC approach)

along > 0 = beyond the pad (long), < 0 = short of it; cross > 0 = right of the track. Same convention as
entry.ground_errors, of which offset_latlon is the exact inverse (on the sphere).

flat2k: 2.0 km short of the pad on the approach ground track (due west), terrain 64.7 m
from 4 intact, upright landings at 1.1-2.2 km west of the pad (logs 2026-10-19/21: 64.6-64.8 m) and the radar
samples of the CA-3 landing 20261021_003538 (64.7-64.8 m, flat). Short of the pad: the vehicle comes from the
west, so it never passes over the KSC complex at low altitude, the flight is ~2 km shorter, and a long miss
(the direction of the DR-012 replay's worst case c-, up to ~650 m) still ends >= 1.3 km from the pad."""
import math
import os
from dataclasses import dataclass

from . import entry

R_KERBIN = entry.KERBIN.radius
MIN_OFFSET_M, MAX_OFFSET_M = 500.0, 10000.0   # keep clear of the pad structure; stay on known ground near KSC

# name -> along, cross (m), azimuth (deg), expected terrain height (m ASL, from the flight logs) and the evidence
SITES = {
    "flat2k": dict(along=-2000.0, cross=0.0, az=90.0, h_exp=64.7,
                   evidence="logs 20261019_074546 (e -2237 n +13, 64.7 m), 20261019_221440 (e -1677 n +115, 64.6 m), "
                            "20261021_003538 (e -1188 n +95, 64.8 m radar), 20261020_234525 (e -1134 n +176): "
                            "all landed upright and intact"),
}


@dataclass(frozen=True)
class AimTarget:
    name: str
    lat: float
    lon: float
    along: float
    cross: float
    az: float
    h_exp: float | None = None       # expected terrain height (m ASL) for a named site; None = not surveyed

    @property
    def latlon(self):
        return (self.lat, self.lon)

    def describe(self):
        return "%s lat %.7f lon %.7f (along %+.0f m, cross %+.0f m of the pad, azimuth %.1f deg)" % (
            self.name, self.lat, self.lon, self.along, self.cross, self.az)


def offset_latlon(along, cross, az_deg=90.0, origin=entry.KSC_PAD, radius=R_KERBIN):
    """(lat, lon) of the point along / cross metres from `origin` for a track of azimuth az_deg (exact inverse of
    entry.ground_errors: p = cos c (cos a t + sin a h) + sin c right, a = along / R, c = cross / R)."""
    import numpy as np
    e, n, t = entry.enu_basis(*origin)
    ps = math.radians(az_deg)
    h = math.sin(ps) * e + math.cos(ps) * n
    right = math.cos(ps) * e - math.sin(ps) * n
    a, c = along / radius, cross / radius
    p = math.cos(c) * (math.cos(a) * t + math.sin(a) * h) + math.sin(c) * right
    p = p / np.linalg.norm(p)
    lat, lon, _ = entry.bf_to_latlonalt(p * radius)
    return lat, lon


def along_cross(lat, lon, az_deg=90.0, origin=entry.KSC_PAD, radius=R_KERBIN):
    """(along, cross) metres of (lat, lon) from `origin` for a track of azimuth az_deg (entry.ground_errors)."""
    return entry.ground_errors(lat, lon, origin, az_deg, radius)


def pad_distance(lat, lon):
    """Great-circle distance (m) of (lat, lon) from the KSC pad."""
    return entry.great_circle(lat, lon, *entry.KSC_PAD)


def from_env(env=None):
    """The offset target selected by the environment, or None (default: the pad, nothing changes).
    Raises ValueError on a malformed / ambiguous / out-of-range request (the caller REFUSES the run)."""
    env = os.environ if env is None else env
    raw, site = env.get("KSP_AIM", "").strip(), env.get("KSP_AIM_SITE", "").strip()
    if not raw and not site:
        return None
    if raw and site:
        raise ValueError("KSP_AIM and KSP_AIM_SITE both set (%r, %r): set one" % (raw, site))
    if site:
        if site not in SITES:
            raise ValueError("unknown KSP_AIM_SITE %r (known: %s)" % (site, ", ".join(sorted(SITES))))
        s = SITES[site]
        along, cross, az, name, h_exp = s["along"], s["cross"], s["az"], site, s["h_exp"]
    else:
        try:
            along, cross = (float(x) for x in raw.split(","))
        except ValueError:
            raise ValueError("KSP_AIM must be 'along_m,cross_m', got %r" % raw)
        az = float(env.get("KSP_AIM_AZ", 90.0))
        name, h_exp = "aim(%+.0f,%+.0f)" % (along, cross), None
    d = math.hypot(along, cross)
    if not (MIN_OFFSET_M <= d <= MAX_OFFSET_M) or not all(map(math.isfinite, (along, cross, az))):
        raise ValueError("offset %.0f m outside %.0f..%.0f m" % (d, MIN_OFFSET_M, MAX_OFFSET_M))
    lat, lon = offset_latlon(along, cross, az)
    return AimTarget(name, lat, lon, along, cross, az, h_exp)


def check_site(height_at, tgt, rings=(50.0, 100.0), n=8, flat_tol=2.0, h_tol=3.0, min_alt=5.0):
    """Pre-flight terrain check with the game's own terrain (height_at(lat, lon) -> m ASL, e.g. kRPC
    CelestialBody.surface_height; reading it does not change the game). The site and n points on each ring
    (m) must be dry land (> min_alt above sea level: below it may be water) and flat (max - min <= flat_tol);
    a named site must also be within h_tol of the height the flight logs gave (wrong site / coordinates).
    Returns (ok, h_site, message)."""
    h0 = float(height_at(tgt.lat, tgt.lon))
    hs = [h0]
    for rr in rings:
        for k in range(n):
            b = 2 * math.pi * k / n
            la, lo = offset_latlon(rr * math.cos(b), rr * math.sin(b), 90.0, origin=(tgt.lat, tgt.lon))
            hs.append(float(height_at(la, lo)))
    spread = max(hs) - min(hs)
    why = []
    if min(hs) < min_alt:
        why.append("terrain %.1f m ASL (water?)" % min(hs))
    if spread > flat_tol:
        why.append("not flat: %.1f m spread within %.0f m" % (spread, max(rings)))
    if tgt.h_exp is not None and abs(h0 - tgt.h_exp) > h_tol:
        why.append("terrain %.1f m, logs say %.1f m" % (h0, tgt.h_exp))
    msg = "site terrain %.2f m ASL, spread %.2f m within %.0f m (%d points)%s" % (
        h0, spread, max(rings), len(hs), (": " + "; ".join(why)) if why else "")
    return not why, h0, msg


def check_clear(ray_down, tgt, rings=(0.0, 25.0, 50.0), n=8, up=300.0, tol=3.0):
    """Obstacle check (buildings are not in the terrain heights): ray_down(lat, lon, up) -> distance (m) to the
    first collider straight down from `up` m above the terrain there (kRPC SpaceCenter.raycast_distance), or
    None / inf when nothing was hit (colliders not loaded). A hit more than tol m short of the terrain is an
    obstacle. Returns (status 'clear' | 'obstacle' | 'unknown', message)."""
    hits, worst, unknown = 0, 0.0, 0
    for rr in rings:
        for k in range(n if rr > 0 else 1):
            b = 2 * math.pi * k / n
            la, lo = offset_latlon(rr * math.cos(b), rr * math.sin(b), 90.0, origin=(tgt.lat, tgt.lon))
            try:
                d = ray_down(la, lo, up)
            except Exception:
                d = None
            if d is None or not math.isfinite(d) or d > 2 * up:
                unknown += 1
                continue
            hits += 1
            worst = max(worst, up - d)
    total = hits + unknown
    if hits and worst > tol:
        return "obstacle", "obstacle %.1f m above the terrain within %.0f m of the site" % (worst, max(rings))
    if hits == 0:
        return "unknown", "raycast: no collider hit at %d points (terrain not loaded?)" % total
    return "clear", "raycast: %d/%d points hit the ground, highest %.1f m above the terrain" % (hits, total, worst)
