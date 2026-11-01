"""Recording pipeline v3 (decision record DR-011, HOP_REC=full, KSP_REC_ENGINE=obs): the clocks that tie a
real-time OBS video to game time. Pure Python (no kRPC, no OBS): tests/test_rec_v3.py.

Two maps, both written during the flight by kspilot/obsrec.py (never in the control loop):
  video t -> wall   VideoClock. t = seconds from the first frame of the OBS file, wall = time.time(). Built by
                    calibrate() from obs-websocket GetRecordStatus samples (t_send, t_recv, outputDuration). OBS
                    computes outputDuration as (frames handed to the encoder) x (frame time), so at the midpoint wall
                    w of a request, w - duration lies in [W0 + L, W0 + L + T) (W0 = wall of the first frame, L = encoder
                    queue latency, T = 1/30 s). Mean of those offsets - T/2 = W0 + L: one-frame accuracy from ~10
                    samples. The samples continue every few seconds for the whole recording, so dropped / skipped
                    output frames (the file then runs "short" of the wall clock) show up as a slope != 1 or as steps:
                    a straight-line fit is used when every sample sits within SNAP_S of it, else a piecewise map.
                    LAG_S (env KSP_REC_SYNC_S): the remaining constant capture latency (game frame -> OBS frame), not
                    measured yet (gate item), default 0.
  wall -> UT        TimeMap. Rows (wall of the UT stream update, UT, warp rate, rails factor, physics factor) at
                    >= 4 Hz from a kRPC stream on its own connection. ut(w) / wall(u) interpolate; runs() classifies
                    stretches as normal / physics warp / rails warp from the warp fields, and the game speed of each
                    stretch is MEASURED (dUT / dwall), never assumed.
"""
import csv
import os
from bisect import bisect_left, bisect_right

FPS = 30
FRAME = 1.0 / FPS
SNAP_S = 0.08            # every clock sample within 80 ms of a straight line -> linear video clock
LAG_S = float(os.environ.get("KSP_REC_SYNC_S", "0") or 0.0)   # capture latency (s): frame at video t shows wall t - LAG
MIN_RUN_S = 0.5          # warp stretches shorter than this (wall s) are not treated separately
TIMEMAP_FIELDS = ("wall", "ut", "warp_rate", "rails", "physics")


# ---------------------------------------------------------------------------------------------------------------------
# piecewise-linear helpers
# ---------------------------------------------------------------------------------------------------------------------
def _interp(xs, ys, x, slope_out=1.0):
    """Linear interpolation of the monotone polyline (xs, ys); outside it: the end slope (slope_out for one knot)."""
    n = len(xs)
    if n == 0:
        raise ValueError("empty map")
    if n == 1:
        return ys[0] + slope_out * (x - xs[0])
    if x <= xs[0]:
        i = 0
    elif x >= xs[-1]:
        i = n - 2
    else:
        i = bisect_right(xs, x) - 1
    dx = xs[i + 1] - xs[i]
    s = (ys[i + 1] - ys[i]) / dx if dx > 1e-12 else slope_out
    return ys[i] + s * (x - xs[i])


def _fit(xs, ys):
    """Least-squares line y = c + a x -> (c, a)."""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx > 1e-12 else 1.0
    return my - a * mx, a


# ---------------------------------------------------------------------------------------------------------------------
# video t <-> wall
# ---------------------------------------------------------------------------------------------------------------------
class VideoClock:
    """wall(t) / video(w): a monotone piecewise-linear map between the OBS video clock and the wall clock."""

    def __init__(self, knots, info=None):
        knots = sorted(knots)
        self.t = [k[0] for k in knots]
        self.w = [k[1] for k in knots]
        self.info = info or {}

    def wall(self, t):
        return _interp(self.t, self.w, t)

    def video(self, w):
        return _interp(self.w, self.t, w)


def calibrate(samples, frame=FRAME, lag=None, snap=SNAP_S):
    """VideoClock from GetRecordStatus samples [(t_send, t_recv, duration_s)] (duration = outputDuration / 1000).
    Samples with duration 0 (the output not running yet) and slow round trips (> 3x the median, > 20 ms) are dropped.
    offset_i = mid_i - duration_i; W0 = mean(offset) - frame / 2 (the duration counts whole frames: offsets are
    uniform over one frame). Linear (one offset + drift slope) when all samples are within `snap` of the fit, else
    piecewise through every sample (dropped frames: steps). The frame at video t shows the game at wall(t), which
    includes `lag` (capture latency, LAG_S)."""
    lag = LAG_S if lag is None else lag
    ok = [(a, b, d) for a, b, d in samples if d is not None and d > 0 and b >= a]
    if not ok:
        raise ValueError("no usable OBS clock samples")
    rtts = sorted(b - a for a, b, _ in ok)
    lim = max(3.0 * rtts[len(rtts) // 2], 0.02)
    ok = [s for s in ok if s[1] - s[0] <= lim] or ok
    vt = [d + 0.5 * frame for _, _, d in ok]                    # video time being produced at the request midpoint
    wm = [0.5 * (a + b) - lag for a, b, _ in ok]
    info = dict(n=len(ok), rtt_ms=round(1e3 * rtts[len(rtts) // 2], 2), lag_s=lag)
    if len(ok) >= 2 and vt[-1] - vt[0] > 1.0:
        c, a = _fit(vt, wm)
        res = [w - (c + a * t) for t, w in zip(vt, wm)]
    else:
        off = sum(w - t for t, w in zip(vt, wm)) / len(vt)
        c, a, res = off, 1.0, [w - t - off for t, w in zip(vt, wm)]
    info.update(w0=c, slope=a, resid_max_ms=round(1e3 * max(abs(r) for r in res), 1))
    if max(abs(r) for r in res) <= snap:
        info["mode"] = "linear"
        return VideoClock([(0.0, c), (1.0, c + a)], info)
    knots, last_w = [], None
    for t, w in sorted(zip(vt, wm)):                            # piecewise: monotone in both clocks
        if last_w is not None and w <= last_w:
            continue
        knots.append((t, w))
        last_w = w
    # the first frame: extend the first segment back to t = 0 at slope 1
    knots.insert(0, (0.0, knots[0][1] - knots[0][0]))
    info["mode"] = "piecewise"
    return VideoClock(knots, info)


# ---------------------------------------------------------------------------------------------------------------------
# wall <-> UT
# ---------------------------------------------------------------------------------------------------------------------
class Row(tuple):
    __slots__ = ()
    wall = property(lambda s: s[0])
    ut = property(lambda s: s[1])
    rate = property(lambda s: s[2])
    rails = property(lambda s: s[3])
    phys = property(lambda s: s[4])


class TimeMap:
    """rows [(wall, ut, warp_rate, rails_factor, physics_factor)] -> ut(w), wall(u), runs(w0, w1)."""

    def __init__(self, rows):
        rows = sorted((Row((float(r[0]), float(r[1]), float(r[2]), int(r[3]), int(r[4]))) for r in rows),
                      key=lambda r: r[0])
        out, top = [], None
        for r in rows:                                           # UT never decreases (stream jitter / reorder)
            if out and r.wall <= out[-1].wall + 1e-9:
                continue
            if top is not None and r.ut < top:
                r = Row((r.wall, top) + tuple(r[2:]))
            top = r.ut
            out.append(r)
        if not out:
            raise ValueError("empty time map")
        self.rows = out
        self.w = [r.wall for r in out]
        self.u = [r.ut for r in out]

    @classmethod
    def load(cls, path):
        rows = []
        with open(path) as f:
            for r in csv.DictReader(f):
                try:
                    rows.append((r["wall"], r["ut"], r.get("warp_rate") or 1, r.get("rails") or 0, r.get("physics") or 0))
                except (KeyError, ValueError):
                    continue
        return cls(rows)

    def ut(self, w):
        return _interp(self.w, self.u, w)

    def wall(self, u):
        """First wall time at which the game reached UT u (a paused stretch maps to its start)."""
        u0, u1 = self.u[0], self.u[-1]
        if u <= u0:
            return self.w[0] - (u0 - u)                      # before the map: 1x
        if u >= u1:
            if len(self.w) >= 2:                             # after it: the last measured speed
                j = len(self.w) - 2
                while j > 0 and self.u[j] >= u1:
                    j -= 1
                s = (u1 - self.u[j]) / max(self.w[-1] - self.w[j], 1e-9)
                return self.w[-1] + (u - u1) / max(s, 1e-6)
            return self.w[-1] + (u - u1)
        i = bisect_left(self.u, u)                           # first knot with ut >= u
        a, b = i - 1, i
        du = self.u[b] - self.u[a]
        return self.w[a] + (self.w[b] - self.w[a]) * ((u - self.u[a]) / du if du > 1e-12 else 0.0)

    def rate(self, w0, w1):
        """Measured game speed (game s per wall s) over [w0, w1]."""
        return (self.ut(w1) - self.ut(w0)) / max(w1 - w0, 1e-9)

    def runs(self, w0, w1, min_run=MIN_RUN_S):
        """[(wa, wb, cls, game speed)] covering [w0, w1]: cls 'rails' (rails warp factor > 0 at a warp rate > 1),
        'phys' (physics warp factor > 0), 'normal' otherwise (the warp fields decide the class, the speed is
        measured). Stretches shorter than min_run join their predecessor."""
        if w1 <= w0:
            return []
        n = len(self.rows)
        i = max(bisect_right(self.w, w0) - 1, 0)
        segs = []

        def cls(r):
            if r.rails > 0 and r.rate > 1.01:
                return "rails"
            if r.phys > 0:
                return "phys"
            return "normal"
        t = w0
        while t < w1 - 1e-12 and i < n:
            b = min(self.w[i + 1] if i + 1 < n else w1, w1)      # interval [row i, row i+1): row i's class
            if b > t:
                c = cls(self.rows[i])
                if segs and segs[-1][2] == c:
                    segs[-1][1] = b
                else:
                    segs.append([t, b, c])
                t = b
            i += 1
        merged = []
        for s in segs:                                       # short blips join the previous stretch
            if merged and (s[1] - s[0] < min_run or s[2] == merged[-1][2]):
                merged[-1][1] = s[1]
            else:
                merged.append(list(s))
        if len(merged) > 1 and merged[0][1] - merged[0][0] < min_run:
            merged[1][0] = merged[0][0]
            merged.pop(0)
        return [(a, b, c, self.rate(a, b)) for a, b, c in merged]


def write_header(f):
    f.write(",".join(TIMEMAP_FIELDS) + "\n")


def format_row(wall, ut, rate, rails, phys):
    return "%.4f,%.4f,%.3f,%d,%d\n" % (wall, ut, rate, rails, phys)
