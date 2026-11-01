"""Shared data access for the figure scripts.

Re-uses the parsers and inclusion rules of paper/analysis/compute_stats.py so that every figure is drawn from the
same populations as the statistics in the report. All inputs are raw records (read-only): data/raw/logs (transcripts,
telemetry, experiment and matrix logs), data/raw/gnc2 (simulation results) and data/raw/screenshots (in-game frames).
Large CSV files may be stored gzip-compressed; the readers here handle both."""
import csv
import functools
import json
import math
import os
import sys
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "analysis"))
import compute_stats as cs  # noqa: E402

G0 = 9.80665


@functools.lru_cache(maxsize=None)
def load(logs, freeze=cs.DATA_FREEZE.isoformat()):
    """Transcripts, experiment-log rows and matrix rows, joined exactly as in compute_stats.main()."""
    fr = datetime.fromisoformat(freeze)
    wd_all = cs.load_wd(logs)
    js_all, _ = cs.load_jsonl(logs)
    mx_all, _ = cs.load_matrix(logs)
    js_by_key = {}
    for r in js_all:
        if r["key"]:
            js_by_key.setdefault(r["key"], []).append(r)
    for r in wd_all:
        j = js_by_key.get(r["key"]) if r["key"] else None
        r["js"] = j[-1] if j else None
        r["miss2"] = float(r["js"]["result"]["miss"]) if r["js"] else float("nan")
        r["csv"] = None        # parsed lazily (telemetry())
    wd = [r for r in wd_all if r["start"] < fr]
    js = [r for r in js_all if r["_start"] < fr]
    mx = [r for r in mx_all if r["_start"] < fr]
    camp = [r for r in wd if r["full"] and r["start"] >= cs.CAMPAIGN_START and r["craft"] in cs.GF_FAMILY]
    done = [r for r in camp if r["result"] in ("PASS", "FAIL")]
    for i, r in enumerate(done, 1):
        r["mission"] = i
    sep = [r for r in wd if r["result"] == "SEPTEST"]
    post = [r for r in wd_all if r["start"] >= fr]          # after the data freeze (not in any report statistic)
    return dict(wd=wd, wd_all=wd_all, js=js, js_all=js_all, mx=mx, done=done, sep=sep, post=post, logs=logs)


def gate_rows(D):
    """Experiment-log rows of the terminal-phase (4 km) checkpoint flights, as in compute_stats (44 before freeze)."""
    out = []
    for r in D["js"]:
        e = r.get("env", {})
        if r["start"] == "term" and e.get("KSP_TERM_SAVE") == cs.TERM_GATE_SAVE and r["craft"] == "T-WorksFineGF":
            out.append(r)
    return out


@functools.lru_cache(maxsize=None)
def _read_csv(path):
    if not cs.raw_exists(path):
        return None
    with cs.raw_open(path, "rt", encoding="utf-8", errors="replace") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    cols = {}
    for k in rows[0]:
        if k == "phase" or k == "src":
            cols[k] = [r.get(k) for r in rows]
            continue
        v = []
        for r in rows:
            try:
                v.append(float(r.get(k)))
            except (TypeError, ValueError):
                v.append(float("nan"))
        cols[k] = v
    return cols


def telemetry(logs, key):
    """Column dict of <key>_landing.csv (game-time descent telemetry) or None."""
    if not key:
        return None
    return _read_csv(os.path.join(logs, key + "_landing.csv"))


def show_screenshot(ax, screens, fn, box=None, post=None, **imshow_kw):
    """Draw an in-game screenshot (cropped to box) on ax with the frame removed. If the file is not available, the
    panel is left empty with a note instead of failing, so that the other panels of the figure are still drawn.
    Returns True if the screenshot was drawn."""
    from PIL import Image
    p = os.path.join(screens, fn)
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(False)
    if not os.path.isfile(p):
        print("note: screenshot %s not found in %s; panel left empty" % (fn, screens), file=sys.stderr)
        ax.text(0.5, 0.5, "screenshot not available\n(data/raw/screenshots/%s)" % fn, transform=ax.transAxes,
                ha="center", va="center", fontsize=5, color="0.4")
        return False
    im = Image.open(p).convert("RGB")
    if box is not None:
        im = im.crop(box)
    if post is not None:
        im = post(im)
    ax.imshow(im, **imshow_kw)
    return True


def transcript(logs, wdname):
    return open(os.path.join(logs, wdname), encoding="utf-8", errors="replace").read()


# --------------------------------------------------------------------------------------------- ascent and descent
import re  # noqa: E402

import numpy as np  # noqa: E402

R_KERBIN = 600e3
GM_KERBIN = 3.5316e12
OMEGA_KERBIN = 2 * math.pi / 21549.425
RE_TEL = re.compile(r"\[\s*([\d.]+)s\] ascent tel: (\w+) alt ([\d.]+) km v (\d+) m/s pitch (-?[\d.]+) aoa (-?[\d.]+) "
                    r"Ap (-?[\d.]+) km stage (\d+) parts (\d+) thr (\d+) kN")
RE_LAUNCH_HDG = re.compile(r"\[\s*([\d.]+)s\] ascent \(own\): target .*launch heading ([\d.]+) deg")


def ascent(text):
    """Ascent telemetry from a transcript: time since own-code ascent start (s), altitude (km), surface speed (m/s),
    Ap (km), stage, and ground distance (km) integrated from the horizontal speed sqrt(v^2 - vz^2), vz from the
    smoothed altitude derivative. Returns None if no telemetry."""
    rows = [m.groups() for m in RE_TEL.finditer(text)]
    if len(rows) < 5:
        return None
    m0 = RE_LAUNCH_HDG.search(text)
    t0 = float(m0.group(1)) if m0 else float(rows[0][0])
    hdg = float(m0.group(2)) if m0 else 90.0
    t = np.array([float(r[0]) for r in rows]) - t0
    h = np.array([float(r[2]) for r in rows]) * 1e3
    v = np.array([float(r[3]) for r in rows])
    ap = np.array([float(r[6]) for r in rows])
    stage = np.array([int(r[7]) for r in rows])
    parts = np.array([int(r[8]) for r in rows])
    thr = np.array([float(r[9]) for r in rows])
    # vertical speed: central differences on a 5-sample moving average of altitude (0.1 km resolution)
    k = 5
    hs = np.convolve(np.pad(h, (k // 2, k // 2), mode="edge"), np.ones(k) / k, mode="valid")
    vz = np.gradient(hs, t)
    vz = np.clip(vz, -v, v)
    vh = np.sqrt(np.maximum(v ** 2 - vz ** 2, 0.0))
    x = np.concatenate([[0.0], np.cumsum(0.5 * (vh[1:] + vh[:-1]) * np.diff(t))])
    # ground distance on the sphere: horizontal path at altitude h maps to R/(R+h) of the arc
    xg = np.concatenate([[0.0], np.cumsum(0.5 * (vh[1:] + vh[:-1]) * np.diff(t) * R_KERBIN / (R_KERBIN + 0.5 * (h[1:] + h[:-1])))])
    return dict(t=t, h=h, v=v, vz=vz, vh=vh, x=xg, ap=ap, stage=stage, parts=parts, thr=thr, heading=hdg)


def coast_to_apoapsis(a, n=200):
    """Reconstructed (two-body, planar, vacuum) coast from the last ascent sample to apoapsis. The horizontal inertial
    speed is the integrated surface value plus the surface rotation along the launch heading; the vertical speed is
    then chosen so that the conic reaches the apoapsis logged at main-engine cut-off. Returns ground distance (m) and
    altitude (m). This is a reconstruction, not telemetry."""
    r0 = R_KERBIN + a["h"][-1]
    ra = R_KERBIN + a["ap"][-1] * 1e3
    vrot = OMEGA_KERBIN * R_KERBIN * math.sin(math.radians(a["heading"]))
    vt0 = a["vh"][-1] + vrot

    def apo(vz):
        e = 0.5 * (vt0 ** 2 + vz ** 2) - GM_KERBIN / r0
        hh = r0 * vt0
        return (-GM_KERBIN - math.sqrt(GM_KERBIN ** 2 + 2 * e * hh ** 2)) / (2 * e)
    lo, hi = 0.0, 1500.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if apo(mid) < ra else (lo, mid)
    vr, vt, r = 0.5 * (lo + hi), vt0, r0
    th, tt, dt = 0.0, 0.0, 0.25
    xs, hs = [], []
    while vr > 0 and tt < 3000:
        ar = -GM_KERBIN / r ** 2 + vt ** 2 / r
        at = -vr * vt / r
        r += vr * dt
        th += vt / r * dt
        vr += ar * dt
        vt += at * dt
        tt += dt
        xs.append(a["x"][-1] + (th - OMEGA_KERBIN * tt * math.sin(math.radians(a["heading"]))) * R_KERBIN)
        hs.append(r - R_KERBIN)
    return np.array(xs), np.array(hs)


def thrust_segments(T, phase="COAST", thr_min=0.01):
    ph, thr = T["phase"], T["thr_cmd"]
    segs, cur = [], None
    for i, (p, c) in enumerate(zip(ph, thr)):
        on = p == phase and c > thr_min
        if on and cur is None:
            cur = [i, i]
        elif on:
            cur[1] = i
        elif cur is not None:
            segs.append(tuple(cur))
            cur = None
    if cur is not None:
        segs.append(tuple(cur))
    return segs


def entry_burn_segment(T):
    """(first, last) row index of the entry burn: first post-entry COAST thrust segment of >= 3 s game time."""
    ut = T["ut"]
    for s in thrust_segments(T):
        if ut[min(s[1] + 1, len(ut) - 1)] - ut[s[0]] >= 3.0:
            return s
    return None


def descent_phases(T):
    """Per-row phase label of a landing CSV: coast (h > 70 km), drag modulation, entry burn, glide (< 10 km and
    before ignition), landing burn (phase BURN)."""
    n = len(T["ut"])
    hcol = T["h"]
    eb = entry_burn_segment(T)
    lab = []
    for i in range(n):
        if T["phase"][i] == "BURN":
            lab.append("landing burn")
        elif eb and eb[0] <= i <= eb[1]:
            lab.append("entry burn")
        elif hcol[i] > 70e3:
            lab.append("coast")
        elif hcol[i] < 10e3:
            lab.append("glide")
        else:
            lab.append("drag modulation")
    return lab


def ground_range(T):
    """Horizontal distance from the target (m) of each landing-CSV row."""
    return np.hypot(np.array(T["e"]), np.array(T["n"]))
