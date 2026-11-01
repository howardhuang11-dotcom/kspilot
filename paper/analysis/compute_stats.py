#!/usr/bin/env python3
"""Reproduce every statistic, table and figure of the exploratory (development-history) part of the kspilot report
from the raw flight records.

usage:
    python paper/analysis/compute_stats.py [LOGS_DIR] [--out PAPER_DIR] [--freeze ISO_TIME]
    python paper/analysis/compute_stats.py --check-manifest

LOGS_DIR defaults to data/raw/logs of this repository (resolved from the location of this script, independent of the
working directory). It must contain the raw records written during development (large CSV files may be stored gzip-
compressed, "<name>.csv.gz"; every reader here and in paper/figures handles both):
    wd_YYYYMMDD_HHMMSS.out                   supervisor (watchdog) transcript of one run; its first "RESULT" line is
                                             the outcome of the run (the authority for outcome classification)
    YYYYMMDD_HHMMSS_orbit_<craft>.txt        per-flight log; only its file name is used (craft and join key); the
                                             repository stores the names in flight_log_names.txt instead of the logs
    YYYYMMDD_HHMMSS_orbit_<craft>_landing.csv  high-rate descent telemetry (game time); optional per flight
    experiments.jsonl                        one JSON record per flight that reached the descent phase; holds the
                                             miss at 0.01 m resolution and the code revision
    matrix_results.txt                       one line per robustness-matrix case attempt

Outputs (PAPER_DIR defaults to the parent of this script's directory):
    tables/*.csv, tables/*.md, tables/stats.json, tables/raw_manifest.sha256
    (the figures are drawn from the same raw records by paper/figures/make_all.py)

--check-manifest verifies the SHA-256 of every file under data/raw against paper/tables/raw_manifest.sha256 and exits
non-zero on any difference. The manifest is written by a normal run when LOGS_DIR is the repository's data/raw/logs.

Join key. A run's key is the base name of its per-flight log without extension (for example
"20261021_170432_orbit_T-WorksFineGF"). The per-flight log is the first one opened after the supervisor transcript starts
(usually 0-15 s later; up to 600 s when the game had to be restarted) and before the next transcript starts;
experiments.jsonl names it in its "log" field (only the base name is kept; local paths never reach the outputs) and
the landing CSV is "<key>_landing.csv".

Data freeze. Every source is filtered on the start time of its parent run (transcript start; for experiment-log rows
the time stamp of the per-flight log; for matrix rows the start of the transcript they name).

Rounding. Printed and tabulated values round half up (decimal.ROUND_HALF_UP).

Only the Python standard library is required.
Quartiles use linear interpolation between order statistics (Hyndman-Fan type 7, the numpy default).
Proportions are reported with Wilson score, Clopper-Pearson and Jeffreys 95 % intervals.
"""
import argparse
import csv
import glob
import gzip
import hashlib
import json
import math
import os
import re
import sys
from datetime import datetime, timedelta
from decimal import Decimal, ROUND_HALF_UP

# --------------------------------------------------------------------------------------------- definitions
GF_FAMILY = ("T-WorksFineGF", "T-WorksFineGF-P45", "T-WorksFineGF-P60", "T-WorksFineGF-LIT")
PAYLOAD_T = {"T-WorksFineGF": 2.48, "T-WorksFineGF-P45": 4.50, "T-WorksFineGF-P60": 6.00, "T-WorksFineGF-LIT": 2.48}
NO_AIRBRAKE = ("T-WorksFineGF0T", "T-WorksFineGF2", "T-WorksFineGF2T")      # grid-fin-only development variants
# Start of the exploratory analysis window: first full mission flown with the recalibrated per-craft drag model.
# The window was chosen after the flights (see the sensitivity analyses, which this script also computes).
CAMPAIGN_START = datetime(2026, 10, 21, 1, 10, 0)
# Data freeze: only runs that started before this instant are analysed (the record directory keeps growing)
DATA_FREEZE = datetime(2026, 10, 22, 0, 0, 0)
PASS_RULE = "landed, miss < 20 m, touchdown speed < 4 m/s, upright cosine > 0.95 (tools/orbit_recover.py)"
TERM_GATE_SAVE = "g5_term_GF"      # 4 km terminal-phase checkpoint used by every regression gate
GATE_LIMIT_M, GATE_NEED = 5.0, 2   # gate passes when >= 2 of 3 flights land within 5 m
LFOX_KG_PER_LF = 5.0 * (1.0 + 1.1 / 0.9)   # kg of LiquidFuel + Oxidizer per LF unit (5 kg/unit, 0.9 : 1.1 ratio)
# Configuration epochs inside the window, bounded by the two corrective actions (first mission of each epoch)
EPOCHS = (("E1: drag recalibration to separation corrective action", 1),
          ("E2: separation corrective action to velocity-increment accounting correction", 13),
          ("E3: after the velocity-increment accounting correction", 22))
# Attributions that are not in the raw records and come from the project's decision record (not released)
DR_RUNS = {
    "wd_20261021_231728.out": dict(craft="T-WorksFineGF-LIT", start="pad",
                                   note="kRPC connection refused before launch (game closed); decision record 23:45"),
    "wd_20261021_015055.out": dict(note="terminated by expiry of an external monitor during the deorbit-node warp; "
                                        "decision record, lesson 63"),
}
DR_RECORDING = {"wd_20261021_035335.out": "orphan frame capture during ascent (decision record 04:15)"}
CHECKPOINT_START = (("g5_term", "4 km checkpoint"), ("g5_mid", "15 km checkpoint"), ("g5_entry", "pre-entry checkpoint"),
                    ("g5_orbit", "orbit checkpoint"))

RE_WD = re.compile(r"wd_(\d{8}_\d{6})\.out$")
RE_FLIGHTLOG = re.compile(r"(\d{8}_\d{6})_orbit_(.+)\.txt$")
RE_TARGET = re.compile(r"target Pe ([\d.]+) km Ap ([\d.]+) km i ([\d.]+) deg")
RE_RESULT = re.compile(r"RESULT (PASS|FAIL|REFUSED|SEPTEST)(.*)")
RE_NUM = {k: re.compile(k + r"=(-?[\d.]+|nan)") for k in ("miss", "v_td", "upright", "fuel_left", "parts",
                                                          "thr", "explosions", "core_hit", "q")}
RE_ORBIT = re.compile(r"orbit Ap=([\d.]+)km Pe=([\d.]+)km inc=([\d.]+) \(err Ap ([+-]?\d+)m Pe ([+-]?\d+)m\)")
RE_CORE = re.compile(r"flying core: (\S+)")
RE_EB_END = re.compile(r"ENTRY BURN end .*?dv flown (\d+) m/s(?: \(propellant (\d+) kg)?")
RE_WIN = re.compile(r"deorbit: window in (\d+) revolutions \(([\d.]+) h\): normal (-?\d+) m/s, total (\d+) m/s")
RE_PLAN = re.compile(r"deorbit plan: miss \S+ dv ([\d.]+) m/s \(retro [\d.]+ normal (-?[\d.]+)\) t_ign \+(\d+)s")
RE_SEPQ = re.compile(r"sep metric: q ([\d.]+) kPa")
RE_SEPP = re.compile(r"sep parts: (\d+) before, expected (\d+) after, now (\d+), thrust (\d+) -> (\d+) kN")
RE_SEPW = re.compile(r"sep window .*?explosions_15s (\d+) \(booster (\d+), core (\d+)\)")
# every hibernated coast segment, whatever its purpose ("node", "deorbit window", ...); Mission.coast_to logs
# "power: <why> <s> s hibernating: EC <before> -> <after> ..." (r2 fix: r1 matched only "node" segments)
RE_HIB = re.compile(r"power: [a-z][a-z ]*? (\d+) s hibernating: EC ([\d.]+) -> ([\d.]+)")
RE_CKPT = re.compile(r"loading checkpoint (\S+)")
RE_T_ASCENT = re.compile(r"\[\s*([\d.]+)s\] ascent \(own\)")
RE_T_STAGE = re.compile(r"\[\s*([\d.]+)s\] ascent: staging \d+ -> \d+ \((\d+) dry engines dropped, 0 engines to light\)")
RE_T_ABORT = re.compile(r"\[\s*([\d.]+)s\] ascent ABORT")
RE_VAC = re.compile(r"vacuum correction (\d+): predicted miss (\d+)m \(along ([+-]\d+) cross ([+-]\d+)\) spread (\d+)m, "
                    r"trim ([\d.]+) m/s")
RE_TRIMFX = re.compile(r"trim effect ([\d.]+)x of prediction")
RE_PULSE = re.compile(r"range trim: retro pulse #(\d+)")
RE_IGN = re.compile(r"IGNITION h=(\d+)m vz=(-?[\d.]+) vh=([\d.]+) offset=(\d+)m")
RE_DM = re.compile(r"DEADMAN: control loop stalled ([\d.]+) s( \(game\))?")


# --------------------------------------------------------------------------------------------- raw-data location
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RAW_ROOT = os.path.join(REPO_ROOT, "data", "raw")
DEFAULT_LOGS = os.path.join(RAW_ROOT, "logs")
DEFAULT_GNC2 = os.path.join(RAW_ROOT, "gnc2")
DEFAULT_SCREENS = os.path.join(RAW_ROOT, "screenshots")
MANIFEST_PATH = os.path.join(REPO_ROOT, "paper", "tables", "raw_manifest.sha256")
RAW_HINT = ("The raw development records are part of this repository under data/raw (see data/raw/README.md and "
            "Section 9 of paper/kspilot_report.md). If data/raw is missing, restore it (git checkout data/raw or "
            "re-clone) or pass the directory containing the records explicitly.")


def die(msg):
    """One readable message on stderr and a non-zero exit instead of a traceback."""
    sys.stderr.write("ERROR: " + msg.rstrip() + "\n")
    sys.exit(2)


def raw_path(path):
    """Path of a raw file, stored plain or as <path>.gz; None if neither exists."""
    for p in (path, path + ".gz"):
        if os.path.isfile(p):
            return p
    return None


def raw_exists(path):
    return raw_path(path) is not None


def raw_open(path, mode="rt", **kw):
    """open() that transparently reads <path>.gz when <path> itself is not present."""
    p = raw_path(path) or path
    if p.endswith(".gz"):
        return gzip.open(p, mode, **kw)
    return open(p, mode, **kw)


def require_logs(logs):
    """Check the records compute_stats and the other analysis scripts need; one readable error otherwise."""
    missing = []
    if not os.path.isdir(logs):
        die("raw-record directory not found: %s\nExpected: a directory with experiments.jsonl, matrix_results.txt, "
            "wd_*.out and *_landing.csv[.gz].\n%s" % (logs, RAW_HINT))
    for n in ("experiments.jsonl", "matrix_results.txt"):
        if not raw_exists(os.path.join(logs, n)):
            missing.append(n)
    if not glob.glob(os.path.join(logs, "wd_*.out")):
        missing.append("wd_*.out")
    if missing:
        die("raw-record directory %s lacks: %s\nExpected: experiments.jsonl, matrix_results.txt, wd_*.out, "
            "*_landing.csv[.gz].\n%s" % (logs, ", ".join(missing), RAW_HINT))


def require_dir(path, what, names):
    """Fail with one message if a raw-data directory (gnc2 results, screenshots) or any listed file is missing."""
    if not os.path.isdir(path):
        die("%s not found: %s\nExpected files: %s\n%s" % (what, path, ", ".join(names), RAW_HINT))
    miss = [n for n in names if not os.path.isfile(os.path.join(path, n))]
    if miss:
        die("%s lacks: %s (in %s)\n%s" % (what, ", ".join(miss), path, RAW_HINT))


# --------------------------------------------------------------------------------------------- statistics
def quantile(xs, p):
    s = sorted(xs)
    if not s:
        return float("nan")
    h = (len(s) - 1) * p
    lo = math.floor(h)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (h - lo) * (s[hi] - s[lo])


def median_ci(xs, level=0.95):
    """Distribution-free confidence interval for the median from binomial order statistics: the widest-coverage
    pair (x_(j), x_(n-j+1)) with P(j <= B <= n-j) >= level, B ~ Binomial(n, 1/2). Returns (lo, hi, coverage).
    For n < 10 it is the sample range or near it (coverage well above 95 %), so it is not reported then."""
    s = sorted(xs)
    n = len(s)
    if n < 6:
        return (float("nan"), float("nan"), float("nan"))
    cdf = lambda k: sum(math.comb(n, i) for i in range(0, k + 1)) / 2.0 ** n
    j = 0
    while 1 - 2 * cdf(j) >= level:      # largest j (1-based rank j+1 ... ) keeping coverage >= level
        j += 1
    cov = 1 - 2 * cdf(j - 1)
    return (s[j - 1], s[n - j], cov)


def describe(xs):
    xs = [x for x in xs if x is not None and not math.isnan(x)]
    if not xs:
        return dict(n=0)
    n = len(xs)
    mean = sum(xs) / n
    sd = math.sqrt(sum((x - mean) ** 2 for x in xs) / (n - 1)) if n > 1 else float("nan")
    lo, hi, cov = median_ci(xs)
    return dict(n=n, median=quantile(xs, 0.5), q1=quantile(xs, 0.25), q3=quantile(xs, 0.75),
                min=min(xs), max=max(xs), mean=mean, sd=sd, median_ci95=[lo, hi], median_ci_coverage=cov)


def wilson(k, n, z=1.959964):
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def _bisect(f, lo=0.0, hi=1.0, it=200):
    """Root of a monotone increasing f on [lo, hi]."""
    for _ in range(it):
        mid = 0.5 * (lo + hi)
        if f(mid) < 0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def binom_sf(k, n, p):
    """P(X >= k) for X ~ Binomial(n, p)."""
    return sum(math.comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(k, n + 1))


def clopper_pearson(k, n, alpha=0.05):
    if n == 0:
        return (float("nan"), float("nan"))
    lo = 0.0 if k == 0 else _bisect(lambda p: binom_sf(k, n, p) - alpha / 2)
    hi = 1.0 if k == n else _bisect(lambda p: alpha / 2 - (1 - binom_sf(k + 1, n, p)))
    return (lo, hi)


def betainc(a, b, x):
    """Regularized incomplete beta function I_x(a, b) (continued fraction, modified Lentz)."""
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    if x > (a + 1.0) / (a + b + 2.0):
        return 1.0 - betainc(b, a, 1.0 - x)
    tiny = 1e-300
    front = math.exp(math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b) + a * math.log(x) + b * math.log(1.0 - x)) / a
    c, d = 1.0, 1.0 - (a + b) * x / (a + 1.0)
    d = 1.0 / (d if abs(d) > tiny else tiny)
    f = d
    for m in range(1, 500):
        for num_ in (m * (b - m) * x / ((a + 2 * m - 1) * (a + 2 * m)),
                     -(a + m) * (a + b + m) * x / ((a + 2 * m) * (a + 2 * m + 1))):
            d = 1.0 + num_ * d
            d = 1.0 / (d if abs(d) > tiny else tiny)
            c = 1.0 + num_ / c
            c = c if abs(c) > tiny else tiny
            f *= c * d
        if abs(c * d - 1.0) < 1e-15:
            break
    return front * f


def jeffreys(k, n, alpha=0.05):
    if n == 0:
        return (float("nan"), float("nan"))
    a, b = k + 0.5, n - k + 0.5
    lo = 0.0 if k == 0 else _bisect(lambda p: betainc(a, b, p) - alpha / 2)
    hi = 1.0 if k == n else _bisect(lambda p: betainc(a, b, p) - (1 - alpha / 2))
    return (lo, hi)


def proportion(k, n):
    return dict(k=k, n=n, p=k / n if n else float("nan"), wilson95=wilson(k, n), clopper_pearson95=clopper_pearson(k, n),
                jeffreys95=jeffreys(k, n))


def rnd(x, nd=1):
    """Round half up (the convention of the manuscript) and format; '-' for missing values."""
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "-"
    q = Decimal(1).scaleb(-nd)
    return str(Decimal(repr(float(x))).quantize(q, rounding=ROUND_HALF_UP))


def fmt(d, unit="", nd=2):
    if not d or d.get("n", 0) == 0:
        return "n = 0"
    lo, hi = d["median_ci95"]
    ci = ("(95%% CI %s-%s)" % (rnd(lo, nd), rnd(hi, nd))) if d["n"] >= 10 else "(CI not reported, n < 10)"
    return "n = %d, median = %s%s %s, IQR %s-%s, range %s-%s, mean %s +- SD %s" % (
        d["n"], rnd(d["median"], nd), unit, ci, rnd(d["q1"], nd), rnd(d["q3"], nd), rnd(d["min"], nd),
        rnd(d["max"], nd), rnd(d["mean"], nd), rnd(d["sd"], nd))


def pct(iv):
    return "%s-%s" % (rnd(100 * iv[0], 1), rnd(100 * iv[1], 1))


# --------------------------------------------------------------------------------------------- parsing
def ts(stamp):
    return datetime.strptime(stamp, "%Y%m%d_%H%M%S")


def num(text, key):
    m = RE_NUM[key].search(text)
    if not m or m.group(1) == "nan":
        return float("nan")
    return float(m.group(1))


def flight_logs(logs):
    """(start time, craft, key) of every per-flight log. Only the file names are used: they come from the *_orbit_*.txt
    files when present, and from flight_log_names.txt (one file name per line) in the released raw records."""
    names = set(os.path.basename(p) for p in glob.glob(os.path.join(logs, "*_orbit_*.txt")))
    idx = os.path.join(logs, "flight_log_names.txt")
    if os.path.isfile(idx):
        with open(idx, encoding="utf-8") as f:
            names.update(ln.strip() for ln in f if ln.strip())
    out = []
    for n in names:
        m = RE_FLIGHTLOG.search(n)
        if m:
            out.append((ts(m.group(1)), m.group(2), n[:-4]))
    return sorted(out)


def flog_for(t0, flogs, t_next):
    """The first per-flight log opened after the supervisor start and before the next supervisor start (at most
    600 s later; usually 0-15 s, longer when the game had to be restarted): (craft, key) or (None, None)."""
    for t, craft, key in flogs:
        if t0 <= t < t_next and t - t0 <= timedelta(seconds=600):
            return craft, key
    return None, None


def start_point(text, wd):
    m = RE_CKPT.search(text)
    if m:
        for prefix, name in CHECKPOINT_START:
            if m.group(1).startswith(prefix):
                return name
        return "other checkpoint (%s)" % m.group(1)
    if "ascent (own)" in text or "TWR" in text or "orbit_recover.py" in text:
        return "pad"
    return DR_RUNS.get(wd, {}).get("start", "other")


def parse_wd(path, flogs, t_next):
    wdname = os.path.basename(path)
    stamp = RE_WD.search(wdname).group(1)
    t0 = ts(stamp)
    text = open(path, encoding="utf-8", errors="replace").read()
    fcraft, key = flog_for(t0, flogs, t_next)
    mc = RE_CORE.search(text)
    craft = mc.group(1) if mc else (fcraft or DR_RUNS.get(wdname, {}).get("craft"))
    rec = dict(wd=wdname, start=t0, key=key, craft=craft, craft_source="transcript" if mc else
               ("flight log" if fcraft else ("decision record" if craft else "unresolved")),
               start_point=start_point(text, wdname), full=False, target=None, result=None, detail="",
               miss=float("nan"), v_td=float("nan"), upright=float("nan"), fuel_left=float("nan"),
               parts=float("nan"), orbit=None, deadman=text.count("DEADMAN: control loop"),
               deadman_s=[(float(a), bool(b)) for a, b in RE_DM.findall(text)], eb_dv=None, eb_prop=None,
               note=DR_RUNS.get(wdname, {}).get("note", ""))
    mt = RE_TARGET.search(text)
    if mt and "ascent (own)" in text:
        rec["full"] = True
        rec["target"] = tuple(float(x) for x in mt.groups())     # (Pe km, Ap km, inc deg)
    mr = RE_RESULT.search(text)
    if mr:
        rec["result"], rec["detail"] = mr.group(1), mr.group(2).strip()
        for k in ("miss", "v_td", "upright", "fuel_left", "parts"):
            rec[k] = num(mr.group(0), k)
        if rec["result"] == "SEPTEST":
            for k in ("parts", "thr", "explosions", "core_hit", "q"):
                rec["sep_" + k] = num(mr.group(0), k)
    # recording workload: whole-mission director with OBS screen recording, or in-game frame capture
    if "obsrec: OBS recording" in text:
        rec["recording"] = "screen recording"
    elif "director: on" in text:
        rec["recording"] = "in-game frame capture"
    else:
        rec["recording"] = DR_RECORDING.get(wdname, "")
    # orbit at payload release = the last orbit report before "payload released" (trim burns follow the
    # circularisation when its error exceeds the trim threshold); the first report is kept as orbit_first
    cut = text.find("payload released")
    obs = list(RE_ORBIT.finditer(text if cut < 0 else text[:cut]))
    if obs:
        for k, mo in (("orbit_first", obs[0]), ("orbit", obs[-1])):
            ap, pe, inc, eap, epe = (float(x) for x in mo.groups())
            rec[k] = dict(ap_km=ap, pe_km=pe, inc=inc, err_ap_m=eap, err_pe_m=epe)
        rec["orbit_reports"] = len(obs)
    mw, mp = RE_WIN.search(text), RE_PLAN.search(text)
    if mw:
        rec["deorbit"] = dict(wait_h=float(mw.group(2)), dv_normal=float(mw.group(3)), dv_total=float(mw.group(4)))
    elif mp:
        rec["deorbit"] = dict(wait_h=float(mp.group(3)) / 3600.0, dv_normal=float(mp.group(2)), dv_total=float(mp.group(1)))
    mq, mpp, msw = RE_SEPQ.search(text), RE_SEPP.search(text), RE_SEPW.search(text)
    rec["sep_q_kPa"] = float(mq.group(1)) if mq else None
    rec["sep_parts_ok"] = (mpp.group(2) == mpp.group(3)) if mpp else None
    rec["sep_win"] = tuple(int(x) for x in msw.groups()) if msw else None     # (all, booster, core) in 15 s
    rec["hib"] = [(float(a), float(b) - float(c)) for a, b, c in RE_HIB.findall(text)]   # (s asleep, EC used)
    me = RE_EB_END.search(text)
    if me:
        rec["eb_dv"] = float(me.group(1))
        rec["eb_prop"] = float(me.group(2)) if me.group(2) else None
    # timing on the supervisor clock; lift-off is taken as the start of the own-code ascent
    ma, ms, mb = RE_T_ASCENT.search(text), RE_T_STAGE.search(text), RE_T_ABORT.search(text)
    rec["t_ascent_s"] = float(ma.group(1)) if ma else None
    rec["t_booster_stage_s"] = float(ms.group(1)) if ms else None
    rec["t_abort_s"] = float(mb.group(1)) if mb else None
    # vacuum mid-course corrections, range-trim pulses, landing-burn ignition
    vac = [dict(i=int(a), miss=float(b), along=float(c), cross=float(d), spread=float(e), trim=float(f))
           for a, b, c, d, e, f in RE_VAC.findall(text)]
    rec["vac"] = vac
    rec["vac_burns"] = sum(1 for v in vac if v["miss"] >= 300 and 0.02 <= v["trim"] <= 15 and v["spread"] <= 500)
    rec["vac_ratio"] = [float(x) for x in RE_TRIMFX.findall(text)]
    pulses = [int(x) for x in RE_PULSE.findall(text)]
    rec["pulse_logged_max"] = max(pulses) if pulses else 0      # only every fifth pulse is logged (#1, #6, #11, ...)
    rec["vernier_flown"] = "vernier done" in text
    mi = RE_IGN.search(text)
    rec["ign_tr"] = dict(h=float(mi.group(1)), vz=float(mi.group(2)), vh=float(mi.group(3)),
                         offset=float(mi.group(4))) if mi else None
    return rec


def load_wd(logs):
    flogs = flight_logs(logs)
    paths = sorted(p for p in glob.glob(os.path.join(logs, "wd_*.out")) if RE_WD.search(os.path.basename(p)))
    starts = [ts(RE_WD.search(os.path.basename(p)).group(1)) for p in paths] + [datetime.max]
    return [parse_wd(p, flogs, starts[i + 1]) for i, p in enumerate(paths)]


def jsonl_key(r):
    log = r.get("log") or ""
    return os.path.basename(log)[:-4] if log.endswith(".txt") else None


def jsonl_start(r):
    m = re.match(r"(\d{8}_\d{6})_", r.get("key") or "")
    return ts(m.group(1)) if m else datetime.fromisoformat(r["time"])


def load_jsonl(logs):
    rows, raw = [], []
    with raw_open(os.path.join(logs, "experiments.jsonl"), "rt", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                r = json.loads(line)
                r["key"] = jsonl_key(r)          # path-free join key
                r.pop("log", None)               # local file path; never copied to the outputs
                r["_start"] = jsonl_start(r)
                rows.append(r)
                raw.append(line)
    return rows, raw


def load_matrix(logs):
    rows, raw = [], []
    pat = re.compile(r"^(\S+ \S+) (\S+) (\S+) (\d+)/(\d+)/(\d+) ?(.*?) ?RESULT (PASS|FAIL) miss=(\S+) fuel=(\S+) "
                     r"orbit=\[(.*?)\] wd=logs/(\S+)$")
    with raw_open(os.path.join(logs, "matrix_results.txt"), "rt", encoding="utf-8") as f:
        for line in f:
            m = pat.match(line.strip())
            if not m:
                continue
            (t, case, craft, pe, ap, inc, env, res, miss, fuel, orbit, wd) = m.groups()
            rows.append(dict(time=t, case=case, craft=craft, pe=float(pe), ap=float(ap), inc=float(inc),
                             env=env.strip(), result=res,
                             miss=float(miss.rstrip("m")) if miss != "-" else float("nan"),
                             fuel=float(fuel) if fuel != "-" else float("nan"), orbit=orbit, wd=wd,
                             _start=ts(RE_WD.search(wd).group(1))))
            raw.append(line)
    return rows, raw


def parse_landing_csv(path):
    """Contact position and speed, ignition state, loop period and post-entry coast thrust from the landing CSV."""
    if not path or not raw_exists(path):
        return None
    with raw_open(path, "rt", encoding="utf-8", errors="replace") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None

    def fl(r, k):
        v = r.get(k)
        try:
            return float(v)
        except (TypeError, ValueError):
            return float("nan")

    hg = "h_gnd" if "h_gnd" in rows[0] else "h"
    burn = [r for r in rows if r.get("phase") == "BURN"]
    out = {}
    last = rows[-1]
    out["contact_e"], out["contact_n"] = fl(last, "e"), fl(last, "n")
    first_contact = next((r for r in burn if fl(r, hg) <= 0.3), last)
    out["v_contact"] = abs(fl(first_contact, "vu"))
    if burn:
        b = burn[0]
        out["ign"] = dict(h=fl(b, "h"), e=fl(b, "e"), n=fl(b, "n"), offset=math.hypot(fl(b, "e"), fl(b, "n")),
                          vh=math.hypot(fl(b, "ve"), fl(b, "vn")), vz=fl(b, "vu"), m=fl(b, "m"))
        uts = [fl(r, "ut") for r in burn]
        dts = sorted(u2 - u1 for u1, u2 in zip(uts, uts[1:]) if u2 > u1)
        if dts:
            out["loop_burn"] = dict(median=quantile(dts, 0.5), p95=quantile(dts, 0.95), max=dts[-1])
    # post-entry coast: COAST rows with a throttle command form segments; the first segment of >= 3 s game time is
    # the entry burn, later ones are the vernier and the range-trim pulses
    segs, cur = [], None
    for i, r in enumerate(rows):
        on = r.get("phase") == "COAST" and fl(r, "thr_cmd") > 0.01
        if on and cur is None:
            cur = [i, i]
        elif on:
            cur[1] = i
        elif cur is not None:
            segs.append(cur)
            cur = None
    eb = next((s for s in segs if fl(rows[min(s[1] + 1, len(rows) - 1)], "ut") - fl(rows[s[0]], "ut") >= 3.0), None)
    if eb and burn:
        i_after = min(eb[1] + 1, len(rows) - 1)
        out["coast_segments"] = sum(1 for s in segs if s[0] > eb[1])
        out["coast_prop_kg"] = fl(rows[i_after], "m") - fl(burn[0], "m")
    return out


# --------------------------------------------------------------------------------------------- outputs
def write_table(out, name, header, rows, note=""):
    with open(os.path.join(out, "tables", name + ".csv"), "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f, lineterminator="\n")        # quotes header and data fields that contain commas
        w.writerow([str(c) for c in header])
        for r in rows:
            w.writerow([str(c) for c in r])
    with open(os.path.join(out, "tables", name + ".md"), "w", encoding="utf-8", newline="\n") as f:
        f.write("| " + " | ".join(header) + " |\n|" + "---|" * len(header) + "\n")
        for r in rows:
            f.write("| " + " | ".join(str(c) for c in r) + " |\n")
        if note:
            f.write("\n" + note + "\n")


def sha256_bytes(b):
    return hashlib.sha256(b).hexdigest()


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def inside(path, root):
    path, root = os.path.realpath(path), os.path.realpath(root)
    return path == root or path.startswith(root + os.sep)


RE_RECORD_FILE = re.compile(r"(wd_\d{8}_\d{6}\.out|.*_landing\.csv(\.gz)?|experiments\.jsonl|matrix_results\.txt|"
                            r"flight_log_names\.txt|rdv_r1_.*\.json)$")


def raw_manifest(root=RAW_ROOT, only_records=False):
    """[(path relative to root with '/', sha256)] of every file under root, sorted by path. Documentation files
    (README.md, ORIGINAL_SHA256.txt) are not raw records and are not listed. The hash is of the stored bytes (so of the .gz for compressed
    files). only_records restricts the listing to the record files named in the module docstring."""
    ents = []
    for dp, dn, fn in os.walk(root):
        dn.sort()
        for n in fn:
            if n.startswith(".") or (n in ("README.md", "ORIGINAL_SHA256.txt") and dp == root) or (only_records and not RE_RECORD_FILE.match(n)):
                continue
            p = os.path.join(dp, n)
            ents.append((os.path.relpath(p, root).replace(os.sep, "/"), sha256_file(p)))
    return sorted(ents)


def write_manifest(man, path, root="data/raw"):
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write("# SHA-256 of the raw records in %s (paths relative to it); written by compute_stats.py, "
                "verified by compute_stats.py --check-manifest\n" % root)
        for rel, h in man:
            f.write("%s  %s\n" % (h, rel))


def check_manifest():
    """Return 0 if data/raw matches paper/tables/raw_manifest.sha256, else 1 (differences are printed)."""
    if not os.path.isfile(MANIFEST_PATH):
        die("manifest not found: %s" % MANIFEST_PATH)
    if not os.path.isdir(RAW_ROOT):
        die("raw-record directory not found: %s\n%s" % (RAW_ROOT, RAW_HINT))
    want = {}
    with open(MANIFEST_PATH, encoding="utf-8") as f:
        for ln in f:
            if ln.strip() and not ln.startswith("#"):
                h, rel = ln.rstrip("\n").split("  ", 1)
                want[rel] = h
    have = dict(raw_manifest(RAW_ROOT))
    missing = sorted(set(want) - set(have))
    extra = sorted(set(have) - set(want))
    bad = sorted(k for k in want if k in have and want[k] != have[k])
    for tag, lst in (("MISSING   ", missing), ("MODIFIED  ", bad), ("UNLISTED  ", extra)):
        for k in lst[:15]:
            print(tag + k)
        if len(lst) > 15:
            print("%s... and %d more" % (tag, len(lst) - 15))
    ok = not (missing or bad or extra)
    print("manifest %s: %d listed, %d present, %d missing, %d modified, %d unlisted" % (
        "OK" if ok else "FAILED", len(want), len(have), len(missing), len(bad), len(extra)))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="?", default=DEFAULT_LOGS,
                    help="directory with wd_*.out, experiments.jsonl, matrix_results.txt (default: data/raw/logs of "
                         "this repository)")
    ap.add_argument("--out", default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ap.add_argument("--check-manifest", action="store_true",
                    help="verify the SHA-256 of every file under data/raw against paper/tables/raw_manifest.sha256 "
                         "and exit")
    ap.add_argument("--no-figures", action="store_true", help="accepted for compatibility; figures are drawn by "
                                                               "paper/figures/make_all.py")
    ap.add_argument("--freeze", default=DATA_FREEZE.isoformat(),
                    help="analyse only runs that started before this local time (default: the report's data freeze)")
    a = ap.parse_args()
    if a.check_manifest:
        sys.exit(check_manifest())
    a.logs = os.path.abspath(a.logs)
    require_logs(a.logs)
    os.makedirs(os.path.join(a.out, "tables"), exist_ok=True)

    freeze = datetime.fromisoformat(a.freeze)
    wd = [r for r in load_wd(a.logs) if r["start"] < freeze]
    js_all, js_raw = load_jsonl(a.logs)
    js = [r for r in js_all if r["_start"] < freeze]
    mx_all, mx_raw = load_matrix(a.logs)
    mx = [r for r in mx_all if r["_start"] < freeze]
    js_by_key = {}
    for r in js:
        if r["key"]:
            js_by_key.setdefault(r["key"], []).append(r)
    for r in wd:                       # join: transcript -> experiment-log record and landing CSV (path-free key)
        j = js_by_key.get(r["key"]) if r["key"] else None
        r["js"] = j[-1] if j else None
        r["miss2"] = float(r["js"]["result"]["miss"]) if r["js"] else float("nan")
        r["git"] = r["js"]["git"] if r["js"] else None
        r["csv"] = parse_landing_csv(os.path.join(a.logs, r["key"] + "_landing.csv")) if r["key"] else None

    script = os.path.abspath(__file__)
    S = dict(definitions=dict(pass_rule=PASS_RULE, campaign_start=CAMPAIGN_START.isoformat(), data_freeze=a.freeze,
                              gf_family=GF_FAMILY, gate="%d of 3 flights within %.0f m from checkpoint %s" % (
                                  GATE_NEED, GATE_LIMIT_M, TERM_GATE_SAVE),
                              freeze_rule="every source filtered on the start time of its parent run",
                              miss_source="experiments.jsonl (0.01 m) joined by key; transcript (0.1 m) as fallback "
                                          "and for outcome classification",
                              propellant="kg of LiquidFuel + Oxidizer = LF units x %.4f" % LFOX_KG_PER_LF,
                              rounding="half up"),
             script_sha256=sha256_bytes(open(script, "rb").read()))
    keys_wd = set(r["key"] for r in wd if r["key"])
    S["records"] = dict(wd_files=len(wd), jsonl_rows=len(js), matrix_rows=len(mx),
                        first_wd=min(r["start"] for r in wd).isoformat(), last_wd=max(r["start"] for r in wd).isoformat(),
                        unresolved_craft=[r["wd"] for r in wd if r["craft"] is None],
                        unclassified_start=[r["wd"] for r in wd if r["start_point"] == "other"],
                        jsonl_unjoined=sum(1 for r in js if r["key"] not in keys_wd))

    # ---- raw-record manifest (SHA-256 of every file of the released raw records) ---------------------------
    # For the repository's data/raw this is the file verified by --check-manifest. For records kept elsewhere, only the
    # record files (transcripts, landing CSVs, experiment and matrix logs) are listed, relative to LOGS_DIR, and the
    # repository's own manifest is never overwritten.
    target = os.path.join(a.out, "tables", "raw_manifest.sha256")
    if inside(a.logs, RAW_ROOT):
        man = raw_manifest(RAW_ROOT)
    else:
        man = raw_manifest(a.logs, only_records=True)
    if inside(a.logs, RAW_ROOT) or os.path.realpath(target) != os.path.realpath(MANIFEST_PATH):
        write_manifest(man, target, "data/raw" if inside(a.logs, RAW_ROOT) else a.logs)
        S["records"]["manifest_entries"] = len(man)
    else:
        print("note: %s is outside data/raw; the repository manifest %s was not overwritten (use --out to write "
              "elsewhere)" % (a.logs, os.path.relpath(MANIFEST_PATH, REPO_ROOT)))
        S["records"]["manifest_entries"] = 0

    # ---- flow of every transcript (CONSORT-style) ------------------------------------------------------------
    def family(cr):
        if cr in GF_FAMILY:
            return "T-WorksFineGF family"
        if cr in NO_AIRBRAKE:
            return "GF variants without airbrakes"
        return cr or "unresolved"

    def inclusion(r):
        in_win = r["start"] >= CAMPAIGN_START
        if r["craft"] in GF_FAMILY and r["start_point"] == "pad" and in_win:
            if not r["full"]:
                return "in window, not launched (counted, not analysed)"
            if r["result"] == "SEPTEST":
                return "excluded: separation-only test"
            if r["result"] is None:
                return "excluded: no outcome line"
            return "analysed: complete mission"
        return "outside the exploratory window definition"

    flow = []
    for r in wd:
        r["inclusion"] = inclusion(r)
        flow.append([r["wd"], r["start"].strftime("%Y-%m-%d %H:%M:%S"), r["craft"] or "unresolved", r["craft_source"],
                     family(r["craft"]), r["start_point"], "in" if r["start"] >= CAMPAIGN_START else "before",
                     r["result"] or "none", r["inclusion"], r["note"]])
    write_table(a.out, "flow_all_transcripts",
                ["Transcript", "Start (local)", "Vehicle", "Vehicle source", "Family", "Starting point", "Window",
                 "Outcome", "Inclusion", "Note"], flow,
                note="Every supervisor transcript started before the data freeze. Window: started on or after %s. "
                     "Vehicle source: transcript ('flying core'), the first per-flight log opened after the transcript "
                     "starts (and before the next one), or the decision record." % CAMPAIGN_START.strftime("%Y-%m-%d %H:%M"))
    agg = {}
    for r in wd:
        k = (family(r["craft"]), r["start_point"], "in" if r["start"] >= CAMPAIGN_START else "before")
        agg.setdefault(k, {})
        agg[k][r["result"] or "none"] = agg[k].get(r["result"] or "none", 0) + 1
    outs = ("PASS", "FAIL", "REFUSED", "SEPTEST", "none")
    fam_order = {"T-WorksFineGF family": 0, "GF variants without airbrakes": 1}
    t_flow = []
    for k in sorted(agg, key=lambda k: (fam_order.get(k[0], 2), k[0], k[1], k[2] != "before")):
        v = agg[k]
        t_flow.append(list(k) + [v.get(o, 0) for o in outs] + [sum(v.values())])
    t_flow.append(["Total", "", ""] + [sum(agg[k].get(o, 0) for k in agg) for o in outs] + [len(wd)])
    write_table(a.out, "flow_summary", ["Family", "Starting point", "Window", "PASS", "FAIL", "REFUSED", "SEPTEST",
                                        "No outcome", "Transcripts"], t_flow)
    S["flow"] = dict(transcripts=len(wd), by_inclusion={}, table=t_flow)
    for r in wd:
        S["flow"]["by_inclusion"][r["inclusion"]] = S["flow"]["by_inclusion"].get(r["inclusion"], 0) + 1

    # ---- robustness matrix ----------------------------------------------------------------------------------
    wd_by = {r["wd"]: r for r in wd}
    t1 = []
    for r in mx:
        w = wd_by.get(r["wd"])
        o = w["orbit"] if w else None
        consistent = w is not None and w["result"] == r["result"]
        t1.append([r["case"], r["craft"], PAYLOAD_T.get(r["craft"], "-"), "%g x %g" % (r["pe"], r["ap"]),
                   "%g" % r["inc"], r["env"] or "defaults",
                   r["result"] + ("" if r["result"] == "PASS" else " (%s)" % (w["detail"] if w else "")),
                   rnd(w["miss2"], 2) if w else "-", rnd(w["v_td"], 2) if w else "-",
                   rnd(r["fuel"] * LFOX_KG_PER_LF, 0) if not math.isnan(r["fuel"]) else "-",
                   ("%+d / %+d" % (o["err_ap_m"], o["err_pe_m"])) if o else "-",
                   rnd(o["inc"] - r["inc"], 3) if o else "-",
                   ("%.0f / %.0f / %s" % (w["deorbit"]["dv_total"], abs(w["deorbit"]["dv_normal"]),
                                          rnd(w["deorbit"]["wait_h"], 1))) if w and w.get("deorbit") else "-",
                   "yes" if consistent else "NO"])
    mpass = [r for r in mx if r["result"] == "PASS"]
    S["matrix"] = dict(attempts=len(mx), cases=len(set(r["case"] for r in mx)), passed=len(mpass),
                       attempt=proportion(len(mpass), len(mx)),
                       miss=describe([wd_by[r["wd"]]["miss2"] for r in mpass if r["wd"] in wd_by]),
                       fuel_kg=describe([r["fuel"] * LFOX_KG_PER_LF for r in mpass]),
                       overrides=sorted(set(r["env"] for r in mx if r["env"])),
                       inc_range=[min(r["inc"] for r in mx), max(r["inc"] for r in mx)],
                       orbit_km=sorted(set(r["ap"] for r in mx)),
                       payload_t=sorted(set(PAYLOAD_T[r["craft"]] for r in mx)))
    write_table(a.out, "matrix_cases",
                ["Case", "Vehicle", "Payload (t)", "Orbit (km)", "Inclination (deg)", "Overrides", "Outcome",
                 "Miss (m)", "Post-contact speed (m/s)", "Propellant remaining (kg)", "Ap / Pe error (m)",
                 "Inclination error (deg, signed)", "Deorbit dv total / normal (m/s) / wait (h)", "Matches supervisor log"],
                t1, note="Miss at 0.01 m from the experiment log. Propellant = LiquidFuel units x %.2f kg (liquid fuel and "
                         "oxidizer at the standard 0.9 : 1.1 ratio, 5 kg per unit)." % LFOX_KG_PER_LF)

    # ---- exploratory window: complete end-to-end missions ----------------------------------------------------
    camp = [r for r in wd if r["full"] and r["start"] >= CAMPAIGN_START and r["craft"] in GF_FAMILY]
    excl_sep = [r for r in camp if r["result"] == "SEPTEST"]
    excl_none = [r for r in camp if r["result"] is None]
    excl_ref = [r for r in camp if r["result"] == "REFUSED"]
    done = [r for r in camp if r["result"] in ("PASS", "FAIL")]
    for i, r in enumerate(done, 1):
        r["mission"] = i
    ok = [r for r in done if r["result"] == "PASS"]
    fails = [r for r in done if r["result"] == "FAIL"]
    not_launched = [r for r in wd if r["inclusion"].startswith("in window, not launched")]
    base = lambda r: r["craft"] == "T-WorksFineGF" and r["target"] == (100.0, 100.0, 6.0)
    csvd = [r for r in ok if r["csv"]]
    rec_done = [r for r in done if r["recording"]]
    c = S["campaign"] = dict(
        candidates=len(camp), excluded_separation_tests=len(excl_sep), excluded_no_result=len(excl_none),
        excluded_refused=len(excl_ref),
        in_window_not_launched=[dict(wd=r["wd"], craft=r["craft"], result=r["result"], detail=r["detail"][:90],
                                     note=r["note"]) for r in not_launched],
        completed=len(done), passed=len(ok), success=proportion(len(ok), len(done)),
        failures=[dict(mission=r["mission"], wd=r["wd"], detail=r["detail"][:80], miss=r["miss"], miss2=r["miss2"])
                  for r in fails],
        excluded_no_result_files=[dict(wd=r["wd"], note=r["note"]) for r in excl_none],
        miss_pass=describe([r["miss2"] for r in ok]), miss_pass_transcript=describe([r["miss"] for r in ok]),
        miss_pass_baseline=describe([r["miss2"] for r in ok if base(r)]),
        miss_pass_nonbaseline=describe([r["miss2"] for r in ok if not base(r)]),
        miss2_missing=[r["wd"] for r in ok if math.isnan(r["miss2"])],
        v_td_pass=describe([r["v_td"] for r in ok]),
        v_contact_pass=describe([r["csv"]["v_contact"] for r in csvd]),
        fuel_pass_kg=describe([r["fuel_left"] * LFOX_KG_PER_LF for r in ok]),
        fuel_pass_lf_units=describe([r["fuel_left"] for r in ok]),
        err_ap=describe([abs(r["orbit"]["err_ap_m"]) for r in done if r["orbit"]]),
        err_pe=describe([abs(r["orbit"]["err_pe_m"]) for r in done if r["orbit"]]),
        err_pe_signed=describe([r["orbit"]["err_pe_m"] for r in done if r["orbit"]]),
        err_ap_signed=describe([r["orbit"]["err_ap_m"] for r in done if r["orbit"]]),
        pe_negative=sum(1 for r in done if r["orbit"] and r["orbit"]["err_pe_m"] < 0),
        err_inc=describe([abs(r["orbit"]["inc"] - r["target"][2]) for r in done if r["orbit"]]),
        orbits_reached=sum(1 for r in done if r["orbit"]),
        trimmed=[dict(mission=r["mission"], first_err_ap=r["orbit_first"]["err_ap_m"],
                      first_err_pe=r["orbit_first"]["err_pe_m"], final_err_ap=r["orbit"]["err_ap_m"],
                      final_err_pe=r["orbit"]["err_pe_m"], recording=r["recording"])
                 for r in done if r["orbit"] and r.get("orbit_reports", 1) > 1],
        by_craft={k: dict(completed=sum(r["craft"] == k for r in done), passed=sum(r["craft"] == k for r in ok))
                  for k in GF_FAMILY},
        under_2m=sum(r["miss2"] < 2.0 for r in ok),
        eb_prop_after_fix=describe([r["eb_prop"] for r in done if r["eb_prop"]]),
        eb_dv_after_fix=describe([r["eb_dv"] for r in done if r["eb_prop"]]),
        eb_dv_all=describe([r["eb_dv"] for r in done if r["eb_dv"]]),
        deadman_total=sum(r["deadman"] for r in done),
        deadman_events=[dict(mission=r["mission"], wd=r["wd"], stalls_s=r["deadman_s"]) for r in done if r["deadman"]],
        hibernation=dict(segments=sum(len(r["hib"]) for r in done),
                         seconds=sum(t for r in done for t, _ in r["hib"]),
                         ec_used=sum(e for r in done for _, e in r["hib"]),
                         longest_s=max([t for r in done for t, _ in r["hib"]] or [0]),
                         flights=sum(1 for r in done if r["hib"])),
        sep_q=describe([r["sep_q_kPa"] for r in done if r["sep_q_kPa"]]),
        sep_parts_checked=sum(1 for r in done if r["sep_parts_ok"] is not None),
        sep_parts_ok=sum(1 for r in done if r["sep_parts_ok"]),
        sep_window_flights=sum(1 for r in done if r["sep_win"]),
        sep_window_booster=sum(1 for r in done if r["sep_win"] and r["sep_win"][1] > 0),
        sep_window_core=sum(1 for r in done if r["sep_win"] and r["sep_win"][2] > 0),
        deorbit_wait_h=describe([r["deorbit"]["wait_h"] for r in done if r.get("deorbit")]),
        deorbit_dv=describe([r["deorbit"]["dv_total"] for r in done if r.get("deorbit")]),
        recording=dict(missions=[dict(mission=r["mission"], wd=r["wd"], kind=r["recording"]) for r in rec_done],
                       rec=proportion(sum(1 for r in ok if r["recording"]), len(rec_done)),
                       nonrec=proportion(sum(1 for r in ok if not r["recording"]), len(done) - len(rec_done))))
    # descent decomposition: ignition state, contact position (signed), coast thrust, loop period
    ign = [r["csv"]["ign"] for r in csvd if "ign" in r["csv"]]
    c["ignition_h"] = describe([x["h"] for x in ign])
    c["ignition_offset_m"] = describe([x["offset"] for x in ign])
    c["ignition_vh"] = describe([x["vh"] for x in ign])
    c["contact_e"] = describe([r["csv"]["contact_e"] for r in csvd])
    c["contact_n"] = describe([r["csv"]["contact_n"] for r in csvd])
    me, mn = c["contact_e"].get("mean", float("nan")), c["contact_n"].get("mean", float("nan"))
    c["contact_bias_m"] = math.hypot(me, mn)
    c["contact_scatter_about_mean_m"] = describe([math.hypot(r["csv"]["contact_e"] - me, r["csv"]["contact_n"] - mn)
                                                  for r in csvd])
    c["contact_radius_m"] = describe([math.hypot(r["csv"]["contact_e"], r["csv"]["contact_n"]) for r in csvd])
    c["landing_csv_flights"] = len(csvd)
    c["coast_segments"] = describe([r["csv"]["coast_segments"] for r in csvd if "coast_segments" in r["csv"]])
    c["coast_prop_kg"] = describe([r["csv"]["coast_prop_kg"] for r in csvd if "coast_prop_kg" in r["csv"]])
    c["pulse_logged_max"] = describe([r["pulse_logged_max"] for r in ok])
    c["loop_burn_median_s"] = describe([r["csv"]["loop_burn"]["median"] for r in csvd if "loop_burn" in r["csv"]])
    c["loop_burn_p95_s"] = describe([r["csv"]["loop_burn"]["p95"] for r in csvd if "loop_burn" in r["csv"]])
    c["loop_burn_max_s"] = describe([r["csv"]["loop_burn"]["max"] for r in csvd if "loop_burn" in r["csv"]])
    # vacuum mid-course corrections
    vac = [r for r in done if r["vac"]]
    c["vac"] = dict(missions=len(vac), predictions=describe([len(r["vac"]) for r in vac]),
                    burns=describe([r["vac_burns"] for r in vac]),
                    first_along_m=describe([r["vac"][0]["along"] for r in vac]),
                    last_along_m=describe([r["vac"][-1]["along"] for r in vac]),
                    last_miss_m=describe([r["vac"][-1]["miss"] for r in vac]),
                    trim_total_ms=describe([sum(v["trim"] for v in r["vac"]
                                                if v["miss"] >= 300 and 0.02 <= v["trim"] <= 15) for r in vac]),
                    effect_ratio=describe([x for r in vac for x in r["vac_ratio"]]))
    # configuration epochs and the window sensitivity analyses
    ep = []
    for k, (name, first) in enumerate(EPOCHS):
        last = EPOCHS[k + 1][1] - 1 if k + 1 < len(EPOCHS) else len(done)
        sub = [r for r in done if first <= r["mission"] <= last]
        ep.append(dict(epoch=name, missions="%d-%d" % (first, last), git=sorted(set(r["git"] for r in sub if r["git"])),
                       **proportion(sum(r["result"] == "PASS" for r in sub), len(sub))))
    c["epochs"] = ep
    pre = [r for r in wd if r["full"] and r["craft"] in GF_FAMILY and r["start"] < CAMPAIGN_START
           and r["result"] in ("PASS", "FAIL")]
    pre_last2 = pre[-2:]
    k_pre = sum(r["result"] == "PASS" for r in pre)
    sens = [("Exploratory window, complete missions (headline population)", len(ok), len(done)),
            ("Return phase, conditional on reaching orbit", len(ok), c["orbits_reached"]),
            ("Window; monitor-terminated run counted as a failure", len(ok), len(done) + len(excl_none)),
            ("Window; also the in-window pad attempts that never launched", len(ok),
             len(done) + len(excl_none) + len(not_launched)),
            ("Window extended to the two preceding complete missions",
             len(ok) + sum(r["result"] == "PASS" for r in pre_last2), len(done) + len(pre_last2)),
            ("All complete GF-family missions in the records", len(ok) + k_pre, len(done) + len(pre)),
            ("All complete GF-family missions; monitor-terminated run as a failure", len(ok) + k_pre,
             len(done) + len(pre) + len(excl_none)),
            ("Baseline configuration only (2.48 t, 100 km, 6 deg)", sum(1 for r in ok if base(r)),
             sum(1 for r in done if base(r))),
            ("Missions without a recording workload", c["recording"]["nonrec"]["k"], c["recording"]["nonrec"]["n"]),
            ("Missions with a recording workload", c["recording"]["rec"]["k"], c["recording"]["rec"]["n"])]
    sens += [("Configuration epoch %s (missions %s)" % (e["epoch"].split(":")[0], e["missions"]), e["k"], e["n"])
             for e in ep]
    sens += [("After the separation corrective action (missions %d-%d)" % (EPOCHS[1][1], len(done)),
              sum(1 for r in ok if r["mission"] >= EPOCHS[1][1]), sum(1 for r in done if r["mission"] >= EPOCHS[1][1]))]
    c["sensitivity"] = [dict(population=p, **proportion(k, n)) for p, k, n in sens]
    c["pre_window_complete"] = [dict(wd=r["wd"], result=r["result"], detail=r["detail"][:60], miss=r["miss"])
                                for r in pre]
    write_table(a.out, "sensitivity",
                ["Population", "Successes / missions", "Proportion (%)", "Wilson 95% CI (%)",
                 "Clopper-Pearson 95% CI (%)", "Jeffreys 95% CI (%)"],
                [[s["population"], "%d / %d" % (s["k"], s["n"]), rnd(100 * s["p"], 1), pct(s["wilson95"]),
                  pct(s["clopper_pearson95"]), pct(s["jeffreys95"])] for s in c["sensitivity"]],
                note="Intervals are nominal: they assume exchangeable missions, which a sequential development series "
                     "with code changes does not provide.")
    # ascent-loss timing on the mission clock
    m12 = next((r for r in fails if math.isnan(r["miss"])), None)
    if m12 and m12["t_ascent_s"] is not None:
        c["ascent_loss_timing"] = dict(
            mission=m12["mission"], wd=m12["wd"], liftoff_supervisor_s=m12["t_ascent_s"],
            staging_supervisor_s=m12["t_booster_stage_s"], abort_supervisor_s=m12["t_abort_s"],
            staging_after_liftoff_s=(m12["t_booster_stage_s"] or float("nan")) - m12["t_ascent_s"],
            abort_after_liftoff_s=(m12["t_abort_s"] or float("nan")) - m12["t_ascent_s"])
    t2 = []
    for r in done:
        o, cv = r["orbit"], r["csv"] or {}
        ig = cv.get("ign")
        t2.append([r["mission"], r["key"] or r["wd"], r["start"].strftime("%Y-%m-%d %H:%M"), r["git"] or "-", r["craft"],
                   "%g/%g/%g" % r["target"], r["recording"] or "none", r["result"], rnd(r["miss2"], 2), rnd(r["miss"], 1),
                   ("%s / %s" % (rnd(cv["contact_e"], 1), rnd(cv["contact_n"], 1))) if "contact_e" in cv else "-",
                   rnd(r["v_td"], 2), rnd(cv.get("v_contact"), 2),
                   rnd(r["fuel_left"] * LFOX_KG_PER_LF, 0) if not math.isnan(r["fuel_left"]) else "-",
                   ("%+d / %+d" % (o["err_ap_m"], o["err_pe_m"])) if o else "-",
                   len(r["vac"]), r["vac_burns"],
                   ("%+.0f / %+.0f" % (r["vac"][0]["along"], r["vac"][-1]["along"])) if r["vac"] else "-",
                   rnd(r["eb_dv"], 0) if r["eb_dv"] else "-", rnd(r["eb_prop"], 0) if r["eb_prop"] else "-",
                   cv.get("coast_segments", "-"), rnd(cv.get("coast_prop_kg"), 0),
                   ("%s / %s / %s" % (rnd(ig["h"], 0), rnd(ig["offset"], 1), rnd(ig["vh"], 1))) if ig else "-",
                   r["deadman"], rnd(cv.get("loop_burn", {}).get("p95"), 3)])
    write_table(a.out, "campaign_flights",
                ["#", "Key", "Start (local)", "Code rev.", "Vehicle", "Pe/Ap/i (km/km/deg)", "Recording", "Outcome",
                 "Miss (m, 0.01)", "Miss (m, transcript)", "Contact E / N (m)", "Post-contact speed (m/s)",
                 "First-contact vertical speed (m/s)", "Propellant remaining (kg)", "Ap / Pe error (m)",
                 "Vacuum predictions", "Vacuum burns", "Vacuum along error first / last (m)", "Entry-burn dv logged (m/s)",
                 "Entry-burn propellant (kg)", "Post-entry thrust segments", "Post-entry coast propellant (kg)",
                 "Ignition h / offset / v_h (m, m, m/s)", "Loop stalls", "Burn loop period p95 (s)"], t2,
                note="Key: base name of the per-flight log (joins transcript, experiment log and landing CSV). Contact E/N, "
                     "first-contact vertical speed, ignition state, post-entry thrust segments (vernier and range-trim "
                     "pulses), post-entry coast propellant (vessel mass loss from the end of the entry burn to landing-burn "
                     "ignition) and loop period come from the landing CSV (game time). Post-contact speed is the logged "
                     "'touchdown speed', read after the landing loop exits.")

    # ---- terminal-phase regression gates ---------------------------------------------------------------------
    gates = {}
    for r in js:
        e = r.get("env", {})
        if r["start"] == "term" and e.get("KSP_TERM_SAVE") == TERM_GATE_SAVE and r["craft"] == "T-WorksFineGF":
            label = e.get("KSP_REVIEW", "(unlabelled)")
            gates.setdefault(label, []).append(r)
    t3, term_all, ngate, npass = [], [], 0, 0
    for label, rows in sorted(gates.items(), key=lambda kv: kv[1][0]["time"]):
        miss = [x["result"]["miss"] for x in rows]
        term_all += miss
        is_gate = "gate" in label.lower() or "regression" in label.lower()
        verdict = "-"
        if is_gate and len(rows) >= 3:
            ngate += 1
            good = sum(m < GATE_LIMIT_M for m in miss[:3]) >= GATE_NEED and all(x["result"]["ok"] for x in rows[:3])
            npass += good
            verdict = "PASS" if good else "FAIL"
        t3.append([rows[0]["time"][:16].replace("T", " "), rows[0]["git"], label, len(rows),
                   " / ".join(rnd(m, 2) for m in miss), verdict])
    S["gates"] = dict(gate_sessions=ngate, gate_passed=npass, term_flights=len(term_all),
                      term_miss=describe(term_all), term_under5=sum(m < 5.0 for m in term_all))
    write_table(a.out, "regression_gates", ["Session record time", "Code rev.", "Review label", "n", "Miss (m)", "Gate"],
                t3, note="Gate rule: >= %d of the first 3 flights within %.0f m and all flights PASS. Session record time is "
                         "the experiment-log write time of the first flight (its run started 1-2 min earlier)." % (
                             GATE_NEED, GATE_LIMIT_M))

    # ---- entry-phase gates -----------------------------------------------------------------------------------
    eg = [r for r in js if r["start"] == "entry" and "entry gate" in r.get("env", {}).get("KSP_REVIEW", "")]
    S["entry_gates"] = [dict(time=r["time"], review=r["env"]["KSP_REVIEW"], miss=r["result"]["miss"],
                             ok=r["result"]["ok"]) for r in eg]
    eg_wd = [r for r in wd if r["eb_prop"] and not r["full"]]
    S["entry_gate_prop"] = [dict(wd=r["wd"], dv=r["eb_dv"], prop=r["eb_prop"], deadman=r["deadman"], miss=r["miss2"])
                            for r in eg_wd]

    # ---- separation-only tests -------------------------------------------------------------------------------
    sep = [r for r in wd if r["result"] == "SEPTEST"]
    t4 = [[r["start"].strftime("%H:%M"), rnd(r["sep_q"], 1), int(r["sep_parts"]), int(r["sep_thr"]),
           int(r["sep_explosions"]), int(r["sep_core_hit"])] for r in sep]
    S["septests"] = [dict(wd=r["wd"], q_kPa=r["sep_q"], explosions=r["sep_explosions"], core_hit=r["sep_core_hit"],
                          parts=r["sep_parts"], thrust_kN=r["sep_thr"]) for r in sep]
    write_table(a.out, "separation_tests",
                ["Start", "Dynamic pressure at separation (kPa)", "Parts after separation", "Core thrust (kN)",
                 "Explosions within 3 s", "Core parts hit"], t4)

    # ---- every descent-phase record -----------------------------------------------------------------------------
    hist = {}
    for r in js:
        k = (r["craft"], r["start"])
        h = hist.setdefault(k, [0, 0, []])
        h[0] += 1
        h[1] += bool(r["result"]["ok"])
        if r["result"]["ok"]:
            h[2].append(r["result"]["miss"])
    t5 = [[k, s, n, kk, rnd(describe(m).get("median", float("nan")), 2) if m else "-"]
          for (k, s), (n, kk, m) in sorted(hist.items())]
    S["history"] = {"%s|%s" % k: dict(n=v[0], ok=v[1]) for k, v in hist.items()}
    write_table(a.out, "descent_records_by_vehicle", ["Vehicle", "Start point", "Flights", "PASS", "Mdn miss of PASS (m)"],
                t5, note="Source: every row of experiments.jsonl (flights that reached the descent phase).")

    with open(os.path.join(a.out, "tables", "stats.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(S, f, indent=1, default=str)

    # ---- console summary -------------------------------------------------------------------------------------
    print("records:", S["records"])
    print("flow by inclusion:", S["flow"]["by_inclusion"])
    print("matrix: %d attempts, %d cases, %d PASS (Wilson %s %%, CP %s %%); miss %s; propellant kg %s; overrides %s" % (
        S["matrix"]["attempts"], S["matrix"]["cases"], S["matrix"]["passed"], pct(S["matrix"]["attempt"]["wilson95"]),
        pct(S["matrix"]["attempt"]["clopper_pearson95"]), fmt(S["matrix"]["miss"], " m"),
        fmt(S["matrix"]["fuel_kg"], "", 0), S["matrix"]["overrides"]))
    print("window: %d candidates; excluded %d separation tests, %d without RESULT, %d refused; not launched %s" % (
        c["candidates"], c["excluded_separation_tests"], c["excluded_no_result"], c["excluded_refused"],
        c["in_window_not_launched"]))
    print("window: %d/%d PASS (%s %%, Wilson %s %%, CP %s %%, Jeffreys %s %%)" % (
        c["passed"], c["completed"], rnd(100 * c["success"]["p"], 1), pct(c["success"]["wilson95"]),
        pct(c["success"]["clopper_pearson95"]), pct(c["success"]["jeffreys95"])))
    print("  miss (PASS, 0.01 m):", fmt(c["miss_pass"], " m"), "| missing:", c["miss2_missing"])
    print("  miss (PASS, transcript 0.1 m):", fmt(c["miss_pass_transcript"], " m"))
    print("  miss (PASS, baseline):", fmt(c["miss_pass_baseline"], " m"))
    print("  miss (PASS, non-baseline):", fmt(c["miss_pass_nonbaseline"], " m"))
    print("  post-contact speed (PASS):", fmt(c["v_td_pass"], " m/s"))
    print("  first-contact vertical speed:", fmt(c["v_contact_pass"], " m/s"))
    print("  propellant remaining (PASS):", fmt(c["fuel_pass_kg"], " kg", 0), "| LF units:",
          fmt(c["fuel_pass_lf_units"], "", 0))
    print("  |Ap err| m:", fmt(c["err_ap"], "", 0), "| |Pe err| m:", fmt(c["err_pe"], "", 0),
          "| |inc err| deg:", fmt(c["err_inc"], "", 3))
    print("  signed Pe err m:", fmt(c["err_pe_signed"], "", 0), "negative in", c["pe_negative"], "| signed Ap err m:",
          fmt(c["err_ap_signed"], "", 0))
    print("  trimmed orbits:", c["trimmed"])
    print("  failures:", c["failures"])
    print("  by craft:", c["by_craft"], "under 2 m:", c["under_2m"])
    print("  recording:", c["recording"]["missions"])
    print("  recording %d/%d, non-recording %d/%d" % (c["recording"]["rec"]["k"], c["recording"]["rec"]["n"],
                                                      c["recording"]["nonrec"]["k"], c["recording"]["nonrec"]["n"]))
    print("  entry-burn propellant (mass-based accounting):", fmt(c["eb_prop_after_fix"], " kg", 0),
          "| logged dv:", fmt(c["eb_dv_after_fix"], " m/s", 0), "| logged dv, all:", fmt(c["eb_dv_all"], " m/s", 0))
    hb = c["hibernation"]
    print("  hibernation: %d segments in %d flights, %.0f s, %.1f EC (%.6f EC/s), longest %.0f s" % (
        hb["segments"], hb["flights"], hb["seconds"], hb["ec_used"], hb["ec_used"] / max(hb["seconds"], 1), hb["longest_s"]))
    print("  deorbit wait h:", fmt(c["deorbit_wait_h"], " h"), "| deorbit dv:", fmt(c["deorbit_dv"], " m/s", 0))
    print("  separation: q %s; parts check %d/%d; 15 s window in %d flights: booster-side %d, core %d" % (
        fmt(c["sep_q"], " kPa", 1), c["sep_parts_ok"], c["sep_parts_checked"], c["sep_window_flights"],
        c["sep_window_booster"], c["sep_window_core"]))
    print("  loop stalls:", c["deadman_total"], c["deadman_events"])
    print("  landing CSVs: %d | ignition h %s | ignition offset %s | ignition v_h %s" % (
        c["landing_csv_flights"], fmt(c["ignition_h"], " m", 0), fmt(c["ignition_offset_m"], " m", 1),
        fmt(c["ignition_vh"], " m/s", 1)))
    print("  contact E %s | N %s | bias %s m | scatter about mean %s | radius %s" % (
        fmt(c["contact_e"], " m"), fmt(c["contact_n"], " m"), rnd(c["contact_bias_m"], 2),
        fmt(c["contact_scatter_about_mean_m"], " m"), fmt(c["contact_radius_m"], " m")))
    print("  post-entry thrust segments %s | coast propellant %s | logged pulse index %s" % (
        fmt(c["coast_segments"], "", 0), fmt(c["coast_prop_kg"], " kg", 0), fmt(c["pulse_logged_max"], "", 0)))
    print("  burn loop period median %s | p95 %s | max %s" % (
        fmt(c["loop_burn_median_s"], " s", 3), fmt(c["loop_burn_p95_s"], " s", 3), fmt(c["loop_burn_max_s"], " s", 3)))
    v = c["vac"]
    print("  vacuum corrections: %d missions; predictions %s; burns %s; first along %s; last along %s; last miss %s; "
          "total trim %s; effect ratio %s" % (v["missions"], fmt(v["predictions"], "", 0), fmt(v["burns"], "", 0),
                                            fmt(v["first_along_m"], " m", 0), fmt(v["last_along_m"], " m", 0),
                                            fmt(v["last_miss_m"], " m", 0), fmt(v["trim_total_ms"], " m/s", 2),
                                            fmt(v["effect_ratio"], "", 2)))
    print("  epochs:", [(e["epoch"].split(":")[0], e["missions"], "%d/%d" % (e["k"], e["n"]), e["git"]) for e in ep])
    for s in c["sensitivity"]:
        print("  SENS %-75s %2d/%2d  %5s %%  W %s  CP %s  J %s" % (s["population"], s["k"], s["n"], rnd(100 * s["p"], 1),
                                                              pct(s["wilson95"]), pct(s["clopper_pearson95"]),
                                                              pct(s["jeffreys95"])))
    print("  ascent-loss timing:", c.get("ascent_loss_timing"))
    g = S["gates"]
    print("gates: %d sessions, %d PASS; %d terminal flights, %d < 5 m; miss %s" % (
        g["gate_sessions"], g["gate_passed"], g["term_flights"], g["term_under5"], fmt(g["term_miss"], " m")))
    print("entry gates:", S["entry_gates"])
    print("entry gate propellant:", S["entry_gate_prop"])
    print("separation tests:", S["septests"])

    # Figures are drawn by paper/figures/make_all.py from tables/stats.json and the per-flight tables.


if __name__ == "__main__":
    main()
