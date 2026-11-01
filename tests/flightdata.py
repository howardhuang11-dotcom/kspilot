"""Recorded flight data for the replay tests.

A small, sanitised subset of real flight logs ships in tests/data (landing CSVs gzip-compressed; see
tests/data/SOURCES.txt). A directory of complete logs can be supplied with KSP_REPLAY_LOGS (default: <repo>/logs);
a file found there takes precedence over the bundled copy. Tests skip with a reason when a file is unavailable.
"""
import gzip
import os
import shutil
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
CACHE = os.path.join(tempfile.gettempdir(), "kspilot-flightdata")


def log_dirs():
    return [d for d in (os.environ.get("KSP_REPLAY_LOGS"), os.path.join(ROOT, "logs")) if d]


def flight_file(name):
    """Path of the flight-data file `name`, or None when neither a log directory nor tests/data provides it."""
    for d in log_dirs():
        p = os.path.join(d, name)
        if os.path.exists(p):
            return p
    p = os.path.join(DATA, name)
    if os.path.exists(p):
        return p
    gz = p + ".gz"
    if os.path.exists(gz):
        out = os.path.join(CACHE, name)
        if not os.path.exists(out) or os.path.getmtime(out) < os.path.getmtime(gz):
            os.makedirs(CACHE, exist_ok=True)
            tmp = out + ".%d.tmp" % os.getpid()
            with gzip.open(gz, "rb") as src, open(tmp, "wb") as dst:
                shutil.copyfileobj(src, dst)
            os.replace(tmp, out)
        return out
    return None
