"""Supervisor for long unattended runs.

usage: watchdog.py [--stall SEC] [--tries N] -- CMD ARGS...

Runs CMD (a kspilot bench script), watching its stdout file for progress.
 - no new output for --stall seconds  -> kill it, retry (up to --tries)
 - kRPC unreachable for 3 minutes     -> restart KSP (tools/restart_ksp.sh), retry
A retry of orbit_recover.py adds --resume when the active vessel is already in orbit.
Exit status 0 when the run printed RESULT PASS.
"""
import os
import subprocess
import sys
import time


def clear_overlay():
    """A killed run cannot clear its HUD (SIGKILL skips atexit): the plugin would keep hiding the stock UI.
    Nor stop its recording: an orphaned recording keeps Time.captureFramerate fixed and the next flight runs in slow
    motion (2026-10-21, ~0.45x game speed). A retry that records requests a new recording itself.
    Recording v3 (DR-011): an OBS recording the killed flight left running is stopped too (kspilot/obsrec.py MARKER;
    OBS only, the sidecars stay for a manual cut with tools/edit_obs.py)."""
    try:
        sys.path.insert(0, ROOT)
        from kspilot import core
        core.overlay("")
        open(os.path.join(core._AL, "rec.stop"), "w").close()
    except Exception:
        pass
    try:
        from kspilot import obsrec
        obsrec.stop_orphan(log=lambda m: print("[watchdog] " + m, flush=True))
    except Exception:
        pass

def _reap(p):
    """Wait for the killed run to be gone (a zombie still answers kill(pid, 0): the OBS orphan check needs it dead)."""
    try:
        p.wait(timeout=15)
    except Exception:
        pass


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
# heartbeat: while it is fresh the AutoLab plugin locks keyboard throttle/pause/quicksave
from kspilot.paths import KSP_DIR  # noqa: E402  ($KSP_DIR or the platform's Steam default)
HB = os.path.join(KSP_DIR, "GameData/AutoLab/lock.hb")
# interpreter for the supervised run: $KSPILOT_PYTHON, else the repository venv, else this interpreter
PY = os.environ.get("KSPILOT_PYTHON") or next(
    (p for p in (os.path.join(ROOT, ".venv", "bin", "python"), os.path.join(ROOT, ".venv", "Scripts", "python.exe"))
     if os.path.exists(p)), sys.executable)


def heartbeat(on=True):
    try:
        if on:
            open(HB, "w").write(str(time.time()))
        elif os.path.exists(HB):
            os.remove(HB)
    except OSError:
        pass


def krpc_alive(timeout=10):
    try:
        import krpc
        c = krpc.connect(name="watchdog", rpc_port=50000, stream_port=50001)
        sit = str(c.space_center.active_vessel.situation) if str(c.krpc.current_game_scene).endswith("flight") else ""
        c.close()
        return True, sit
    except Exception:
        return False, ""


def main():
    args = sys.argv[1:]
    stall, tries = 600, 3
    while args and args[0] != "--":
        if args[0] == "--stall":
            stall = float(args[1]); args = args[2:]
        elif args[0] == "--tries":
            tries = int(args[1]); args = args[2:]
        else:
            break
    cmd = args[1:] if args and args[0] == "--" else args
    out_path = os.path.join(ROOT, "logs", "wd_%s.out" % time.strftime("%Y%m%d_%H%M%S"))
    for attempt in range(1, tries + 1):
        run = list(cmd)
        if attempt > 1 and any("orbit_recover" in c for c in run):
            alive, sit = krpc_alive()
            if sit.endswith("orbiting") and "--resume" not in run:
                run.append("--resume")
        print("[watchdog] attempt %d: %s" % (attempt, " ".join(run)), flush=True)
        start = os.path.getsize(out_path) if os.path.exists(out_path) else 0
        f = open(out_path, "a")
        p = subprocess.Popen([PY] + run, stdout=f, stderr=subprocess.STDOUT, cwd=ROOT)
        last_size, last_change, dead_since = -1, time.time(), None
        heartbeat()
        while p.poll() is None:
            time.sleep(10)
            heartbeat()
            size = os.path.getsize(out_path)
            if size != last_size:
                last_size, last_change = size, time.time()
            alive, _ = krpc_alive()
            dead_since = None if alive else (dead_since or time.time())
            if dead_since and time.time() - dead_since > 180:
                print("[watchdog] kRPC unreachable for 3 min: restarting KSP", flush=True)
                p.kill()
                _reap(p)
                clear_overlay()
                subprocess.run([os.path.join(ROOT, "tools/restart_ksp.sh")])
                break
            if time.time() - last_change > stall:
                print("[watchdog] no progress for %.0fs: killing run" % stall, flush=True)
                p.kill()
                _reap(p)
                clear_overlay()
                break
        f.close()
        heartbeat(False)
        text = open(out_path).read()[start:]
        if "RESULT PASS" in text:
            print("[watchdog] PASS", flush=True)
            return 0
        tail = [l for l in text.splitlines() if "RESULT" in l]
        print("[watchdog] attempt %d ended: %s" % (attempt, tail[-1] if tail else "no result"), flush=True)
        if tail and ("REFUSED" in tail[-1]):
            return 2
    return 1


if __name__ == "__main__":
    sys.exit(main())
