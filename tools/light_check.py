"""V17 light check on the pad (video requirements v2 V17; kit built by tools/craft_lights.py) - run ONLY when no
mission is flying (the tool refuses if the active vessel is in flight).

  1. installs craft/VAB/<craft>.craft into the AutoPilotLab save's Ships/VAB (as T-WorksFineGF-P45 was; byte-verified;
     a cloud-evicted 'dataless' working copy (macOS file providers) reads empty -> the committed git HEAD version is used),
  2. rolls it out to the LaunchPad (sc.launch_vessel, as orbit_recover.py), warps to night (core.set_daylight,
     sun --sun deg, default -30, before dawn), Light action group ON (vessel.control.lights),
  3. three AutoLab director cameras -> screenshots/light_check_<stamp>_<cam>.png:
       side   ground camera 160 m south of the pad, 12 m up, long lens framing ~45 m (as the D2 pad camera)
       chase  close rel camera 35 m out, 12 deg up, 60 deg round from north
       below  ground camera ~22 m from the pad centre, 1.5 m above the pad surface, wide lens looking up the core
  4. per-lamp state (on/off, EC/s, temperature / max) right after switching on and again after the shots,
  5. cleanup: camera back to the stock chase, lights off, vessel recovered (--keep leaves it on the pad).
RESULT PASS = all <n> spotLight3 lamps of the craft report active, sun below the horizon, three screenshots written.

usage: light_check.py [--craft T-WorksFineGF-LIT] [--sun -30] [--keep] [--no-copy] [--dry-run]"""
import argparse
import math
import os
import re
import subprocess
import sys
import tempfile
import time
import traceback

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from kspilot import atmo, core  # noqa: E402

CRAFT = "T-WorksFineGF-LIT"
SHIPS_VAB = os.path.join(core.KSP_DIR, "saves", "AutoPilotLab", "Ships", "VAB")
LAMP_PART = "spotLight3"
R_KERBIN = 600000.0
PAD_ALT = atmo.KERBIN["ksc_pad"]["surface_height"]          # m ASL
SETTLE_S = 3.0               # after a camera cut: director polls shot.txt at 5 Hz, ground lens eases with tau 0.6 s


class Refused(Exception):
    pass


SF_DATALESS = 0x40000000      # macOS st_flags: file content evicted by a cloud file provider (ls -lO shows "dataless")


def dataless(path):
    try:
        return bool(getattr(os.stat(path), "st_flags", 0) & SF_DATALESS)
    except OSError:
        return False


def craft_bytes(name, root=ROOT):
    """The craft file; when the working copy is cloud-evicted ('dataless': a read blocks) or empty, git HEAD's."""
    rel = "craft/VAB/%s.craft" % name
    data = b""
    try:
        if not dataless(os.path.join(root, rel)):       # reading an evicted file blocks until the provider downloads it
            with open(os.path.join(root, rel), "rb") as f:
                data = f.read()
    except OSError:
        pass
    if not data.startswith(b"ship ="):
        try:
            data = subprocess.run(["git", "-C", root, "show", "HEAD:" + rel], capture_output=True, timeout=60).stdout
        except Exception:  # noqa: BLE001
            data = b""
    if not data.startswith(b"ship ="):
        raise Refused("craft %s unreadable (working copy and git HEAD)" % rel)
    return data


def lamp_count(data):
    return len(re.findall(rb"^\s*part = " + LAMP_PART.encode() + rb"_", data, re.M))


def install(data, name, ships_dir, log):
    """Copy into the save's Ships/VAB (atomic, verified); identical file -> untouched."""
    if not os.path.isdir(ships_dir):
        raise Refused("no Ships/VAB folder at %s" % ships_dir)
    dst = os.path.join(ships_dir, name + ".craft")
    try:
        with open(dst, "rb") as f:
            cur = f.read()
    except OSError:
        cur = None
    if cur == data:
        log("craft: %s already in Ships/VAB (identical, %d bytes)" % (name, len(data)))
        return dst
    with open(dst + ".tmp", "wb") as f:
        f.write(data)
    os.replace(dst + ".tmp", dst)
    with open(dst, "rb") as f:
        if f.read() != data:
            raise Refused("craft copy to %s does not verify" % dst)
    log("craft: %s %s (%d bytes, %d lamps) -> %s" % (name, "updated" if cur is not None else "copied", len(data),
                                                     lamp_count(data), dst))
    return dst


def shot_strings(lat, lon):
    """AutoLab director shot strings (plugins/AutoLab/AutoLab.cs CameraDirector) around the vessel at (lat, lon)."""
    dlat = math.degrees(1.0 / R_KERBIN)                         # deg per metre north
    dlon = dlat / max(math.cos(math.radians(lat)), 1e-6)        # deg per metre east
    return [("side", "ground %.6f %.6f %.1f 20 45 4 cut" % (lat - 160.0 * dlat, lon, PAD_ALT + 12.0)),
            ("chase", "rel 35 12 60 55 cut"),
            ("below", "ground %.6f %.6f %.1f 80 cut" % (lat - 20.0 * dlat, lon - 9.0 * dlon, PAD_ALT + 1.5))]


def _safe(fn, default=None):
    try:
        return fn()
    except Exception:  # noqa: BLE001
        return default


def lamps(v):
    rows = []
    for i, lt in enumerate(_safe(lambda: list(v.parts.lights), [])):
        p = _safe(lambda: lt.part)
        rows.append(dict(i=i, part=_safe(lambda: p.name, "?"), title=_safe(lambda: p.title, "?"),
                         on=_safe(lambda: bool(lt.active)), ec=_safe(lambda: float(lt.power_usage), float("nan")),
                         temp=_safe(lambda: float(p.temperature), float("nan")),
                         tmax=_safe(lambda: float(p.max_temperature), float("nan"))))
    return rows


def log_lamps(rows, log, when):
    n_on = sum(1 for r in rows if r["on"])
    log("lamps %s: %d light part(s), %d on" % (when, len(rows), n_on))
    for r in rows:
        log("  lamp %d %s (%s): %s, %.3f EC/s, %.1f / %.0f K" % (r["i"] + 1, r["part"], r["title"],
                                                               "ON" if r["on"] else "OFF" if r["on"] is False else "?",
                                                               r["ec"], r["temp"], r["tmax"]))
    return n_on


class Env:
    """Side-effect hooks; the real game by default."""

    def __init__(self, conn, log, sleep=None, shot=None, camera=None, daylight=None, ships_dir=SHIPS_VAB):
        self.conn, self.sc, self.log = conn, conn.space_center, log
        self.sleep = sleep or time.sleep
        self.shot = shot or real_shot
        self.camera = camera or core.camera_shot
        self.daylight = daylight or (lambda c, el, log: core.set_daylight(c, el, afternoon=False, log=log))
        self.ships_dir = ships_dir


def real_shot(cam, stamp):
    """core.screenshot (AutoLab, UI hidden), renamed to screenshots/light_check_<stamp>_<cam>.png."""
    p = core.screenshot("light_check_" + cam, timeout=20)
    if p is None:
        return None
    dst = os.path.join(os.path.dirname(p), "light_check_%s_%s.png" % (stamp, cam))
    time.sleep(0.5)
    os.replace(p, dst)
    return dst


def guard(env):
    """No game state change while a mission flies: refuse unless at the space centre / a vessel on the
    ground is active."""
    scene = str(_safe(lambda: env.conn.krpc.current_game_scene, "?"))
    env.log("scene: %s" % scene)
    if not scene.endswith("flight"):
        return
    v = _safe(lambda: env.sc.active_vessel)
    if v is None:
        return
    sit = str(_safe(lambda: v.situation, "?"))
    if not any(k in sit for k in ("pre_launch", "landed", "splashed")):
        raise Refused("active vessel %s is %s: a mission is flying" % (_safe(lambda: v.name, "?"), sit))


def check(env, args):
    log, sc = env.log, env.sc
    data = craft_bytes(args.craft)
    want = lamp_count(data)
    if want == 0:
        raise Refused("craft %s has no %s lamps" % (args.craft, LAMP_PART))
    guard(env)
    if not args.no_copy:
        install(data, args.craft, env.ships_dir, log)
    sc.launch_vessel("VAB", args.craft, "LaunchPad", [], True)
    env.sleep(6.0)
    v = sc.active_vessel                                         # re-acquire after a scene change
    log("on the pad: %s (%s)" % (_safe(lambda: v.name, "?"), _safe(lambda: str(v.situation), "?")))
    shots, fails = {}, []
    try:
        el = env.daylight(env.conn, args.sun, log)
        env.sleep(3.0)
        if not (el is not None and el < 0.0):
            fails.append("sun not below the horizon (%s deg)" % el)
        ec0 = _safe(lambda: v.resources.amount("ElectricCharge"), float("nan"))
        v.control.lights = True
        log("lights ON (control.lights reads %s), EC %.1f" % (_safe(lambda: v.control.lights), ec0))
        env.sleep(1.0)
        rows = lamps(v)
        log_lamps(rows, log, "after switch-on")
        lat = _safe(lambda: float(v.flight().latitude), core.KSC_PAD[0])
        lon = _safe(lambda: float(v.flight().longitude), core.KSC_PAD[1])
        stamp = time.strftime("%Y%m%d_%H%M%S")
        for cam, text in shot_strings(lat, lon):
            env.camera(text)
            env.sleep(SETTLE_S)
            p = env.shot(cam, stamp)
            shots[cam] = p
            log("camera %s: %s -> %s" % (cam, text, p or "NO SCREENSHOT"))
            if p is None:
                fails.append("no %s screenshot" % cam)
        rows2 = lamps(v)
        log_lamps(rows2, log, "after the shots")
        log("EC %.1f -> %.1f" % (ec0, _safe(lambda: v.resources.amount("ElectricCharge"), float("nan"))))
        kit = [r for r in rows2 if r["part"] == LAMP_PART]
        if len(kit) != want:
            fails.append("%d %s parts on the vessel, craft has %d" % (len(kit), LAMP_PART, want))
        off = [r["i"] + 1 for r in kit if not r["on"]]
        if off:
            fails.append("lamp(s) %s not on" % off)
        hot = [r["i"] + 1 for r in kit if r["temp"] > 0.8 * r["tmax"]]
        if hot:
            fails.append("lamp(s) %s above 80 %% of max temperature" % hot)
    finally:
        env.camera("chase")
        _safe(lambda: setattr(v.control, "lights", False))
        if args.keep:
            log("cleanup: camera chase, lights off, vessel kept on the pad (--keep)")
        elif _safe(lambda: bool(v.recoverable), False):
            v.recover()
            env.sleep(3.0)
            log("cleanup: camera chase, lights off, vessel recovered")
        else:
            log("cleanup: WARNING vessel not recoverable, left on the pad")
    log("screenshots: %s" % ", ".join("%s=%s" % kv for kv in shots.items()))
    log("RESULT %s" % ("PASS %d/%d lamps on" % (want, want) if not fails else "FAIL " + "; ".join(fails)))
    return not fails


def parse(argv):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--craft", default=CRAFT)
    ap.add_argument("--sun", type=float, default=-30.0, help="sun elevation over KSC (deg, negative = night)")
    ap.add_argument("--keep", action="store_true", help="leave the vessel on the pad")
    ap.add_argument("--no-copy", action="store_true", help="do not install the craft into Ships/VAB")
    ap.add_argument("--dry-run", action="store_true", help="offline against tests/fake_krpc.PadWorld (no game)")
    return ap.parse_args(argv)


def dry_env(log, world=None):
    sys.path.insert(0, os.path.join(ROOT, "tests"))
    import fake_krpc as fk
    w = world or fk.PadWorld()
    tmp = tempfile.mkdtemp(prefix="light_check_dry_")
    os.makedirs(os.path.join(tmp, "Ships", "VAB"))
    cams = []

    def shot(cam, stamp):
        p = os.path.join(tmp, "light_check_%s_%s.png" % (stamp, cam))
        open(p, "wb").close()
        return p
    env = Env(w.conn, log, sleep=lambda s: None, shot=shot, camera=cams.append, daylight=w.daylight,
              ships_dir=os.path.join(tmp, "Ships", "VAB"))
    env.world, env.cams = w, cams
    return env


def run(args, env=None, log=None):
    if log is None:
        log = core.Log("light_check%s" % ("_dry" if args.dry_run else ""))
    try:
        if env is None:
            if args.dry_run:
                env = dry_env(log)
                log("DRY RUN against tests/fake_krpc.PadWorld (no game; craft installed into %s)" % env.ships_dir)
            else:
                env = Env(core.connect("light_check"), log)
        return check(env, args)
    except Refused as e:
        log("RESULT REFUSED %s" % e)
    except Exception as e:  # noqa: BLE001 - one RESULT line for the watchdog on every path
        log("".join(traceback.format_exception(type(e), e, e.__traceback__)).rstrip())
        log("RESULT FAIL exception %s: %s" % (type(e).__name__, e))
    return None


if __name__ == "__main__":
    run(parse(sys.argv[1:]))
