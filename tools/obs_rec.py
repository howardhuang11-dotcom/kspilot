"""Real-time recording of the KSP game window through OBS Studio (obs-websocket v5).

usage: obs_rec.py start NAME     start recording to videos/obs/NAME_<timestamp>.mp4
       obs_rec.py stop           stop; prints the output file path (last stdout line)
       obs_rec.py status         recording state, OBS stats, capture targets

Never touches the game: no kRPC, no keys, no clicks. It only talks to OBS on localhost:4455.
Connection settings: .obs_ws.json in the repo root (git-ignored; {"host","port","password"}).
Client: obsws-python==1.8.0 (PyPI, aatikturk/obsws-python; pinned here, the repo has no requirements file).

OBS setup it maintains (idempotent, on every start):
  scene "KSP" = "KSP Window" (screen_capture, type=1 window; the KSP window id is re-resolved each start, since it
  changes when KSP restarts) + "KSP Audio" (sck_audio_capture, type=1 application, bundle id below = game sound only).
  Any microphone / desktop-audio input is muted, so only the game's audio reaches the file.
  Canvas/output 1920x1080 @ 30 fps; recording: hybrid MP4, Apple VT H264 hardware, CBR 14 Mbps, AAC 192 kbps
  (the encoder/format live in the OBS profile, set offline 2026-10-21; `status` reports them).

OBS 32.2.2 BUG: GetInputPropertiesListPropertyItems(..., "application") segfaults OBS (both SCK sources;
seen twice 2026-10-21). Never list the "application" property; the bundle id is hard-coded.
"""
import json
import os
import re
import shutil
import struct
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(ROOT, ".obs_ws.json")
REC_DIR = os.path.join(ROOT, "videos", "obs")
SCENE = "KSP"
WIN_INPUT = "KSP Window"
AUD_INPUT = "KSP Audio"
KSP_BUNDLE = "unity.Squad.Kerbal Space Program"
KSP_WINDOW_RE = re.compile(r"^\[(KSP|Kerbal Space Program)\]")  # item names look like "[KSP] KSP"
CANVAS = (1920, 1080)
FPS = 30
START_TIMEOUT = 60
MIN_FREE_GB = 3.0
MIC_KINDS = ("coreaudio_input_capture", "coreaudio_output_capture")
# OBS Studio configuration directory (used for the sentinel files); override with $OBS_CONFIG_DIR
OBS_SUPPORT = os.path.expanduser(os.environ.get("OBS_CONFIG_DIR") or (
    "~/Library/Application Support/obs-studio" if sys.platform == "darwin" else
    os.path.join(os.environ.get("APPDATA", "~"), "obs-studio") if sys.platform.startswith("win") else
    "~/.config/obs-studio"))


class RecError(Exception):
    pass


def log(msg):
    print(f"[obs_rec] {msg}", file=sys.stderr, flush=True)


def _obs_running():
    return subprocess.run(["pgrep", "-x", "OBS"], capture_output=True).returncode == 0


def _launch_obs():
    """Start OBS minimized. Stale crash sentinels are removed first: after a crash OBS otherwise blocks on a
    safe-mode dialog, and safe mode disables the websocket server (a dead end for an unattended run)."""
    sdir = os.path.join(OBS_SUPPORT, ".sentinel")
    if os.path.isdir(sdir):
        for f in os.listdir(sdir):
            if f.startswith("run_"):
                os.remove(os.path.join(sdir, f))
    log("OBS not running; launching it")
    subprocess.run(["open", "-a", "OBS", "--args", "--minimize-to-tray", "--disable-missing-files-check"], check=True)


def connect(tries=20, launch=True):
    import logging
    import obsws_python as obs
    logging.getLogger("obsws_python").setLevel(logging.CRITICAL)  # it logs a traceback per refused attempt
    if not os.path.exists(CFG):
        raise RecError(f"missing {CFG} (obs-websocket host/port/password)")
    cfg = json.load(open(CFG))
    if not _obs_running():
        if not launch:
            raise RecError("OBS is not running")
        _launch_obs()
    last = None
    for _ in range(tries):
        try:
            return obs.ReqClient(host=cfg["host"], port=cfg["port"], password=cfg["password"], timeout=5)
        except Exception as e:  # refused while OBS boots, or auth failure
            last = e
            if "auth" in str(e).lower():
                raise RecError(f"obs-websocket authentication failed: {e}")
            time.sleep(1.5)
    raise RecError(f"cannot reach obs-websocket at {cfg['host']}:{cfg['port']} ({last}); is OBS running with "
                   "Tools > WebSocket Server Settings enabled?")


def find_ksp_window(c):
    """The KSP window's capture id, or None. Lists only the 'window' property (safe)."""
    items = c.get_input_properties_list_property_items(WIN_INPUT, "window").property_items
    for it in items:
        if KSP_WINDOW_RE.match(it.get("itemName", "")) and it.get("itemEnabled", True):
            return it["itemValue"], it["itemName"]
    return None


def _inputs(c):
    return {i["inputName"]: i["inputKind"] for i in c.get_input_list().inputs}


def ensure_setup(c):
    scenes = [s["sceneName"] for s in c.get_scene_list().scenes]
    if SCENE not in scenes:
        c.create_scene(SCENE)
    ins = _inputs(c)
    if WIN_INPUT not in ins:
        c.create_input(SCENE, WIN_INPUT, "screen_capture", {"type": 1, "show_cursor": False, "hide_obs": True}, True)
    if AUD_INPUT not in ins:
        c.create_input(SCENE, AUD_INPUT, "sck_audio_capture", {"type": 1, "application": KSP_BUNDLE}, True)
    c.set_input_settings(AUD_INPUT, {"type": 1, "application": KSP_BUNDLE}, True)
    c.set_input_mute(AUD_INPUT, False)
    # only the game's sound: mute every mic / desktop-audio input wherever it lives
    for name, kind in _inputs(c).items():
        if name == AUD_INPUT:
            continue
        if kind in MIC_KINDS or kind == "sck_audio_capture":
            if not c.get_input_mute(name).input_muted:
                log(f"muting non-game audio input '{name}' ({kind})")
                c.set_input_mute(name, True)

    win = find_ksp_window(c)
    if win is None:
        raise RecError("KSP window not found by OBS (is KSP running and on screen?); refusing to record")
    wid, wname = win
    if c.get_input_settings(WIN_INPUT).input_settings.get("window") != wid:
        c.set_input_settings(WIN_INPUT, {"type": 1, "window": wid, "show_cursor": False, "hide_obs": True}, True)
        time.sleep(0.5)
    c.set_current_program_scene(SCENE)

    vs = c.get_video_settings()
    want = (CANVAS[0], CANVAS[1], CANVAS[0], CANVAS[1], FPS, 1)
    have = (vs.base_width, vs.base_height, vs.output_width, vs.output_height, vs.fps_numerator, vs.fps_denominator)
    if have != want:
        c.set_video_settings(FPS, 1, CANVAS[0], CANVAS[1], CANVAS[0], CANVAS[1])
    os.makedirs(REC_DIR, exist_ok=True)
    if c.get_record_directory().record_directory != REC_DIR:
        c.set_record_directory(REC_DIR)
    _fit_window(c)
    return wid, wname


def _fit_window(c):
    """Fit the window into the canvas; crop the macOS title bar (the window is ~56 px taller than 16:9)."""
    item = next(i for i in c.get_scene_item_list(SCENE).scene_items if i["sourceName"] == WIN_INPUT)
    t = item["sceneItemTransform"]
    w, h = t["sourceWidth"], t["sourceHeight"]
    if not w or not h:
        raise RecError("KSP window capture has zero size (Screen Recording permission? window minimized?)")
    extra = h - round(w * 9 / 16)
    crop_top = extra if 0 < extra <= 100 else 0
    c.set_scene_item_transform(SCENE, item["sceneItemId"], {
        "positionX": 0.0, "positionY": 0.0, "alignment": 5, "cropTop": int(crop_top),
        "cropBottom": 0, "cropLeft": 0, "cropRight": 0,
        "boundsType": "OBS_BOUNDS_SCALE_INNER", "boundsAlignment": 0,
        "boundsWidth": float(CANVAS[0]), "boundsHeight": float(CANVAS[1])})


def _frame_brightness(c):
    """Mean byte value (0-255) of a small BMP screenshot of the window source; ~0 means a black capture."""
    import base64
    data = c.get_source_screenshot(WIN_INPUT, "bmp", 160, 90, -1).image_data
    raw = base64.b64decode(data.split(",", 1)[1])
    off = struct.unpack_from("<I", raw, 10)[0]
    px = raw[off:]
    return sum(px[::7]) / max(1, len(px[::7]))


def status_sample(c):
    """(t_send, t_recv, output_duration s or None, active) of one GetRecordStatus: a video-clock sample for
    kspilot/timemap.calibrate (outputDuration = frames handed to the encoder x frame time)."""
    t0 = time.time()
    st = c.get_record_status()
    t1 = time.time()
    dur = getattr(st, "output_duration", None)
    return t0, t1, (dur / 1000.0 if dur is not None else None), bool(st.output_active)


def start_recording(name, on_sample=None):
    """Start an OBS recording of the KSP window (checks, setup, StartRecord, wait until the output is active).
    Returns (client, info). on_sample(sample) receives every status poll made while waiting (clock samples).
    Raises RecError."""
    safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("_") or "rec"
    c = connect()
    st = c.get_record_status()
    if st.output_active:
        raise RecError(f"already recording ({st.output_timecode}); run `obs_rec.py stop` first")
    free_gb = shutil.disk_usage(REC_DIR if os.path.isdir(REC_DIR) else ROOT).free / 1e9
    if free_gb < MIN_FREE_GB:
        raise RecError(f"only {free_gb:.1f} GB free (14 Mbps = ~6.3 GB/hour); refusing to record")
    wid, wname = ensure_setup(c)
    b = _frame_brightness(c)
    if b < 2:
        raise RecError(f"KSP window capture is black (mean {b:.1f}); refusing to record")
    c.set_profile_parameter("Output", "FilenameFormatting", f"{safe}_%CCYY%MM%DD-%hh%mm%ss")
    t0 = time.time()
    c.start_record()
    # usually <0.5 s; once (2026-10-21, OBS freshly relaunched hidden) the start landed ~30 s late, so wait long
    while time.time() - t0 < START_TIMEOUT:
        time.sleep(0.1)
        smp = status_sample(c)
        if on_sample is not None:
            on_sample(smp[:3])
        if smp[3]:
            log(f"recording '{safe}' after {time.time() - t0:.1f}s window={wname!r} (id {wid}) "
                f"brightness={b:.0f} free={free_gb:.1f}GB dir={REC_DIR}")
            return c, dict(t_req=t0, t_active=smp[1], window=wname, brightness=round(b, 1), free_gb=round(free_gb, 1),
                           prefix=safe)
    raise RecError(f"StartRecord sent but output not active after {START_TIMEOUT}s (see OBS log; it may still "
                   "start late: check `obs_rec.py status`)")


def stop_recording(c):
    """Stop; returns (output file path, info: t_send / t_recv of StopRecord, OBS frame stats before the stop)."""
    if not c.get_record_status().output_active:
        raise RecError("not recording")
    info = {}
    try:
        s = c.get_stats()
        info.update(render_skipped=[s.render_skipped_frames, s.render_total_frames],
                    output_skipped=[s.output_skipped_frames, s.output_total_frames], active_fps=round(s.active_fps, 2))
    except Exception:
        pass
    info["t_send"] = time.time()
    path = c.stop_record().output_path
    info["t_recv"] = time.time()
    for _ in range(40):  # wait for the muxer to close the file
        if not c.get_record_status().output_active:
            break
        time.sleep(0.25)
    return path, info


def cmd_start(name):
    start_recording(name)
    print("RECORDING")
    return 0


def cmd_stop():
    c = connect(launch=False)
    path, _ = stop_recording(c)
    print(path)
    return 0


def cmd_status():
    c = connect(launch=False)
    st = c.get_record_status()
    s = c.get_stats()
    win = find_ksp_window(c) if WIN_INPUT in _inputs(c) else None
    print(json.dumps({
        "recording": st.output_active, "paused": st.output_paused, "timecode": st.output_timecode,
        "record_dir": c.get_record_directory().record_directory,
        "obs_cpu_pct": round(s.cpu_usage, 1), "active_fps": round(s.active_fps, 1),
        "render_skipped": [s.render_skipped_frames, s.render_total_frames],
        "output_skipped": [s.output_skipped_frames, s.output_total_frames],
        "ksp_window": win[1] if win else None,
        "rec_encoder": c.get_profile_parameter("AdvOut", "RecEncoder").parameter_value,
        "rec_format": c.get_profile_parameter("AdvOut", "RecFormat2").parameter_value,
    }, indent=1))
    return 0


def main(argv):
    if not argv or argv[0] not in ("start", "stop", "status") or (argv[0] == "start" and len(argv) != 2):
        print(__doc__.split("\n\n")[1], file=sys.stderr)
        return 2
    try:
        return {"start": lambda: cmd_start(argv[1]), "stop": cmd_stop, "status": cmd_status}[argv[0]]()
    except RecError as e:
        log(f"ERROR: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
