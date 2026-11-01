"""Post-production cut of a real-time OBS recording (recording pipeline v3, decision record DR-011; video
requirements V1-V11; the EDL rules are v2's: director v2 specification sections 2-4A).

usage: edit_obs.py REC_JSON [--name NAME] [--out OUT.mp4] [--plan-only] [--keep-work]

Inputs (videos/obs/<base>.*, written by kspilot/obsrec.py and the flight script):
  .rec.json      OBS file path, start / stop info          .clock.csv    OBS clock samples (video t -> wall)
  .timemap.csv   wall, UT, warp rate, rails, physics        .events.json  HUD events, burns, planned lift-off, ...
Steps:
  1. clocks: VideoClock (kspilot/timemap.calibrate: OBS outputDuration samples) and TimeMap (wall <-> UT)
  2. the SAME EDL as v2: showrun.mission_from_events -> playback_plan (P0-P11) -> rails warp inside a 1x window
     becomes a badged lapse at its MEASURED speed (badge_compressed) -> check_plan (V9 / V1-V6)
  3. every plan segment -> source video spans by wall time ("pieces"): 1x = copied at 1x (real time); physics warp
     inside a 1x window (the entry coast at x2) is slowed to game 1x (frames repeat; V6: no speed-up); time-lapse =
     piecewise speed-up so game time runs uniformly at S (the rails parts are already fast in real time: the video
     speed-up is computed from the time map, never assumed); skip (P7 only, V9) = the span squeezed into 2.5 s
  4. overlays (PIL, edit_video.Overlays; V16: the approved HUD style, one small top-left badge, white, English):
     "▶▶ ×N" with its intro, "▶▶| SKIP 5 H 46 MIN" over the squeezed P7 span (no dimming, no card), "● REAL TIME";
     the P8 deorbit widget (thin white line art + numbers in a HudV2 picture-in-picture frame); no shadow lift
     (the darkness is solved with lights on the rocket) -> PNG runs -> ffconcat overlay track; no drawtext here
  5. ffmpeg: one H.264 pass per segment (exact frame counts), one audio pass, stream-copy concat + mux
Audio (V11): 1x segments keep the game sound (slowed physics-warp pieces through a pitch-safe atempo chain);
time-lapse and skip segments carry a low game-sound BED (default since the 2026-10-21 QA: total silence there sounded
like a dropout): the game sound from the start of the span at -18 dB, sped up no more than x2 (atempo-limited; a
bed, not in sync - the picture runs at x20-x100). KSP_REC_LAPSE_AUDIO=mute (silence) / atempo (whole span through an
atempo chain at -12 dB) keep the older modes. Every cut crossfades the audio over 0.2 s (no clicks; the outgoing
sound runs 0.2 s past the cut, so picture and sound stay in sync). The final mix is loudness-normalised to -16 LUFS
integrated, true peak <= -1.5 dBTP (measured gain + a 4x-oversampled lookahead limiter, delay-compensated; the AAC
result is measured again and corrected up to twice). KSP's own space music (settings.cfg MUSIC_VOLUME) is game sound
and stays (V13). Output: mp4 1080p30 H.264 + AAC 192 kbps, the EDL next to it (<out>.edl.txt).
The OBS master file is never modified or deleted.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from collections import namedtuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kspilot import showrun as sr, timemap as tmap  # noqa: E402
import edit_video as ed  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FPS = sr.FPS
W, H = 1920, 1080
XFADE = 0.2               # s: audio crossfade at every cut
LAPSE_KNOT = 0.5          # s (video): piecewise speed of a time-lapse, one knot every 0.5 s of output
MERGE_TOL = 0.03          # neighbouring lapse pieces whose speed differs < 3 % are merged
PHYS_SLOW_MIN = 1.10      # a physics-warp stretch of a 1x window measured faster than this is slowed to game 1x
PAUSE_RATE = 0.2          # a 1x stretch slower than this (game s / wall s) for >= 1 s is reported (paused game)
KEYONLY_K = 20.0          # a skip span sped up > 20x decodes key frames only
CRF = os.environ.get("KSP_REC_CRF", "20")
LAPSE_AUDIO = os.environ.get("KSP_REC_LAPSE_AUDIO", "bed").strip().lower()   # bed | mute | atempo
LAPSE_GAIN_DB = -12.0     # atempo mode
BED_GAIN_DB = -18.0       # bed mode: the game sound under a time-lapse / skip card, relative to 1x
BED_TEMPO_MAX = 2.0       # bed mode: never sped up more than this (atempo-limited: no chipmunk / granular noise)
LOUD_I = -16.0            # LUFS integrated, final mix (phone playback; QA v5 measured -20.4)
LOUD_TP = -1.5            # dBTP ceiling
LOUD_TOL = 0.5            # LU: accepted |I - LOUD_I|
LOUD_TRIES = 3            # encode + measure passes at most

Piece = namedtuple("Piece", "a b out kind")   # source video s [a, b] -> `out` output s; kind copy|slow|lapse|skip


def k_of(p):
    """Source seconds per output second (> 1: sped up, < 1: slowed)."""
    return (p.b - p.a) / max(p.out, 1e-9)


# ---------------------------------------------------------------------------------------------------------------------
# inputs
# ---------------------------------------------------------------------------------------------------------------------
def probe(path):
    """(duration s, has audio) of a media file."""
    r = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration:stream=codec_type", "-of", "json",
                        path], capture_output=True, text=True)
    j = json.loads(r.stdout or "{}")
    dur = float(j.get("format", {}).get("duration", 0.0))
    return dur, any(s.get("codec_type") == "audio" for s in j.get("streams", []))


def load_clock(path):
    out = []
    with open(path) as f:
        next(f, None)
        for line in f:
            p = line.strip().split(",")
            if len(p) == 3 and p[2]:
                try:
                    out.append((float(p[0]), float(p[1]), float(p[2])))
                except ValueError:
                    pass
    return out


Ctx = namedtuple("Ctx", "meta video vdur audio vc tm evj")


def load(rec_json):
    meta = json.load(open(rec_json))
    base = meta.get("base") or rec_json[:-len(".rec.json")]
    video = meta.get("video")
    if not video or not os.path.exists(video):          # a killed flight: find the OBS file by its prefix
        d = os.path.dirname(base)
        pre = (meta.get("start") or {}).get("prefix") or os.path.basename(base).rsplit("_", 1)[0]
        cands = sorted(f for f in os.listdir(d) if f.startswith(pre) and f.endswith((".mp4", ".mov", ".mkv")))
        if not cands:
            raise SystemExit("edit_obs: no OBS video for %s" % rec_json)
        video = os.path.join(d, cands[-1])
    vdur, audio = probe(video)
    vc = tmap.calibrate(load_clock(meta.get("clock") or base + ".clock.csv"))
    tm = tmap.TimeMap.load(meta.get("timemap") or base + ".timemap.csv")
    evj = json.load(open(meta.get("events") or base + ".events.json"))
    return Ctx(meta, video, vdur, audio, vc, tm, evj)


def ut_to_video(ctx, u):
    return ctx.vc.video(ctx.tm.wall(u))


def video_to_ut(ctx, t):
    return ctx.tm.ut(ctx.vc.wall(t))


# ---------------------------------------------------------------------------------------------------------------------
# EDL
# ---------------------------------------------------------------------------------------------------------------------
def build_plan(ctx):
    """(mission, plan, problems, notes): v2's EDL on the v3 clocks."""
    rec0 = max(video_to_ut(ctx, 0.0), ctx.tm.u[0])
    rec1 = min(video_to_ut(ctx, ctx.vdur), ctx.tm.u[-1])
    m = sr.mission_from_events(ctx.evj, rec0, rec1)
    if ctx.evj.get("start") in ("entry", "term", "mid"):
        m["gap_phase"] = "P9"                            # checkpoint recording: the coast before the entry burn
    plan = sr.playback_plan(m)
    runs, notes = [], []
    for s in plan:                                       # rails warp inside a 1x window: badged at its real speed
        if s.kind != "real":
            continue
        for wa, wb, c, rate in ctx.tm.runs(ctx.tm.wall(s.ut0), ctx.tm.wall(s.ut1)):
            if c == "rails" and rate > PHYS_SLOW_MIN:
                runs.append((ctx.tm.ut(wa), ctx.tm.ut(wb), max(int(round((wb - wa) * FPS)), 1)))
    plan = sr.badge_compressed(plan, runs)
    problems = ["1x window %.1f-%.1f (%.1f s game) flown on rails warp: shown as a BADGED lapse" % (a, b, b - a)
                for a, b, _ in runs]
    ev = {k: tuple(v) for k, v in ctx.evj.get("ev", {}).items()}
    txs = sr.transitions(ev, ctx.evj.get("marks", {}), v2=True)
    problems += sr.check_plan(plan, m, [x for x in txs if x.firm])
    for s in plan:
        if s.kind != "drop" and (s.ut0 < ctx.tm.u[0] - 1.0 or s.ut1 > ctx.tm.u[-1] + 1.0):
            problems.append("%s %s %.1f-%.1f outside the time map (%.1f-%.1f)" % (
                s.phase, s.kind, s.ut0, s.ut1, ctx.tm.u[0], ctx.tm.u[-1]))
    return m, plan, problems, notes


def _merge_lapse(raw, tol=MERGE_TOL):
    out = []
    for p in raw:
        if out and out[-1].kind == p.kind and abs(p.a - out[-1].b) < 1e-6 and \
                abs(k_of(p) - k_of(out[-1])) <= tol * max(k_of(out[-1]), 1e-9):
            q = out[-1]
            out[-1] = Piece(q.a, p.b, q.out + p.out, q.kind)
        else:
            out.append(p)
    return out


def seg_pieces(ctx, s, notes=None):
    """Source pieces of one plan segment (see the module docstring, step 3)."""
    notes = notes if notes is not None else []
    tm, vc = ctx.tm, ctx.vc
    if s.kind == "drop":
        return []
    if s.kind == "skip":
        a, b = ut_to_video(ctx, s.ut0), ut_to_video(ctx, s.ut1)
        return [Piece(a, max(b, a + 1.0 / FPS), s.video, "skip")]
    if s.kind == "lapse":
        n = max(int(round(s.video / LAPSE_KNOT)), 1)
        taus = {s.video * j / n for j in range(n + 1)}
        S = (s.ut1 - s.ut0) / max(s.video, 1e-9)
        for wa, wb, _, _ in tm.runs(tm.wall(s.ut0), tm.wall(s.ut1)):        # a knot wherever the source speed changes
            for w in (wa, wb):
                tau = (tm.ut(w) - s.ut0) / S
                if 1.0 / FPS < tau < s.video - 1.0 / FPS:
                    taus.add(tau)
        taus = sorted(taus)
        n = len(taus) - 1
        vs = [ut_to_video(ctx, s.ut0 + (s.ut1 - s.ut0) * t / s.video) for t in taus]
        raw = [Piece(vs[j], max(vs[j + 1], vs[j]), taus[j + 1] - taus[j], "lapse") for j in range(n)]
        return _merge_lapse(raw)
    out = []                                             # 1x
    for wa, wb, c, rate in tm.runs(tm.wall(s.ut0), tm.wall(s.ut1)):
        a, b = vc.video(wa), vc.video(wb)
        if b - a < 0.5 / FPS:
            continue
        if c == "phys" and rate > PHYS_SLOW_MIN:
            g = tm.ut(wb) - tm.ut(wa)
            out.append(Piece(a, b, g, "slow"))
            notes.append("%s physics warp %.1f-%.1f: game x%.2f in real time, slowed to game 1x (%.1f s -> %.1f s)" % (
                s.phase, tm.ut(wa), tm.ut(wb), rate, b - a, g))
        else:
            if rate < PAUSE_RATE and wb - wa >= 1.0:
                notes.append("%s game paused / stalled %.1f s at UT %.1f (shown as it happened)" % (
                    s.phase, wb - wa, tm.ut(wa)))
            if out and out[-1].kind == "copy" and abs(out[-1].b - a) < 1e-6:
                out[-1] = Piece(out[-1].a, b, out[-1].out + (b - a), "copy")
            else:
                out.append(Piece(a, b, b - a, "copy"))
    return out


def frame_counts(durs, fps=FPS):
    """Exact frame count per segment from the cumulative output time (no drift over the video)."""
    out, t, f0 = [], 0.0, 0
    for d in durs:
        t += d
        f1 = int(round(t * fps))
        out.append(f1 - f0)
        f0 = f1
    return out


def build(ctx):
    """(m, plan, segs, problems, notes); segs = [(plan index, pieces, frames)] for every segment in the video."""
    m, plan, problems, notes = build_plan(ctx)
    segs = []
    for i, s in enumerate(plan):
        p = seg_pieces(ctx, s, notes)
        if p:
            segs.append([i, p, 0])
    for sg, n in zip(segs, frame_counts([sum(p.out for p in sg[1]) for sg in segs])):
        sg[2] = n
    segs = [tuple(sg) for sg in segs if sg[2] > 0]
    return m, plan, segs, problems, notes


# ---------------------------------------------------------------------------------------------------------------------
# ffmpeg command builders (pure: tests/test_rec_v3.py)
# ---------------------------------------------------------------------------------------------------------------------
def atempo_chain(k):
    """atempo factors (each in [0.5, 2], the filter's clean range) whose product is k."""
    out = []
    while k > 2.0 + 1e-9:
        out.append(2.0)
        k /= 2.0
    while k < 0.5 - 1e-9:
        out.append(0.5)
        k /= 0.5
    if abs(k - 1.0) > 1e-6:
        out.append(k)
    return out


def bed_tempo(pieces, dur):
    """Speed of a lapse / skip segment's sound bed: the segment's own source-per-output speed, capped at BED_TEMPO_MAX."""
    k = sum(p.b - p.a for p in pieces) / max(dur, 1e-9)
    return min(k, BED_TEMPO_MAX)


def _f(x):
    return "%.6f" % x


def input_window(pieces, pre=0.0, post=1.0, vdur=None):
    """(-ss, -t) of the input read for a segment."""
    a = max(min(p.a for p in pieces) - pre, 0.0)
    b = max(p.b for p in pieces) + post
    if vdur is not None:
        b = min(b, vdur + post)
    return a, b - a


def video_graph(pieces, nframes, ss, overlay=False, fps=FPS, size=(W, H)):
    """filter_complex for one segment: trim + setpts per piece (time relative to the -ss input), concat, CFR, 1080p,
    [overlay track], exact frame count (tpad clone + -frames:v)."""
    n = len(pieces)
    g = []
    src = "[0:v]"
    if n > 1:
        g.append("[0:v]split=%d%s" % (n, "".join("[s%d]" % i for i in range(n))))
    for i, p in enumerate(pieces):
        inp = "[s%d]" % i if n > 1 else src
        g.append("%strim=start=%s:end=%s,setpts=(PTS-STARTPTS)/%s[p%d]" % (
            inp, _f(p.a - ss), _f(p.b - ss), _f(k_of(p)), i))
    if n > 1:
        g.append("%sconcat=n=%d:v=1:a=0[cat]" % ("".join("[p%d]" % i for i in range(n)), n))
        cur = "[cat]"
    else:
        cur = "[p0]"
    w, h = size
    chain = ("fps=%d,scale=%d:%d:force_original_aspect_ratio=decrease,pad=%d:%d:(ow-iw)/2:(oh-ih)/2,setsar=1" % (
        fps, w, h, w, h))
    g.append("%s%s[base]" % (cur, chain))
    cur = "[base]"
    if overlay:
        g.append("[1:v]fps=%d,format=rgba[ov]" % fps)
        g.append("%s[ov]overlay=0:0:eof_action=pass:format=auto[ovd]" % cur)
        cur = "[ovd]"
    g.append("%sformat=yuv420p,tpad=stop_mode=clone:stop_duration=%s[v]" % (cur, _f(2.0 + 2.0 / fps)))
    return ";".join(g)


def video_cmd(src, pieces, nframes, out, overlay_list=None, vdur=None, fps=FPS):
    ss, t = input_window(pieces, vdur=vdur)
    pre = []
    if len(pieces) == 1 and pieces[0].kind == "skip" and k_of(pieces[0]) > KEYONLY_K:
        pre = ["-skip_frame", "nokey"]                   # skip background: key frames are enough
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-nostdin"] + pre + ["-ss", _f(ss), "-t", _f(t), "-i", src]
    if overlay_list:
        cmd += ["-f", "concat", "-safe", "0", "-i", overlay_list]
    cmd += ["-filter_complex", video_graph(pieces, nframes, ss, bool(overlay_list), fps), "-map", "[v]",
            "-frames:v", str(nframes), "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", CRF,
            "-pix_fmt", "yuv420p", "-r", str(fps), "-video_track_timescale", "15360", out]
    return cmd


def audio_plan(plan, segs, fps=FPS, lapse_audio=LAPSE_AUDIO):
    """[(mode, pieces, duration, extension)] per video segment: mode keep (1x) | bed (lapse / skip, default) | mute |
    tempo (lapse with KSP_REC_LAPSE_AUDIO=atempo); duration = the segment's exact video length, extension = XFADE
    except the last."""
    out = []
    for j, (i, pieces, n) in enumerate(segs):
        kind = plan[i].kind
        if kind == "real":
            mode = "keep"
        elif lapse_audio == "bed":
            mode = "bed"
        else:
            mode = "tempo" if kind == "lapse" and lapse_audio == "atempo" else "mute"
        out.append((mode, pieces, n / float(fps), XFADE if j < len(segs) - 1 else 0.0))
    return out


def audio_graph(aplan, inputs):
    """filter_complex for the audio pass. inputs[j] = (input index, ss) of segment j's source input (None: muted).
    Each segment is built to duration + extension, then the segments are chained with acrossfade (d = XFADE): the
    total equals the video length and every segment starts exactly at its cut."""
    g, labels = [], []
    fmt = "aformat=sample_fmts=fltp:sample_rates=48000:channel_layouts=stereo"
    for j, (mode, pieces, dur, ext) in enumerate(aplan):
        total = dur + ext
        lab = "[a%d]" % j
        if mode == "mute" or inputs[j] is None:
            g.append("aevalsrc=0|0:c=stereo:s=48000:d=%s,%s%s" % (_f(total), fmt, lab))
        elif mode == "bed":
            # low game-sound bed: one contiguous read from the span's start at tempo <= BED_TEMPO_MAX (<= the
            # segment's own speed-up, so the read never runs past the span's source + its run-on)
            idx, ss = inputs[j]
            tempo = bed_tempo(pieces, dur)
            a = pieces[0].a
            f = ["atrim=start=%s:end=%s" % (_f(max(a - ss, 0.0)), _f(a + total * tempo - ss)), "asetpts=PTS-STARTPTS"]
            f += ["atempo=%s" % _f(x) for x in atempo_chain(tempo)]
            f.append("volume=%.1fdB" % BED_GAIN_DB)
            g.append("[%d:a]%s,%s,apad=whole_dur=%s,atrim=end=%s%s" % (idx, ",".join(f), fmt, _f(total), _f(total), lab))
        else:
            idx, ss = inputs[j]
            n = len(pieces)
            src = "[%d:a]" % idx
            if n > 1:
                g.append("%sasplit=%d%s" % (src, n, "".join("[a%ds%d]" % (j, i) for i in range(n))))
            parts = []
            for i, p in enumerate(pieces):
                k = k_of(p)
                b = p.b + (ext * k if i == n - 1 else 0.0)      # the outgoing sound runs on past the cut
                f = ["atrim=start=%s:end=%s" % (_f(max(p.a - ss, 0.0)), _f(b - ss)), "asetpts=PTS-STARTPTS"]
                f += ["atempo=%s" % _f(x) for x in atempo_chain(k)]
                if mode == "tempo":
                    f.append("volume=%.1fdB" % LAPSE_GAIN_DB)
                g.append("%s%s,%s[a%dp%d]" % ("[a%ds%d]" % (j, i) if n > 1 else src, ",".join(f), fmt, j, i))
                parts.append("[a%dp%d]" % (j, i))
            cat = "".join(parts) + ("concat=n=%d:v=0:a=1," % n if n > 1 else "")
            g.append("%sapad=whole_dur=%s,atrim=end=%s%s" % (cat, _f(total), _f(total), lab))
        labels.append(lab)
    cur = labels[0]
    for j in range(1, len(labels)):
        nxt = "[x%d]" % j
        g.append("%s%sacrossfade=d=%s:c1=tri:c2=tri%s" % (cur, labels[j], _f(XFADE), nxt))
        cur = nxt
    g.append("%sanull[aout]" % cur)
    return ";".join(g)


def audio_cmd(src, aplan, out, has_audio=True):
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-nostdin"]
    inputs, k = [], 0
    for mode, pieces, dur, ext in aplan:
        if mode == "mute" or not has_audio:
            inputs.append(None)
            continue
        kk = max(k_of(p) for p in pieces)
        ss, t = input_window(pieces, post=1.0 + ext * kk)
        cmd += ["-vn", "-ss", _f(ss), "-t", _f(t), "-i", src]
        inputs.append((k, ss))
        k += 1
    if k == 0:                                             # aevalsrc needs no input; give ffmpeg a null one
        cmd += ["-f", "lavfi", "-i", "anullsrc=r=48000:cl=stereo"]
    codec = ["-c:a", "pcm_f32le"] if out.endswith(".wav") else ["-c:a", "aac", "-b:a", "192k"]   # .wav: the pre-loudness mix
    cmd += ["-filter_complex", audio_graph(aplan, inputs), "-map", "[aout]"] + codec + [out]
    return cmd


# ---------------------------------------------------------------------------------------------------------------------
# loudness (next video round: QA v5 measured -20.4 LUFS integrated, +0.2 dBTP - quiet on phones)
# ---------------------------------------------------------------------------------------------------------------------
def measure_loudness(path):
    """EBU R128 of a media file's audio: dict(I LUFS integrated, TP dBTP true peak, LRA LU, thresh) (ffmpeg loudnorm
    analysis pass, print_format=json; I = -inf for digital silence)."""
    r = subprocess.run(["ffmpeg", "-hide_banner", "-nostdin", "-i", path, "-vn", "-af",
                        "loudnorm=I=%.1f:TP=%.1f:LRA=11:print_format=json" % (LOUD_I, LOUD_TP), "-f", "null", "-"],
                       capture_output=True, text=True)
    txt = r.stderr
    try:
        j = json.loads(txt[txt.rindex("{"):txt.rindex("}") + 1])
        return dict(I=float(j["input_i"]), TP=float(j["input_tp"]), LRA=float(j["input_lra"]),
                    thresh=float(j["input_thresh"]))
    except (ValueError, KeyError) as e:
        raise RuntimeError("loudness measurement of %s failed (%r): %s" % (path, e, txt[-800:]))


def loud_chain(gain_db, limit_db):
    """Gain, then a true-peak-safe limiter: 4x oversampled (192 kHz = the BS.1770 true-peak rate), lookahead,
    delay-compensated (latency=1: A/V sync unchanged), no auto level; back to 48 kHz."""
    return ("volume=%.2fdB,aresample=192000,alimiter=limit=%.5f:attack=1:release=60:level=0:latency=1,"
            "aresample=48000" % (gain_db, min(max(10.0 ** (limit_db / 20.0), 0.0625), 1.0)))


def loud_cmd(src, out, gain_db, limit_db):
    return ["ffmpeg", "-y", "-loglevel", "error", "-nostdin", "-i", src, "-vn", "-af", loud_chain(gain_db, limit_db),
            "-c:a", "aac", "-b:a", "192k", out]


def loudness_normalise(src, out, target_i=LOUD_I, target_tp=LOUD_TP, tol=LOUD_TOL, tries=LOUD_TRIES, log=print):
    """Final mix -> out (AAC) at target_i LUFS integrated, true peak <= target_tp dBTP: measured gain + limiter
    (ceiling target_tp - 0.5 dB), then the AAC itself is measured and the gain / ceiling corrected (the encoder's
    overshoot), at most `tries` encodes. Digital silence: plain encode. Returns an info dict (also the EDL line)."""
    m = measure_loudness(src)
    if not m["I"] > -70.0:
        run(["ffmpeg", "-y", "-loglevel", "error", "-nostdin", "-i", src, "-vn", "-c:a", "aac", "-b:a", "192k", out],
            "audio encode")
        return dict(src=m, out=m, gain=0.0, limit=None, tries=1, ok=False, silent=True)
    gain, lim = target_i - m["I"], target_tp - 0.5
    r = None
    for k in range(tries):
        run(loud_cmd(src, out, gain, lim), "loudness pass %d" % (k + 1))
        r = measure_loudness(out)
        ok = abs(r["I"] - target_i) <= tol and r["TP"] <= target_tp
        log("edit_obs: loudness pass %d: gain %+.2f dB, ceiling %.2f dBFS -> %.2f LUFS, %.2f dBTP%s" % (
            k + 1, gain, lim, r["I"], r["TP"], " OK" if ok else ""))
        if ok:
            return dict(src=m, out=r, gain=gain, limit=lim, tries=k + 1, ok=True, silent=False)
        gain += target_i - r["I"]
        if r["TP"] > target_tp:
            lim -= r["TP"] - target_tp + 0.2
    return dict(src=m, out=r, gain=gain, limit=lim, tries=tries, ok=False, silent=False)


def loudness_text(info):
    s, o = info["src"], info["out"]
    return "# loudness: mix %.1f LUFS / %.1f dBTP (LRA %.1f) -> %.1f LUFS / %.1f dBTP (gain %+.1f dB, %d pass(es)) %s\n" % (
        s["I"], s["TP"], s["LRA"], o["I"], o["TP"], info["gain"], info["tries"],
        "PASS" if info["ok"] else ("silent: not normalised" if info.get("silent") else "OUT OF TARGET"))


def mux_cmd(list_path, audio, out):
    return ["ffmpeg", "-y", "-loglevel", "error", "-nostdin", "-f", "concat", "-safe", "0", "-i", list_path, "-i", audio,
            "-map", "0:v:0", "-map", "1:a:0", "-c", "copy", "-movflags", "+faststart", out]


# ---------------------------------------------------------------------------------------------------------------------
# overlays
# ---------------------------------------------------------------------------------------------------------------------
def piece_at(pieces, t):
    """Source video time shown at output time t of a segment."""
    acc = 0.0
    for p in pieces:
        if t < acc + p.out or p is pieces[-1]:
            return p.a + (p.b - p.a) * min(max((t - acc) / max(p.out, 1e-9), 0.0), 1.0)
        acc += p.out
    return pieces[-1].b


def overlay_keys(ctx, plan, i, pieces, n, deo=None, fps=FPS):
    """Per output frame of segment i: None or (badge state or None, deorbit-widget game time or None)."""
    keys = []
    p8 = deo is not None and plan[i].kind == "real" and "P8" in plan[i].phase.split("+")
    for k in range(n):
        st = ed.overlay_state(plan, i, k, fps)
        if st is not None:
            st = (st[0], st[1], round(st[2], 3), round(st[3], 2), st[4])
        u = None
        if p8:
            u = round(video_to_ut(ctx, piece_at(pieces, k / float(fps))), 1)
            if not deo["ut0"] - 1.0 <= u <= deo["ut1"] + 1.0:
                u = None
        keys.append(None if st is None and u is None else (st, u))
    return keys


def runs_of(keys):
    out = []
    for k in keys:
        if out and out[-1][0] == k:
            out[-1][1] += 1
        else:
            out.append([k, 1])
    return out


def render_key(ov, key, coast, deo):
    """One overlay PNG: transparent; the badge and/or the deorbit widget (V16: no dimming, no card, no panel)."""
    from PIL import Image
    im = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    if key is None:
        return im
    st, u = key
    if u is not None:
        al = ed.deorbit_alpha(deo, u)
        if al > 0.0:
            im.alpha_composite(ov.deorbit_widget(deo, u, al), (int(ed.DEO_BOX[0] * ov.k), int(ed.DEO_BOX[1] * ov.k)))
    if st is not None:
        kind, nn, sc, al, is_skip = st
        b = ov.badge(kind, nn, sc, al, coast if is_skip else None)
        im.alpha_composite(b, (int(64 * ov.k), int(120 * ov.k)))
    return im


def write_overlay_track(runs, work, tag, ov, coast, deo, cache, fps=FPS):
    """PNG per distinct key (cached across segments) + an ffconcat list with frame-exact durations."""
    lines = ["ffconcat version 1.0"]
    last = None
    for key, cnt in runs:
        if key not in cache:
            path = os.path.join(work, "ov_%04d.png" % len(cache))
            render_key(ov, key, coast, deo).save(path, compress_level=1)
            cache[key] = path
        last = cache[key]
        lines += ["file '%s'" % last, "duration %s" % _f(cnt / float(fps))]
    lines.append("file '%s'" % last)                     # the concat demuxer needs the last file twice
    path = os.path.join(work, "%s.ffconcat" % tag)
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return path


# ---------------------------------------------------------------------------------------------------------------------
def edl_text(ctx, plan, segs, problems, notes, m):
    vi = ctx.vc.info
    lines = ["# recording pipeline v3 EDL (tools/edit_obs.py, OBS real-time capture, DR-011)",
             "# video %s (%.1f s, audio %s)" % (ctx.video, ctx.vdur, "yes" if ctx.audio else "NO"),
             "# video clock: %s, %d samples, rtt %.1f ms, W0 %.3f, slope %.6f, max residual %.1f ms, lag %.3f s" % (
                 vi.get("mode"), vi.get("n", 0), vi.get("rtt_ms", 0.0), vi.get("w0", 0.0), vi.get("slope", 1.0),
                 vi.get("resid_max_ms", 0.0), vi.get("lag_s", 0.0))]
    st = ctx.meta.get("stop") or {}
    if st.get("t_send"):
        lines.append("# stop check: StopRecord sent %.3f s after the last frame by the clock (video %.2f s); OBS "
                     "skipped render %s output %s" % (st["t_send"] - ctx.vc.wall(ctx.vdur), ctx.vdur,
                                                      st.get("render_skipped"), st.get("output_skipped")))
    lines.append("# time map: %d rows, UT %.1f-%.1f, wall %.1f s" % (
        len(ctx.tm.rows), ctx.tm.u[0], ctx.tm.u[-1], ctx.tm.w[-1] - ctx.tm.w[0]))
    lines.append("phase kind ut0 ut1 game_s speed badge plan_video_s | src_v0 src_v1 frames out_s video_k pieces")
    by = {i: (p, n) for i, p, n in segs}
    tot = real = 0.0
    for i, s in enumerate(plan):
        base = "%-8s %-5s %.2f %.2f %8.1f %8s %6s %7.2f" % (
            s.phase, s.kind, s.ut0, s.ut1, s.ut1 - s.ut0, "-" if s.speed is None else "%.1f" % s.speed,
            "x%d" % sr.tier(s.speed) if s.kind == "lapse" else ("SKIP" if s.kind == "skip" else "-"), s.video)
        if i in by:
            p, n = by[i]
            out = n / float(FPS)
            tot += out
            real += out if s.kind == "real" else 0.0
            ks = sorted({round(k_of(x), 2) for x in p})
            base += " | %9.2f %9.2f %6d %7.2f %s %d%s" % (
                p[0].a, p[-1].b, n, out, ("%.2f" % ks[0]) if len(ks) == 1 else "%.2f-%.2f" % (ks[0], ks[-1]), len(p),
                " slow" if any(x.kind == "slow" for x in p) else "")
        lines.append(base)
    lines.append("# total %.1f s = %d:%04.1f (frames %d), 1x %.1f s (%.0f %%); audio: 1x kept, lapse/skip %s, "
                 "%.1f s crossfade at cuts" % (tot, int(tot) // 60, tot % 60, sum(n for _, _, n in segs), real,
                                                100 * real / max(tot, 1e-9), LAPSE_AUDIO, XFADE))
    lines += ["# note: " + x for x in notes]
    lines.append("# checks: " + ("PASS" if not problems else "FAILED, %d problem(s)" % len(problems)))
    lines += ["#   " + x for x in problems[:50]]
    return "\n".join(lines) + "\n"


def run(cmd, what):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError("%s failed (rc %d): %s" % (what, r.returncode, r.stderr[-2000:]))


def render(ctx, plan, segs, m, out, deo=None, work=None, log=print):
    """Encode the cut. Returns the output path."""
    work = work or tempfile.mkdtemp(prefix="edit_obs_", dir=os.path.dirname(out))
    os.makedirs(work, exist_ok=True)
    ov = coast = None
    cache = {}
    names = []
    t0 = time.time()
    for j, (i, pieces, n) in enumerate(segs):
        keys = overlay_keys(ctx, plan, i, pieces, n, deo)
        lst = None
        if any(k is not None for k in keys):
            if ov is None:
                ov = ed.Overlays(W, H)
                coast = ed.card_text(plan, m)
            lst = write_overlay_track(runs_of(keys), work, "seg_%03d" % j, ov, coast, deo, cache)
        path = os.path.join(work, "seg_%03d.mp4" % j)
        run(video_cmd(ctx.video, pieces, n, path, lst, ctx.vdur), "segment %d (%s %s)" % (
            j, plan[i].phase, plan[i].kind))
        names.append(path)
        log("edit_obs: segment %d/%d %s %s %d frames (%.0f s)" % (j + 1, len(segs), plan[i].phase, plan[i].kind, n,
                                                                  time.time() - t0))
    mix = os.path.join(work, "audio_mix.wav")
    run(audio_cmd(ctx.video, audio_plan(plan, segs), mix, ctx.audio), "audio")
    aud = os.path.join(work, "audio.m4a")
    loud = loudness_normalise(mix, aud, log=log)
    lp = os.path.join(work, "segments.ffconcat")
    with open(lp, "w") as f:
        f.write("ffconcat version 1.0\n" + "".join("file '%s'\n" % p for p in names))
    run(mux_cmd(lp, aud, out), "mux")
    return out, work, loud


def main(argv):
    if not argv or argv[0].startswith("-"):
        print(__doc__)
        return 2
    ctx = load(argv[0])
    name = argv[argv.index("--name") + 1] if "--name" in argv else ctx.meta.get("name", "rec")
    os.makedirs(os.path.join(ROOT, "videos"), exist_ok=True)
    out = argv[argv.index("--out") + 1] if "--out" in argv else \
        os.path.join(ROOT, "videos", time.strftime("%Y%m%d_%H%M%S_") + name + ".mp4")
    m, plan, segs, problems, notes = build(ctx)
    edl = edl_text(ctx, plan, segs, problems, notes, m)
    deo = ed.deorbit_info(plan, m, ctx.evj)
    if deo is not None:
        edl += "# deorbit widget %.1f-%.1f (HUD style, no shadow lift)\n" % (deo["ut0"], deo["ut1"])
    edl_path = os.path.splitext(out)[0] + ".edl.txt"
    with open(edl_path, "w") as f:
        f.write(edl)
    print(edl, end="", flush=True)
    ed.loud_failure(problems, edl_path)
    if "--plan-only" in argv:
        return 0
    try:
        path, work, loud = render(ctx, plan, segs, m, out, deo)
    except RuntimeError as e:
        print("edit_obs: %s" % e, flush=True)
        print("video: None", flush=True)
        return 1
    line = loudness_text(loud)
    print(line, end="", flush=True)
    with open(edl_path, "a") as f:
        f.write(line)
    want = sum(n for _, _, n in segs) / float(FPS)
    got, aud = probe(path)
    print("video: %s (%.2f s, expected %.2f s, audio %s)" % (path, got, want, "yes" if aud else "NO"), flush=True)
    ed.loud_failure(problems, edl_path)
    if "--keep-work" not in argv:
        shutil.rmtree(work, ignore_errors=True)
    return 0 if abs(got - want) < 0.2 else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
