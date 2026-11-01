"""Post-production cut of a whole-mission recording (recording pipeline v2, HOP_REC=full; director v2
specification sections 2-4A, video requirements V1-V9).

usage: edit_video.py FRAMES_DIR [--name NAME] [--out OUT.mp4] [--plan-only] [--keep]

Inputs (written during the flight):
  FRAMES_DIR/index.csv    one line per recorded frame: frame, UT, ... (AutoLab Recorder v2)
  FRAMES_DIR/events.json  HUD events, director burns, planned lift-off, ... (tools/orbit_recover.py)
Steps: kspilot.showrun.playback_plan() (P0-P11: 1x windows whole, time-lapses P3/P5/P9, P7 bridge + skip badge) ->
fallback (fix round 2): frames of a 1x window captured on rails (index.csv mode 'rails', warp > 1) or too far apart
to play at 1x become a BADGED time-lapse (showrun.compressed_runs / badge_compressed), never an unbadged speed-up ->
select_frames() (every recorded frame of a 1x window, UT-sampled frames of a lapse) -> V9 / V1-V6 checks ->
PIL overlays (V16, the approved HUD style: one small badge "▶▶ ×N", "▶▶| SKIP 5 H 46 MIN", "● REAL TIME", no card,
no dimming; the P8 deorbit widget as thin white line art in a HUD picture-in-picture frame, no shadow lift)
-> ffmpeg (JPEG pipe -> H.264).
The EDL (plan, video seconds per phase, check results) is written next to the video (<video>.edl.txt). Any check
problem prints a loud 'EDL CHECKS FAILED' line (before and after the encode); the video is still produced.
The frames are deleted after a successful encode unless --keep or KSP_REC_KEEP=1 (they allow a re-cut without
flying again, but take ~10 GB).
"""
import csv
import io
import json
import os
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot import showrun as sr  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIN = "/System/Library/Fonts/Supplemental/DIN Alternate Bold.ttf"


def python_with_pil():
    """An interpreter that can run this editor: it needs Pillow (PIL), not kRPC. This interpreter if it has Pillow,
    otherwise the first common system python3 that has it; returns None if none is found."""
    try:
        import PIL  # noqa: F401
        return sys.executable
    except ImportError:
        pass
    for py in ("/usr/bin/python3", shutil.which("python3"), "/opt/homebrew/bin/python3", "/usr/local/bin/python3"):
        if py and os.path.exists(py) and subprocess.run([py, "-c", "import PIL"], capture_output=True).returncode == 0:
            return py
    return None


def load_index(frames):
    """[(frame number, UT, on rails)] in frame order (the plugin writes frames in order; UT never decreases).
    on rails: index.csv mode 'rails' at a warp rate > 1 (KSP keeps WarpMode HIGH at rate 1: that is not rails)."""
    out = []
    with open(os.path.join(frames, "index.csv")) as f:
        for r in csv.DictReader(f):
            try:
                fr, ut = int(r["frame"]), float(r["ut"])
            except (KeyError, ValueError, TypeError):
                continue
            try:
                rails = (r.get("mode") or "").strip() == "rails" and float(r.get("warp") or 1.0) > 1.01
            except ValueError:
                rails = False
            out.append((fr, ut, rails))
    out.sort()
    return out


FAIL_BANNER = "!" * 78


def loud_failure(problems, edl_path):
    """The run must not look clean when the EDL checks failed (QA 2026-10-21: 29 problems, video shipped quietly)."""
    if problems:
        print(FAIL_BANNER, flush=True)
        print("EDL CHECKS FAILED: %d problem(s) - the video is still produced; see %s" % (len(problems), edl_path),
              flush=True)
        print(FAIL_BANNER, flush=True)


# ---------------------------------------------------------------------------------------------------------------------
# overlays (PIL, 1080p design units scaled to the source frame)
# ---------------------------------------------------------------------------------------------------------------------
class Overlays:
    """V16 (requirement 2026-10-21): every post-production graphic follows the approved HUD style
    (plugins/AutoLab/HudV2.cs): DIN caps, small labels 12 px (letter-spaced like HudV2.Sp: one space between letters)
    at white 0.8, values 14 px at white 0.92 (HudV2 MID), a 1 px soft shadow (black 0.5, HudV2.L), no panels, no dimming,
    no accent colour, no big text, English only. The single element is the top-left screen-state badge."""
    LABEL_PX, VALUE_PX = 12, 14           # HudV2 sRowL / sNextV
    LABEL_A, VALUE_A = 0.8, 0.92          # HudV2 sRowL alpha / MID

    def __init__(self, w, h):
        from PIL import ImageFont
        self.w, self.h, self.k = w, h, h / 1080.0
        self._fonts = {}
        self._cache = {}
        self.ImageFont = ImageFont

    def font(self, px):
        key = int(round(px * self.k))
        if key not in self._fonts:
            try:
                self._fonts[key] = self.ImageFont.truetype(DIN, max(key, 6))
            except OSError:
                self._fonts[key] = self.ImageFont.load_default()
        return self._fonts[key]

    @staticmethod
    def spaced(s):
        """HudV2.Sp: letter-spaced caps, 'TIME' -> 'T I M E' (a space becomes three)."""
        return " ".join(s.upper())

    def _text(self, d, xy, txt, font, alpha):
        """White text with the HUD's 1 px soft shadow; returns the x after it."""
        x, y = xy
        d.text((x + self.k, y + self.k), txt, font=font, fill=(0, 0, 0, int(128 * alpha)))
        d.text((x, y), txt, font=font, fill=(255, 255, 255, int(255 * alpha)))
        return x + d.textlength(txt, font=font)

    def _tri(self, d, x, y, s, alpha, bar=False):
        """▶▶ (or ▶▶|) as polygons, 10 design px tall (DIN has no such glyph), white, 1 px shadow."""
        k = self.k * s
        for off, fill in ((self.k, (0, 0, 0, int(128 * alpha))), (0.0, (255, 255, 255, int(255 * alpha)))):
            for i in range(2):
                x0 = x + i * 7 * k + off
                d.polygon([(x0, y + off), (x0 + 7 * k, y + 5 * k + off), (x0, y + 10 * k + off)], fill=fill)
            if bar:
                d.rectangle([x + 15 * k + off, y + off, x + 16.5 * k + off, y + 10 * k + off], fill=fill)
        return x + (17 if bar else 14) * k

    def badge(self, kind, n=None, scale=1.0, alpha=1.0, sub=None):
        """Top-left screen-state badge at (64, 120) design px, one line: 'lapse' '▶▶ ×N', 'skip' '▶▶| SKIP  <sub>'
        (sub: the coast duration, '5 H 46 MIN'), 'real' '● REAL TIME'. scale: the lapse intro (2x -> 1x), alpha: fade."""
        key = ("badge", kind, n, round(scale, 3), round(alpha, 2), sub)
        if key in self._cache:
            return self._cache[key]
        from PIL import Image, ImageDraw
        k = self.k * scale
        im = Image.new("RGBA", (int(420 * k) + 4, int(24 * k) + 4), (0, 0, 0, 0))
        d = ImageDraw.Draw(im)
        fv, fl = self.font(self.VALUE_PX * scale), self.font(self.LABEL_PX * scale)
        va, la = self.VALUE_A * alpha, self.LABEL_A * alpha
        x, y = 2.0, 2.0
        if kind == "real":
            d.ellipse([x + self.k, y + 7 * k + self.k, x + 6 * k + self.k, y + 13 * k + self.k],
                      fill=(0, 0, 0, int(128 * va)))
            d.ellipse([x, y + 7 * k, x + 6 * k, y + 13 * k], fill=(255, 255, 255, int(255 * va)))
            self._text(d, (x + 14 * k, y + 3 * k), self.spaced("REAL TIME"), fl, la)
        else:
            x = self._tri(d, x, y + 5 * k, scale, va, bar=(kind == "skip")) + 8 * k
            x = self._text(d, (x, y + 1 * k), ("×%d" % n) if kind == "lapse" else "SKIP", fv, va)
            if kind == "skip" and sub:
                self._text(d, (x + 14 * k, y + 3 * k), self.spaced(sub), fl, la)
        self._cache[key] = im
        return im

    # ---- deorbit burn widget (P8). V16 restyle (requirement 2026-10-21: keep the content, HUD style only) ----
    def deorbit_widget(self, d_info, u, alpha=1.0, ss=3):
        """The deorbit burn as a HUD picture-in-picture widget, same family as HudV2's ASCENT / CONVERGENCE box
        (DEO_BOX: 340 x 330 design px, HudV2 Box() thin frame: radius 10, white 0.22, 1 px; NO fill, no panel):
        header 'D E O R B I T   B U R N' (12 px white, HudV2 sWinL) + the burn state on the right (12 px white 0.92,
        sWinR); thin white line art (HudV2 OrbitIcon weights): planet outline, atmosphere edge, the parking orbit dashed,
        the post-burn trajectory solid and drawn in as the burn progresses, the stage as a small dot; the numbers
        (10 px labels white 0.92 like HudV2 sSpark, 14 px values white): PE before -> after, delta-v done / total;
        a thin progress line at the bottom. White only, English only. Drawn at ss x and downsampled (smooth 1 px lines)."""
        import math
        from PIL import Image, ImageDraw
        BW, BH = DEO_BOX[2], DEO_BOX[3]
        k = self.k * ss
        im = Image.new("RGBA", (int(BW * k) + 2 * ss, int(BH * k) + 2 * ss), (0, 0, 0, 0))
        d = ImageDraw.Draw(im)
        A = lambda a: int(255 * a * alpha)      # noqa: E731
        sh = max(1, int(round(k)))              # 1 design px

        def line(pts, w, a, dash=None):
            """HudV2 Seg: a soft dark halo (black 0.3 a, w + 2.5) under a white line of width w (design px)."""
            segs = [pts]
            if dash:                             # (on, off) in design px along the polyline
                segs, cur, run, on = [], [pts[0]], 0.0, True
                for p0, p1 in zip(pts, pts[1:]):
                    L = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                    s = 0.0
                    while s < L:
                        left = (dash[0] if on else dash[1]) * k - run
                        step = min(left, L - s)
                        s += step
                        run += step
                        q = (p0[0] + (p1[0] - p0[0]) * s / L, p0[1] + (p1[1] - p0[1]) * s / L)
                        if on:
                            cur.append(q)
                        if run >= (dash[0] if on else dash[1]) * k - 1e-9:
                            if on and len(cur) > 1:
                                segs.append(cur)
                            on, run, cur = not on, 0.0, [q]
                if on and len(cur) > 1:
                    segs.append(cur)
            for sg in segs:
                d.line(sg, fill=(0, 0, 0, A(0.3 * a)), width=max(1, int(round((w + 2.5) * k))), joint="curve")
            for sg in segs:
                d.line(sg, fill=(255, 255, 255, A(a)), width=max(1, int(round(w * k))), joint="curve")

        def text(x, y, t, px, a, right=False):
            f = self.font(px * ss)
            if right:
                x = x * k - d.textlength(t, font=f)
            else:
                x = x * k
            d.text((x + sh, y * k + sh), t, font=f, fill=(0, 0, 0, A(0.5 * a)))
            d.text((x, y * k), t, font=f, fill=(255, 255, 255, A(a)))

        def circle(cx, cy, r, n=180):
            return [(cx + r * math.cos(2 * math.pi * i / n), cy - r * math.sin(2 * math.pi * i / n)) for i in range(n + 1)]

        # frame (HudV2 Box: RoundRect radius 10, line white 0.22) - outline only
        d.rounded_rectangle([0, 0, BW * k, BH * k], radius=10 * k, outline=(255, 255, 255, A(0.22)), width=sh)
        ign, cut = d_info["ign"], d_info["cutoff"]
        prog = min(max((u - ign) / max(cut - ign, 1e-6), 0.0), 1.0)
        if u < ign:
            state = "T−%d S" % int(math.ceil(ign - u))
        elif prog < 1.0:
            state = "%.1f / %.1f S" % (u - ign, cut - ign)
        else:
            state = "COMPLETE"
        text(14, 10, self.spaced("DEORBIT BURN"), 12, 1.0)
        text(BW - 14, 10, self.spaced(state), 12, 0.92, right=True)
        # orbit schematic, not to scale: altitudes at 0.42 px/km over a 46 px planet
        cx, cy, Rp, sc = 112 * k, 182 * k, 46 * k, 0.42 * k
        pe0 = d_info.get("pe0") if d_info.get("pe0") is not None else 100.0
        pe1 = d_info.get("pe1") if d_info.get("pe1") is not None else -40.0
        ro = Rp + pe0 * sc
        line(circle(cx, cy, Rp), 1.0, 0.55)                                  # planet outline (OrbitIcon 0.55)
        line(circle(cx, cy, Rp + 70.0 * sc), 1.0, 0.18)                      # atmosphere edge (70 km)
        line(circle(cx, cy, ro), 1.0, 0.6 - 0.25 * prog, dash=(4, 4))         # parking orbit, dashed
        if prog > 0.0:                         # the new trajectory: apoapsis at the burn point (top), periapsis down
            rp = ro + (Rp + pe1 * sc - ro) * prog
            sma, ecc = 0.5 * (ro + rp), (ro - rp) / (ro + rp)
            pts, nu = [], math.pi
            while nu > -math.pi:
                r = sma * (1 - ecc * ecc) / (1 + ecc * math.cos(nu))
                if r < Rp:
                    break
                phi = -math.pi / 2 + nu           # periapsis straight down, travel clockwise from the top
                pts.append((cx + r * math.cos(phi), cy - r * math.sin(phi)))
                nu -= 0.02
            if len(pts) > 1:
                line(pts, 1.6, 0.92)
        bx, by, rr = cx, cy - ro, 3.2 * k                                    # the stage at the burn point
        d.ellipse([bx - rr - 1.25 * k, by - rr - 1.25 * k, bx + rr + 1.25 * k, by + rr + 1.25 * k], fill=(0, 0, 0, A(0.3)))
        d.ellipse([bx - rr, by - rr, bx + rr, by + rr], fill=(255, 255, 255, A(1.0)))
        # numbers (right column)
        tx = 228
        text(tx, 62, self.spaced("PE NOW"), 10, 0.92)
        text(tx, 76, "%.0f KM" % pe0, 14, 1.0)
        text(tx, 112, self.spaced("PE AFTER"), 10, 0.92)
        text(tx, 126, ("%.0f KM" % pe1).replace("-", "−") if d_info.get("pe1") is not None else "IN THE AIR", 14, 1.0)
        dv = d_info.get("dv")
        if dv:
            text(tx, 162, self.spaced("DELTA-V"), 10, 0.92)
            text(tx, 176, "%.0f / %.0f M/S" % (dv * prog, dv), 14, 1.0)
        # progress: thin base line + the burnt part
        y = (BH - 22) * k
        d.line([(14 * k, y), ((BW - 14) * k, y)], fill=(255, 255, 255, A(0.22)), width=sh)
        if prog > 0:
            d.line([(14 * k, y), ((14 + (BW - 28) * prog) * k, y)], fill=(255, 255, 255, A(0.92)), width=2 * sh)
        return im.resize((im.width // ss, im.height // ss), Image.LANCZOS)


DEO_BOX = (90, 600, 340, 330)   # design px: HudV2's picture-in-picture box (WY 600, WW 340, WH 330) mirrored to the left
#                               side (x = 1920 - (W/2 + 530) - 340): the right slot is under the plume in the P8 shots


def deorbit_info(plan, m, evj):
    """The P8 1x window and the deorbit burn for the widget: dict(ut0, ut1, ign, cutoff, pe0, pe1, dv) or None."""
    if not m.get("deorbit"):
        return None
    segs = [s for s in plan if "P8" in s.phase.split("+") and s.kind == "real"]
    if not segs:
        return None
    return dict(ut0=min(s.ut0 for s in segs), ut1=max(s.ut1 for s in segs), ign=m["deorbit"][0],
                cutoff=m["deorbit"][1], pe0=evj.get("pe0_km"), pe1=evj.get("pe1_km"), dv=evj.get("dv"))


def deorbit_alpha(d_info, u):
    """0.5 s fade in / out at the P8 window edges."""
    return min(1.0, max(0.0, (u - d_info["ut0"]) / 0.5), max(0.0, (d_info["ut1"] - u) / 0.5))


def deorbit_frame(ov, im, d_info, u):
    """A P8 picture (PIL RGB) with the deorbit widget; no shadow lift (V16: the darkness is solved with lights)."""
    al = deorbit_alpha(d_info, u)
    if al > 0.0:
        p = ov.deorbit_widget(d_info, u, al)
        im.paste(p, (int(DEO_BOX[0] * ov.k), int(DEO_BOX[1] * ov.k)), p)
    return im


def ease(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def overlay_state(plan, si, k, fps=sr.FPS):
    """What is drawn on video frame k of segment si: None or (kind, n, scale, alpha, skip?). Badge rules (spec 3.2):
    every compressed frame carries the badge (intro: 2x for 1.5 s, 0.4 s shrink); the first 2 s of a 1x window that
    follows a compressed span carry REAL TIME (then a 0.5 s fade); 1x elsewhere: nothing."""
    s = plan[si]
    t = k / float(fps)
    prev = next((plan[j] for j in range(si - 1, -1, -1) if plan[j].kind != "drop"), None)
    if s.kind == "lapse":
        if prev is not None and prev.kind in ("lapse", "skip"):
            sc = 1.0
        else:
            sc = 2.0 if t < 1.5 else 2.0 - ease((t - 1.5) / 0.4)
        return ("lapse", sr.tier(s.speed), sc, 1.0, False)
    if s.kind == "skip":
        return ("skip", None, 1.0, 1.0, True)
    if prev is not None and prev.kind in ("lapse", "skip") and t < sr.REALTIME_BADGE + 0.5:
        return ("real", None, 1.0, 1.0 if t < sr.REALTIME_BADGE else 1.0 - (t - sr.REALTIME_BADGE) / 0.5, False)
    return None


def card_text(plan, m, period=None):
    """The skip badge's coast duration, '5 H 20 MIN', from the whole P7 span (release + 10 s -> deorbit - 10 s).
    V16: English only, no orbit count in Chinese; `period` is unused (kept for the callers)."""
    p7 = [s for s in plan if s.phase == "P7"]
    dur = (p7[-1].ut1 - p7[0].ut0) if p7 else 0.0
    n = int(dur) // 60
    return ("%d H %02d MIN" % (n // 60, n % 60)) if n >= 60 else ("%d MIN" % n)


# ---------------------------------------------------------------------------------------------------------------------
def build(frames):
    """(index, evj, mission, plan, out, problems) for a frames directory (no image work)."""
    idx = load_index(frames)
    evj = json.load(open(os.path.join(frames, "events.json")))
    uts = [x[1] for x in idx]
    rails = [x[2] for x in idx]
    m = sr.mission_from_events(evj, uts[0], uts[-1])
    plan = sr.playback_plan(m)
    runs = sr.compressed_runs(uts, rails, plan)
    plan = sr.badge_compressed(plan, runs)          # fallback: rails / gapped frames in a 1x window -> badged lapse
    out = sr.select_frames(uts, plan)
    ev = {k: tuple(v) for k, v in evj.get("ev", {}).items()}
    txs = sr.transitions(ev, evj.get("marks", {}), v2=True)
    problems = ["1x window %.1f-%.1f (%.1f s game, %d frames) captured on rails / gapped: shown as a BADGED lapse" % (
        a, b, b - a, n) for a, b, n in runs]
    problems += sr.check_plan(plan, m, [x for x in txs if x.firm]) + sr.check_frames(out, uts, plan)
    return idx, evj, m, plan, out, problems


def edl_text(plan, m, out, problems, evj):
    lines = ["# recording pipeline v2 EDL (tools/edit_video.py)", "phase kind ut0 ut1 game_s speed badge video_s"]
    for s in plan:
        lines.append("%-8s %-5s %.2f %.2f %8.1f %8s %6s %7.2f" % (
            s.phase, s.kind, s.ut0, s.ut1, s.ut1 - s.ut0, "-" if s.speed is None else "%.1f" % s.speed,
            "x%d" % sr.tier(s.speed) if s.kind == "lapse" else ("SKIP" if s.kind == "skip" else "-"), s.video))
    summ = sr.plan_summary(plan)
    tot = sum(summ.values())
    real = sum(s.video for s in plan if s.kind == "real")
    lines.append("# video seconds per phase: " + ", ".join("%s %.1f" % kv for kv in summ.items()))
    lines.append("# total %.1f s = %d:%04.1f (frames %d), 1x %.1f s (%.0f %%)" % (
        tot, int(tot) // 60, tot % 60, len(out), real, 100 * real / max(tot, 1e-9)))
    lines.append("# checks: " + ("PASS" if not problems else "FAILED, %d problem(s)" % len(problems)))
    lines += ["#   " + p for p in problems[:50]]
    return "\n".join(lines) + "\n"


def encode(frames, idx, plan, out, m, evj, path, fps=sr.FPS, deo=None):
    """deo: deorbit_info(): the P8 frames get the deorbit widget (V16 HUD style, no shadow lift)."""
    from PIL import Image
    first = os.path.join(frames, "%06d.jpg" % idx[0][0])
    with Image.open(first) as im0:
        w, h = im0.size
    ov = Overlays(w, h)
    black = io.BytesIO()
    Image.new("RGB", (w, h), (0, 0, 0)).save(black, "JPEG", quality=90)
    black = black.getvalue()
    coast = card_text(plan, m)
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "image2pipe", "-framerate", str(fps), "-c:v", "mjpeg", "-i", "-",
           "-vf", "scale=1920:-2", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "24", "-r", str(fps),
           "-movflags", "+faststart", path]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    t0 = time.time()
    try:
        for j, o in enumerate(out):
            st = overlay_state(plan, o.seg, o.k, fps)
            src = os.path.join(frames, "%06d.jpg" % idx[o.src][0]) if o.src is not None else None
            night = (deo is not None and src is not None and plan[o.seg].kind == "real"
                     and deo["ut0"] <= idx[o.src][1] <= deo["ut1"])
            if st is None and src is not None and not night:
                with open(src, "rb") as f:
                    p.stdin.write(f.read())
            else:
                im = Image.open(src).convert("RGB") if src is not None else Image.open(io.BytesIO(black)).convert("RGB")
                if night:
                    im = deorbit_frame(ov, im, deo, idx[o.src][1])
                if st is not None:                     # V16: the badge only - no dimming, no card
                    kind, n, sc, al, is_skip = st
                    b = ov.badge(kind, n, sc, al, coast if is_skip else None)
                    im.paste(b, (int(64 * ov.k), int(120 * ov.k)), b)
                buf = io.BytesIO()
                im.save(buf, "JPEG", quality=90)
                p.stdin.write(buf.getvalue())
            if j % 1500 == 0:
                print("edit_video: frame %d / %d (%.0f s)" % (j, len(out), time.time() - t0), flush=True)
        p.stdin.close()
    except BrokenPipeError:
        pass
    return p.wait() == 0


def main(argv):
    if not argv or argv[0].startswith("-"):
        print(__doc__)
        return 2
    frames = argv[0]
    name = argv[argv.index("--name") + 1] if "--name" in argv else os.path.basename(frames.rstrip("/")).replace("frames_", "")
    os.makedirs(os.path.join(ROOT, "videos"), exist_ok=True)
    path = argv[argv.index("--out") + 1] if "--out" in argv else \
        os.path.join(ROOT, "videos", time.strftime("%Y%m%d_%H%M%S_") + name + ".mp4")
    idx, evj, m, plan, out, problems = build(frames)
    edl = edl_text(plan, m, out, problems, evj)
    deo = deorbit_info(plan, m, evj)
    if deo is not None:
        edl += "# deorbit widget %.1f-%.1f (HUD style, no shadow lift)\n" % (deo["ut0"], deo["ut1"])
    edl_path = os.path.splitext(path)[0] + ".edl.txt"
    with open(edl_path, "w") as f:
        f.write(edl)
    print(edl, end="", flush=True)
    loud_failure(problems, edl_path)
    if "--plan-only" in argv:
        return 0
    ok = encode(frames, idx, plan, out, m, evj, path, deo=deo)
    print("video:", path if ok else None, flush=True)
    loud_failure(problems, edl_path)                # again at the end: the last lines of the flight log
    if ok and "--keep" not in argv and os.environ.get("KSP_REC_KEEP") != "1":
        shutil.rmtree(frames, ignore_errors=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
