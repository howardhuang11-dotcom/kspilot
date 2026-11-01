#!/usr/bin/env python3
"""Sanitise the raw development records for release, and hash the unsanitised originals (data/raw, report Section 9.1).

This is the transform that produced data/raw/ from the author's private working directories. The originals are not
released; this script and data/raw/ORIGINAL_SHA256.txt let a holder of the originals check the release.

usage (all three source directories are those of the private working copies; paths are given on the command line, so no
machine path is stored here):
    python tools/sanitize_logs.py --logs LOGS_DIR --gnc2 GNC2_DIR --screenshots SHOTS_DIR \\
        [--out OUT_DIR] [--hashes FILE] [--manifest paper/tables/raw_manifest.sha256] [--map PREFIX=REPLACEMENT ...]

    --logs         directory with wd_*.out, <key>_landing.csv, experiments.jsonl, matrix_results.txt, rdv_r1_*.json and
                   the per-flight text logs (only their names are used)
    --gnc2         directory with the simulation result JSON files (docs/gnc2 of the simulation worktree)
    --screenshots  directory with the screenshots shown by the figures
    --manifest     the released file list (default paper/tables/raw_manifest.sha256); it selects which originals are used
    --out          write the sanitised copies to OUT_DIR/{logs,gnc2,screenshots} (default: do not write them)
    --hashes       write the SHA-256 of every unsanitised original used to FILE (sha256sum format, original names under
                   logs/, gnc2/, screenshots/); default data/raw/ORIGINAL_SHA256.txt when --out is not given
    --map          absolute path prefix of a private working directory and its replacement, repeatable, applied in the
                   given order (the first match wins for a given prefix; give the longest prefixes first). The release was
                   made with the worktrees mapped to "worktrees/<name>/" and the main working directory mapped to "".

Transform (applied to text records, nothing else is changed):
  1. absolute paths of the author's machine -> relative paths (--map);
  2. e-mail addresses other than the corresponding author's -> "[email removed]" (none occurred);
  3. every line that contains CJK characters (flight-planner and MechJeb status strings, part names): the phrases of the
     dictionary ZH below are replaced by English equivalents by plain substring replacement, longest first, and any CJK
     run left over becomes "[non-English text removed]" (none was left); numbers are not touched;
  4. two emoji marks -> "[ok]" / "[no]".
The descent CSVs must need no sanitising (asserted) and are stored gzip-compressed (level 9, empty header name, mtime 0).
Screenshots are re-encoded as PNG with Pillow (pixels unchanged; the byte stream depends on the Pillow version, so they
are compared by pixels). The dictionary is a substring dictionary, not a translation by a reader of Chinese: for example
the single character for "level/stage" becomes "stage" wherever it occurs on a line that is rewritten.

--check OUT_DIR compares an existing release tree with what this script produces from the originals and prints the files
that differ.
"""
import argparse
import glob
import gzip
import hashlib
import os
import re
import sys

# CJK dictionary, longest phrase first (as applied). Keys are written as \u escapes so that this file is pure ASCII.
ZH = [
 ("\uff08\u5df2\u6263\u56de\u6536\u63a8\u8fdb\u5242\uff0c\u5e26\u8f7d\u8377\uff09\uff0c\u4e0a\u5347\u9700\u6c42", " (recovery propellant deducted, with payload), ascent required"),
 ("\u5206\u79bb\u540e\u7684\u56de\u6536\u7ea7\u8ba1\uff09\uff1b\u9884\u8ba1\u5165\u8f68\u5269", " of the recovery stage after separation); expected remaining in orbit"),
 ("\u6ed1\u884c\u5230\u5706\u5316\u8f68\u9053\u6267\u884c\u70b9", "coast to circularization burn point"),
 ("\u4f4e\u63a8\u91cd\u6bd4\u91cd\u529b\u635f\u5931", "low-TWR gravity loss"),
 ("\u6db2\u4f53\u71c3\u6599\u5f15\u64ce", "liquid-fuel engine"),
 ("\u592a\u9633\u80fd\u7535\u6c60\u677f", "solar panel"),
 ("\uff08\u771f\u7a7a\u5f53\u91cf\uff09", " (vacuum equivalent)"),
 ("\u4fef\u4ef0\u89d2\u53d8\u5316\u7a0b\u5e8f", "pitch program"),
 ("\u5165\u8f68\u9700\u6c42", "insertion requirement"),
 ("\u56de\u6536\u63a8\u8fdb\u5242", "recovery propellant"),
 ("\u7740\u9646\u70b9\u706b", "landing burn"),
 ("\u518d\u5165\u70b9\u706b", "entry burn"),
 ("\u56de\u6536\u9700\u6c42", "recovery requirement"),
 ("\u7ec8\u7aef\u901f\u5ea6", "terminal velocity"),
 ("\u4e0a\u5347\u53ef\u7528", "ascent available"),
 ("\uff0c\u603b\u9700\u6c42", ", total required"),
 ("\uff0c\u7740\u9646\u540e\u5269", ", remaining after landing"),
 ("\uff0c\u6309\u8f7d\u8377", ", for payload"),
 ("\uff08\u56de\u6536\u4fa7", " (recovery side"),
 ("\u7a33\u5b9a\u7684\u5bfc\u5411", "stable guidance"),
 ("\u5782\u76f4\u4e0a\u5347", "vertical ascent"),
 ("\u5706\u5316\u8f68\u9053", "circularize orbit"),
 ("\u62d2\u7edd\u53d1\u5c04", "launch refused"),
 ("\u4e2d\u7ee7\u536b\u661f", "relay satellite"),
 ("\u603b\u53ef\u7528", "total available"),
 ("\u7740\u9646\u817f", "landing legs"),
 ("\u53d1\u52a8\u673a", "engines"),
 ("\uff0c\u4f59\u91cf", ", margin"),
 ("\u4f59\u91cf\uff09", "margin)"),
 ("\uff08\u79bb\u8f68", " (deorbit"),
 ("\u56de\u6536\u7ea7", "recovery stage"),
 ("\u6781\u9650\u706b\u7bad", "[mod part name]"),
 ("\u71c3\u6599\u7bb1", "fuel tank"),
 ("\u63a2\u6d4b\u5668", "probe"),
 ("\u63a8\u8fdb\u5668", "thruster"),
 ("\u6d77\u5e73\u9762", "sea level"),
 ("\u771f\u7a7a", "vacuum"),
 ("\u71c3\u70e7", "burn"),
 ("\u8d28\u91cf", "mass"),
 ("\u5e72\u91cd", "dry"),
 ("\u6e7f\u91cd", "wet"),
 ("\u5224\u5b9a", "verdict"),
 ("\u653e\u884c", "go"),
 ("\u7740\u9646", "landing"),
 ("\u4e0a\u5347", "ascent"),
 ("\u4e0d\u8db3", "short"),
 ("\u8239\u957f", "[part name]"),
 ("\u7ea7", "stage"),
 ("\uff08", " ("),
 ("\uff09", ")"),
]
CJK = re.compile("[\u3000-\u9fff\uff00-\uffef]+")
EMAIL = re.compile(r"[\w.+-]+@[\w-]+(?:\.[\w-]+)+")
KEEP_EMAIL = "howardhuang11@outlook.com"
MARKS = (("\u2705", "[ok]"), ("\u274c", "[no]"))


def sanitize(t, path_map=(), stats=None):
    """Sanitise one text record; stats (dict) accumulates counts."""
    st = stats if stats is not None else {}
    for k in ("paths", "emails", "cjk_lines", "cjk_runs", "emoji", "residual_cjk_runs"):
        st.setdefault(k, 0)
    for a, b in path_map:
        n = t.count(a)
        if n:
            st["paths"] += n
            t = t.replace(a, b)

    def em(m):
        if m.group(0) == KEEP_EMAIL:
            return m.group(0)
        st["emails"] += 1
        return "[email removed]"

    t = EMAIL.sub(em, t)
    out = []
    for ln in t.split("\n"):
        if CJK.search(ln):
            st["cjk_lines"] += 1
            st["cjk_runs"] += len(CJK.findall(ln))
            for a, b in ZH:
                ln = ln.replace(a, "\x00" + b + "\x01")
            ln = re.sub(r"(?<=[A-Za-z0-9])\x00", " ", ln)
            ln = re.sub(r"\x01(?=[A-Za-z0-9])", " ", ln)
            ln = ln.replace("\x00", "").replace("\x01", "")
            r = CJK.findall(ln)
            if r:
                st["residual_cjk_runs"] += len(r)
                ln = CJK.sub("[non-English text removed]", ln)
        out.append(ln)
    t = "\n".join(out)
    for a, b in MARKS:
        n = t.count(a)
        st["emoji"] += n
        t = t.replace(a, b)
    return t


def gz_bytes(b):
    import io
    buf = io.BytesIO()
    with gzip.GzipFile(filename="", mode="wb", fileobj=buf, compresslevel=9, mtime=0) as g:
        g.write(b)
    return buf.getvalue()


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def released_files(manifest):
    """Released relative paths (under data/raw) listed in the manifest."""
    rels = []
    for ln in open(manifest, encoding="utf-8"):
        parts = ln.rstrip("\n").split(None, 1)
        if len(parts) == 2 and not ln.startswith("#"):
            rels.append(parts[1].lstrip("*").strip())
    return sorted(rels)


def original_of(rel, src):
    """(class, original file name, path) of the unsanitised original of a released file; None for derived files."""
    cls, _, name = rel.partition("/")
    if cls == "logs" and name == "flight_log_names.txt":
        return None
    if cls == "logs" and name.endswith("_landing.csv.gz"):
        name = name[:-3]
    return cls, name, os.path.join(src[cls], name)


def flight_log_names(logs):
    return sorted(os.path.basename(p) for p in glob.glob(os.path.join(logs, "*_orbit_*.txt"))
                  if re.search(r"\d{8}_\d{6}_orbit_.+\.txt$", p))


def produce(rel, src, path_map, stats):
    """Bytes of the released file rel, or ("png", PIL image) for screenshots."""
    o = original_of(rel, src)
    if o is None:
        return ("\n".join(flight_log_names(src["logs"])) + "\n").encode("utf-8")
    cls, name, path = o
    if cls == "screenshots":
        from PIL import Image
        im = Image.open(path)
        im.load()
        return ("png", im)
    if name.endswith("_landing.csv"):
        b = open(path, "rb").read()
        assert sanitize(b.decode("utf-8"), path_map) == b.decode("utf-8"), path  # csv must need no sanitising
        return gz_bytes(b)
    if cls == "gnc2":
        t = open(path, encoding="utf-8").read()
    else:
        t = open(path, encoding="utf-8", errors="strict", newline="").read()
    return sanitize(t, path_map, stats).encode("utf-8")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--logs", required=True)
    ap.add_argument("--gnc2", required=True)
    ap.add_argument("--screenshots", required=True)
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--manifest", default=os.path.join(root, "paper", "tables", "raw_manifest.sha256"))
    ap.add_argument("--out", default=None)
    ap.add_argument("--hashes", default=None)
    ap.add_argument("--check", default=None, metavar="OUT_DIR")
    ap.add_argument("--map", action="append", default=[], metavar="PREFIX=REPLACEMENT")
    a = ap.parse_args(argv)
    path_map = [tuple(m.split("=", 1)) for m in a.map]
    src = {"logs": a.logs, "gnc2": a.gnc2, "screenshots": a.screenshots}
    rels = released_files(a.manifest)
    stats = {}

    if a.hashes or not (a.out or a.check):
        target = a.hashes or os.path.join(root, "data", "raw", "ORIGINAL_SHA256.txt")
        lines = []
        for rel in rels:
            o = original_of(rel, src)
            if o is None:
                continue
            cls, name, path = o
            lines.append("%s  %s/%s" % (sha256(path), cls, name))
        with open(target, "w", encoding="utf-8", newline="\n") as fh:
            fh.write("\n".join(sorted(lines, key=lambda s: s.split(None, 1)[1])) + "\n")
        print("wrote %d hashes to %s" % (len(lines), target))

    if a.out or a.check:
        base = a.out or a.check
        bad = []
        for rel in rels:
            r = produce(rel, src, path_map, stats)
            dst = os.path.join(base, rel)
            if a.out:
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                if isinstance(r, tuple):
                    r[1].save(dst, optimize=True)
                else:
                    open(dst, "wb").write(r)
            else:
                if not os.path.isfile(dst):
                    bad.append(rel + " (missing)")
                elif isinstance(r, tuple):
                    from PIL import Image
                    if list(Image.open(dst).convert("RGBA").getdata()) != list(r[1].convert("RGBA").getdata()):
                        bad.append(rel)
                elif open(dst, "rb").read() != r:
                    bad.append(rel)
        print("sanitiser counts:", stats)
        if a.check:
            print("%d of %d released files differ from the sanitised originals" % (len(bad), len(rels)))
            for b in bad:
                print("  ", b)
            sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
