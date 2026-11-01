"""tools/sanitize_logs.py on synthetic records: path mapping, CJK dictionary, e-mail rule, numbers untouched, hashes.

The CJK strings are built from code points so that this file stays free of CJK characters.
"""
import hashlib
import importlib.util
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
spec = importlib.util.spec_from_file_location("sanitize_logs", os.path.join(ROOT, "tools", "sanitize_logs.py"))
S = importlib.util.module_from_spec(spec)
spec.loader.exec_module(S)

LANDING = "".join(chr(c) for c in (0x7740, 0x9646))        # "landing"
DRY = "".join(chr(c) for c in (0x5e72, 0x91cd))            # "dry mass"
STAGE = chr(0x7ea7)                                        # "level/stage"


def test_paths_are_mapped_and_counted():
    st = {}
    t = "log /private/wt/gnc2/docs/a.json and /private/main/logs/x.txt\n"
    out = S.sanitize(t, [("/private/wt/gnc2/", "worktrees/gnc2/"), ("/private/main/", "")], st)
    assert out == "log worktrees/gnc2/docs/a.json and logs/x.txt\n" and st["paths"] == 2


def test_cjk_phrase_replaced_numbers_kept():
    st = {}
    line = "feasibility: %s 11.4t, %s 3 burn 590 m/s" % (DRY, LANDING)
    out = S.sanitize(line + "\nplain 1.0 line\n", [], st)
    assert out == "feasibility: dry 11.4t, landing 3 burn 590 m/s\nplain 1.0 line\n"
    assert st["cjk_lines"] == 1 and st["cjk_runs"] == 2 and st["residual_cjk_runs"] == 0
    assert not S.CJK.search(out)


def test_substring_dictionary_is_blunt():
    # documented limitation: the single character for "stage" becomes "stage" wherever it occurs
    assert S.sanitize("%s 2" % STAGE, []) == "stage 2"


def test_unknown_cjk_is_removed_not_kept():
    st = {}
    out = S.sanitize("x %s y" % chr(0x9f98), [], st)    # a character that is in no dictionary phrase
    assert out == "x [non-English text removed] y" and st["residual_cjk_runs"] == 1


def test_email_rule_and_marks():
    st = {}
    other = "a" + chr(64) + "b.com"       # built from parts so that the repository holds no other address
    out = S.sanitize("%s howardhuang11@outlook.com %s" % (other, chr(0x2705)), [], st)
    assert out == "[email removed] howardhuang11@outlook.com [ok]" and st["emails"] == 1 and st["emoji"] == 1


def test_gzip_is_deterministic_and_roundtrips():
    import gzip
    b = b"a,b\n1,2\n" * 50
    assert S.gz_bytes(b) == S.gz_bytes(b) and gzip.decompress(S.gz_bytes(b)) == b


def test_released_hash_file_is_consistent_with_manifest():
    hashes = [l.split(None, 1) for l in open(os.path.join(ROOT, "data", "raw", "ORIGINAL_SHA256.txt"), encoding="utf-8")]
    names = {n.strip() for _, n in hashes}
    assert all(len(h) == 64 for h, _ in hashes) and len(names) == len(hashes)
    rels = S.released_files(os.path.join(ROOT, "paper", "tables", "raw_manifest.sha256"))
    expected = set()
    for r in rels:
        cls, _, n = r.partition("/")
        if r == "logs/flight_log_names.txt":
            continue
        expected.add("%s/%s" % (cls, n[:-3] if n.endswith("_landing.csv.gz") else n))
    assert names == expected
    # gnc2 JSON files are released unchanged, so their original hash equals the released file's hash
    for h, n in hashes:
        if n.startswith("gnc2/"):
            assert hashlib.sha256(open(os.path.join(ROOT, "data", "raw", n.strip()), "rb").read()).hexdigest() == h
