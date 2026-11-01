"""Static checks of the payload ballast craft files (no KSP running; masses need the install's part cfgs)."""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import craft_ballast as cb  # noqa: E402

VAB = os.path.join(ROOT, "craft", "VAB")
HAVE_KSP = os.path.isdir(cb.GAMEDATA)
_DB = []


def db():
    if not _DB:
        _DB.append(cb.stock_db())
    return _DB[0]


def _check(name):
    return cb.check(os.path.join(VAB, name), cb.SRC, db())


def test_committed_files_match_the_generator():
    for tag, ore in cb.VARIANTS.items():
        ship = "T-WorksFineGF-%s" % tag
        assert open(os.path.join(VAB, ship + ".craft")).read() == cb.build(ore, ship), ship


def test_structure_and_staging():
    if not HAVE_KSP:
        print("skip (no KSP install)")
        return
    for tag in cb.VARIANTS:
        probs, info = _check("T-WorksFineGF-%s.craft" % tag)
        assert not probs, probs
        assert info["n_parts"] == 54 and info["ship"] == "T-WorksFineGF-%s" % tag


def test_payload_masses():
    if not HAVE_KSP:
        print("skip (no KSP install)")
        return
    _, base = cb.check(cb.SRC, None, db())
    assert abs(base["total_no_clamps"] - 92.7) < 0.05          # the stock calculator's lift-off mass, as logged
    assert abs(base["payload"] - 2.48) < 0.005
    for tag, target in (("P45", 4.5), ("P60", 6.0)):
        _, info = _check("T-WorksFineGF-%s.craft" % tag)
        assert abs(info["payload"] - target) < 0.05, (tag, info["payload"])
        added = 2.0 + 0.010 * cb.VARIANTS[tag]           # LargeTank dry + Ore, only above the decoupler
        assert abs(info["total"] - base["total"] - added) < 1e-6
        assert abs(info["payload"] - base["payload"] - added) < 1e-6


def test_checker_catches_broken_files():
    if not HAVE_KSP:
        print("skip (no KSP install)")
        return
    import tempfile
    good = cb.build(2, "T-X")
    cases = {
        "dangling": good.replace("\tlink = fairingSize2_4291793832\n\tattN = top,fairingSize2",
                                 "\tlink = fairingSize9_1\n\tattN = top,fairingSize2", 1),
        "duplicate": good.replace("part = LargeTank_4291000045", "part = Decoupler.2_4291537548", 1),
        "restaged": good.replace("\tistg = 1\n\tresPri = 0\n\tdstg = 1\n\tsidx = 0", "\tistg = 2\n\tresPri = 0\n"
                                 "\tdstg = 1\n\tsidx = 0", 1),
    }
    for name, text in cases.items():
        assert text != good, name
        with tempfile.NamedTemporaryFile("w", suffix=".craft", delete=False) as f:
            f.write(text)
        try:
            probs, _ = cb.check(f.name, cb.SRC, db())
        finally:
            os.unlink(f.name)
        assert probs, name


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
