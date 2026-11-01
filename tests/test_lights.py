"""V17 light kit (tools/craft_lights.py, craft/VAB/*-LIT.craft) and its flight-script hook (tools/orbit_recover.py,
HOP_REC=full only: lamps on for each node burn, off after it). No KSP."""
import math
import os
import sys
import textwrap

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
import craft_ballast as cb  # noqa: E402
import craft_lights as cl  # noqa: E402

SRC = open(os.path.join(ROOT, "tools", "orbit_recover.py")).read()
GAME = os.path.join(os.path.dirname(cb.GAMEDATA), "GameData", "Squad", "Parts")


# ---------------------------------------------------------------- flight-script hook
def _rec_full_block():
    return SRC.split("\nif REC_FULL:\n", 1)[1].split("\nif FROM_ORBIT or FROM_ENTRY:", 1)[0]


def test_lights_hook_only_under_rec_full():
    blk = _rec_full_block()
    assert SRC.count("control.lights") == 1 and "vessel.control.lights = on" in blk
    assert SRC.count("_burn_mod.execute_node = _exec_node_lit") == 1 and "_burn_mod.execute_node = _exec_node_lit" in blk
    for root, _, files in os.walk(os.path.join(ROOT, "kspilot")):
        for f in files:
            if f.endswith(".py"):
                body = open(os.path.join(root, f)).read()
                assert "control.lights" not in body and "execute_node = " not in body, f


class _Ctl:
    def __init__(self, seen):
        self._on, self.seen = False, seen

    @property
    def lights(self):
        return self._on

    @lights.setter
    def lights(self, v):
        self._on = bool(v)
        self.seen.append(self._on)


class _Vessel:
    def __init__(self):
        self.seen = []
        self.control = _Ctl(self.seen)


def _hook(execute_node):
    """Run the hook's code (from the flight script, verbatim) against a fake kspilot.burn module."""
    blk = _rec_full_block()
    start = blk.index("    # V17 (video requirements v2)")
    end = blk.index("    _burn_mod.execute_node = _exec_node_lit") + len("    _burn_mod.execute_node = _exec_node_lit")
    code = textwrap.dedent(blk[start:end]).replace("from kspilot import burn as _burn_mod  # noqa: E402", "")
    fake = type("burn", (), {})()
    fake.execute_node = execute_node
    logs = []
    exec(code, {"_burn_mod": fake, "log": logs.append})
    return fake, logs


def test_lights_on_during_the_burn_and_off_after():
    v = _Vessel()
    during = []

    def execute_node(conn, vessel, node, log, autostage_limit=0):
        during.append(vessel.control.lights)
        return 1.5
    fake, logs = _hook(execute_node)
    assert fake.execute_node("conn", v, "node", print) == 1.5
    assert during == [True] and v.seen == [True, False] and not v.control.lights
    assert logs == ["lights on (V17)", "lights off (V17)"]


def test_lights_off_after_a_missed_burn_or_an_exception_and_never_fatal():
    v = _Vessel()
    fake, _ = _hook(lambda c, vessel, n, log: None)
    assert fake.execute_node("c", v, "n", print) is None and v.seen == [True, False]

    def boom(c, vessel, n, log):
        raise RuntimeError("lost")
    fake, _ = _hook(boom)
    with pytest.raises(RuntimeError):
        fake.execute_node("c", v, "n", print)
    assert v.seen[-2:] == [True, False]

    class NoLights:                         # kRPC failure on the lights call: the burn still flies
        class control:
            @property
            def lights(self):
                raise RuntimeError

            @lights.setter
            def lights(self, v):
                raise RuntimeError("no")
        control = control()
    fake, logs = _hook(lambda c, vessel, n, log: 2.0)
    assert fake.execute_node("c", NoLights(), "n", print) == 2.0
    assert len(logs) == 2 and all("failed" in s for s in logs)


# ---------------------------------------------------------------- crafts
def _fake_db(*paths):
    parts, dens = {}, {}
    for p in paths:
        _, blocks, _ = cb.split_parts(open(p).read())
        for b in blocks:
            f = cb.fields(b)
            parts[f["part"][0].rsplit("_", 1)[0]] = 0.1
            for r in f["_res"]:
                dens[r] = 0.005
    return parts, dens


@pytest.mark.parametrize("ship", sorted(cl.BASES))
def test_lit_craft_is_the_committed_build_and_checks(ship):
    base, dst = cl.BASES[ship], cl.dst_of(ship)
    text = cl.build(base, ship + "-LIT")
    assert text == open(dst).read(), "%s is stale: run tools/craft_lights.py build" % dst
    probs, info = cb.check(dst, base, _fake_db(base, dst))
    assert probs == [], probs
    assert ("ship = %s-LIT" % ship) in text
    _, blocks, _ = cb.split_parts(text)
    fs = {cb.fields(b)["part"][0]: (cb.fields(b), b) for b in blocks}
    lamps = [p for p in fs if p.startswith("spotLight3_")]
    assert len(lamps) == 8 and info["n_parts"] == len(cb.split_parts(open(base).read())[1]) + 8
    assert text.count("name = ModuleLight") == 8
    for p in lamps:
        f, b = fs[p]
        assert "ToggleLightAction\n\t\t\t{\n\t\t\t\tactionGroup = Light" in b and "isOn = False" in b
        assert f["istg"] == ["-1"] and f["sidx"] == ["-1"] and f["attm"] == ["1"] and len(f["sym"]) == 3
    # the baseline parts are byte-identical apart from the two parent tanks' new links
    _, bblocks, _ = cb.split_parts(open(base).read())
    for b in bblocks:
        pid = cb.fields(b)["part"][0]
        new = fs[pid][1]
        if pid in (cl.TANK32, cl.TANK64):
            assert new.replace("".join("\n\tlink = %s" % l for l in lamps if l in new), "") == b
        else:
            assert new == b


def _lamps(path):
    _, _, _, f = cl._parse(path)
    ax, _, az0 = cl._vec(f[cl.TANK64]["pos"][0])
    out = []
    for p, g in f.items():
        if p.startswith("spotLight3_"):
            x, y, z = cl._vec(g["pos"][0])
            q = cl._vec(g["rot"][0])
            out.append((p, y, cl.azimuth(x - ax, z - az0), math.hypot(x - ax, z - az0), cl.qrot(q, (0, 0, -1)),
                        g["srfN"][0]))
    return f, out


@pytest.mark.parametrize("ship", sorted(cl.BASES))
def test_lit_geometry_symmetric_and_beams_along_the_body(ship):
    f, lamps = _lamps(cl.dst_of(ship))
    top = sorted(l for l in lamps if abs(l[1] - cl.TOP_Y) < 1e-6)
    bot = sorted(l for l in lamps if abs(l[1] - cl.BOT_Y) < 1e-6)
    assert len(top) == 4 and len(bot) == 4
    for ring, parent, sgn in ((top, cl.TANK32, -1.0), (bot, cl.TANK64, 1.0)):
        azs = sorted(l[2] for l in ring)
        assert all(abs((azs[i + 1] - azs[i]) - 90.0) < 0.05 for i in range(3))       # x4 radial symmetry
        for p, y, az, r, beam, srfn in ring:
            assert r == pytest.approx(cl.R_ATTACH, abs=1e-4) and srfn.startswith("srfAttach,%s," % parent)
            out = (-math.cos(math.radians(az)), math.sin(math.radians(az)))
            assert beam[1] == pytest.approx(sgn * math.sin(math.radians(cl.TILT)), abs=1e-6)   # down / up
            horiz = (beam[0], beam[2])
            assert horiz[0] * out[0] + horiz[1] * out[1] == pytest.approx(math.cos(math.radians(cl.TILT)), abs=1e-6)
    # rings sit in the gaps: 22.5 deg off the airbrakes / grid fins, between the SRB decouplers and the legs
    ax, _, az0 = cl._vec(f[cl.TANK64]["pos"][0])

    def az_of(prefix):
        return [cl.azimuth(cl._vec(g["pos"][0])[0] - ax, cl._vec(g["pos"][0])[2] - az0) for p, g in f.items()
                if p.startswith(prefix)]

    def sep(a, b):
        return abs((a - b + 180.0) % 360.0 - 180.0)
    for l in top:
        assert min(sep(l[2], a) for a in az_of("airbrake1_")) == pytest.approx(22.5, abs=0.1)
        assert min(sep(l[2], a) for a in az_of("Grid Fin M_")) == pytest.approx(22.5, abs=0.1)
    for l in bot:
        assert min(sep(l[2], a) for a in az_of("radialDecoupler_")) > 25.0
        assert min(sep(l[2], a) for a in az_of("landingLeg1-2_")) > 25.0
        assert min(sep(l[2], a) for a in az_of("launchClamp1_")) > 15.0


def test_quaternion_convention_matches_a_flown_part():
    """Untilted lamp at azimuth 0 = the launch clamp at azimuth 0 (attach node orientation 0,0,-1 too): same
    rotation (q and -q are the same rotation)."""
    q = cl.lamp_quat(0.0, 0.0, True)
    clamp = (1.26440597e-07, -0.707106829, -1.26440597e-07, -0.707106829)       # launchClamp1_4294708966
    assert abs(sum(a * b for a, b in zip(q, clamp))) == pytest.approx(1.0, abs=1e-6)
    for az in (0.0, 44.6, 254.25):
        for down in (True, False):
            q = cl.lamp_quat(az, cl.TILT, down)
            assert sum(c * c for c in q) == pytest.approx(1.0, abs=1e-8)
            assert cl.qrot(q, (0, 0, 0))[0] == 0.0


@pytest.mark.skipif(not os.path.isdir(GAME), reason="KSP install not present")
@pytest.mark.parametrize("ship", sorted(cl.BASES))
def test_lit_clearances_against_the_part_envelopes(ship):
    boxes = cl._cubes_raw()
    assert "spotLight3" in boxes
    # launch clamps: their box envelope includes the top arm 3.9 m above the lamps; they stay on the pad
    for lamp, gap, who in cl.clearances(cl.dst_of(ship), boxes, skip=("launchClamp1_",)):
        assert gap > 0.15, (lamp, who, gap)
