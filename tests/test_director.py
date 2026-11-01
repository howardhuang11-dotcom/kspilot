"""Shot table of kspilot.director against a nominal mission timeline (no KSP needed; director shot list)."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot import core, director as d  # noqa: E402

T0 = 1000.0     # lift-off UT


def st(ut, ev=None, h=0.0, burning=False, rails=False, srb=0, over=False, rng=400.0, met=True):
    return d.State(ut, (ut - T0) if met else None, ev or {}, h, burning, rails, srb, over, rng, 5.0, 67.0)


def shot(s):
    p = d.pick(s)
    return p.id if p else None


def test_timeline():
    lift = {"LIFTOFF": (T0, True)}
    sep_plan = dict(lift, **{"SRB SEP": (T0 + 51, False)})
    sep_done = dict(lift, **{"SRB SEP": (T0 + 52, True)})
    meco = dict(sep_done, **{"MECO": (T0 + 200, True)})
    orbit = dict(meco, **{"ORBIT": (T0 + 390, True)})
    pay = dict(orbit, **{"PAYLOAD SEP": (T0 + 400, True)})
    seq = [
        ("pre-roll on the pad", st(T0 - 4, met=False), "S1"),
        ("liftoff, low", st(T0 + 10, lift, h=900, rng=1200), "S1"),
        ("climb out of the long lens", st(T0 + 40, sep_plan, h=6000, rng=7000), "S2"),
        ("booster sep pre-roll", st(T0 + 49, sep_plan, h=8000, srb=2, rng=9000), "S3"),
        ("booster sep hold", st(T0 + 57, sep_done, h=10000, rng=11000), "S3"),
        ("upper ascent", st(T0 + 70, sep_done, h=14000, rng=15000), "S2"),
        ("MECO hold", st(T0 + 204, meco, h=60000, over=True, rng=9e4), "S2"),
        ("after MECO, before warp: dead air", st(T0 + 215, meco, h=62000, over=True, rng=9e4), None),
        ("coast to apoapsis on rails", st(T0 + 300, meco, h=90000, over=True, rails=True, rng=3e5), "S6"),
        ("circularisation pre-roll", st(T0 + 385, dict(meco, ORBIT=(T0 + 390, False)), h=1e5, over=True, rng=4e5), "S4"),
        ("payload framing", st(T0 + 395, orbit, h=1e5, over=True, rng=4e5), "S5"),
        ("payload hold", st(T0 + 408, pay, h=1e5, over=True, rng=4e5), "S5"),
        ("drag probe: dead air", st(T0 + 470, pay, h=1e5, over=True, rng=4e5), None),
        ("warp to the node", st(T0 + 2000, pay, h=1e5, over=True, rails=True, rng=4e5), "S6"),
        ("deorbit burn pre-roll", st(T0 + 2600, dict(pay, **{"DEORBIT BURN": (T0 + 2608, False)}), h=1e5, over=True, rng=4e5), "S4"),
        ("deorbit burn", st(T0 + 2610, pay, h=1e5, over=True, burning=True, rng=4e5), "S4"),
    ]
    for what, s, want in seq:
        assert shot(s) == want, (what, shot(s), want)
    assert len({sh.id for sh in d.SHOTS}) + 3 <= 10       # + S7-S9 of the lander: no more than ~10 shots


def test_s1_holds_to_t35():
    """Unified timeline (show-timeline notes): the S1 -> S2 cut is fixed at T+35 s (the auto-zoom keeps the stage
    framed above 3 km); the full replay with the HUD guards is tests/test_showrun.py."""
    lift = {"LIFTOFF": (T0, True)}
    assert shot(st(T0 + 34.5, lift, h=3600, rng=4200)) == "S1"
    assert shot(st(T0 + 35.0, lift, h=3800, rng=4400)) == "S2"


def test_no_booster_shot_without_boosters():
    ev = {"LIFTOFF": (T0, True), "SRB SEP": (T0 + 51, False)}      # nominal plan, but a core-only craft
    assert shot(st(T0 + 50, ev, h=8000, srb=0, rng=9000)) == "S2"


def test_tiny_trim_is_not_a_burn_shot():
    ev = {"LIFTOFF": (T0, True), "MECO": (T0 + 200, True)}
    assert shot(st(T0 + 3000, ev, h=1e5, over=True, burning=False, rng=4e5)) is None


def test_look_angles_from_the_pad_camera():
    cam = d.pad_camera(pad_alt=70.0)
    rng, el, az = d.look_angles(cam, core.KSC_PAD[0], core.KSC_PAD[1], 70.0)
    assert abs(rng - math.hypot(math.hypot(160, 380), 12)) < 2.0
    assert abs(az - math.degrees(math.atan2(380, 160))) < 0.5      # pad lies NE of the camera
    assert -2.5 < el < -1.0                                        # camera 12 m above the pad deck
    rng, el, az = d.look_angles(cam, core.KSC_PAD[0], core.KSC_PAD[1], 3070.0)
    assert 80 < el < 83                                            # 3 km straight up over the pad


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
