"""Unified show timeline (show-timeline notes): replays a nominal mission through the director (Cutter), the lander
camera (showrun.landing_shot) and the HUD transition plan (showrun.transitions) and checks every rule:
no camera cut within 2 s (ORBIT icon 3 s) of a HUD transition, no rec.pause over one, the schedule and the order
of events. No KSP needed."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot import director as d, hud, showrun as sr  # noqa: E402

T0 = 1000.0                     # lift-off UT
DT = d.PERIOD                   # director tick
# nominal mission (UT); (planned, actual, reported at) for the events the flight code predicts
MAXQ = (T0 + 42, T0 + 44, T0 + 47)          # predicted at lift-off, detected 3 s after the peak
SRB = (T0 + 51, T0 + 51.5, T0 + 51.5)
MECO = (T0 + 180, T0 + 200, T0 + 200)
ORBIT = (T0 + 390, T0 + 390, T0 + 390)      # predicted at MECO
PAYLOAD = T0 + 400
DEORBIT = (T0 + 2608, T0 + 2608, T0 + 2640)  # predicted at T+1990, reported after the burn
RAILS = [(T0 + 230, T0 + 375), (T0 + 2000, T0 + 2590)]
BURNS = [(T0 + 381, T0 + 390), (T0 + 2608, T0 + 2640)]


def mission_ev(ut):
    ev = {}
    if ut >= T0:
        ev["LIFTOFF"] = (T0, True)
        ev["MAX-Q"] = (MAXQ[1], True) if ut >= MAXQ[2] else (MAXQ[0], False)
        ev["SRB SEP"] = (SRB[1], True) if ut >= SRB[2] else (SRB[0], False)
        ev["MECO"] = (MECO[1], True) if ut >= MECO[2] else (MECO[0], False)
    if ut >= MECO[2]:
        ev["ORBIT"] = (ORBIT[1], ut >= ORBIT[2])
    if ut >= PAYLOAD:
        ev["PAYLOAD SEP"] = (PAYLOAD, True)
    if ut >= T0 + 1990:
        ev["DEORBIT BURN"] = (DEORBIT[1], ut >= DEORBIT[2])
    return ev


def mission_state(ut):
    t = ut - T0
    h = 0.0 if t <= 0 else (2.5 * t * t if t < 60 else min(9000 + 400 * (t - 60), 100000.0))
    x = 0.0 if t <= 0 else 0.6 * t * t
    rng = math.hypot(410.0 + x, h)
    over = ut >= MECO[2]
    return d.State(ut, t if t >= 0 else None, mission_ev(ut), h, any(a + 1 < ut < b for a, b in BURNS),
                   any(a <= ut < b for a, b in RAILS), 2 if ut < SRB[2] else 0, over, rng, 30.0, 67.0)


def replay_director(t_end=T0 + 2650, states=mission_state, marks=None, extra_tx=()):
    c = d.Cutter()
    log, on_air = [], {}
    ut = T0 - 6.0
    while ut <= t_end:
        s = states(ut)
        txs = sr.transitions(s.ev, marks) + list(extra_tx)
        for a in c.step(s, txs):
            log.append((ut,) + a)
        on_air[round(ut, 2)] = (c.cur.id if c.cur else None, c.paused)
        ut = round(ut + DT, 2)
    return log, on_air


def final_tx():
    return sr.transitions(mission_ev(T0 + 1e5))


def cuts(log):
    """Every cut the viewer sees: camera cuts and the video cut of a resumed capture."""
    return [e for e in log if e[1] in ("cut", "resume")]


def paused_spans(log, t_end=T0 + 2650):
    spans, start = [], None
    for e in log:
        if e[1] == "pause":
            start = e[0]
        elif e[1] == "resume" and start is not None:
            spans.append((start, e[0]))
            start = None
    if start is not None:
        spans.append((start, t_end))
    return spans


def first_cut(log, frm, to):
    return next(e[0] for e in log if e[1] == "cut" and e[2] == frm and e[3] == to)


def test_transition_plan():
    tx = {t.name: t for t in final_tx()}
    assert tx["ASCENT_IN"].ut == T0 + 3.0 and tx["CAPTION_OUT"].ut == T0 + 30.0
    assert tx["ORBIT_ICON"].ut == ORBIT[1] + 2.0 and tx["ORBIT_ICON"].guard == 3.0
    assert tx["PAYLOAD_SPLIT"].ut == PAYLOAD                     # the split cue at the separation instant
    assert all(t.firm for t in tx.values())
    pred = {t.name: t for t in sr.transitions(mission_ev(T0 + 250))}
    assert not pred["ORBIT_ICON"].firm                         # planned from the predicted ORBIT


def test_no_cut_near_a_hud_transition():
    log, _ = replay_director()
    tx = final_tx()
    assert len(cuts(log)) >= 8
    for e in cuts(log):
        assert sr.cut_ok(e[0], tx), ("cut inside a HUD transition window", e, [x for x in tx if not sr.cut_ok(e[0], [x])])
        for x in tx:
            assert abs(e[0] - x.ut) >= x.guard - 1e-6, (e, x)


def test_no_pause_over_a_hud_transition():
    log, _ = replay_director()
    spans = paused_spans(log)
    assert spans, "the nominal mission has dead air"
    for a, b in spans:
        for x in final_tx():
            w0, w1 = sr.window(x)
            assert b <= w0 or a >= w1, ("paused over", x, (a, b))


def test_ascent_schedule():
    log, on_air = replay_director(t_end=T0 + 120)
    assert on_air[round(T0 - 4, 2)][0] == "S1"                     # pre-roll on the pad
    s1_s2 = first_cut(log, "S1", "S2")
    assert abs(s1_s2 - (T0 + sr.S1_END)) <= DT                     # S1 -> S2 at T+35
    assert T0 + sr.CAPTION_OUT + sr.KIND["CAPTION_OUT"][0] <= s1_s2 - 2.0   # caption gone before the cut
    assert all(on_air[round(T0 + k * DT, 2)][0] == "S1" for k in range(int(34 / DT)))   # ascent widget in S1
    for e in cuts(log):
        assert abs(e[0] - MAXQ[1]) >= sr.MAXQ_QUIET and abs(e[0] - MAXQ[0]) >= sr.MAXQ_QUIET   # MAX-Q: no cut
    assert on_air[round(SRB[1], 2)][0] == "S3"                      # booster separation on the hull camera


def test_orbit_schedule():
    log, on_air = replay_director(t_end=T0 + 500)
    o = ORBIT[1]
    for e in cuts(log):
        assert not (o - 1.0 < e[0] < o + 2.0 + 3.0), ("cut near the orbit icon shrink", e)
    assert on_air[round(o + 2.0, 2)][0] == "S4"                     # the burn shot holds through the shrink
    assert first_cut(log, "S4", "S5") >= o + 5.0 - 1e-6             # HUD first: the cut waited
    assert on_air[round(PAYLOAD, 2)][0] == "S5"                     # split cue + S5 at the same instant
    assert any(v[0] == "S6" for k, v in on_air.items() if RAILS[0][0] + 1 < k < RAILS[0][1])   # warp: wide shot


def test_event_order():
    log, _ = replay_director()
    seq = [(e[0], e[3]) for e in log if e[1] == "cut"]
    ids = [i for _, i in seq]
    assert ids[0] == "S1" and ids.index("S2") < ids.index("S3") < ids.index("S4") < ids.index("S5")
    assert "S6" in ids[ids.index("S5"):]                             # coast to the deorbit node
    tx = [x.name for x in final_tx()]
    assert tx == ["ASCENT_IN", "CAPTION_OUT", "ORBIT_ICON", "PAYLOAD_SPLIT"]


def test_pause_lifts_before_a_transition_and_cut_waits():
    """Adversarial: a transition inside dead air; an urgent shot wanted inside a transition window."""
    TX = sr._tx("PAYLOAD_SPLIT", T0 + 300.0, True)

    def dead(ut):                       # after MECO, nothing on air (no shot matches)
        return d.State(ut, ut - T0, {"LIFTOFF": (T0, True), "MECO": (T0 + 200, True)}, 9e4, False, False, 0,
                       True, 4e5, 30.0, 67.0)
    log, on_air = replay_director(t_end=T0 + 330, states=lambda ut: dead(ut) if ut > T0 + 200 else mission_state(ut),
                                  extra_tx=[TX])
    for a, b in paused_spans(log, T0 + 330):
        w0, w1 = sr.window(TX)
        assert b <= w0 or a >= w1, (a, b)
    res = [e[0] for e in log if e[1] == "resume" and e[0] < TX.ut]
    assert res and TX.ut - TX.guard - sr.PAUSE_LOOK <= res[-1] <= TX.ut - TX.guard, res
    # an urgent burn shot wanted 1 s after the transition starts: it waits for the end of the window
    c = d.Cutter()
    burn = lambda ut: d.State(ut, ut - T0, {"LIFTOFF": (T0, True), "MECO": (T0 + 200, True)}, 9e4,   # noqa: E731
                              ut >= TX.ut + 1.0, False, 0, True, 4e5, 30.0, 67.0)
    c.step(dead(TX.ut - 20), [TX])
    first = None
    ut = TX.ut - 1.0
    while ut < TX.ut + 6:
        for a in c.step(burn(ut), [TX]):
            if a[0] in ("cut", "resume") and first is None:
                first = ut
        ut += DT
    assert first is not None and first >= sr.window(TX)[1] - 1e-6, first


# ---- descent: the lander's cameras S7-S9 and the HUD widgets on the same marks ----
TE0, TE1 = 5000.0, 5020.0           # entry burn start / end (UT)
TLB, TTD = 5160.0, 5185.0           # landing burn / touchdown


def descent_h(ut):
    if ut < TLB:
        return max(36000.0 - 220.0 * (ut - TE0), 3000.0) if ut < 5150.0 else 3000.0 - 180.0 * (ut - 5150.0)
    return max(1200.0 - 1200.0 * ((ut - TLB) / (TTD - TLB)) ** 0.7, 0.0)


def replay_lander():
    marks, ev, log, shot, handoff = {}, {}, [], None, None
    ut = TE0 - 10.0
    while ut <= TTD + 20.0:
        h = descent_h(ut)
        vz = (descent_h(ut + 0.1) - h) / 0.1
        if ut >= TE0:
            ev["ENTRY BURN"] = (TE0, True)
        if ut >= TE1:
            ev["AERO GLIDE"] = (TE1, True)
        if ut >= TLB:
            ev["LANDING BURN"] = (TLB, True)
        td = ut >= TTD
        if td:
            ev["TOUCHDOWN"] = (TTD, True)
        if handoff is None and not td:
            eta = sr.handoff_eta(h, vz)
            if eta is not None and eta <= sr.HANDOFF_LEAD:
                handoff = ut + eta
                marks.setdefault("HANDOFF", handoff)
        if not td:                                         # the lander loop ends at touchdown
            sid = sr.landing_shot(ut, TE0 <= ut < TE1, TE0 - 4.0 <= ut < TE0, handoff, False)
            if sid == "S8" and shot == "S7":
                marks.setdefault("S8", ut)
            if sid != shot:
                log.append((ut, shot, sid))
                shot = sid
        ut = round(ut + DT, 2)
    return log, sr.transitions(ev, marks), marks


def test_landing_schedule():
    log, tx, marks = replay_lander()
    t = {x.name: x for x in tx}
    s8 = next(u for u, a, b in log if (a, b) == ("S7", "S8"))
    s9 = next(u for u, a, b in log if (a, b) == ("S8", "S9"))
    assert TE1 <= s8 < TE1 + DT + 1e-6                               # camera first at the end of the entry burn
    assert t["FUNNEL_IN"].ut == s8 + sr.FUNNEL_AFTER_CUT             # funnel 2 s after that cut
    assert abs(descent_h(t["HANDOFF"].ut) - sr.HANDOFF_H) < 100.0     # hand-off at ~1 km (frozen 1.5 s ahead)
    assert t["HANDOFF"].ut + sr.S9_AFTER_HANDOFF <= s9 < t["HANDOFF"].ut + sr.S9_AFTER_HANDOFF + DT + 1e-6
    assert t["TD_PLOT"].ut == TTD + sr.TD_PLOT
    assert all(u < TTD for u, _, _ in log)                          # no cut after touchdown: hold the frame
    for u, a, b in log:
        if a is not None:
            assert sr.cut_ok(u, tx), (u, a, b)
    order = [u for u, a, b in log if (a, b) == ("S8", "S7")] + [s8, t["FUNNEL_IN"].ut, t["HANDOFF"].ut, s9, TTD,
                                                                 t["TD_PLOT"].ut]
    assert order == sorted(order), order


# ---- the feed's overlay (v8) ----
def test_overlay_v8():
    f = hud.HudFeed(craft="TEST", target_km=100)
    f._ut = T0 - 1.0
    f.plan("MECO", T0 + 180)                    # first prediction = plan
    f.ev["LIFTOFF"] = [T0, True]                # (event() would also store t0 in logs/)
    f.t0, f.t0_src = T0, "liftoff"
    f._ut = T0 + 51.0
    f.plan("MECO", T0 + 200)                    # live prediction 20 s late -> red deviation label
    txt = f._compose(dict(alt=12700.0, speed=457.0, vs=370.0, ap=22100.0, pe=-581000.0, q=17700.0, thr=1.0, warp=4.0,
                          x=5600.0))
    kv = dict(l.split("=", 1) for l in txt.split("\n")[1:])
    assert kv["v"] == "8" and kv["t0"] == "%.3f" % T0 and kv["warp"] == "4"
    assert "ASCENT_IN@%.3f" % (T0 + 3) in kv["hud_tx"] and "CAPTION_OUT" in kv["hud_tx"]
    meco = [e.split(";") for e in kv["evx"].split("|") if e.startswith("MECO")][0]
    assert meco[2] == "%.3f" % (T0 + 180) and meco[4] == "2" and meco[5] == "+20s"
    assert kv["left"].startswith("ALT;12.7;KM") and kv["right"].startswith("THR;100;%")
    assert kv["asc"].startswith("12.700,5.600") and kv["target"] == "100"
    f.ev["TOUCHDOWN"] = [T0 + 10000, True]      # long after touchdown: no collapse
    f._ut = T0 + 10030
    assert "end=1" not in f._compose({})


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
