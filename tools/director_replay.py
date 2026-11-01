"""Offline replay of the recording pipeline v2 (no game, no kRPC) against a flown mission: the v2 director (ascent /
orbit, kspilot.director.CutterV2), the v2 lander camera (kspilot.showrun.DescentDirector, fed from the landing CSV)
and the editor's playback plan (kspilot.showrun.playback_plan + select_frames on a synthesized frame index).
Prints the predicted shot list with video seconds, video seconds per phase and the V1-V10 / V8 checks.

usage: director_replay.py LANDING_CSV [--flight 075302|105003|121138]
       (events of the 2026-10-21 07:53 flight (FLIGHT), of recording flight #2 10:50 (FLIGHT2) and of recording
       flight #3 12:11 (FLIGHT3) are built in; the flight is picked by --flight or by the CSV name:
       20261021_105006_* -> 105003, 20261021_121140_* -> 121138)

Event UTs of the 07:53 flight (logs/wd_20261021_075302.out + the landing CSV); [meas] = measured, [derived] =
interpolated from the log's wall clock between director lines that carry T+ (game) times:
  lift-off UT 189520.5 [meas: director v2 specification 1.1, CSV vs HUD frame]; MAX-Q T+42 [meas: hud line];
  SRB SEP T+59 [meas: staging line = S3 cut T+59]; MECO T+179.2 [derived: wall 464.2 s between T+66 @ 207.4 s and
  T+188 @ 484.3 s; consistent with the T+188 dead-air pause = MECO + 6 s hold + 3 s tail]; insertion burn
  T+398.4-406.0 [derived: 'burn done' wall 605.9 s between T+390 @ 581.7 and T+408 @ 608.9; 7.6 s burn logged];
  payload release T+1869 [derived: wall 774.9 s between T+1846 @ 728.4 and T+1925 @ 888.4]; deorbit node T+21252
  [derived: old S4 pre-roll cut 10 s before the node at T+21242], burn 4.8 s [log] -> T+21249.6-21254.4;
  deorbit planned at T+1959 [log: 'deorbit: window in 7 revolutions']; hand-off to the lander T+21477.5 [derived];
  entry burn, 1 km, h_gnd < 60 m, touchdown: from the CSV [meas].
Rails-warp spans for the synthesized frame index are approximate (log); they only affect the frame count.

FLIGHT2 = recording flight #2 (logs/wd_20261021_105003.out, EDL videos/20261021_112522_orbit_T-WorksFineGF.edl.txt,
landing CSV logs/20261021_105006_orbit_T-WorksFineGF_landing.csv). [meas] = EDL window edges (the editor's own
event UTs): lift-off 103121.48 (P0 start + 10), MECO T+179.68 (P1 end - 5), insertion burn 103512.22-103525.22
(P4 +-10), release 104990.36 (P6 centre), deorbit burn 124360.35-124370.71 (P8 -10 / -5); entry burn, touchdown,
lander start (first CSV row) from the CSV. MAX-Q T+42 [meas: hud line], SRB SEP T+60 [meas: staging = S3 cut T+60].
Deorbit planned at T+2228 [derived: wall 862.1 s between BRIDGE T+1878 @ 760.2 and P7 pause T+2303 @ 883.8].
The replay flies the fix-round-2 behaviour: the release is announced PRE_SEP_LEAD s ahead (orbit_recover REC_FULL),
the first warp after the insertion cutoff starts at cutoff + 12 s (core.WarpGuard).
"""
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kspilot import director as d, showrun as sr  # noqa: E402

T0 = 189520.5
FLIGHT = dict(t0=T0, maxq=T0 + 42.0, srb=T0 + 59.0, meco=T0 + 179.2, orbit=(T0 + 398.4, T0 + 406.0),
              sep=T0 + 1869.0, deorbit=(T0 + 21249.6, T0 + 21254.4), deorbit_plan_at=T0 + 1959.0,
              handoff=T0 + 21477.5, period=1958.0,
              # (start, end, warp rate) of rails warp / P7 capture pause, for the synthesized frame index only
              rails=[(T0 + 193.0, T0 + 338.0, 14.0), (T0 + 420.0, T0 + 840.0, 50.0), (T0 + 880.0, T0 + 1830.0, 50.0),
                     (T0 + 1959.0, T0 + 21180.0, 50.0), (T0 + 21364.0, T0 + 21476.0, 46.0)])


def read_csv(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                rows.append(dict(ut=float(r["ut"]), h=float(r["h"]), e=float(r["e"]), n=float(r["n"]),
                                 vu=float(r["vu"]), thr=float(r["thr_act"]), hg=float(r["h_gnd"])))
            except (KeyError, ValueError):
                pass
    return rows


def descent_events(rows):
    burn = None
    for r in rows:
        if r["thr"] > 0.01 and r["h"] > 30000 and burn is None:
            burn = [r["ut"], None]
        elif burn is not None and burn[1] is None and r["thr"] <= 0.01:
            burn[1] = r["ut"]
    td = next(r["ut"] for r in rows if r["hg"] == r["hg"] and r["hg"] < 1.5) + 0.5    # lander reports ~0.5 s later
    return tuple(burn), td


def replay_descent(rows, entry, td, txs_fn):
    """DescentDirector over the CSV (the lander calls it at 4 Hz wall ~ 0.25 s; the CSV rows are ~0.05 s apart)."""
    dd = sr.DescentDirector()
    marks, cuts, handoff, last, pad_ok = {}, [], None, -1.0, False
    for r in rows:
        ut = r["ut"]
        if ut - last < 0.25:
            continue
        last = ut
        if handoff is None:
            eta = sr.handoff_eta(r["h"], r["vu"])
            if eta is not None and eta <= sr.HANDOFF_LEAD:
                handoff = ut + eta
                marks["HANDOFF"] = handoff
        rng = math.sqrt((r["e"] + 380.0) ** 2 + (r["n"] + 160.0) ** 2 + (r["h"] - 12.0) ** 2)
        burning = entry[0] <= ut < entry[1]
        pre = ut < entry[0] and entry[0] - ut < sr.V2_PRE_S7
        pad_ok = pad_ok or sr.pad_lens(rng)[2]               # as Lander._camera_v2 (latched)
        sid = dd.step(ut, r["h"], r["hg"], r["vu"], rng, burning, pre, ut >= entry[1], handoff, txs_fn(marks),
                      pad_ok=pad_ok)
        if sid == "S8" and "S8" not in marks and ut >= entry[1]:
            marks["S8"] = ut
        if not cuts or cuts[-1][1] != sid:
            cuts.append((ut, sid, r["h"], r["hg"], rng))
    return cuts, marks


T2 = 103121.48
FLIGHT2 = dict(t0=T2, maxq=T2 + 42.0, srb=T2 + 60.0, meco=T2 + 179.68, orbit=(103512.22, 103525.22),
               sep=104990.36, deorbit=(124360.35, 124370.71), deorbit_plan_at=T2 + 2228.0, handoff=124598.39,
               period=1958.0,
               rails=[(T2 + 200.0, T2 + 330.7, 14.0), (103525.22 + 12.0, 104990.36 - 40.0, 50.0),
                      (T2 + 2228.0, 124290.0, 50.0), (124370.71 + 12.0, 124598.39, 46.0)])
# FLIGHT3 = recording flight #3 (logs/wd_20261021_121138.out, EDL videos/20261021_124856_orbit_T-WorksFineGF.edl.txt,
# landing CSV logs/20261021_121140_orbit_T-WorksFineGF_landing.csv). [meas] = EDL window edges: lift-off 103110.38
# (P0 start + 10; log: planned 103110.4), MECO 103289.96 (P1 end - 5), insertion burn 103498.88-103512.02 (P4 +-10;
# QA: THR 100 first at T+06:28, tail to T+06:41), release 104989.0 (P6 centre = the HUD event UT) announced for
# 104987.6 [log 'payload release planned'], deorbit burn 124350.58-124360.86 (P8 -10 / -5); deorbit node 124368.4
# [meas: HUD node 'T+ 05:54:18' in screenshots/qa_v3_03, 7.5 s after the cutoff - so S4D starts at the ignition, as
# the log shows: S4D at T+21241]; lander start 124587.42 (first CSV row; director hand-off [log]).
# MAX-Q T+42 / SRB SEP T+60 [log: S3 cut T+60]; deorbit planned at ~T+15700 [derived: the 'deorbit plan' line after
# the 13457 s hibernating warp]. HUD ORBIT event T+401.0 [derived: QA 'tail to T+06:41'; the director's own thrust
# end T+401.6 comes a tick later] - with it the replay reproduces the flown S4 of ~17 s on the fix-round-2 code
# (the ORBIT icon window then holds S4 to T+406, log: S4b at T+406). Rails spans: approximate (frame count only).
T3 = 103110.38
FLIGHT3 = dict(t0=T3, maxq=T3 + 42.0, srb=T3 + 60.0, meco=103289.96, orbit=(103498.88, 103512.02),
               orbit_event=T3 + 401.0,
               sep=104989.0, sep_plan=104987.6, deorbit=(124350.58, 124360.86), deorbit_node=124368.4,
               deorbit_plan_at=T3 + 15700.0, handoff=124587.42, period=1958.0,
               rails=[(T3 + 190.0, T3 + 375.0, 14.0), (103512.02 + 12.0, 104987.6 - 40.0, 50.0),
                      (T3 + 2300.0, 124290.0, 50.0), (124360.86 + 12.0, 124587.42, 46.0)])
FLIGHTS = {"075302": FLIGHT, "105003": FLIGHT2, "121138": FLIGHT3}


def mission_state(ut, F, director_state):
    """Synthetic v2 director State at ut for the 07:53 flight (events as the flight code reported them)."""
    met = ut - F["t0"] if ut >= F["t0"] else None
    ev = {}
    if met is not None:
        ev["LIFTOFF"] = (F["t0"], True)
        ev["MAX-Q"] = (F["maxq"], ut >= F["maxq"] + 3.0)
        ev["SRB SEP"] = (F["srb"], ut >= F["srb"])
        ev["MECO"] = (F["meco"], ut >= F["meco"])
    if ut >= F["meco"]:
        oe = F.get("orbit_event", F["orbit"][1])               # the HUD ORBIT event (reported at the cutoff)
        ev["ORBIT"] = ((oe, True) if ut >= oe else (0.5 * sum(F["orbit"]), False))
    plan = F.get("sep_plan", F["sep"])                          # the announced UT (the HUD logs the event later)
    if ut >= F["sep"]:
        ev["PAYLOAD SEP"] = (F["sep"], True)
    elif ut >= plan - sr.PRE_SEP_LEAD:                          # fix round 2: the release is announced (REC_FULL)
        ev["PAYLOAD SEP"] = (plan, False)
    node = F.get("deorbit_node", 0.5 * sum(F["deorbit"]))
    if ut >= F["deorbit_plan_at"]:
        ev["DEORBIT BURN"] = (node, ut >= F["deorbit"][1] + 3.0)
    burns = [F["orbit"], F["deorbit"]]
    burning = any(a + 1.0 < ut < b for a, b in burns)
    ended = [b for a, b in burns if b <= ut]
    burn_end = max(ended) if ended else None
    rails = any(a <= ut < b for a, b, _ in F["rails"])
    over = ut >= F["meco"]
    srb = 4 if ut < F["srb"] else 0
    guard = any(a - d.GUARD_S < ut < b + 30.0 for a, b in burns) or ut < F["sep"] + 100.0 and ut > F["sep"]
    start = F["sep"] + sr.SEP_WIN
    p7 = ut > start + sr.BRIDGE_S * sr.BRIDGE_VIDEO + 20.0 and not (node - ut < d.GUARD_S)
    started = [a for a, b in burns if a <= ut < b]
    return d.State(ut, met, ev, 0.0 if met is None else min(2.5 * met * met, 1e5), burning, rails, srb, over, 1e4,
                   30.0, 90.0, guard, burn_end, p7, started[0] if started else None)


def replay_ascent_orbit(F, txs_fn):
    c = d.CutterV2()
    cuts, pauses = [], []
    ut = F["t0"] - sr.REC_LEAD
    while ut < F["handoff"]:
        s = mission_state(ut, F, None)
        for a in c.step(s, txs_fn(s.ev)):
            if a[0] == "cut":
                cuts.append((ut, a[2]))
            else:
                pauses.append((ut, a[0]))
        # coarse steps where nothing happens (the 5 h coast): 0.25 s near events, 5 s on rails far from them
        near = s.met is None or s.met < 200 or abs(ut - F["sep"]) < 60 or not s.rails or c.paused is False and s.p7
        ut += 0.25 if near or any(abs(ut - x) < 200 for x in (F["orbit"][0], F["deorbit"][0])) else 5.0
    return cuts, pauses


def synth_index(F, rec0, rec1, pauses):
    """Frame UTs the plugin would record: 1/30 s per frame at 1x and at physics warp (B1), rate/30 s on rails,
    nothing while paused (P7)."""
    spans, p0 = [], None
    for u, a in pauses:
        if a == "pause":
            p0 = u
        elif p0 is not None:
            spans.append((p0, u))
            p0 = None
    uts, u = [], rec0
    rails = sorted(F["rails"])
    while u < rec1:
        if any(a <= u < b for a, b in spans):
            u = next(b for a, b in spans if a <= u < b)
            continue
        uts.append(u)
        r = next((w for a, b, w in rails if a <= u < b), 1.0)
        u += r / 30.0
    return uts


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    base = os.path.basename(argv[0])
    key = argv[argv.index("--flight") + 1] if "--flight" in argv else (
        "105003" if "20261021_105006" in base else "121138" if "20261021_121140" in base else "075302")
    F = dict(FLIGHTS[key])
    T0 = F["t0"]
    print("== flight %s (lift-off UT %.2f) ==" % (key, T0))
    rows = read_csv(argv[0])
    entry, td = descent_events(rows)
    ev_all = {"LIFTOFF": (F["t0"], True), "ORBIT": (F["orbit"][1], True), "PAYLOAD SEP": (F["sep"], True),
              "ENTRY BURN": (entry[0], True), "TOUCHDOWN": (td, True)}

    def txs_desc(marks):
        return sr.transitions(ev_all, marks, v2=True)

    def txs_asc(ev):
        return sr.transitions(ev, None, v2=True)

    a_cuts, pauses = replay_ascent_orbit(F, txs_asc)
    d_cuts, marks = replay_descent(rows, entry, td, txs_desc)
    rec0, rec1 = F["t0"] - sr.REC_LEAD, td + sr.TD_TAIL
    m = dict(rec0=rec0, rec1=rec1, cd=F["t0"], t0=F["t0"], meco=F["meco"], orbit=F["orbit"], sep=F["sep"],
             deorbit=F["deorbit"], entry=entry[0], td=td, lander=F["handoff"], burns=[F["orbit"], F["deorbit"]])
    if F.get("sep_plan") is not None:                     # fix round 3 (V4): the announced release UT
        m["sep_plan"] = F["sep_plan"]
    plan = sr.playback_plan(m)
    p6 = next((s for s in plan if "P6" in s.phase and s.kind == "real"), None)
    if p6 is not None:
        print("== V4: 1x from %.1f s before the earliest release UT (HUD %.1f, planned %s) to %.1f s after ==" % (
            sr.sep_start(m) - p6.ut0, F["sep"], F.get("sep_plan"), p6.ut1 - F["sep"]))
    txs = [x for x in sr.transitions(ev_all, marks, v2=True)]
    problems = sr.check_plan(plan, m, txs)
    uts = synth_index(F, rec0, rec1, pauses)
    out = sr.select_frames(uts, plan)
    problems += sr.check_frames(out, uts, plan)

    print("== playback plan (EDL) ==")
    for s in plan:
        print("  %-8s %-5s T%+10.1f .. T%+10.1f  game %8.1f s  S %7s  video %6.1f s" % (
            s.phase, s.kind, s.ut0 - T0, s.ut1 - T0, s.ut1 - s.ut0, "-" if s.speed is None else "%.1f" % s.speed,
            s.video))
    summ = sr.plan_summary(plan)
    tot = sum(summ.values())
    real = sum(s.video for s in plan if s.kind == "real")
    print("== video seconds per phase ==")
    for k, v in summ.items():
        print("  %-8s %6.1f" % (k, v))
    print("  TOTAL    %6.1f s = %d:%04.1f   (1x %.1f s = %.0f %%; spec target ~8:30 = 510 s)" % (
        tot, int(tot) // 60, tot % 60, real, 100 * real / tot))
    print("  video frames %d, recorded frames (synthesized index) %d" % (len(out), len(uts)))

    cuts = [(u, s) for u, s in a_cuts] + [(u, s) for u, s, _, _, _ in d_cuts]
    shots = sr.shot_durations(plan, cuts, rec1)
    print("== predicted shot list (video seconds) ==")
    viol = []
    for sid, u0, u1, v in shots:
        typ = sr.SHOT_TYPE.get(sid, "?")
        hmax = sr.MAX_HOLD.get(typ, 1e9)
        flag = ""
        if v < sr.MIN_SHOT - 1e-6 and v > 0:
            flag = "  < MIN %.0f s" % sr.MIN_SHOT
        burn_cover = sid == "S7"                      # the entry-burn close-up covers its whole burn (research row 5)
        if v > hmax + (20.0 if burn_cover else 0.0):
            flag = "  > MAX %.0f s (%s)" % (hmax, typ)
        if flag:
            viol.append((sid, u0 - T0, v, flag))
        print("  T%+10.1f  %-6s %-8s %6.1f s%s" % (u0 - T0, sid, typ, v, flag))
    for u, s, h, hg, rng in d_cuts:
        if s == "S10":
            print("  S10 cut at h_gnd %.1f m (h %.1f m), %.1f s before touchdown" % (hg, h, td - u))
        if s == "D2":
            f, fov, _ = sr.pad_lens(rng)
            print("  D2 pad lens at range %.1f km: fov %.2f deg, stage %.0f %% of the picture height" % (
                rng / 1e3, max(fov, sr.PAD_LENS_MIN_FOV), 100 * sr.stage_fill(rng, max(fov, sr.PAD_LENS_MIN_FOV))))
    print("== P7 capture pauses (disk saving) ==", [(round(u - T0, 1), a) for u, a in pauses])
    print("== checks ==")
    print("  plan / frames:", "PASS" if not problems else problems[:10])
    print("  shot lengths:", "PASS" if not viol else viol)
    return 0 if not problems and not viol else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
