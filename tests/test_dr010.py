"""DR-010 (decision log 2026-10-21 14:35 MISHAP / 14:55 DR-010), offline only (no kRPC):
  1  entry-burn dv from propellant mass, all modes: robust to ticks whose thrust sample reads 0 (DEADMAN cut)
  2  DEADMAN on game time with a 3x wall-clock cap (REC_FULL only); non-recording keeps the 2.5 s wall rule
  3  RPCs per lander-loop tick under REC_FULL, counted on the real Lander.run with a fake kRPC (tests/fake_krpc.py)
and the non-recording guarantee (no REC-only path taken without rec_v2)."""
import math
import os
import sys
import types

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))
from kspilot import core, flightctl, landing  # noqa: E402
import fake_krpc as fk  # noqa: E402

G0 = 9.80665


def bare_lander(**kw):
    """A Lander without kRPC (only what the accounting / DEADMAN helpers touch)."""
    L = landing.Lander.__new__(landing.Lander)
    L.log = lambda *a: L.__dict__.setdefault("lines", []).append(" ".join(map(str, a)))
    L.__dict__.update(kw)
    return L


# ------------------------------------------------------------------------- 1. dv accounting from propellant mass
def synthetic_burn(dts, zero, isp=319.99, F=649971.0, m0=20308.0):
    """Ticks of length dts (game s) at full thrust; the thrust SAMPLE of the ticks in `zero` reads 0 although the
    engine burned through the whole tick (flight #4: the DEADMAN cut lands right before the sample). Returns
    (true dv, mass per tick, thrust samples)."""
    mdot = F / (isp * G0)
    ms, samples, m = [m0], [], m0
    for i, dt in enumerate(dts):
        m -= mdot * dt
        ms.append(m)
        samples.append(0.0 if i in zero else F)
    return isp * G0 * math.log(m0 / m), ms, samples


def test_mass_accounting_survives_zero_thrust_samples():
    """Synthetic REC_FULL entry burn: 0.1-1.2 s ticks with three 1-s gaps whose thrust sample reads 0. The new
    accounting must match the true dv within 1 % (it is exact); the old thrust-sample x tick sum loses the gaps."""
    dts = [0.1, 0.5, 1.2, 0.8, 1.0, 0.3, 1.0, 0.1, 0.7, 1.0, 0.5, 1.1] * 2
    zero = {4, 6, 9}                              # the 1-s gaps: sample 0, engine burned all along
    true, ms, samples = synthetic_burn(dts, zero)
    L = bare_lander(thrust=lambda: 649971.0)
    L.v = types.SimpleNamespace(specific_impulse=319.99)
    L._eb_start(ms[0])
    new = old = 0.0
    for i, dt in enumerate(dts):
        new += L._dv_mass(ms[i], ms[i + 1], dt)
        old += samples[i] / ms[i + 1] * dt
    assert abs(new - true) / true < 0.01, (new, true)
    assert abs(new - true) < 1e-6 * true           # the ln telescopes: exact for a constant Isp
    assert (true - old) / true > 0.08, (old, true)   # the flaw this replaces: ~3 lost 1-s ticks = ~100 m/s
    assert not getattr(L, "_eb_capped", False)


def test_mass_accounting_ignores_a_lost_part():
    """A part falling off is mass, not dv: one tick's count is capped at 1.25 x the engine's dv (+0.5 m/s)."""
    L = bare_lander(thrust=lambda: 649971.0)
    L.v = types.SimpleNamespace(specific_impulse=319.99)
    L._eb_start(20000.0)
    dv = L._dv_mass(20000.0, 20000.0 - 21.0 - 300.0, 0.1)      # 21 kg of propellant + a 300 kg part
    assert dv <= 1.25 * 649971.0 / 20000.0 * 0.1 + 0.5 + 1e-9
    assert L._eb_capped and any("part lost" in s for s in L.lines)
    assert L._dv_mass(20000.0, 20000.0, 0.1) == 0.0 and L._dv_mass(None, 19000.0, 0.1) == 0.0


def test_isp_fallbacks():
    L = bare_lander()
    L.v = types.SimpleNamespace(specific_impulse=0.0, parts=types.SimpleNamespace(
        engines=[types.SimpleNamespace(active=True, vacuum_specific_impulse=320.0)]))
    assert L._isp_now() == 320.0                   # vessel Isp unreadable (0) -> the active engines' vacuum Isp
    L.v = types.SimpleNamespace(specific_impulse=0.0, parts=types.SimpleNamespace(engines=[]))
    assert L._isp_now() == 300.0


def test_loop_accounting_end_to_end(tmp_path):
    """The real Lander.run (fake kRPC, engine burning propellant): with stalls (1 s extra game time) whose thrust
    samples read 0, entry_flown is the propellant dv (within 1 %) in both modes and the burn stops at entry_dv;
    the logged thrust-sample sum shows what the old accounting would have lost."""
    for rec in (False, True):
        # non-recording kRPC serves many RPCs per frame: ~0.1 s ticks there (1 ms per RPC); REC_FULL 1 per frame
        out = fk.run_lander(landing, core, tmp_path, rec, ticks=1000, entry_dv=590.0, burn=True,
                            frame_s=None if rec else 0.001, gaps={5: 1.0, 9: 1.0, 14: 1.0}, zero_thrust={6, 10, 15})
        L, st = out["lander"], out["world"]
        assert not out["missing"], out["missing"]
        assert L.entry_done, "burn never ended"
        m_end = L._eb_m0 - [float(x.split("propellant ")[1].split(" kg")[0]) for x in out["log"]
                            if x.startswith("ENTRY BURN end")][0]
        true = fk.ISP * G0 * math.log(L._eb_m0 / m_end)
        assert abs(L.entry_flown - true) / true < 0.01, (rec, L.entry_flown, true)
        tick = max(g for g in out["game"] if g < 0.9)              # longest normal (non-stalled) tick
        assert 590.0 <= L.entry_flown < 590.0 + tick * 649971.0 / m_end + 1.0, (rec, L.entry_flown, tick)
        # the old thrust-sample sum is off by > 50 m/s: 3 lost 1-s ticks (~100 m/s short); with REC_FULL also the
        # long first tick of the fake (one-time setup reads) sampled at full thrust after the throttle-up (long)
        assert abs(L._eb_thr_sum - L.entry_flown) > 50.0, (rec, L._eb_thr_sum, L.entry_flown)
        if not rec:
            assert L._eb_thr_sum < L.entry_flown - 60.0, L._eb_thr_sum


# --------------------------------------------------------------------------------- 2. DEADMAN game time (REC_FULL)
class Clock:
    def __init__(self):
        self.wall, self.game_rate, self.game = 0.0, 0.4, 1000.0

    def advance(self, d):
        self.wall += d
        self.game += self.game_rate * d


def deadman_run(rec, beat_every, t_end=12.0, game_rate=0.4):
    """Run the real _deadman loop on a fake clock: the control loop beats every `beat_every` wall s (None: never
    after t=0). Returns the wall time of the throttle cut (None: no cut)."""
    clk = Clock()
    clk.game_rate = game_rate
    ctl = types.SimpleNamespace(throttle=1.0)
    conn = types.SimpleNamespace(space_center=types.SimpleNamespace(active_vessel=types.SimpleNamespace(control=ctl)),
                                 close=lambda: None)
    L = bare_lander(rec_v2=rec, _stop=False, _hb_hold=False)
    L.ut = lambda: clk.game
    L._gc = core.GameClock(L.ut, wall=lambda: clk.wall)
    cut = {}
    fake_time = types.SimpleNamespace(time=lambda: clk.wall)

    def sleep(d):
        clk.advance(d)
        if ctl.throttle == 0.0 and "t" not in cut:
            cut["t"] = clk.wall
        if beat_every and clk.wall - L._hb >= beat_every - 1e-9:
            L._hb_beat()
        if clk.wall >= t_end:
            L._stop = True
    fake_time.sleep = sleep
    old_time, old_connect = landing.time, core.connect
    landing.time, core.connect = fake_time, lambda name="": conn
    try:
        L._hb = clk.wall
        L._hb_mark = L._gclock().mark() if rec else None
        L._deadman()
    finally:
        landing.time, core.connect = old_time, old_connect
    return cut.get("t")


def test_deadman_nonrec_unchanged_wall_clock():
    assert abs(deadman_run(False, None) - 2.75) < 0.26          # 2.5 s wall (0.25 s polling)
    assert deadman_run(False, 2.8) is not None                  # a 2.8 s wall tick still trips it (as before)
    assert deadman_run(False, 2.0) is None


def test_deadman_rec_game_time_with_wall_cap():
    assert deadman_run(True, 2.8) is None                       # flight #4: 2.8 s wall = 1.1 game s, not a stall
    assert deadman_run(True, 3.2) is None
    t = deadman_run(True, None)                                 # a real stall: 2.5 GAME s = 6.25 wall s at 0.4x
    assert 6.25 <= t <= 6.6, t
    t = deadman_run(True, None, game_rate=0.0)                  # game paused / UT frozen: the 3 x 2.5 s wall cap
    assert 7.5 <= t <= 8.1, t
    t = deadman_run(True, None, game_rate=1.0)                  # real time: same as the wall rule
    assert 2.5 <= t <= 3.1, t                                  # (0.25 s polling + 1 poll to see it)


def test_deadman_source_uses_the_shared_rule():
    import inspect
    src = inspect.getsource(landing.Lander._deadman)
    assert "self._hb_age() > self.DEADMAN_S" in src and landing.Lander.DEADMAN_S == 2.5


# ------------------------------------------------------------------------ 3. RPCs per tick (REC_FULL) + guarantee
def test_rpc_budget_per_tick_rec_full(tmp_path):
    """REC_FULL entry burn, 1 RPC per game frame (FRAME_S = 1/30 s): before DR-010 a tick cost ~100 RPCs (fin
    hold scan, control reads, warp, flight roll, parts, predictor vessel queries) = ~3.3 game s; now the bare tick
    is the two attitude writes (target direction, roll) and the 2 s fin hold adds 4 has_event (+ triggers)."""
    out = fk.run_lander(landing, core, tmp_path, rec=True, ticks=60)
    assert not out["missing"], out["missing"]
    c, g = out["counts"][2:], out["game"][2:]                   # skip the first ticks (one-time setup reads)
    assert max(c) <= 12, c
    assert sorted(c)[len(c) // 2] <= 3, c
    assert sum(g) / len(g) <= 0.3 and max(g) <= 0.45, g        # target: tick <= 0.3 game s at fps 12
    L = out["lander"]
    assert L._tel is not None and L.entry_burn
    busiest = max(out["names"][2:], key=lambda d: sum(d.values()))
    assert "vessel.control" not in busiest and "parts.all" not in busiest, busiest


def test_rpc_nonrec_takes_no_rec_path(tmp_path):
    """Without REC_FULL nothing of item 3 runs: no streams beyond the old ones, no cached Control on the actuator,
    per-tick RPCs as before (the old loop: ~100 per entry-burn tick in this fake)."""
    out = fk.run_lander(landing, core, tmp_path, rec=False, ticks=12)
    assert not out["missing"], out["missing"]
    L = out["lander"]
    assert L._tel is None and getattr(L, "_hb_mark", None) is None and not L._hb_game()
    assert min(out["counts"][1:]) >= 90, out["counts"]
    assert out["seq"].count("vessel.specific_impulse") == 1     # item 1's one read at burn start (all modes)


def test_set_thrust_nonrec_identical_and_rec_skips_unchanged():
    """flightctl.set_thrust: without act.ctl (all non-REC callers) it writes vessel.control.throttle every call as
    before; with REC_FULL's cached Control it writes only when the game's throttle (stream) differs, so a DEADMAN
    cut on the other connection is always re-commanded."""
    fk.CTR = fk.Counter()
    ctl = fk.Remote("control", throttle=0.0)
    v = fk.Remote("vessel", control=ctl)
    ap = fk.Remote("ap", target_direction=None)
    frame = types.SimpleNamespace(to_body=lambda x: np.asarray(x))
    act = types.SimpleNamespace(v=v, ap=ap, frame=frame, nose=lambda: (0.0, 0.0, 1.0))
    for _ in range(3):
        flightctl.set_thrust(act, 1.0, (0.0, 0.0, 1.0))
    assert fk.CTR.names.get("vessel.control") == 3 and fk.CTR.names.get("control.throttle=") == 3
    fk.CTR = fk.Counter()
    act.ctl, act.thr_s = ctl, lambda: ctl._peek("throttle")
    for _ in range(3):
        flightctl.set_thrust(act, 1.0, (0.0, 0.0, 1.0))
    assert fk.CTR.names.get("control.throttle=", 0) == 0 and act.thr_cmd == 1.0    # already 1.0 in the game
    object.__getattribute__(ctl, "_a")["throttle"] = 0.0                              # DEADMAN cut elsewhere
    flightctl.set_thrust(act, 1.0, (0.0, 0.0, 1.0))
    assert fk.CTR.names.get("control.throttle=") == 1 and "vessel.control" not in fk.CTR.names
