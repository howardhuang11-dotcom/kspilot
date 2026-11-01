"""Feasibility gate: recovery budgeted on the core after payload release (no KSP needed)."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
from kspilot import feasibility as fz  # noqa: E402
import payload_gate as pg  # noqa: E402


class Kerbin:
    gravitational_parameter = 3.5316e12
    equatorial_radius = 600000.0


def test_calibrated_constants_kept():
    assert fz.REC_MARGIN == 1.08 and fz.LANDING_FIXED == 60.0
    # the logged recovery requirement of the 5-mission streak: 100 km circular, v_t 143 m/s, entry burn 590 m/s
    need = fz.recovery_dv(Kerbin, 100e3, 100e3, 143.0, 590.0)
    assert abs(need - 1064) < 3, need


def test_no_payload_reduces_to_old_sum():
    st = [dict(dv_asl=900.0, dv_vac=1050.0, m0=90e3, m1=60e3),
          dict(dv_asl=3600.0, dv_vac=4200.0, m0=55e3, m1=12e3)]
    b = fz.budget(st, 3500.0, core_dry=12e3, need_rec=1000.0)
    assert b['payload'] == 0.0
    assert abs(b['margin'] - (900 + 4200 - 3500 - 1000)) < 1e-6
    assert abs(b['margin_rec'] - b['margin']) < 1e-6     # same propellant, same mass: both views agree


def test_no_recovery_is_plain_ascent_margin():
    st = [dict(dv_asl=900.0, dv_vac=1050.0, m0=90e3, m1=60e3), dict(dv_asl=3600.0, dv_vac=4200.0, m0=55e3, m1=12e3)]
    assert fz.budget(st, 3500.0)['margin'] == 900 + 4200 - 3500


def test_recovery_propellant_on_core_only():
    st = [dict(dv_asl=3600.0, dv_vac=4200.0, m0=60e3, m1=16e3)]   # lone stage: counted at sea level
    b = fz.budget(st, 2000.0, core_dry=12e3, need_rec=1000.0)
    ve = 4200.0 / math.log(60 / 16)
    assert abs(b['payload'] - 4e3) < 1e-6
    assert abs(b['prop_rec'] - 12e3 * (math.exp(1000.0 / ve) - 1)) < 1e-6
    # the reserve is carried through the ascent together with the payload
    assert abs(b['avail_ascent'] - 3600.0 / 4200.0 * ve * math.log(60 / (16 + b['prop_rec'] / 1e3))) < 1e-6


def test_insufficient_recovery_is_no_go():
    st = [dict(dv_asl=900.0, dv_vac=1050.0, m0=90e3, m1=60e3), dict(dv_asl=3600.0, dv_vac=4200.0, m0=55e3, m1=14e3)]
    assert fz.budget(st, 3500.0, core_dry=11e3, need_rec=2500.0)['margin'] < 0
    assert fz.budget(st, 3500.0, core_dry=11e3, need_rec=500.0)['margin'] > 0


def test_flown_baseline_unchanged_in_sign_and_close_to_flight():
    r = pg.gate(pg.P0)
    assert abs(r['old'] - 421) < 10, r['old']          # the gate the 5-mission streak flew with printed +421
    b = r['new']
    assert b['margin'] > 0 and b['margin_rec'] > 0
    # flown: 370-377 LF left after landing (5 missions); the gate's expectation must be close and not optimistic
    left_lf = b['prop_spare'] / 1e3 / fz.LFO_T_PER_LF
    assert 300 < left_lf <= 377, left_lf


def test_heavy_payloads_go_and_monotonic():
    r45, r60 = pg.gate(4500.0), pg.gate(6000.0)
    assert r60['old'] < 0                                # the old gate refused 6 t ...
    assert r45['new']['margin'] > 0 and r60['new']['margin'] > 0   # ... the core-only recovery budget does not
    assert pg.gate(pg.P0)['new']['margin'] > r45['new']['margin'] > r60['new']['margin']
    # the recovery requirement does not depend on the payload (it is released before the deorbit burn)
    assert abs(r45['new']['prop_rec'] / r60['new']['prop_rec'] - 1) < 0.005


def _fake_vessel(payload):
    """kRPC-shaped stand-in for T-WorksFineGF with `payload` kg above the decoupler (stage data as pg models them)."""
    from types import SimpleNamespace as NS
    body = NS(gravitational_parameter=3.5316e12, equatorial_radius=600000.0, rotational_speed=2 * math.pi / 21549.425,
              surface_gravity=9.81, density_at=lambda alt: 1.136)   # -> the logged v_t 143 m/s
    stages = []
    for s in pg.stages_for(payload):
        twr_asl = (1568.75e3 if s['n'] == 3 else 568.75e3) / (s['m0'] * 9.81)
        stages.append(NS(number=s['n'], delta_v=s['dv_vac'], vacuum_delta_v=s['dv_vac'], sea_level_delta_v=s['dv_asl'],
                         sea_level_twr=twr_asl, vacuum_twr=s['twr_vac'], burn_time=s['burn'], start_mass=s['m0'],
                         end_mass=s['m1']))
    cmd = NS(name='ModuleCommand')
    probe = NS(modules=[cmd], decouple_stage=-1, mass=pg.CORE_DRY, dry_mass=pg.CORE_DRY)
    eng_part = NS(modules=[], decouple_stage=-1, mass=0.0, dry_mass=0.0)
    engine = NS(part=eng_part, gimballed=True, throttle_locked=False, max_thrust=650e3,
                max_thrust_at=lambda p: 568.75e3)
    legs = [NS(part=NS(decouple_stage=-1)) for _ in range(4)]
    pay = NS(modules=[], decouple_stage=1, mass=payload, dry_mass=payload)
    parts = NS(all=[probe, eng_part, pay], engines=[engine], legs=legs, reaction_wheels=[], control_surfaces=[])
    return NS(orbit=NS(body=body), parts=parts, stages=stages)


def test_analyse_gate_with_fake_vessel():
    real = fz.estimate_cda
    fz.estimate_cda = lambda conn, v: 9.59              # as the gate measured (terminal speed ~143 m/s)
    try:
        res = {}
        for p in (pg.P0, 4500.0, 6000.0):
            ok, rep, lines = fz.analyse(None, _fake_vessel(p), 100e3, 100e3, 6.0, True, entry_burn_dv=590.0)
            res[p] = (ok, rep)
            assert ok, (p, lines)
            assert abs(rep['need_recovery'] - 1064) < 5, rep['need_recovery']
            assert abs(rep['need_ascent'] - pg.gate(p)['need_asc']) < 15, rep['need_ascent']
            assert abs(rep['budget']['payload'] - p) < 1e-6
        assert abs(res[pg.P0][1]['margin'] - pg.gate(pg.P0)['new']['margin']) < 20
    finally:
        fz.estimate_cda = real


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
