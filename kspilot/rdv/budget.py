"""Propellant budget of the docking sortie for the recoverable core (R0).

Sortie (settled requirement, docking concept study): the core carries the cargo module to a
station in a 100 km circular orbit, the stack docks (port on the cargo nose), the cargo decouples and stays,
the core backs away, deorbits and lands as today.

Everything is anchored on FLOWN data (logs/wd_20261021_*.out, T-WorksFineGF, own ascent + recovery):
`flying core: ... fuel N` is the core's LiquidFuel units right after the payload release (orbit insertion
state) and `RESULT ... fuel_left=N` the units left on the pad. 1 LF unit = 11.11 kg of LF+Ox (0.9:1.1 mix,
kspilot.feasibility.LFO_T_PER_LF). Core dry mass 11.4 t (feasibility line "recovery stage: dry 11.4t").
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..feasibility import LFO_T_PER_LF
from .orbit import MU, R_KERBIN, hohmann

G0 = 9.80665
ISP_SKIPPER_VAC = 320.0      # s   [cfg: skipperLiquidEngine_v2.cfg key 0 320]
ISP_RCS_VAC = 240.0          # s   [cfg: rcsBlockRV-105.cfg key 0 240]
MONO_T_PER_UNIT = 0.004      # t   [cfg: ResourcesGeneric MonoPropellant density 0.004]
CORE_DRY_T = 11.4            # t   [log: feasibility "dry 11.4t"]
KG_PER_LF = LFO_T_PER_LF * 1000.0

# (log, orbit km, inc deg, payload t, LF at release, LF on the pad) -- only RESULT PASS flights
FLOWN = [
    ("wd_20261021_011054", 100, 6, 2.48, 917, 375),
    ("wd_20261021_022014", 100, 6, 2.48, 917, 377),
    ("wd_20261021_013138", 100, 6, 2.48, 912, 371),
    ("wd_20261021_020157", 100, 6, 2.48, 917, 375),
    ("wd_20261021_023831", 100, 6, 2.48, 918, 375),
    ("wd_20261021_030422", 100, 6, 2.48, 918, 379),
    ("wd_20261021_035335", 100, 6, 2.48, 914, 372),
    ("wd_20261021_061840", 100, 6, 2.48, 901, 364),
    ("wd_20261021_075302", 100, 6, 2.48, 898, 367),
    ("wd_20261021_105003", 100, 6, 2.48, 898, 354),
    ("wd_20261021_121138", 100, 6, 2.48, 898, 347),
    ("wd_20261021_045413", 100, 0, 2.48, 916, 375),
    ("wd_20261021_052801", 80, 6, 2.48, 943, 389),
    ("wd_20261021_043543", 110, 6, 2.48, 901, 367),
    ("wd_20261021_065601", 100, 6, 4.50, 774, 279),
    ("wd_20261021_073413", 100, 6, 6.00, 688, 228),
]


def _t(lf):
    return lf * LFO_T_PER_LF


def recovery_dv_eff(lf0, lf1, dry=CORE_DRY_T, isp=ISP_SKIPPER_VAC):
    """Effective (vacuum-Isp equivalent) recovery delta-v implied by a flown deorbit -> landing."""
    return isp * G0 * math.log((dry + _t(lf0)) / (dry + _t(lf1)))


def flown_table():
    rows = []
    for log, alt, inc, pay, lf0, lf1 in FLOWN:
        rows.append(dict(log=log, alt=alt, inc=inc, payload=pay, lf0=lf0, lf1=lf1,
                         used=lf0 - lf1, dv_eff=recovery_dv_eff(lf0, lf1)))
    return rows


def baseline_stats():
    """Statistics of the reference mission (100 km, 6 deg, 2.48 t payload) and the derived sensitivities."""
    tab = flown_table()
    ref = [r for r in tab if r['alt'] == 100 and r['inc'] == 6 and r['payload'] == 2.48]
    lf0 = np.array([r['lf0'] for r in ref], float)
    lf1 = np.array([r['lf1'] for r in ref], float)
    dv = np.array([r['dv_eff'] for r in ref])
    heavy = [r for r in tab if r['payload'] > 2.5]
    # LF at insertion lost per tonne of extra mass carried to orbit (measured with 4.5 t and 6.0 t payloads)
    slope = float(np.mean([(lf0.mean() - r['lf0']) / (r['payload'] - 2.48) for r in heavy]))
    p80 = [r for r in tab if r['alt'] == 80][0]
    return dict(n=len(ref), lf0_mean=lf0.mean(), lf0_min=lf0.min(), lf0_max=lf0.max(),
                lf1_mean=lf1.mean(), lf1_min=lf1.min(), lf1_max=lf1.max(),
                dv_eff_mean=dv.mean(), dv_eff_max=dv.max(), dv_eff_min=dv.min(), dv_eff_std=dv.std(ddof=1),
                lf_per_t_to_orbit=slope, lf0_80km_gain=p80['lf0'] - lf0.mean())


def predict_landed_lf(lf0, dry=CORE_DRY_T, dv_eff=None, isp=ISP_SKIPPER_VAC):
    """LF units left on the pad for a core starting the deorbit with lf0 units (rocket equation on dv_eff)."""
    if dv_eff is None:
        dv_eff = baseline_stats()['dv_eff_mean']
    m1 = (dry + _t(lf0)) / math.exp(dv_eff / (isp * G0))
    return (m1 - dry) / LFO_T_PER_LF


def burn_lf(mass_t, dv, isp=ISP_SKIPPER_VAC):
    """LF units for a main-engine burn of dv on a stack of mass_t (t)."""
    return mass_t * (1 - math.exp(-dv / (isp * G0))) / LFO_T_PER_LF


def mono_kg(mass_t, dv, isp=ISP_RCS_VAC, eta=0.9):
    return 1000.0 * mass_t * (1 - math.exp(-dv / (isp * G0 * eta)))


@dataclass
class Kit:
    """RCS kit added to the core (stays on the core, flies home). [ASSUMPTION: proposed configuration]"""
    rcs_blocks: int = 4                  # RV-105, one ring at the stack CoM (4 kN axial, 2 kN lateral per axis)
    rcs_block_t: float = 0.04            # [cfg]
    tanks: int = 2                       # radial FL-R1 long tanks, symmetric (50 units each) [cfg]
    tank_dry_t: float = 0.03             # [cfg rcsTankRadialLong]
    tank_units: int = 50
    mono_load_units: float = 60.0        # loaded (partial fill allowed in the VAB)

    @property
    def dry_t(self):
        return self.rcs_blocks * self.rcs_block_t + self.tanks * self.tank_dry_t

    @property
    def mono_t(self):
        return self.mono_load_units * MONO_T_PER_UNIT


@dataclass
class Sortie:
    kit: Kit = field(default_factory=Kit)
    payload_t: float = 2.48              # cargo module (stays at the station) incl. its docking port
    parking_alt: float = 80e3
    station_alt: float = 100e3
    rdv_corr_dv: float = 10.0            # m/s main engine: plane trim (0.1 deg = 3.9 m/s) + mid-course [ASSUMPTION]
    terminal_mono_kg: float = 80.0       # from the docking MC (p99 of the nominal campaign)
    backaway_dv: float = 1.0             # m/s RCS after the cargo decouples [ASSUMPTION]
    credit_80km: bool = True             # credit the cheaper ascent to 80 km (measured, one flight)


def sortie_budget(s: Sortie, dv_eff=None, lf0_base=None):
    """Walk the sortie. Returns dict with every step in LF units / kg and the landed LF."""
    st = baseline_stats()
    lf = st['lf0_mean'] if lf0_base is None else lf0_base
    steps = [("insertion LF (flown, 100 km, 2.48 t)", lf)]
    if s.credit_80km:
        # measured gain is one flight; cap it by the Hohmann the 100 km ascent no longer flies
        m_orbit = CORE_DRY_T + _t(lf) + s.payload_t
        theory = burn_lf(m_orbit, sum(abs(x) for x in hohmann(R_KERBIN + s.parking_alt, R_KERBIN + s.station_alt)[:2]))
        gain = min(st['lf0_80km_gain'], theory)
        lf += gain
        steps.append(("ascent to 80 km parking instead of 100 km", gain))
    kit_to_orbit = s.kit.dry_t + s.kit.mono_t
    pen = st['lf_per_t_to_orbit'] * kit_to_orbit
    lf -= pen
    steps.append(("kit %.2f t carried to orbit (%.0f LF/t, flown)" % (kit_to_orbit, st['lf_per_t_to_orbit']), -pen))
    m_stack = CORE_DRY_T + s.kit.dry_t + s.kit.mono_t + s.payload_t + _t(lf)
    dv1, dv2, _ = hohmann(R_KERBIN + s.parking_alt, R_KERBIN + s.station_alt)
    b = burn_lf(m_stack, abs(dv1) + abs(dv2))
    lf -= b
    steps.append(("Hohmann 80->100 km, %.1f m/s, stack %.1f t" % (abs(dv1) + abs(dv2), m_stack), -b))
    m_stack -= _t(b)
    b = burn_lf(m_stack, s.rdv_corr_dv)
    lf -= b
    steps.append(("rendezvous corrections %.0f m/s (budget)" % s.rdv_corr_dv, -b))
    mono_left = s.kit.mono_t * 1000 - s.terminal_mono_kg
    m_core = CORE_DRY_T + s.kit.dry_t + _t(lf) + mono_left / 1000.0
    back = mono_kg(m_core, s.backaway_dv)
    mono_left -= back
    dry_home = CORE_DRY_T + s.kit.dry_t + max(mono_left, 0.0) / 1000.0
    dv = baseline_stats()['dv_eff_mean'] if dv_eff is None else dv_eff
    landed = predict_landed_lf(lf, dry=dry_home, dv_eff=dv)
    margin_dv = ISP_SKIPPER_VAC * G0 * math.log((dry_home + _t(lf)) / dry_home) - dv
    return dict(steps=steps, lf_deorbit=lf, mono_terminal_kg=s.terminal_mono_kg, mono_backaway_kg=back,
                mono_left_kg=mono_left, dry_home_t=dry_home, landed_lf=landed, landed_t=_t(landed),
                recovery_margin_dv=margin_dv, dv_eff=dv)
