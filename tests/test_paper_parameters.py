"""Most defaults quoted in the paper (Table 2 and Section 3 of paper/kspilot_report.md) must match the released code.

Covered: the parameters listed in PARAMETERS below. Not covered: the predictor-corrector constants of Section 5.2 (the
cutoff is not in the released code) and the divert-envelope numbers of Section 5.3 (simulation outputs); the paper
says "most defaults", not all.

The check reads the source text, so it needs neither the game nor kRPC. When a default changes, this test fails
until the paper and this table are updated together.
"""
import os
import re

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def src(rel):
    with open(os.path.join(ROOT, rel), encoding="utf-8") as f:
        return f.read()


# (file, regular expression that must match, value quoted in the paper)
PARAMETERS = [
    ("kspilot/feasibility.py", r"REC_MARGIN = 1\.08\b", "pre-launch recovery margin factor 1.08"),
    ("kspilot/ascent_own.py", r'KSP_Q_SEP_MAX", 10000\.0\)', "booster separation held until q <= 10 kPa"),
    ("kspilot/ascent_own.py", r"\(low and now - s\[\"t_low\"\] >= 0\.5\)", "thrust < 50 % for 0.5 s after staging aborts"),
    ("kspilot/ascent_own.py", r"ANOMALY: unplanned part loss", "part count vs expectation is logged only"),
    ("kspilot/mission.py", r'KSP_MAX_DVN", 30\.0\)', "deorbit window normal-dv threshold 30 m/s"),
    ("kspilot/deorbit.py", r"max_dvn: float = 30\.0, n_revs: int = 12", "window survey over 12 revolutions"),
    ("kspilot/mission.py", r'KSP_HIBERNATE_MIN_H", 2\.0\)', "hibernation on coasts longer than 2 h"),
    ("kspilot/power.py", r"EC_RATE_HIB = 0\.01\b", "budgeted hibernation rate 0.01 EC/s"),
    ("kspilot/power.py", r"EC_KEEP = 0\.3\b", "at least 30 % charge at touchdown"),
    ("kspilot/mission.py", r"def coast_and_correct\(self, v, cda, tol=300\.0, drag=None, max_dv=15\.0, aim_bias=-1500\.0",
     "vacuum corrections: tolerance 300 m, at most 15 m/s per trim, aim 1.5 km short"),
    ("kspilot/mission.py", r"for i in range\(6\):", "at most six vacuum corrections"),
    ("kspilot/mission.py", r"gain, last = 0\.6, None", "initial vacuum-correction gain 0.6"),
    ("kspilot/mission.py", r"gain = min\(1\.0, max\(0\.2, gain / ratio \* 0\.9\)\)", "secant gain update with factor 0.9"),
    ("tools/orbit_recover.py", r"m\.coast_and_correct\(v, cda, drag=drag_out", "corrections use the airbrakes-out table"),
    ("kspilot/mission.py", r"self\.entry_alt, self\.entry_dv = 45000\.0, 590\.0", "entry burn near 45 km, 590 m/s"),
    ("kspilot/landing.py", r"tilt = max\(-3\.0, min\(3\.0,", "entry-burn cross-track tilt <= 3 deg"),
    ("kspilot/landing.py", r"cap = 1\.25 \* max\(float\(self\.thrust\(\)\), 1\.0\) / m_prev \* max\(dt, 0\.0\) \+ 0\.5",
     "per-tick dv cap 1.25 x full-thrust dv + 0.5 m/s"),
    ("kspilot/landing.py", r"if ut - self\._dm_t < 1\.0", "drag modulation at 1 Hz"),
    ("kspilot/landing.py", r"if q_true > 10000:", "airbrakes forced out above 10 kPa"),
    ("kspilot/landing.py", r"c_out\.along > tgt \+ 150 and alt_now > 6000", "retro pulse when long by > 150 m above 6 km"),
    ("kspilot/landing.py", r"self\.retro_until = ut \+ 0\.5", "retro pulses of 0.5 s"),
    ("kspilot/landing.py", r"if self\._pulses % 5 == 1:", "only every fifth pulse is logged"),
    ("kspilot/landing.py", r'want = 1 if \(phase == "COAST" and h > 25000\) else 0', "2x physics warp above 25 km"),
    ("kspilot/landing.py", r'KSP_GLIDE_BELOW", 10000\.0\)', "glide below 10 km"),
    ("kspilot/landing.py", r'KSP_GLIDE_MAX", 8\.0\)', "glide tilt <= 8 deg"),
    ("kspilot/landing.py", r"glide_tmin = 15\.0", "glide time-to-go floor 15 s"),
    ("kspilot/landing.py", r"glide_h_ign = 450\.0", "glide end point 450 m"),
    ("kspilot/landing.py", r"h_gate=4\.0, v_td=1\.5", "gate 4 m above the pad, final descent 1.5 m/s"),
    ("kspilot/landing.py", r"drag_frac=0\.35, h_stop=25\.0, throttle=0\.65,\s+lat_eff=0\.35, t_att=3\.0",
     "ignition test: throttle 0.65, stop 25 m, drag credit 0.35, lateral efficiency 0.35, 3 s attitude dead time"),
    ("kspilot/landing.py", r"commit_h, funnel_k = 25\.0, 0\.2", "commit 25 m above the gate; funnel 0.2 h"),
    ("kspilot/landing.py", r"commit_cap = False", "nullable-before-commit lateral cap disabled"),
    ("kspilot/landing.py", r"cap = min\(3\.0, self\.funnel_k \* max\(h, 0\.0\)\)", "funnel min(3 m/s, 0.2 h)"),
    ("kspilot/landing.py", r"DEADMAN_S = 2\.5", "dead-man threshold 2.5 s"),
    ("kspilot/landing.py", r"time\.sleep\(3\.0\)\s+act\.release\(\)", "3 s attitude hold after touchdown"),
    ("kspilot/landing.py", r"time\.sleep\(10\)\s+# judge only after it has settled", "10 s settling before the miss"),
]


@pytest.mark.parametrize("rel, pattern, quoted", PARAMETERS, ids=[p[2] for p in PARAMETERS])
def test_documented_default(rel, pattern, quoted):
    assert re.search(pattern, src(rel)), "paper quotes '%s' but %s does not match /%s/" % (quoted, rel, pattern)
