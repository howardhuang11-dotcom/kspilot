"""Pre-launch vehicle analysis and whole-mission delta-v budget.

Stage delta-v comes from the stock calculator (kRPC 0.6 Stage API). The
mission budget is ascent + orbit shaping + (optional) recovery reserve, where
the recovery reserve must be carried by the parts that are never decoupled
(the "core", which is what flies home). The ascent is budgeted with the payload on;
the recovery as core propellant after the payload is released (budget()).
"""
import math

import numpy as np

G0 = 9.80665
ASCENT_DV_LKO = 3400.0     # m/s, Kerbin to 80 km circular, equatorial (calibrated by flights)
LKO = 80000.0
REC_MARGIN = 1.08          # multiplicative margin on the recovery budget (calibrated on flown recoveries 2026-10-20)
LANDING_FIXED = 60.0       # m/s: divert + hover + touchdown margin (flown landing burns ~210 m/s vs 1.3 v_t + 60)
TWR_REF = 1.6              # vacuum TWR above which ASCENT_DV_LKO's gravity loss holds


def _hohmann(mu, r1, r2):
    a = (r1 + r2) / 2
    return abs(math.sqrt(mu / r1) * (math.sqrt(r2 / a) - 1)) + abs(math.sqrt(mu / r2) * (1 - math.sqrt(r1 / a)))


def ascent_dv(body, pe, ap, inc_deg, site_lat=-0.0972):
    """Delta-v (vacuum-equivalent) to reach Pe x Ap at the given inclination."""
    mu, R = body.gravitational_parameter, body.equatorial_radius
    v_rot = body.rotational_speed * R * math.cos(math.radians(site_lat))
    v_o = math.sqrt(mu / (R + LKO))
    i = math.radians(max(abs(inc_deg), abs(site_lat)))
    plane = math.sqrt(v_o ** 2 + v_rot ** 2 - 2 * v_o * v_rot * math.cos(i)) - (v_o - v_rot)
    rp, ra = R + max(pe, LKO), R + max(ap, LKO)
    shape = _hohmann(mu, R + LKO, rp) + _hohmann(mu, rp, ra) / 2 if ra > rp else _hohmann(mu, R + LKO, rp)
    return ASCENT_DV_LKO + plane + shape


def gravity_penalty(stages, need_asc, g):
    """Extra gravity loss of ascent stages with a weak vacuum TWR: g * t_burn * (1/TWR - 1/TWR_REF),
    counted over the part of each stage's burn the ascent uses (first flight check: 248 m/s vs ~240 measured)."""
    left, pen = need_asc, 0.0
    for s in stages:
        if left <= 0:
            break
        dv = s['dv_vac']
        use = min(dv, left)
        if dv > 0 and s['twr_vac'] > 0:
            pen += g * s['burn'] * (use / dv) * max(0.0, 1.0 / s['twr_vac'] - 1.0 / TWR_REF)
        left -= use
    return pen


def recovery_dv(body, pe, ap, v_terminal, entry_burn_dv=0.0):
    """Deorbit (at apoapsis, lower Pe into the atmosphere) + entry burn + landing burn."""
    mu, R = body.gravitational_parameter, body.equatorial_radius
    ra, rp = R + ap, R + pe
    a0, a1 = (ra + rp) / 2, (ra + R - 60000) / 2     # steep entry: Pe 60 km below the surface
    deorbit = math.sqrt(mu * (2 / ra - 1 / a0)) - math.sqrt(mu * (2 / ra - 1 / a1))
    landing = 1.3 * v_terminal + LANDING_FIXED      # 1.3: gravity loss during the burn
    return REC_MARGIN * (max(deorbit, 0) + entry_burn_dv + landing)


LFO_T_PER_LF = 5 * (1 + 1.1 / 0.9) / 1000.0     # t of LF+Ox propellant per LiquidFuel unit (stock 0.9:1.1 mix)


def budget(stages, need_asc, core_dry=None, need_rec=0.0):
    """Mission margin with the recovery requirement budgeted on the CORE after payload release.

    stages: ascent order (first = lift-off), dicts with dv_asl, dv_vac, m0, m1 (kg) from the stock calculator.
    The stock calculator counts all of the core's propellant with the payload still attached. The recovery burns
    (deorbit, entry burn, landing) fly after the payload is released, so their requirement is a propellant mass on
    the core alone, prop_rec = core_dry * (exp(need_rec / ve) - 1), reserved off the end of the last stage; the
    ascent requirement is then met with that propellant AND the payload still on board.
    Without a payload (last-stage end mass == core_dry) this reduces exactly to total - need_asc - need_rec.

    Returns dict: total_vac; avail_ascent (dv left for the ascent after the reserve, payload on); margin =
    avail_ascent - need_asc (m/s, the gate number); payload, prop_rec, prop_orbit (core propellant expected in
    orbit), prop_spare (expected left after landing), all kg; margin_rec (m/s the core has beyond need_rec).
    """
    total = sum(s['dv_asl'] if i == 0 else s['dv_vac'] for i, s in enumerate(stages))
    out = dict(total_vac=total, avail_ascent=total, margin=total - need_asc, payload=0.0, prop_rec=0.0,
               prop_orbit=0.0, prop_spare=0.0, margin_rec=0.0)
    if not stages or core_dry is None or need_rec <= 0:
        return out
    last = stages[-1]
    m0, m1 = last['m0'], last['m1']
    if m0 <= m1 or last['dv_vac'] <= 0:
        out['margin'] = -need_asc - need_rec        # no propellant in the last stage: nothing to come home on
        out['avail_ascent'] = total
        return out
    ve = last['dv_vac'] / math.log(m0 / m1)
    k = last['dv_asl'] / last['dv_vac'] if len(stages) == 1 else 1.0   # a lone stage is counted at sea level
    payload = max(m1 - core_dry, 0.0)
    prop_rec = core_dry * (math.exp(need_rec / ve) - 1.0)
    before = total - k * last['dv_vac']             # ascent dv of the stages before the last
    avail = before + k * ve * math.log(m0 / (m1 + prop_rec))
    m_orbit = m0 * math.exp(-max(need_asc - before, 0.0) / (k * ve))
    prop_orbit = max(m_orbit - m1, 0.0)
    out.update(avail_ascent=avail, margin=avail - need_asc, payload=payload, prop_rec=prop_rec,
               prop_orbit=prop_orbit, prop_spare=prop_orbit - prop_rec, ve=ve,
               margin_rec=ve * math.log((core_dry + prop_orbit) / core_dry) - need_rec)
    return out


def analyse(conn, vessel, pe, ap, inc_deg, recover=True, entry_burn_dv=0.0):
    """Return (ok, report dict, list of human-readable lines)."""
    body = vessel.orbit.body
    g = body.surface_gravity
    parts = vessel.parts
    lines, problems = [], []

    # --- stages (stock calculator) ---
    stages = []
    for st in vessel.stages:
        if st.delta_v > 0.5:
            stages.append(dict(n=st.number, dv=st.delta_v, dv_vac=st.vacuum_delta_v, dv_asl=st.sea_level_delta_v,
                               twr_asl=st.sea_level_twr, twr_vac=st.vacuum_twr, burn=st.burn_time,
                               m0=st.start_mass, m1=st.end_mass))
    stages.sort(key=lambda s: -s['n'])
    # the first stage burns in the atmosphere: count its sea-level delta-v, the rest at vacuum
    total_vac = sum(s['dv_asl'] if i == 0 else s['dv_vac'] for i, s in enumerate(stages))
    for s in stages:
        lines.append("stage %d: ΔV %4.0f m/s (vac %4.0f) TWR(sea level) %.2f burn %3.0fs mass %.1f→%.1ft" % (
            s['n'], s['dv'], s['dv_vac'], s['twr_asl'], s['burn'], s['m0'] / 1e3, s['m1'] / 1e3))
    if not stages:
        problems.append("no stage produces any ΔV")

    # --- liftoff TWR ---
    first = stages[0] if stages else None
    if first and first['twr_asl'] < 1.2:
        problems.append("lift-off TWR %.2f < 1.2" % first['twr_asl'])

    # --- control ---
    has_cmd = any(p.modules and any(m.name == 'ModuleCommand' for m in p.modules) for p in parts.all)
    if not has_cmd:
        problems.append("no control source (command pod / probe core)")
    authority = len(parts.reaction_wheels) + sum(1 for e in parts.engines if e.gimballed) + len(parts.control_surfaces)
    if authority == 0:
        problems.append("no attitude control authority (reaction wheels / gimbals / control surfaces)")

    # --- mission budget ---
    need_asc = ascent_dv(body, pe, ap, inc_deg)
    grav = gravity_penalty(stages, need_asc, g)
    if grav > 1:
        lines.append("low-TWR gravity loss +%.0f m/s" % grav)
    need_asc += grav
    lines.append("ascent + insertion need ≈ %.0f m/s (vacuum equivalent)" % need_asc)
    report = dict(stages=stages, total_vac=total_vac, need_ascent=need_asc)

    if recover:
        core = [p for p in parts.all if p.decouple_stage == -1]
        core_engines = [e for e in parts.engines if e.part.decouple_stage == -1]
        legs = [l for l in parts.legs if l.part.decouple_stage == -1]
        core_wet = sum(p.mass for p in core)
        core_dry = sum(p.dry_mass for p in core)
        if not core_engines:
            problems.append("recovery stage (the part never separated) has no engine")
        if any(not e.throttle_locked is False for e in core_engines):
            problems.append("recovery stage engine cannot throttle (solid motor)")
        if len(legs) < 3:
            problems.append("recovery stage has too few landing legs (%d, at least 3)" % len(legs))
        thrust_asl = sum(e.max_thrust_at(1.0) if hasattr(e, 'max_thrust_at') else e.max_thrust for e in core_engines)
        # terminal velocity of the empty core falling tail-first at sea level
        cda = estimate_cda(conn, vessel)
        rho0 = body.density_at(0)
        v_term = math.sqrt(2 * core_dry * g / (rho0 * cda)) if cda > 0 else 300.0
        # core terminal speed as estimated (142 m/s est. vs 145 flown for GF2); the old 250 m/s floor dated from
        # stack-CdA estimates and over-asked ~160 m/s. Flown recovery 2026-10-20: 5.91 t = ~960 m/s vs 1202 asked.
        need_rec = recovery_dv(body, pe, ap, min(max(v_term, 100.0), 400.0), entry_burn_dv)
        twr_land = thrust_asl / (core_dry * 1.1 * g) if core_dry else 0
        lines.append("recovery stage: dry %.1ft wet %.1ft engines %d legs %d CdA≈%.2fm² terminal speed≈%.0fm/s landing TWR≈%.1f" % (
            core_dry / 1e3, core_wet / 1e3, len(core_engines), len(legs), cda, v_term, twr_land))
        lines.append("recovery need ≈ %.0f m/s (deorbit + %slanding burn + margin)" % (need_rec, "entry burn + " if entry_burn_dv else ""))
        if core_engines and twr_land < 1.3:
            problems.append("recovery stage landing TWR %.2f < 1.3" % twr_land)
        last_dv = stages[-1]['dv_vac'] if stages else 0
        report.update(core_dry=core_dry, core_wet=core_wet, v_term=v_term, cda=cda, need_recovery=need_rec,
                      twr_land=twr_land, last_stage_dv=last_dv)
        need_total = need_asc + need_rec
        b = budget(stages, need_asc, core_dry, need_rec)
    else:
        need_total = need_asc
        b = budget(stages, need_asc)
    report['need_total'] = need_total
    report['budget'] = b
    margin = b['margin']
    if recover and b['prop_rec'] > 0:
        # recovery budgeted as core propellant AFTER payload release (the payload does not fly home)
        lines.append("recovery propellant %.2ft (≈%.0f LF, for the recovery stage after releasing a %.2ft payload); expected at orbit %.2ft, after landing %.2ft (≈%.0f LF)" % (
            b['prop_rec'] / 1e3, b['prop_rec'] / 1e3 / LFO_T_PER_LF, b['payload'] / 1e3, b['prop_orbit'] / 1e3,
            b['prop_spare'] / 1e3, b['prop_spare'] / 1e3 / LFO_T_PER_LF))
        lines.append("ascent ΔV available %.0f m/s (recovery propellant reserved, with payload), ascent need %.0f m/s, margin %+.0f m/s (recovery side %+.0f m/s)" % (
            b['avail_ascent'], need_asc, margin, b['margin_rec']))
    else:
        lines.append("total ΔV available %.0f m/s, total need %.0f m/s, margin %+.0f m/s" % (total_vac, need_total, margin))
    if margin < 0:
        problems.append("ΔV short by %.0f m/s" % -margin)
    report['margin'] = margin
    report['problems'] = problems
    ok = not problems
    lines.append("verdict: " + ("GO" if ok else "NO-GO (launch refused) - " + "; ".join(problems)))
    return ok, report, lines


def estimate_cda(conn, vessel):
    """Drag area (Cd*A, m^2) for tail-first descent, from the stock aero model at sea level."""
    try:
        body = vessel.orbit.body
        rf = vessel.orbit.body.reference_frame
        flight = vessel.flight(rf)
        pos = vessel.position(rf)
        up = vessel.direction(rf)              # vessel nose direction
        v = 100.0
        vel = tuple(-v * c for c in up)        # moving tail-first
        f = flight.simulate_aerodynamic_force_at(body, pos, vel, vessel.rotation(rf))
        drag = -sum(a * b for a, b in zip(f, vel)) / v   # component opposing motion
        rho = body.density_at(max(flight.mean_altitude, 0))
        return 2 * drag / (rho * v * v)
    except Exception:
        return 0.0


def estimate_cda_at(conn, vessel, alt, speed=150.0):
    """Tail-first drag area (m^2) of the vessel in its current configuration, simulated at `alt`."""
    try:
        body = vessel.orbit.body
        rf = body.reference_frame
        lat, lon = vessel.flight(rf).latitude, vessel.flight(rf).longitude
        pos = body.position_at_altitude(lat, lon, alt, rf)
        up = np.asarray(pos) / np.linalg.norm(pos)
        vel = tuple(-speed * up)
        # orientation: nose up (engine first into a falling airflow)
        rot = vessel.rotation(rf)
        f = vessel.flight(rf).simulate_aerodynamic_force_at(body, pos, vel, rot)
        drag = -float(np.dot(f, vel)) / speed
        return max(2 * drag / (body.density_at(alt) * speed * speed), 0.5)
    except Exception:
        return 3.0


DT_ALTS = [0, 2000, 5000, 10000, 15000, 20000, 25000, 30000, 35000, 40000, 44000, 48000, 52000, 56000, 60000, 62000, 64000, 66000, 68000, 69500]
DT_SPEEDS = [30, 60, 100, 150, 200, 300, 400, 600, 800, 1000, 1300, 1600, 2000, 2400, 2800]


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def drag_table(conn, vessel, alts=DT_ALTS, speeds=DT_SPEEDS, tilt_deg=0.0):
    """Engine-first drag force table F[alt][speed] (N) of the vessel in its current configuration,
    from the stock aero model (Mach effects included). The vessel should point nose-up (radial).
    tilt_deg: the nose tilted that far off the airflow ("belly" drag: GF0T CdA 3.7 -> 19 m^2 at 45 deg, Mach 6)."""
    body = vessel.orbit.body
    rf = body.reference_frame
    fl = vessel.flight(rf)
    lat, lon = fl.latitude, fl.longitude
    rot = vessel.rotation(rf)
    if tilt_deg:
        pos0 = np.asarray(body.position_at_altitude(lat, lon, 1000.0, rf))
        up0 = pos0 / np.linalg.norm(pos0)
        axis = np.cross(up0, [0.0, 1.0, 0.0])
        axis /= np.linalg.norm(axis)
        hs = math.sin(math.radians(tilt_deg) / 2)
        rot = _qmul((axis[0] * hs, axis[1] * hs, axis[2] * hs, math.cos(math.radians(tilt_deg) / 2)), rot)
    F = []
    for a in alts:
        pos = body.position_at_altitude(lat, lon, a, rf)
        up = np.asarray(pos) / np.linalg.norm(pos)
        row = []
        for s in speeds:
            vel = tuple(-s * up)
            f = fl.simulate_aerodynamic_force_at(body, pos, vel, rot)
            row.append(max(-float(np.dot(f, vel)) / s, 0.0))
        F.append(row)
    return {"alts": list(alts), "speeds": list(speeds), "F": F}
