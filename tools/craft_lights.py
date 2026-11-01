"""Light kit for the recovery core (V17, video requirements: the deorbit burn is on the night side and looks black;
requirement: put lights on the rocket, no brightening in post).

Variants (the baselines stay untouched; NOT copied into the game's Ships folder):
  craft/VAB/T-WorksFineGF-LIT.craft       = T-WorksFineGF      + kit
  craft/VAB/T-WorksFineGF-DOCK-LIT.craft  = T-WorksFineGF-DOCK + kit
Kit: 8 x stock "Spotlight Mk1" (part spotLight3, Squad/Parts/Utility/Lights/spotLight1.cfg: 0.005 t, maxTemp 2000 K,
PhysicsSignificance = 1 (physicsless: mass and drag go to the parent tank), ModuleLight 0.02 EC/s, model light_08:
Unity spot light, range 15 m, spot angle 135 deg, white; ToggleLightAction in the Light action group = kRPC
vessel.control.lights). All surface-attached on the core tanks, staging untouched, radial symmetry x4:
  TOP ring  4 on the Rockomax32 at y = TOP_Y, 0.33 m below the lower edge of the stowed airbrakes (20.43 m; deployed
            they swing up/out, 21.41 m) and 0.8 m below the grid fins (21.25 m), azimuths in the middle of the
            airbrake gaps (airbrake + 22.5 deg, 45 deg off the grid fins); beam tilted TILT deg DOWN along the body.
  BOTTOM ring 4 on the Rockomax64 at y = BOT_Y (tank bottom 11.57 m), azimuths midway between an SRB radial
            decoupler (45 + 90k) and the next landing leg (103.5 + 90k), i.e. away from the SRB separation paths and
            the leg swing planes; beam tilted TILT deg UP along the body.
Geometry convention (checked against the launch clamps, whose attach node orientation is also 0,0,-1): the attach
node orientation (part -z) points away from the parent; the lamp beam is along part -z (model light_08: the light
transform is rotated 180 deg about y). Azimuth convention as tools/craft_dock.py (0 = -x side).

usage: craft_lights.py build     writes both LIT crafts and checks them (tools/craft_ballast.check)
       craft_lights.py report    mass / CoM / drag / EC / heat deltas and part clearances
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import craft_ballast as cb  # noqa: E402
import craft_dock as cd  # noqa: E402

ROOT = cb.ROOT
BASES = {"T-WorksFineGF": cb.SRC, "T-WorksFineGF-DOCK": cd.DST}
TANK32 = "Rockomax32.BW_4294373742"
TANK64 = "Rockomax64.BW_4294712106"
LAMP_CFG = "spotLight3"
LAMP_MASS = 0.005            # t        [cfg]
LAMP_EC = 0.02               # EC/s     [cfg resourceAmount]
LAMP_MAXTEMP = 2000.0        # K        [cfg]
AIRBRAKE_MAXTEMP = 2400.0    # K        [Squad/Parts/Aero/airbrake/Airbrake.cfg]
TOP = [("spotLight3_42910003%02d" % (11 + i), "14500003%02d" % (11 + i)) for i in range(4)]
BOT = [("spotLight3_42910003%02d" % (21 + i), "14500003%02d" % (21 + i)) for i in range(4)]
TOP_Y = 20.10                # m (craft frame)
BOT_Y = 12.20
TILT = 35.0                  # deg from the outward normal towards the body axis (= the lamp head's own pitch limit)
R_ATTACH = 1.2484            # m: Rockomax collider radius at an attach point [craft_dock: RV-105 offset + node]
LAMP_R = 0.12                # m: lamp envelope radius (cube 0.224 x 0.224 x 0.134)


def _parse(path):
    header, blocks, trailer = cb.split_parts(open(path).read())
    return header, blocks, trailer, {cb.fields(b)["part"][0]: cb.fields(b) for b in blocks}


def _vec(s):
    return [float(t) for t in s.split(",")]


def azimuth(dx, dz):
    """deg, craft_dock convention: offset (-r cos az, r sin az)."""
    return math.degrees(math.atan2(dz, -dx)) % 360.0


def ring_azimuths(f):
    """(top, bottom) lamp azimuths from the craft itself: mid airbrake gaps beside the grid fins / mid between each
    SRB decoupler and the next landing leg."""
    ax, _, az_ = _vec(f[TANK64]["pos"][0])

    def az_of(pid):
        x, _, z = _vec(f[pid]["pos"][0])
        return azimuth(x - ax, z - az_)
    fins = sorted(az_of(p) for p in f if p.startswith("Grid Fin M_"))
    decs = sorted(az_of(p) for p in f if p.startswith("radialDecoupler_"))
    legs = sorted(az_of(p) for p in f if p.startswith("landingLeg1-2_"))
    top = [(a + 22.5) % 360.0 for a in fins]
    bot = []
    for d in decs:
        nxt = min(((l - d) % 360.0 for l in legs))
        bot.append((d + nxt / 2.0) % 360.0)
    return top, bot


def lamp_quat(az_deg, tilt_deg, down):
    """Rotation (x, y, z, w) whose part -z (beam) is the outward normal tilted tilt_deg towards -y (down) or +y."""
    a, t = math.radians(az_deg), math.radians(tilt_deg)
    u = (-math.cos(a), 0.0, math.sin(a))
    s = -1.0 if down else 1.0
    b = (math.cos(t) * u[0], s * math.sin(t), math.cos(t) * u[2])           # beam
    zc = tuple(-c for c in b)                                                # part +z
    yc = (-s * math.sin(t) * u[0], math.cos(t), -s * math.sin(t) * u[2])     # part +y: in the (u, Y) plane, _|_ beam
    xc = (yc[1] * zc[2] - yc[2] * zc[1], yc[2] * zc[0] - yc[0] * zc[2], yc[0] * zc[1] - yc[1] * zc[0])
    m = [[xc[0], yc[0], zc[0]], [xc[1], yc[1], zc[1]], [xc[2], yc[2], zc[2]]]   # columns = images of x, y, z
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        q = ((m[2][1] - m[1][2]) / S, (m[0][2] - m[2][0]) / S, (m[1][0] - m[0][1]) / S, 0.25 * S)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        S = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        q = (0.25 * S, (m[0][1] + m[1][0]) / S, (m[0][2] + m[2][0]) / S, (m[2][1] - m[1][2]) / S)
    elif m[1][1] > m[2][2]:
        S = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        q = ((m[0][1] + m[1][0]) / S, 0.25 * S, (m[1][2] + m[2][1]) / S, (m[0][2] - m[2][0]) / S)
    else:
        S = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        q = ((m[0][2] + m[2][0]) / S, (m[1][2] + m[2][1]) / S, 0.25 * S, (m[1][0] - m[0][1]) / S)
    return tuple(round(c, 9) + 0.0 for c in q)


def qrot(q, v):
    """Rotate vector v by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz, vz + w * tz + x * ty - y * tx)


def _light_module():
    out = ["\tMODULE", "\t{", "\t\tname = ModuleLight", "\t\tisEnabled = True", "\t\tisOn = False",
           "\t\tuiWriteLock = False", "\t\tlightR = 1", "\t\tlightG = 1", "\t\tlightB = 1",
           "\t\tstagingEnabled = True", "\t\tEVENTS", "\t\t{", "\t\t}", "\t\tACTIONS", "\t\t{"]
    for a, g in (("ToggleLightAction", "Light"), ("LightOnAction", "None"), ("LightOffAction", "None")):
        out += ["\t\t\t%s" % a, "\t\t\t{", "\t\t\t\tactionGroup = %s" % g,
                "\t\t\t\twasActiveBeforePartWasAdjusted = False", "\t\t\t}"]
    return out + ["\t\t}", "\t\tUPGRADESAPPLIED", "\t\t{", "\t\t}", "\t}"]


_VARIANTS = cd._module("ModulePartVariants", [("useVariantMass", "True"), ("stagingEnabled", "True")])


def lamp_block(ring, i, parent, parent_pos, y, az, down):
    pid, persist = ring[i]
    a = math.radians(az)
    dx, dz = round(-R_ATTACH * math.cos(a), 8) + 0.0, round(R_ATTACH * math.sin(a), 8) + 0.0
    px, py, pz = parent_pos
    rot = ",".join(cb._num(c) for c in lamp_quat(az, TILT, down))
    sym = [ring[j][0] for j in range(len(ring)) if j != i]
    out = cd._head(pid, persist, (px + dx, y, pz + dz), (dx, y - py, dz), rot, -1, 0, -1, 1, sym=sym,
                   srfn="srfAttach,%s,collider,0|0|0,0|0|-1,0|0|0" % parent)
    out += _light_module() + _VARIANTS + cd._CARGO + cd._KOS
    return "\n".join(out + ["}"])


def build(src, ship):
    header, blocks, trailer, f = _parse(src)
    ids = list(f)
    new = [p for p, _ in TOP + BOT]
    assert TANK32 in ids and TANK64 in ids and not set(new) & set(ids)
    top_az, bot_az = ring_azimuths(f)
    assert len(top_az) == 4 and len(bot_az) == 4
    out = []
    for b in blocks:
        pid = cb.fields(b)["part"][0]
        if pid in (TANK32, TANK64):
            ring, y, azs, down = (TOP, TOP_Y, top_az, True) if pid == TANK32 else (BOT, BOT_Y, bot_az, False)
            last = "\tlink = %s" % f[pid]["link"][-1]
            assert b.count(last) == 1
            b = b.replace(last, last + "".join("\n\tlink = %s" % r[0] for r in ring))
            out.append(b)
            pos = _vec(f[pid]["pos"][0])
            out += [lamp_block(ring, i, pid, pos, y, azs[i], down) for i in range(4)]
            continue
        out.append(b)
    import re
    header = re.sub(r"^ship = .*$", "ship = %s" % ship, header, count=1, flags=re.M)
    return "\n".join([header] + out + ([trailer] if trailer else []))


def dst_of(base_ship):
    return os.path.join(ROOT, "craft", "VAB", base_ship + "-LIT.craft")


def check(path, base_path, db=None):
    probs, info = cb.check(path, base_path, db)
    return probs, info


# ---------------------------------------------------------------- report
def _cubes_raw(path=cd.PARTDB):
    """{part name: [(centre (3), size (3))]} from PartDatabase.cfg (read-only)."""
    out, url = {}, None
    for ln in open(path, encoding="utf-8", errors="replace"):
        s = ln.strip()
        if s.startswith("url = "):
            url = s[6:].rsplit("/", 1)[-1].replace("_", ".")
        elif s.startswith("cube = ") and url:
            v = [float(t) for t in s[7:].split(",", 1)[1].split(",")]
            out.setdefault(url, []).append((tuple(v[18:21]), tuple(v[21:24])))
    return out


def clearances(path, boxes, skip=()):
    """Per lamp: the nearest other part (envelope over all its cubes, oriented box) and the gap in m minus LAMP_R."""
    _, _, _, f = _parse(path)
    lamps = [p for p in f if p.startswith(LAMP_CFG + "_")]
    res = []
    for lp in lamps:
        L = _vec(f[lp]["pos"][0])
        best = (1e9, "")
        for pid, g in f.items():
            if pid == lp or pid in (TANK32, TANK64) or pid.startswith((LAMP_CFG + "_",) + tuple(skip)):
                continue
            cfg = pid.rsplit("_", 1)[0]
            bl = boxes.get(cfg) or boxes.get(cfg.replace(" ", "."))
            if not bl:
                continue
            P, q = _vec(g["pos"][0]), _vec(g["rot"][0])
            qi = (-q[0], -q[1], -q[2], q[3])
            d = qrot(qi, (L[0] - P[0], L[1] - P[1], L[2] - P[2]))       # lamp centre in the part frame
            for c, s in bl:
                gap = math.sqrt(sum(max(abs(d[k] - c[k]) - s[k] / 2.0, 0.0) ** 2 for k in range(3)))
                if gap < best[0]:
                    best = (gap, pid)
        res.append((lp, best[0] - LAMP_R, best[1]))
    return res


def drag_area(path, cubes):
    """Lamp kit area x Cd: (axial, flow from below = engine-first descent; upper bound = largest face). Faces of a
    rotated lamp are weighted by max(0, n . flow-facing direction); unoccluded."""
    _, _, _, f = _parse(path)
    faces_n = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
    ax, ub = 0.0, 0.0
    for pid, g in f.items():
        if not pid.startswith(LAMP_CFG + "_"):
            continue
        q = _vec(g["rot"][0])
        best = 0.0
        for _, faces in cubes[LAMP_CFG]:
            a = 0.0
            for k, n in enumerate(faces_n):
                w = max(0.0, -qrot(q, n)[1])                         # face normal towards -y (the oncoming flow)
                a += w * faces[k][0] * faces[k][1]
            best = max(best, a)
        ax += best
        ub += max(fa[0] * fa[1] for _, faces in cubes[LAMP_CFG] for fa in faces)
    return ax, ub


def report():
    db = cb.stock_db()
    cubes = cd.drag_cubes()
    boxes = _cubes_raw()
    lines = []

    def p(s):
        lines.append(s)
        print(s)
    p("lamp %s: %.3f t, %.2f EC/s, maxTemp %.0f K; kit %d lamps = %.3f t, %.2f EC/s" % (
        LAMP_CFG, db[0][LAMP_CFG], LAMP_EC, LAMP_MAXTEMP, len(TOP + BOT), len(TOP + BOT) * db[0][LAMP_CFG],
        len(TOP + BOT) * LAMP_EC))
    for ship, base in BASES.items():
        dst = dst_of(ship)
        probs, info = check(dst, base, db)
        _, binfo = cb.check(base, None, db)
        p("%s: %s; launch mass %.3f -> %.3f t (%+.3f), parts %d -> %d" % (
            os.path.basename(dst), "OK" if not probs else probs, binfo["total_no_clamps"], info["total_no_clamps"],
            info["total_no_clamps"] - binfo["total_no_clamps"], binfo["n_parts"], info["n_parts"]))
        rb, bb = cd._parts(base, db)
        rl, bl_ = cd._parts(dst, db)
        for lf, what in ((0, "dry"), (910, "deorbit (910 LF)"), (369, "touchdown (369 LF)")):
            mb, yb = cd._com(cd.recovery_items(rb, bb, lf))
            ml, yl = cd._com(cd.recovery_items(rl, bl_, lf))
            p("  recovery stage %s: mass %.3f -> %.3f t (%+.3f), CoM y %.3f -> %.3f m (%+.4f)" % (
                what, mb, ml, ml - mb, yb, yl, yl - yb))
        ax, ub = drag_area(dst, cubes)
        p("  kit area x Cd (unoccluded): axial %.3f m2 (upper bound, largest face of each lamp %.3f m2) = "
          "%+.2f %% (%+.2f %%) of the measured core CdA brakes out 27.1 m2; %+.2f %% (%+.2f %%) of brakes in 11.8 m2" % (
              ax, ub, 100 * ax / 27.1, 100 * ub / 27.1, 100 * ax / 11.8, 100 * ub / 11.8))
        for (lp, gap, who), (_, gap2, who2) in zip(clearances(dst, boxes), clearances(dst, boxes, ("launchClamp1_",))):
            p("  clearance %s -> nearest %s: %.2f m%s" % (lp, who, gap, (
                " (box envelope incl. the clamp's top arm, which meets the tank at y 16.06 m, 3.9 m above the lamp;"
                " the clamps stay on the pad: check on the pad); nearest flight part %s: %.2f m" % (who2, gap2))
                if who.startswith("launchClamp") else ""))
    p("heat: logged peak entry heat ratio on T-WorksFineGF 0.53-0.62 (A.I.R.B.R.A.K.E.S, maxTemp %.0f K) = %.0f-%.0f K;"
      " a lamp at the same temperature: %.2f-%.2f of its %.0f K (lander IDENT/A1E abort at 0.75; heat_safety 9.0 "
      "on airbrake craft)" % (AIRBRAKE_MAXTEMP, 0.53 * AIRBRAKE_MAXTEMP, 0.62 * AIRBRAKE_MAXTEMP,
                             0.53 * AIRBRAKE_MAXTEMP / LAMP_MAXTEMP, 0.62 * AIRBRAKE_MAXTEMP / LAMP_MAXTEMP,
                             LAMP_MAXTEMP))
    return lines


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "build":
        db = cb.stock_db()
        for ship, base in BASES.items():
            dst = dst_of(ship)
            open(dst, "w").write(build(base, ship + "-LIT"))
            probs, info = check(dst, base, db)
            print("%s: %d parts, total %.2f t (%.2f t without clamps), payload %.3f t, %s" % (
                os.path.basename(dst), info["n_parts"], info["total"], info["total_no_clamps"], info["payload"],
                "OK" if not probs else "PROBLEMS: %s" % probs))
    elif len(sys.argv) > 1 and sys.argv[1] == "report":
        report()
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
