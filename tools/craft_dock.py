"""Docking variant T-WorksFineGF-DOCK (rung R1 of docking concept study; R0 recommendation docking R0 results).

Changes against craft/VAB/T-WorksFineGF.craft (everything else byte-identical, staging untouched):
  cargo  RelayAntenna50 (RA-15, 0.30 t) removed: CommNet is off in the AutoPilotLab save (EnableCommNet = False) and
         the HECS2 has its own antenna, so it has no function; its stack node (cargo probe top) now carries a
         dockingPort2 (Clamp-O-Tron, size1 = the cargo's 1.25 m diameter, 0.05 t). It stays inside the fairing.
  core   (the RCS kit flies home with the core, R0 section 1)
         4 x RCSBlock.v2 (RV-105, variant "4Horn" = orthogonal 4 nozzles, R0's 4 kN axial / 2 kN lateral) on the
           Rockomax64 at y = RCS_Y (the docking-approach CoM of the core + cargo, see `report`), azimuths
           0/90/180/270 deg = between the four SRBs (they stand at 45/135/225/315) and above the launch-clamp arms;
           pitch/yaw/roll channels DISABLED in the craft (R0 A7: attitude by the reaction wheels only; and the lander
           sets control.rcs = True whenever the vessel has RCS -- kspilot/landing.py run() -- so with rotation
           enabled the RCS would join the EDL attitude control, a new variable).
         4 x radialRCSTank (Stratus-V Roundified, 20 u max) at y = TANK_Y above the SRB nose cones (top 17.24 m),
           azimuths 45/135/225/315; 12.5 u each = 50 u = 200 kg MonoPropellant (R0 section 5).
Geometry of the surface parts follows standard KSP-editor (1.12.5) radial placements: RV-105 on a
2.5 m Rockomax tank (offset 1.23966 m, identity rotation = -x side) and radialRCSTank on a probe (node 0.22 m).

usage: craft_dock.py build     writes craft/VAB/T-WorksFineGF-DOCK.craft and checks it (tools/craft_ballast.check)
       craft_dock.py report    mass, CoM and drag-area deltas of the recovery stage and the docking stack
"""
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import craft_ballast as cb  # noqa: E402

ROOT = cb.ROOT
SRC = cb.SRC
SHIP = "T-WorksFineGF-DOCK"
DST = os.path.join(ROOT, "craft", "VAB", SHIP + ".craft")
PARTDB = os.path.join(os.path.dirname(cb.GAMEDATA), "PartDatabase.cfg")

CARGO_PROBE = "HECS2.ProbeCore_4291353178"
ANTENNA = "RelayAntenna50_4290772886"
TANK64 = "Rockomax64.BW_4294712106"
PORT = ("dockingPort2_4291000201", "1450000201")
RCS = [("RCSBlock.v2_42910002%02d" % (11 + i), "14500002%02d" % (11 + i)) for i in range(4)]
MONO = [("radialRCSTank_42910002%02d" % (21 + i), "14500002%02d" % (21 + i)) for i in range(4)]
RCS_AZ = (0.0, 90.0, 180.0, 270.0)        # deg; 0 = -x (KSP editor convention: identity rotation)
MONO_AZ = (45.0, 135.0, 225.0, 315.0)
RCS_Y = 16.97            # m (craft frame): midway between the approach CoM for top-first (16.59) / proportional
                         # (17.36) tank drain, `report`
TANK_Y = 18.30           # m: tank spans ~18.0-18.6, SRB nose cones end at 17.24
RCS_R = 1.23966277       # m from the tank axis [RV-105 on a Rockomax32 in the editor]
MONO_R = 1.2484 + 0.22   # m: tank collider radius (RV-105 offset + its 0.00875 node) + radialRCSTank node 0.22
MONO_UNITS = 12.5        # per tank (maxAmount 20)
PORT_DY = 0.5            # cargo HECS2 top node
PORT_H = 0.2828832       # dockingPort2 top node [cfg]
LF_T = 0.005 * (1 + 1.1 / 0.9)            # t per LF unit with its oxidizer (ResourcesGeneric densities)


def _q(az_deg):
    a = math.radians(az_deg) / 2.0
    return "0,%s,0,%s" % (cb._num(round(math.sin(a), 9) + 0.0), cb._num(round(math.cos(a), 9) + 0.0))


def _off(r, az_deg):
    a = math.radians(az_deg)
    return round(-r * math.cos(a), 8) + 0.0, round(r * math.sin(a), 8) + 0.0


_SAMEVESSEL = ["\tACTIONS", "\t{"] + sum([["\t\t%s" % n, "\t\t{", "\t\t\tactionGroup = None",
                                           "\t\t\twasActiveBeforePartWasAdjusted = False", "\t\t}"]
                                          for n in ("ToggleSameVesselInteraction", "SetSameVesselInteraction",
                                                    "RemoveSameVesselInteraction")], []) + ["\t}"]


def _module(name, fields, actions=()):
    out = ["\tMODULE", "\t{", "\t\tname = %s" % name, "\t\tisEnabled = True"] + ["\t\t%s = %s" % kv for kv in fields]
    out += ["\t\tEVENTS", "\t\t{", "\t\t}", "\t\tACTIONS", "\t\t{"]
    for a in actions:
        out += ["\t\t\t%s" % a, "\t\t\t{", "\t\t\t\tactionGroup = None", "\t\t\t\twasActiveBeforePartWasAdjusted = False",
                "\t\t\t}"]
    out += ["\t\t}", "\t\tUPGRADESAPPLIED", "\t\t{", "\t\t}", "\t}"]
    return out


_CARGO = _module("ModuleCargoPart", [("beingAttached", "False"), ("beingSettled", "False"),
                                     ("reinitResourcesOnStoreInVessel", "False"), ("stagingEnabled", "True")])
_KOS = _module("KOSNameTag", [("nameTag", ""), ("stagingEnabled", "True")])


def _head(pid, persist, pos, attpos0, rot, istg, dstg, sepi, attm, sym=(), srfn=None, links=(), attn=()):
    out = ["PART", "{", "\tpart = %s" % pid, "\tpartName = Part", "\tpersistentId = %s" % persist,
           "\tpos = %s" % ",".join(cb._num(x) for x in pos), "\tattPos = 0,0,0",
           "\tattPos0 = %s" % ",".join(cb._num(x) for x in attpos0), "\trot = %s" % rot, "\tattRot = 0,0,0,1",
           "\tattRot0 = %s" % rot, "\tmir = 1,1,1", "\tsymMethod = Radial", "\tautostrutMode = Off",
           "\trigidAttachment = False", "\tistg = %s" % istg, "\tresPri = 0", "\tdstg = %s" % dstg, "\tsidx = -1",
           "\tsqor = -1", "\tsepI = %s" % sepi, "\tattm = %d" % attm, "\tsameVesselCollision = False",
           "\tmodCost = 0", "\tmodMass = 0", "\tmodSize = 0,0,0"]
    out += ["\tlink = %s" % l for l in links] + ["\tsym = %s" % s for s in sym]
    out += ["\tattN = %s" % a for a in attn]
    if srfn:
        out.append("\tsrfN = %s" % srfn)
    return out + ["\tEVENTS", "\t{", "\t}"] + _SAMEVESSEL + ["\tPARTDATA", "\t{", "\t}"]


def port_block(probe_pos):
    x, y, z = probe_pos
    out = _head(PORT[0], PORT[1], (x, y + PORT_DY, z), (0, PORT_DY, 0), "0,0,0,1", 0, 4, 1, 0,
                attn=["top,Null_0_0|%s|0_0|1|0_0|%s|0_0|1|0" % (PORT_H, PORT_H),
                      "bottom,%s_0|0|0_0|-1|0_0|0|0_0|-1|0" % CARGO_PROBE])
    out += _module("ModuleDockingNode", [("crossfeed", "True"), ("stagingEnabled", "False"), ("state", "Ready"),
                                         ("dockUId", "0"), ("dockNodeIdx", "0")],
                   ("UndockAction", "DecoupleAction", "MakeReferenceToggle", "EnableXFeedAction",
                    "DisableXFeedAction", "ToggleXFeedAction"))
    return "\n".join(out + _CARGO + _KOS + ["}"])


def rcs_block(i, tank_pos):
    x, y, z = tank_pos
    dx, dz = _off(RCS_R, RCS_AZ[i])
    sym = [RCS[j][0] for j in range(4) if j != i]
    out = _head(RCS[i][0], RCS[i][1], (x + dx, RCS_Y, z + dz), (dx, RCS_Y - y, dz), _q(RCS_AZ[i]), -1, 0, -1, 1,
                sym=sym, srfn="srfAttach,%s,collider,-0.00875000004|0|0,0.125|0|0,-0.00875000004|0|0" % TANK64)
    out += _module("ModulePartVariants", [("useVariantMass", "True"), ("stagingEnabled", "True"),
                                          ("selectedVariant", "4Horn")])
    out += _module("ModuleRCSFX", [("rcsEnabled", "True"), ("thrustPercentage", "100"), ("currentShowToggles", "False"),
                                   ("enableYaw", "False"), ("enablePitch", "False"), ("enableRoll", "False"),
                                   ("enableX", "True"), ("enableY", "True"), ("enableZ", "True"),
                                   ("useThrottle", "False"), ("fullThrust", "False"), ("stagingEnabled", "False")],
                   ("ToggleAction",))
    out += _module("FXModuleAnimateRCS", [("stagingEnabled", "True"), ("animStates", "0,0,0,0,0,0,0,0,0")])
    return "\n".join(out + _CARGO + _KOS + ["}"])


def mono_block(i, tank_pos):
    x, y, z = tank_pos
    dx, dz = _off(MONO_R, MONO_AZ[i])
    sym = [MONO[j][0] for j in range(4) if j != i]
    out = _head(MONO[i][0], MONO[i][1], (x + dx, TANK_Y, z + dz), (dx, TANK_Y - y, dz), _q(MONO_AZ[i] + 90.0), -1, 0,
                -1, 1, sym=sym, srfn="srfAttach,%s,collider,0|0|0.219999999,0|0|-1,0|0|0.219999999" % TANK64)
    out += _CARGO + _KOS
    out += ["\tRESOURCE", "\t{", "\t\tname = MonoPropellant", "\t\tamount = %s" % cb._num(MONO_UNITS),
            "\t\tmaxAmount = 20", "\t\tflowState = True", "\t\tisTweakable = True", "\t\thideFlow = False",
            "\t\tisVisible = True", "\t\tflowMode = Both", "\t}"]
    return "\n".join(out + ["}"])


def build(src=SRC):
    header, blocks, trailer = cb.split_parts(open(src).read())
    ids = [cb.fields(b)["part"][0] for b in blocks]
    new_ids = [PORT[0]] + [r[0] for r in RCS] + [m[0] for m in MONO]
    assert ANTENNA in ids and CARGO_PROBE in ids and TANK64 in ids and not set(new_ids) & set(ids)
    out = []
    for b in blocks:
        f = cb.fields(b)
        pid = f["part"][0]
        if pid == ANTENNA:
            continue                                   # removed (see module doc)
        if pid == CARGO_PROBE:
            assert ("\tlink = %s" % ANTENNA) in b and ("\tattN = top,%s_" % ANTENNA) in b
            b = b.replace("\tlink = %s" % ANTENNA, "\tlink = %s" % PORT[0])
            b = b.replace("\tattN = top,%s_" % ANTENNA, "\tattN = top,%s_" % PORT[0])
            out.append(b)
            out.append(port_block([float(t) for t in f["pos"][0].split(",")]))
            continue
        if pid == TANK64:
            pos = [float(t) for t in f["pos"][0].split(",")]
            last = "\tlink = %s" % f["link"][-1]
            assert b.count(last) == 1
            b = b.replace(last, last + "".join("\n\tlink = %s" % n for n in new_ids[1:]))
            out.append(b)
            out += [rcs_block(i, pos) for i in range(4)] + [mono_block(i, pos) for i in range(4)]
            continue
        out.append(b)
    header = re.sub(r"^ship = .*$", "ship = %s" % SHIP, header, count=1, flags=re.M)
    return "\n".join([header] + out + ([trailer] if trailer else []))


# ---------------------------------------------------------------- report: mass, CoM, drag area
def drag_cubes(path=PARTDB):
    """{craft part name: [(cube name, [6 faces (area, cd, depth)])]} from the game's PartDatabase.cfg (read-only).
    Face order XP, XN, YP, YN, ZP, ZN (part frame)."""
    out, url = {}, None
    for ln in open(path, encoding="utf-8", errors="replace"):
        s = ln.strip()
        if s.startswith("url = "):
            url = s[6:].rsplit("/", 1)[-1].replace("_", ".")
        elif s.startswith("cube = ") and url:
            name, rest = s[7:].split(",", 1)
            v = [float(t) for t in rest.split(",")]
            out.setdefault(url, []).append((name.strip(), [tuple(v[3 * k:3 * k + 3]) for k in range(6)]))
    return out


def _parts(path, db):
    header, blocks, _ = cb.split_parts(open(path).read())
    m = cb.masses(blocks, db)
    rows = {}
    for b in blocks:
        f = cb.fields(b)
        pid = f["part"][0]
        cfg = pid.rsplit("_", 1)[0]
        dry = db[0][cfg] + float(f.get("modMass", ["0"])[0])
        rows[pid] = dict(cfg=cfg, y=float(f["pos"][0].split(",")[1]), dry=dry, mod=float(f.get("modMass", ["0"])[0]),
                         res=f["_res"], m=m[pid], link=f.get("link", []))
    return rows, blocks


def _com(items):
    mt = sum(m for m, _ in items)
    return mt, sum(m * y for m, y in items) / mt


def recovery_items(rows, blocks, lf, mono_u=None, top_first=True):
    """(mass t, y) of the recovery stage = core after payload release (no SRBs, clamps on the pad) with `lf` LF
    units (+ matching oxidizer) left. top_first: the upper tank (Rockomax32) drains first (KSP stack flow);
    otherwise proportional. mono_u: MonoPropellant units left (None = as loaded)."""
    cargo = set(cb.subtree(rows[cb.DECOUPLER]["link"][0], blocks))
    items, tanks = [], []
    for pid, r in rows.items():
        if pid in cargo or r["cfg"] in ("solidBooster1-1", "noseCone", "radialDecoupler", "launchClamp1"):
            continue
        m = r["dry"]
        if "LiquidFuel" in r["res"]:
            tanks.append((pid, r))
            continue
        if "MonoPropellant" in r["res"]:
            u = r["res"]["MonoPropellant"] if mono_u is None else mono_u / 4.0
            m += u * 0.004
        items.append((m, r["y"]))
    tanks.sort(key=lambda t: -t[1]["y"])                       # upper first
    cap = [t[1]["res"]["LiquidFuel"] for t in tanks]
    if top_first:
        left, fill = lf, []
        for c in reversed(cap):                                # the lowest tank keeps its fuel longest
            fill.append(min(c, left))
            left -= fill[-1]
        fill = list(reversed(fill))
    else:
        fill = [lf * c / sum(cap) for c in cap]
    for (pid, r), u in zip(tanks, fill):
        items.append((r["dry"] + u * LF_T, r["y"]))
    return items


def cargo_items(rows, blocks, fairing_on=False):
    cargo = cb.subtree(rows[cb.DECOUPLER]["link"][0], blocks)
    out = []
    for pid in cargo:
        r = rows[pid]
        m = r["m"] - (0.0 if fairing_on else r["mod"] if r["cfg"] == "fairingSize2" else 0.0)
        out.append((m, r["y"] + (PORT_H / 2 if r["cfg"] == "dockingPort2" else 0.0)))
    return out


def axial_cda(rows, cubes, face=3):
    """Sum over the radially attached kit parts of area x cube drag of one face (3 = YN, the face meeting the flow in
    the engine-first descent; 2 = YP; 0 = XP side). The kit parts are only rotated about the vessel axis, so their
    part y axis is the vessel axis. Largest cube of the part (variants) = conservative. Unoccluded (KSP does not
    shadow surface-attached parts)."""
    tot = 0.0
    for pid, r in rows.items():
        if r["cfg"] not in ("RCSBlock.v2", "radialRCSTank"):
            continue
        tot += max(f[face][0] * f[face][1] for _, f in cubes[r["cfg"]])
    return tot


def report(base=SRC, dock=DST):
    db = cb.stock_db()
    cubes = drag_cubes()
    rb, bb = _parts(base, db)
    rd, bd = _parts(dock, db)
    lines = []

    def p(s):
        lines.append(s)
        print(s)
    pb, ib = cb.check(base, None, db)
    pd, idd = check(dock, db)
    p("check %s: %s" % (os.path.basename(dock), "OK" if not pd else pd))
    p("launch mass %.3f t (baseline %.3f, %+.3f); payload above the decoupler %.3f t (baseline %.3f, %+.3f)" % (
        idd["total_no_clamps"], ib["total_no_clamps"], idd["total_no_clamps"] - ib["total_no_clamps"],
        idd["payload"], ib["payload"], idd["payload"] - ib["payload"]))
    for tf in (True, False):
        tag = "top-first drain" if tf else "proportional drain"
        for lf_b, lf_d, mono, what in ((0, 0, 0, "dry (no LF, no mono)"),
                                       (910, 875, 27.5, "deorbit (baseline 910 LF flown; DOCK 875 LF, 27.5 u mono R0)"),
                                       (369, 335, 27.5, "touchdown (baseline 369 LF flown; DOCK 335 LF R0)"),
                                       (369, 369, 27.5, "touchdown, same LF 369 (kit effect only)")):
            mb, yb = _com(recovery_items(rb, bb, lf_b, top_first=tf))
            md, yd = _com(recovery_items(rd, bd, lf_d, mono_u=mono, top_first=tf))
            p("recovery stage, %s, %s: mass %.3f -> %.3f t (%+.3f), CoM y %.3f -> %.3f m (%+.3f)" % (
                what, tag, mb, md, md - mb, yb, yd, yd - yb))
        for lf in (910, 875):
            core = recovery_items(rd, bd, lf, top_first=tf)
            ms, ys = _com(core + cargo_items(rd, bd))
            p("docking stack (core %d LF + cargo, fairing shell gone), %s: %.2f t, CoM y %.3f m; RCS ring y %.2f "
              "-> thrust centre %+.2f m from CoM (R0 MC range +-1.5 m)" % (lf, tag, ms, ys, RCS_Y, RCS_Y - ys))
    yn, yp, xp = axial_cda(rd, cubes, 3), axial_cda(rd, cubes, 2), axial_cda(rd, cubes, 0)
    p("kit drag cube area x Cd (unoccluded): axial front face %.2f m2, aft face %.2f m2, side %.2f m2" % (yn, yp, xp))
    p("  vs measured core CdA (logs/g5_entry.json, T-WorksFineGF): brakes out 27.1 m2 -> %+.1f %%, brakes in 11.8 m2 "
      "-> %+.1f %% (static estimate; the mission re-measures the drag table per craft in orbit)" % (
          100 * yn / 27.1, 100 * yn / 11.8))
    return pd, lines


EXPECTED = ["parts lost: ['%s']" % ANTENNA]      # the one deliberate removal


def check(path=DST, db=None):
    probs, info = cb.check(path, SRC, db)
    return [p for p in probs if p not in EXPECTED], info


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "build":
        open(DST, "w").write(build())
        probs, info = check(DST)
        print("%s: %d parts, total %.2f t (%.2f t without clamps), payload %.3f t, %s" % (
            SHIP, info["n_parts"], info["total"], info["total_no_clamps"], info["payload"],
            "OK" if not probs else "PROBLEMS: %s" % probs))
    elif len(sys.argv) > 1 and sys.argv[1] == "report":
        report()
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
