"""Payload ballast variants of T-WorksFineGF (and a static checker for any .craft file).

Build: a stock LargeTank (Large Holding Tank, 2.0 t dry, 1.8 m, size 2) holding Ore (0.010 t/unit) is inserted between
the payload decoupler (Decoupler.2_4291537548) and the fairing base (fairingSize2_4291793832); every part above the
decoupler moves up by 1.8 m. The flown payload above the decoupler is 2.48 t from the part cfgs (the gate prints
13.8 - 11.4 = 2.4 t, rounded; the decoupler stays on the core), so Ore 2 -> 4.50 t (T-WorksFineGF-P45) and
Ore 152 -> 6.00 t (T-WorksFineGF-P60) (the earlier spec's 10 / 160 assumed 2.4 t and gives 4.58 / 6.08 t).
Check: parse all PART blocks, every link / attN / srfN / sym references an existing part, no duplicate part ids or
persistentIds, every original part keeps its staging fields, and the total / payload mass from the stock part cfgs
(read-only from the KSP install) + modMass + resources.
usage: craft_ballast.py build            (writes craft/VAB/T-WorksFineGF-P45.craft and -P60.craft)
       craft_ballast.py check FILE [ORIGINAL]
"""
import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from kspilot.paths import GAMEDATA  # noqa: E402  ($KSP_DIR/GameData, read-only)
SRC = os.path.join(ROOT, "craft", "VAB", "T-WorksFineGF.craft")
DECOUPLER = "Decoupler.2_4291537548"
FAIRING = "fairingSize2_4291793832"
TANK = "LargeTank_4291000045"            # new part ids: unused in the source craft (checked)
TANK_PID = "1450000045"
TANK_H = 1.8                             # LargeTank stack nodes at +-0.9 m
VARIANTS = {"P45": 2, "P60": 152}        # Ore units
STAGE_FIELDS = ("istg", "dstg", "sidx", "sqor", "sepI", "resPri")


# ---------------------------------------------------------------- craft file
def split_parts(text):
    """(header, [part block text], trailer). Each block starts at a top-level 'PART' line and ends at its '}'."""
    lines = text.split("\n")
    header, parts, cur, depth, trailer = [], [], None, 0, []
    for ln in lines:
        if cur is None:
            if ln == "PART":
                cur, depth = [ln], 0
            elif parts:
                trailer.append(ln)
            else:
                header.append(ln)
            continue
        cur.append(ln)
        s = ln.strip()
        if s == "{":
            depth += 1
        elif s == "}":
            depth -= 1
            if depth == 0:
                parts.append("\n".join(cur))
                cur = None
    if cur is not None:
        raise ValueError("unterminated PART block")
    return "\n".join(header), parts, "\n".join(trailer)


def fields(block):
    """Top-level key -> [values] of a PART block, plus its RESOURCE amounts as '_res' {name: amount}."""
    out, depth, res, node = {}, 0, {}, None
    for ln in block.split("\n")[1:]:
        s = ln.strip()
        if s == "{":
            depth += 1
            continue
        if s == "}":
            depth -= 1
            continue
        if depth == 1 and " = " in s:
            k, v = s.split(" = ", 1)
            out.setdefault(k, []).append(v)
        elif depth == 1 and s:
            node = s
        elif depth == 2 and node == "RESOURCE" and " = " in s:
            k, v = s.split(" = ", 1)
            if k == "name":
                res[v] = 0.0
                cur = v
            elif k == "amount":
                res[cur] = float(v)
    out["_res"] = res
    return out


def ref_of(v):
    """Part id referenced by an attN / srfN / link / sym value ('Null' for an empty node)."""
    if "," in v:
        node, rest = v.split(",", 1)
        if rest.startswith("Null"):
            return None
        if rest.split(",")[0] != rest:     # srfN: 'srfAttach,PARTID,collider,...'
            return rest.split(",")[0]
        return re.sub(r"_[-0-9.e]+\|.*$", "", rest)
    return v


def tree(blocks):
    ids = [fields(b)["part"][0] for b in blocks]
    parent = {}
    for b in blocks:
        f = fields(b)
        for c in f.get("link", []):
            parent[c] = f["part"][0]
    return ids, parent


def subtree(root, blocks):
    kids = {}
    for b in blocks:
        f = fields(b)
        kids[f["part"][0]] = f.get("link", [])
    out, todo = [], [root]
    while todo:
        p = todo.pop()
        out.append(p)
        todo += kids.get(p, [])
    return out


def _shift_pos(block, dy):
    def sh(m):
        x, y, z = (float(t) for t in m.group(2).split(","))
        return "%s%s,%s,%s" % (m.group(1), _num(x), _num(y + dy), _num(z))
    return re.sub(r"^(\tpos = )([-0-9.e,]+)$", sh, block, count=1, flags=re.M)


def _num(x):
    s = "%.9g" % x
    return s


def _set(block, key, value):
    new, n = re.subn(r"^(\t%s = ).*$" % re.escape(key), lambda m: m.group(1) + value, block, count=1, flags=re.M)
    assert n == 1, key
    return new


def tank_block(pos, ore):
    x, y, z = pos
    return "\n".join([
        "PART", "{",
        "\tpart = %s" % TANK, "\tpartName = Part", "\tpersistentId = %s" % TANK_PID,
        "\tpos = %s,%s,%s" % (_num(x), _num(y), _num(z)), "\tattPos = 0,0,0", "\tattPos0 = 0,1,0",
        "\trot = 0,0,0,1", "\tattRot = 0,0,0,1", "\tattRot0 = 0,0,0,1", "\tmir = 1,1,1", "\tsymMethod = Radial",
        "\tautostrutMode = Off", "\trigidAttachment = False", "\tistg = 1", "\tresPri = 0", "\tdstg = 1",
        "\tsidx = -1", "\tsqor = -1", "\tsepI = 1", "\tattm = 0", "\tsameVesselCollision = False",
        "\tmodCost = 0", "\tmodMass = 0", "\tmodSize = 0,0,0",
        "\tlink = %s" % FAIRING,
        "\tattN = top,%s_0|0.899999976|0_0|1|0_0|0.899999976|0_0|1|0" % FAIRING,
        "\tattN = bottom,%s_0|-0.899999976|0_0|-1|0_0|-0.899999976|0_0|-1|0" % DECOUPLER,
        "\tEVENTS", "\t{", "\t}",
        "\tACTIONS", "\t{",
        "\t\tToggleSameVesselInteraction", "\t\t{", "\t\t\tactionGroup = None",
        "\t\t\twasActiveBeforePartWasAdjusted = False", "\t\t}",
        "\t\tSetSameVesselInteraction", "\t\t{", "\t\t\tactionGroup = None",
        "\t\t\twasActiveBeforePartWasAdjusted = False", "\t\t}",
        "\t\tRemoveSameVesselInteraction", "\t\t{", "\t\t\tactionGroup = None",
        "\t\t\twasActiveBeforePartWasAdjusted = False", "\t\t}",
        "\t}",
        "\tPARTDATA", "\t{", "\t}",
        "\tMODULE", "\t{", "\t\tname = ModuleFuelJettison", "\t\tisEnabled = True", "\t\tstagingEnabled = True",
        "\t\tEVENTS", "\t\t{", "\t\t}", "\t\tACTIONS", "\t\t{", "\t\t}", "\t\tUPGRADESAPPLIED", "\t\t{", "\t\t}", "\t}",
        "\tMODULE", "\t{", "\t\tname = KOSNameTag", "\t\tisEnabled = True", "\t\tnameTag = ", "\t\tstagingEnabled = True",
        "\t\tEVENTS", "\t\t{", "\t\t}", "\t\tACTIONS", "\t\t{", "\t\t}", "\t\tUPGRADESAPPLIED", "\t\t{", "\t\t}", "\t}",
        "\tRESOURCE", "\t{", "\t\tname = Ore", "\t\tamount = %d" % ore, "\t\tmaxAmount = 1500", "\t\tflowState = True",
        "\t\tisTweakable = True", "\t\thideFlow = False", "\t\tisVisible = True", "\t\tflowMode = Both", "\t}",
        "}"])


def build(ore, ship, src=SRC):
    text = open(src).read()
    header, blocks, trailer = split_parts(text)
    ids, _ = tree(blocks)
    assert TANK not in ids and DECOUPLER in ids and FAIRING in ids
    above = set(subtree(FAIRING, blocks))
    out, dec_pos = [], None
    for b in blocks:
        pid = fields(b)["part"][0]
        if pid == DECOUPLER:
            dec_pos = [float(t) for t in fields(b)["pos"][0].split(",")]
            b = b.replace("\tlink = %s" % FAIRING, "\tlink = %s" % TANK)
            b = b.replace("\tattN = top,%s_" % FAIRING, "\tattN = top,%s_" % TANK)
            out.append(b)
            # decoupler top node +0.1 m, tank centre 0.9 m above it
            out.append(tank_block((dec_pos[0], dec_pos[1] + 0.1 + TANK_H / 2, dec_pos[2]), ore))
            continue
        if pid in above:
            b = _shift_pos(b, TANK_H)
            if pid == FAIRING:
                b = _set(b, "attPos0", "0,1.10000002,0")      # 0.9 (tank top node) + 0.2 (fairing bottom node)
                b = b.replace("\tattN = bottom,%s_" % DECOUPLER, "\tattN = bottom,%s_" % TANK)
        out.append(b)
    header = re.sub(r"^ship = .*$", "ship = %s" % ship, header, count=1, flags=re.M)
    sx, sy, sz = (float(t) for t in re.search(r"^size = (.*)$", header, re.M).group(1).split(","))
    header = re.sub(r"^size = .*$", "size = %s,%s,%s" % (_num(sx), _num(sy + TANK_H), _num(sz)), header, count=1,
                    flags=re.M)
    return "\n".join([header] + out + ([trailer] if trailer else []))


# ---------------------------------------------------------------- stock masses
def _cfg_nodes(text):
    """Minimal ConfigNode parser: yields (name, dict(values), [children]) top-level nodes."""
    text = text.lstrip("﻿")
    toks = []
    for ln in text.split("\n"):
        ln = ln.split("//")[0].strip()
        if not ln:
            continue
        ln = ln.replace("{", "\n{\n").replace("}", "\n}\n")
        toks += [t.strip() for t in ln.split("\n") if t.strip()]
    root = ("root", {}, [])
    stack, pending = [root], None
    for t in toks:
        if t == "{":
            node = (pending or "", {}, [])
            stack[-1][2].append(node)
            stack.append(node)
            pending = None
        elif t == "}":
            if len(stack) > 1:
                stack.pop()
        elif "=" in t:
            k, v = t.split("=", 1)
            stack[-1][1].setdefault(k.strip(), v.strip())
            pending = None
        else:
            pending = t
    return root[2]


def stock_db(gamedata=GAMEDATA):
    """{craft part name: (dry mass t, {res: maxAmount})}, {resource: density t/unit} from every PART cfg."""
    parts, dens = {}, {}
    for f in glob.glob(os.path.join(gamedata, "**", "*.cfg"), recursive=True):
        try:
            nodes = _cfg_nodes(open(f, encoding="utf-8", errors="replace").read())
        except Exception:
            continue
        for name, vals, kids in nodes:
            if name == "PART" and "name" in vals:
                try:
                    m = float(vals.get("mass", "0").split()[0])
                except ValueError:
                    continue
                parts.setdefault(vals["name"].replace("_", "."), m)
            elif name == "RESOURCE_DEFINITION" and "name" in vals and "density" in vals:
                dens[vals["name"]] = float(vals["density"])
    return parts, dens


def masses(blocks, db=None):
    parts, dens = db or stock_db()
    out = {}
    for b in blocks:
        f = fields(b)
        pid = f["part"][0]
        cfg = pid.rsplit("_", 1)[0]
        if cfg not in parts:
            raise KeyError("no stock cfg for %s" % cfg)
        res = sum(a * dens[r] for r, a in f["_res"].items())
        out[pid] = parts[cfg] + float(f.get("modMass", ["0"])[0]) + res
    return out


# ---------------------------------------------------------------- static check
def check(path, original=None, db=None):
    """Return (problems, info). info: n_parts, total, total_no_clamps, payload (t, parts above the decoupler)."""
    header, blocks, _ = split_parts(open(path).read())
    problems = []
    fs = [fields(b) for b in blocks]
    ids = [f["part"][0] for f in fs]
    pids = [f["persistentId"][0] for f in fs]
    if len(set(ids)) != len(ids):
        problems.append("duplicate part ids: %s" % sorted({i for i in ids if ids.count(i) > 1}))
    if len(set(pids)) != len(pids):
        problems.append("duplicate persistentIds")
    idset = set(ids)
    _, parent = tree(blocks)
    for f in fs:
        me = f["part"][0]
        for key in ("link", "attN", "srfN", "sym"):
            for v in f.get(key, []):
                r = ref_of(v)
                if r is not None and r not in idset:
                    problems.append("%s %s -> missing part %s" % (me, key, r))
        # an attN to another part must be mirrored by a parent/child link
        for v in f.get("attN", []):
            r = ref_of(v)
            if r is not None and not (parent.get(me) == r or parent.get(r) == me):
                problems.append("%s attN %s is neither its parent nor a child" % (me, r))
    roots = [i for i in ids if i not in parent]
    if len(roots) != 1:
        problems.append("expected one root part, got %s" % roots)
    if original:
        _, oblocks, _ = split_parts(open(original).read())
        ofs = {fields(b)["part"][0]: fields(b) for b in oblocks}
        for f in fs:
            o = ofs.get(f["part"][0])
            if o is None:
                continue
            for k in STAGE_FIELDS:
                if o.get(k) != f.get(k):
                    problems.append("%s %s changed %s -> %s" % (f["part"][0], k, o.get(k), f.get(k)))
        missing = set(ofs) - idset
        if missing:
            problems.append("parts lost: %s" % sorted(missing))
        byid = {f["part"][0]: f for f in fs}
        for f in fs:                        # added parts: not staged, same inverse stage as their parent
            me = f["part"][0]
            if me in ofs:
                continue
            if f["sidx"][0] != "-1" or f["sqor"][0] != "-1":
                problems.append("added part %s is in the staging sequence" % me)
            par = byid.get(parent.get(me))
            if par is not None and par["istg"] != f["istg"]:
                problems.append("added part %s istg %s != parent's %s" % (me, f["istg"], par["istg"]))
    m = masses(blocks, db)
    above = subtree(fields([b for b in blocks if fields(b)["part"][0] == DECOUPLER][0])["link"][0], blocks) \
        if DECOUPLER in idset else []
    info = dict(n_parts=len(blocks), total=sum(m.values()),
                total_no_clamps=sum(v for k, v in m.items() if not k.startswith("launchClamp")),
                payload=sum(m.get(k, 0.0) for k in above), decoupler=m.get(DECOUPLER, 0.0),
                ship=re.search(r"^ship = (.*)$", header, re.M).group(1))
    return problems, info


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "build":
        db = stock_db()
        for tag, ore in VARIANTS.items():
            ship = "T-WorksFineGF-%s" % tag
            dst = os.path.join(ROOT, "craft", "VAB", ship + ".craft")
            open(dst, "w").write(build(ore, ship))
            probs, info = check(dst, SRC, db)
            print("%s: %d parts, total %.2f t (%.2f t without clamps), payload %.3f t, %s" % (
                ship, info["n_parts"], info["total"], info["total_no_clamps"], info["payload"],
                "OK" if not probs else "PROBLEMS: %s" % probs))
    elif len(sys.argv) > 2 and sys.argv[1] == "check":
        probs, info = check(sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else None)
        print(info)
        print("OK" if not probs else "\n".join(probs))
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
