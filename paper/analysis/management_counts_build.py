#!/usr/bin/env python3
"""Build the management_*.csv tables behind the section "How the AI agents were managed" (Figs 13-16).

usage:
    python paper/analysis/management_counts_build.py --dev-docs DIR --subagents DIR [--out paper/tables]

This script documents HOW the committed tables were derived. It cannot be re-run from the public release, because its
inputs are private development files that are not released: DIR for --dev-docs holds DECISIONS.md (the decision record)
and LESSONS.md (the lessons file) of the development repository; DIR for --subagents holds the sub-agent transcript
index of the orchestrating session (agent-*.meta.json plus the first line of the matching agent-*.jsonl, which gives the
dispatch time in UTC). The figure scripts (fig11-fig14) read only the CSVs written here, which are committed.

Counting rules (all counts are single-coder unless stated):
  decision-log entries   every level-2 heading in DECISIONS.md; date from the clock time in the heading, the day advanced
                         when the clock time falls by more than 6 h against the previous entry (two roll-overs; start 20 Oct).
                         Class by keyword in the heading: secretary, andon/mishap, numbered DR, gate/result, other.
  dispatches             one per agent-*.meta.json; time = first transcript line, converted from UTC to ACDT (UTC+10:30);
                         role and workstream by keyword rules on the one-line description (the rules below are
                         abbreviated; the descriptions themselves are not published).
  lessons                the 78 numbered entries of LESSONS.md; category and finder were assigned by one coder (the AI
                         orchestrator) from the entry text and are listed in LESSON_CODING. 'Finder' is who or what exposed
                         the problem as the entry itself states it; when it states none, the entry is coded to the flight
                         or test outcome that showed it.
  events                 hand-curated from the decision record, the secretary reports and the mishap reports (EVENTS)."""
import argparse
import csv
import datetime as dt
import glob
import json
import os
import re
from collections import Counter

ACDT = dt.timedelta(hours=10, minutes=30)

# ---------------------------------------------------------------- lessons: number -> (category, finder)
CAT = dict(M="Model and measurement validity", X="Experiment and verification discipline", S="Control-software logic",
           V="Vehicle and actuator configuration", O="Operations and infrastructure", R="Reporting and conduct",
           D="Physical insight")
FND = dict(F="Flight or test outcome", H="Human lead (H.H.)", A="Sub-agent or audit", C="Orchestrator inspection")
_cat = ("M M S S X X O O R S S S S S S O M D D D S S S X S R S V M S D S S S S O V M O M V O M M X V S M "
        "M S S X S S M M X R R M X R O O O M X O R O M X R O M R M X").split()
_cat_n = {i + 1: c for i, c in enumerate(_cat)}
# finder overrides (default F); H = the entry says H.H.'s question, reminder or return exposed it
_fnd = {26: "H", 37: "H", 46: "H", 49: "H", 52: "H", 68: "H",
        58: "A", 59: "A", 71: "A", 73: "A",
        62: "C", 69: "C", 70: "C", 76: "C", 55: "C", 61: "C", 60: "C", 66: "C", 67: "C", 75: "F", 77: "A", 78: "A"}
LESSON_CODING = {n: (_cat_n[n], _fnd.get(n, "F")) for n in range(1, 79)}

# ---------------------------------------------------------------- events: (local time, kind, label, source)
# kinds: gate, andon, mishap, secretary, audit, exit_rule
EVENTS = [
    ("2026-10-20 21:13", "gate", "Regression gate 3/3 (DR-001)", "DECISIONS DR-001"),
    ("2026-10-20 21:40", "gate", "Regression gate after C-mode deletion", "DECISIONS"),
    ("2026-10-20 22:55", "gate", "Readiness review, identification flight (DR-003)", "DECISIONS DR-003"),
    ("2026-10-21 00:23", "gate", "Regression gate 3/3 (CA-3)", "DECISIONS"),
    ("2026-10-21 06:20", "gate", "Separation test B 3/3 (DR-007)", "DECISIONS DR-007"),
    ("2026-10-21 07:53", "gate", "Robustness matrix 8/8", "DECISIONS"),
    ("2026-10-21 10:52", "gate", "Regression gate (rec v2)", "DECISIONS"),
    ("2026-10-21 12:12", "gate", "Regression gate (rec v2 fix 2)", "DECISIONS"),
    ("2026-10-21 14:00", "gate", "Regression gate (fix round 3)", "DECISIONS"),
    ("2026-10-21 15:52", "gate", "Entry gates (DR-010)", "DECISIONS"),
    ("2026-10-21 16:15", "gate", "Entry gate (rec v3)", "DECISIONS"),
    ("2026-10-21 18:00", "gate", "Reliability block 3/3", "DECISIONS"),
    ("2026-10-21 21:05", "gate", "Pad night check (lights)", "DECISIONS"),
    ("2026-10-22 00:30", "gate", "Offset-aim merge gates", "DECISIONS"),
    ("2026-10-22 02:19", "gate", "R1-c review accepted", "DECISIONS"),
    ("2026-10-22 03:28", "gate", "R1-d power-on gates (third attempt)", "DECISIONS"),
    ("2026-10-20 21:04", "andon", "Stop after nine failed fins-only flights (retrospective)", "LESSONS 57"),
    ("2026-10-21 00:05", "andon", "Second consecutive failure, verification block", "DECISIONS"),
    ("2026-10-21 05:50", "andon", "Vehicle lost at booster separation (matrix case)", "DECISIONS MISHAP"),
    ("2026-10-21 14:35", "andon", "Recording flight 3.8 km short", "DECISIONS MISHAP"),
    ("2026-10-22 02:28", "andon", "Divert plant refuted by flight", "DECISIONS"),
    ("2026-10-22 03:02", "andon", "R1-d second consecutive failure", "DECISIONS ANDON"),
    ("2026-10-20 23:36", "mishap", "Investigation: Block 1 mission 1 (CA-1 to CA-4)", "dispatch index"),
    ("2026-10-21 05:50", "mishap", "Investigation: separation loss", "MISHAP_inc45"),
    ("2026-10-21 14:32", "mishap", "Investigation: recording-mode stalls", "dispatch index"),
    ("2026-10-22 02:43", "mishap", "Investigation: R1-d force-gate abort", "MISHAP_R1D"),
    ("2026-10-22 03:05", "exit_rule", "Divert: option O4 chosen with automatic O1 fallback", "DECISIONS"),
    ("2026-10-22 03:25", "exit_rule", "Exit rule triggered: O1 (flown law frozen)", "DECISIONS"),
]

# ---------------------------------------------------------------- dispatch classification
ROLE_RULES = [
    ("Secretary", r"secretary"),
    ("Direction audit", r"project-level|project-direction|process audit"),
    ("Andon panel", r"^andon: "),
    ("Investigator", r"mishap|investigate|root-cause|reproduce|andon r1-d|same-code|zero rcs|check the zero"),
    ("Review, red team, QA", r"red team|red-team|review|independent|qa |critic|audit|benchmark|check"),
]
WS_RULES = [
    ("Process oversight", r"secretary|project-level|project-direction|process audit"),
    ("Docking and rendezvous", r"docking|dock |\bdock\b|rcs|\br1\b|r1-|r1 prep|refuel"),
    ("Recording, HUD and video", r"hud|director|video|recording|obs |post-production|broadcast|data-vis|media studies|"
                                 r"lights|frame gaps|drag-table settle|mute ksp|deadman|boredom|shot-length"),
    ("Report and release", r"paper|report|reviewer|figure|open-source|citation|every number|freeze|ablation|critic|"
                            r"management section|raw data|tools fixes|campaign|typeset|apa|disk cleanup|disk space|"
                            r"broadcast"),
    ("Ascent, orbit and flight operations", r"ascent|mechjeb|matrix|c1\+c2|c3\+c4|separation|power|morning|daylight|monitor|"
                                     r"dr-010 fixes|robust|45|payload"),
]
WS_DEFAULT = "Landing, entry and aerodynamics"


RE_START = r"^(fix|merge|implement|build|write|prepare|plan|draft|add|set up|restyle|produce|typeset|ablation|" \
           r"research|find|summarize|recalibrate|finalize|video|author|paper|tools|raw data)"
WS_OVERRIDE = [(r"g-fold|open-source ksp autopilot", "Landing, entry and aerodynamics"),
               (r"stock rocket craft|c1-c4|max_dvn|morning report|recording flight: daylight|mute ksp|disk |"
                r"hud next label", "Ascent, orbit and flight operations")]


def classify(desc):
    d = desc.lower()
    role = "Responsible engineer"
    if not re.search(RE_START, d):
        for r, pat in ROLE_RULES:
            if re.search(pat, d):
                role = r
                break
    ws = WS_DEFAULT
    for w, pat in WS_RULES:
        if re.search(pat, d):
            ws = w
            break
    for pat, w in WS_OVERRIDE:
        if re.search(pat, d):
            ws = w
    return role, ws


def read_dispatches(sub):
    rows = []
    for f in sorted(glob.glob(os.path.join(sub, "agent-*.meta.json"))):
        m = json.load(open(f))
        j = f.replace(".meta.json", ".jsonl")
        ts = json.loads(open(j).readline())["timestamp"]
        t = dt.datetime.fromisoformat(ts.replace("Z", "+00:00")).replace(tzinfo=None) + ACDT
        role, ws = classify(m["description"])
        rows.append((t, role, ws, m.get("spawnDepth", 1), m.get("model") or "inherited"))
    rows.sort()
    return rows


def read_decisions(path):
    rows, day, prev = [], dt.date(2026, 10, 20), None
    for ln, line in enumerate(open(path, encoding="utf-8").read().split("\n"), 1):
        if not line.startswith("## "):
            continue
        mm = re.search(r"(\d\d):(\d\d)", line)
        minutes = int(mm.group(1)) * 60 + int(mm.group(2))
        if prev is not None and minutes < prev - 360:
            day += dt.timedelta(days=1)
        prev = minutes
        h = line.lower()
        if "secretary" in h or "project audit" in h:
            cls = "secretary or direction audit"
        elif "andon" in h or "mishap" in h:
            cls = "andon or mishap"
        elif re.search(r"dr-\d{3}", h):
            cls = "numbered decision (DR)"
        elif re.search(r"gate|pass|frr|go$|result|complete", h):
            cls = "gate or flight result"
        else:
            cls = "other decision, correction or status"
        t = dt.datetime.combine(day, dt.time(minutes // 60, minutes % 60))
        rows.append((ln, t, cls, bool(re.search(r"dr-\d{3}", h))))
    return rows


def distinct_dr(path):
    return len(set(re.findall(r"DR-\d{3}", open(path, encoding="utf-8").read())))


def read_lessons(path):
    t = open(path, encoding="utf-8").read()
    ents = [e for e in re.split(r"\n(?=\d+\. )", t) if re.match(r"\d+\. ", e)]
    rep = 0
    for e in ents:
        n = int(e.split(".")[0])
        if n > 1 and re.search("\u91cd\u72af|\u53c8\u4e00\u4f8b|\u53d8\u79cd|\u518d\u72af", e):
            rep += 1
    return len(ents), rep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev-docs", required=True)
    ap.add_argument("--subagents", required=True)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tables"))
    a = ap.parse_args()
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    fmt = "%Y-%m-%d %H:%M"

    disp = read_dispatches(a.subagents)
    with open(os.path.join(out, "management_dispatches.csv"), "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["index", "dispatch_local_acdt", "role", "workstream", "spawn_depth", "model_setting"])
        for i, (t, role, ws, depth, model) in enumerate(disp, 1):
            w.writerow([i, t.strftime(fmt), role, ws, depth, model])

    dec = read_decisions(os.path.join(a.dev_docs, "DECISIONS.md"))
    with open(os.path.join(out, "management_decision_log.csv"), "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["entry", "time_local_acdt", "class", "numbered_dr"])
        for i, (ln, t, cls, dr) in enumerate(dec, 1):
            w.writerow([i, t.strftime(fmt), cls, int(dr)])

    n_les, n_rep = read_lessons(os.path.join(a.dev_docs, "LESSONS.md"))
    assert n_les == len(LESSON_CODING), (n_les, len(LESSON_CODING))
    with open(os.path.join(out, "management_lessons.csv"), "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["lesson", "category", "finder"])
        for n in sorted(LESSON_CODING):
            c, fd = LESSON_CODING[n]
            w.writerow([n, CAT[c], FND[fd]])

    with open(os.path.join(out, "management_events.csv"), "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["time_local_acdt", "kind", "label", "source"])
        for t, k, lab, src in sorted(EVENTS):
            w.writerow([t, k, lab, src])
        for t, role, ws, depth, model in disp:      # audit-type dispatches go to the same event table
            if role in ("Secretary", "Direction audit"):
                w.writerow([t.strftime(fmt), "secretary" if role == "Secretary" else "audit",
                            "Secretary audit dispatched" if role == "Secretary" else "Independent direction audit dispatched",
                            "dispatch index"])

    sec = [t for t, r, *_ in disp if r == "Secretary"]
    gaps = [round((b - a_).total_seconds() / 3600, 2) for a_, b in zip(sec, sec[1:])]
    c_role, c_ws = Counter(r for _, r, *_ in disp), Counter(w_ for _, _, w_, *_ in disp)
    c_cls = Counter(c for _, _, c, _ in dec)
    c_cat, c_fnd = Counter(LESSON_CODING[n][0] for n in LESSON_CODING), Counter(LESSON_CODING[n][1] for n in LESSON_CODING)
    kinds = Counter(k for _, k, *_ in EVENTS)
    rows = [("dispatches_total", len(disp), "agent-*.meta.json in the session's sub-agent index"),
            ("dispatches_depth2", sum(1 for d in disp if d[3] == 2), "sub-agents spawned by sub-agents"),
            ("dispatches_model_explicit", sum(1 for d in disp if d[4] != "inherited"), "model named at dispatch"),
            ("decision_log_entries", len(dec), "level-2 headings in DECISIONS.md"),
            ("decision_log_numbered_dr", sum(1 for d in dec if d[3]), "headings naming a DR-nnn record"),
            ("decision_log_distinct_dr", distinct_dr(os.path.join(a.dev_docs, "DECISIONS.md")),
             "distinct DR-nnn identifiers anywhere in DECISIONS.md"),
            ("lessons_total", n_les, "numbered entries in LESSONS.md"),
            ("lessons_citing_earlier_repeat", n_rep, "entries that say they repeat or vary an earlier lesson (keyword count)"),
            ("secretary_audits_dispatched", len(sec), "dispatches whose description names the secretary"),
            ("secretary_gaps_hours", ";".join(map(str, gaps)), "hours between consecutive secretary dispatches"),
            ("andons_recorded", kinds["andon"], "EVENTS kind andon"),
            ("mishap_investigations", kinds["mishap"], "EVENTS kind mishap"),
            ("gate_events_listed", kinds["gate"], "EVENTS kind gate (listed in the decision record headings)")]
    rows += [("role:" + k, v, "dispatch role") for k, v in sorted(c_role.items())]
    rows += [("workstream:" + k, v, "dispatch workstream") for k, v in sorted(c_ws.items())]
    rows += [("decision_class:" + k, v, "decision-log class") for k, v in sorted(c_cls.items())]
    rows += [("lesson_category:" + CAT[k], v, "single coder") for k, v in sorted(c_cat.items())]
    rows += [("lesson_finder:" + FND[k], v, "single coder") for k, v in sorted(c_fnd.items())]
    with open(os.path.join(out, "management_counts.csv"), "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["key", "value", "method"])
        for r in rows:
            w.writerow(r)
    for r in rows:
        print(*r, sep=" | ")


if __name__ == "__main__":
    main()
