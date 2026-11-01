#!/usr/bin/env python3
"""Regenerate every main (Fig. 1-16) and supplementary (Supplementary Fig. 1-5) figure of the kspilot report from the
raw records in data/raw.

usage:
    python paper/figures/make_all.py [LOGS_DIR] [--screens DIR] [--gnc2 DIR] [--out DIR] [--only NAME ...]

LOGS_DIR   raw flight records (wd_*.out, *_landing.csv[.gz], experiments.jsonl, matrix_results.txt, rdv_r1_*.json);
           default: data/raw/logs of this repository (resolved from this file, independent of the working directory)
--screens  in-game screenshots (default: data/raw/screenshots). A missing screenshot leaves its panel empty with a note
--gnc2     simulation results (entry_*.json, divert_*.json, burn_attitude_id.json; default: data/raw/gnc2)
--out      output directory (default: paper/figures)
--only     module names to draw (see the table below); default: all

Figure files and the module (paper/figures/src/<module>.py) that draws them:
    fig1_mission_system         fig1_mission              Fig. 1
    fig2_campaign_timeline      fig2_campaign_timeline    Fig. 2 (reads paper/tables/flow_all_transcripts.csv)
    fig3_population_sensitivity fig3_population_sensitivity Fig. 3 (reads paper/tables/sensitivity.csv)
    fig4_trajectories           fig4_trajectories         Fig. 4
    fig5_landing_accuracy       fig5_accuracy             Fig. 5
    fig6_guidance_mishaps       fig6_guidance             Fig. 6
    fig7_gates                  fig7_gates                Fig. 7
    fig8_robustness_power       fig8_robustness           Fig. 8
    fig9_plant_id               fig9_plant_id             Fig. 9
    fig10_divert                fig10_divert              Fig. 10
    fig11_ablation              fig11_ablation            Fig. 11
    fig12_entry_counterfactual  fig12_entry_counterfactual Fig. 12
    fig13_management_loop       fig13_management_loop     Fig. 13 (management section; reads paper/tables/management_*.csv)
    fig14_management_timeline   fig14_management_timeline Fig. 14
    fig15_lessons               fig15_lessons             Fig. 15
    fig16_dispatches            fig16_dispatches          Fig. 16
    sfig1_separation_frames     sfig1_separation_frames   Supplementary Fig. 1
    sfig2_timing                sfig2_timing              Supplementary Fig. 2
    sfig3_offset_aim            sfig3_offset_aim          Supplementary Fig. 3
    sfig4_lights                sfig4_lights              Supplementary Fig. 4
    sfig5_rendezvous            sfig5_rendezvous          Supplementary Fig. 5

Requires matplotlib, numpy and Pillow (see paper/figures/requirements-fig.txt). The data populations and inclusion
rules are imported from paper/analysis/compute_stats.py, so the figures and the report statistics cannot diverge.
Each figure is written as a vector PDF and a 300 dpi PNG."""
import argparse
import importlib
import os
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "src"))
sys.path.insert(0, os.path.join(HERE, "..", "analysis"))
import compute_stats as cs  # noqa: E402

FIGURES = ["fig1_mission", "fig2_campaign_timeline", "fig3_population_sensitivity", "fig4_trajectories", "fig5_accuracy",
           "fig6_guidance", "fig7_gates", "fig8_robustness", "fig9_plant_id", "fig10_divert", "fig11_ablation",
           "fig12_entry_counterfactual", "fig13_management_loop", "fig14_management_timeline", "fig15_lessons", "fig16_dispatches",
           "sfig1_separation_frames", "sfig2_timing", "sfig3_offset_aim", "sfig4_lights", "sfig5_rendezvous"]

# simulation results (data/raw/gnc2) needed by a module; every other module reads only data/raw/logs and screenshots
NEEDS_GNC2 = {"fig6_guidance": ["entry_replay.json"],
              "fig12_entry_counterfactual": ["entry_replay.json", "entry_mc.json", "entry_mc_stress.json"],
              "fig9_plant_id": ["burn_attitude_id.json", "divert_measured_mc.json"],
              "fig10_divert": ["divert_optimal.json"]}
NEEDS_LOGS = {"sfig5_rendezvous": ["rdv_r1_rdv_20261022_014347.json"]}


def context(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="?", default=cs.DEFAULT_LOGS)
    ap.add_argument("--screens", default=cs.DEFAULT_SCREENS)
    ap.add_argument("--gnc2", default=cs.DEFAULT_GNC2)
    ap.add_argument("--out", default=HERE)
    ap.add_argument("--only", nargs="*", default=None)
    a = ap.parse_args(argv)
    return dict(logs=os.path.abspath(a.logs), screens=os.path.abspath(a.screens), gnc2=os.path.abspath(a.gnc2),
                out=os.path.abspath(a.out), only=a.only)


def preflight(ctx, names):
    """One readable error (and no traceback) if the raw records the selected figures need are missing."""
    unknown = [n for n in names if n not in FIGURES]
    if unknown:
        cs.die("unknown figure module(s): %s\nAvailable: %s" % (", ".join(unknown), ", ".join(FIGURES)))
    cs.require_logs(ctx["logs"])
    gnc2 = sorted(set(f for n in names for f in NEEDS_GNC2.get(n, [])))
    if gnc2:
        cs.require_dir(ctx["gnc2"], "simulation-result directory (--gnc2)", gnc2)
    logs = sorted(set(f for n in names for f in NEEDS_LOGS.get(n, [])))
    if logs:
        cs.require_dir(ctx["logs"], "raw-record directory", logs)
    if not os.path.isdir(ctx["screens"]):
        print("note: screenshot directory %s not found; the panels that show in-game screenshots (Fig. 1b, "
              "Supplementary Figs. 1, 3, 4) are left empty. %s" % (ctx["screens"], cs.RAW_HINT), file=sys.stderr)


def run_one(make):
    """Entry point for running a single figure script directly: python src/figX.py [LOGS_DIR] [options]."""
    ctx = context()
    cs.require_logs(ctx["logs"])
    print(make(ctx))


def main():
    ctx = context()
    names = ctx["only"] or FIGURES
    preflight(ctx, names)
    failed = []
    for n in names:
        t0 = time.time()
        try:
            mod = importlib.import_module(n)
            res = mod.make(ctx)
            print("%-26s ok   %5.1f s  %s" % (n, time.time() - t0, res))
        except Exception:
            failed.append(n)
            print("%-26s FAILED" % n)
            traceback.print_exc()
    if failed:
        sys.exit("failed: " + ", ".join(failed))


if __name__ == "__main__":
    main()
