#!/usr/bin/env python3
"""Regenerate every table (paper/tables/) and figure (paper/figures/) of the report from the raw records in data/raw.

usage:
    python paper/regenerate_all.py [--skip-figures] [--out-dir DIR]

Steps, each stopping the run with a non-zero exit status if it fails:
    1. paper/analysis/compute_stats.py --check-manifest   SHA-256 of data/raw against paper/tables/raw_manifest.sha256
    2. paper/analysis/compute_stats.py                    tables/*.csv, *.md, stats.json (and prints every quoted value)
    3. paper/analysis/divert_flight_record.py             tables/divert_flight_record.*
    4. paper/analysis/ablation_record.py                  tables/ablation_dispersion*.csv
    5. paper/figures/make_all.py                          Figs. 1-16 and Supplementary Figs. 1-5 (PDF and 300 dpi PNG)

Works from any working directory. Needs Python 3.9 or later; the figures need matplotlib, numpy and Pillow
(pip install -r paper/figures/requirements-fig.txt). Regenerated tables are content-identical to the committed ones, not guaranteed
bit-for-bit: every CSV and Markdown file is written with LF line endings on every platform, and the numbers in stats.json can
differ in the last digit (relative differences of about 1e-16, from floating-point summation and libm differences between
Python builds; tested to 1e-12). Figure bytes may differ slightly between matplotlib versions (PNG pixels were identical). --out-dir writes the tables and figures to DIR/tables and DIR/figures instead of
into the repository (the manifest check is unaffected)."""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--skip-figures", action="store_true")
    ap.add_argument("--out-dir", default=None)
    a = ap.parse_args()
    out = ["--out", os.path.abspath(a.out_dir)] if a.out_dir else []
    fig_out = ["--out", os.path.join(os.path.abspath(a.out_dir), "figures")] if a.out_dir else []
    py = sys.executable
    steps = [("check manifest", [py, os.path.join(HERE, "analysis", "compute_stats.py"), "--check-manifest"]),
             ("compute_stats", [py, os.path.join(HERE, "analysis", "compute_stats.py")] + out),
             ("divert_flight_record", [py, os.path.join(HERE, "analysis", "divert_flight_record.py")] + out),
             ("ablation_record", [py, os.path.join(HERE, "analysis", "ablation_record.py")] + out)]
    if not a.skip_figures:
        steps.append(("figures", [py, os.path.join(HERE, "figures", "make_all.py")] + fig_out))
    if a.out_dir:
        os.makedirs(os.path.join(os.path.abspath(a.out_dir), "tables"), exist_ok=True)
    for name, cmd in steps:
        print("== %s" % name, flush=True)
        rc = subprocess.call(cmd)
        if rc:
            sys.exit("regenerate_all: step '%s' failed (exit status %d)" % (name, rc))
    print("== all steps completed")


if __name__ == "__main__":
    main()
