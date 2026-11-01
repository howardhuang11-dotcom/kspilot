"""The table writers use LF line endings on every platform (report Section 9).

Regenerates the divert and ablation tables into a temporary directory and checks that no CR byte is written and that the
files equal the committed ones (content and line endings).
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def test_csv_tables_are_lf_and_match_committed(tmp_path):
    os.makedirs(tmp_path / "tables")
    for script in ("divert_flight_record.py", "ablation_record.py"):
        subprocess.run([sys.executable, os.path.join(ROOT, "paper", "analysis", script), "--out", str(tmp_path)],
                       check=True, stdout=subprocess.DEVNULL)
    names = sorted(n for n in os.listdir(tmp_path / "tables") if n.endswith((".csv", ".md")))
    assert "divert_flight_record.csv" in names and "ablation_dispersion.csv" in names
    for n in names:
        new = (tmp_path / "tables" / n).read_bytes()
        assert b"\r" not in new, n
        assert new == open(os.path.join(ROOT, "paper", "tables", n), "rb").read(), n
