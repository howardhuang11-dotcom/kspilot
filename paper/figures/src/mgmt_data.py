"""Readers for the management_*.csv tables (paper/tables). They are committed, so the management figures need no
private development files; paper/analysis/management_counts_build.py documents how they were derived."""
import csv
import datetime as dt
import os

TABLES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tables")
FMT = "%Y-%m-%d %H:%M"


def rows(name):
    with open(os.path.join(TABLES, name), newline="") as f:
        return list(csv.DictReader(f))


def when(r, key):
    return dt.datetime.strptime(r[key], FMT)


def counts():
    return {r["key"]: r["value"] for r in rows("management_counts.csv")}
