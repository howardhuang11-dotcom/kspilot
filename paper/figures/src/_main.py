"""Shared __main__ helper for the figure scripts."""
import os
import sys


def main(make):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import make_all
    make_all.run_one(make)
