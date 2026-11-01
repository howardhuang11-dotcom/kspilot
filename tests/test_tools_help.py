"""Smoke test for the offline tools that docs/INSTALL.md advertises: each prints a usage text for --help (exit 0,
within 60 s, no side effects), and the tools that need flight logs fail gracefully (one line on stderr, exit status
2, no traceback) when KSP_REPLAY_LOGS points to a directory without them."""
import os
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.join(ROOT, "tools")
OFFLINE = ("replay_sim.py", "belly_mc.py", "rdv_mc.py", "dr002_trade.py", "aim_replay.py")
NEEDS_LOGS = {"replay_sim.py": ["validate"], "dr002_trade.py": ["run"], "aim_replay.py": []}


def run(tool, args, cwd, env_extra=None):
    env = dict(os.environ)
    env.pop("KSP_REPLAY_LOGS", None)
    env.update(env_extra or {})
    return subprocess.run([sys.executable, os.path.join(TOOLS, tool)] + list(args), cwd=str(cwd), env=env,
                          capture_output=True, text=True, timeout=60, stdin=subprocess.DEVNULL)


@pytest.mark.parametrize("tool", OFFLINE)
def test_help_prints_usage(tool, tmp_path):
    r = run(tool, ["--help"], tmp_path)
    assert r.returncode == 0, r.stderr[-500:]
    assert "usage" in r.stdout.lower()
    assert "Traceback" not in r.stderr
    assert list(tmp_path.iterdir()) == []              # --help writes nothing


@pytest.mark.parametrize("tool", sorted(NEEDS_LOGS))
def test_missing_logs_fail_gracefully(tool, tmp_path):
    empty = tmp_path / "empty"
    empty.mkdir()
    r = run(tool, NEEDS_LOGS[tool], tmp_path, {"KSP_REPLAY_LOGS": str(empty)})
    assert r.returncode == 2, (r.stdout[-300:], r.stderr[-500:])
    assert "Traceback" not in r.stderr
    lines = [x for x in r.stderr.splitlines() if x.strip()]
    assert len(lines) == 1 and "needs" in lines[0] and "KSP_REPLAY_LOGS" in lines[0], lines
    assert r.stdout.strip() == ""
