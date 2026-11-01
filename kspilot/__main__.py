"""kspilot command line.

  python -m kspilot check   CRAFT PE_KM AP_KM INC [--no-recover]   pre-launch feasibility report only
  python -m kspilot launch  CRAFT PE_KM AP_KM INC                  check, then fly to orbit
  python -m kspilot mission CRAFT PE_KM AP_KM INC                  check, orbit, payload, return to the pad
  python -m kspilot land                                           land the active vessel on the pad now

The game must be running with the kRPC server (the AutoLab plugin loads the sandbox save).
"""
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PY = sys.executable


def main(argv):
    if len(argv) < 1 or argv[0] in ("-h", "--help"):
        print(__doc__)
        return 0
    cmd, args = argv[0], argv[1:]
    tools = os.path.join(ROOT, "tools")
    if cmd == "check":
        extra = ["norecover"] if "--no-recover" in args else []
        args = [a for a in args if not a.startswith("--")]
        return subprocess.call([PY, os.path.join(tools, "check.py")] + args + extra)
    if cmd == "launch":
        return subprocess.call([PY, os.path.join(tools, "fly_ascent.py")] + args)
    if cmd == "mission":
        return subprocess.call([PY, os.path.join(tools, "watchdog.py"), "--stall", "900", "--tries", "2", "--",
                                os.path.join(tools, "orbit_recover.py")] + args)
    if cmd == "land":
        return subprocess.call([PY, os.path.join(tools, "land_now.py")] + args)
    print("unknown command", cmd)
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
