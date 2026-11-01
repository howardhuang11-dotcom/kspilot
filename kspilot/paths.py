"""Location of the Kerbal Space Program installation.

Set the ``KSP_DIR`` environment variable to the game's root directory (the directory that contains ``GameData``).
When it is unset, the default Steam library location for the current platform is used:

  macOS    ~/Library/Application Support/Steam/steamapps/common/Kerbal Space Program
  Windows  C:\\Program Files (x86)\\Steam\\steamapps\\common\\Kerbal Space Program
  Linux    ~/.steam/steam/steamapps/common/Kerbal Space Program

This module has no third-party dependencies, so the offline tools can import it without kRPC.
"""
import os
import sys

STEAM_DEFAULTS = {
    "darwin": "~/Library/Application Support/Steam/steamapps/common/Kerbal Space Program",
    "win32": r"C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program",
    "linux": "~/.steam/steam/steamapps/common/Kerbal Space Program",
}


def default_ksp_dir(platform=None):
    """Default Steam install directory of KSP for a sys.platform value (Linux layout for unknown platforms)."""
    platform = platform or sys.platform
    key = "darwin" if platform == "darwin" else "win32" if platform.startswith(("win", "cygwin")) else "linux"
    return os.path.expanduser(STEAM_DEFAULTS[key])


def ksp_dir():
    """The KSP root directory: $KSP_DIR if set, otherwise the platform default."""
    return os.path.expanduser(os.environ.get("KSP_DIR") or default_ksp_dir())


KSP_DIR = ksp_dir()
GAMEDATA = os.path.join(KSP_DIR, "GameData")
