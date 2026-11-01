#!/bin/zsh
# Restart KSP with the freshly built AutoLab plugin; returns when kRPC is up in the Space Center.
# macOS only (Steam's KSP.app, BSD sed). On other platforms copy the plugin and start the game by hand.
# KSP_DIR: game root (default: the macOS Steam library); KSPILOT_PYTHON: interpreter (default: the repository venv).
if [[ $(uname) != Darwin ]]; then
  echo "restart_ksp.sh supports macOS only: copy plugins/AutoLab/bin/Release/net472/AutoLab.dll to \$KSP_DIR/GameData/AutoLab/Plugins/ and start KSP manually" >&2
  exit 1
fi
K="${KSP_DIR:-$HOME/Library/Application Support/Steam/steamapps/common/Kerbal Space Program}"
R="$(cd "$(dirname "$0")/.." && pwd)"
PY="${KSPILOT_PYTHON:-$R/.venv/bin/python}"
pkill -f "MacOS/KSP"; sleep 5
mkdir -p "$K/GameData/AutoLab/Plugins"
cp "$R/plugins/AutoLab/bin/Release/net472/AutoLab.dll" "$K/GameData/AutoLab/Plugins/"
# window size (pixels; Retina: half that in points) — override with KSP_W / KSP_H
W=${KSP_W:-1920}; H=${KSP_H:-1080}      # 1080p recordings
sed -i '' -e "s/^SCREEN_RESOLUTION_WIDTH = .*/SCREEN_RESOLUTION_WIDTH = $W/" -e "s/^SCREEN_RESOLUTION_HEIGHT = .*/SCREEN_RESOLUTION_HEIGHT = $H/" "$K/settings.cfg"
open -g "$K/KSP.app"; sleep 40
for i in $(seq 1 60); do nc -z 127.0.0.1 50000 2>/dev/null && break; sleep 5; done
"$PY" -c "
import sys; sys.path.insert(0, '$R')
from kspilot import core
c = core.connect('restart')
print('KSP up:', core.wait_until(lambda: str(c.krpc.current_game_scene).endswith('space_center'), 120, 2))"
