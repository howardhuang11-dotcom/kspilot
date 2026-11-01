# Installation and Use

This document describes how to install kspilot, run the offline test suite and simulators, build the AutoLab game
plugin, and fly a mission in Kerbal Space Program (KSP). The software was developed and flight-tested on macOS
(Apple silicon, Steam edition of KSP). Windows and Linux are supported by the configuration described below but have
not been tested.

## 1. Tested configuration

| Component | Version |
|---|---|
| Python | 3.12 (3.10 or later is required) |
| Kerbal Space Program | 1.12.5 (no expansions) |
| kRPC server mod and Python client | 0.6.0 |
| Kerbal Reusability Expansion (KRE) mod | 2.9.3 (required only for the grid-fin craft) |
| .NET SDK (to build the plugin) | 8.0 |
| ffmpeg; OBS Studio with obs-websocket 5 | optional (recording tools only) |

## 2. Python environment

`requirements.txt` needs **Python 3.10 or later** (the pinned `krpc==0.6.0` and `numpy>=2.0` do not install on the
macOS system Python 3.9). Tested with 3.12.

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

Alternatively, `pip install -e ".[test]"` installs the package with the test dependencies. The optional recording
tools need `pip install -r requirements-optional.txt` (or the `recording` extra). Note that this installs
obsws-python, which is licensed under GPL-3.0-only (see `NOTICE`).

**Analysis only (no flight software, no game).** To regenerate the report's statistics and figures from the raw
records you do not need `requirements.txt`: `paper/analysis/compute_stats.py` uses only the Python standard library
(it runs on Python 3.9 as well), and the figure scripts need only

```sh
python3 -m pip install -r paper/figures/requirements-fig.txt
```

The raw records themselves are described in the report (Section 9) and, where released, are under `data/raw/`.

## 3. Offline verification (no game required)

The test suite runs without KSP. Flight-data fixtures in `tests/data` are used for the replay tests.

```sh
.venv/bin/python -m pytest -q
```

A small number of tests skip with a stated reason, for example when an optional external program (ffmpeg, Pillow,
the .NET SDK) is absent or when a test needs data that is not distributed. Complete flight logs can be supplied
through the `KSP_REPLAY_LOGS` environment variable (default: `logs/`).

### Offline tools in `tools/`

Every Python tool listed here prints a usage text with `--help` (checked by `tests/test_tools_help.py`) and needs no
game connection.

| Tool | Needs raw data? | Purpose |
|---|---|---|
| `tools/belly_mc.py` | no | vectorised Monte Carlo of the high-altitude aerodynamic guidance laws (theory experiment) |
| `tools/rdv_mc.py` | no | Monte Carlo of the terminal rendezvous and docking (results to `logs/rdv_mc_*.json`) |
| `tools/replay_sim.py` | **yes**: at least two GF0T landing CSVs (`*_orbit_T-WorksFineGF0T_landing.csv`) | descent replay simulator: identifies the plant from logged flights and replays them |
| `tools/aim_replay.py` | **yes**: one GF0T landing CSV (a copy is bundled in `tests/data`) | offset-aim replay check built on `replay_sim.py` |
| `tools/dr002_trade.py` | **yes**: the GF0T landing CSVs named in the script (`BASES`, and for `check` also `CHECK_FLIGHTS`) | recovery-architecture trade study on the identified replay model |

The tools that need flight logs look for them, in this order, in `--logs DIR`; or else `KSP_REPLAY_LOGS` (an explicit
directory is then the only place searched); or else `data/raw/logs/`, `logs/` and `tests/data/` of the repository
(plain `.csv` or `.csv.gz`). Only one GF0T landing CSV is bundled in `tests/data`, so `replay_sim.py validate` and
`dr002_trade.py` need the full set of logs, which belongs in `data/raw/logs/` of the released data. When data are
missing they print one line saying which files are needed and exit with status 2.

**Stale or removed.** `tools/montecarlo.py` and `tools/tune_mc.py` (a Monte Carlo of the terminal descent and its gain
tuner) were removed: the simulator had drifted from the flight code (it crashed with an `AttributeError` and, once
repaired, it landed 0 of 12 dispersed vehicles in a quick check, which is not what the flight code achieves) and the report does not use it as evidence. The replay-simulator Monte
Carlo of `dr002_trade.py` is likewise not evidence for any result in the report (Sections 5.1 and 4.7). The remaining
scripts in `tools/` (for example `hop.py`, `highhop.py`, `fly_ascent.py`, `land_now.py`, `orbit_recover.py`,
`airbrake_probe.py`, `power_check.py`, `check.py`) need a running game; `dm_mc.py`, `aero_authority.py`,
`aero_truth.py`, `calibrate_drag.py` and `director_replay.py` are development scripts that read logs which are not
released. They have no `--help` and are not supported.

**Not in this repository.** The entry-burn predictor-corrector replay simulator (`tools/gnc2_sim.py`) and the divert
envelope tool (`tools/divert_envelope.py`) that the report names in Section 5.3 were not released; only their outputs are released (`data/raw/gnc2/`, where present). The predictor-corrector entry-burn
cutoff of report Section 5.2 is likewise not in the flown or released flight code, so the simulation results that depend on it
(Tables 7 and 8, Figs 10a and 12) cannot be regenerated from this repository.

**Exploratory ablation switches.** `kspilot/landing.py` contains `KSP_ABL_*` environment switches (`NOBRAKES`,
`NODRAGLEARN`, `NAVNOISE`/`SEED`, `DRAGBIAS`, `TAG`) used for the exploratory ablation and dispersion flights of
report Section 4.8. They are off by default and, with all of them
unset, the landing code is bit-identical to the code without them (`tests/test_ablation.py`).

## 4. Game installation path (`KSP_DIR`)

All tools locate the game through the `KSP_DIR` environment variable, which must point to the game's root directory
(the directory that contains `GameData`). When it is not set, the default Steam location is assumed:

| Platform | Default `KSP_DIR` |
|---|---|
| macOS | `~/Library/Application Support/Steam/steamapps/common/Kerbal Space Program` |
| Windows | `C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program` |
| Linux | `~/.steam/steam/steamapps/common/Kerbal Space Program` |

## 5. Game-side prerequisites

1. Install the kRPC server mod 0.6.0 into `GameData`. Keep the default server settings (address 127.0.0.1, RPC
   port 50000) and enable automatic acceptance of client connections. The server should not be exposed beyond the
   local machine.
2. For the grid-fin craft (`T-WorksFineGF*`), install the Kerbal Reusability Expansion mod.
3. Build and install the AutoLab plugin (Section 6). The guidance code itself does not need it; it is needed for
   unattended operation (dismissal of modal dialogs, screenshots, and the input lock that the watchdog keeps alive).

## 6. Building the AutoLab plugin

The plugin is compiled against the game's own assemblies, which are referenced from `KSP_DIR` and are neither
distributed with this repository nor copied into the build output.

```sh
cd plugins/AutoLab
dotnet build -c Release -p:KSP_DIR="/path/to/Kerbal Space Program"
```

If the game's managed-assembly directory is in a non-standard location, pass `-p:KSPManaged=/path/to/Managed`
instead. Copy `bin/Release/net472/AutoLab.dll` to `$KSP_DIR/GameData/AutoLab/Plugins/`. On macOS,
`tools/restart_ksp.sh` performs the copy and restarts the game.

The plugin is not needed by the guidance code (which talks to the game only through kRPC); it is needed for unattended
operation. At the main menu it loads, or creates on first start, the sandbox save `AutoPilotLab`, and it dismisses
modal dialogs, takes screenshots and holds an input lock, so that the game can be driven without a person at the
keyboard. After the save has been created, copy the craft
files to be flown into it:

```sh
cp craft/VAB/*.craft "$KSP_DIR/saves/AutoPilotLab/Ships/VAB/"
```

The craft files are licensed under the GNU GPL version 3 (see `craft/SOURCES.md`).

## 7. Flying a mission

With the game running at the Space Center:

```sh
.venv/bin/python -m kspilot check   T-WorksFineGF 100 100 6   # pre-launch feasibility report only
.venv/bin/python -m kspilot mission T-WorksFineGF 100 100 6   # ascent, payload release, deorbit, entry, landing
```

`mission` runs `tools/orbit_recover.py` under the supervisor `tools/watchdog.py`, which restarts a stalled run and
keeps the AutoLab input lock alive (so unattended runs need the AutoLab plugin of Section 6). The interpreter used
for the supervised run is taken from `KSPILOT_PYTHON`, otherwise from `.venv`, otherwise the current interpreter. Logs are written to `logs/` (not version-controlled).
Do not interact with the game while a mission is in progress.

The batch scripts `tools/series.sh` and `tools/matrix_run.sh` (zsh) run flight series unattended. On macOS they
pause when the computer runs on battery below 20 % (via `pmset`); on other platforms this check is skipped.

## 8. Optional recording tools

`HOP_REC=full` records the whole mission. The real-time pipeline uses OBS Studio through obs-websocket; connection
settings are read from `.obs_ws.json` in the repository root (`{"host", "port", "password"}`; this file is
git-ignored and must never be committed). The OBS configuration directory can be overridden with `OBS_CONFIG_DIR`.
Post-production (`tools/edit_obs.py`, `tools/edit_video.py`) requires ffmpeg and Pillow. These tools are optional
and are not needed for guidance, simulation or testing.
