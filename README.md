# kspilot

kspilot is an autonomous flight system for the spaceflight game Kerbal Space Program (KSP). Through the kRPC remote procedure call interface, it launches a vehicle to a commanded orbit, releases the payload, and returns the core stage to a powered vertical landing on the launch pad. Every mission phase is implemented in this repository, without third-party autopilot code: pre-launch feasibility check, ascent guidance, booster-separation gating, deorbit window and electric-power planning, an entry burn, airbrake drag modulation, aerodynamic glide steering, and the landing burn.

All results were obtained in a simulation game with simplified physics and exact state knowledge. They describe the behaviour of this software in that environment and make no claim about physical launch vehicles.

## Results

The results come from a series of 28 development missions. The window was defined after the flights, it starts right after two failed missions, and the flight code changed within it, so the series records the system as it developed. The proportions are not a reliability estimate, and every interval is nominal.

- 26 of 28 complete missions from the pad met the landing criterion (miss below 20 m, touchdown speed below 4 m/s, upright). Under other population definitions the proportion ranges from 26 of 27 to 26 of 34. The median horizontal miss of the 26 successful landings was 1.5 m (range 1.2 to 2.3 m). This miss is dominated by a fixed westward offset of unknown cause and depends on the start state: the same law misses by a median of 3.7 m from a checkpoint 4 km above the pad.
- A one-factor robustness matrix (inclinations 0 to 90 degrees, orbits 80 to 110 km, payloads 2.48 to 6.00 t) passed all eight cases in nine attempts. The 45-degree case needed a second attempt after the first was lost on ascent, and four of the eight cases (28 degrees, 80 km, 45 degrees, 90 degrees) were flown with a raised deorbit-window threshold.
- A 22-flight ablation and dispersion study showed that the airbrakes and the glide are essential: removing either crashed every flight (two each). A test flight with a 50 m injected landing-burn offset landed 2.1 m from the pad. Eight earlier large-offset landing burns closed only 19 to 111 m of their offset; they were flown on seven code revisions with software attitude limits, so they do not measure a vehicle or law limit.
- A docking extension flew a rendezvous hold 1 km from a station and the first power-on of its reaction-control thrusters.
- Items from replay simulations (a proposed entry-burn predictor-corrector cutoff, a convex-planner divert envelope) are design studies and not findings. The cutoff is not in the flown or released code.

Definitions, exclusions, failures, limitations and the sensitivity of the proportion to the population definition are in the technical report. A confirmatory campaign and a comparison with MechJeb2 and BoosterGuidance are planned.

## Video

[`media/kspilot_full_mission_v7.mp4`](media/kspilot_full_mission_v7.mp4) (8 min 49 s, 27 MB) shows one complete autonomous mission recorded: lift-off, booster separation, orbit and payload release, deorbit burn, entry and the powered landing on the launch pad. Long coasts in orbit are cut; all other segments play in real time. The overlay is drawn by the project's AutoLab plugin.

## Technical report

The report [`paper/kspilot_report.md`](paper/kspilot_report.md) is the system and development report: system description and test infrastructure, guidance and control methods as flown, the development flights, design options evaluated in simulation only, how the AI agents were managed, discussion, conclusions and reproducibility. Supplementary notes, tables and figures are in [`paper/supplementary.md`](paper/supplementary.md). The report is typeset with `paper/latex/build_pdf.sh`, which needs pandoc 3.1 or later, tectonic (downloads LaTeX packages on first use) and poppler (`brew install pandoc tectonic poppler`).

## Repository contents

| Path | Content |
|---|---|
| `kspilot/` | Flight software (Python package): guidance, mission sequencing, impact prediction, landing, power management |
| `plugins/AutoLab/` | In-game helper plugin (C# source) for unattended operation, cameras, and overlay |
| `tools/` | Mission runners, flight supervisor, replay simulator, Monte Carlo and calibration tools (not `gnc2_sim.py` and `divert_envelope.py`: see below), and `sanitize_logs.py`, the transform that produced `data/raw/` |
| `tests/` | Offline test suite, which runs without the game |
| `craft/` | Vehicle definition files and their provenance |
| `data/` | Planet constants and atmosphere samples measured in the game |
| `data/raw/` | Sanitised raw flight records from which every table and figure of the report is regenerated; `ORIGINAL_SHA256.txt` holds the SHA-256 of the unsanitised originals |
| `paper/` | System and development report, analysis scripts, generated tables and figures |
| `media/` | Full-mission video |
| `docs/INSTALL.md` | Installation, offline verification, plugin build, and flight instructions |

## Installation and quick start

See [`docs/INSTALL.md`](docs/INSTALL.md) for the tested configuration (KSP 1.12.5, kRPC 0.6.0, Python 3.10 or later), installation of the Python environment, the offline test suite, the plugin build, and the commands to fly a mission. We recommend running the offline tests and simulators first. Flights require a licensed copy of KSP and the kRPC server mod, neither of which is distributed with this repository.

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m pytest -q          # offline test suite, no game needed
```

## Reproducing the tables and figures

The sanitised raw records are in `data/raw/`, and one command regenerates every statistic, table and figure of the report from them:

```sh
python paper/regenerate_all.py                            # verifies data/raw, then regenerates tables and figures
python paper/analysis/compute_stats.py --check-manifest   # only verify the raw files
```

The analysis script `compute_stats.py` runs on Python 3.9 or later with the standard library only; the figure scripts need `pip install -r paper/figures/requirements-fig.txt`; the flight software needs Python 3.10 or later. KSP is not deterministic across sessions, so re-flown missions reproduce statistics and not individual values.

**What does not reproduce from this repository** (details in report Section 9):

- **Revision hashes.** The revision hashes in the tables of the report (the "Code rev." columns) refer to the development repository and do not resolve here, and the "385 commits" of the report cannot be verified from this repository. The code that flew each mission cannot be identified from the history.
- **Simulation tools.** `tools/gnc2_sim.py` (entry-burn replay simulator) and `tools/divert_envelope.py` (convex-planner divert envelope) are not in this repository, and the predictor-corrector entry-burn cutoff of report Section 5.2 is not in the flown or released code. The simulation results of Tables 7 and 8 and Figs 10a and 12 are re-plotted from stored outputs in `data/raw/gnc2/` and cannot be re-run.
- **Not released:** the decision record and the lessons file; numbers that were cited only from them cannot be regenerated.
- **Ablation switches.** The `KSP_ABL_*` switches of report Section 4.8 are in `kspilot/landing.py` (off by default, bit-identical when unset), but the 22 flights were flown on development revisions.
- `tools/montecarlo.py` and `tools/tune_mc.py` (stale landing-burn simulators) were removed. `replay_sim.py`, `aim_replay.py` and `dr002_trade.py` need landing CSVs and exit with a one-line message if they are missing.

## Citation

If you use this software, please cite it using the metadata in [`CITATION.cff`](CITATION.cff):

> Huang, H. & Claude (2026). *kspilot: an autonomous launch-and-booster-return controller for Kerbal Space Program: system and development report* (software release 1.0.0, report version 2.3).

## Licence

| Part | Licence |
|---|---|
| Source code (`kspilot/`, `tools/`, `plugins/`, `tests/`, build files, analysis and figure scripts) | GNU General Public License v3.0 or later, GPL-3.0-or-later ([`LICENSE`](LICENSE)) |
| Text, figures and data (`paper/`, `docs/`, `data/`, `tests/data/`) | Creative Commons Attribution-ShareAlike 4.0 International, CC BY-SA 4.0 ([`LICENSE-docs`](LICENSE-docs)) |
| Vehicle definition files (`craft/`) | GNU General Public License version 3, because they derive from GPL-3.0 sources ([`craft/LICENSE`](craft/LICENSE), `craft/SOURCES.md`) |

Both licences are share-alike: a modified or derived version of the code, text, figures or data must be released under the same licence, with attribution. Citing the work does not create that obligation; copyright binds adaptations and redistribution, not references. Third-party files keep their own licences (see [`NOTICE`](NOTICE)).

Third-party software, trademarks, and acknowledgements are listed in [`NOTICE`](NOTICE). Kerbal Space Program is a trademark of Take-Two Interactive Software, Inc. This project is not affiliated with or endorsed by Take-Two Interactive, Private Division, or Squad, and it contains no game files.

## Acknowledgements

This project builds on kRPC, on the grid fins of the Kerbal Reusability Expansion modification, and on community vehicle designs listed in the craft provenance file. The development of the software and its documentation was assisted by large language models; Section 6 of the technical report describes this use.
