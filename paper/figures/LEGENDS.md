# Figure map

The figure legends are in the report (`paper/kspilot_report.md`, Figs. 1-16) and in `paper/supplementary.md`
(Supplementary Figs. 1-5). They are not duplicated here. This file only maps each figure to its file, script and inputs.

All figures are drawn by `python paper/figures/make_all.py` (or `python paper/regenerate_all.py`) from the raw records in
`data/raw` (`logs/`, `gnc2/`, `screenshots/`; see `data/raw/README.md`). Each figure is written as a vector PDF and a
300 dpi PNG. Scripts are in `paper/figures/src/`; `data.py` and `paper/analysis/compute_stats.py` supply the shared
parsers and inclusion rules, so the figures and the report statistics use the same populations.

| Figure | File (`paper/figures/`) | Script (`src/`) | Inputs |
|---|---|---|---|
| Fig. 1 | `fig1_mission_system` | `fig1_mission.py` | logs (transcripts, landing CSVs, experiment log); screenshots (panel b) |
| Fig. 2 | `fig2_campaign_timeline` | `fig2_campaign_timeline.py` | `paper/tables/flow_all_transcripts.csv` |
| Fig. 3 | `fig3_population_sensitivity` | `fig3_population_sensitivity.py` | `paper/tables/sensitivity.csv` |
| Fig. 4 | `fig4_trajectories` | `fig4_trajectories.py` | logs |
| Fig. 5 | `fig5_landing_accuracy` | `fig5_accuracy.py` | logs |
| Fig. 6 | `fig6_guidance_mishaps` | `fig6_guidance.py` | logs; gnc2 (`entry_replay.json`) |
| Fig. 7 | `fig7_gates` | `fig7_gates.py` | logs (experiment log) |
| Fig. 8 | `fig8_robustness_power` | `fig8_robustness.py` | logs (transcripts, matrix log) |
| Fig. 9 | `fig9_plant_id` | `fig9_plant_id.py` | gnc2 (`burn_attitude_id.json`, `divert_measured_mc.json`); logs (landing CSVs) |
| Fig. 10 | `fig10_divert` | `fig10_divert.py` | gnc2 (`divert_optimal.json`); logs via `paper/analysis/divert_flight_record.py` |
| Fig. 11 | `fig11_ablation` | `fig11_ablation.py` | logs via `paper/analysis/ablation_record.py` |
| Fig. 12 | `fig12_entry_counterfactual` | `fig12_entry_counterfactual.py` | gnc2 (`entry_replay.json`, `entry_mc.json`, `entry_mc_stress.json`) |
| Fig. 13 | `fig13_management_loop` | `fig13_management_loop.py` | `paper/tables/management_*.csv` |
| Fig. 14 | `fig14_management_timeline` | `fig14_management_timeline.py` | `paper/tables/management_*.csv` |
| Fig. 15 | `fig15_lessons` | `fig15_lessons.py` | `paper/tables/management_lessons.csv` |
| Fig. 16 | `fig16_dispatches` | `fig16_dispatches.py` | `paper/tables/management_dispatches.csv` |
| Supplementary Fig. 1 | `sfig1_separation_frames` | `sfig1_separation_frames.py` | logs (separation-test transcripts); screenshots |
| Supplementary Fig. 2 | `sfig2_timing` | `sfig2_timing.py` | logs (landing CSVs) |
| Supplementary Fig. 3 | `sfig3_offset_aim` | `sfig3_offset_aim.py` | logs (`wd_20261022_001621.out` and its landing CSV); screenshots |
| Supplementary Fig. 4 | `sfig4_lights` | `sfig4_lights.py` | screenshots |
| Supplementary Fig. 5 | `sfig5_rendezvous` | `sfig5_rendezvous.py` | logs (`rdv_r1_rdv_20261022_014347.json`) |

Panels that show in-game screenshots (Fig. 1b, Supplementary Figs. 1, 3, 4) are left empty, with a note, if
`data/raw/screenshots` is missing. Simulation results (`data/raw/gnc2`) come from the simulators of the development
repository, which are not in this repository's code tree (report, Section 9.2).
