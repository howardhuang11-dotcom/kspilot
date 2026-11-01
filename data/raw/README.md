# Raw development records

These are the raw records from which every table in `paper/tables/` and every figure in `paper/figures/` of the
System and Development Report is generated (report, Section 9.2). Regenerate everything with

    python paper/regenerate_all.py

and verify that the files are the ones the report was computed from with

    python paper/analysis/compute_stats.py --check-manifest

(`paper/tables/raw_manifest.sha256` lists the SHA-256 of every file here, paths relative to `data/raw`; this README is
not listed).

## Contents

| Directory | Files | Used by |
|---|---|---|
| `logs/` | `wd_YYYYMMDD_HHMMSS.out`: supervisor (watchdog) transcript of each run (325 files; 278 started before the data freeze of 22 October 2026 00:00 local time and enter the statistics, the rest are only parsed; the post-freeze ones used by a figure are those of Fig. 3 and Supplementary Figs. 3 and 5) | `compute_stats.py`, all figures |
| | `YYYYMMDD_HHMMSS_orbit_<craft>_landing.csv.gz`: high-rate descent telemetry of one flight (game time), gzip-compressed | `compute_stats.py`, figures |
| | `experiments.jsonl`: one record per flight that reached the descent phase (miss at 0.01 m, code revision, environment switches) | `compute_stats.py`, `divert_flight_record.py`, `ablation_record.py`, figures |
| | `matrix_results.txt`: one line per robustness-matrix case attempt | `compute_stats.py` |
| | `flight_log_names.txt`: the file names of the 317 per-flight logs (`*_orbit_*.txt`). Only the names are used (they join a transcript to its flight); the logs themselves are not released | `compute_stats.py` |
| | `rdv_r1_rdv_20261022_014347.json`: rendezvous record of Supplementary Fig. 5 | `sfig5_rendezvous.py` |
| `gnc2/` | simulation results: `entry_replay.json`, `entry_mc.json`, `entry_mc_stress.json`, `burn_attitude_id.json`, `divert_measured_mc.json`, `divert_optimal.json` (from the development repository's simulators, which are not in this repository's code tree) | Figs. 4, 7, 8, 9 |
| `screenshots/` | the 11 in-game screenshots that the figures show (lossless PNG re-encoding, pixels unchanged) | Fig. 1b, Supplementary Figs. 1, 3, 4 |

Only the files that the scripts read are included; the development directories hold more. Every reader handles a
plain or `.gz` file transparently.

## Sanitisation

The files are copies of the original records with these changes only, applied before the manifest was written:

- Absolute paths of the author's machine were replaced by paths relative to the project directory (for example
  `.../logs/20261021_011056_orbit_T-WorksFineGF.txt` became `logs/20261021_011056_orbit_T-WorksFineGF.txt`;
  other working copies became `worktrees/<name>/`). Only the base name of a log path is used by the scripts.
- Chinese text in 1,001 lines of 81 transcripts (flight-planner and MechJeb status strings, part names) was replaced by
  English equivalents; numbers were not touched. Emoji marks became `[ok]` / `[no]`.
- No e-mail address, credential or host name was found in the records.
- Counts: 781 path strings replaced; 1,001 lines with Chinese text in 81 transcripts; 67 emoji marks; 298 of the 334 text and JSON files changed (the 6 simulation JSON files and the CSVs are unchanged; the CSVs are only compressed).
- The Chinese-text replacement is a substring dictionary applied to every line that contains Chinese, not a translation by a reader of Chinese; "[mod part name]" and "[part name]" stand for part names, and the character for "level" becomes "stage" wherever it occurs on such a line.
- The per-flight text logs were not released (names only), and screenshots carry no metadata.

Every table produced from these files is identical to the one produced from the unsanitised records (verified for
all tables; the only differences are the table headers that were fixed to quote commas, the script hash and the
manifest). The figures are pixel-identical.

## Checking the sanitisation

`tools/sanitize_logs.py` is the transform itself (dictionary included). Given the private original directories it
reproduces every file of this directory (`--check`; the screenshots are compared by pixels, since the PNG byte stream
depends on the Pillow version). `ORIGINAL_SHA256.txt` lists, in `sha256sum` format, the SHA-256 of the 609 unsanitised
originals that were used (`logs/`, `gnc2/` and `screenshots/` name the private source directories; the descent CSVs are
listed uncompressed, as `<key>_landing.csv`). `flight_log_names.txt` is derived from the names of the per-flight text logs,
which are not hashed because no script reads their contents. The originals stay private; the hashes let anyone who later
receives them check that they are the files this release was made from.

## Counts

`logs/` holds 325 transcripts: 303 started on or before 22 October 03:33 (the cut-off of the report's process counts; 278
before the data freeze) and 22 later ones. `experiments.jsonl` holds 191 records, of which 169 are dated up to 03:33 and
158 before the freeze; the other 22 were added by the ablation flights and later work. The report's 382 transcripts to
03:33 are these 303 plus 79 earlier transcripts with descriptive names, which are not released.

The 11 screenshots show in-game content of Kerbal Space Program, which remains the property of its owners (`NOTICE`); the
share-alike licence applies to this project's own contribution only.

## Licence

CC BY-SA 4.0, as the other data of the report (`LICENSE-docs`).
