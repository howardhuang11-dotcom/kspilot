# Contributing to kspilot

Contributions are welcome, including bug reports, flight data from other platforms, documentation corrections and
code. By participating you agree to follow the [Code of Conduct](CODE_OF_CONDUCT.md).

## Development setup

Follow [docs/INSTALL.md](docs/INSTALL.md). The offline test suite is the minimum gate for every change:

```sh
.venv/bin/python -m pytest -q
```

All tests must pass or skip with a stated reason. A change to shared guidance code (`kspilot/`) must not alter the
behaviour of an existing, verified configuration unless that is the purpose of the change; several tests compare
the default code path against recorded flights for exactly this reason.

## Verification before flight

Guidance changes are verified in stages, from the cheapest to the most expensive:

1. **Unit tests** against the fake kRPC environment (`tests/fake_krpc.py`), without the game.
2. **Replay simulation** (`tools/replay_sim.py`) and Monte Carlo runs. A simulator is trusted only after it has been
   shown to reproduce recorded flights.
3. **In-game flights**, starting from the simplest and lowest-risk scenario (for example a landing from a saved
   checkpoint) and extending the envelope one step at a time (terminal descent, entry, a full mission from orbit,
   a full mission from the pad).

Each in-game test should change one variable, state its expected result and its failure criterion in advance, and
run under `tools/watchdog.py`. A failed flight is followed by a root-cause analysis and a regression test before the
next attempt. Do not interact with the game while an automated flight is running.

## Reporting flight results

When reporting a flight, include the craft, the command line and environment variables, the game, kRPC and mod
versions, the platform, and the relevant files from `logs/` (the watchdog `wd_*.out` file and the landing CSV).
Remove any personal information, such as absolute paths containing a user name, before sharing logs.

## Coding conventions

- Python: standard library style (PEP 8) with lines of up to 120 characters; NumPy for vector arithmetic; no new
  runtime dependency without discussion.
- State units in names or comments (SI units; angles in degrees unless stated otherwise) and the reference frame of
  every vector.
- Mark assumed values as assumptions and measured values with their source.
- Comments, log messages and documentation are written in English.
- C# (plugin): C# 7.3, .NET Framework 4.7.2; reference the game assemblies through `KSPManaged` / `KSP_DIR` only, and
  never commit game assemblies or other game files.

## Licensing of contributions

Unless stated otherwise, contributions to the source code are accepted under the GNU GPL version 3 or later (`LICENSE`),
contributions to text, figures and data under CC BY-SA 4.0 (`LICENSE-docs`), and contributions to `craft/` under
the GNU GPL version 3 (`craft/LICENSE`). Do not submit material that you do not have the right to license on these
terms, including game assets or craft files from sources without a compatible licence.

## Pull requests

Keep pull requests focused on one change, describe the motivation and the verification performed (tests run,
simulations, flights with their logs), and update the documentation when behaviour changes.
