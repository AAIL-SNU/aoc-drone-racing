# Local prototype validation — 2026-10-15

Environment: Apple Silicon macOS, Python 3.12.12, RotorPy 3.0.0. Exact dependency versions are in `requirements.txt` and `requirements-dev.txt`. The project was relocated to `/Users/oxcarxierra/aail_workspace/aoc-drone-racing/` and setup now uses conda or standard venv with pip.
These results describe this local prototype and the scenarios below, not full competition readiness or a robustness guarantee.

| Acceptance item | Observed result |
|---|---|
| Four nominal gates, zero wind | Completed gates 0,1,2,3 in order in 17.31276435727225 simulation seconds. No collision or load penalty. |
| Replace `controller.py` | The alternative hover submission runs through the same loader and evaluator, then times out without gate progress. |
| Wrong direction, skipped gates, contacts, incomplete run | Pure geometry tests cover signed rotated-gate crossings, out-of-order crossing, eroded opening, frame contact, thin-obstacle sweep, collision-before-gate, finish-before-later-collision and bounds. Episode tests cover collision and timeout statuses. |
| Rendering parity | Offscreen streaming view and actual native `MacOSX` interactive view produce exactly the same samples, events and score as the headless run. Wall-clock/controller timing is excluded from this comparison. |
| Logs, replay, small disturbances | PNG and 105-frame GIF replay generated. Seed 17 with wind norm bounded by 0.5 m/s and mass sampled from ±5% completes in 17.98961211247155 s. Actual mass scale = 1.0345074792797901. Repeated episode states/events/scores are identical. |
| Physics resolution check | Halving physics step 0.002 -> 0.001 s retains completion; finish-time difference is below 0.05 s. This is one nominal convergence check, not a general numerical error bound. |

Tests: `pytest -q` passed 28 tests; after adding a track serialization/directory-creation test, that test passed separately (29 distinct passing tests total).
The initial log-roundtrip assertion compared tuples with JSON arrays; it was corrected to compare the JSON representation, and the suite passed afterward.

Artifacts (generated and excluded from source control):

- `outputs/nominal.json`: headless reference flight and provenance.
- `outputs/rendered.json`: offscreen renderer run.
- `outputs/live_parity.json`, `outputs/live.png`: native macOS viewer comparison and final frame.
- `outputs/perturbed.json`: seed-17 disturbed run.
- `outputs/nominal.png`, `outputs/replay.gif`: read-only log replay.

Still provisional: final vehicle/track, G limit/penalty weights, collision recovery policy, failure ranking and compute limits.
Still outside this prototype: isolated execution/hard watchdog, official submission service, photorealistic rendering, terrain/pylon courses, and validation of 20 m/s wind.
All existing synced `sources/` references were left untouched.

## Relocation and conda verification

Project moved to `/Users/oxcarxierra/aail_workspace/aoc-drone-racing/`. The original temporary project copy and its virtual environment were removed after byte-for-byte verification of all 30 source/result files.

Created the `aoc-drone-racing` conda environment from `environment.yml`. No uv executable or lockfile is needed. `conda run -n aoc-drone-racing python -m pytest -q` passed all 29 tests together in 41.33 s. `python -m pip check` reported no broken requirements. The relocated baseline completed in 17.31276435727225 simulation seconds; log: `outputs/conda-nominal.json`.

## Vehicle mesh visualization

Replaced the position marker with RotorPy Quadrotor/Cuboid meshes plus arms and body direction markers. Both the course overview and a body-centered, world-aligned pose panel use the logged quaternion. No plant/controller/rules changes.

The pose-transform test and the two existing renderer/log parity tests passed (3 tests in 15.47 s). A native MacOSX viewer run matched the reference samples, events and score exactly (`outputs/vehicle-live-parity.json`). GIF preview: `outputs/vehicle-replay.gif` (also copied to `outputs/replay.gif`); tilted-pose PNG: `outputs/vehicle-pose.png`. RotorPy 3.0 mesh ndarray.shape deprecation notices are narrowly filtered during mesh calls under pinned NumPy 2.5; unrelated warnings are preserved.

## Single course view

Removed the separate vehicle-pose panel at the user's request. The single course view retains the library rotor mesh, added arms/body/direction indicators and logged orientation.
