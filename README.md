# Autonomous Drone Racing Competition for Applied Optimal Control (M2795.007200), Fall 2026

Write a controller that flies **5 laps of a fixed circuit as fast as possible** while wind pushes the drone around.

| Document | Contents |
|---|---|
| **README.md** (this file) | Installing, running, testing and submitting the code |
| **[COMPETITION.md](COMPETITION.md)** | The rules: schedule, race, dynamics, disturbances, constraints, compute limits, scoring |

## Repository layout

| Path | What it is | Do you edit it? |
|---|---|---|
| `submission/` | **Your controller.** The only folder that is collected and scored | **Yes** |
| `examples/` | A slow PD example controller to start from | Copy from it |
| `config/tracks/` | The five shipped tracks, named `<shape>.json` (with `"name": "<shape>_track"` inside): `eight`, `twisted_eight`, `inner_loop`, `trefoil`, `coaster`. The official circuit is one of them with its gates moved at random (COMPETITION.md Section 3). Each `.png` beside a track is its picture (top view, 3D view, altitude profile) | No |
| `config/race.json` | The default race (track + laps + disturbance) that `drone-race run` flies. Edit it to experiment; `git checkout config/race.json` restores it | Yes, to experiment |
| `config/server_evaluation_race.json` | The race the shared practice server flies every upload on. Run it locally with `--race` to reproduce a server result | No |
| `src/drone_race/` | Simulator, referee, scoring, helper modules, the `drone-race` command | No (changes are not used for scoring) |
| `outputs/` | Everything `drone-race` writes: run logs, calibration | Generated |

## Installation

Run from the project root. Conda is the default setup and pins Python 3.12.12 with the verified package versions.

```bash
conda env create -f environment.yml
conda activate aoc-drone-racing
```

Alternatively, with Python 3.12 and venv:

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m pip install --no-deps -e .
```

Either way installs the `drone_race` package in editable mode and the `drone-race` command.

## Writing a controller

Everything you hand in lives in `submission/`:

```
submission/
  controller.py        # defines Controller; the interface is documented at its top
  my_model.py          # any other modules, imported from controller.py
  assets/              # weights, precomputed trajectories, ... (whole folder <= 50 MB)
```

`controller.py` must define a class `Controller` with two methods:

```python
from drone_race import Action, Observation, PublicConfig

class Controller:
    def reset(self, config: PublicConfig) -> None:    # once, before the flight
        ...
    def act(self, observation: Observation) -> Action:  # every control step (50 Hz)
        return Action(thrust_N=4.9, body_rates_rad_s=(0.0, 0.0, 0.0))
```

The types are defined in `src/drone_race/api.py`:

- **`PublicConfig`**: `track` (`gates`, `obstacles`, `bounds`, `start`), `laps`, `control_dt_s`, `physics_dt_s`, `nominal_mass_kg`, `gravity_m_s2`, `thrust_min_N`/`thrust_max_N`, `body_rate_limit_rad_s`, `body_rate_gain_s_inv`, `vehicle_radius_m`, `act_budget_s`, `reset_budget_s` and the `battery_*` parameters. Each `Gate` has `center`, `yaw_rad`, `width_m`/`height_m` (inner opening) and `frame_m`.
- **`Observation`**: `time_s`, `position_m`, `velocity_m_s`, `quaternion_xyzw` (body → world), `body_rates_rad_s`, `next_gate`, `lap` (0-based), `gates_passed`, `gate_collisions`, `battery_soc`, `battery_voltage_V`, `thrust_limit_N`.
- **`Action`**: `thrust_N` (collective thrust along body +z) and `body_rates_rad_s` (p, q, r).

All quantities are SI; the world frame is z-up. The meaning of each value, the limits and what happens when they are exceeded are in [COMPETITION.md](COMPETITION.md). The docstring at the top of `submission/controller.py` summarizes the same interface.

The shipped `submission/controller.py` is a template that only holds its start position. Replace its `reset()` and `act()`, or start from the PD example in `examples/`.

How the controller is run:

- It runs in its own process and only exchanges observations and actions with the simulator, as JSON messages.
- Its working directory is `submission/`, which is also on `sys.path`, so `open("assets/w.npz")` and `import my_model` work.
- It gets one thread, no GPU, and the package versions in `requirements.txt`.
- `print()` output goes to stderr and never interferes with the simulator.

### Optional helper modules

These ship with the package and may be imported from a submission, or ignored:

| Module | What it offers |
|---|---|
| `drone_race.battery` | The exact battery equations the simulator uses: `terminal_voltage`, `thrust_limit`, `current`, `BatteryModel.step`, ... |
| `drone_race.helpers` | `track_position`: a PD position tracker (optional velocity feedforward `target_velocity`, gains `kp`/`kd`, `max_accel`) that returns an `Action` |
| `drone_race.rules` | The referee's gate geometry: `gate_local`, `inside_opening`, `frame_boxes`, ... |
| `drone_race.randomize` | How races are randomized: `perturb_track` (moved gates), `random_wind_zones`, `threads_gates` (the acceptance check) |
| `drone_race.trackplot` | `plot_track(track)` returns a matplotlib figure (top view with pass directions, 3D view, altitude profile); `save_track_plot(track, path)` writes it as a PNG |

## Running

```bash
drone-race run                          # submission/ on config/race.json, then a playback window
drone-race run --laps 1                 # just one lap
drone-race run --headless               # no window: print the result only
drone-race run --seed 5                 # another random circuit and wind (moved gates, new wind zones)
drone-race run --fixed                  # the unmoved track, no wind zones (turbulence only)
drone-race run --calm                   # no wind zones and no turbulence
drone-race run --turbulence 1.0         # stronger turbulence (std in m/s, 0 to 2)
drone-race run --debug                  # controller inside this process: breakpoints/pdb work, timing not enforced

drone-race run --controller examples/pd_controller.py --laps 2    # any controller folder or .py file
drone-race run --track config/tracks/eight.json              # any track: 1 lap, no disturbance
drone-race run --race path/to/race.json                          # any race file

drone-race replay outputs/submission_practice.json                # watch a saved run again
drone-race replay outputs/submission_practice.json --output outputs/replay.gif
drone-race make-track outputs/random.json --gates 10 --seed 3     # a seeded random track for testing (+ outputs/random.png)
drone-race plot-track config/tracks/twisted_eight.json               # picture of a track -> config/tracks/twisted_eight.png
drone-race plot-track --race config/race.json --seed 5   # the circuit and wind zones of seed 5 -> outputs/<track>-seed5.png
```

`run` simulates the whole race first, printing each lap as it completes, and only then opens the window, which plays the flight back at real time (`--speed 2` for double speed). Drawing therefore never slows the controller down. The window shows the course with the flight path, a chase view of the vehicle, roll/pitch/yaw over time, and the battery's remaining charge [%] and terminal voltage over time. The terminal shows the status, score, lap times, gate collisions, `act()` timing, peak tilt and battery use.

### Race files

The default race is `config/race.json`. A race file bundles a track, the number of laps, the randomization and the disturbance:

```json
{
  "name": "practice",
  "track": "tracks/eight.json",
  "laps": 5,
  "randomize": {"gate_shift_m": 1.0, "gate_height_m": 0.5, "gate_yaw_deg": 15.0, "wind": true},
  "scenario": {
    "seed": 0,
    "turbulence_m_s": 0.6,
    "max_time_s": 900.0
  }
}
```

`randomize` moves every gate of the track a little and places three random wind zones on the course, both drawn from the seed, exactly as the official race does (COMPETITION.md Sections 3 and 5; code in `src/drone_race/randomize.py`). So `--seed N` gives a new circuit and wind each time, while the same seed always gives the same one. `--fixed` ignores `randomize`: the unmoved track and the file's own `wind_zones` (none here). Without `randomize`, a race uses its track as is and the `wind_zones` it lists, for example:

```json
"wind_zones": [{"name": "east crosswind", "lower": [12, -3, 0], "upper": [20, 5, 6],
                "direction": [1, 0, 0], "speed_m_s": [1.5, 3.0], "heading_jitter_deg": 30, "ramp_m": 1.5}]
```

`track` is relative to the race file. The scenario fields are defined by `Scenario` in `src/drone_race/env.py` and `WindZone` in `src/drone_race/wind.py`. `--laps`, `--seed`, `--turbulence`, `--calm` and `--max-time` override the file. To experiment, edit `config/race.json` (another of the five tracks, larger moves, stronger turbulence) or keep variants as separate files and pass them with `--race`. The laps, turbulence and randomization amounts in the shipped file match the official race.

The shared practice server does not read your `config/race.json`: it flies every upload on `config/server_evaluation_race.json`. To reproduce a server result locally, run `drone-race run --race config/server_evaluation_race.json`.

### Logs

Every run is saved as JSON in `outputs/` (`outputs/<submission>_<race>.json` unless `--log` is given):

- `summary`: status, score (`score_s` = time + `penalty_s`), lap times, gates passed, gate collisions, `act()` timing (mean, p99, max, deadline misses), peak tilt and load, battery use, thrust-limited steps.
- `events`: gate passes, gate collisions (`gate_collision`, one per contact), laps and the terminal event, with times.
- `samples`: the state, battery, true wind and applied action at every control step.
- `config`, `scenario`, `realization` (the zone winds the seed drew) and `provenance`.

## Timing on your computer

`act()` must fit in 20 ms **on the scoring server** (see COMPETITION.md). Your computer is not the scoring server, so calibrate it once:

```bash
drone-race bench
```

`bench` times three fixed workloads (controllers in `src/drone_race/bench_workloads/` that hover and do a fixed amount of NumPy, SciPy and plain-Python work per `act()`), compares them with the same measurement made on the scoring server, and saves the speed ratio to `outputs/calibration.json`. From then on `drone-race run` scales the 20 ms budget to this computer (e.g. 14 ms on a computer that is faster than the server), so a call that would be late on the server is also late locally. `run` prints which budget it used. `bench` also estimates your own controller's `act()` time on the server; keep about 30 % margin at p99. Use `--budget-scale` on `run` to override the calibration.

## Checking a submission

```bash
drone-race check                # layout, size, packages and a short flight: must end with "OK"
```

## Example controller

`examples/pd_controller.py` is a complete single-file submission: copy it to `submission/controller.py`, or pass it with `--controller`. It is a deliberately slow gate-following baseline: pure pursuit along the next gate's axis at a fixed 2.5 m/s, with the `track_position` PD loop from `drone_race.helpers`. It finishes reliably but makes no attempt at speed, and is the simplest place to see the interface working.

Going faster is the assignment: the planner, the prediction model and the controller are yours to write. `drone_race.battery` and `drone_race.rules` give the exact battery and gate formulas the simulator uses, and COMPETITION.md Section 4 describes the vehicle model. To compare versions over several disturbance draws, run them with different seeds:

```bash
for seed in 1 2 3 4 5; do drone-race run --headless --seed $seed; done
```

## References

- RotorPy: https://github.com/spencerfolk/rotorpy (MIT License, Copyright 2023 RotorPy Authors).
- Spencer Folk, James Paulos, Vijay Kumar, "RotorPy: A Python-based Multirotor Simulator with Aerodynamics for Education and Research", ICRA 2023 UAV simulator workshop: https://arxiv.org/abs/2306.04485.
- RotorPy is used as an external dependency; no modified copy of its source is included. Preserve its copyright and license notice when redistributing.
