# Autonomous Drone Racing Competition for Applied Optimal Control (M2795.007200), Fall 2026

Write a controller that flies **10 laps of a fixed circuit as fast as possible**
while wind and model errors push the drone around. The rules, schedule, scoring
and compute limits are in **[COMPETITION.md](COMPETITION.md)** (Korean).

## Repository layout

| Path | What it is | Do you edit it? |
|---|---|---|
| `submission/` | **Your controller.** The only folder that is collected and scored | **Yes** |
| `examples/` | Example MPC and MPPI controllers to start from | Copy from it |
| `config/tracks/`, `config/races/` | The circuit and the practice race (track + laps + disturbance) | No |
| `src/quad_race/` | Simulator, referee, scoring, `drone-race` command | No (changes are not used for scoring) |
| `tests/` | Tests of the simulator itself, for the organizers | No |
| `docker/`, `.github/` | Scoring-server image and the automatic check on push | No |
| `outputs/` | Everything `drone-race` writes: run logs, calibration, evaluations | Generated |

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
python -m pip install -r requirements-dev.txt
python -m pip install --no-deps -e .
```

## Your submission

Everything you hand in lives in `submission/`:

```
submission/
  controller.py        # Controller.reset(config) / act(observation); interface documented at its top
  my_model.py          # any other modules, imported from controller.py
  assets/              # weights, precomputed trajectories, ... (whole folder <= 50 MB)
```

The controller runs in its own process and only exchanges observations and actions
with the simulator. Its working directory is `submission/`, so `open("assets/w.npz")`
works. It gets one thread, no GPU, and the package versions in `requirements.txt`.

## How to run

```bash
drone-race run                          # your submission/, practice race (10 laps), then a playback window
drone-race run --laps 1                 # just one lap
drone-race run --headless               # no window: print the result only
drone-race run --seed 5                 # another draw of the practice disturbance
drone-race run --debug                  # controller inside this process: breakpoints/pdb work, timing not enforced

drone-race run --controller examples/mpc_controller.py --laps 2   # any controller file
drone-race run --track config/tracks/four_gates.json                     # any track: 1 lap, no disturbance

drone-race replay outputs/submission_practice.json                # watch a saved run again
drone-race replay outputs/submission_practice.json --output outputs/replay.gif
```

`run` simulates the whole race first, printing each lap as it completes, and only
then opens the window, which plays the flight back at real time (`--speed 2` for
double speed). Drawing therefore never slows the controller down. The result and the
log path are printed in the terminal; every log goes to `outputs/`
(`outputs/<submission>_<race>.json` unless `--log` is given).

The default race is `config/races/practice.json`: the competition circuit
(`config/tracks/circuit.json`), 10 laps, and a practice disturbance (wind up to 0.5 m/s,
±5% mass error). A race file just bundles track + laps + disturbance; the official
one has the same circuit and laps, with a seed chosen by the instructor and kept private.

### Timing: 50 Hz on the official machine

`act()` is called every 20 ms of simulated time and must return within 20 ms
**on the scoring server**. A late result is dropped and the previous action is held
for that step. A call that hangs for more than 0.5 s ends the run. `reset()` may take
up to 60 s, which is the place for offline optimization or JIT compilation.

Your laptop is not the scoring server. Calibrate it once:

```bash
drone-race bench
```

`bench` times the two example controllers, compares them with the same measurement
made on the scoring server, and saves the speed ratio to `outputs/calibration.json`.
From then on `drone-race run` scales the 20 ms budget to this computer (e.g. 14 ms on a
laptop that is faster than the server), so whatever is late on the server is also late
on your laptop, and `run` prints which budget it used. `bench` also estimates your own
controller's `act()` time on the server; keep about 30% margin at p99.

The controller always runs with one thread: BLAS/OpenMP/numba/JAX are limited to a
single thread, and the budget counts the CPU time of the whole controller process, so
extra threads would only use up the budget faster.

```bash
drone-race check                # layout, packages, a short flight: must end with "OK"
```

Pushing to GitHub runs the same check on Linux (`.github/workflows/check.yml`).

## Example optimal controllers

Two example racers live in `examples/`. Each is a complete single-file submission: copy it to `submission/controller.py`, or pass it with `--controller`.

- `examples/mpc_controller.py`: nonlinear MPC solved with iLQR (Gauss-Newton DDP) over a 1 s horizon. Input bounds are built into the model through tanh squashing. A smooth "gate funnel" cost keeps the drone centred near upcoming gates.
- `examples/mppi_controller.py`: MPPI with 768 sampled rollouts per step. Crossing a gate outside its opening or leaving the bounds is a hard penalty in the cost.

Both use `quad_race.planning`, an optional helper module, for a racing line through the gate centres (repeated for every lap) with a speed profile, a batched quadrotor model that includes RotorPy's rotor drag and translational lift, and a disturbance observer. `LINE` at the top of each file sets how aggressive the racing line is, and it is the main lap-time knob.

Practice race (`config/races/practice.json`: circuit, 10 laps, wind 0.5 m/s, ±5% mass), Apple Silicon laptop:

| Controller | Status | Score [s] | Fastest lap [s] | p99 `act()` [ms] | Deadline misses |
|---|---|---|---|---|---|
| baseline (`submission/`) | finished | 655.85 | 64.56 | 0.4 | 0 |
| MPC (iLQR) | finished | 68.80 | 6.78 | 23.7 | 89 |
| MPPI (512 samples) | finished | 68.91 | 6.79 | 20.8 | 53 |

The examples sit right at the 20 ms budget on this laptop, which is why they drop a few
percent of their actions. `examples/benchmark.py` compares controllers over several seeds:

```bash
python examples/benchmark.py --laps 1 --seeds 1 2 3
```

## For TAs: official scoring

```bash
docker build -f docker/Dockerfile -t aoc-drone-racing:2026 .     # controller image (official pins)
drone-race bench --write-reference                               # once, on the scoring server
drone-race evaluate teams/ --race config/races/official.json --seed <instructor seed> \
    --docker aoc-drone-racing:2026 --cpu 3
```

`teams/` holds one folder per team (each a copy of its `submission/`). `evaluate` runs
the teams one at a time and writes `outputs/evaluation/logs/<team>.json` (full
trajectory), `outputs/evaluation/stderr/<team>.txt`
and `outputs/evaluation/leaderboard.json`.

`--docker` runs each controller in the image above with no network, a read-only file
system, one CPU and 4 GB of memory. Without it the controllers are isolated by process
only: fair for timing, but a submission could still read files on the server (such as
the private race file) or use the network. Fine for practice runs; use Docker for the
official one.

## References

- RotorPy: https://github.com/spencerfolk/rotorpy (MIT License, Copyright 2023 RotorPy Authors).
- Spencer Folk, James Paulos, Vijay Kumar, "RotorPy: A Python-based Multirotor Simulator with Aerodynamics for Education and Research", ICRA 2023 UAV simulator workshop: https://arxiv.org/abs/2306.04485.
- RotorPy is used as an external dependency; no modified copy of its source is included. Preserve its copyright and license notice when redistributing.
