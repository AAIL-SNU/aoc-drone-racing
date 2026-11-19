# Competition Rule

> **Draft.** Rules may change before the competition. When they do, this document and `config/races/` are updated together. Items marked **TBD** are not final yet.

This document is the rulebook: the race, the vehicle, the disturbances, the constraints, the compute limits and the scoring. How to install, run and test the code is in [README.md](README.md).

## 1. Overview

- Each participant writes a controller that flies a quadrotor around a fixed circuit **as fast as possible**, while wind pushes the drone around.
- The aim is to implement and practice the optimal control methods covered in Applied Optimal Control (M2795.007200).

## 2. Schedule and procedure

1. The repository is released two weeks before the competition. Each participant clones it and implements a controller.
2. Submissions (the `submission/` folder) are due through GitHub (or eTL) on the night two days before the competition.
3. The TAs run every submission on the official Linux server, all on **the same circuit with the same disturbance**, and record times and scores.
4. A video is rendered from each participant's flight trajectory.
5. On competition day the running order is drawn at random. The videos are played in that order on the dashboard web page, and the leaderboard is updated live. The dashboard shows the flight video together with the flight path, lap times and fastest lap.
6. The final ranking is decided by the score (Section 9).
7. (Optional, time permitting) The winner receives a small 3D-printed trophy. It has no effect on grades.

## 3. The race

| Item | Rule |
|---|---|
| Circuit | `config/tracks/circuit_1.json` (`aoc-circuit-v1`): a closed figure-eight of **10 gates**, where a high path and a low path cross in the middle. No obstacles. |
| Arena | The vehicle must stay inside the box x ∈ [−20, 20] m, y ∈ [−10, 10] m, z ∈ [0, 6] m (`track.bounds`). The ground is the bottom face of this box. |
| Laps | **5 laps.** One lap passes gates 0 → 9 in order; gate 9 is the start/finish line. |
| Start | At rest, hovering at `track.start` = (−1.6, −1.6, 1.5) m, just past gate 9, at simulated time t = 0. |
| Time limit | Set by the official race file. If the file sets none, 45 s per lap (225 s for 5 laps). |
| Gates | Each gate has a 2 m × 2 m square opening framed by 0.1 m bars. It is passed along its local +x axis, (cos yaw, sin yaw, 0). |

Coordinates are SI, world frame right-handed with z up, gravity (0, 0, −9.81) m/s².

## 4. Vehicle and dynamics

The plant is [RotorPy](https://github.com/spencerfolk/rotorpy)'s AscTec Hummingbird model with aerodynamics enabled. It is the same for everyone and is assumed exact: there is no model mismatch apart from the wind.

| Property | Value |
|---|---|
| Mass | 0.5 kg |
| Inertia | diag(3.65, 3.68, 7.03) × 10⁻³ kg·m² |
| Arm length / rotor radius | 0.17 m / 0.10 m |
| Aerodynamics | rotor drag, parasitic (frame) drag and translational lift, all driven by the airspeed v − wind |
| Motors | first-order lag, time constant 5 ms; rotor speed ≤ 1500 rad/s |
| Collision shape | sphere of radius 0.28 m around the center of mass |
| Physics step | 2 ms |

**Action.** Every control step the controller returns a collective thrust T [N] along body +z and a body-rate command ω_cmd = (p, q, r) [rad/s]. A built-in rate loop tracks the command, ω̇ ≈ 25 (ω_cmd − ω), followed by motor allocation, rotor saturation and motor lag.

**Observation.** Noise-free: time, position, velocity, attitude quaternion (x, y, z, w; body → world), body rates, the next gate to pass, the current lap, the number of gates passed and gate collisions so far, and the battery state (state of charge, terminal voltage, current thrust limit). The wind is **not** observed.

**Control rate and delay.**
- The controller runs at **50 Hz** (`control_dt_s = 0.02`). The TAs may adjust this after testing.
- Each action is held (zero-order hold) until the next one takes effect: one control step normally, longer after a late `act()` call (Section 7).
- There is a **one-step actuation delay**: the action computed from the observation at time t_k is applied from t_k + 20 ms to t_k + 40 ms. Until the first action takes effect, the drone flies hover thrust (4.905 N) with zero body rates.

`drone_race.planning.QuadModel` reproduces this model for use inside a controller.

## 5. Disturbances

The wind is an air velocity in the world frame. It acts on the drone only through the aerodynamic terms above, which use the airspeed v − wind. It has two parts:

**Wind zones (layout public, strength random).** Boxes on the course, each with a nominal wind direction. For one race the seed draws each zone's speed uniformly from its `speed_m_s` range, and turns its direction about the vertical axis by a uniform angle within ±`heading_jitter_deg`. The zone wind is then constant in time, identical on every lap, and fades smoothly to zero within `ramp_m` outside the box. The practice race (`config/races/competition.json`) has three zones:

| Zone | Box (lower → upper) [m] | Direction | Speed [m/s] | Heading jitter |
|---|---|---|---|---|
| east crosswind | (12, −3, 0) → (20, 5, 6) | +x | 1.5–3.0 | ±30° |
| center downwash | (−2, −2, 0) → (2, 2, 6) | −z | 1.0–2.0 | 0° |
| west tailwind | (−20, −8, 0) → (−12, 3, 6) | −y | 1.5–3.0 | ±30° |

**Turbulence (unpredictable).** A first-order Dryden (Gauss–Markov) process on each world axis, driven by fresh Gaussian noise every physics step. Its standard deviation is `turbulence_m_s` horizontally (0.6 m/s in the practice race) and half that vertically. Its correlation time is the length scale divided by the airspeed (20 m horizontally, 3 m vertically; airspeed floored at 1 m/s).

**Seed.** All random draws come from the race seed alone, so every submission meets exactly the same zone winds and the same turbulence sequence. The official seed is chosen by the instructor and kept private until the results are published.

## 6. Constraints

| Constraint | Rule |
|---|---|
| Thrust | Clipped to [0, `observation.thrust_limit_N`] **without penalty**. 15 N is the limit at full charge; the battery lowers it (below). The clipping uses the limit reported in the observation the action was computed from. |
| Body rates | Each of p, q, r must be within ±3 rad/s. A command outside this range, a NaN, or a malformed action ends the run as `invalid_action`. |
| Battery | 3S LiPo model (`drone_race.battery`, equations below). Reaching zero charge ends the run as `battery_depleted`. The 500 mAh pack is sized so that an aggressive controller still finishes 5 laps with most of its charge left; the battery costs time through the lower thrust limit, not by running out. |
| Maximum tilt | **TBD.** A real racing drone needs its camera to see ahead, so tilt will be limited. The limit and how violations are handled are not decided; for now only `peak_tilt_deg` is logged. |
| Arena | The collision sphere must stay inside the arena box (`out_of_bounds` otherwise). |
| Gate frames | Touching a gate bar with the collision sphere adds 5 s per collision; the flight continues (Section 9). |

**Battery model.** With T the applied (clipped) thrust and SoC ∈ [0, 1]:

```
power              P     = 3.75 W/N^1.5 · T^1.5                      (≈ 41 W at hover)
open-circuit volt  V_oc  = 9.9 V + (12.6 V − 9.9 V) · SoC
terminal voltage   V     = V_oc − R·I,  with P = V·I,  R = 0.05 Ω
discharge          dSoC/dt = −I / (500 mAh)
thrust limit       T_max = 15 N · (V / 12.6 V)²
```

Flying hard drains the pack faster, and both a drained pack (lower V_oc) and a large current draw (voltage sag R·I) lower the thrust limit. The observation's `battery_voltage_V` is the terminal voltage under the thrust applied during the previous control step, and `thrust_limit_N` is computed from it. The simulator uses exactly the functions in `drone_race.battery`, which controllers may call.

## 7. Compute resources and fairness

- **One machine, one team at a time.** All submissions are scored on the same Linux server, sequentially, never in parallel, so timing is comparable.
- **Isolation.** The controller runs in a separate process and exchanges only observations and actions with the simulator. On the scoring server it also runs in a Docker container with no network, a read-only file system (only `/tmp` is writable), one CPU core and 4 GB of memory.
- **One thread, no GPU.** BLAS, OpenMP, numba, JAX and similar libraries are fixed to one thread, and the process is pinned to one core. Run neural networks on the CPU. Train offline on your own machine and submit the weights in `assets/`.
- **Python packages.** The official environment is Python 3.12.12 with the versions pinned in `requirements.txt`. If you need a package that is not listed, ask the TAs at least one week before the competition. The list is frozen after that.
- **50 Hz compute budget.**
  - `act()` must return within **20 ms on the official server**. Time is measured as the CPU time of the whole controller process, so OS preemption never makes a call late, and extra threads only use up the budget faster.
  - A late call's result is **dropped**. Simulated time keeps running while the controller computes: the previous action stays applied for as many 20 ms periods as the call took (rounded up), and the next observation comes after that. Each late call counts as one `deadline_misses`.
  - An `act()` call that takes longer than 0.5 s is treated as hung and ends the run (`controller_timeout`).
  - `reset()` may take up to **60 s**. Use it for offline trajectory optimization or JIT compilation.
  - Importing the submission may take up to 120 s.
- **Laptop vs. server.** `drone-race bench` estimates the speed ratio between your computer and the official server and makes `drone-race run` enforce an equivalent budget locally (see README). It is an estimate: keep at least 30 % margin at the p99 `act()` time. Before the competition, the TAs will run practice submissions on the official server and share the measured timing.

## 8. Submission rules

```
submission/
  controller.py        # defines Controller with reset(config) and act(observation)
  my_model.py          # optional extra modules, imported from controller.py
  assets/              # optional data: weights, precomputed trajectories, ...
```

- Only the `submission/` folder is collected. Changes to the simulator (`src/drone_race/`) or any other file are not used for scoring.
- The whole folder must be **50 MB or less**; a larger submission is disqualified (ranked as not finished).
- At run time the working directory is `submission/`, so relative paths such as `open("assets/weights.npz")` work.
- Only the Python standard library, the packages in `requirements.txt` and `drone_race` itself may be imported.
- `drone-race check` must end with `OK` before you submit.

## 9. Scoring

The **score** is a time in seconds; **lower is better**:

```
score = T_finish + 5 s × (gate collisions)
```

- **T_finish** is the simulated time at which the drone passes the finish line (gate 9) at the end of lap 5, interpolated within the physics step.
- **Gate passed:** the center of the drone crosses the gate's plane in its +x direction inside the 2 m × 2 m opening, **and the gate is the next one in order**. Gates must be passed strictly in order (0 → 9 on every lap):
  - crossing any other gate does not count; after gate 3, flying through gate 5 does not pass gate 5, and gate 4 is still the next gate;
  - crossing a gate backwards does not count;
  - a gate that is never passed in order means the race cannot be completed (eventually `timeout`).
- **Gate collision (+5 s, the flight continues):** the collision sphere (radius 0.28 m) touches a gate frame bar. There is no impact physics; the drone flies on.
  - Continuous contact is one collision, however long it lasts.
  - Separating from the frame and touching it again is a new collision.
  - Touching two bars of the same gate at the same time is one collision.
  - Brushing a bar while the center crosses inside the opening counts both the pass and the collision.
- **Did not finish (DNF):** the run ends without completing 5 laps when
  - the drone leaves the arena (`out_of_bounds`),
  - the battery is empty (`battery_depleted`),
  - the time limit is reached (`timeout`),
  - the controller raises an exception (`controller_error`), hangs (`controller_timeout`) or returns an invalid action (`invalid_action`).

  On tracks with obstacles, hitting one also ends the run (`collision`); the competition circuit has none. A non-finite physics state ends the run as `simulation_error` and is reviewed by the TAs.
- **Ranking.** Finished runs are ranked by score. Every DNF ranks below every finished run; among DNFs, more gates passed ranks higher, then a longer flight time. A disqualified submission is ranked as not finished.
- **Official result.** The log produced on the scoring server is final. Re-running the same seed can give slightly different results, because compute-time jitter can change which `act()` calls are late.
