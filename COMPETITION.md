# Competition Rule Book

## Contents


- [Competition Rule Book](#competition-rule-book)
  - [Contents](#contents)
  - [1. Overview](#1-overview)
  - [2. Schedule and procedure](#2-schedule-and-procedure)
  - [3. The race](#3-the-race)
    - [Gate randomization](#gate-randomization)
  - [4. Vehicle and dynamics](#4-vehicle-and-dynamics)
    - [Action](#action)
    - [Observation](#observation)
    - [Control rate and delay](#control-rate-and-delay)
  - [5. Disturbances](#5-disturbances)
    - [Wind zones](#wind-zones)
    - [Turbulence](#turbulence)
    - [Seed](#seed)
  - [6. Constraints](#6-constraints)
    - [Battery model](#battery-model)
  - [7. Compute resources and fairness](#7-compute-resources-and-fairness)
  - [8. Submission rules](#8-submission-rules)
  - [9. Scoring](#9-scoring)

## 1. Overview

- Each student writes a controller that flies a quadrotor around a track **as fast as possible** while random wind pushes it around.
- Students are encouraged to apply as much of the theory from the AOC lectures as they can.

## 2. Schedule and procedure

1. Clone this repo and implement your own controller.
2. If you want to test the controller on the server computer (which will be used for competition for fair computation resource), submit your controller (`submission/`) to the [practice server](https://admin-aoc-drone-race-live.tailfeb8a3.ts.net/). Then your controller will be run on the server, and the computation time will be measured and provided.
3. Select the version that you would like to submit and press **Final submit**.
4. Submissions are made on the practice server: the last version confirmed by the deadline, **Dec 9, 2026, 12:00 KST**, is the entry.
5. On the competition day (**Dec 10, 2026**), the track and disturbance will be determined by a random seed, and your submitted controller will be run with this setting, with video rendered.

## 3. The race

| Item | Rule |
|---|---|
| Track | One of the five tracks in `config/tracks/` (`eight`, `twisted_eight`, `inner_loop`, `trefoil`, `coaster`), with every gate moved at random. The controller learns which track it is and where the gates are only from `config.track` in `reset()`. No obstacles. |
| Arena | `track.bounds`, a box whose bottom face is the ground. Each track has its own; the largest is 40 × 20 × 6 m (`eight`). |
| Laps | **5 laps.** One lap passes every gate in order (10 gates, 12 on `trefoil`). The last gate is the start/finish line. |
| Start | Hovering at rest at `track.start`, 2 m past the last gate, level with zero yaw, at t = 0. |
| Time limit | Set by the official race file; 45 s per lap (225 s for 5 laps) if the file sets none. |
| Gates | 2 m × 2 m square opening framed by 0.1 m bars, passed along the gate's local +x axis, (cos yaw, sin yaw, 0). |

The world frame is right-handed with z up, and gravity is (0, 0, −9.81) m/s².

### Gate randomization

Each gate of the track moves independently:

- horizontally, to a uniform random point within **1.0 m** of its original center,
- vertically, by a uniform amount within **±0.5 m**,
- in yaw, by a uniform angle within **±15°**.

The start moves with the last gate. A draw is rejected and drawn again if

- two gate centers are closer than 3 m,
- a gate opening comes within 0.3 m of the arena walls, floor or ceiling, or
- a smooth reference path cannot pass every gate without touching a frame. The path is a cubic curve through the gate centers that crosses each gate along its axis, flown by the 0.28 m collision sphere (`drone_race.randomize.threads_gates`).

These amounts are the `randomize` section of `config/race.json`.

A trajectory optimized in advance for the original track will clip the frames of the moved gates. Plan on `config.track` in `reset()`, which may use up to 60 s, possibly starting from a precomputed solution.

## 4. Vehicle and dynamics

The vehicle is [RotorPy](https://github.com/spencerfolk/rotorpy)'s AscTec Hummingbird model with aerodynamics enabled. Apart from the wind, there is no model mismatch: the simulator uses exactly these parameters.

| Property | Value |
|---|---|
| Mass | 0.5 kg |
| Inertia | diag(3.65, 3.68, 7.03) × 10⁻³ kg·m² |
| Arm length / rotor radius | 0.17 m / 0.10 m |
| Aerodynamics | rotor drag, parasitic (frame) drag and translational lift, all driven by the airspeed v − wind |
| Motors | first-order lag, time constant 5 ms; rotor speed ≤ 1500 rad/s |
| Collision shape | sphere of radius 0.28 m around the center of mass |
| Physics step | 2 ms |

The aerodynamic coefficients are RotorPy's Hummingbird parameters (`rotorpy.vehicles.hummingbird_params`). Building a prediction model of the vehicle is part of the task.

### Action

Each control step, the controller returns a collective thrust T [N] along body +z and a body-rate command ω_cmd = (p, q, r) [rad/s]. A built-in rate loop, ω̇ ≈ 25 (ω_cmd − ω), tracks the command, followed by motor allocation, rotor saturation and motor lag.

### Observation

The observation is noise-free: time, position, velocity, attitude quaternion (x, y, z, w; body → world), body rates, the next gate to pass, the current lap, the gates passed and gate collisions so far, and the battery state (state of charge, terminal voltage, current thrust limit). The wind is not observed.

### Control rate and delay

- The controller runs at **50 Hz** (`control_dt_s = 0.02`). The TAs may change this after testing.
- Each action is held (zero-order hold) until the next one takes effect: one control step normally, longer after a late `act()` call.
- There is a one-step actuation delay: the action computed from the observation at t_k is applied from t_k + 20 ms to t_k + 40 ms. Until the first action takes effect, the drone flies hover thrust (4.905 N) with zero body rates.

## 5. Disturbances

The wind is an air velocity in the world frame. It acts on the drone only through the vehicle's aerodynamic terms, which use the airspeed v − wind. It has two parts: wind zones and turbulence.

### Wind zones

Three boxes, each spanning the full arena height. Each box is centered on a uniform random point of the lap (the closed path through the gate centers), with the three centers at least 8 m apart horizontally. Boxes are cut off at the arena walls.

| Zone | Footprint (square side) | Direction | Speed [m/s] |
|---|---|---|---|
| `downwash` | 3 to 5 m | straight down (−z) | 1.0 to 2.0 |
| `crosswind 1`, `crosswind 2` | 6 to 9 m | horizontal, uniform random heading | 1.5 to 3.0 |

The seed draws each zone's place, size, heading and speed. A zone's wind is then constant in time, the same on every lap, and fades smoothly to zero within 1.5 m outside its box. Where zones or their fringes overlap, their winds add. Neither the zones nor their wind appear in `config` or the observation.

### Turbulence

Turbulence is a first-order Dryden (Gauss–Markov) process on each world axis, driven by new Gaussian noise every physics step. The standard deviation is **0.6 m/s** horizontally (`turbulence_m_s`) and half that vertically. The correlation time is the length scale (20 m horizontal, 3 m vertical) divided by the airspeed, with the airspeed floored at 1 m/s.

### Seed

Every random draw (gate moves, wind zones, their speeds, the turbulence noise) comes from the race seed, so every submission faces the same track, the same zone winds and the same noise sequence. The turbulence a drone feels still depends on its own airspeed. The instructor chooses the official track and seed and keeps them private until the results are published. `config/race.json` draws practice races the same way, with a different track and wind for each `--seed`.

## 6. Constraints

| Constraint | Rule |
|---|---|
| Thrust | Clipped without penalty to [0, `thrust_limit_N`] of the observation the action was computed from. The limit is 15 N for a full battery with no load (about 14.6 N while hovering at full charge) and drops as the battery drains or sags under load. |
| Body rates | Each of p, q, r must be within ±3 rad/s. A command outside this range, a NaN, or a malformed action ends the run as `invalid_action`. |
| Battery | 3S LiPo model with the equations below. The 200 mAh pack lasts about 3.3 min in hover; the TA reference MPC finishes 5 laps with 40 to 60 % left, depending on the track. The battery mostly costs time through the lower thrust limit, but a slow or wasteful controller can drain it. Zero charge ends the run as `battery_depleted`. |
| Arena | The collision sphere must stay inside the arena box (`out_of_bounds` otherwise). |
| Gate frames | Each contact between the collision sphere and a gate bar adds 5 s; the flight continues. |

### Battery model

T is the applied (clipped) thrust and SoC ∈ [0, 1]:

```
power              P     = 3.75 W/N^1.5 · T^1.5                      (≈ 41 W at hover)
open-circuit volt  V_oc  = 9.9 V + (12.6 V − 9.9 V) · SoC
terminal voltage   V     = V_oc − R·I,  with P = V·I,  R = 0.05 Ω
discharge          dSoC/dt = −I / (200 mAh)
thrust limit       T_max = 15 N · (V / 12.6 V)²
```

Hard flying drains the pack faster, and both a drained pack (lower V_oc) and a large current draw (voltage sag R·I) lower the thrust limit. The observation's `battery_voltage_V` is the terminal voltage under the thrust applied during the previous control step, and `thrust_limit_N` is computed from it. The simulator uses the functions in `drone_race.battery`, and controllers may call them too.

## 7. Compute resources and fairness

- All submissions are scored one at a time on the same Linux server, so timing is comparable.
- The controller runs in its own process and exchanges only observations and actions with the simulator. On the scoring server each evaluation is pinned to one physical core and its memory is capped.
- BLAS, OpenMP, numba, JAX and similar libraries are limited to one thread, and there is no GPU. Run neural networks on the CPU: train offline on your own machine and submit the weights in `assets/`.
- The official environment is Python 3.12.12 with the versions pinned in `requirements.txt`. If you need a package that is not listed, ask the TAs at least one week before the competition; the list is frozen after that.
- 50 Hz compute budget:
  - `act()` must return within **20 ms** of CPU time on the official server. Each call is charged all CPU the whole controller process used since the previous call returned. OS preemption therefore never makes a call late, extra threads only use up the budget faster, and work done by background threads between calls counts too.
  - A late call's result is dropped. Simulated time keeps running while the controller computes: the previous action stays applied for as many 20 ms periods as the call took (rounded up), and the next observation comes after that. Each late call counts as one `deadline_misses`.
  - An `act()` call that has not returned after 5 s of wall-clock time is treated as hung and ends the run (`controller_timeout`).
  - Importing the submission and `reset()` together may use up to **60 s** of CPU time; more ends the run before the start (`controller_timeout`). Use `reset()` for offline trajectory optimization or JIT compilation.
- `drone-race run` enforces the 20 ms budget on your own computer, which may be faster or slower than the server. The practice server runs on the scoring hardware and shows the timing measured there; keep at least 30 % margin at the p99 `act()` time.

## 8. Submission rules

```
submission/
  controller.py        # defines Controller with reset(config) and act(observation)
  assets/              # optional: modules, learning code, weights, precomputed trajectories, ...
```

- Only the uploaded submission (`controller.py`, or a ZIP of the `submission/` folder) is evaluated. Changes to the simulator (`src/drone_race/`) or any other file are not used for scoring.
- The whole submission must be **50 MiB or less**; the practice server rejects a larger upload.
- Only the Python standard library, the packages in `requirements.txt` and `drone_race` itself may be imported.
- `drone-race check` must end with `OK` before you submit.
- Fair play: the controller must do all of its computing in its own process. Starting other processes (`subprocess`, `multiprocessing`, `os.fork`, ...) and tampering with the simulator, the timing measurement or the controller–simulator protocol (for example by replacing `time` functions or writing to the protocol stream) are forbidden. The TAs review the submitted code and disqualify any submission that does this.

## 9. Scoring

The score is a time in seconds; **lower is better**:

```
score = T_finish + 5 s × (gate collisions)
```

- T_finish is the simulated time at which the drone passes the finish line (the last gate) at the end of lap 5, interpolated within the physics step.
- A gate is passed when the center of the drone crosses the gate's plane in its +x direction inside the 2 m × 2 m opening, and the gate is the next one in order. Gates must be passed strictly in order (0 to N−1 on every lap, for N gates):
  - crossing any other gate does not count: after gate 3, flying through gate 5 does not pass gate 5, and gate 4 is still the next gate;
  - crossing a gate backwards does not count;
  - a gate that is never passed in order means the race cannot be completed (eventually `timeout`).
- A gate collision (+5 s) is the collision sphere (radius 0.28 m) touching a gate frame bar. There is no impact physics; the drone flies on.
  - Continuous contact is one collision, however long it lasts.
  - Separating from the frame and touching it again is a new collision.
  - Touching two bars of the same gate at the same time is one collision.
  - Brushing a bar while the center crosses inside the opening counts both the pass and the collision.
- The run does not finish (DNF) when
  - the drone leaves the arena (`out_of_bounds`),
  - the battery is empty (`battery_depleted`),
  - the time limit is reached (`timeout`),
  - the controller raises an exception (`controller_error`), hangs (`controller_timeout`) or returns an invalid action (`invalid_action`).

  On tracks with obstacles, hitting one also ends the run (`collision`); the competition track has none. A non-finite physics state ends the run as `simulation_error` and is reviewed by the TAs.
- Finished runs are ranked by score. Every DNF ranks below every finished run; among DNFs, more gates passed ranks higher, then a longer flight time. A disqualified submission is ranked as not finished.
- The log produced on the scoring server is final. Re-running the same seed can give slightly different results, because compute-time jitter can change which `act()` calls are late.
