# Autonomous Drone Racing Competition for Applied Optimal Control (M2795.007200), Fall 2026

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

## How to run

Implement `Controller.reset(config)` and `Controller.act(observation)` in `controller.py`.
The interface (observation, config and action) is documented at the top of that file.

```bash
# Simulate and score a controller (default: four-gate demo track)
quad-race run --controller controller.py --log outputs/nominal.json
quad-race run --controller controller.py --render            # live 3D view
quad-race run --render-to outputs/nominal.png                # headless image

# Ten-gate random track (the baseline controller needs ~51 s)
quad-race run --track tracks/ten_gates.json --max-time 90 --log outputs/ten_gates.json

# Disturbed scenario: smooth wind up to 0.5 m/s, mass error up to ±5%
quad-race run --wind 0.5 --mass-error 0.05 --seed 17 --log outputs/perturbed.json

# Generate a new random track
quad-race make-track tracks/random_seed7.json --gates 10 --seed 7

# Replay a saved log
quad-race replay outputs/nominal.json --show
quad-race replay outputs/nominal.json --output outputs/replay.gif
```

## References

- RotorPy: https://github.com/spencerfolk/rotorpy (MIT License, Copyright 2023 RotorPy Authors).
- Spencer Folk, James Paulos, Vijay Kumar, "RotorPy: A Python-based Multirotor Simulator with Aerodynamics for Education and Research", ICRA 2023 UAV simulator workshop: https://arxiv.org/abs/2306.04485.
- RotorPy is used as an external dependency; no modified copy of its source is included. Preserve its copyright and license notice when redistributing.
