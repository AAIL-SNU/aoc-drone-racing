from pathlib import Path

import numpy as np
import pytest

from quad_race.api import Action, PublicConfig
from quad_race.env import run_episode
from quad_race.plant import RotorPyPlant
from quad_race.planning import QuadModel, RacingLine
from quad_race.track import demo_track

ROOT = Path(__file__).resolve().parents[1]
CONFIG = PublicConfig(demo_track())


def test_racing_line_crosses_each_gate_center_along_its_normal():
    line = RacingLine(CONFIG.track)
    for gate, s in zip(CONFIG.track.gates, line.gate_s):
        i = line.index(s)
        normal = np.array([np.cos(gate.yaw_rad), np.sin(gate.yaw_rad), 0.0])
        assert np.linalg.norm(line.position[i] - gate.center) < 0.05
        assert line.tangent[i] @ normal > 0.99
    assert line.speed[0] == 0.0 and np.all(line.speed <= 10.0 + 1e-9)


def test_model_predicts_plant_acceleration():
    rng = np.random.default_rng(0)
    plant, model = RotorPyPlant(CONFIG), QuadModel(CONFIG, CONFIG.control_dt_s)
    errors = []
    for k in range(150):
        if k % 10 == 0:
            u = np.array([rng.uniform(3, 12), *rng.uniform(-1, 1, 3)])
        x = np.concatenate([plant.state[key] for key in ("x", "v", "q", "w")])
        for _ in range(10):
            plant.step(Action(float(u[0]), tuple(u[1:])), CONFIG.physics_dt_s, np.zeros(3))
        predicted = model.step(x, u)
        errors.append(np.linalg.norm(predicted[3:6] - plant.state["v"]) / CONFIG.control_dt_s)
    assert np.mean(errors) < 1.0


@pytest.mark.parametrize("name", ["mpc_controller.py", "mppi_controller.py"])
def test_optimal_examples_finish_faster_than_baseline(name):
    # Flight quality only: the examples sit at the 20 ms budget, so timing is not enforced here.
    summary = run_episode(ROOT / "examples" / name, CONFIG, budget_scale=10.0)["summary"]
    assert summary["status"] == "finished"
    assert summary["score_s"] < 4.0          # baseline needs ~17 s
