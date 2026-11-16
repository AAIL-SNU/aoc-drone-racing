from dataclasses import replace
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import numpy as np
import pytest

from quad_race.api import Action, Box, PublicConfig
from quad_race.env import Scenario, run_episode, save_log, validate_action, validate_config
from quad_race.replay import RaceView, replay
from quad_race.track import demo_track

ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT / "submission"
CONFIG = PublicConfig(demo_track())


def flight(log):
    """Samples without the wall-clock act() timing, which differs between runs."""
    return [{k: v for k, v in s.items() if k not in ("act_s", "late")} for s in log["samples"]]


@pytest.fixture(scope="module")
def nominal():
    return run_episode(BASELINE, CONFIG)


def test_nominal_baseline_completes_all_four_gates(nominal):
    assert nominal["summary"]["status"] == "finished", nominal["summary"]
    assert nominal["summary"]["gates_passed"] == 4
    assert [e["gate"] for e in nominal["events"] if e["kind"] == "gate_passed"] == [0, 1, 2, 3]
    assert nominal["summary"]["flight_time_s"] < 45
    assert nominal["summary"]["lap_times_s"] == [nominal["summary"]["flight_time_s"]]
    assert nominal["summary"]["timing_enforced"] and nominal["provenance"]["isolation"] == "process"


def test_controller_replacement_hover_is_a_timeout():
    log = run_episode(ROOT / "examples/hover.py", CONFIG, Scenario(max_time_s=0.2))
    assert log["summary"]["status"] == "timeout"
    assert not log["summary"]["completed"]
    assert log["summary"]["gates_passed"] == 0
    assert log["summary"]["flight_time_s"] == pytest.approx(0.2)


def test_episode_collision_has_failure_status_and_penalty():
    track = replace(CONFIG.track, obstacles=(Box((-0.1, -0.1, 1.4), (0.1, 0.1, 1.6)),))
    log = run_episode(ROOT / "examples/hover.py", replace(CONFIG, track=track), Scenario(max_time_s=0.2))
    assert log["summary"]["status"] == "collision"
    assert not log["summary"]["completed"]
    assert log["summary"]["failure_penalty_s"] == 30.0


@pytest.mark.parametrize("action", [Action(float("nan"), (0, 0, 0)), Action(5, (0, 0)),
                                     Action(100, (0, 0, 0)), Action(5, (4, 0, 0)),
                                     Action(5, (0, float("inf"), 0)), None])
def test_invalid_action_rejected(action):
    with pytest.raises(ValueError):
        validate_action(action, CONFIG)


def test_invalid_submission_is_reported_in_score(tmp_path):
    path = tmp_path / "controller.py"
    path.write_text("from quad_race import Action\nclass Controller:\n def reset(self,c): pass\n def act(self,o): return Action(float('nan'),(0,0,0))\n")
    assert run_episode(path, CONFIG)["summary"]["status"] == "invalid_action"


def test_rendering_observer_does_not_change_flight_or_events(nominal, tmp_path):
    view = RaceView(CONFIG.track)
    try:
        rendered = run_episode(BASELINE, CONFIG, observer=view.update)
        view.save(tmp_path / "rendered.png")
    finally:
        view.close()
    assert flight(rendered) == flight(nominal)
    assert rendered["events"] == nominal["events"]
    assert rendered["summary"]["score_s"] == nominal["summary"]["score_s"]
    assert (tmp_path / "rendered.png").stat().st_size > 1000


def test_log_roundtrip_and_replay_does_not_change_log(nominal, tmp_path):
    path, output = tmp_path / "flight.json", tmp_path / "replay.png"
    save_log(nominal, path)
    before = path.read_bytes()
    assert replay(path, output=output) == nominal["summary"]
    assert json.loads(path.read_text()) == json.loads(json.dumps(nominal))
    assert path.read_bytes() == before
    assert output.stat().st_size > 1000


def test_small_wind_and_mass_error_reproducible_and_completes():
    scenario = Scenario(seed=17, wind_max_m_s=0.5, mass_error_fraction=0.05)
    a, b = run_episode(BASELINE, CONFIG, scenario), run_episode(BASELINE, CONFIG, scenario)
    assert a["summary"]["completed"], a["summary"]
    assert flight(a) == flight(b)
    assert a["events"] == b["events"]
    assert a["summary"]["score_s"] == b["summary"]["score_s"]
    assert max(np.linalg.norm(s["wind_m_s"]) for s in a["samples"]) <= scenario.wind_max_m_s
    assert 0.95 <= a["realization"]["mass_scale"] <= 1.05
    c = run_episode(BASELINE, CONFIG, replace(scenario, seed=18, max_time_s=0.1))
    assert c["realization"] != a["realization"]


def test_physics_step_halving_has_consistent_completion(nominal):
    fine = run_episode(BASELINE, replace(CONFIG, physics_dt_s=0.001))
    assert fine["summary"]["completed"], fine["summary"]
    assert abs(fine["summary"]["flight_time_s"] - nominal["summary"]["flight_time_s"]) < 0.05


def test_plant_preserves_students_numpy_rng():
    from quad_race.plant import RotorPyPlant
    state = np.random.get_state()
    try:
        np.random.seed(123)
        expected = np.random.random(3)
        np.random.seed(123)
        plant = RotorPyPlant(CONFIG)
        plant.step(Action(4.905, (0, 0, 0)), 0.002, np.zeros(3))
        assert np.array_equal(np.random.random(3), expected)
    finally:
        np.random.set_state(state)


@pytest.mark.parametrize("dt", [0.0, -0.01, float("nan")])
def test_bad_time_step_rejected(dt):
    with pytest.raises(ValueError):
        validate_config(replace(CONFIG, physics_dt_s=dt))
