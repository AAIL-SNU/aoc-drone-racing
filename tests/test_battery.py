"""Battery: the public model is exactly what the simulator applies."""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from drone_race import battery
from drone_race.api import PublicConfig
from drone_race.env import Scenario, run_episode, validate_config
from drone_race.track import demo_track

ROOT = Path(__file__).resolve().parents[1]
CONFIG = PublicConfig(demo_track())
HOVER = ROOT / "tests/controllers/hover.py"


def constant_thrust(tmp_path, thrust):
    path = tmp_path / "controller.py"
    path.write_text("from drone_race import Action\nclass Controller:\n def reset(self, c): pass\n"
                    f" def act(self, o): return Action({thrust!r}, (0.0, 0.0, 0.0))\n")
    return path


def test_power_balance_and_monotone_limits():
    soc, thrust = np.linspace(0, 1, 11)[:, None], np.linspace(0, CONFIG.thrust_max_N, 7)[None]
    v, i = battery.terminal_voltage(soc, thrust, CONFIG), battery.current(soc, thrust, CONFIG)
    assert np.allclose(v * i, battery.power(thrust, CONFIG))
    assert np.all(np.diff(v, axis=0) > 0) and np.all(np.diff(v, axis=1) <= 0)
    full = battery.thrust_limit(battery.terminal_voltage(1.0, 0.0, CONFIG), CONFIG)
    assert full == pytest.approx(CONFIG.thrust_max_N)
    # Even empty and at full draw the pack can still hover: depletion, not sag, ends a run.
    empty = battery.thrust_limit(battery.terminal_voltage(0.0, CONFIG.thrust_max_N, CONFIG), CONFIG)
    assert CONFIG.nominal_mass_kg * CONFIG.gravity_m_s2 < empty < full


def test_observations_and_log_follow_the_public_model():
    log = run_episode(HOVER, CONFIG, Scenario(max_time_s=0.5))
    model = battery.BatteryModel(CONFIG)
    soc = 1.0
    substeps = round(CONFIG.control_dt_s / CONFIG.physics_dt_s)
    for before, after in zip(log["samples"], log["samples"][1:]):
        thrust = after["action"]["thrust_N"]
        assert before["thrust_limit_N"] == pytest.approx(battery.thrust_limit(before["battery_voltage_V"], CONFIG))
        for _ in range(substeps):
            soc = model.step(soc, thrust, CONFIG.physics_dt_s)
        assert after["battery_soc"] == pytest.approx(soc, rel=1e-12)
        assert after["battery_voltage_V"] == pytest.approx(model.voltage(soc, thrust))
    assert log["summary"]["battery_final_soc"] == pytest.approx(soc)
    assert 0 < log["summary"]["battery_energy_Wh"]


def test_excess_thrust_is_clipped_not_rejected(tmp_path):
    log = run_episode(constant_thrust(tmp_path, 100.0), CONFIG, Scenario(max_time_s=0.1))
    assert log["summary"]["status"] != "invalid_action"
    # Every flown control step but the first (the initial hover command) is clipped.
    assert log["summary"]["thrust_saturated_steps"] == len(log["samples"]) - 2
    # One-step actuation delay: the first control step flies the initial hover command, and
    # the command from the observation in sample i is flown in the step that ends at sample i + 2.
    assert log["samples"][1]["action"]["thrust_N"] == pytest.approx(CONFIG.nominal_mass_kg * CONFIG.gravity_m_s2)
    for observed, flown in zip(log["samples"], log["samples"][2:]):
        assert flown["thrust_requested_N"] == 100.0
        assert flown["action"]["thrust_N"] == pytest.approx(observed["thrust_limit_N"])
        assert flown["action"]["thrust_N"] < CONFIG.thrust_max_N
    negative = run_episode(constant_thrust(tmp_path, -3.0), CONFIG, Scenario(max_time_s=0.04))
    assert negative["samples"][-1]["action"]["thrust_N"] == CONFIG.thrust_min_N


def test_depleted_battery_ends_the_run():
    log = run_episode(HOVER, replace(CONFIG, battery_capacity_mAh=0.2), Scenario(max_time_s=5.0))
    s = log["summary"]
    assert s["status"] == "battery_depleted" and not s["completed"]
    assert s["failure_penalty_s"] > 0 and s["battery_final_soc"] == 0.0
    assert log["events"][-1]["kind"] == "battery_depleted"
    assert s["flight_time_s"] < 5.0


@pytest.mark.parametrize("change", [dict(battery_capacity_mAh=0.0), dict(battery_voltage_empty_V=13.0),
                                    dict(battery_resistance_ohm=-0.1), dict(battery_resistance_ohm=10.0),
                                    dict(battery_power_coeff=float("nan"))])
def test_bad_battery_parameters_rejected(change):
    with pytest.raises(ValueError):
        validate_config(replace(CONFIG, **change))


def test_worker_controller_receives_battery_and_gate_fields(tmp_path):
    path = tmp_path / "controller.py"
    path.write_text(
        "from drone_race import Action\n"
        "class Controller:\n"
        " def reset(self, c): self.n = len(c.track.gates)\n"
        " def act(self, o):\n"
        "  assert o.gates_passed == o.lap * self.n + o.next_gate and o.gate_collisions == 0\n"
        "  assert 0 < o.battery_soc <= 1 and o.battery_voltage_V > 9\n"
        "  return Action(o.thrust_limit_N, (0.0, 0.0, 0.0))\n")
    log = run_episode(path, CONFIG, Scenario(max_time_s=0.2))
    assert log["summary"]["status"] == "timeout", log["summary"]["error"]
    assert log["summary"]["thrust_saturated_steps"] == 0
