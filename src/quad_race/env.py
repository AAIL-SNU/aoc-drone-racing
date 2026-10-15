"""Authoritative simulation clock, disturbances, event checks and scoring.

This first prototype loads trusted controllers in-process. It is not an execution
sandbox; a hard watchdog and process isolation belong in the official evaluator.
"""

from dataclasses import asdict, dataclass
import hashlib
import importlib.metadata
import importlib.util
import json
from pathlib import Path
import platform
import time

import numpy as np

from .api import Action, Observation, PublicConfig
from .plant import RotorPyPlant
from .rules import RaceRules
from .track import track_to_dict


@dataclass(frozen=True)
class Scenario:
    seed: int = 0
    wind_max_m_s: float = 0.0
    mass_error_fraction: float = 0.0
    max_time_s: float = 45.0
    load_limit_g: float = 3.0
    load_penalty_s: float = 2.0
    failure_penalty_s: float = 30.0

    def __post_init__(self):
        values = (self.wind_max_m_s, self.mass_error_fraction, self.max_time_s,
                  self.load_limit_g, self.load_penalty_s, self.failure_penalty_s)
        if not all(np.isfinite(values)):
            raise ValueError("Scenario values must be finite")
        if not 0 <= self.mass_error_fraction <= 0.2 or not 0 <= self.wind_max_m_s <= 2.0:
            raise ValueError("Prototype supports wind in [0, 2] m/s and mass error in [0, 0.2]")
        if self.max_time_s <= 0 or self.load_limit_g <= 0 or min(self.load_penalty_s, self.failure_penalty_s) < 0:
            raise ValueError("Invalid duration or penalty")


def load_controller(path: str | Path):
    path = Path(path).resolve()
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    spec = importlib.util.spec_from_file_location(f"submission_{digest[:16]}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    controller = module.Controller()
    if not callable(getattr(controller, "reset", None)) or not callable(getattr(controller, "act", None)):
        raise TypeError("Controller must implement reset(config) and act(observation)")
    return controller, digest


def validate_action(action: Action, config: PublicConfig) -> Action:
    if not isinstance(action, Action):
        raise ValueError("act() must return quad_race.Action")
    rates = np.asarray(action.body_rates_rad_s, dtype=float)
    thrust = np.asarray(action.thrust_N, dtype=float)
    if rates.shape != (3,) or thrust.shape != () or not np.all(np.isfinite(rates)) or not np.isfinite(thrust):
        raise ValueError("Action must contain one finite thrust and three finite body rates")
    if not config.thrust_min_N <= thrust <= config.thrust_max_N:
        raise ValueError("Thrust outside public limits")
    if np.any(np.abs(rates) > config.body_rate_limit_rad_s + 1e-12):
        raise ValueError("Body rate command outside public limits")
    return Action(float(thrust), tuple(float(x) for x in rates))


def observation(state, time_s: float, next_gate: int) -> Observation:
    return Observation(time_s, tuple(state["x"]), tuple(state["v"]), tuple(state["q"]), tuple(state["w"]), next_gate)


def validate_config(config: PublicConfig):
    if not np.isfinite(config.physics_dt_s) or not np.isfinite(config.control_dt_s) or min(config.physics_dt_s, config.control_dt_s) <= 0:
        raise ValueError("Time steps must be finite and positive")
    ratio = config.control_dt_s / config.physics_dt_s
    if not np.isclose(ratio, round(ratio)):
        raise ValueError("control_dt_s must be a positive integer multiple of physics_dt_s")
    if not config.track.gates or config.vehicle_radius_m <= 0:
        raise ValueError("A track requires gates and a positive vehicle radius")
    if config.nominal_mass_kg != 0.5 or config.body_rate_gain_s_inv != 25.0:
        raise ValueError("Prototype plant uses a fixed nominal 0.5 kg Hummingbird and rate gain 25/s")
    if config.gravity_m_s2 != 9.81:
        raise ValueError("RotorPy backend uses gravity 9.81 m/s^2")
    for gate in config.track.gates:
        if min(gate.width_m, gate.height_m) <= 2 * config.vehicle_radius_m or gate.frame_m <= 0:
            raise ValueError("Gate opening must accommodate the vehicle sphere")


def run_episode(controller_path, config: PublicConfig, scenario: Scenario = Scenario(), observer=None) -> dict:
    validate_config(config)
    controller, digest = load_controller(controller_path)
    rng = np.random.default_rng(scenario.seed)
    mass_scale = 1 + rng.uniform(-scenario.mass_error_fraction, scenario.mass_error_fraction)
    phase = rng.uniform(0, 2 * np.pi, 3)

    def wind_at(t):
        # Smooth deterministic gust; vector magnitude is bounded by wind_max_m_s.
        return scenario.wind_max_m_s / np.sqrt(3) * np.sin(0.7 * t + phase)

    plant = RotorPyPlant(config, mass_scale)
    rules = RaceRules(config.track, config.vehicle_radius_m)
    records, events, act_times = [], [], []
    time_s, load_integral, peak_load = 0.0, 0.0, 0.0
    status, error, init_time = "timeout", None, 0.0
    started = time.perf_counter()

    def record(action=None):
        item = {"time_s": time_s, "position_m": plant.state["x"].tolist(),
                "velocity_m_s": plant.state["v"].tolist(), "quaternion_xyzw": plant.state["q"].tolist(),
                "body_rates_rad_s": plant.state["w"].tolist(), "next_gate": rules.next_gate,
                "wind_m_s": plant.state["wind"].tolist(), "action": asdict(action) if action else None}
        records.append(item)
        if observer is not None:
            observer(item)

    try:
        t0 = time.perf_counter()
        controller.reset(config)
        init_time = time.perf_counter() - t0
    except Exception as exc:
        status, error = "controller_error", f"reset: {type(exc).__name__}: {exc}"
    record()
    done = status == "controller_error"
    substeps = round(config.control_dt_s / config.physics_dt_s)
    while time_s < scenario.max_time_s - 1e-12 and not done:
        try:
            t0 = time.perf_counter()
            raw_action = controller.act(observation(plant.state, time_s, rules.next_gate))
            act_times.append(time.perf_counter() - t0)
        except Exception as exc:
            status, error, done = "controller_error", f"act: {type(exc).__name__}: {exc}", True
            break
        try:
            action = validate_action(raw_action, config)
        except (ValueError, TypeError, OverflowError) as exc:
            status, error, done = "invalid_action", str(exc), True
            break
        for _ in range(substeps):
            dt = min(config.physics_dt_s, scenario.max_time_s - time_s)
            if dt <= 1e-12:
                break
            before = {key: value.copy() for key, value in plant.state.items()}
            try:
                plant.step(action, dt, wind_at(time_s))
                if not all(np.all(np.isfinite(value)) for value in plant.state.values()):
                    raise FloatingPointError("Non-finite plant state")
            except Exception as exc:
                plant.state = before
                status, error, done = "simulation_error", str(exc), True
                break
            step_events = rules.inspect(before["x"], plant.state["x"])
            terminal = next((event for event in step_events if event.kind != "gate_passed"), None)
            fraction = terminal.fraction if terminal else 1.0
            elapsed = dt * fraction
            load = np.linalg.norm((plant.state["v"] - before["v"]) / dt + [0, 0, config.gravity_m_s2]) / config.gravity_m_s2
            peak_load = max(peak_load, float(load))
            load_integral += max(0.0, load - scenario.load_limit_g) ** 2 * elapsed
            for event in step_events:
                events.append({"kind": event.kind, "time_s": time_s + event.fraction * dt, "gate": event.gate})
            if terminal:
                # Terminal position/time use the same linear within-step event estimate.
                plant.state = {key: before[key] + fraction * (value - before[key]) for key, value in plant.state.items()}
                plant.state["q"] /= np.linalg.norm(plant.state["q"])
                status, done = terminal.kind, True
            time_s += elapsed
            if done:
                break
        record(action)
    if status == "timeout":
        events.append({"kind": "timeout", "time_s": time_s, "gate": rules.next_gate})
    failure = 0.0 if status == "finished" else scenario.failure_penalty_s
    summary = {
        "status": status, "completed": status == "finished", "flight_time_s": time_s,
        "gates_passed": rules.next_gate, "gates_total": len(config.track.gates),
        # DNF scores are diagnostics only and must never rank ahead of completed runs.
        "score_s": time_s + scenario.load_penalty_s * load_integral + failure,
        "load_penalty_s": scenario.load_penalty_s * load_integral, "failure_penalty_s": failure,
        "peak_load_g": peak_load, "controller_reset_s": init_time,
        "controller_mean_act_s": float(np.mean(act_times)) if act_times else 0.0,
        "controller_max_act_s": max(act_times, default=0.0),
        "wall_time_s": time.perf_counter() - started, "error": error,
    }
    return {
        "schema_version": 1, "config": {**asdict(config), "track": track_to_dict(config.track)}, "scenario": asdict(scenario),
        "realization": {"mass_scale": mass_scale, "wind_phase_rad": phase.tolist()},
        "provenance": {"controller_sha256": digest, "python": platform.python_version(),
                       "packages": {name: importlib.metadata.version(name) for name in ("quad-race", "rotorpy", "numpy", "scipy")}},
        "summary": summary, "events": events, "samples": records,
    }


def save_log(log: dict, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(log, indent=2, allow_nan=False) + "\n")
