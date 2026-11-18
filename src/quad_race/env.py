"""Authoritative simulation clock, disturbances, event checks and scoring.

The controller runs in a separate worker process (quad_race.worker) and only ever
sees Observation messages. ``debug=True`` loads it into this process instead, for
breakpoints; timing is then measured but not enforced.
"""

from dataclasses import asdict, dataclass
import importlib.metadata
import hashlib
import json
import os
from pathlib import Path
import platform
import time

import numpy as np

from .api import Action, Observation, PublicConfig
from .plant import RotorPyPlant
from .protocol import action_from_wire
from .rules import RaceRules, TERMINAL
from .scoring import POLICY_VERSION, GATE_COLLISION_PENALTY_S, score_run
from . import submission as submissions
from .track import track_to_dict
from .worker import ControllerError, InlineController, WorkerController

SCHEMA_VERSION = 2
DEFAULT_SECONDS_PER_LAP = 45.0


@dataclass(frozen=True)
class Scenario:
    seed: int = 0
    wind_max_m_s: float = 0.0
    mass_error_fraction: float = 0.0
    max_time_s: float | None = None          # None: DEFAULT_SECONDS_PER_LAP per lap

    def __post_init__(self):
        values = (self.wind_max_m_s, self.mass_error_fraction)
        if not all(np.isfinite(values)) or (self.max_time_s is not None and not np.isfinite(self.max_time_s)):
            raise ValueError("Scenario values must be finite")
        if not 0 <= self.mass_error_fraction <= 0.2 or not 0 <= self.wind_max_m_s <= 2.0:
            raise ValueError("Prototype supports wind in [0, 2] m/s and mass error in [0, 0.2]")
        if self.max_time_s is not None and self.max_time_s <= 0:
            raise ValueError("Invalid duration")

    def time_limit(self, laps: int) -> float:
        return self.max_time_s if self.max_time_s is not None else DEFAULT_SECONDS_PER_LAP * laps


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


def observation(state, time_s: float, next_gate: int, lap: int = 0) -> Observation:
    return Observation(time_s, tuple(state["x"]), tuple(state["v"]), tuple(state["q"]), tuple(state["w"]), next_gate, lap)


def validate_config(config: PublicConfig):
    if not np.isfinite(config.physics_dt_s) or not np.isfinite(config.control_dt_s) or min(config.physics_dt_s, config.control_dt_s) <= 0:
        raise ValueError("Time steps must be finite and positive")
    ratio = config.control_dt_s / config.physics_dt_s
    if not np.isclose(ratio, round(ratio)):
        raise ValueError("control_dt_s must be a positive integer multiple of physics_dt_s")
    if not config.track.gates or config.vehicle_radius_m <= 0:
        raise ValueError("A track requires gates and a positive vehicle radius")
    if not isinstance(config.laps, int) or config.laps < 1:
        raise ValueError("laps must be a positive integer")
    if not min(config.act_budget_s, config.reset_budget_s) > 0:
        raise ValueError("Compute budgets must be positive")
    if config.nominal_mass_kg != 0.5 or config.body_rate_gain_s_inv != 25.0:
        raise ValueError("Prototype plant uses a fixed nominal 0.5 kg Hummingbird and rate gain 25/s")
    if config.gravity_m_s2 != 9.81:
        raise ValueError("RotorPy backend uses gravity 9.81 m/s^2")
    for gate in config.track.gates:
        if min(gate.width_m, gate.height_m) <= 2 * config.vehicle_radius_m or gate.frame_m <= 0:
            raise ValueError("Gate opening must accommodate the vehicle sphere")


def hard_act_timeout(budget_s: float) -> float:
    """act() calls this slow are not late but hung: the worker is killed and the run ends."""
    return max(0.5, 25 * budget_s)


def run_episode(submission, config: PublicConfig, scenario: Scenario = Scenario(), observer=None, *,
                debug: bool = False, budget_scale: float = 1.0, cpu=None, command_prefix=None,
                mount_root=None, stderr=None, isolation: str | None = None) -> dict:
    """Simulate one race.

    ``budget_scale`` multiplies config.act_budget_s/reset_budget_s, to emulate the
    official machine on a faster or slower computer (see ``drone-race bench``).
    A late act() result is discarded and the previous action is held for that step.
    """
    validate_config(config)
    sub = submissions.resolve(submission)
    digest = sub.sha256()
    rng = np.random.default_rng(scenario.seed)
    mass_scale = 1 + rng.uniform(-scenario.mass_error_fraction, scenario.mass_error_fraction)
    phase = rng.uniform(0, 2 * np.pi, 3)
    max_time = scenario.time_limit(config.laps)

    def wind_at(t):
        # Smooth deterministic gust; vector magnitude is bounded by wind_max_m_s.
        return scenario.wind_max_m_s / np.sqrt(3) * np.sin(0.7 * t + phase)

    plant = RotorPyPlant(config, mass_scale)
    rules = RaceRules(config.track, config.vehicle_radius_m, config.laps)
    records, events, act_times = [], [], []
    time_s, peak_load, peak_tilt, misses = 0.0, 0.0, 0.0, 0
    status, error, init_time, modules = "timeout", None, 0.0, []
    budget = config.act_budget_s * budget_scale
    started = time.perf_counter()
    held = Action(config.nominal_mass_kg * config.gravity_m_s2, (0.0, 0.0, 0.0))

    def record(action=None, act_s=None, late=False):
        item = {"time_s": time_s, "position_m": plant.state["x"].tolist(),
                "velocity_m_s": plant.state["v"].tolist(), "quaternion_xyzw": plant.state["q"].tolist(),
                "body_rates_rad_s": plant.state["w"].tolist(), "next_gate": rules.next_gate, "lap": rules.lap,
                "wind_m_s": plant.state["wind"].tolist(), "action": asdict(action) if action else None,
                "act_s": act_s, "late": late}
        records.append(item)
        if observer is not None:
            observer(item)

    handle = None
    try:
        handle = (InlineController(sub) if debug else
                  WorkerController(sub, command_prefix=command_prefix, mount_root=mount_root, cpu=cpu, stderr=stderr))
        modules = handle.hello.get("modules", [])
        reply = handle.reset(config, timeout=config.reset_budget_s * budget_scale)
        init_time, modules = reply["elapsed"], reply.get("modules", modules)
    except ControllerError as exc:
        status, error = exc.status, str(exc)
    enforced = handle is not None and handle.enforces_timing
    record()
    done = error is not None
    substeps = round(config.control_dt_s / config.physics_dt_s)
    while time_s < max_time - 1e-12 and not done:
        try:
            reply = handle.act(observation(plant.state, time_s, rules.next_gate, rules.lap),
                               timeout=hard_act_timeout(budget))
        except ControllerError as exc:
            status, error = exc.status, str(exc)
            break
        act_times.append(reply["elapsed"])
        late = reply["elapsed"] > budget
        misses += late
        if late and enforced:
            action = held
        else:
            try:
                if reply["invalid"]:
                    raise ValueError(reply["invalid"])
                action = validate_action(action_from_wire(reply["action"]), config)
            except (ValueError, TypeError, OverflowError, IndexError) as exc:
                status, error = "invalid_action", str(exc)
                break
        held = action
        for _ in range(substeps):
            dt = min(config.physics_dt_s, max_time - time_s)
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
            terminal = next((e for e in step_events if e.kind in TERMINAL), None)
            fraction = terminal.fraction if terminal else 1.0
            elapsed = dt * fraction
            load = np.linalg.norm((plant.state["v"] - before["v"]) / dt + [0, 0, config.gravity_m_s2]) / config.gravity_m_s2
            peak_load = max(peak_load, float(load))
            qx, qy = plant.state["q"][0], plant.state["q"][1]
            peak_tilt = max(peak_tilt, float(np.degrees(np.arccos(np.clip(1 - 2 * (qx * qx + qy * qy), -1, 1)))))
            for event in step_events:
                events.append({"event_id": f"event-{len(events) + 1}", "kind": event.kind, "time_s": time_s + event.fraction * dt,
                               "gate": event.gate, "lap": event.lap})
            if terminal:
                # Terminal position/time use the same linear within-step event estimate.
                plant.state = {key: before[key] + fraction * (value - before[key]) for key, value in plant.state.items()}
                plant.state["q"] /= np.linalg.norm(plant.state["q"])
                status, done = terminal.kind, True
            time_s += elapsed
            if done:
                break
        record(action, reply["elapsed"], bool(late))
    if handle is not None:
        handle.close()
    if status == "timeout":
        events.append({"kind": "timeout", "time_s": time_s, "gate": rules.next_gate, "lap": rules.lap})
    return _log(config, scenario, sub, digest, debug, isolation, budget, enforced, mass_scale, phase, records, events,
                act_times, misses, status, error, time_s, rules, peak_load, peak_tilt, init_time, modules, started)


def _log(config, scenario, sub, digest, debug, isolation, budget, enforced, mass_scale, phase, records, events,
         act_times, misses, status, error, time_s, rules, peak_load, peak_tilt, init_time, modules, started):
    lap_ends = [e["time_s"] for e in events if e["kind"] == "lap_completed"]
    lap_times = list(np.diff([0.0, *lap_ends]))
    missed = sum(e["kind"] == "gate_missed" for e in events)
    infractions = sum(e["kind"] == "wrong_way" for e in events)
    score = score_run(status, time_s, events)
    times = np.asarray(act_times) if act_times else np.zeros(1)
    summary = {
        "status": status, "completed": status == "finished", "flight_time_s": time_s,
        "laps_total": config.laps, "laps_completed": len(lap_ends),
        "lap_times_s": [float(t) for t in lap_times], "fastest_lap_s": float(min(lap_times)) if lap_times else None,
        "gates_total": config.laps * len(config.track.gates), "gates_resolved": rules.progress,
        "gates_passed": sum(e["kind"] == "gate_passed" for e in events), "gates_missed": missed,
        "infractions": infractions, "failure_penalty_s": 0.0,
        **score,
        "peak_load_g": peak_load, "peak_tilt_deg": peak_tilt,
        "controller_reset_s": init_time,
        "controller_mean_act_s": float(times.mean()), "controller_p99_act_s": float(np.percentile(times, 99)),
        "controller_max_act_s": float(times.max()),
        "act_budget_s": budget, "deadline_misses": int(misses), "timing_enforced": bool(enforced),
        "wall_time_s": time.perf_counter() - started, "error": error,
    }
    return {
        "schema_version": SCHEMA_VERSION, "config": {**asdict(config), "track": track_to_dict(config.track)},
        "scoring_policy": {"version": POLICY_VERSION, "gate_collision_penalty_s": GATE_COLLISION_PENALTY_S},
        "scenario": asdict(scenario),
        "realization": {"mass_scale": mass_scale, "wind_phase_rad": phase.tolist()},
        "provenance": {
            "simulator_sha256": hashlib.sha256(b"".join(p.name.encode() + p.read_bytes()
                for p in sorted(Path(__file__).parent.glob("*.py")))).hexdigest(),
            "submission_sha256": digest, "submission": sub.entry.name if sub.entry.name != "controller.py" else sub.root.name,
            "isolation": isolation or ("inline" if debug else "process"),
            "third_party_modules": modules, "python": platform.python_version(),
            "machine": {"platform": platform.platform(), "processor": platform.processor() or platform.machine(),
                        "cpus": os.cpu_count()},
            "packages": {name: importlib.metadata.version(name) for name in ("quad-race", "rotorpy", "numpy", "scipy")},
        },
        "summary": summary, "events": events, "samples": records,
    }


def save_log(log: dict, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(log, indent=2, allow_nan=False) + "\n")
