"""Authoritative simulation clock, disturbances, event checks and scoring.

The controller runs in a separate worker process (drone_race.worker) and only ever
sees Observation messages. ``debug=True`` loads it into this process instead, for
breakpoints; timing is then measured but not enforced.
"""

from dataclasses import asdict, dataclass
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import time

import numpy as np

from .api import Action, Observation, PublicConfig
from .battery import Battery
from .plant import RotorPyPlant
from .protocol import action_from_wire
from .rules import RaceRules
from .wind import WindField, WindZone
from . import submission as submissions
from .track import track_to_dict
from .worker import ControllerError, InlineController, WorkerController

SCHEMA_VERSION = 2
DEFAULT_SECONDS_PER_LAP = 45.0


@dataclass(frozen=True)
class Scenario:
    seed: int = 0
    turbulence_m_s: float = 0.0              # horizontal turbulence std; vertical is half (see drone_race.wind)
    wind_zones: tuple[WindZone, ...] = ()    # wind boxes on the course; the seed draws each zone's strength
    max_time_s: float | None = None          # None: DEFAULT_SECONDS_PER_LAP per lap
    gate_miss_penalty_s: float = 5.0
    infraction_penalty_s: float = 1.0
    failure_penalty_s: float = 30.0          # diagnostic only; unfinished runs rank after finished ones

    def __post_init__(self):
        zones = tuple(z if isinstance(z, WindZone) else WindZone(**z) for z in self.wind_zones)
        object.__setattr__(self, "wind_zones", zones)
        values = (self.turbulence_m_s, self.gate_miss_penalty_s, self.infraction_penalty_s, self.failure_penalty_s)
        if not all(np.isfinite(values)) or (self.max_time_s is not None and not np.isfinite(self.max_time_s)):
            raise ValueError("Scenario values must be finite")
        if not 0 <= self.turbulence_m_s <= 2.0:
            raise ValueError("turbulence_m_s must be in [0, 2] m/s")
        if (self.max_time_s is not None and self.max_time_s <= 0) or min(values[1:]) < 0:
            raise ValueError("Invalid duration or penalty")

    def time_limit(self, laps: int) -> float:
        return self.max_time_s if self.max_time_s is not None else DEFAULT_SECONDS_PER_LAP * laps


def validate_action(action: Action, config: PublicConfig) -> Action:
    """Well-formed action with finite values and body rates in range. Thrust is clipped later (clip_thrust)."""
    if not isinstance(action, Action):
        raise ValueError("act() must return drone_race.Action")
    rates = np.asarray(action.body_rates_rad_s, dtype=float)
    thrust = np.asarray(action.thrust_N, dtype=float)
    if rates.shape != (3,) or thrust.shape != () or not np.all(np.isfinite(rates)) or not np.isfinite(thrust):
        raise ValueError("Action must contain one finite thrust and three finite body rates")
    if np.any(np.abs(rates) > config.body_rate_limit_rad_s + 1e-12):
        raise ValueError("Body rate command outside public limits")
    return Action(float(thrust), tuple(float(x) for x in rates))


def clip_thrust(action: Action, config: PublicConfig, thrust_limit: float) -> Action:
    """The motors saturate: thrust is clipped to [thrust_min_N, thrust_limit] without penalty."""
    return Action(float(np.clip(action.thrust_N, config.thrust_min_N, thrust_limit)), action.body_rates_rad_s)


def observation(state, time_s: float, rules, gates_passed: int, gates_missed: int, battery: Battery) -> Observation:
    return Observation(time_s, tuple(state["x"]), tuple(state["v"]), tuple(state["q"]), tuple(state["w"]),
                       rules.next_gate, rules.lap, gates_passed, gates_missed,
                       battery.soc, battery.voltage, battery.thrust_limit)


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
    battery = (config.battery_capacity_mAh, config.battery_voltage_full_V, config.battery_voltage_empty_V,
               config.battery_resistance_ohm, config.battery_power_coeff)
    if not all(np.isfinite(battery)) or min(battery) < 0 or min(battery[:3]) <= 0:
        raise ValueError("Battery parameters must be finite and positive")
    if config.battery_voltage_empty_V >= config.battery_voltage_full_V:
        raise ValueError("battery_voltage_empty_V must be below battery_voltage_full_V")
    max_power = config.battery_power_coeff * config.thrust_max_N ** 1.5
    if 4 * config.battery_resistance_ohm * max_power >= config.battery_voltage_empty_V ** 2:
        raise ValueError("Battery resistance too high to deliver full thrust")


def hard_act_timeout(budget_s: float) -> float:
    """act() calls this slow are not late but hung: the worker is killed and the run ends."""
    return max(0.5, 25 * budget_s)


def run_episode(submission, config: PublicConfig, scenario: Scenario = Scenario(), observer=None, *,
                debug: bool = False, budget_scale: float = 1.0, cpu=None, command_prefix=None,
                mount_root=None, stderr=None, isolation: str | None = None) -> dict:
    """Simulate one race.

    ``budget_scale`` multiplies config.act_budget_s/reset_budget_s, to emulate the
    official machine on a faster or slower computer (see ``drone-race bench``).
    Timing: act() is called every control_dt_s of simulated time and its action is applied
    one control step later (actuation delay). A call longer than the budget is dropped:
    the previous action is held for as many control steps as the call took, and the next
    observation comes after that.
    """
    validate_config(config)
    sub = submissions.resolve(submission)
    digest = sub.sha256()
    wind = WindField(scenario.wind_zones, scenario.turbulence_m_s, np.random.default_rng(scenario.seed))
    max_time = scenario.time_limit(config.laps)

    plant = RotorPyPlant(config)
    rules = RaceRules(config.track, config.vehicle_radius_m, config.laps)
    battery = Battery(config)
    records, events, act_times = [], [], []
    time_s, peak_load, peak_tilt, misses = 0.0, 0.0, 0.0, 0
    gates_passed, gates_missed, saturated = 0, 0, 0
    status, error, init_time, modules = "timeout", None, 0.0, []
    budget = config.act_budget_s * budget_scale
    started = time.perf_counter()
    held = Action(config.nominal_mass_kg * config.gravity_m_s2, (0.0, 0.0, 0.0))

    def record(action=None, act_s=None, late=False, requested=None):
        item = {"time_s": time_s, "position_m": plant.state["x"].tolist(),
                "velocity_m_s": plant.state["v"].tolist(), "quaternion_xyzw": plant.state["q"].tolist(),
                "body_rates_rad_s": plant.state["w"].tolist(), "next_gate": rules.next_gate, "lap": rules.lap,
                "gates_passed": gates_passed, "gates_missed": gates_missed,
                "battery_soc": battery.soc, "battery_voltage_V": battery.voltage,
                "thrust_limit_N": battery.thrust_limit,
                "wind_m_s": plant.state["wind"].tolist(), "action": asdict(action) if action else None,
                "thrust_requested_N": requested, "act_s": act_s, "late": late}
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
    # One-step actuation delay: the command computed from the observation at t_k is applied
    # from t_k + control_dt; until then the drone keeps flying the previous command.
    command, requested = held, held.thrust_N
    while time_s < max_time - 1e-12 and not done:
        try:
            limit = battery.thrust_limit          # the limit this observation reports
            reply = handle.act(observation(plant.state, time_s, rules, gates_passed, gates_missed, battery),
                               timeout=hard_act_timeout(budget))
        except ControllerError as exc:
            status, error = exc.status, str(exc)
            break
        act_times.append(reply["elapsed"])
        late = reply["elapsed"] > budget
        misses += late
        new_command, busy_steps = None, 1
        if late and enforced:
            # Simulated time keeps running while the controller computes: a late call keeps it
            # busy for this many control periods (in official-machine time), during which the
            # previous command is held. Its result is dropped.
            busy_steps = max(1, int(np.ceil(reply["elapsed"] / budget_scale / config.control_dt_s - 1e-9)))
        else:
            try:
                if reply["invalid"]:
                    raise ValueError(reply["invalid"])
                new_command = validate_action(action_from_wire(reply["action"]), config)
            except (ValueError, TypeError, OverflowError, IndexError) as exc:
                status, error = "invalid_action", str(exc)
                break
            # Clipped to the thrust limit reported in the observation it was computed from.
            new_requested = new_command.thrust_N
            new_command = clip_thrust(new_command, config, limit)
        for busy_step in range(busy_steps):
            if done or time_s >= max_time - 1e-12:
                break
            action = command
            saturated += requested != action.thrust_N       # control steps flown with a clipped thrust
            for _ in range(substeps):
                dt = min(config.physics_dt_s, max_time - time_s)
                if dt <= 1e-12:
                    break
                before = {key: value.copy() for key, value in plant.state.items()}
                try:
                    plant.step(action, dt, wind.step(plant.state["x"], plant.state["v"], dt))
                    if not all(np.all(np.isfinite(value)) for value in plant.state.values()):
                        raise FloatingPointError("Non-finite plant state")
                except Exception as exc:
                    plant.state = before
                    status, error, done = "simulation_error", str(exc), True
                    break
                step_events = rules.inspect(before["x"], plant.state["x"])
                terminal = next((e for e in step_events if e.kind in ("finished", "collision", "out_of_bounds")), None)
                fraction = terminal.fraction if terminal else 1.0
                elapsed = dt * fraction
                load = np.linalg.norm((plant.state["v"] - before["v"]) / dt + [0, 0, config.gravity_m_s2]) / config.gravity_m_s2
                peak_load = max(peak_load, float(load))
                qx, qy = plant.state["q"][0], plant.state["q"][1]
                peak_tilt = max(peak_tilt, float(np.degrees(np.arccos(np.clip(1 - 2 * (qx * qx + qy * qy), -1, 1)))))
                for event in step_events:
                    events.append({"kind": event.kind, "time_s": time_s + event.fraction * dt,
                                   "gate": event.gate, "lap": event.lap})
                    gates_passed += event.kind == "gate_passed"
                    gates_missed += event.kind == "gate_missed"
                if terminal:
                    # Terminal position/time use the same linear within-step event estimate.
                    plant.state = {key: before[key] + fraction * (value - before[key]) for key, value in plant.state.items()}
                    plant.state["q"] /= np.linalg.norm(plant.state["q"])
                    status, done = terminal.kind, True
                time_s += elapsed
                battery.step(action.thrust_N, elapsed)
                if battery.depleted and not done:
                    status, done = "battery_depleted", True
                    events.append({"kind": "battery_depleted", "time_s": time_s, "gate": rules.next_gate, "lap": rules.lap})
                if done:
                    break
            first = busy_step == 0
            record(action, reply["elapsed"] if first else None, bool(late) and first, requested)
        if new_command is not None:
            command, requested = new_command, new_requested
    if handle is not None:
        handle.close()
    if status == "timeout":
        events.append({"kind": "timeout", "time_s": time_s, "gate": rules.next_gate, "lap": rules.lap})
    return _log(config, scenario, sub, digest, debug, isolation, budget, enforced, wind.realization(), records, events,
                act_times, misses, status, error, time_s, rules, peak_load, peak_tilt, init_time, modules, started,
                battery, saturated)


def _log(config, scenario, sub, digest, debug, isolation, budget, enforced, realization, records, events,
         act_times, misses, status, error, time_s, rules, peak_load, peak_tilt, init_time, modules, started,
         battery, saturated):
    lap_ends = [e["time_s"] for e in events if e["kind"] == "lap_completed"]
    lap_times = list(np.diff([0.0, *lap_ends]))
    missed = sum(e["kind"] == "gate_missed" for e in events)
    infractions = sum(e["kind"] == "wrong_way" for e in events)
    penalty = scenario.gate_miss_penalty_s * missed + scenario.infraction_penalty_s * infractions
    failure = 0.0 if status == "finished" else scenario.failure_penalty_s
    times = np.asarray(act_times) if act_times else np.zeros(1)
    summary = {
        "status": status, "completed": status == "finished", "flight_time_s": time_s,
        "laps_total": config.laps, "laps_completed": len(lap_ends),
        "lap_times_s": [float(t) for t in lap_times], "fastest_lap_s": float(min(lap_times)) if lap_times else None,
        "gates_total": config.laps * len(config.track.gates), "gates_resolved": rules.progress,
        "gates_passed": sum(e["kind"] == "gate_passed" for e in events), "gates_missed": missed,
        "infractions": infractions, "penalty_s": penalty, "failure_penalty_s": failure,
        # DNF scores are diagnostics only and must never rank ahead of completed runs.
        "score_s": time_s + penalty + failure,
        "peak_load_g": peak_load, "peak_tilt_deg": peak_tilt,
        "battery_final_soc": max(battery.soc, 0.0),
        "battery_min_voltage_V": min(r["battery_voltage_V"] for r in records),
        "battery_energy_Wh": battery.energy_J / 3600,
        "thrust_saturated_steps": int(saturated),
        "controller_reset_s": init_time,
        "controller_mean_act_s": float(times.mean()), "controller_p99_act_s": float(np.percentile(times, 99)),
        "controller_max_act_s": float(times.max()),
        "act_budget_s": budget, "deadline_misses": int(misses), "timing_enforced": bool(enforced),
        "wall_time_s": time.perf_counter() - started, "error": error,
    }
    return {
        "schema_version": SCHEMA_VERSION, "config": {**asdict(config), "track": track_to_dict(config.track)},
        "scenario": asdict(scenario),
        "realization": realization,
        "provenance": {
            "submission_sha256": digest, "submission": sub.entry.name if sub.entry.name != "controller.py" else sub.root.name,
            "isolation": isolation or ("inline" if debug else "process"),
            "third_party_modules": modules, "python": platform.python_version(),
            "machine": {"platform": platform.platform(), "processor": platform.processor() or platform.machine(),
                        "cpus": os.cpu_count()},
            "packages": {name: importlib.metadata.version(name) for name in ("drone-race", "rotorpy", "numpy", "scipy")},
        },
        "summary": summary, "events": events, "samples": records,
    }


def save_log(log: dict, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(log, indent=2, allow_nan=False) + "\n")
