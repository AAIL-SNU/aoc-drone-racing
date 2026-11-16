"""JSON messages between the simulator (host) and a controller worker.

Plain JSON in both directions: the host never unpickles anything a submission sends.
Python's float repr round-trips exactly, so a worker run sees bit-identical numbers
to an in-process run.
"""

from dataclasses import asdict, fields

from .api import Action, Observation, PublicConfig
from .track import track_from_dict, track_to_dict


def config_to_dict(config: PublicConfig) -> dict:
    return {**asdict(config), "track": track_to_dict(config.track)}


def config_from_dict(data: dict) -> PublicConfig:
    known = {f.name for f in fields(PublicConfig)}
    values = {key: value for key, value in data.items() if key in known and key != "track"}
    return PublicConfig(track=track_from_dict(data["track"]), **values)


def observation_to_dict(observation: Observation) -> dict:
    return {
        "time_s": float(observation.time_s),
        "position_m": [float(x) for x in observation.position_m],
        "velocity_m_s": [float(x) for x in observation.velocity_m_s],
        "quaternion_xyzw": [float(x) for x in observation.quaternion_xyzw],
        "body_rates_rad_s": [float(x) for x in observation.body_rates_rad_s],
        "next_gate": int(observation.next_gate),
        "lap": int(observation.lap),
    }


def observation_from_dict(data: dict) -> Observation:
    return Observation(
        data["time_s"], tuple(data["position_m"]), tuple(data["velocity_m_s"]),
        tuple(data["quaternion_xyzw"]), tuple(data["body_rates_rad_s"]), data["next_gate"], data["lap"],
    )


def action_to_wire(action):
    """[thrust, p, q, r] for a well-formed Action; anything else becomes an error string."""
    if not isinstance(action, Action):
        return None, f"act() must return quad_race.Action, got {type(action).__name__}"
    try:
        rates = [float(x) for x in action.body_rates_rad_s]
        return [float(action.thrust_N), *rates], None
    except (TypeError, ValueError, OverflowError) as exc:
        return None, f"Action is not numeric: {exc}"


def action_from_wire(values) -> Action:
    return Action(values[0], tuple(values[1:]))
