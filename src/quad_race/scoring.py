"""Public scoring policy shared by practice runs and official evaluations."""

import math

POLICY_VERSION = "aoc-gate-contact-v1"
GATE_COLLISION_PENALTY_S = 5.0


def score_run(status, flight_time_s, events):
    """Only finishers receive J. A missed gate invalidates even a claimed finish.

    Each gate_collision event marks entry into contact, not a physics sample.
    Event IDs make accidental repeated delivery idempotent.
    """
    if not math.isfinite(flight_time_s) or flight_time_s < 0:
        raise ValueError("Invalid flight time")
    contacts, seen = 0, set()
    for event in events:
        if event["kind"] != "gate_collision":
            continue
        key = event.get("event_id", (event["time_s"], event.get("gate"), event.get("lap")))
        if key not in seen:
            seen.add(key)
            contacts += 1
    missed = sum(e["kind"] == "gate_missed" for e in events)
    if missed:
        status = "gate_missed"
    completed = status == "finished"
    penalty = GATE_COLLISION_PENALTY_S * contacts
    return {"status": status, "completed": completed, "gate_collisions": contacts,
            "gates_missed": missed, "penalty_s": penalty,
            "score_s": flight_time_s + penalty if completed else None,
            "diagnostic_cost_s": flight_time_s + penalty}
