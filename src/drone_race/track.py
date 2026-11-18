"""Track serialization and a small nominal four-gate course."""

from dataclasses import asdict
import json
from pathlib import Path

import numpy as np

from .api import Box, Gate, Track


def demo_track() -> Track:
    return Track(
        name="four-gates-v1",
        start=(0.0, 0.0, 1.5),
        gates=(
            Gate((3.0, 0.0, 1.5)),
            Gate((6.0, 1.0, 1.8)),
            Gate((9.0, -0.8, 1.5)),
            Gate((12.0, 0.0, 2.0)),
        ),
    )


def random_track(num_gates: int = 10, seed: int = 0, *, max_turn_rad: float = np.deg2rad(60),
                 spacing_m=(3.5, 5.5), height_m=(1.2, 2.6), clearance_m: float = 3.0,
                 max_attempts: int = 1000) -> Track:
    """Seeded course: each gate faces the heading it is approached from.

    The heading turns by at most ``max_turn_rad`` per leg, so consecutive gates stay
    flyable. Gate centers keep ``clearance_m`` from each other and from every
    earlier leg, which prevents legs from cutting through other gates' frames.
    """
    rng = np.random.default_rng(seed)
    start = np.array([0.0, 0.0, 1.5])
    for _ in range(max_attempts):
        centers, yaws, heading, previous = [], [], 0.0, start
        for index in range(num_gates):
            if index:
                heading += rng.uniform(-max_turn_rad, max_turn_rad)
            distance = rng.uniform(*spacing_m)
            center = np.array([*(previous[:2] + distance * np.array([np.cos(heading), np.sin(heading)])),
                               rng.uniform(*height_m)])
            if not _leg_is_clear(previous, center, centers, clearance_m):
                break
            centers.append(center)
            yaws.append(float(np.arctan2(np.sin(heading), np.cos(heading))))
            previous = center
        else:
            points = np.vstack([start, *centers])
            margin = 3.0
            lower = (*(points[:, :2].min(axis=0) - margin), 0.0)
            upper = (*(points[:, :2].max(axis=0) + margin), 6.0)
            return Track(
                name=f"random-{num_gates}-gates-seed{seed}",
                start=tuple(float(x) for x in start),
                gates=tuple(Gate(tuple(round(float(x), 3) for x in c), round(yaw, 4)) for c, yaw in zip(centers, yaws)),
                bounds=Box(tuple(round(float(x), 3) for x in lower), tuple(round(float(x), 3) for x in upper)),
            )
    raise RuntimeError(f"Could not generate a {num_gates}-gate track for seed {seed}")


def _leg_is_clear(start, end, centers, clearance: float) -> bool:
    """New gate and the leg leading to it stay away from all earlier gates."""
    if any(np.linalg.norm(end[:2] - c[:2]) < clearance for c in centers):
        return False
    a, d = start[:2], end[:2] - start[:2]
    for c in centers[:-1]:  # the leg legitimately starts at the latest gate
        u = np.clip((c[:2] - a) @ d / (d @ d), 0.0, 1.0)
        if np.linalg.norm(a + u * d - c[:2]) < clearance:
            return False
    return True


# JSON "type" tag -> obstacle dataclass. Register new obstacle types here.
OBSTACLE_TYPES = {"box": Box}


def _tuples(data: dict) -> dict:
    return {key: tuple(value) if isinstance(value, list) else value for key, value in data.items()}


def obstacle_to_dict(obstacle) -> dict:
    tag = next(tag for tag, cls in OBSTACLE_TYPES.items() if type(obstacle) is cls)
    return {"type": tag, **asdict(obstacle)}


def obstacle_from_dict(data: dict):
    fields = dict(data)
    tag = fields.pop("type", "box")  # untagged obstacles predate the tag and are boxes
    if tag not in OBSTACLE_TYPES:
        raise ValueError(f"Unknown obstacle type {tag!r}")
    return OBSTACLE_TYPES[tag](**_tuples(fields))


def track_to_dict(track: Track) -> dict:
    return {**asdict(track), "obstacles": [obstacle_to_dict(o) for o in track.obstacles]}


def track_from_dict(data: dict) -> Track:
    return Track(
        name=data["name"], start=tuple(data["start"]),
        gates=tuple(Gate(**_tuples(g)) for g in data["gates"]),
        obstacles=tuple(obstacle_from_dict(o) for o in data.get("obstacles", [])),
        bounds=Box(**_tuples(data["bounds"])),
    )


def load_track(path: str | Path | None) -> Track:
    return demo_track() if path is None else track_from_dict(json.loads(Path(path).read_text()))


def save_track(track: Track, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(track_to_dict(track), indent=2) + "\n")
