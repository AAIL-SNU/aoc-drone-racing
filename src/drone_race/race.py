"""Race files: one JSON that fixes the track, the number of laps and the disturbance.

    {"name": "competition", "track": "../tracks/circuit_1.json", "laps": 5,
     "scenario": {"seed": 0, "turbulence_m_s": 0.6, "wind_zones": [{"name": ..., "lower": ..., ...}]}}

Wind zones and turbulence are described in drone_race.wind.

``track`` is relative to the race file. The official race file (with the seed chosen
by the instructor) is kept private until the results are published.
"""

from dataclasses import fields
import json
from pathlib import Path

from .api import PublicConfig
from .env import Scenario
from .track import load_track


def load_race(path, *, laps=None, seed=None) -> tuple[str, PublicConfig, Scenario]:
    path = Path(path)
    data = json.loads(path.read_text())
    track = load_track(path.parent / data["track"])
    config = PublicConfig(track, laps=int(laps if laps is not None else data.get("laps", 1)))
    known = {f.name for f in fields(Scenario)}
    values = {key: value for key, value in data.get("scenario", {}).items() if key in known}
    if seed is not None:
        values["seed"] = seed
    return data.get("name", path.stem), config, Scenario(**values)
