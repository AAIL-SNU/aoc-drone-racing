"""Race files: one JSON that fixes the track, the number of laps and the disturbance.

    {"name": "practice", "track": "tracks/eight.json", "laps": 5,
     "scenario": {"seed": 0, "turbulence_m_s": 0.6, "wind_zones": [{"name": ..., "lower": ..., ...}]}}

Wind zones and turbulence are described in drone_race.wind. An optional ``randomize``
section perturbs the gates and draws the wind zones from the seed (drone_race.randomize).

``track`` is relative to the race file. ``config/race.json`` is the default practice race;
the official race file (track, wind and seed) is kept private until the results are published.
"""

from dataclasses import fields
import json
from pathlib import Path

from .api import PublicConfig
from .env import Scenario
from .randomize import Randomization, randomize
from .track import load_track


def load_race(path, *, laps=None, seed=None, randomized=True) -> tuple[str, PublicConfig, Scenario]:
    """randomized=False ignores the ``randomize`` section: the base track and the file's own wind zones."""
    path = Path(path)
    data = json.loads(path.read_text())
    track = load_track(path.parent / data["track"])
    known = {f.name for f in fields(Scenario)}
    values = {key: value for key, value in data.get("scenario", {}).items() if key in known}
    if seed is not None:
        values["seed"] = seed
    scenario = Scenario(**values)
    if randomized and "randomize" in data:
        track, zones = randomize(track, scenario.wind_zones, Randomization(**data["randomize"]), scenario.seed)
        scenario = Scenario(**{**values, "wind_zones": zones})
    config = PublicConfig(track, laps=int(laps if laps is not None else data.get("laps", 1)))
    return data.get("name", path.stem), config, scenario
