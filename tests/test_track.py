from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest

from quad_race.api import Box
from quad_race.track import demo_track, load_track, random_track, save_track, track_from_dict


def test_track_roundtrip_creates_output_directory(tmp_path):
    path = tmp_path / "tracks" / "demo.json"
    save_track(demo_track(), path)
    assert load_track(path) == demo_track()


def test_random_track_is_seeded_and_spaced():
    track = random_track(10, seed=4)
    assert track == random_track(10, seed=4) != random_track(10, seed=5)
    assert len(track.gates) == 10 and len({g.yaw_rad for g in track.gates}) > 5
    centers = np.array([g.center for g in track.gates])
    gaps = np.linalg.norm(centers[:, None, :2] - centers[None, :, :2], axis=-1)
    assert gaps[~np.eye(10, dtype=bool)].min() >= 3.0
    lo, hi = np.array(track.bounds.lower), np.array(track.bounds.upper)
    assert np.all((centers > lo) & (centers < hi))


def test_ten_gate_track_file_matches_generator():
    assert load_track(Path(__file__).resolve().parents[1] / "config" / "tracks" / "ten_gates.json") == random_track(10, seed=4)


def test_obstacles_are_type_tagged_and_untagged_boxes_still_load(tmp_path):
    track = replace(demo_track(), obstacles=(Box((1., -1., 0.), (2., 1., 1.)),))
    path = tmp_path / "obstacle.json"
    save_track(track, path)
    data = json.loads(path.read_text())
    assert data["obstacles"][0]["type"] == "box"
    assert load_track(path) == track
    del data["obstacles"][0]["type"]
    assert track_from_dict(data) == track
    data["obstacles"][0]["type"] = "pylon"
    with pytest.raises(ValueError, match="Unknown obstacle type"):
        track_from_dict(data)


def test_competition_circuit_is_closed_and_flyable():
    from quad_race.planning import RacingLine
    from quad_race.rules import RaceRules
    track = load_track(Path(__file__).resolve().parents[1] / "config" / "tracks" / "circuit.json")
    assert len(track.gates) == 10
    # The start sits just behind the last gate, which is the start/finish line.
    assert np.linalg.norm(np.subtract(track.start, track.gates[-1].center)) < 3.0
    line = RacingLine(track, laps=2, v_max=16.0, a_lateral=30.0, a_accel=12.0, a_brake=14.0)
    rules = RaceRules(track, 0.28, laps=2)
    kinds = []
    for a, b in zip(line.position[:-1], line.position[1:]):
        kinds += [e.kind for e in rules.inspect(a, b)]
    assert kinds.count("gate_passed") == 20 and kinds[-1] == "finished"
    assert not {"collision", "out_of_bounds", "gate_missed", "wrong_way"} & set(kinds)
