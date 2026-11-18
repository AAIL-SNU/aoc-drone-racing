import numpy as np
import pytest

from quad_race.api import Box, Gate, Track
from quad_race.rules import RaceRules, crossing_fraction, segment_box_fraction


def course(*gates, obstacles=()):
    return Track("test", (-1., 0., 2.), tuple(gates), obstacles,
                 Box((-10., -10., 0.), (20., 10., 10.)))


def test_forward_crossing_and_rotated_gate():
    gate = Gate((0., 0., 2.), yaw_rad=np.pi / 2)
    assert crossing_fraction((0, -1, 2), (0, 3, 2), gate, 0.2) == pytest.approx(0.25)
    rules = RaceRules(course(gate), 0.2)
    assert [e.kind for e in rules.inspect((0, -1, 2), (0, 3, 2))] == ["gate_passed", "lap_completed", "finished"]


def test_reverse_crossing_is_an_infraction_without_progress():
    rules = RaceRules(course(Gate((0., 0., 2.))), 0.2)
    assert [e.kind for e in rules.inspect((1, 0, 2), (-1, 0, 2))] == ["wrong_way"]
    assert rules.next_gate == 0


def test_passing_a_later_gate_resolves_the_skipped_one_as_missed():
    rules = RaceRules(course(Gate((0., 0., 2.)), Gate((3., 0., 2.))), 0.2)
    events = rules.inspect((2, 0, 2), (4, 0, 2))
    assert [(e.kind, e.gate) for e in events] == [("gate_missed", 0)]


def test_flying_beside_the_next_gate_is_a_miss_but_far_away_is_not():
    rules = RaceRules(course(Gate((0., 0., 2.)), Gate((3., 0., 2.))), 0.2)
    assert rules.inspect((-1, 5, 2), (1, 5, 2)) == []          # more than MISS_WINDOW_M beyond the frame
    assert [(e.kind, e.gate) for e in rules.inspect((-1, 2, 2), (1, 2, 2))] == [("gate_missed", 0)]
    assert rules.next_gate == 0


def test_laps_wrap_gate_index_and_count_laps():
    gates = tuple(Gate((3. * i, 0., 2.)) for i in range(4))
    rules = RaceRules(course(*gates), 0.2, laps=2)
    kinds = [e.kind for e in rules.inspect((-1, 0, 2), (10, 0, 2))]
    assert kinds == ["gate_passed"] * 4 + ["lap_completed"]
    assert (rules.next_gate, rules.lap, rules.finished) == (0, 1, False)
    # Flying back through the last gate (resolved recently) never counts again.
    assert rules.inspect((8, 0, 2), (10, 0, 2)) == []
    events = rules.inspect((-1, 0, 2), (10, 0, 2))
    assert [(e.kind, e.lap) for e in events][-3:] == [("gate_passed", 1), ("lap_completed", 1), ("finished", 1)]
    assert rules.progress == 8


def test_fast_motion_through_two_gates_is_ordered():
    rules = RaceRules(course(Gate((0., 0., 2.)), Gate((3., 0., 2.))), 0.2)
    events = rules.inspect((-1, 0, 2), (4, 0, 2))
    assert [e.kind for e in events] == ["gate_passed", "gate_passed", "lap_completed", "finished"]
    assert [e.fraction for e in events[:2]] == pytest.approx([0.2, 0.8])


def test_vehicle_volume_hits_frame_even_when_center_is_inside_opening():
    rules = RaceRules(course(Gate((0., 0., 2.))), 0.2)
    events = rules.inspect((-1, 0.9, 2), (1, 0.9, 2))
    assert [e.kind for e in events] == ["gate_collision", "gate_passed", "lap_completed", "finished"]
    assert rules.finished


def test_swept_collision_does_not_tunnel_through_thin_obstacle():
    box = Box((1., -0.1, 1.), (1.02, 0.1, 3.))
    assert segment_box_fraction((-2., 0., 2.), (4., 0., 2.), box, 0.2) == pytest.approx(2.8 / 6)
    rules = RaceRules(course(Gate((5., 0., 2.)), obstacles=(box,)), 0.2)
    assert rules.inspect((-2., 0., 2.), (4., 0., 2.))[0].kind == "collision"


def test_collision_before_gate_prevents_awarding_gate():
    box = Box((-0.5, -0.1, 1.), (-0.4, 0.1, 3.))
    rules = RaceRules(course(Gate((0., 0., 2.)), obstacles=(box,)), 0.2)
    assert [e.kind for e in rules.inspect((-1, 0, 2), (1, 0, 2))] == ["collision"]
    assert rules.next_gate == 0


def test_finish_before_later_obstacle_stops_race():
    box = Box((0.6, -0.1, 1.), (0.7, 0.1, 3.))
    rules = RaceRules(course(Gate((0., 0., 2.)), obstacles=(box,)), 0.2)
    assert [e.kind for e in rules.inspect((-1, 0, 2), (1, 0, 2))] == ["gate_passed", "lap_completed", "finished"]


def test_gate_miss_outside_frame_is_not_a_pass():
    rules = RaceRules(course(Gate((0., 0., 2.)), Gate((5., 0., 2.))), 0.2)
    assert [e.kind for e in rules.inspect((-1, 2, 2), (1, 2, 2))] == ["gate_missed"]
    assert rules.next_gate == 0


def test_out_of_bounds_checks_vehicle_radius():
    rules = RaceRules(course(Gate((0., 0., 2.))), 0.2)
    assert rules.inspect((-1, 0, 0.5), (-1, 0, 0.1))[0].kind == "out_of_bounds"
