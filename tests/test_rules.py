import numpy as np
import pytest

from drone_race.api import Box, Gate, Track
from drone_race.rules import RaceRules, crossing_fraction, segment_box_fraction, segment_box_interval


def course(*gates, obstacles=()):
    return Track("test", (-1., 0., 2.), tuple(gates), obstacles,
                 Box((-10., -10., 0.), (20., 10., 10.)))


def kinds(events):
    return [e.kind for e in events]


def test_forward_crossing_and_rotated_gate():
    gate = Gate((0., 0., 2.), yaw_rad=np.pi / 2)
    assert crossing_fraction((0, -1, 2), (0, 3, 2), gate, 0.2) == pytest.approx(0.25)
    rules = RaceRules(course(gate), 0.2)
    assert kinds(rules.inspect((0, -1, 2), (0, 3, 2))) == ["gate_passed", "lap_completed", "finished"]


def test_reverse_crossing_is_ignored():
    rules = RaceRules(course(Gate((0., 0., 2.))), 0.2)
    assert rules.inspect((1, 0, 2), (-1, 0, 2)) == []
    assert rules.next_gate == 0


def test_out_of_order_gate_does_not_count():
    """After gate 0, flying through gate 2 skips gate 1: gate 2 is not passed and gate 1 stays next."""
    gates = (Gate((0., 0., 2.)), Gate((3., 5., 2.)), Gate((6., 0., 2.)))
    rules = RaceRules(course(*gates), 0.2)
    assert kinds(rules.inspect((-1, 0, 2), (1, 0, 2))) == ["gate_passed"]
    assert rules.inspect((5, 0, 2), (7, 0, 2)) == []
    assert (rules.next_gate, rules.progress) == (1, 1)
    events = rules.inspect((2, 5, 2), (4, 5, 2))
    assert [(e.kind, e.gate) for e in events] == [("gate_passed", 1)]
    assert kinds(rules.inspect((5, 0, 2), (7, 0, 2))) == ["gate_passed", "lap_completed", "finished"]


def test_flying_beside_a_gate_is_neither_a_pass_nor_a_contact():
    rules = RaceRules(course(Gate((0., 0., 2.)), Gate((3., 0., 2.))), 0.2)
    assert rules.inspect((-1, 2, 2), (1, 2, 2)) == []
    assert rules.next_gate == 0


def test_laps_wrap_gate_index_and_count_laps():
    gates = tuple(Gate((3. * i, 0., 2.)) for i in range(4))
    rules = RaceRules(course(*gates), 0.2, laps=2)
    assert kinds(rules.inspect((-1, 0, 2), (10, 0, 2))) == ["gate_passed"] * 4 + ["lap_completed"]
    assert (rules.next_gate, rules.lap, rules.finished) == (0, 1, False)
    # Flying back through the last gate (not the next one) never counts again.
    assert rules.inspect((8, 0, 2), (10, 0, 2)) == []
    events = rules.inspect((-1, 0, 2), (10, 0, 2))
    assert [(e.kind, e.lap) for e in events][-3:] == [("gate_passed", 1), ("lap_completed", 1), ("finished", 1)]
    assert rules.progress == 8


def test_fast_motion_through_two_gates_is_ordered():
    rules = RaceRules(course(Gate((0., 0., 2.)), Gate((3., 0., 2.))), 0.2)
    events = rules.inspect((-1, 0, 2), (4, 0, 2))
    assert kinds(events) == ["gate_passed", "gate_passed", "lap_completed", "finished"]
    assert [e.fraction for e in events[:2]] == pytest.approx([0.2, 0.8])


def test_frame_contact_is_counted_and_the_pass_still_counts():
    """The center crosses inside the opening while the sphere overlaps the bar at y = 1."""
    rules = RaceRules(course(Gate((0., 0., 2.)), Gate((5., 0., 2.))), 0.2)
    events = rules.inspect((-1, 0.9, 2), (1, 0.9, 2))
    assert [(e.kind, e.gate) for e in events] == [("gate_collision", 0), ("gate_passed", 0)]
    assert rules.next_gate == 1


def test_center_through_the_bar_is_a_contact_but_not_a_pass():
    rules = RaceRules(course(Gate((0., 0., 2.)), Gate((5., 0., 2.))), 0.2)
    assert kinds(rules.inspect((-1, 1.05, 2), (1, 1.05, 2))) == ["gate_collision"]
    assert rules.next_gate == 0


def test_continuous_contact_over_many_steps_is_one_collision():
    rules = RaceRules(course(Gate((0., 0., 2.)), Gate((5., 0., 2.))), 0.2)
    path = np.linspace(-0.5, 0.5, 51)
    events = []
    for a, b in zip(path[:-1], path[1:]):          # brushing the bar at y = 1 the whole way
        events += rules.inspect((a, 0.9, 2), (b, 0.9, 2))
    assert kinds(events).count("gate_collision") == 1


def test_separating_and_touching_again_is_a_new_collision():
    rules = RaceRules(course(Gate((0., 0., 2.)), Gate((5., 0., 2.))), 0.2)
    assert kinds(rules.inspect((0, 0.7, 2), (0, 0.9, 2))) == ["gate_collision"]   # touch
    assert rules.inspect((0, 0.9, 2), (0, 0.95, 2)) == []                         # still touching
    assert rules.inspect((0, 0.95, 2), (0, 0.5, 2)) == []                         # separate
    assert kinds(rules.inspect((0, 0.5, 2), (0, 0.9, 2))) == ["gate_collision"]   # touch again


def test_touch_and_release_within_one_step_counts_each_touch():
    rules = RaceRules(course(Gate((0., 0., 2.)), Gate((5., 0., 2.))), 0.2)
    # The sphere sweeps across both side bars of the gate in one long sideways step.
    events = rules.inspect((0, -2, 2), (0, 2, 2))
    assert kinds(events) == ["gate_collision", "gate_collision"]


def test_touching_two_bars_of_one_gate_at_once_is_one_collision():
    rules = RaceRules(course(Gate((0., 0., 2.)), Gate((5., 0., 2.))), 0.2)
    # Corner of the opening: the side bar (y = 1) and the top bar (z = 3) at the same time.
    events = rules.inspect((-1, 0.9, 2.9), (1, 0.9, 2.9))
    assert kinds(events).count("gate_collision") == 1


def test_frame_contact_does_not_end_the_run():
    rules = RaceRules(course(Gate((0., 0., 2.)), Gate((5., 0., 2.))), 0.2)
    rules.inspect((-1, 0.9, 2), (1, 0.9, 2))
    assert kinds(rules.inspect((4, 0, 2), (6, 0, 2))) == ["gate_passed", "lap_completed", "finished"]


def test_segment_box_interval_enters_and_leaves():
    box = Box((1., -0.1, 1.), (1.02, 0.1, 3.))
    enter, leave = segment_box_interval((-2., 0., 2.), (4., 0., 2.), box, 0.2)
    assert (enter, leave) == pytest.approx((2.8 / 6, 3.22 / 6))
    assert segment_box_interval((-2., 5., 2.), (4., 5., 2.), box, 0.2) is None


def test_swept_collision_does_not_tunnel_through_thin_obstacle():
    box = Box((1., -0.1, 1.), (1.02, 0.1, 3.))
    assert segment_box_fraction((-2., 0., 2.), (4., 0., 2.), box, 0.2) == pytest.approx(2.8 / 6)
    rules = RaceRules(course(Gate((5., 0., 2.)), obstacles=(box,)), 0.2)
    assert rules.inspect((-2., 0., 2.), (4., 0., 2.))[0].kind == "collision"


def test_obstacle_collision_before_gate_prevents_awarding_gate():
    box = Box((-0.5, -0.1, 1.), (-0.4, 0.1, 3.))
    rules = RaceRules(course(Gate((0., 0., 2.)), obstacles=(box,)), 0.2)
    assert kinds(rules.inspect((-1, 0, 2), (1, 0, 2))) == ["collision"]
    assert rules.next_gate == 0


def test_finish_before_later_obstacle_stops_race():
    box = Box((0.6, -0.1, 1.), (0.7, 0.1, 3.))
    rules = RaceRules(course(Gate((0., 0., 2.)), obstacles=(box,)), 0.2)
    assert kinds(rules.inspect((-1, 0, 2), (1, 0, 2))) == ["gate_passed", "lap_completed", "finished"]


def test_out_of_bounds_checks_vehicle_radius():
    rules = RaceRules(course(Gate((0., 0., 2.))), 0.2)
    assert rules.inspect((-1, 0, 0.5), (-1, 0, 0.1))[0].kind == "out_of_bounds"
