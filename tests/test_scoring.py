import pytest
from quad_race.api import Box, Gate, Track
from quad_race.rules import RaceRules
from quad_race.scoring import score_run


def rules():
    return RaceRules(Track('contacts', (-1, 0, 2), (Gate((0, 0, 2)), Gate((5, 0, 2))),
                           bounds=Box((-10, -10, 0), (20, 10, 10))), .2)


def test_persistent_contact_counts_once_and_reentry_counts_again():
    r = rules()
    assert [e.kind for e in r.inspect((-1, .9, 2), (-.1, .9, 2))] == ['gate_collision']
    assert r.inspect((-.1, .9, 2), (-.1, .9, 2)) == []
    assert r.inspect((-.1, .9, 2), (-1, .9, 2)) == []
    assert [e.kind for e in r.inspect((-1, .9, 2), (-.1, .9, 2))] == ['gate_collision']


def test_corner_hits_two_bars_but_is_one_event():
    events = rules().inspect((-1, .9, 2.9), (1, .9, 2.9))
    assert [e.kind for e in events] == ['gate_collision', 'gate_passed']


def test_gate_contact_does_not_end_race_before_next_gate():
    r = rules()
    events = r.inspect((-1, .9, 2), (6, .9, 2))
    assert [e.kind for e in events] == ['gate_collision', 'gate_passed', 'gate_collision', 'gate_passed', 'lap_completed', 'finished']
    contacts = [dict(kind=e.kind, time_s=e.fraction, gate=e.gate, lap=e.lap) for e in events]
    assert score_run('finished', 100, contacts)['score_s'] == 110


def test_miss_cannot_finish_a_lap():
    r = rules()
    events = r.inspect((-1, 2, 2), (6, 2, 2))
    assert [e.kind for e in events] == ['gate_missed']
    assert r.progress == 0 and not r.finished


def test_score_has_no_pd_thrust_or_tilt_threshold_and_deduplicates():
    e = dict(kind='gate_collision', event_id='c1', time_s=12, gate=0, lap=0)
    result = score_run('finished', 400, [e, e])
    assert result['score_s'] == 405 and result['gate_collisions'] == 1
    assert score_run('finished', 400, [])['score_s'] == 400
    assert score_run('timeout', 100, [e])['score_s'] is None
    assert score_run('finished', 100, [dict(kind='gate_missed')])['score_s'] is None


@pytest.mark.parametrize('t', [-1, float('nan'), float('inf')])
def test_bad_time_rejected(t):
    with pytest.raises(ValueError):
        score_run('finished', t, [])

def test_episode_contact_continues_and_terminal_time_matches_event(monkeypatch):
    from pathlib import Path
    import numpy as np
    from quad_race.api import PublicConfig
    from quad_race import env
    # Exercise the real episode/referee/scorer path with a deterministic short motion.
    # Physics accuracy itself is covered by the real RotorPy smoke run.
    class Plant:
        def __init__(self, config, mass_scale):
            self.state = {'x': np.array([-1., .9, 2.]), 'v': np.zeros(3),
                          'q': np.array([0., 0., 0., 1.]), 'w': np.zeros(3), 'wind': np.zeros(3)}
            self.points = iter([(-.1, .9, 2), (-.1, .9, 2), (1, .9, 2), (6, .9, 2)])
        def step(self, action, dt, wind):
            self.state['x'] = np.array(next(self.points), dtype=float)
    monkeypatch.setattr(env, 'RotorPyPlant', Plant)
    track = rules().track
    config = PublicConfig(track, control_dt_s=.02, physics_dt_s=.02)
    hover = Path(__file__).parent / 'controllers/hover.py'
    log = env.run_episode(hover, config, env.Scenario(max_time_s=1), debug=True)
    assert log['summary']['completed']
    assert log['summary']['gate_collisions'] == 2
    assert log['summary']['score_s'] == pytest.approx(log['summary']['flight_time_s'] + 10)
    assert log['summary']['flight_time_s'] == pytest.approx(.076)
    assert log['events'][-1]['time_s'] == pytest.approx(log['summary']['flight_time_s'])
    assert log['samples'][-1]['time_s'] == pytest.approx(log['summary']['flight_time_s'])
