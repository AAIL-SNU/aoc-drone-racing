"""The controller worker process: isolation, timing rules and failure reporting."""

from dataclasses import replace
import json
from pathlib import Path
import textwrap

import pytest

from drone_race.api import PublicConfig
from drone_race.env import Scenario, run_episode
from drone_race.evaluate import check, evaluate, rank
from drone_race.track import demo_track, save_track

ROOT = Path(__file__).resolve().parents[1]
CONFIG = PublicConfig(demo_track())
HOVER = """
from drone_race import Action
class Controller:
    def reset(self, config):
        self.thrust = config.nominal_mass_kg * config.gravity_m_s2
    def act(self, observation):
{body}
        return Action(self.thrust, (0.0, 0.0, 0.0))
"""


def controller(tmp_path, body="", name="team"):
    folder = tmp_path / name
    folder.mkdir()
    (folder / "controller.py").write_text(HOVER.format(body=textwrap.indent(textwrap.dedent(body), " " * 8) or ""))
    return folder


def test_worker_and_debug_runs_are_identical():
    scenario = Scenario(seed=3, turbulence_m_s=0.6, max_time_s=3.0)
    worker = run_episode(ROOT / "submission", CONFIG, scenario)
    inline = run_episode(ROOT / "submission", CONFIG, scenario, debug=True)
    strip = lambda log: [{k: v for k, v in s.items() if k not in ("act_s", "late")} for s in log["samples"]]
    assert strip(worker) == strip(inline)
    assert inline["provenance"]["isolation"] == "inline" and not inline["summary"]["timing_enforced"]


BUSY = "import time\nt = time.process_time()\nwhile time.process_time() - t < 0.03: pass"


def test_late_act_holds_previous_action_and_is_counted(tmp_path):
    slow = controller(tmp_path, BUSY)
    log = run_episode(slow, CONFIG, Scenario(max_time_s=0.2))
    # Each 30 ms call keeps the controller busy for two 20 ms control steps: 5 calls in 0.2 s.
    assert log["summary"]["deadline_misses"] == 5
    called = [s for s in log["samples"][1:] if s["act_s"] is not None]
    assert len(called) == 5 and all(s["late"] for s in called)
    assert len(log["samples"]) == 11 and log["samples"][-1]["time_s"] == pytest.approx(0.2)
    relaxed = run_episode(slow, CONFIG, Scenario(max_time_s=0.2), budget_scale=3.0)
    assert relaxed["summary"]["deadline_misses"] == 0


def test_sleeping_is_not_compute_time(tmp_path):
    sleepy = controller(tmp_path, "import time; time.sleep(0.03)")
    assert run_episode(sleepy, CONFIG, Scenario(max_time_s=0.1))["summary"]["deadline_misses"] == 0


def test_hung_act_ends_the_run(tmp_path):
    hung = controller(tmp_path, "import time; time.sleep(30)")
    log = run_episode(hung, CONFIG, Scenario(max_time_s=1.0))
    assert log["summary"]["status"] == "controller_timeout"
    assert log["summary"]["wall_time_s"] < 10


def test_exception_and_prints_are_reported_not_fatal_to_the_simulator(tmp_path):
    noisy = controller(tmp_path, "print('hello from act'); raise RuntimeError('boom')")
    log = run_episode(noisy, CONFIG, Scenario(max_time_s=0.2), stderr=open(tmp_path / "stderr.txt", "w"))
    assert log["summary"]["status"] == "controller_error"
    assert "RuntimeError: boom" in log["summary"]["error"]
    assert "hello from act" in (tmp_path / "stderr.txt").read_text()


def test_submission_can_import_own_modules_and_read_assets(tmp_path):
    team = controller(tmp_path, "assert self.helper.GAIN == float(open('assets/gain.txt').read())")
    (team / "helper.py").write_text("GAIN = 2.5\n")
    (team / "assets").mkdir()
    (team / "assets" / "gain.txt").write_text("2.5")
    source = (team / "controller.py").read_text().replace("    def reset(self, config):\n",
                                                          "    def reset(self, config):\n        import helper; self.helper = helper\n")
    (team / "controller.py").write_text(source)
    for debug in (False, True):
        assert run_episode(team, CONFIG, Scenario(max_time_s=0.1), debug=debug)["summary"]["status"] == "timeout"


def test_rank_puts_finishers_first_then_progress():
    base = dict(penalty_s=0, laps_total=2, lap_times_s=[], fastest_lap_s=None, gates_missed=0, infractions=0,
                deadline_misses=0, controller_p99_act_s=0.0, error=None)
    summaries = {
        "slow": {**base, "status": "finished", "completed": True, "score_s": 30.0, "flight_time_s": 30.0,
                 "laps_completed": 2, "gates_resolved": 8},
        "fast": {**base, "status": "finished", "completed": True, "score_s": 20.0, "flight_time_s": 20.0,
                 "laps_completed": 2, "gates_resolved": 8},
        "crash_late": {**base, "status": "collision", "completed": False, "score_s": 99.0, "flight_time_s": 9.0,
                       "laps_completed": 1, "gates_resolved": 6},
        "crash_early": {**base, "status": "collision", "completed": False, "score_s": 50.0, "flight_time_s": 2.0,
                        "laps_completed": 0, "gates_resolved": 1},
    }
    assert [r["team"] for r in rank(summaries)] == ["fast", "slow", "crash_late", "crash_early"]


def test_evaluate_writes_logs_and_leaderboard(tmp_path):
    teams = tmp_path / "teams"
    teams.mkdir()
    controller(teams, name="hover")
    controller(teams, "raise ValueError('bad')", name="broken")
    save_track(demo_track(), tmp_path / "track.json")
    (tmp_path / "race.json").write_text(json.dumps({"name": "unit", "track": "track.json", "laps": 1,
                                                    "scenario": {"max_time_s": 0.3}}))
    rows = evaluate(teams, tmp_path / "race.json", tmp_path / "out", seed=7, progress=lambda *_: None)
    assert [r["team"] for r in rows] == ["hover", "broken"]
    board = json.loads((tmp_path / "out" / "leaderboard.json").read_text())
    assert board["seed"] == 7 and board["results"][1]["status"] == "controller_error"
    assert (tmp_path / "out" / "logs" / "hover.json").is_file()


def test_check_accepts_the_baseline_and_rejects_a_broken_submission(tmp_path):
    assert check(ROOT / "submission", ROOT / "config/races/competition.json", seconds=0.5, progress=lambda *_: None)
    assert not check(controller(tmp_path, "raise ValueError('bad')"), seconds=0.2, progress=lambda *_: None)
