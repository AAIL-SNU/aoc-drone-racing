import argparse
from pathlib import Path
import sys

from .api import PublicConfig
from .env import Scenario, run_episode, save_log
from .track import load_track, random_track, save_track


DEFAULT_RACE = "config/races/competition.json"


def _race_args(parser):
    parser.add_argument("--race", default=DEFAULT_RACE,
                        help=f"Race file: track + laps + disturbance (default {DEFAULT_RACE})")
    parser.add_argument("--track", default=None, help="Track file, instead of a race file")
    parser.add_argument("--laps", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--turbulence", type=float, default=None, help="Horizontal turbulence std [0,2] m/s")
    parser.add_argument("--calm", action="store_true", help="No wind zones and no turbulence")
    parser.add_argument("--max-time", type=float, default=None, help="Simulated time limit [s]")


def _race(args):
    from dataclasses import replace
    if args.track:
        name = Path(args.track).stem
        config, scenario = PublicConfig(load_track(args.track), laps=args.laps or 1), Scenario(seed=args.seed or 0)
    else:
        from .race import load_race
        from .submission import find_repo_file
        race = Path(args.race)
        if not race.exists() and args.race == DEFAULT_RACE and find_repo_file(DEFAULT_RACE):
            race = find_repo_file(DEFAULT_RACE)
        name, config, scenario = load_race(race, laps=args.laps, seed=args.seed)
    overrides = {key: value for key, value in (("turbulence_m_s", args.turbulence), ("max_time_s", args.max_time))
                 if value is not None}
    if args.calm:
        overrides.update(turbulence_m_s=0.0, wind_zones=())
    return name, config, replace(scenario, **overrides)


def _progress(laps):
    """Observer that prints a line whenever a lap starts, while the simulation runs."""
    state = {"lap": 0, "start": 0.0}

    def observe(item):
        if item["lap"] != state["lap"]:
            print(f"  lap {state['lap'] + 1:>2}/{laps}  {item['time_s'] - state['start']:6.2f} s", flush=True)
            state.update(lap=item["lap"], start=item["time_s"])
    return observe


def _print_summary(s, log_path):
    status = "FINISHED" if s["completed"] else f"DID NOT FINISH ({s['status']})"
    print(f"\n{status}" + (f": {s['error']}" if s["error"] else ""))
    if s["completed"]:
        print(f"  score          {s['score_s']:8.2f} s   = time {s['flight_time_s']:.2f} s + penalties {s['penalty_s']:.2f} s")
    else:
        print(f"  flown          {s['flight_time_s']:8.2f} s   ({s['gates_resolved']}/{s['gates_total']} gates)")
    print(f"  laps           {s['laps_completed']}/{s['laps_total']}"
          + (f"   fastest {s['fastest_lap_s']:.2f} s" if s["fastest_lap_s"] else ""))
    if s["lap_times_s"]:
        print("  lap times      " + "  ".join(f"{t:.2f}" for t in s["lap_times_s"]))
    print(f"  missed gates   {s['gates_missed']}   wrong-way {s['infractions']}")
    print(f"  act()          mean {1e3 * s['controller_mean_act_s']:.1f} ms   p99 {1e3 * s['controller_p99_act_s']:.1f} ms"
          f"   late {s['deadline_misses']} (budget {1e3 * s['act_budget_s']:.1f} ms"
          + ("" if s["timing_enforced"] else ", not enforced in --debug") + ")")
    print(f"  peak tilt      {s['peak_tilt_deg']:.0f} deg   peak load {s['peak_load_g']:.2f} g")
    print(f"  battery        {s['battery_final_soc']:.0%} left   min {s['battery_min_voltage_V']:.2f} V"
          f"   {s['battery_energy_Wh']:.2f} Wh   thrust-limited steps {s['thrust_saturated_steps']}")
    print(f"  log            {Path(log_path).resolve()}")


def main():
    parser = argparse.ArgumentParser(description="Drone racing competition: simulate, check, benchmark and score controllers")
    sub = parser.add_subparsers(dest="command", required=True)

    run = sub.add_parser("run", help="Simulate and self-grade one submission")
    run.add_argument("--controller", default="submission",
                     help="Controller to fly: a folder with controller.py (default submission/) or a .py file")
    _race_args(run)
    run.add_argument("--log", default=None, help="Log path (default outputs/<submission>_<race>.json)")
    run.add_argument("--headless", action="store_true", help="No window: print the result only")
    run.add_argument("--speed", type=float, default=1.0, help="Playback speed of the window (1 = real time)")
    run.add_argument("--debug", action="store_true",
                     help="Run the controller inside this process (breakpoints work; timing is not enforced)")
    run.add_argument("--budget-scale", type=float, default=None,
                     help="Multiply the act()/reset() budgets (default: the calibration saved by `bench`)")

    check = sub.add_parser("check", help="Check a submission before handing it in")
    check.add_argument("controller", nargs="?", default="submission",
                       help="Folder with controller.py (default submission/) or a .py file")
    check.add_argument("--race", default=DEFAULT_RACE)

    bench = sub.add_parser("bench", help="Estimate act() timing on the official machine")
    bench.add_argument("--controller", default="submission")
    bench.add_argument("--race", default=DEFAULT_RACE)
    bench.add_argument("--seconds", type=float, default=10.0, help="Simulated seconds of your controller to time")
    bench.add_argument("--write-reference", action="store_true", help="TAs: store this machine as the official one")

    ev = sub.add_parser("evaluate", help="Official scoring of every team folder in a directory")
    ev.add_argument("submissions", help="Directory with one sub-folder per team")
    ev.add_argument("--race", required=True)
    ev.add_argument("--seed", type=int, default=None, help="Overrides the race file's disturbance seed")
    ev.add_argument("--laps", type=int, default=None)
    ev.add_argument("--out", default="outputs/evaluation")
    ev.add_argument("--cpu", type=int, default=None, help="Pin each controller to this CPU core (Linux)")
    ev.add_argument("--docker", default=None, metavar="IMAGE", help="Run each controller in this Docker image")
    ev.add_argument("--teams", nargs="+", default=None, help="Only these team folders")

    play = sub.add_parser("replay", help="Visualize a saved log without simulation")
    play.add_argument("log")
    play.add_argument("--output", help="Save a PNG or GIF instead of opening a window")
    play.add_argument("--speed", type=float, default=1.0, help="Playback speed (1 = real time)")
    make = sub.add_parser("make-track", help="Generate a seeded random gate course")
    make.add_argument("output")
    make.add_argument("--gates", type=int, default=10)
    make.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    if args.command == "make-track":
        save_track(random_track(args.gates, args.seed), args.output)
        print(f"Track: {Path(args.output).resolve()}")
        return
    if args.command == "replay":
        if args.output:
            import matplotlib
            matplotlib.use("Agg")
        from .replay import replay
        try:
            summary = replay(args.log, output=args.output, show=not args.output, speed=args.speed)
        except KeyboardInterrupt:
            return
        try:
            _print_summary(summary, args.log)
        except KeyError:     # logs written before laps existed
            print(f"{summary['status']}  score {summary['score_s']:.2f} s")
        if args.output:
            print(f"  saved          {Path(args.output).resolve()}")
        return
    if args.command == "check":
        from .evaluate import check as run_check
        sys.exit(0 if run_check(args.controller, args.race if Path(args.race).exists() else None) else 1)
    if args.command == "bench":
        from .bench import bench as run_bench
        run_bench(args.controller, args.race, seconds=args.seconds, write_reference=args.write_reference)
        return
    if args.command == "evaluate":
        from .evaluate import evaluate
        rows = evaluate(args.submissions, args.race, args.out, seed=args.seed, laps=args.laps, cpu=args.cpu,
                        docker_image=args.docker, teams=args.teams)
        print(f"\n{'rank':>4}  {'team':<20} {'status':<18} {'score [s]':>10} {'laps':>6} {'fastest':>8}")
        for row in rows:
            score = f"{row['score_s']:.2f}" if row["score_s"] is not None else "-"
            fastest = f"{row['fastest_lap_s']:.2f}" if row["fastest_lap_s"] is not None else "-"
            print(f"{row['rank']:>4}  {row['team']:<20} {row['status']:<18} {score:>10} "
                  f"{row['laps_completed']:>2}/{row['laps_total']:<3} {fastest:>8}")
        print(f"\nLeaderboard: {(Path(args.out) / 'leaderboard.json').resolve()}")
        return

    name, config, scenario = _race(args)
    submission_name = Path(args.controller).resolve()
    submission_name = submission_name.stem if submission_name.suffix == ".py" else submission_name.name
    log_path = Path(args.log or f"outputs/{submission_name}_{name}.json")
    print(f"{submission_name} on {config.track.name}: {config.laps} lap(s), seed {scenario.seed}, "
          f"{len(scenario.wind_zones)} wind zone(s), turbulence {scenario.turbulence_m_s} m/s")
    budget_scale = args.budget_scale
    if args.debug:
        print("timing: not enforced (--debug)")
    elif budget_scale is None:
        from .bench import load_calibration
        calibration = load_calibration()
        if calibration:
            budget_scale = calibration["budget_scale"]
            print(f"timing: {1e3 * config.act_budget_s:.0f} ms on the scoring server = "
                  f"{1e3 * config.act_budget_s * budget_scale:.1f} ms on this computer (calibrated {calibration['measured']})")
        else:
            print(f"timing: {1e3 * config.act_budget_s:.0f} ms budget, NOT calibrated -- run `drone-race bench` once "
                  "so it matches the scoring server")
    print("simulating..." + ("" if args.headless else " (the window opens when the race is over)"), flush=True)
    log = run_episode(args.controller, config, scenario, observer=_progress(config.laps),
                      debug=args.debug, budget_scale=budget_scale or 1.0)
    save_log(log, log_path)
    laps = log["summary"]["lap_times_s"]
    if log["summary"]["completed"] and laps:      # the observer never sees a lap start after the last one
        print(f"  lap {len(laps):>2}/{config.laps}  {laps[-1]:6.2f} s")
    _print_summary(log["summary"], log_path)
    if not args.headless:
        from .replay import replay
        try:
            replay(log_path, show=True, speed=args.speed)
        except KeyboardInterrupt:
            pass

if __name__ == "__main__":
    main()
