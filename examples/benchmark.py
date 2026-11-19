"""Compare the PD, MPC and MPPI example controllers on one race.

    python examples/benchmark.py                                  # practice race, 1 lap
    python examples/benchmark.py --laps 5 --controllers mpc mppi  # the full race
    python examples/benchmark.py --track config/tracks/circuit_1.json      # any track, no disturbance
    python examples/benchmark.py --seeds 1 2 3 4 5 --turbulence 1.0 --jobs 8 --controllers mpc mppi
    python examples/benchmark.py --log-dir outputs/benchmark      # also save logs for replay

With --jobs 1 (default) episodes run one after another, so the act() timing columns
are comparable. With more jobs, runs are parallel and timings are inflated.
"""

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

from drone_race.api import PublicConfig
from drone_race.env import Scenario, run_episode, save_log
from drone_race.race import load_race
from drone_race.track import load_track

ROOT = Path(__file__).resolve().parents[1]
CONTROLLERS = {
    "pd": ROOT / "examples/pd_controller.py",
    "mpc": ROOT / "examples/mpc_controller.py",
    "mppi": ROOT / "examples/mppi_controller.py",
}
HEADER = (f"{'controller':<10} {'seed':>4} {'status':<14} {'laps':>5} {'contacts':>8} {'time [s]':>9} "
          f"{'score [s]':>9} {'fastest':>8} {'peak tilt':>9} {'p99 act [ms]':>12} {'late':>5}")


def run(job):
    name, config, scenario, log_dir = job
    log = run_episode(CONTROLLERS[name], config, scenario)
    if log_dir:
        save_log(log, Path(log_dir) / f"{name}_seed{scenario.seed}.json")
    s = log["summary"]
    fastest = f"{s['fastest_lap_s']:.2f}" if s["fastest_lap_s"] else "-"
    row = (f"{name:<10} {scenario.seed:>4} {s['status']:<14} "
           f"{s['laps_completed']:>2}/{s['laps_total']:<2} {s['gate_collisions']:>8} {s['flight_time_s']:>9.2f} "
           f"{s['score_s']:>9.2f} {fastest:>8} {s['peak_tilt_deg']:>8.0f}° {1e3 * s['controller_p99_act_s']:>12.1f} "
           f"{s['deadline_misses']:>5}")
    return name, s["completed"], s["score_s"], row


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--race", default=str(ROOT / "config/races/competition.json"))
    parser.add_argument("--track", default=None, help="Track file instead of the race (no disturbance by default)")
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--controllers", nargs="+", default=list(CONTROLLERS), choices=list(CONTROLLERS))
    parser.add_argument("--seed", type=int, default=0, help="Single disturbance seed")
    parser.add_argument("--seeds", type=int, nargs="+", default=None, help="Several seeds (overrides --seed)")
    parser.add_argument("--turbulence", type=float, default=None, help="Horizontal turbulence std [0,2] m/s")
    parser.add_argument("--calm", action="store_true", help="No wind zones and no turbulence")
    parser.add_argument("--max-time", type=float, default=None)
    parser.add_argument("--jobs", type=int, default=1, help="Parallel episodes (inflates timing columns)")
    parser.add_argument("--log-dir", default=None)
    args = parser.parse_args()

    if args.track:
        config, base = PublicConfig(load_track(args.track), laps=args.laps), Scenario()
    else:
        _, config, base = load_race(args.race, laps=args.laps)
    overrides = {key: value for key, value in (("turbulence_m_s", args.turbulence), ("max_time_s", args.max_time))
                 if value is not None}
    if args.calm:
        overrides.update(turbulence_m_s=0.0, wind_zones=())
    base = replace(base, **overrides)
    seeds = args.seeds or [args.seed]
    jobs = [(name, config, replace(base, seed=seed), args.log_dir) for name in args.controllers for seed in seeds]
    print(f"track: {config.track.name}   laps: {config.laps}   wind zones: {len(base.wind_zones)}   "
          f"turbulence: {base.turbulence_m_s} m/s")
    print(HEADER)
    print("-" * len(HEADER))
    results = []
    if args.jobs > 1:
        with ProcessPoolExecutor(args.jobs) as pool:
            for result in pool.map(run, jobs):
                print(result[3], flush=True)
                results.append(result)
    else:
        for job in jobs:
            result = run(job)
            print(result[3], flush=True)
            results.append(result)
    if len(seeds) > 1:
        print()
        for name in args.controllers:
            rows = [r for r in results if r[0] == name]
            scores = [r[2] for r in rows]
            print(f"{name:<10} finished {sum(r[1] for r in rows)}/{len(rows)}   "
                  f"score mean {sum(scores) / len(scores):.2f}  worst {max(scores):.2f}")


if __name__ == "__main__":
    main()
