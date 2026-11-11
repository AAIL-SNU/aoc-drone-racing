"""Compare the baseline, MPC and MPPI example controllers on one track.

    python examples/benchmark.py                                  # ten-gate track, nominal
    python examples/benchmark.py --track tracks/four_gates.json
    python examples/benchmark.py --wind 0.5 --mass-error 0.05 --seed 17
    python examples/benchmark.py --wind 2 --mass-error 0.2 --seeds 1 2 3 4 5 --jobs 8 --controllers mpc mppi
    python examples/benchmark.py --log-dir outputs/benchmark      # also save logs for replay

With --jobs 1 (default) episodes run one after another, so the act() timing columns
are comparable. With more jobs, runs are parallel and timings are inflated.
"""

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

from quad_race.api import PublicConfig
from quad_race.env import Scenario, run_episode, save_log
from quad_race.track import load_track

ROOT = Path(__file__).resolve().parents[1]
CONTROLLERS = {
    "baseline": ROOT / "controller.py",
    "mpc": ROOT / "examples/mpc_controller.py",
    "mppi": ROOT / "examples/mppi_controller.py",
}
HEADER = (f"{'controller':<10} {'seed':>4} {'mass x':>6} {'status':<14} {'gates':>5} {'time [s]':>9} {'score [s]':>9} "
          f"{'peak load [g]':>13} {'mean act [ms]':>13} {'max act [ms]':>12}")


def run(job):
    name, track_path, scenario, log_dir = job
    log = run_episode(CONTROLLERS[name], PublicConfig(load_track(track_path)), scenario)
    if log_dir:
        save_log(log, Path(log_dir) / f"{name}_seed{scenario.seed}.json")
    s = log["summary"]
    row = (f"{name:<10} {scenario.seed:>4} {log['realization']['mass_scale']:>6.3f} {s['status']:<14} "
           f"{s['gates_passed']:>2}/{s['gates_total']:<2} {s['flight_time_s']:>9.2f} {s['score_s']:>9.2f} "
           f"{s['peak_load_g']:>13.2f} {1e3 * s['controller_mean_act_s']:>13.1f} {1e3 * s['controller_max_act_s']:>12.1f}")
    return name, s["completed"], s["score_s"], row


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--track", default=str(ROOT / "tracks/ten_gates.json"))
    parser.add_argument("--controllers", nargs="+", default=list(CONTROLLERS), choices=list(CONTROLLERS))
    parser.add_argument("--seed", type=int, default=0, help="Single disturbance seed")
    parser.add_argument("--seeds", type=int, nargs="+", default=None, help="Several seeds (overrides --seed)")
    parser.add_argument("--wind", type=float, default=0.0, help="Maximum smooth wind speed [0,2] m/s")
    parser.add_argument("--mass-error", type=float, default=0.0, help="Uniform relative mass-error bound [0,0.2]")
    parser.add_argument("--max-time", type=float, default=90.0)
    parser.add_argument("--jobs", type=int, default=1, help="Parallel episodes (inflates timing columns)")
    parser.add_argument("--log-dir", default=None)
    args = parser.parse_args()

    base = Scenario(wind_max_m_s=args.wind, mass_error_fraction=args.mass_error, max_time_s=args.max_time)
    seeds = args.seeds or [args.seed]
    jobs = [(name, args.track, replace(base, seed=seed), args.log_dir) for name in args.controllers for seed in seeds]
    print(f"track: {load_track(args.track).name}   wind <= {args.wind} m/s   mass error <= ±{args.mass_error:.0%}")
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
