import argparse
import json
from pathlib import Path

from .api import PublicConfig
from .env import Scenario, run_episode, save_log
from .track import load_track, random_track, save_track


def main():
    parser = argparse.ArgumentParser(description="RotorPy controller-racing prototype")
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="Simulate and self-grade one controller")
    run.add_argument("--controller", default="controller.py")
    run.add_argument("--track", default=None)
    run.add_argument("--seed", type=int, default=0)
    run.add_argument("--wind", type=float, default=0.0, help="Maximum smooth wind speed [0,2] m/s")
    run.add_argument("--mass-error", type=float, default=0.0, help="Uniform relative mass-error bound [0,0.2]")
    run.add_argument("--max-time", type=float, default=45.0)
    run.add_argument("--log", default="outputs/run.json")
    run.add_argument("--render", action="store_true", help="Show a live Matplotlib view")
    run.add_argument("--render-to", default=None, help="Save a PNG using the streaming renderer")
    play = sub.add_parser("replay", help="Visualize a saved log without simulation")
    play.add_argument("log")
    play.add_argument("--output", help="PNG or GIF path")
    play.add_argument("--show", action="store_true")
    play.add_argument("--speed", type=float, default=2.0)
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
        if not args.show and not args.output:
            parser.error("replay requires --show or --output")
        if not args.show:
            import matplotlib
            matplotlib.use("Agg")
        from .replay import replay
        print(json.dumps(replay(args.log, output=args.output, show=args.show, speed=args.speed), indent=2))
        return
    config = PublicConfig(load_track(args.track))
    scenario = Scenario(seed=args.seed, wind_max_m_s=args.wind, mass_error_fraction=args.mass_error, max_time_s=args.max_time)
    view = None
    if args.render or args.render_to:
        if not args.render:
            import matplotlib
            matplotlib.use("Agg")
        from .replay import RaceView
        view = RaceView(config.track, live=args.render)
    try:
        log = run_episode(args.controller, config, scenario, observer=view.update if view else None)
        save_log(log, args.log)
        if args.render_to:
            view.save(args.render_to)
        print(json.dumps(log["summary"], indent=2))
        print(f"Log: {Path(args.log).resolve()}")
        if args.render:
            view.draw_latest()
            view.plt.ioff()
            view.plt.show()
    finally:
        if view:
            view.close()


if __name__ == "__main__":
    main()
