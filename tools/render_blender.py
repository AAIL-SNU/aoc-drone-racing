"""Render a saved race log to an MP4 with Blender.

Needs the ``bpy`` module, which has no cp312 wheel, so it runs in its own env:

    conda env create -f environment-render.yml
    env -u DISPLAY conda run -n aoc-render python tools/render_blender.py outputs/run.json

Unsetting DISPLAY matters: with an X display set, Blender takes a software GL
path and runs ten times slower than it does over EGL.

A ``blender`` binary works too, if one is around:

    blender -b -P tools/render_blender.py -- outputs/run.json --cam fpv

By default this lays three views out in one frame: chase on the left, with the
overhead and FPV feeds stacked down the right. Pass ``--cam`` to render a single
view full frame instead.

Everything in the scene comes from the log. Samples are logged at 50 Hz, so one
sample becomes one frame and the clip plays back at wall-clock speed whatever
``--stride`` is set to.
"""

import argparse
import json
from pathlib import Path
import sys

import bpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from quad_race.track import track_from_dict  # noqa: E402
from race import output  # noqa: E402
from race.cameras import add_chase_camera, add_fpv_camera, add_overhead_camera  # noqa: E402
from race.scene import add_lighting, build_course, mark_passed  # noqa: E402
from race.vehicle import DRONE_COLORS, animate, animate_props, build_drone, build_trail  # noqa: E402

DRONE_SCALE = 1.0


def parse_args():
    # Under `blender -P` our arguments sit after `--`; as a plain script they are
    # just argv. Blender also injects flags of its own, so never read argv blindly.
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    parser = argparse.ArgumentParser(prog="render_blender.py", description=__doc__)
    parser.add_argument("logs", nargs="+", help="Run logs; the first one supplies the course")
    parser.add_argument("-o", "--output", help="MP4 path (default: alongside the first log)")
    parser.add_argument("--cam", choices=("chase", "fpv", "overhead"), default=None,
                        help="Render this one view full frame instead of the three-pane default")
    parser.add_argument("--res", default="1920x1080")
    parser.add_argument("--stride", type=int, default=1,
                        help="Render every Nth sample and drop the frame rate to match. The clip "
                             "keeps real-time pacing and costs N times less; this is the one knob "
                             "that scales linearly")
    return parser.parse_args(argv)


def main():
    args = parse_args()
    logs = [json.loads(Path(path).read_text()) for path in args.logs]
    if args.stride < 1:
        raise SystemExit("--stride must be 1 or more")
    # Read the rate off the log rather than trusting a flag: one sample becomes
    # one frame, so the clip runs at wall-clock speed whatever the stride.
    times = logs[0]["samples"]
    args.source_hz = round(1.0 / (times[1]["time_s"] - times[0]["time_s"])) if len(times) > 1 else 50
    args.fps = args.source_hz / args.stride
    if args.stride > 1:
        for log in logs:
            log["samples"] = log["samples"][::args.stride]
    if args.output is None:
        args.output = str(Path(args.logs[0]).with_suffix(".mp4"))

    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene

    track = track_from_dict(logs[0]["config"]["track"])
    gate_materials, _ = build_course(track)
    add_lighting(track)

    leader = None
    for index, log in enumerate(logs):
        color = DRONE_COLORS[index % len(DRONE_COLORS)]
        drone, props = build_drone(f"drone_{index}", color, DRONE_SCALE)
        animate(drone, log["samples"])
        animate_props(props, log["samples"], args.fps)
        build_trail(f"trail_{index}", log["samples"], color)
        if leader is None:
            leader = drone

    for event in logs[0]["events"]:
        if event["kind"] == "gate_passed":
            mark_passed(gate_materials[event["gate"]],
                        round(event["time_s"] * args.fps) + 1)

    def camera_for(kind):
        if kind == "fpv":
            return add_fpv_camera(leader)
        if kind == "overhead":
            return add_overhead_camera(track, logs)
        return add_chase_camera(logs)

    frames = max(len(log["samples"]) for log in logs)
    summary = logs[0]["summary"]
    print(f"[render] {track.name}: {len(track.gates)} gates, {frames} frames @ {args.fps} fps, "
          f"status={summary['status']} score={summary['score_s']:.2f}s -> {args.output}")

    if args.cam:
        output.setup_render(scene, args, frames, camera_for(args.cam), args.res, args.output)
        bpy.ops.render.render(animation=True)
        return

    # One render per camera, then one ffmpeg pass to lay them out. Blender shoots
    # a single camera per pass, so three panes cost three passes - but the side
    # panes are small, and the total pixel count lands near a plain 1080p frame.
    width, height = (int(value) for value in args.res.split("x"))
    side = width // 3
    panes = {"chase": (width - side, height),
             "overhead": (side, height // 2),
             "fpv": (side, height - height // 2)}
    staging = Path(args.output).with_suffix("")
    staging.parent.mkdir(parents=True, exist_ok=True)
    parts = {}
    for kind, (pane_w, pane_h) in panes.items():
        parts[kind] = f"{staging}_{kind}.mp4"
        print(f"[render]   {kind} pane {pane_w}x{pane_h} -> {parts[kind]}")
        output.setup_render(scene, args, frames, camera_for(kind), f"{pane_w}x{pane_h}", parts[kind])
        bpy.ops.render.render(animation=True)
    output.compose_broadcast(parts, panes, args.output, args.fps)
    for path in parts.values():
        Path(path).unlink(missing_ok=True)


if __name__ == "__main__":
    main()
