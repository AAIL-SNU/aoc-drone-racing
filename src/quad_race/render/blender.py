"""Render a saved race log to an MP4 with Blender.

Needs the ``bpy`` module, which has no cp312 wheel, so it runs in its own env:

    conda env create -f environment-render.yml
    env -u DISPLAY conda run -n aoc-render drone-race render outputs/submission_practice.json

Unsetting DISPLAY matters: with an X display set, Blender takes a software GL
path and runs ten times slower than it does over EGL.

A ``blender`` binary works too, if one is around:

    blender -b -P src/quad_race/render/blender.py -- outputs/submission_practice.json --cam fpv

By default this lays three views out in one frame: chase on the left, with the
overhead and FPV feeds stacked down the right. Pass ``--cam`` to render a single
view full frame instead.

Everything in the scene comes from the log. Samples are logged at 50 Hz, so one
sample becomes one frame and the clip plays back at wall-clock speed whatever
``--stride`` is set to.
"""

import argparse
import json
import subprocess
from pathlib import Path
import sys

import bpy

# Runs as a plain script in the render env (no quad_race install there): put src/ on the path.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from quad_race.track import track_from_dict  # noqa: E402
from quad_race.render import output  # noqa: E402
from quad_race.render.cameras import add_chase_camera, add_fpv_camera, add_overhead_camera  # noqa: E402
from quad_race.render.scene import add_lighting, build_course, mark_passed  # noqa: E402
from quad_race.render.vehicle import (DRONE_COLORS, animate, animate_props, build_contact_shadow,  # noqa: E402
                          build_drone, build_trail, load_drone, prepared_path)

ASSETS = Path(__file__).resolve().parent / "assets"

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
    parser.add_argument("--res", default="1920x1080",
                        help="Nearly free to raise: 2.25x the pixels of 720p costs about 15%% more, "
                             "because the per-frame cost is scene work rather than shading")
    parser.add_argument("--drone", nargs="?", const=str(ASSETS / "drone.blend"), default=None,
                        help="Use a downloaded model instead of the built-in airframe; "
                             "bare flag takes the downloaded drone.blend")
    parser.add_argument("--env", nargs="?", const=str(ASSETS / "env.exr"), default=None,
                        help="Light the scene from an HDRI; bare flag takes the downloaded env.exr")
    parser.add_argument("--arena", action="store_true",
                        help="Add a floor grid and perimeter pylons. The floor under them "
                             "is separate: a dark plate without --env, the HDRI's own "
                             "terrain projected onto a plane with it")
    parser.add_argument("--stride", type=int, default=2,
                        help="Render every Nth sample and drop the frame rate to match. The clip "
                             "keeps real-time pacing and costs N times less; this is the one knob "
                             "that scales linearly. 2 gives 25 fps, 4 gives 12.5")
    return parser.parse_args(argv)


def ensure_prepared(model):
    """Build the merged cache for a model if it is missing or out of date.

    Preparing needs a scene of its own, so it runs as a subprocess rather than
    inside this render. Without the cache the renderer still works - it merges at
    load time instead - but the merge is what takes the model from 1.6 s/frame to
    0.21, so it is worth never leaving to chance.
    """
    cache = prepared_path(model)
    if cache.exists() and cache.stat().st_mtime >= model.stat().st_mtime:
        return
    script = Path(__file__).resolve().parent / "prepare_assets.py"
    print(f"[render] preparing {model.name} -> {cache.name} (one time)")
    result = subprocess.run([sys.executable, str(script), str(model)],
                            capture_output=True, text=True)
    if result.returncode or not cache.exists():
        print(f"[render] preparation failed, merging at load time instead\n"
              f"{result.stderr.strip()[:400]}")


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

    for flag, value in (("--drone", args.drone), ("--env", args.env)):
        if value and not Path(value).exists():
            raise SystemExit(f"{flag}: {value} not found; run drone-race render --fetch-assets")
    if args.drone:
        ensure_prepared(Path(args.drone))

    track = track_from_dict(logs[0]["config"]["track"])
    # An HDRI supplies its own terrain, so the plate would only bury it.
    gate_materials = build_course(track, arena=args.arena, ground=not args.env)
    ground_material = ASSETS / "ground.blend"
    add_lighting(track, args.env,
                 ground_material=ground_material if ground_material.exists() else None)

    leader = None
    for index, log in enumerate(logs):
        color = DRONE_COLORS[index % len(DRONE_COLORS)]
        if args.drone:
            drone, props = load_drone(args.drone, f"drone_{index}", DRONE_SCALE)
        else:
            drone, props = build_drone(f"drone_{index}", color, DRONE_SCALE)
        animate(drone, log["samples"])
        animate_props(props, log["samples"], args.fps)
        build_trail(f"trail_{index}", log["samples"], color)
        build_contact_shadow(f"contact_{index}", log["samples"])
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
