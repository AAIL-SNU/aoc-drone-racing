"""Bake downloaded models into the form the renderer wants, once.

The expensive part of using a downloaded airframe is not its triangles or its
textures - it is that the artist ships it as two dozen separate objects, and
every one of them is evaluated again for every motion-blur substep. Merging the
body into a single mesh takes it from 1.61 s/frame to 0.216, which is what the
hand-built airframe costs. This script does that merge once and saves the result.

    python tools/prepare_assets.py                  # uses tools/assets/drone.blend

Needs the render env, because it drives Blender:

    env -u DISPLAY conda run -n aoc-render python tools/prepare_assets.py
"""

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from race.vehicle import prepare_drone, prepared_path  # noqa: E402

ASSETS = Path(__file__).resolve().parent / "assets"


def main():
    parser = argparse.ArgumentParser(prog="prepare_assets.py", description=__doc__)
    parser.add_argument("model", nargs="?", default=str(ASSETS / "drone.blend"))
    args = parser.parse_args()
    source = Path(args.model)
    if not source.exists():
        raise SystemExit(f"{source} not found; run python tools/fetch_assets.py first")
    destination = prepared_path(source)
    meshes, rotors = prepare_drone(source, destination)
    print(f"[prepare] {source.name} -> {destination.name}: "
          f"{meshes} meshes ({rotors} rotors + body), "
          f"{destination.stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
