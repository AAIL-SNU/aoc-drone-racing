"""Download the BlenderKit assets the renderer can optionally use.

Runs in any Python 3; it only speaks HTTP and never imports ``bpy``. The free
and CC0 assets need no API key - the download endpoint only wants a throwaway
``scene_uuid`` - so this is a plain fetch into ``tools/assets/``.

    python tools/fetch_assets.py              # everything in ASSETS
    python tools/fetch_assets.py env          # just the HDRI

Assets are deliberately NOT committed: the drone is royalty-free rather than
CC0, which permits rendering with it but not redistributing the file itself.
"""

import argparse
import json
from pathlib import Path
import shutil
import urllib.request
import uuid

SEARCH = "https://www.blenderkit.com/api/v1/search/"
HEADERS = {"User-Agent": "aoc-drone-racing/1.0"}
TARGETS = Path(__file__).resolve().parent / "assets"

# Resolution variants keep these at a few MB instead of the 1.7 GB and 314 MB
# originals, which is the difference between a usable scene and a stalled one.
ASSETS = {
    "env": {
        "base_id": "1a83ff4b-fc1c-42b3-ac9c-c92b1e9ccce7",
        "asset_type": "hdr",
        "variant": "resolution_4K",
        "filename": "env.exr",
        "note": "Aarfontein Sunrise - CC0, Poly Haven",
    },
    "drone": {
        "base_id": "53f618e7-310a-4118-b218-c116efcca5d2",
        "asset_type": "model",
        "variant": "resolution_2K",
        "filename": "drone.blend",
        "note": "FPV Drone - royalty-free, Dovydas Alytas (do not redistribute)",
    },
}


def fetch_json(url):
    with urllib.request.urlopen(urllib.request.Request(url, headers=HEADERS), timeout=60) as response:
        return json.load(response)


def resolve(spec):
    """Asset id -> the signed URL of the wanted resolution variant."""
    query = f"query=asset_base_id:{spec['base_id']}+asset_type:{spec['asset_type']}"
    results = fetch_json(f"{SEARCH}?{query}")["results"]
    if not results:
        raise SystemExit(f"BlenderKit returned nothing for {spec['base_id']}")
    record = results[0]
    wanted = spec["variant"]
    available = [f["fileType"] for f in record["files"]]
    if wanted not in available:
        raise SystemExit(f"{record['name']}: no {wanted}; has {available}")
    entry = next(f for f in record["files"] if f["fileType"] == wanted)
    # The endpoint refuses without a scene_uuid, but does not check who we are.
    signed = fetch_json(f"{entry['downloadUrl']}?scene_uuid={uuid.uuid4()}")
    return record, signed["filePaths"][0]["url"]


def download(name, spec):
    destination = TARGETS / spec["filename"]
    if destination.exists():
        print(f"  {name}: already at {destination} ({destination.stat().st_size / 1e6:.1f} MB)")
        return
    record, url = resolve(spec)
    TARGETS.mkdir(parents=True, exist_ok=True)
    partial = destination.with_suffix(destination.suffix + ".part")
    with urllib.request.urlopen(urllib.request.Request(url, headers=HEADERS), timeout=600) as response:
        with partial.open("wb") as handle:
            shutil.copyfileobj(response, handle)
    partial.replace(destination)
    print(f"  {name}: {record['name']} [{spec['variant']}, {record['license']}] "
          f"-> {destination} ({destination.stat().st_size / 1e6:.1f} MB)")


def main():
    parser = argparse.ArgumentParser(prog="fetch_assets.py", description=__doc__)
    parser.add_argument("names", nargs="*", metavar="NAME",
                        help=f"Which assets to fetch (default: all of {', '.join(ASSETS)})")
    args = parser.parse_args()
    unknown = [n for n in args.names if n not in ASSETS]
    if unknown:
        parser.error(f"unknown asset {unknown}; choose from {', '.join(ASSETS)}")
    print(f"Fetching into {TARGETS}")
    for name in args.names or ASSETS:
        download(name, ASSETS[name])
    print("\n".join(f"  {name}: {spec['note']}" for name, spec in ASSETS.items()))


if __name__ == "__main__":
    main()
