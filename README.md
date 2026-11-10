# Autonomous Drone Racing Competition for Applied Optimal Control (M2795.007200), Fall 2026

## Installation

Run from the project root. Conda is the default setup and pins Python 3.12.12 with the verified package versions.

```bash
conda env create -f environment.yml
conda activate aoc-drone-racing
```

Alternatively, with Python 3.12 and venv:

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-dev.txt
python -m pip install --no-deps -e .
```

## How to run

Implement `Controller.reset(config)` and `Controller.act(observation)` in `controller.py`.
The interface (observation, config and action) is documented at the top of that file.

```bash
# Simulate and score a controller (default: four-gate demo track)
quad-race run --controller controller.py --log outputs/nominal.json
quad-race run --controller controller.py --render            # live 3D view
quad-race run --render-to outputs/nominal.png                # headless image

# Ten-gate random track (the baseline controller needs ~51 s)
quad-race run --track tracks/ten_gates.json --max-time 90 --log outputs/ten_gates.json

# Disturbed scenario: smooth wind up to 0.5 m/s, mass error up to ±5%
quad-race run --wind 0.5 --mass-error 0.05 --seed 17 --log outputs/perturbed.json

# Generate a new random track
quad-race make-track tracks/random_seed7.json --gates 10 --seed 7

# Replay a saved log
quad-race replay outputs/nominal.json --show
quad-race replay outputs/nominal.json --output outputs/replay.gif
```

## Rendering

`quad-race replay` is enough to check a run. For something to put on a screen,
`tools/render_blender.py` turns the same log into an MP4 with Blender.

[**demo/four_gates.mp4**](demo/four_gates.mp4) is one, straight out of the
command below: the baseline controller around `tracks/four_gates.json`, 17 s of
flight in the default three-pane layout. It is the only render committed here -
everything else lands in the gitignored `outputs/`. The assets it was made with
are royalty-free, which covers the render even though it does not cover
redistributing the models themselves.

It needs the `bpy` module, which ships no cp312 wheel, so it lives in an env of
its own and never touches the simulator's:

```bash
conda env create -f environment-render.yml        # once
python tools/fetch_assets.py                      # once: ~40 MB of free assets
```

```bash
env -u DISPLAY conda run -n aoc-render \
  python tools/render_blender.py outputs/run.json --drone --env
```

**Always pass `env -u DISPLAY`.** With a display set, Blender renders through
GLX into whatever X server is around and falls back to software Mesa; without
one it picks up NVIDIA's EGL and the GPU. On this machine that is 0.76 s versus
11.28 s for a single frame.

By default it lays three views in one frame - chase on the left, overhead and
the vehicle's own FPV feed stacked down the right. `--cam chase|fpv|overhead`
renders one of them full frame instead.

| Flag | Default | Notes |
| --- | --- | --- |
| `--cam` | three-pane | One view full frame |
| `--res` | `1920x1080` | Cheap to raise: 2.25x the pixels of 720p costs ~15% |
| `--stride` | `2` (25 fps) | Every Nth sample, frame rate dropped to match. The one knob that scales linearly |
| `--drone` | built-in airframe | Use the downloaded FPV model |
| `--env` | code-lit dark room | Light from an HDRI, which also becomes the distance |
| `--arena` | off | Floor grid and perimeter pylons |

The floor is not a flag, it follows `--env`. Without one, a dark plate, because
a dark room needs a floor. With one, two things that meet in the middle.

An environment texture is sampled by ray direction alone, which puts it at
infinity: fly thirty metres and the terrain does not shift by a single pixel.
Rendering the same scene twice with the camera translated that far changed 0 of
360,000 pixels. So the panorama's lower hemisphere is projected onto a real
plane, each point sampled along the direction from the tripod it was shot from.
The ground becomes geometry at a known distance and streaks past the vehicle the
way the gates do - the same test now changes 53% of them.

That projection is only exact from the tripod, though. A panorama carries no
depth, so the plane is a guess that the ground was flat, and every bush and rock
that was not gets painted flat onto it; from far off the origin, or from high
up, they smear. So it is used only for the distance, where the error is small
and the detail is far away, and a tiled PBR material covers the near field where
the vehicle actually is. That half is ordinary geometry with an ordinary BSDF:
right from any angle, and it takes the vehicle's shadow.

The crossfade is keyed to distance from the course centre, not from the camera,
so the boundary is fixed in the world and nothing swims underneath a moving
shot. It runs from 1.4x to 4x the flight extent, which puts it past the gates
and well short of the horizon.

Two details earn their keep on that floor. The near material repeats every
seven metres, which nothing can see from a metre up at racing speed and the
overhead camera sees immediately, so its brightness drifts across a forty-metre
noise - the repeat survives, but no two patches of it read the same. And the
vehicle gets a soft dark patch beneath it. That is not a sun shadow: this HDRI
is a sunrise whose sun sits 0.1 degrees above the horizon, so the real shadow is
tens of metres long and horizontal, never underneath, and a second sun hung
somewhere more convenient would light the course from a direction the sky
visibly contradicts. What is underneath is the sky the vehicle blocks. The patch
widens and fades with altitude, and darkens the ground under it by about a
quarter.

`--arena` is the separate question of markings and pylons, and works on either
floor.

Gate frames are built from `quad_race.rules.frame_boxes`, the function the
collision checker tests against, so what the video shows is what was judged.
Samples are logged at 50 Hz and one sample becomes one frame, so a clip always
runs at wall-clock speed whatever `--stride` is.

### Downloaded assets

`tools/fetch_assets.py` pulls three free BlenderKit assets into `tools/assets/`,
which is gitignored - the drone and the ground material are royalty-free rather
than CC0, so they may be rendered with but not redistributed.

| Asset | License | Downloaded | Source |
| --- | --- | --- | --- |
| FPV Drone, Dovydas Alytas | royalty-free | 2K, 16.6 MB | [BlenderKit](https://www.blenderkit.com/asset-gallery-detail/79365807-cdb4-434a-9a7e-05b77beb6597/) |
| Dry Flattened Gravel Dirt Path | royalty-free | 2K, 8.5 MB | [BlenderKit](https://www.blenderkit.com/asset-gallery-detail/84deca31-2aef-4165-9b1e-cefb5c8af1ab/) |
| Aarfontein Sunrise, Poly Haven | CC0 | 4K, 22.9 MB | [BlenderKit](https://www.blenderkit.com/asset-gallery-detail/dc6aa810-3fc9-4870-bbc7-7bf0bcb9395f/), [Poly Haven](https://polyhaven.com/a/aarfontein_sunrise) |

The first render with `--drone` merges the model's two dozen separate objects
into one body plus four rotors and caches the result next to it. That merge is
not cosmetic: unmerged the model costs 1.61 s/frame, merged 0.216, which is what
the hand-built airframe costs. The cache rebuilds itself when the model changes
and is skipped if it is unreadable, so deleting `tools/assets/` only ever costs
a download.

### What a render costs

Default three-pane layout, 1080p at 25 fps, downloaded airframe and HDRI, on an
RTX 4090 over EGL. `tracks/` ships both of these courses:

| Track | Gates | Flight | Simulated in | Frames | Render | with `--arena` |
| --- | --- | --- | --- | --- | --- | --- |
| `four_gates` | 4 | 17.31 s | 23.88 s | 434 | 316 s | 329 s |
| `ten_gates` | 10 | 50.79 s | 42.77 s | 1271 | 982 s | 961 s |

That is about 0.75 s per output frame on either course. Ten gates are no more
expensive to draw than four; the longer course costs more only because flying it
takes three times as long, and `--stride` is the only knob that changes how many
frames there are.

Rendering one camera instead of three takes about half as long - 159 s and 512 s
for the same two courses. Not a third: the chase pane is two thirds of the frame
width on its own, so the two small panes add less than the first.

`--arena` buys no speed. It used to be up to 1.28x *faster* on a single camera,
because back then `--env` left no floor at all, and the arena's plate covered
the lower half of the frame with something cheaper than an environment lookup.
Both looks have that covered half now, so what is left is the cost of its own
geometry: 159 s against 169 s, 512 s against 529 s.

The floor is free, all of it. Three builds of it - bare projection, projection
plus a near-field material, then both plus the tiling tint and the contact patch
- rendered the same four videos in 323/315/960/974, 320/327/953/957 and
316/329/982/961 seconds. That spread is measurement noise. What this render
charges for is per-object scene evaluation, not pixels and not polygons, and the
floor is one more object however much is happening inside its shader.

## References

- RotorPy: https://github.com/spencerfolk/rotorpy (MIT License, Copyright 2023 RotorPy Authors).
- Spencer Folk, James Paulos, Vijay Kumar, "RotorPy: A Python-based Multirotor Simulator with Aerodynamics for Education and Research", ICRA 2023 UAV simulator workshop: https://arxiv.org/abs/2306.04485.
- RotorPy is used as an external dependency; no modified copy of its source is included. Preserve its copyright and license notice when redistributing.

Rendering assets, fetched on demand by `tools/fetch_assets.py` and never committed:

- "FPV Drone" by Dovydas Alytas, BlenderKit, royalty-free: https://www.blenderkit.com/asset-gallery-detail/79365807-cdb4-434a-9a7e-05b77beb6597/
  Royalty-free permits rendering with the model but not redistributing the model itself.
- "Dry Flattened Gravel Dirt Path", BlenderKit, royalty-free: https://www.blenderkit.com/asset-gallery-detail/84deca31-2aef-4165-9b1e-cefb5c8af1ab/
  Same terms as above: render with it, do not redistribute it.
- "Aarfontein Sunrise" by Poly Haven, CC0: https://polyhaven.com/a/aarfontein_sunrise
  Mirrored on BlenderKit: https://www.blenderkit.com/asset-gallery-detail/dc6aa810-3fc9-4870-bbc7-7bf0bcb9395f/
- BlenderKit: https://www.blenderkit.com/ - its free and CC0 assets download without an API key.
- Blender is used through the `bpy` PyPI module (GPL); it is a separate environment and no Blender source is included.
