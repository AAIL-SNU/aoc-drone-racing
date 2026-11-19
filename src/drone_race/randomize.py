"""Random race variations: perturbed gates on a base track, and random wind zones.

The official race is one of the shipped tracks with every gate moved a little, and three
wind zones placed at random on the course. Both are drawn from the race seed, so the same
race file and seed always give the same track and wind; the official seed is secret.

A race file turns this on with a ``randomize`` section (all keys optional):

    "randomize": {"gate_shift_m": 1.0, "gate_height_m": 0.5, "gate_yaw_deg": 15, "wind": true}

Gates
    Each gate center moves horizontally to a uniform random point within ``gate_shift_m``
    of its original position, vertically by a uniform amount within +-``gate_height_m``,
    and its yaw turns by a uniform angle within +-``gate_yaw_deg``. The start moves along
    with the last gate (the start/finish line). A draw is rejected, and drawn again, if
      * two gate centers end up closer than ``GATE_CLEARANCE_M``,
      * a gate opening comes within ``ARENA_MARGIN_M`` of the arena's walls, floor or ceiling, or
      * the smooth reference path does not thread every gate cleanly: a cubic Hermite curve
        from the start through the gate centers (two laps), crossing each gate along its
        normal, flown by a vehicle of radius ``vehicle_radius_m`` (drone_race.rules), must pass
        every gate in order without touching a frame or leaving the arena.

Wind (``"wind": true``)
    Replaces the race file's wind zones by exactly three boxes spanning the full arena
    height, each centered on a uniform random point of the lap (the closed path through
    the gate centers), at least ``ZONE_SEPARATION_M`` apart:

    * one downwash: air moving straight down, square side 3-5 m, 1.0-2.0 m/s;
    * two crosswinds: horizontal, uniform random heading, square side 6-9 m, 1.5-3.0 m/s.

    The speed of each zone is then drawn from its range by drone_race.wind, as for any zone.
"""

from dataclasses import dataclass, replace

import numpy as np

from .api import Gate, PublicConfig, Track
from .rules import RaceRules
from .wind import WindZone

GATE_CLEARANCE_M = 3.0       # minimum distance between two gate centers after the perturbation
ARENA_MARGIN_M = 0.3         # gate openings keep this distance from the arena floor, ceiling and walls
ZONE_SEPARATION_M = 8.0      # minimum horizontal distance between two wind zone centers
MAX_ATTEMPTS = 1000

DOWNWASH = dict(side_m=(3.0, 5.0), speed_m_s=(1.0, 2.0))
CROSSWIND = dict(side_m=(6.0, 9.0), speed_m_s=(1.5, 3.0))
RAMP_M = 1.5


@dataclass(frozen=True)
class Randomization:
    gate_shift_m: float = 0.0
    gate_height_m: float = 0.0
    gate_yaw_deg: float = 0.0
    wind: bool = False

    def __post_init__(self):
        values = (self.gate_shift_m, self.gate_height_m, self.gate_yaw_deg)
        if not all(np.isfinite(values)) or min(values) < 0 or self.gate_yaw_deg > 90:
            raise ValueError("randomize: gate_shift_m, gate_height_m >= 0 and gate_yaw_deg in [0, 90]")


def _gate_fits(gate: Gate, track: Track) -> bool:
    lo, hi = np.array(track.bounds.lower), np.array(track.bounds.upper)
    c, half_w, half_h = np.array(gate.center), gate.width_m / 2, gate.height_m / 2
    horizontal = np.all(c[:2] - half_w - ARENA_MARGIN_M >= lo[:2]) and np.all(c[:2] + half_w + ARENA_MARGIN_M <= hi[:2])
    return bool(horizontal and c[2] - half_h - ARENA_MARGIN_M >= lo[2] and c[2] + half_h + ARENA_MARGIN_M <= hi[2])


def reference_path(track: Track, laps: int = 2, samples_per_leg: int = 60) -> np.ndarray:
    """Cubic Hermite curve from the start through the gate centers, along each gate's normal."""
    normals = [np.array([np.cos(g.yaw_rad), np.sin(g.yaw_rad), 0.0]) for g in track.gates] * laps
    points = [np.asarray(track.start, dtype=float)] + [np.asarray(g.center, dtype=float) for g in track.gates] * laps
    first = points[1] - points[0]
    directions = [first / np.linalg.norm(first)] + normals
    u = np.linspace(0, 1, samples_per_leg, endpoint=False)[:, None]
    h00, h10, h01, h11 = 2 * u**3 - 3 * u**2 + 1, u**3 - 2 * u**2 + u, -2 * u**3 + 3 * u**2, u**3 - u**2
    legs = []
    for p0, p1, d0, d1 in zip(points[:-1], points[1:], directions[:-1], directions[1:]):
        length = np.linalg.norm(p1 - p0)
        legs.append(h00 * p0 + h10 * d0 * length + h01 * p1 + h11 * d1 * length)
    return np.vstack([*legs, points[-1][None]])


def threads_gates(track: Track, laps: int = 2) -> bool:
    """The reference path passes every gate in order, without frame contact or leaving the arena."""
    rules = RaceRules(track, PublicConfig(track).vehicle_radius_m, laps=laps)
    path = reference_path(track, laps)
    kinds = [event.kind for a, b in zip(path[:-1], path[1:]) for event in rules.inspect(a, b)]
    return (kinds.count("gate_passed") == laps * len(track.gates) and kinds[-1] == "finished"
            and not {"collision", "out_of_bounds", "gate_collision"} & set(kinds))


def perturb_track(track: Track, rng: np.random.Generator, settings: Randomization) -> Track:
    """``track`` with every gate moved within the limits of ``settings``."""
    if not (settings.gate_shift_m or settings.gate_height_m or settings.gate_yaw_deg):
        return track
    for _ in range(MAX_ATTEMPTS):
        radius = settings.gate_shift_m * np.sqrt(rng.uniform(size=len(track.gates)))
        angle = rng.uniform(0, 2 * np.pi, len(track.gates))
        dz = rng.uniform(-settings.gate_height_m, settings.gate_height_m, len(track.gates))
        dyaw = np.radians(rng.uniform(-settings.gate_yaw_deg, settings.gate_yaw_deg, len(track.gates)))
        gates = []
        for gate, r, a, z, y in zip(track.gates, radius, angle, dz, dyaw):
            center = np.array(gate.center) + (r * np.cos(a), r * np.sin(a), z)
            yaw = float(np.arctan2(np.sin(gate.yaw_rad + y), np.cos(gate.yaw_rad + y)))
            gates.append(replace(gate, center=tuple(round(float(x), 3) for x in center), yaw_rad=round(yaw, 4)))
        centers = np.array([g.center for g in gates])
        distances = np.linalg.norm(centers[:, None] - centers[None], axis=-1) + np.eye(len(gates)) * 1e9
        if distances.min() < GATE_CLEARANCE_M or not all(_gate_fits(g, track) for g in gates):
            continue
        # The start keeps its place relative to the last gate (the start/finish line).
        last_old, last_new = track.gates[-1], gates[-1]
        turn = last_new.yaw_rad - last_old.yaw_rad
        offset = np.subtract(track.start, last_old.center)
        c, s = np.cos(turn), np.sin(turn)
        offset = np.array([c * offset[0] - s * offset[1], s * offset[0] + c * offset[1], offset[2]])
        start = tuple(round(float(x), 3) for x in np.add(last_new.center, offset))
        perturbed = replace(track, gates=tuple(gates), start=start)
        if threads_gates(perturbed):
            return perturbed
    raise RuntimeError(f"Could not perturb {track.name} within the clearance limits")


def _lap_points(track: Track, rng: np.random.Generator, count: int) -> np.ndarray:
    """``count`` uniform random points (by arc length) on the closed path through the gate centers."""
    corners = np.array([g.center for g in track.gates] + [track.gates[0].center])
    lengths = np.linalg.norm(np.diff(corners, axis=0), axis=1)
    s = rng.uniform(0, lengths.sum(), count)
    leg = np.minimum(np.searchsorted(np.cumsum(lengths), s), len(lengths) - 1)
    fraction = (s - np.concatenate(([0.0], np.cumsum(lengths)[:-1]))[leg]) / lengths[leg]
    return corners[leg] + fraction[:, None] * (corners[leg + 1] - corners[leg])


def random_wind_zones(track: Track, rng: np.random.Generator) -> tuple[WindZone, ...]:
    """One downwash and two crosswinds at random places on the lap."""
    lo, hi = np.array(track.bounds.lower), np.array(track.bounds.upper)
    for _ in range(MAX_ATTEMPTS):
        points = _lap_points(track, rng, 3)
        gaps = np.linalg.norm(points[:, None, :2] - points[None, :, :2], axis=-1) + np.eye(3) * 1e9
        if gaps.min() >= ZONE_SEPARATION_M:
            break
    else:
        raise RuntimeError(f"Could not place three wind zones {ZONE_SEPARATION_M} m apart on {track.name}")
    zones = []
    for index, point in enumerate(points):
        kind = DOWNWASH if index == 0 else CROSSWIND
        half = rng.uniform(*kind["side_m"]) / 2
        if index == 0:
            name, direction = "downwash", (0.0, 0.0, -1.0)
        else:
            heading = rng.uniform(0, 2 * np.pi)
            name, direction = f"crosswind {index}", (round(float(np.cos(heading)), 4), round(float(np.sin(heading)), 4), 0.0)
        lower = np.maximum([point[0] - half, point[1] - half, lo[2]], lo)
        upper = np.minimum([point[0] + half, point[1] + half, hi[2]], hi)
        zones.append(WindZone(name, tuple(np.round(lower, 3)), tuple(np.round(upper, 3)), direction,
                              kind["speed_m_s"], 0.0, RAMP_M))
    return tuple(zones)


def randomize(track: Track, wind_zones, settings: Randomization, seed: int):
    """(track, wind_zones) of one random race. Each part has its own stream of the seed, so
    the gates, the wind layout and drone_race.wind's draws (speeds, turbulence) are independent."""
    perturbed = perturb_track(track, np.random.default_rng([seed, 1]), settings)
    if perturbed is not track:
        perturbed = replace(perturbed, name=f"{track.name}-seed{seed}")
    zones = random_wind_zones(perturbed, np.random.default_rng([seed, 2])) if settings.wind else tuple(wind_zones)
    return perturbed, zones
