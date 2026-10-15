"""Public, simulator-independent controller contract. All quantities are SI."""

from dataclasses import dataclass

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class Gate:
    center: Vec3
    yaw_rad: float = 0.0
    width_m: float = 2.0
    height_m: float = 2.0
    frame_m: float = 0.10


@dataclass(frozen=True)
class Box:
    lower: Vec3
    upper: Vec3


# Every obstacle type needs a collision rule (rules.obstacle_fraction), a renderer
# (replay.draw_obstacle) and a JSON tag (track.OBSTACLE_TYPES). Pylons and terrain
# are planned additions to this union.
Obstacle = Box


@dataclass(frozen=True)
class Track:
    name: str
    start: Vec3
    gates: tuple[Gate, ...]
    obstacles: tuple[Obstacle, ...] = ()
    bounds: Box = Box((-2.0, -5.0, 0.0), (18.0, 5.0, 6.0))


@dataclass(frozen=True)
class PublicConfig:
    track: Track
    control_dt_s: float = 0.02
    physics_dt_s: float = 0.002
    nominal_mass_kg: float = 0.5
    gravity_m_s2: float = 9.81
    thrust_min_N: float = 0.0
    thrust_max_N: float = 15.0
    body_rate_limit_rad_s: float = 3.0
    body_rate_gain_s_inv: float = 25.0
    vehicle_radius_m: float = 0.28


@dataclass(frozen=True)
class Observation:
    time_s: float
    position_m: Vec3
    velocity_m_s: Vec3
    quaternion_xyzw: tuple[float, float, float, float]  # body -> world
    body_rates_rad_s: Vec3
    next_gate: int


@dataclass(frozen=True)
class Action:
    thrust_N: float
    body_rates_rad_s: Vec3
