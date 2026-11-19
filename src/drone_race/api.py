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
    thrust_max_N: float = 15.0     # thrust limit at the full-charge voltage; the live limit is lower (drone_race.battery)
    body_rate_limit_rad_s: float = 3.0
    body_rate_gain_s_inv: float = 25.0
    vehicle_radius_m: float = 0.28
    laps: int = 1                  # race length; a lap is one pass through every gate in order
    act_budget_s: float = 0.02     # compute budget per act() call on the official machine
    reset_budget_s: float = 60.0   # compute budget for reset() on the official machine
    # Battery (drone_race.battery): P = battery_power_coeff * T^1.5, V = V_oc(soc) - R I, T_max ~ V^2.
    battery_capacity_mAh: float = 200.0        # 5 fast laps leave ~40-60% (reference MPC); hovering lasts ~3.3 min
    battery_voltage_full_V: float = 12.6       # 3S LiPo, 4.2 V per cell
    battery_voltage_empty_V: float = 9.9       # 3.3 V per cell at soc = 0
    battery_resistance_ohm: float = 0.05       # pack internal resistance (voltage sag under load)
    battery_power_coeff: float = 3.75          # W / N^1.5: ideal rotor power / (figure of merit 0.6 * motor 0.8), ~40 W hover


@dataclass(frozen=True)
class Observation:
    time_s: float
    position_m: Vec3
    velocity_m_s: Vec3
    quaternion_xyzw: tuple[float, float, float, float]  # body -> world
    body_rates_rad_s: Vec3
    next_gate: int                 # index into config.track.gates, wraps to 0 on every new lap
    lap: int                       # current lap, 0-based
    gates_passed: int              # gates passed in order so far (all laps) = lap * n_gates + next_gate
    gate_collisions: int           # contacts with gate frames so far (+5 s each in the score)
    battery_soc: float             # state of charge, 1 = full, 0 = depleted (run ends)
    battery_voltage_V: float       # terminal voltage under the previous step's applied thrust
    thrust_limit_N: float          # the thrust is clipped to this observation's thrust_limit_N and flown one control step later


@dataclass(frozen=True)
class Action:
    thrust_N: float
    body_rates_rad_s: Vec3
