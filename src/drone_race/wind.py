"""Wind: fixed wind zones on the course plus Dryden-style turbulence everywhere.

The wind is an air velocity [m/s, world frame]. It acts on the drone only through
RotorPy's aerodynamics (rotor drag, parasitic drag, translational lift), which use
the airspeed ``v - wind``. A given wind therefore pushes harder the faster the
rotors spin, and the turbulence a drone meets gets faster the faster it flies.

Wind zones (layout from the race file or drawn by drone_race.randomize, random strength)
    Each zone is a box on the course with a nominal wind direction. For one race the
    seed draws the zone's speed from ``speed_m_s`` and turns its direction about the
    vertical axis by up to ``heading_jitter_deg``. The zone wind is then constant in
    time and fades to zero over ``ramp_m`` outside the box. It is the same on every
    lap.

Turbulence (unpredictable)
    A first-order Dryden (Gauss-Markov) process per world axis, driven by fresh
    Gaussian noise at every physics step. Its standard deviation is ``sigma_m_s``
    horizontally and half of it vertically; its correlation time is the turbulence
    length scale divided by the airspeed (frozen turbulence, Taylor's hypothesis),
    with the low-altitude Dryden length scales of MIL-F-8785C at about 3 m height.

The random draws come from the race seed only, one fixed sequence per physics step,
so every controller faces the same zone winds and the same noise sequence.
"""

from dataclasses import dataclass, field
import math

import numpy as np

Vec3 = tuple[float, float, float]

LENGTH_SCALE_HORIZONTAL_M = 20.0   # Dryden L_u = L_v at ~3 m altitude
LENGTH_SCALE_VERTICAL_M = 3.0      # Dryden L_w = altitude
VERTICAL_SIGMA_RATIO = 0.5         # sigma_w / sigma_u at low altitude
MIN_AIRSPEED_M_S = 1.0             # keeps the correlation time finite while hovering


@dataclass(frozen=True)
class WindZone:
    name: str
    lower: Vec3
    upper: Vec3
    direction: Vec3                          # nominal direction the air moves to, normalised on use
    speed_m_s: tuple[float, float]           # range of the zone wind speed for one race
    heading_jitter_deg: float = 0.0          # random turn of the direction about the vertical axis
    ramp_m: float = 1.0                      # distance outside the box over which the wind fades out

    def __post_init__(self):
        for name in ("lower", "upper", "direction", "speed_m_s"):
            object.__setattr__(self, name, tuple(float(x) for x in getattr(self, name)))
        lo, hi = np.array(self.lower), np.array(self.upper)
        values = (*self.lower, *self.upper, *self.direction, *self.speed_m_s, self.heading_jitter_deg, self.ramp_m)
        if not np.all(np.isfinite(values)) or len(self.lower) != 3 or len(self.upper) != 3 or len(self.direction) != 3:
            raise ValueError(f"wind zone {self.name!r}: bounds and direction need three finite numbers each")
        if np.any(lo >= hi) or self.ramp_m <= 0 or np.linalg.norm(self.direction) == 0:
            raise ValueError(f"wind zone {self.name!r}: needs lower < upper, ramp_m > 0 and a nonzero direction")
        if len(self.speed_m_s) != 2 or not 0 <= self.speed_m_s[0] <= self.speed_m_s[1] <= 5.0:
            raise ValueError(f"wind zone {self.name!r}: speed_m_s must be a range within [0, 5] m/s")
        if not 0 <= self.heading_jitter_deg <= 180:
            raise ValueError(f"wind zone {self.name!r}: heading_jitter_deg must be in [0, 180]")


def _smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3 - 2 * t)


def _ramp(t: float) -> float:
    """``_smoothstep`` for one number. Called six times per wind zone per physics
    step, where a NumPy call on three floats costs more than the arithmetic."""
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    return t * t * (3.0 - 2.0 * t)


def _weight(lower, upper, ramp: float, x: float, y: float, z: float) -> float:
    """``zone_weight`` over plain floats, rejecting the common case first.

    A zone is a fixed box and the vehicle moves a few centimetres per step, so
    almost every call is for a position well outside one: test that before
    computing six smoothsteps.
    """
    if (x <= lower[0] - ramp or x >= upper[0] + ramp
            or y <= lower[1] - ramp or y >= upper[1] + ramp
            or z <= lower[2] - ramp or z >= upper[2] + ramp):
        return 0.0
    weight = 1.0
    for axis, value in enumerate((x, y, z)):
        weight *= (_ramp((value - lower[axis] + ramp) / ramp)
                   * _ramp((upper[axis] + ramp - value) / ramp))
        if weight == 0.0:
            return 0.0
    return weight


def zone_weight(zone: WindZone, position) -> float:
    """1 inside the box, smoothly down to 0 at ``ramp_m`` outside it."""
    return _weight(zone.lower, zone.upper, zone.ramp_m, *(float(v) for v in position))


@dataclass
class WindField:
    """One race's wind. ``step`` advances the turbulence by one physics step."""

    zones: tuple[WindZone, ...]
    sigma_m_s: float
    rng: np.random.Generator
    zone_wind: list = field(init=False)
    turbulence: np.ndarray = field(init=False)

    def __post_init__(self):
        self.zone_wind = []
        for zone in self.zones:
            speed = self.rng.uniform(*zone.speed_m_s)
            turn = np.radians(self.rng.uniform(-zone.heading_jitter_deg, zone.heading_jitter_deg))
            d = np.asarray(zone.direction) / np.linalg.norm(zone.direction)
            c, s = np.cos(turn), np.sin(turn)
            d = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]])
            self.zone_wind.append(speed * d)
        self.sigma = self.sigma_m_s * np.array([1.0, 1.0, VERTICAL_SIGMA_RATIO])
        self.length = np.array([LENGTH_SCALE_HORIZONTAL_M] * 2 + [LENGTH_SCALE_VERTICAL_M])
        self.turbulence = self.sigma * self.rng.standard_normal(3)      # start in the stationary distribution
        # The same numbers as plain floats. ``step`` runs once per physics step
        # on three components, which is below the size where NumPy pays off.
        self._sigma = tuple(float(x) for x in self.sigma)
        self._length = tuple(float(x) for x in self.length)
        self._turbulence = tuple(float(x) for x in self.turbulence)
        self._boxes = [(zone.lower, zone.upper, float(zone.ramp_m),
                        tuple(float(x) for x in vector))
                       for zone, vector in zip(self.zones, self.zone_wind)]

    def mean(self, position) -> np.ndarray:
        return np.array(self._mean(*(float(v) for v in position)))

    def _mean(self, x: float, y: float, z: float):
        mx = my = mz = 0.0
        for lower, upper, ramp, vector in self._boxes:
            weight = _weight(lower, upper, ramp, x, y, z)
            if weight > 0.0:
                mx += weight * vector[0]
                my += weight * vector[1]
                mz += weight * vector[2]
        return mx, my, mz

    def step(self, position, velocity, dt: float) -> np.ndarray:
        """Wind at ``position`` for the next ``dt``; the noise is drawn even when sigma is 0.

        The draw stays exactly one ``standard_normal(3)`` per step whatever the
        wind is, so two runs of the same seed meet the same noise sequence.
        """
        noise = self.rng.standard_normal(3)
        mx, my, mz = self._mean(float(position[0]), float(position[1]), float(position[2]))
        dx, dy, dz = float(velocity[0]) - mx, float(velocity[1]) - my, float(velocity[2]) - mz
        airspeed = max((dx * dx + dy * dy + dz * dz) ** 0.5, MIN_AIRSPEED_M_S)
        out = np.empty(3)
        turbulence = []
        for axis in range(3):
            decay = math.exp(-dt * airspeed / self._length[axis])
            value = (decay * self._turbulence[axis]
                     + self._sigma[axis] * math.sqrt(1.0 - decay * decay) * noise[axis])
            turbulence.append(value)
            out[axis] = value
        self._turbulence = tuple(turbulence)
        self.turbulence = np.array(self._turbulence)
        out[0] += mx
        out[1] += my
        out[2] += mz
        return out

    def realization(self) -> dict:
        return {"zone_wind_m_s": {zone.name: vector.tolist() for zone, vector in zip(self.zones, self.zone_wind)}}
