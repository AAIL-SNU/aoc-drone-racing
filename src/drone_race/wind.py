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


def zone_weight(zone: WindZone, position) -> float:
    """1 inside the box, smoothly down to 0 at ``ramp_m`` outside it."""
    p, lo, hi = np.asarray(position, dtype=float), np.array(zone.lower), np.array(zone.upper)
    rise = _smoothstep((p - lo + zone.ramp_m) / zone.ramp_m)
    fall = _smoothstep((hi + zone.ramp_m - p) / zone.ramp_m)
    return float(np.prod(rise * fall))


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

    def mean(self, position) -> np.ndarray:
        wind = np.zeros(3)
        for zone, vector in zip(self.zones, self.zone_wind):
            weight = zone_weight(zone, position)
            if weight > 0:
                wind += weight * vector
        return wind

    def step(self, position, velocity, dt: float) -> np.ndarray:
        """Wind at ``position`` for the next ``dt``; the noise is drawn even when sigma is 0."""
        noise = self.rng.standard_normal(3)
        mean = self.mean(position)
        airspeed = max(float(np.linalg.norm(np.asarray(velocity) - mean)), MIN_AIRSPEED_M_S)
        decay = np.exp(-dt * airspeed / self.length)
        self.turbulence = decay * self.turbulence + self.sigma * np.sqrt(1 - decay**2) * noise
        return mean + self.turbulence

    def realization(self) -> dict:
        return {"zone_wind_m_s": {zone.name: vector.tolist() for zone, vector in zip(self.zones, self.zone_wind)}}
