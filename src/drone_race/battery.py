"""Battery model: the simulator uses exactly these functions, so controllers can too.

All functions are pure and broadcast over NumPy arrays. With T the applied collective
thrust [N] (the action's thrust after clipping) and soc the state of charge in [0, 1]:

    electrical power   P     = battery_power_coeff * T^1.5                       [W]
    open-circuit volt  V_oc  = V_empty + (V_full - V_empty) * soc                 [V]
    terminal voltage   V     = V_oc - R * I,   with P = V * I
                       ->  I = (V_oc - sqrt(V_oc^2 - 4 R P)) / (2 R)              [A]
    discharge          d soc / dt = -I / capacity                                 [1/s]
    thrust limit       T_max = thrust_max_N * (V / V_full)^2                      [N]

T^1.5 is momentum theory (rotor power ~ thrust^1.5). The thrust limit follows from the
rotor speed limit scaling with the voltage the motors see, so it drops both as the
pack empties (lower V_oc) and right after drawing a lot of current (sag R * I).

Timing, as the simulator applies it:
  * Each act() observation reports ``battery_voltage_V``: the terminal voltage under the
    thrust applied during the previous control step (hover thrust before the first one),
    and ``thrust_limit_N = thrust_limit(battery_voltage_V, config)``.
  * The returned thrust is clipped to [thrust_min_N, this observation's thrust_limit_N]
    and flown one control step later, for one control step (held longer if the next act()
    is late). Clipping is not penalized.
  * soc is integrated with explicit Euler at physics_dt_s using the applied thrust.
    ``BatteryModel.step`` reproduces that exactly when called with the same dt.
  * soc reaching 0 ends the run as ``battery_depleted`` (did not finish).
"""

import numpy as np

from .api import PublicConfig


def open_circuit_voltage(soc, config: PublicConfig):
    soc = np.clip(soc, 0.0, 1.0)
    return config.battery_voltage_empty_V + (config.battery_voltage_full_V - config.battery_voltage_empty_V) * soc


def power(thrust, config: PublicConfig):
    return config.battery_power_coeff * np.maximum(thrust, 0.0) ** 1.5


def current(soc, thrust, config: PublicConfig):
    """Pack current [A] drawn at this soc and applied thrust."""
    v_oc = open_circuit_voltage(soc, config)
    r = config.battery_resistance_ohm
    p = power(thrust, config)
    if r == 0:
        return p / v_oc
    # The thrust limit keeps P well below V_oc^2 / (4R); the clamp only guards misuse.
    return (v_oc - np.sqrt(np.maximum(v_oc * v_oc - 4 * r * p, 0.0))) / (2 * r)


def terminal_voltage(soc, thrust, config: PublicConfig):
    return open_circuit_voltage(soc, config) - config.battery_resistance_ohm * current(soc, thrust, config)


def thrust_limit(voltage, config: PublicConfig):
    """Largest thrust the motors deliver at this terminal voltage [N]."""
    return config.thrust_max_N * (np.asarray(voltage) / config.battery_voltage_full_V) ** 2


def capacity_C(config: PublicConfig) -> float:
    return config.battery_capacity_mAh * 3.6


class BatteryModel:
    """Batched discrete battery step: the same update the simulator applies every physics step."""

    def __init__(self, config: PublicConfig):
        self.config = config
        self.capacity = capacity_C(config)

    def step(self, soc, thrust, dt):
        """soc after holding ``thrust`` for ``dt`` (not clamped: <= 0 means depleted)."""
        return soc - current(soc, thrust, self.config) * dt / self.capacity

    def voltage(self, soc, thrust):
        return terminal_voltage(soc, thrust, self.config)

    def thrust_limit(self, voltage):
        return thrust_limit(voltage, self.config)


class Battery:
    """Stateful pack used by the simulator."""

    def __init__(self, config: PublicConfig, soc: float = 1.0):
        self.model = BatteryModel(config)
        self.config = config
        self.soc = float(soc)
        self.load_N = config.nominal_mass_kg * config.gravity_m_s2   # hover draw before the first action
        self.energy_J = 0.0

    @property
    def voltage(self) -> float:
        return float(self.model.voltage(self.soc, self.load_N))

    @property
    def thrust_limit(self) -> float:
        return float(self.model.thrust_limit(self.voltage))

    @property
    def depleted(self) -> bool:
        return self.soc <= 0.0

    def step(self, thrust: float, dt: float) -> None:
        self.load_N = float(thrust)
        self.energy_J += float(power(thrust, self.config)) * dt
        self.soc = float(self.model.step(self.soc, thrust, dt))
