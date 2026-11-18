"""RotorPy physics adapter. Swap the plant backend here, not in student code."""

from copy import deepcopy
import numpy as np
from rotorpy.vehicles.hummingbird_params import quad_params
from rotorpy.vehicles.multirotor import Multirotor

from .api import Action, PublicConfig


def nominal_parameters() -> dict:
    params = deepcopy(quad_params)
    params["k_w"] = 25.0
    # RotorPy calls the global NumPy RNG even with zero motor noise. The adapter
    # saves/restores it so a student's own random draws cannot alter the plant.
    params["motor_noise_std"] = 0.0
    return params


class RotorPyPlant:
    def __init__(self, config: PublicConfig, mass_scale: float = 1.0):
        params = nominal_parameters()
        params["mass"] *= mass_scale
        self.vehicle = Multirotor(params, control_abstraction="cmd_ctbr", aero=True)
        self.mass_kg = params["mass"]
        speed = np.sqrt(self.mass_kg * config.gravity_m_s2 / (4 * params["k_eta"]))
        self.state = {
            "x": np.array(config.track.start, dtype=float), "v": np.zeros(3),
            "q": np.array([0., 0., 0., 1.]), "w": np.zeros(3),
            "wind": np.zeros(3), "rotor_speeds": np.full(4, speed),
        }

    def step(self, action: Action, dt: float, wind) -> None:
        self.state["wind"] = np.asarray(wind, dtype=float)
        rng_state = np.random.get_state()
        try:
            self.state = self.vehicle.step(
                self.state, {"cmd_thrust": action.thrust_N, "cmd_w": np.array(action.body_rates_rad_s)}, dt,
            )
        finally:
            np.random.set_state(rng_state)
