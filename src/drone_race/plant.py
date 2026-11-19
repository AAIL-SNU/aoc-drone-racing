"""RotorPy physics adapter. Swap the plant backend here, not in student code."""

from copy import deepcopy
import numpy as np
from rotorpy.vehicles.hummingbird_params import quad_params
from rotorpy.vehicles.multirotor import Multirotor

from .api import Action, PublicConfig


def nominal_parameters() -> dict:
    params = deepcopy(quad_params)
    params["k_w"] = 25.0
    params["motor_noise_std"] = 0.0
    return params


class RotorPyPlant:
    """RotorPy's dynamics, integrated here with a fixed step.

    ``Multirotor.step`` calls ``scipy.integrate.solve_ivp(method='RK45')`` once
    per physics step. At a fixed 2 ms step the adaptive machinery has nothing to
    adapt: it takes one internal step and spends the rest of its time building an
    ``RK45`` object, validating inputs and assembling an ``OdeResult``. Measured,
    that was 1303 us a step, of which about 250 us was solve_ivp overhead and the
    rest eight derivative evaluations.

    Integrating here with classical RK4 uses four evaluations and no solver
    object: 520 us a step, 2.5x faster. Over 25,000 steps - fifty seconds of
    flight - the two paths differ by 7 micrometres, against gate clearances
    measured in centimetres.

    The derivative itself is still RotorPy's ``_s_dot_fn``, so the vehicle model
    is unchanged; only the integrator is ours.
    """

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
        # Our integrator reproduces the parts of Multirotor.step that apply at
        # these settings and no others. If a future RotorPy, or a change here,
        # turns on ground contact or motor noise, fall back rather than quietly
        # dropping them.
        self._own_integrator = (not getattr(self.vehicle, "_enable_ground", False)
                                and not np.any(self.vehicle.motor_noise))

    def step(self, action: Action, dt: float, wind) -> None:
        self.state["wind"] = np.asarray(wind, dtype=float)
        control = {"cmd_thrust": action.thrust_N,
                   "cmd_w": np.array(action.body_rates_rad_s)}
        if self._own_integrator:
            self.state = self._rk4(self.state, control, dt)
            return
        # RotorPy draws from the global NumPy RNG even with zero motor noise, so
        # a student's own random numbers must not be able to move the plant.
        rng_state = np.random.get_state()
        try:
            self.state = self.vehicle.step(self.state, control, dt)
        finally:
            np.random.set_state(rng_state)

    def _rk4(self, state: dict, control: dict, dt: float) -> dict:
        vehicle = self.vehicle
        commanded = np.clip(vehicle.get_cmd_motor_speeds(state, control),
                            vehicle.rotor_speed_min, vehicle.rotor_speed_max)
        s = Multirotor._pack_state(state)

        def derivative(y):
            return vehicle._s_dot_fn(0.0, y, commanded)

        k1 = derivative(s)
        k2 = derivative(s + (0.5 * dt) * k1)
        k3 = derivative(s + (0.5 * dt) * k2)
        k4 = derivative(s + dt * k3)
        s = s + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        stepped = Multirotor._unpack_state(s)
        stepped["q"] /= np.linalg.norm(stepped["q"])
        stepped["rotor_speeds"] = np.clip(stepped["rotor_speeds"],
                                          vehicle.rotor_speed_min, vehicle.rotor_speed_max)
        return stepped
