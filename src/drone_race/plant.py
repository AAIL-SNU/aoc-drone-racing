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
                                and not np.any(self.vehicle.motor_noise)
                                and self.vehicle.aero)
        self._constants()

    def _constants(self):
        """Everything in the derivative that does not change while flying.

        Read off the vehicle rather than restated, so the model stays RotorPy's,
        and kept as plain floats: at three and four elements a NumPy call costs
        more in dispatch than the arithmetic it performs. Measured, the same
        derivative written over scalars runs 4.8x faster than RotorPy's and
        agrees with it to 2e-16.
        """
        v = self.vehicle
        self.mass = float(v.mass)
        self.weight_z = float(v.weight[2])
        self.tau_m = float(v.tau_m)
        self.k_eta, self.k_m = float(v.k_eta), float(v.k_m)
        self.k_h, self.k_flap = float(v.k_h), float(v.k_flap)
        self.k_d, self.k_z = float(v.rotor_drag_matrix[0, 0]), float(v.rotor_drag_matrix[2, 2])
        self.drag = tuple(float(v.drag_matrix[i, i]) for i in range(3))
        self.geometry = tuple(tuple(float(x) for x in arm) for arm in v.rotor_geometry)
        self.rotor_dir = tuple(float(d) for d in np.ravel(v.rotor_dir))
        self.inertia = tuple(tuple(float(x) for x in row) for row in v.inertia)
        self.inv_inertia = tuple(tuple(float(x) for x in row) for row in v.inv_inertia)
        self.rotors = int(v.num_rotors)

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

        k1 = self._derivative(s, commanded)
        k2 = self._derivative(s + (0.5 * dt) * k1, commanded)
        k3 = self._derivative(s + (0.5 * dt) * k2, commanded)
        k4 = self._derivative(s + dt * k3, commanded)
        s = s + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        stepped = Multirotor._unpack_state(s)
        stepped["q"] /= np.linalg.norm(stepped["q"])
        stepped["rotor_speeds"] = np.clip(stepped["rotor_speeds"],
                                          vehicle.rotor_speed_min, vehicle.rotor_speed_max)
        return stepped

    def _derivative(self, s, commanded):
        """RotorPy's ``_s_dot_fn``, over scalars.

        Same equations and the same parameters; what is gone is NumPy. The
        arrays here hold three or four numbers, and at that size every call
        spends more time dispatching than computing - RotorPy rebuilds a 4x3x3
        of airframe geometry and a SciPy ``Rotation`` object five hundred times
        a second to reach nine multiplies.
        """
        vx, vy, vz = s[3], s[4], s[5]
        qx, qy, qz, qw = s[6], s[7], s[8], s[9]
        rate_x, rate_y, rate_z = s[10], s[11], s[12]
        wx, wy, wz = s[13], s[14], s[15]

        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        xu, yu, zu = qx * qw, qy * qw, qz * qw
        r00, r01, r02 = 1 - 2 * (yy + zz), 2 * (xy - zu), 2 * (xz + yu)
        r10, r11, r12 = 2 * (xy + zu), 1 - 2 * (xx + zz), 2 * (yz - xu)
        r20, r21, r22 = 2 * (xz - yu), 2 * (yz + xu), 1 - 2 * (xx + yy)

        # Airspeed in the body frame: R^T (v - wind).
        dx, dy, dz = vx - wx, vy - wy, vz - wz
        air_x = r00 * dx + r10 * dy + r20 * dz
        air_y = r01 * dx + r11 * dy + r21 * dz
        air_z = r02 * dx + r12 * dy + r22 * dz
        speed = (air_x * air_x + air_y * air_y + air_z * air_z) ** 0.5

        force_x = force_y = force_z = 0.0
        moment_x = moment_y = moment_z = 0.0
        for index in range(self.rotors):
            arm_x, arm_y, arm_z = self.geometry[index]
            omega = s[16 + index]
            # Airspeed seen by this hub: the body airspeed plus w x arm.
            local_x = air_x + rate_y * arm_z - rate_z * arm_y
            local_y = air_y + rate_z * arm_x - rate_x * arm_z
            local_z = air_z + rate_x * arm_y - rate_y * arm_x
            # Thrust along body z, with translational lift, minus hub drag.
            hub_x, hub_y = -omega * self.k_d * local_x, -omega * self.k_d * local_y
            hub_z = (self.k_eta * omega * omega
                     + self.k_h * (local_x * local_x + local_y * local_y)
                     - omega * self.k_z * local_z)
            force_x += hub_x
            force_y += hub_y
            force_z += hub_z
            # arm x force, plus flapping about the hub and the rotor's yaw torque.
            moment_x += arm_y * hub_z - arm_z * hub_y - self.k_flap * omega * local_y
            moment_y += arm_z * hub_x - arm_x * hub_z + self.k_flap * omega * local_x
            moment_z += arm_x * hub_y - arm_y * hub_x + self.rotor_dir[index] * self.k_m * omega * omega
        # Parasitic drag at the centre of mass.
        force_x -= speed * self.drag[0] * air_x
        force_y -= speed * self.drag[1] * air_y
        force_z -= speed * self.drag[2] * air_z

        inertia, inv_inertia = self.inertia, self.inv_inertia
        spin_x = inertia[0][0] * rate_x + inertia[0][1] * rate_y + inertia[0][2] * rate_z
        spin_y = inertia[1][0] * rate_x + inertia[1][1] * rate_y + inertia[1][2] * rate_z
        spin_z = inertia[2][0] * rate_x + inertia[2][1] * rate_y + inertia[2][2] * rate_z
        net_x = moment_x - (rate_y * spin_z - rate_z * spin_y)
        net_y = moment_y - (rate_z * spin_x - rate_x * spin_z)
        net_z = moment_z - (rate_x * spin_y - rate_y * spin_x)

        s_dot = np.empty_like(s)
        s_dot[0], s_dot[1], s_dot[2] = vx, vy, vz
        s_dot[3] = (r00 * force_x + r01 * force_y + r02 * force_z) / self.mass
        s_dot[4] = (r10 * force_x + r11 * force_y + r12 * force_z) / self.mass
        s_dot[5] = (r20 * force_x + r21 * force_y + r22 * force_z + self.weight_z) / self.mass
        s_dot[6] = 0.5 * (qw * rate_x - qz * rate_y + qy * rate_z)
        s_dot[7] = 0.5 * (qz * rate_x + qw * rate_y - qx * rate_z)
        s_dot[8] = 0.5 * (-qy * rate_x + qx * rate_y + qw * rate_z)
        s_dot[9] = 0.5 * (-qx * rate_x - qy * rate_y - qz * rate_z)
        s_dot[10] = inv_inertia[0][0] * net_x + inv_inertia[0][1] * net_y + inv_inertia[0][2] * net_z
        s_dot[11] = inv_inertia[1][0] * net_x + inv_inertia[1][1] * net_y + inv_inertia[1][2] * net_z
        s_dot[12] = inv_inertia[2][0] * net_x + inv_inertia[2][1] * net_y + inv_inertia[2][2] * net_z
        s_dot[13] = s_dot[14] = s_dot[15] = 0.0
        for index in range(self.rotors):
            s_dot[16 + index] = (commanded[index] - s[16 + index]) / self.tau_m
        return s_dot
