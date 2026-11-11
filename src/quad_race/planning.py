"""Optional planning helpers used by the example MPC and MPPI controllers.

RacingLine: a smooth path through the gate centers (perpendicular to each gate)
with a speed profile limited by lateral and longitudinal acceleration.

QuadModel: a batched, discrete-time model of the plant seen through the public
action interface (collective thrust + body-rate commands). It includes the
first-order body-rate loop and RotorPy's Hummingbird rotor drag, parasitic drag
and translational lift, whose coefficients are public RotorPy parameters.
Students may replace all of this.
"""

import numpy as np

from .api import PublicConfig, Track

# RotorPy Hummingbird aerodynamic coefficients (rotorpy.vehicles.hummingbird_params).
K_ETA = 5.57e-6                      # rotor thrust coefficient [N/(rad/s)^2]
ROTOR_DRAG = np.array([1.19e-4, 1.19e-4, 2.32e-4])   # k_d, k_d, k_z [N/(rad/s)/(m/s)]
K_H = 3.39e-3                        # translational lift per rotor [N/(m/s)^2]
PARASITIC_DRAG = np.array([5e-3, 5e-3, 1e-2])        # c_Dx, c_Dy, c_Dz [N/(m/s)^2]


# ---------------------------------------------------------------- quaternions (x, y, z, w)
def quat_multiply(a, b):
    ax, ay, az, aw = np.moveaxis(a, -1, 0)
    bx, by, bz, bw = np.moveaxis(b, -1, 0)
    return np.stack((aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz), axis=-1)


def quat_to_matrix(q):
    x, y, z, w = np.moveaxis(q, -1, 0)
    return np.stack((
        np.stack((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)), axis=-1),
        np.stack((2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)), axis=-1),
        np.stack((2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)), axis=-1),
    ), axis=-2)


def quat_exp(rotation_vector):
    angle = np.linalg.norm(rotation_vector, axis=-1, keepdims=True)
    scale = np.where(angle > 1e-9, np.sin(angle / 2) / np.maximum(angle, 1e-9), 0.5)
    return np.concatenate((scale * rotation_vector, np.cos(angle / 2)), axis=-1)


# ---------------------------------------------------------------- model
class QuadModel:
    """x = [p(3), v(3), q_xyzw(4), w(3)], u = [thrust_N, p_cmd, q_cmd, r_cmd]. Works on any batch shape."""

    NX, NU = 13, 4

    def __init__(self, config: PublicConfig, dt: float, aero: bool = True):
        self.dt, self.aero = dt, aero
        self.mass = config.nominal_mass_kg
        self.gravity = np.array([0.0, 0.0, -config.gravity_m_s2])
        self.rate_alpha = 1 - np.exp(-config.body_rate_gain_s_inv * dt)
        self.u_low = np.array([config.thrust_min_N, *[-config.body_rate_limit_rad_s] * 3])
        self.u_high = np.array([config.thrust_max_N, *[config.body_rate_limit_rad_s] * 3])
        self.hover_thrust = self.mass * config.gravity_m_s2

    def body_force(self, R, v, thrust):
        """Thrust plus aerodynamic force in the body frame."""
        force = np.zeros(v.shape)
        force[..., 2] = thrust
        if self.aero:
            vb = np.einsum("...ji,...j->...i", R, v)
            rotor_speed_sum = 2 * np.sqrt(np.maximum(thrust, 0.0) / K_ETA)
            force -= rotor_speed_sum[..., None] * ROTOR_DRAG * vb
            force -= np.linalg.norm(vb, axis=-1, keepdims=True) * PARASITIC_DRAG * vb
            force[..., 2] += 4 * K_H * (vb[..., 0] ** 2 + vb[..., 1] ** 2)
        return force

    def acceleration(self, x, u, disturbance=None):
        R = quat_to_matrix(x[..., 6:10])
        a = np.einsum("...ij,...j->...i", R, self.body_force(R, x[..., 3:6], u[..., 0])) / self.mass + self.gravity
        return a if disturbance is None else a + disturbance

    def step(self, x, u, disturbance=None):
        p, v, q, w = x[..., 0:3], x[..., 3:6], x[..., 6:10], x[..., 10:13]
        a = self.acceleration(x, u, disturbance)
        w_next = w + self.rate_alpha * (u[..., 1:4] - w)
        v_next = v + self.dt * a
        p_next = p + 0.5 * self.dt * (v + v_next)
        q_next = quat_multiply(q, quat_exp(0.5 * self.dt * (w + w_next)))
        q_next /= np.linalg.norm(q_next, axis=-1, keepdims=True)
        return np.concatenate((p_next, v_next, q_next, w_next), axis=-1)


def state_from_observation(observation) -> np.ndarray:
    return np.concatenate((observation.position_m, observation.velocity_m_s,
                           observation.quaternion_xyzw, observation.body_rates_rad_s)).astype(float)


class DisturbanceObserver:
    """Low-pass estimate of the acceleration the model misses (mass error, wind, model error)."""

    def __init__(self, config: PublicConfig, gain: float = 0.1, limit: float = 4.0, substeps: int = 5):
        self.model = QuadModel(config, config.control_dt_s / substeps)
        self.substeps, self.gain, self.limit = substeps, gain, limit
        self.estimate = np.zeros(3)
        self.previous = None

    def update(self, x):
        """Correct the estimate with the velocity error of the last one-step prediction."""
        if self.previous is not None:
            x_pred = self.previous[0]
            for _ in range(self.substeps):
                x_pred = self.model.step(x_pred, self.previous[1], self.estimate)
            residual = (x[3:6] - x_pred[3:6]) / (self.model.dt * self.substeps)
            self.estimate = np.clip(self.estimate + self.gain * residual, -self.limit, self.limit)
        return self.estimate

    def remember(self, x, u_applied):
        """Store the state and the action sent this step for the next update."""
        self.previous = (x.copy(), np.asarray(u_applied, dtype=float))


# ---------------------------------------------------------------- racing line
def _hermite(p0, p1, m0, m1, u):
    u = u[:, None]
    h00, h10 = 2 * u**3 - 3 * u**2 + 1, u**3 - 2 * u**2 + u
    h01, h11 = -2 * u**3 + 3 * u**2, u**3 - u**2
    return h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1


class RacingLine:
    """Arc-length sampled path through all gates plus a feasible speed profile v(s)."""

    def __init__(self, track: Track, *, v_max=10.0, a_lateral=14.0, a_accel=9.0, a_brake=10.0,
                 tangent_scale=1.0, lead_out_m=5.0, ds=0.05):
        normals = [np.array([np.cos(g.yaw_rad), np.sin(g.yaw_rad), 0.0]) for g in track.gates]
        start = np.asarray(track.start, dtype=float)
        points = [start] + [np.asarray(g.center, dtype=float) for g in track.gates]
        points.append(points[-1] + lead_out_m * normals[-1])
        first = points[1] - start
        directions = [first / np.linalg.norm(first)] + normals + [normals[-1]]

        dense, knot_index = [], [0]
        for i in range(len(points) - 1):
            length = np.linalg.norm(points[i + 1] - points[i]) * tangent_scale
            u = np.linspace(0, 1, 400, endpoint=False)
            dense.append(_hermite(points[i], points[i + 1], directions[i] * length, directions[i + 1] * length, u))
            knot_index.append(knot_index[-1] + len(u))
        dense.append(points[-1][None])
        dense = np.vstack(dense)
        arc = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))))

        self.ds = ds
        self.s = np.arange(0.0, arc[-1], ds)
        self.position = np.column_stack([np.interp(self.s, arc, dense[:, j]) for j in range(3)])
        self.gate_s = arc[knot_index[1:-1]]          # arc length of each gate center
        tangent = np.gradient(self.position, ds, axis=0)
        self.tangent = tangent / np.linalg.norm(tangent, axis=1, keepdims=True)
        self.curvature_vector = np.gradient(self.tangent, ds, axis=0)
        curvature = np.linalg.norm(self.curvature_vector, axis=1)

        # Speed profile: lateral limit, then forward (accelerate) and backward (brake) passes.
        speed = np.minimum(v_max, np.sqrt(a_lateral / np.maximum(curvature, 1e-6)))
        speed[0] = 0.0
        for i in range(1, len(speed)):
            speed[i] = min(speed[i], np.sqrt(speed[i - 1] ** 2 + 2 * a_accel * ds))
        for i in range(len(speed) - 2, -1, -1):
            speed[i] = min(speed[i], np.sqrt(speed[i + 1] ** 2 + 2 * a_brake * ds))
        self.speed = speed
        dv_ds = np.gradient(speed, ds)
        self.acceleration = speed[:, None] ** 2 * self.curvature_vector + (speed * dv_ds)[:, None] * self.tangent
        self.lap_time_s = float(np.sum(ds / np.maximum(0.5 * (speed[1:] + speed[:-1]), 1e-3)))

    def index(self, s):
        return np.clip(np.round(np.asarray(s) / self.ds).astype(int), 0, len(self.s) - 1)

    def project(self, position, s_min, s_max):
        """Closest path arc length to ``position`` inside the window [s_min, s_max]."""
        lo, hi = self.index(max(s_min, 0.0)), self.index(s_max) + 1
        distances = np.linalg.norm(self.position[lo:hi] - position, axis=1)
        return self.s[lo + int(np.argmin(distances))]

    def horizon(self, s0, dt, steps, min_speed=1.0):
        """Reference (p, v, a) at steps+1 times, advancing along the speed profile from s0."""
        s = np.empty(steps + 1)
        s[0] = s0
        for k in range(steps):
            s[k + 1] = s[k] + max(self.speed[self.index(s[k])], min_speed) * dt
        i = self.index(s)
        return self.position[i], self.speed[i, None] * self.tangent[i], self.acceleration[i], s
