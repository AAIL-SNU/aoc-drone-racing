"""Example MPPI racer (Model Predictive Path Integral control).

Every act() call samples SAMPLES perturbed input sequences around the current
plan, rolls them all out in parallel through drone_race.planning.QuadModel, scores
each rollout and updates the plan with the softmin-weighted average of the noise:

    U <- U + sum_i w_i eps_i,   w_i ∝ exp(-(S_i - min S) / lambda)

Because rollouts are only evaluated, never differentiated, the cost can contain
hard, non-smooth terms: here a large penalty for crossing a gate plane outside
the (vehicle-radius-eroded) opening and for leaving the track bounds.

Copy this file to submission/controller.py to submit it.
"""

import numpy as np

from drone_race import Action
from drone_race.battery import terminal_voltage, thrust_limit
from drone_race.planning import DisturbanceObserver, QuadModel, RacingLine, quat_to_matrix, state_from_observation

SAMPLES = 512           # sized to fit the 20 ms act() budget with margin
HORIZON = 25           # steps
MODEL_DT = 0.04        # s, horizon = 1.0 s
TEMPERATURE = 0.05     # lambda, relative to the median cost spread (see act)
NOISE_STD = np.array([1.5, 1.0, 1.0, 0.6])   # thrust [N], body rates [rad/s]
NOISE_SMOOTHING = 0.6  # first-order filter on the noise along the horizon
THRUST_MIN = 1.5       # N, near-zero thrust leaves the rotors no authority to turn the body
THRUST_MAX = 14.0      # N, keeps a little thrust margin for disturbances; the battery limit is applied on top
# Racing-line aggressiveness (drone_race.planning.RacingLine); the main lap-time knob.
# a_lateral above ~26 m/s^2 exceeds what THRUST_MAX can deliver, so the reference
# runs ahead and MPPI cuts corners as far as the gate penalty allows. Finished all
# shipped, six random and disturbed test runs with this setting.
LINE = dict(v_max=16.0, a_lateral=40.0, a_accel=18.0, a_brake=20.0)
# Battery pacing: LINE is the full-battery line. As the pack drains, act() switches to a slower
# speed profile on the same path, scaled by the lateral acceleration the battery can still sustain.
PACE_LEVELS = np.linspace(0.4, 1.0, 13)   # a_lateral/a_accel/a_brake scales relative to LINE
SEED = 0               # private generator: never touches the global NumPy RNG

# Quadratic weights.
Q_POS, Q_VEL, Q_TILT, Q_RATE = 36.0, 1.0, 4.0, 0.1
R_THRUST, R_SMOOTH = 0.02, 0.5
R_RATE_CMD = np.array([0.1, 0.1, 1.0])   # yaw is free on this course; spend authority on tilt
Q_POS_TERMINAL, Q_VEL_TERMINAL = 100.0, 4.0
GATE_PENALTY, BOUNDS_PENALTY = 1e4, 1e4
GATE_MARGIN = 0.05     # m, extra clearance inside the eroded gate opening


class Controller:
    def reset(self, config):
        self.config = config
        self.rng = np.random.default_rng(SEED)
        self.model = QuadModel(config, MODEL_DT)
        self.model.u_low[0] = max(self.model.u_low[0], THRUST_MIN)
        self.model.u_high[0] = min(self.model.u_high[0], THRUST_MAX)
        # Same path, different speed profiles: arc length s is shared, so switching is seamless.
        self.lines = [RacingLine(config.track, laps=config.laps, v_max=LINE["v_max"],
                                 **{k: scale * LINE[k] for k in ("a_lateral", "a_accel", "a_brake")})
                      for scale in PACE_LEVELS]
        self.line = self.lines[-1]
        self.a_full = self.sustained_lateral(1.0)
        self.observer = DisturbanceObserver(config)
        self.U = np.tile([self.model.hover_thrust, 0.0, 0.0, 0.0], (HORIZON, 1))
        # Actuation delay: an action is flown one control step after it is returned.
        self.delay_model = QuadModel(config, config.control_dt_s)
        self.flying = np.array([self.model.hover_thrust, 0.0, 0.0, 0.0])   # command flown until the next step
        self.s = 0.0
        gates = config.track.gates
        self.gate_center = np.array([g.center for g in gates])
        self.gate_yaw = np.array([g.yaw_rad for g in gates])
        self.gate_half = np.array([[g.width_m / 2, g.height_m / 2] for g in gates]) - config.vehicle_radius_m - GATE_MARGIN
        bounds = config.track.bounds
        self.lower = np.asarray(bounds.lower) + config.vehicle_radius_m
        self.upper = np.asarray(bounds.upper) - config.vehicle_radius_m

    def references(self, observation, x0, d):
        gates = self.line.gate_s
        nxt = min(observation.lap * len(self.gate_yaw) + observation.next_gate, len(gates) - 1)
        s_min = gates[nxt - 1] - 0.5 if nxt > 0 else 0.0
        self.s = self.line.project(x0[:3], max(s_min, self.s - 0.5), min(gates[nxt] + 1.0, self.s + 3.0))
        p_ref, v_ref, a_ref, _ = self.line.horizon(self.s, MODEL_DT, HORIZON)
        force = self.model.mass * (a_ref - self.model.gravity - d)
        thrust_ref = np.linalg.norm(force, axis=1)
        return p_ref, v_ref, force / thrust_ref[:, None], thrust_ref

    def gate_violation(self, before, after, gate):
        """1 where a step crosses the gate plane forward outside the safe opening."""
        c, s = np.cos(self.gate_yaw[gate]), np.sin(self.gate_yaw[gate])
        def local(p):
            delta = p - self.gate_center[gate]
            return np.stack((c * delta[..., 0] + s * delta[..., 1], -s * delta[..., 0] + c * delta[..., 1], delta[..., 2]), -1)
        a, b = local(before), local(after)
        crosses = (a[..., 0] < 0) & (b[..., 0] >= 0)
        alpha = np.clip(-a[..., 0] / np.where(crosses, b[..., 0] - a[..., 0], 1.0), 0, 1)[..., None]
        hit = a + alpha * (b - a)
        outside = (np.abs(hit[..., 1]) > self.gate_half[gate, 0]) | (np.abs(hit[..., 2]) > self.gate_half[gate, 1])
        return crosses & outside

    def sustained_lateral(self, soc):
        """Lateral acceleration the pack sustains at full draw at this soc (ignores the momentary sag)."""
        thrust = min(THRUST_MAX, float(thrust_limit(terminal_voltage(soc, THRUST_MAX, self.config), self.config)))
        return np.sqrt(max((thrust / self.model.mass) ** 2 - self.config.gravity_m_s2 ** 2, 0.0))

    def act(self, observation):
        # The battery (drone_race.battery) lowers the thrust ceiling as it drains and sags.
        self.model.u_high[0] = min(THRUST_MAX, observation.thrust_limit_N)
        scale = self.sustained_lateral(observation.battery_soc) / self.a_full
        self.line = self.lines[max(int(np.searchsorted(PACE_LEVELS, scale, side="right")) - 1, 0)]
        x_now = state_from_observation(observation)
        d = self.observer.update(x_now)
        # The action returned now is flown from the next control step; until then the drone
        # flies the previous one (u_0 is fixed). Plan from where that leaves it.
        x0 = self.delay_model.step(x_now, self.flying, d)
        p_ref, v_ref, z_ref, thrust_ref = self.references(observation, x0, d)

        # Time-correlated exploration noise around the current plan.
        white = self.rng.standard_normal((SAMPLES, HORIZON, 4)) * NOISE_STD
        noise = np.empty_like(white)
        noise[:, 0] = white[:, 0]
        for k in range(1, HORIZON):
            noise[:, k] = NOISE_SMOOTHING * noise[:, k - 1] + np.sqrt(1 - NOISE_SMOOTHING**2) * white[:, k]
        noise[0] = 0.0                                      # keep the unperturbed plan as one sample
        U = np.clip(self.U + noise, self.model.u_low, self.model.u_high)
        noise = U - self.U

        gates = [observation.next_gate, (observation.next_gate + 1) % len(self.gate_yaw)]
        x = np.repeat(x0[None], SAMPLES, axis=0)
        cost = np.zeros(SAMPLES)
        for k in range(HORIZON):
            x_next = self.model.step(x, U[:, k], d)
            body_z = quat_to_matrix(x_next[:, 6:10])[:, :, 2]
            q_pos, q_vel = (Q_POS_TERMINAL, Q_VEL_TERMINAL) if k == HORIZON - 1 else (Q_POS, Q_VEL)
            cost += (q_pos * np.sum((x_next[:, 0:3] - p_ref[k + 1]) ** 2, axis=1)
                     + q_vel * np.sum((x_next[:, 3:6] - v_ref[k + 1]) ** 2, axis=1)
                     + Q_TILT * np.sum((body_z - z_ref[k + 1]) ** 2, axis=1)
                     + Q_RATE * np.sum(x_next[:, 10:13] ** 2, axis=1)
                     + R_THRUST * (U[:, k, 0] - thrust_ref[k]) ** 2
                     + np.sum(R_RATE_CMD * U[:, k, 1:] ** 2, axis=1))
            if k:
                cost += R_SMOOTH * np.sum((U[:, k] - U[:, k - 1]) ** 2, axis=1)
            for gate in gates:
                cost += GATE_PENALTY * self.gate_violation(x[:, 0:3], x_next[:, 0:3], gate)
            out = np.any((x_next[:, 0:3] < self.lower) | (x_next[:, 0:3] > self.upper), axis=1)
            cost += BOUNDS_PENALTY * out
            x = x_next

        # Softmin weights; lambda scales with the cost spread so tuning is track-independent.
        shifted = cost - cost.min()
        lam = TEMPERATURE * max(np.median(shifted), 1e-6)
        weights = np.exp(-shifted / lam)
        weights /= weights.sum()
        plan = np.clip(self.U + np.einsum("i,ikj->kj", weights, noise), self.model.u_low, self.model.u_high)

        u0 = plan[0].copy()
        t = np.arange(HORIZON) * MODEL_DT + self.config.control_dt_s
        self.U = np.column_stack([np.interp(t, np.arange(HORIZON) * MODEL_DT, plan[:, j]) for j in range(4)])
        self.observer.remember(x_now, self.flying)
        self.flying = u0
        return Action(float(u0[0]), tuple(float(w) for w in u0[1:]))
