"""Example nonlinear MPC racer (iLQR / Gauss-Newton DDP).

Every act() call solves, warm-started from the previous solution,

    min_Z  sum_k 1/2 |r(x_k, u_k)|^2 + 1/2 |r_N(x_N)|^2
    s.t.   x_{k+1} = f(x_k, u_k),   u_k = sat(z_k) = u_mid + u_half * tanh(z_k)

over a 1 s horizon, where f is drone_race.planning.QuadModel (thrust, body-rate
loop and rotor drag) and r penalizes deviation from the racing line reference
(position, velocity and thrust direction) plus input effort.

The input bounds (thrust, and especially the 3 rad/s body-rate limit) are built
into the model through the tanh squashing, so the optimizer plans with
saturation instead of discovering it after clipping. Dynamics and residual
Jacobians come from batched finite differences, so the cost is easy to change.

Copy this file to submission/controller.py to submit it.
"""

import numpy as np

from drone_race import Action
from drone_race.battery import terminal_voltage, thrust_limit
from drone_race.planning import DisturbanceObserver, QuadModel, RacingLine, quat_to_matrix, state_from_observation

HORIZON = 25           # steps
MODEL_DT = 0.04        # s, horizon = 1.0 s
ITERATIONS = 3         # iLQR iterations per control step (warm-started)
LINE_SEARCH = np.array([1.0, 0.5, 0.25, 0.1])   # step sizes, evaluated together
THRUST_MIN = 1.5       # N, near-zero thrust leaves the rotors no authority to turn the body
THRUST_MAX = 14.0      # N, keeps a little thrust margin for disturbances; the battery limit is applied on top
# Racing-line aggressiveness (drone_race.planning.RacingLine); the main lap-time knob.
# Tuned on both shipped tracks, six random tracks and disturbed runs; a_lateral 35+ is faster on clean runs
# but crashes under wind and mass error.
LINE = dict(v_max=16.0, a_lateral=30.0, a_accel=12.0, a_brake=14.0)
# Battery pacing: the line above is the full-battery line. As the pack drains, the thrust it can
# sustain drops, so act() switches to a slower speed profile on the same path (see pace()).
# PACE_RATIO maps the sustainable lateral acceleration to the line's a_lateral.
PACE_LEVELS = np.linspace(0.4, 1.0, 13)   # a_lateral/a_accel/a_brake scales relative to LINE
PACE_RATIO = 0.8

# Residual weights (square roots of quadratic weights).
W_POS, W_VEL, W_TILT, W_RATE = 6.0, 1.0, 2.0, 0.3
W_THRUST, W_SQUASH = 0.15, 0.05
W_RATE_CMD = np.array([0.3, 0.3, 1.0])   # yaw is free on this course; spend authority on tilt
W_POS_TERMINAL, W_VEL_TERMINAL = 10.0, 2.0
# Gate funnel: near an upcoming gate plane, penalize the offset from the gate center.
W_GATE, GATE_FUNNEL_M = 4.0, 0.6


class Controller:
    def reset(self, config):
        self.config = config
        self.model = QuadModel(config, MODEL_DT)
        self.model.u_low[0] = max(self.model.u_low[0], THRUST_MIN)
        self.model.u_high[0] = min(self.model.u_high[0], THRUST_MAX)
        self.u_mid = 0.5 * (self.model.u_high + self.model.u_low)
        self.u_half = 0.5 * (self.model.u_high - self.model.u_low)
        # Same path, different speed profiles: arc length s is shared, so switching is seamless.
        self.lines = [RacingLine(config.track, laps=config.laps, v_max=LINE["v_max"],
                                 **{k: scale * LINE[k] for k in ("a_lateral", "a_accel", "a_brake")})
                      for scale in PACE_LEVELS]
        self.line = self.lines[-1]
        self.observer = DisturbanceObserver(config)
        hover = np.array([self.model.hover_thrust, 0.0, 0.0, 0.0])
        # Actuation delay: an action is flown one control step after it is returned.
        self.delay_model = QuadModel(config, config.control_dt_s)
        self.flying = hover.copy()           # command flown until the next control step
        self.Z = np.tile(np.arctanh((hover - self.u_mid) / self.u_half), (HORIZON, 1))
        self.s = 0.0
        gates = config.track.gates
        self.gate_center = np.array([g.center for g in gates])
        self.gate_yaw = np.array([g.yaw_rad for g in gates])
        self.funnel = []                     # gate indices the funnel applies to this step

    def gate_residual(self, p):
        """W_GATE * (lateral, vertical) offset, faded out away from each funnel gate's plane."""
        terms = []
        for gate in self.funnel:
            c, s = np.cos(self.gate_yaw[gate]), np.sin(self.gate_yaw[gate])
            delta = p - self.gate_center[gate]
            along = c * delta[..., 0] + s * delta[..., 1]
            fade = W_GATE * np.exp(-0.5 * (along / GATE_FUNNEL_M) ** 2)
            terms.append(np.stack((fade * (-s * delta[..., 0] + c * delta[..., 1]), fade * delta[..., 2]), axis=-1))
        return np.concatenate(terms, axis=-1) if terms else p[..., :0]

    def saturate(self, z):
        return self.u_mid + self.u_half * np.tanh(z)

    def set_thrust_ceiling(self, ceiling):
        """Move the thrust squashing range, keeping the warm-start thrust plan (clipped) unchanged."""
        ceiling = max(min(THRUST_MAX, ceiling), self.model.u_low[0] + 0.5)
        if ceiling == self.model.u_high[0]:
            return
        thrust = self.saturate(self.Z)[:, 0]
        self.model.u_high[0] = ceiling
        self.u_mid = 0.5 * (self.model.u_high + self.model.u_low)
        self.u_half = 0.5 * (self.model.u_high - self.model.u_low)
        self.Z[:, 0] = np.arctanh(np.clip((thrust - self.u_mid[0]) / self.u_half[0], -0.999, 0.999))

    def dynamics(self, x, z, d):
        return self.model.step(x, self.saturate(z), d)

    # ------------------------------------------------------------ cost
    def residual(self, x, z, ref, terminal=False):
        """Stacked residuals for any batch shape; ref = (p_ref, v_ref, z_ref, thrust_ref)."""
        p_ref, v_ref, thrust_dir_ref, thrust_ref = ref
        body_z = quat_to_matrix(x[..., 6:10])[..., :, 2]
        if terminal:
            return np.concatenate((W_POS_TERMINAL * (x[..., 0:3] - p_ref), W_VEL_TERMINAL * (x[..., 3:6] - v_ref),
                                   W_TILT * (body_z - thrust_dir_ref)), axis=-1)
        u = self.saturate(z)
        return np.concatenate((W_POS * (x[..., 0:3] - p_ref), W_VEL * (x[..., 3:6] - v_ref),
                               W_TILT * (body_z - thrust_dir_ref), W_RATE * x[..., 10:13],
                               W_THRUST * (u[..., :1] - thrust_ref[..., None]), W_RATE_CMD * u[..., 1:4],
                               W_SQUASH * z, self.gate_residual(x[..., 0:3])), axis=-1)

    def total_cost(self, X, Z, refs):
        """Cost of one trajectory, or of a batch with leading axes before (time, dim)."""
        stage = self.residual(X[..., :-1, :], Z, tuple(r[:-1] for r in refs))
        final = self.residual(X[..., -1, :], None, tuple(r[-1] for r in refs), terminal=True)
        return 0.5 * (np.sum(stage**2, axis=(-2, -1)) + np.sum(final**2, axis=-1))

    def rollout(self, x0, Z, d):
        X = np.empty((HORIZON + 1, QuadModel.NX))
        X[0] = x0
        for k in range(HORIZON):
            X[k + 1] = self.dynamics(X[k], Z[k], d)
        return X

    # ------------------------------------------------------------ derivatives
    def linearize(self, X, Z, refs, d, eps=1e-5):
        """Batched forward differences of dynamics and residuals around (X, Z)."""
        nx, nu = QuadModel.NX, QuadModel.NU
        n = nx + nu
        XZ = np.concatenate((X[:-1], Z), axis=1)                                 # (N, n)
        XZp = np.repeat(XZ[:, None, :], n + 1, axis=1)
        XZp[:, 1:, :] += eps * np.eye(n)                                         # (N, n+1, n)
        stage_ref = tuple(np.repeat(r[:-1, None], n + 1, axis=1) for r in refs)
        F = self.dynamics(XZp[..., :nx], XZp[..., nx:], d)
        R = self.residual(XZp[..., :nx], XZp[..., nx:], stage_ref)
        J_f = (F[:, 1:] - F[:, :1]).transpose(0, 2, 1) / eps                     # (N, nx, n)
        J_r = (R[:, 1:] - R[:, :1]).transpose(0, 2, 1) / eps                     # (N, nr, n)
        Xt = np.repeat(X[-1][None], nx + 1, axis=0)
        Xt[1:] += eps * np.eye(nx)
        Rt = self.residual(Xt, None, tuple(np.repeat(r[-1][None], nx + 1, axis=0) for r in refs), terminal=True)
        J_t = (Rt[1:] - Rt[:1]).T / eps
        return J_f[..., :nx], J_f[..., nx:], J_r, R[:, 0], J_t, Rt[0]

    # ------------------------------------------------------------ iLQR
    def solve(self, x0, refs, d):
        nx, nu = QuadModel.NX, QuadModel.NU
        Z = self.Z
        X = self.rollout(x0, Z, d)
        cost = self.total_cost(X, Z, refs)
        mu = 1e-4
        for _ in range(ITERATIONS):
            A, B, J_r, r, J_t, r_t = self.linearize(X, Z, refs, d)
            Vx, Vxx = J_t.T @ r_t, J_t.T @ J_t
            K = np.empty((HORIZON, nu, nx))
            k_ff = np.empty((HORIZON, nu))
            for k in range(HORIZON - 1, -1, -1):
                Jx, Ju = J_r[k, :, :nx], J_r[k, :, nx:]
                VB = Vxx @ B[k]
                Qx, Qu = Jx.T @ r[k] + A[k].T @ Vx, Ju.T @ r[k] + B[k].T @ Vx
                Qxx = Jx.T @ Jx + A[k].T @ Vxx @ A[k]
                Quu = Ju.T @ Ju + B[k].T @ VB + mu * np.eye(nu)
                Qux = Ju.T @ Jx + VB.T @ A[k]
                Quu_inv = np.linalg.inv(Quu)
                K[k], k_ff[k] = -Quu_inv @ Qux, -Quu_inv @ Qu
                Vx = Qx + K[k].T @ Quu @ k_ff[k] + K[k].T @ Qu + Qux.T @ k_ff[k]
                Vxx = Qxx + K[k].T @ Quu @ K[k] + K[k].T @ Qux + Qux.T @ K[k]
                Vxx = 0.5 * (Vxx + Vxx.T)
            # Line search: roll out all step sizes at once as a batch, keep the best.
            X_new = np.empty((len(LINE_SEARCH), HORIZON + 1, nx))
            Z_new = np.empty((len(LINE_SEARCH), HORIZON, nu))
            X_new[:, 0] = x0
            for k in range(HORIZON):
                Z_new[:, k] = Z[k] + LINE_SEARCH[:, None] * k_ff[k] + (X_new[:, k] - X[k]) @ K[k].T
                X_new[:, k + 1] = self.dynamics(X_new[:, k], Z_new[:, k], d)
            new_cost = self.total_cost(X_new, Z_new, refs)
            best = int(np.argmin(new_cost))
            if new_cost[best] < cost:
                X, Z, cost = X_new[best], Z_new[best], new_cost[best]
                mu = max(mu / 10, 1e-6)
            else:
                mu *= 10
        return X, Z

    # ------------------------------------------------------------ interface
    def references(self, observation, x0, d):
        gates = self.line.gate_s
        nxt = min(observation.lap * len(self.gate_yaw) + observation.next_gate, len(gates) - 1)
        s_min = gates[nxt - 1] - 0.5 if nxt > 0 else 0.0
        self.s = self.line.project(x0[:3], max(s_min, self.s - 0.5), min(gates[nxt] + 1.0, self.s + 3.0))
        p_ref, v_ref, a_ref, _ = self.line.horizon(self.s, MODEL_DT, HORIZON)
        force = self.model.mass * (a_ref - self.model.gravity - d)
        thrust_ref = np.linalg.norm(force, axis=1)
        return p_ref, v_ref, force / thrust_ref[:, None], thrust_ref

    def pace(self, soc):
        """Fastest speed profile the battery can sustain: thrust limit at full draw, from soc alone
        (not the instantaneous sag, which would flip lines back and forth)."""
        sustained = min(THRUST_MAX, float(thrust_limit(terminal_voltage(soc, THRUST_MAX, self.config), self.config)))
        a_avail = np.sqrt(max((sustained / self.model.mass) ** 2 - self.config.gravity_m_s2 ** 2, 0.0))
        scale = PACE_RATIO * a_avail / LINE["a_lateral"]
        index = int(np.searchsorted(PACE_LEVELS, scale, side="right")) - 1
        return self.lines[max(index, 0)]

    def act(self, observation):
        # The battery (drone_race.battery) lowers the thrust ceiling as it drains and sags.
        self.set_thrust_ceiling(observation.thrust_limit_N)
        self.line = self.pace(observation.battery_soc)
        x_now = state_from_observation(observation)
        d = self.observer.update(x_now)
        # The action returned now is flown from the next control step; until then the drone
        # flies the previous one (u_0 is fixed). Plan from where that leaves it.
        x0 = self.delay_model.step(x_now, self.flying, d)
        refs = self.references(observation, x0, d)
        self.funnel = [observation.next_gate, (observation.next_gate + 1) % len(self.gate_yaw)]
        _, Z = self.solve(x0, refs, d)
        u0 = self.saturate(Z[0])
        # Warm start: shift the plan by one control period.
        t = np.arange(HORIZON) * MODEL_DT + self.config.control_dt_s
        self.Z = np.column_stack([np.interp(t, np.arange(HORIZON) * MODEL_DT, Z[:, j]) for j in range(4)])
        self.observer.remember(x_now, self.flying)
        self.flying = u0
        return Action(float(u0[0]), tuple(float(w) for w in u0[1:]))
