"""bench workload: batched NumPy rollouts (elementwise math on (samples, state) arrays)."""

import numpy as np

from drone_race import Action

SAMPLES, STEPS = 512, 100


class Controller:
    def reset(self, config):
        self.config = config
        rng = np.random.default_rng(0)
        self.x0 = rng.normal(size=(SAMPLES, 6))
        self.u = rng.normal(size=(STEPS, SAMPLES, 3))
        self.A = np.eye(6) + 0.02 * rng.normal(size=(6, 6))

    def act(self, observation):
        x, cost = self.x0.copy(), np.zeros(SAMPLES)
        for k in range(STEPS):
            x = x @ self.A.T
            x[:, 3:] += 0.02 * (np.tanh(self.u[k]) - 0.1 * np.sin(x[:, :3]))
            cost += np.sum(x * x, axis=1) + 0.1 * np.sum(self.u[k] ** 2, axis=1)
        weights = np.exp(-(cost - cost.min()) / max(cost.std(), 1e-9))
        self.last = weights @ self.u[0] / weights.sum()
        return Action(self.config.nominal_mass_kg * self.config.gravity_m_s2, (0.0, 0.0, 0.0))
