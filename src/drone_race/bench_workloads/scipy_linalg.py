"""bench workload: SciPy dense linear algebra and a bounded least-squares (QP-like) solve."""

import numpy as np
from scipy import linalg, optimize

from drone_race import Action

N = 160
REPEATS = 5


class Controller:
    def reset(self, config):
        self.config = config
        rng = np.random.default_rng(1)
        a = rng.normal(size=(N, N))
        self.H = a @ a.T + N * np.eye(N)
        self.g = rng.normal(size=N)
        self.A, self.B = np.eye(12) + 0.02 * rng.normal(size=(12, 12)), rng.normal(size=(12, 4))
        self.C, self.d = rng.normal(size=(80, 40)), rng.normal(size=80)

    def act(self, observation):
        total = 0.0
        for _ in range(REPEATS):
            z = linalg.cho_solve(linalg.cho_factor(self.H), self.g)
            P = linalg.solve_discrete_are(self.A, self.B, np.eye(12), np.eye(4))
            bounded = optimize.lsq_linear(self.C, self.d, bounds=(-0.5, 0.5), method="bvls")
            total += float(z.sum() + np.trace(P) + bounded.cost)
        self.last = total
        return Action(self.config.nominal_mass_kg * self.config.gravity_m_s2, (0.0, 0.0, 0.0))
