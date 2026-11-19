"""bench workload: plain Python (interpreter-bound loops over floats, lists and dicts)."""

import math

from drone_race import Action

ITERATIONS = 15000


class Controller:
    def reset(self, config):
        self.config = config

    def act(self, observation):
        x, y, table, points = 0.1, 0.2, {}, []
        for i in range(ITERATIONS):
            x, y = math.sin(x + y) + 0.5 * y, math.cos(x * y) - 0.25 * x
            points.append((x, y))
            table[i % 97] = table.get(i % 97, 0.0) + x * y
        self.last = sum(p[0] for p in points) + sum(table.values())
        return Action(self.config.nominal_mass_kg * self.config.gravity_m_s2, (0.0, 0.0, 0.0))
