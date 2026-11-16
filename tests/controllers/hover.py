"""Alternative submission for testing a valid controller that does not finish."""
from quad_race import Action


class Controller:
    def reset(self, config):
        self.thrust = config.nominal_mass_kg * config.gravity_m_s2

    def act(self, observation):
        return Action(self.thrust, (0.0, 0.0, 0.0))
