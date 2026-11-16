"""Your submission: a conservative, non-optimal gate-following baseline to replace.

Only the submission/ folder is collected. See COMPETITION.md for the rules.

Conventions: SI units, world frame right-handed z-up, gravity (0, 0, -9.81) m/s^2,
quaternions (x, y, z, w) rotating body -> world. Body +z is the thrust axis.

reset(config: PublicConfig) is called once before flight with:
    config.track.gates[i]     Gate(center (x, y, z) [m], yaw_rad, width_m, height_m, frame_m)
                              Pass each gate in order along its local +x = (cos yaw, sin yaw, 0).
                              width_m/height_m are the inner opening.
    config.track.obstacles    Boxes (lower, upper corners) [m]
    config.track.bounds       Box the vehicle must stay inside, minus vehicle_radius_m
    config.track.start        initial hover position [m]
    config.control_dt_s       0.02, act() period; the action is held for this long
    config.nominal_mass_kg    0.5 (the true mass may differ in disturbed scenarios)
    config.gravity_m_s2       9.81
    config.thrust_min_N/max_N 0 / 15
    config.body_rate_limit_rad_s   3, per axis
    config.body_rate_gain_s_inv    25, built-in rate loop: w_dot = 25 * (w_cmd - w)
    config.vehicle_radius_m   0.28, collision sphere
    config.laps               number of laps (10 in the competition); gate 0 follows the last gate
    config.act_budget_s       0.02, compute budget of one act() call on the official machine
    config.reset_budget_s     60, compute budget of reset() on the official machine

act(observation: Observation) is called every control_dt_s with noise-free state:
    observation.time_s            simulation time [s]
    observation.position_m        (x, y, z) world position [m]
    observation.velocity_m_s      (vx, vy, vz) world velocity [m/s]
    observation.quaternion_xyzw   attitude, body -> world
    observation.body_rates_rad_s  (p, q, r) body angular velocity [rad/s]
    observation.next_gate         index of the next gate to pass (judged by the referee); wraps to 0 each lap
    observation.lap               current lap, 0-based (config.laps laps in total)

act() must return Action(thrust_N, body_rates_rad_s):
    thrust_N           collective thrust of all four rotors along body +z [N], in [0, 15]
    body_rates_rad_s   commanded (p, q, r) body angular velocity [rad/s], each in [-3, 3]
The simulator's rate controller, motor allocation, rotor saturation and motor lag turn this
into rotor speeds. Out-of-range, NaN or malformed actions end the run as invalid_action.
"""

import numpy as np
from quad_race.helpers import track_position


class Controller:
    def reset(self, config):
        self.config = config
        self.gate_index = -1
        self.approach = True

    def act(self, observation):
        index = observation.next_gate
        gate = self.config.track.gates[index]
        normal = np.array([np.cos(gate.yaw_rad), np.sin(gate.yaw_rad), 0.0])
        if index != self.gate_index:
            self.gate_index, self.approach = index, True
        approach = np.asarray(gate.center) - 0.9 * normal
        if self.approach and np.linalg.norm(np.asarray(observation.position_m) - approach) < 0.30:
            self.approach = False
        target = approach if self.approach else np.asarray(gate.center) + 0.9 * normal
        return track_position(observation, target, self.config)
