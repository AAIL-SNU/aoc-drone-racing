"""Modify this file to implement your own controller.
The defalut implementation is a conservative, non-optimal gate-following baseline to replace.

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
    config.thrust_min_N/max_N 0 / 15, max_N is the limit at full charge (see observation.thrust_limit_N)
    config.body_rate_limit_rad_s   3, per axis
    config.body_rate_gain_s_inv    25, built-in rate loop: w_dot = 25 * (w_cmd - w)
    config.vehicle_radius_m   0.28, collision sphere
    config.laps               number of laps (5 in the competition); gate 0 follows the last gate
    config.act_budget_s       0.02, compute budget of one act() call on the official machine
    config.reset_budget_s     60, compute budget of reset() on the official machine
    config.battery_*          battery model parameters (drone_race.battery has the exact equations)

act(observation: Observation) is called every control_dt_s with noise-free state:
    observation.time_s            simulation time [s]
    observation.position_m        (x, y, z) world position [m]
    observation.velocity_m_s      (vx, vy, vz) world velocity [m/s]
    observation.quaternion_xyzw   attitude, body -> world
    observation.body_rates_rad_s  (p, q, r) body angular velocity [rad/s]
    observation.next_gate         index of the next gate to pass (judged by the referee); wraps to 0 each lap
    observation.lap               current lap, 0-based (config.laps laps in total)
    observation.gates_passed      gates flown through so far (all laps)
    observation.gates_missed      gates resolved as missed so far; passed + missed = lap * n_gates + next_gate
    observation.battery_soc       state of charge, 1 = full; reaching 0 ends the run (battery_depleted)
    observation.battery_voltage_V terminal voltage under the previous step's thrust (sags under load)
    observation.thrust_limit_N    current thrust ceiling = thrust_max_N * (battery_voltage_V / 12.6)^2

act() must return Action(thrust_N, body_rates_rad_s):
    thrust_N           collective thrust of all four rotors along body +z [N]; clipped (no penalty)
                       to [thrust_min_N, observation.thrust_limit_N] and flown one control step later
    body_rates_rad_s   commanded (p, q, r) body angular velocity [rad/s], each in [-3, 3]
The simulator's rate controller, motor allocation, rotor saturation and motor lag turn this
into rotor speeds. Body rates out of range, NaN or malformed actions end the run as invalid_action.

Battery: drawing thrust T costs power battery_power_coeff * T^1.5, so hard flying drains the pack
faster, and a drained or sagging pack lowers thrust_limit_N. See drone_race.battery.
"""

import numpy as np
from drone_race.helpers import track_position

CRUISE_M_S = 2.5      # speed along the path; 3.0 crashes on some wind seeds
LOOKAHEAD_M = 2.5     # carrot distance along the next gate's axis


class Controller:
    """PD tracking of a point that slides along the next gate's axis (pure pursuit).

    The carrot sits LOOKAHEAD_M ahead of the drone's projection on the gate axis, so the
    drone lines up with the gate from far away and flies through it without stopping.
    The desired velocity points at the carrot with speed CRUISE_M_S.
    """

    def reset(self, config):
        self.config = config

    def act(self, observation):
        gate = self.config.track.gates[observation.next_gate]
        center = np.asarray(gate.center)
        normal = np.array([np.cos(gate.yaw_rad), np.sin(gate.yaw_rad), 0.0])
        position = np.asarray(observation.position_m)
        along = float((position - center) @ normal)          # < 0 before the gate plane
        lateral = np.linalg.norm(position - center - along * normal)   # distance off the gate axis
        # Hold the carrot in front of the gate until the drone is lined up with the opening.
        target = center + min(along + LOOKAHEAD_M, 1.0 - lateral) * normal
        direction = target - position
        velocity = CRUISE_M_S * direction / max(np.linalg.norm(direction), 1e-6)
        return track_position(observation, target, self.config, target_velocity=velocity, max_accel=5.0)
