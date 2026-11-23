"""Your controller. Replace the body of reset() and act() below.
As shipped it only holds the start position; examples/pd_controller.py is a complete controller to start from.

Only this submission/ folder is evaluated: upload it on the practice server. See COMPETITION.md for the rules.

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
    config.nominal_mass_kg    0.5
    config.gravity_m_s2       9.81
    config.thrust_min_N/max_N 0 / 15, max_N is the limit at full charge (see observation.thrust_limit_N)
    config.body_rate_limit_rad_s   3, per axis
    config.body_rate_gain_s_inv    25, built-in rate loop: w_dot = 25 * (w_cmd - w)
    config.vehicle_radius_m   0.28, collision sphere
    config.laps               number of laps (5 in the competition); gate 0 follows the last gate
    config.act_budget_s       0.02, compute budget of one act() call on the official machine
    config.reset_budget_s     60, CPU budget of importing this file + reset() on the official machine
    config.battery_*          battery model parameters (drone_race.battery has the exact equations)

act(observation: Observation) is called every control_dt_s with noise-free state:
    observation.time_s            simulation time [s]
    observation.position_m        (x, y, z) world position [m]
    observation.velocity_m_s      (vx, vy, vz) world velocity [m/s]
    observation.quaternion_xyzw   attitude, body -> world
    observation.body_rates_rad_s  (p, q, r) body angular velocity [rad/s]
    observation.next_gate         index of the next gate to pass (judged by the referee); wraps to 0 each lap
    observation.lap               current lap, 0-based (config.laps laps in total)
    observation.gates_passed      gates passed in order so far (all laps) = lap * n_gates + next_gate
    observation.gate_collisions   contacts with gate frames so far; each adds 5 s to the score
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

from drone_race import Action
from drone_race.helpers import track_position


class Controller:
    def reset(self, config):
        """Called once before the flight. Precompute paths, models, ... here (reset_budget_s)."""
        self.config = config

    def act(self, observation) -> Action:
        """Called every control_dt_s; must return within act_budget_s."""
        # TODO: fly through config.track.gates in order. This placeholder holds the start
        # position with the PD helper; a raw Action(thrust_N, body_rates_rad_s) works the same way.
        return track_position(observation, self.config.track.start, self.config)
