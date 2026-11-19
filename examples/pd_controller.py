"""Example PD gate follower: a deliberately slow, conservative baseline.

Pure pursuit along the next gate's axis, with drone_race.helpers.track_position
(a PD position loop) turning the carrot into thrust and body rates. It ignores the
racing line, the battery and the actuation delay, and cruises at CRUISE_M_S, so it
finishes reliably but slowly. Start from it to see the interface working end to end.

Copy this file to submission/controller.py to submit it.
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
