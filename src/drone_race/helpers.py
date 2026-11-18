"""Optional public position-tracking helper; students may replace it entirely."""

import numpy as np
from scipy.spatial.transform import Rotation

from .api import Action, Observation, PublicConfig


def track_position(observation: Observation, target, config: PublicConfig, *, target_velocity=(0.0, 0.0, 0.0),
                   kp: float = 2.0, kd: float = 2.8, max_accel: float = 3.0) -> Action:
    """PD on position/velocity to a desired acceleration, then thrust along it and rates to tilt there."""
    position, velocity = np.asarray(observation.position_m), np.asarray(observation.velocity_m_s)
    acceleration = kp * (np.asarray(target) - position) + kd * (np.asarray(target_velocity) - velocity)
    norm = np.linalg.norm(acceleration)
    if norm > max_accel:
        acceleration *= max_accel / norm
    force = config.nominal_mass_kg * (acceleration + [0, 0, config.gravity_m_s2])
    rotation = Rotation.from_quat(observation.quaternion_xyzw).as_matrix()
    thrust = float(force @ rotation[:, 2])
    z_des = force / np.linalg.norm(force)
    y_des = np.cross(z_des, [1., 0., 0.])
    y_des /= np.linalg.norm(y_des)
    desired = np.column_stack((np.cross(y_des, z_des), y_des, z_des))
    skew = 0.5 * (desired.T @ rotation - rotation.T @ desired)
    error = np.array([-skew[1, 2], skew[0, 2], -skew[0, 1]])
    rates = np.clip(-6.0 * error, -config.body_rate_limit_rad_s, config.body_rate_limit_rad_s)
    return Action(float(np.clip(thrust, config.thrust_min_N, observation.thrust_limit_N)), tuple(rates))
