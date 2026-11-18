from copy import deepcopy

import matplotlib
matplotlib.use("Agg")
import numpy as np
from scipy.spatial.transform import Rotation

from drone_race.replay import RaceView
from drone_race.track import demo_track


def test_vehicle_mesh_follows_world_pose_and_keeps_log_unchanged():
    """An x-forward body yawed +90 degrees must point along world +y."""
    sample = {
        "time_s": 1.0, "position_m": [3.0, 0.0, 1.5], "velocity_m_s": [1.0, 0.0, 0.0],
        "quaternion_xyzw": Rotation.from_euler("z", 90, degrees=True).as_quat().tolist(),
        "next_gate": 1,
    }
    original = deepcopy(sample)
    view = RaceView(demo_track())
    try:
        view.update(sample)
        nose = np.array(view.drone.nose.get_data_3d()).T
        assert np.allclose(nose[0], sample["position_m"])
        assert np.allclose(nose[1] - nose[0], [0.0, 0.23, 0.0])
        # The library-provided Quadrotor contains four rotor meshes.
        assert len(view.drone.rotors.rotors) == 4
        assert sample == original
    finally:
        view.close()
