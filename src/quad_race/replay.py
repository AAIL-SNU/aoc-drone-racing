"""Read-only course visualization using RotorPy's vehicle meshes."""

from functools import singledispatch
import json
from pathlib import Path
from contextlib import contextmanager
import warnings
import numpy as np
from scipy.spatial.transform import Rotation
from rotorpy.utils.shapes import Cuboid, Quadrotor

from .api import Box
from .track import track_from_dict


@contextmanager
def _mesh_compatibility():
    # RotorPy 3.0 still assigns ndarray.shape, deprecated (but functional) in
    # our pinned NumPy 2.5. Suppress only that upstream notice, inside mesh calls.
    # Owned input copies below avoid its in-place modifications reaching logs.
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", category=DeprecationWarning,
                                message="Setting the shape on a NumPy array has been deprecated.*",
                                module=r"rotorpy\.utils\.shapes")
        yield


class VehicleGlyph:
    """Actual Hummingbird-scale rotors, arms, center body and body +x marker.

    This geometry is only for display. Collision rules keep using their explicit
    conservative sphere. RotorPy.transform reshapes its inputs in place, so all
    data passed to upstream mesh objects is an owned copy.
    """

    def __init__(self, axes):
        with _mesh_compatibility():
            self.rotors = Quadrotor(axes, arm_length=0.17, rotor_radius=0.10,
                                   color="#23738b", wind=False)
            self.body = Cuboid(axes, 0.12, 0.085, 0.045, facecolors="#253744")
        self.arms = tuple(axes.plot([], [], [], color="#253744", linewidth=2.6)[0]
                          for _ in range(4))
        self.nose, = axes.plot([], [], [], color="#d4483b", linewidth=3.0)
        self.up, = axes.plot([], [], [], color="#438c55", linewidth=1.4)
        self.artists = (*self.rotors.artists, *self.body.artists, *self.arms, self.nose, self.up)

    def transform(self, position, rotation):
        center = np.array(position, dtype=float, copy=True)
        with _mesh_compatibility():
            self.rotors.transform(center.copy(), rotation)
            offset = rotation @ np.array([-0.06, -0.0425, -0.0225])
            self.body.transform(center + offset, rotation)
        for arm, local in zip(self.arms, self.rotors.rotor_position.T):
            endpoints = np.stack((center, center + rotation @ local))
            arm.set_data_3d(*endpoints.T)
        self.nose.set_data_3d(*np.stack((center, center + 0.23 * rotation[:, 0])).T)
        self.up.set_data_3d(*np.stack((center, center + 0.18 * rotation[:, 2])).T)


@singledispatch
def draw_obstacle(obstacle, axes) -> np.ndarray:
    """Draw an obstacle; return points that the course view should keep in frame."""
    raise TypeError(f"No renderer for obstacle type {type(obstacle).__name__}")


@draw_obstacle.register
def _(obstacle: Box, axes) -> np.ndarray:
    lo, hi = np.array(obstacle.lower), np.array(obstacle.upper)
    axes.bar3d(*lo, *(hi - lo), color="#ba5d48", alpha=0.5)
    return np.array([lo, hi])


class RaceView:
    def __init__(self, track, *, live=False):
        import matplotlib.pyplot as plt
        self.plt, self.live = plt, live
        self.figure = plt.figure(figsize=(10, 6))
        self.axes = self.figure.add_subplot(111, projection="3d")
        ax = self.axes
        self.gates = []
        for i, gate in enumerate(track.gates):
            w, h = gate.width_m / 2, gate.height_m / 2
            corners = np.array([[0, -w, -h], [0, w, -h], [0, w, h], [0, -w, h], [0, -w, -h]])
            c, s = np.cos(gate.yaw_rad), np.sin(gate.yaw_rad)
            rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            corners = corners @ rotation.T + gate.center
            line, = ax.plot(*corners.T, color="#7c8b96", linewidth=2.4)
            self.gates.append(line)
            ax.text(*gate.center, f"  {i + 1}", fontsize=9)
            ax.quiver(*gate.center, *rotation[:, 0], length=0.55, color="#7c8b96")
        extents = [draw_obstacle(obstacle, ax) for obstacle in track.obstacles]
        self.line, = ax.plot([], [], [], color="#dd8b28", linewidth=1.7, label="Flight path")
        self.drone = VehicleGlyph(ax)
        self.positions = []
        self.last_item = None
        self.title = self.figure.suptitle("Quadrotor racing", y=0.97, fontsize=15)
        # Frame the course for inspection; these display limits are not flight bounds.
        centers = np.array([track.start, *(g.center for g in track.gates)])
        padding = np.array([1.2, 1.5, 1.2])
        for gate in track.gates:
            padding = np.maximum(padding, [1.2, gate.width_m / 2 + 0.6, gate.height_m / 2 + 0.6])
        centers = np.vstack((centers, *extents))
        lo, hi = centers.min(axis=0) - padding, centers.max(axis=0) + padding
        lo = np.maximum(lo, np.array(track.bounds.lower))
        hi = np.minimum(hi, np.array(track.bounds.upper))
        ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), zlim=(lo[2], hi[2]),
               xlabel="x [m]", ylabel="y [m]", zlabel="z [m]")
        ax.set_box_aspect(np.maximum(hi - lo, 0.1))
        ax.view_init(elev=27, azim=-65)
        ax.set_title("Course overview", fontsize=11, pad=5)
        ax.legend(loc="upper left", fontsize=9)
        self.figure.subplots_adjust(left=0.02, right=0.96, bottom=0.03, top=0.87)
        if live:
            plt.ion()
            plt.show(block=False)

    def update(self, item):
        self.positions.append(item["position_m"])
        self.last_item = item
        # Display throttling is independent of the plant and the score.
        if self.live and len(self.positions) % 5:
            return
        self.draw_latest()
        if self.live:
            self.figure.canvas.draw_idle()
            self.figure.canvas.flush_events()

    def draw_latest(self):
        if not self.positions:
            return
        points = np.asarray(self.positions)
        self.line.set_data_3d(*points.T)
        rotation = Rotation.from_quat(self.last_item["quaternion_xyzw"]).as_matrix()
        self.drone.transform(points[-1], rotation)
        passed = self.last_item["next_gate"]
        for index, line in enumerate(self.gates):
            line.set_color("#438c55" if index < passed else "#276fa4" if index == passed else "#9ca7b0")
        self.title.set_text(f"Quadrotor racing | t = {self.last_item['time_s']:.2f} s | gates passed: {passed}")

    def save(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.draw_latest()
        self.figure.savefig(path, dpi=130)

    def close(self):
        self.plt.close(self.figure)


def replay(log_path, *, output=None, show=False, speed=2.0):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    if speed <= 0:
        raise ValueError("Replay speed must be positive")
    log = json.loads(Path(log_path).read_text())
    view = RaceView(track_from_dict(log["config"]["track"]))
    samples = log["samples"]
    try:
        if output and Path(output).suffix.lower() != ".gif":
            for item in samples:
                view.update(item)
            view.save(output)
            return log["summary"]
        times = np.array([item["time_s"] for item in samples])
        frame_times = np.arange(0, max(times[-1], 0.01), speed / 12)
        indices = np.unique(np.r_[np.searchsorted(times, frame_times).clip(0, len(samples) - 1), len(samples) - 1])

        def frame(index):
            view.positions = [item["position_m"] for item in samples[:index]]
            view.update(samples[index])
            return view.line, *view.drone.artists, view.title

        animation = FuncAnimation(view.figure, frame, frames=indices, interval=1000 / 12, blit=False)
        if output:
            Path(output).parent.mkdir(parents=True, exist_ok=True)
            animation.save(output, writer=PillowWriter(fps=12))
        if show:
            plt.show()
        return log["summary"]
    finally:
        view.close()
