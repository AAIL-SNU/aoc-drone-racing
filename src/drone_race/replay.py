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


def euler_deg(quaternions_xyzw):
    """Roll, pitch, yaw [deg] (intrinsic Z-Y-X), with yaw unwrapped over time."""
    yaw, pitch, roll = Rotation.from_quat(quaternions_xyzw).as_euler("ZYX").T
    return np.degrees(np.stack((roll, pitch, np.unwrap(yaw)), axis=1))


class AttitudePanel:
    """Close-up chase view of the vehicle plus roll/pitch/yaw history.

    The chase camera follows yaw, so roll and pitch read directly off the glyph.
    """

    COLORS = ("#c0392b", "#2e86c1", "#7d8b96")

    def __init__(self, figure, pose_spec, plot_spec, times, angles_deg):
        self.times, self.angles = np.asarray(times), np.asarray(angles_deg)
        self.pose = figure.add_subplot(pose_spec, projection="3d")
        r = 0.3
        self.pose.set(xlim=(-r, r), ylim=(-r, r), zlim=(-r, r))
        self.pose.set_box_aspect((1, 1, 1), zoom=1.15)
        self.pose.set_axis_off()
        self.pose.plot([-r, r], [0, 0], [0, 0], color="#c7ced4", linewidth=0.8)
        self.pose.plot([0, 0], [-r, r], [0, 0], color="#c7ced4", linewidth=0.8)
        self.pose.plot([0, 0], [0, 0], [-r, r], color="#c7ced4", linewidth=0.8)
        self.glyph = VehicleGlyph(self.pose)
        self.readout = self.pose.set_title("", fontsize=10, pad=2, family="monospace")

        self.plot = figure.add_subplot(plot_spec)
        self.traces, self.markers = [], []
        for k, (name, color) in enumerate(zip(("roll", "pitch", "yaw"), self.COLORS)):
            style = dict(color=color, linewidth=1.3 if k < 2 else 1.0, linestyle="-" if k < 2 else "--")
            self.plot.plot(self.times, self.angles[:, k], alpha=0.18, **{**style, "linewidth": 1.0})
            self.traces.append(self.plot.plot([], [], label=name, **style)[0])
            self.markers.append(self.plot.plot([], [], "o", color=color, markersize=4)[0])
        self.cursor = self.plot.axvline(0, color="#253744", linewidth=0.8)
        self.plot.axhline(0, color="#c7ced4", linewidth=0.6)
        self.plot.set(xlim=(self.times[0], max(self.times[-1], self.times[0] + 0.01)),
                      xlabel="t [s]", ylabel="angle [deg]")
        self.plot.grid(alpha=0.25)
        self.plot.legend(loc="upper right", fontsize=8, ncol=3)
        self.plot.tick_params(labelsize=8)
        self.artists = (*self.glyph.artists, self.readout, *self.traces, *self.markers, self.cursor)

    def draw(self, time_s, rotation):
        index = int(np.searchsorted(self.times, time_s, side="right"))
        index = max(index, 1)
        t, a = self.times[:index], self.angles[:index]
        for k, (trace, marker) in enumerate(zip(self.traces, self.markers)):
            trace.set_data(t, a[:, k])
            marker.set_data([t[-1]], [a[-1, k]])
        self.cursor.set_xdata([time_s, time_s])
        roll, pitch, yaw = a[-1]
        tilt = np.degrees(np.arccos(np.clip(rotation[2, 2], -1.0, 1.0)))
        self.glyph.transform(np.zeros(3), rotation)
        self.pose.view_init(elev=18, azim=np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0])) + 205)
        self.readout.set_text(f"roll {roll:+6.1f}°  pitch {pitch:+6.1f}°\n"
                              f"yaw {(yaw + 180) % 360 - 180:+6.1f}°  tilt {tilt:5.1f}°")


def draw_wind_zone(axes, lo, hi, wind, name):
    """Translucent box plus an arrow of the zone's wind (1 m of arrow per 1 m/s)."""
    if np.any(hi <= lo):
        return
    axes.bar3d(*lo, *(hi - lo), color="#5aa9d6", alpha=0.07, shade=False)
    center = (lo + hi) / 2
    speed = float(np.linalg.norm(wind))
    if speed > 0:
        axes.quiver(*(center - wind / 2), *wind, length=1.0, color="#2b7bb9", linewidth=2.2,
                    arrow_length_ratio=0.3)
    axes.text(*(center + [0, 0, 0.4 + max(0.0, -wind[2] / 2)]), f"{name}\n{speed:.1f} m/s",
              fontsize=7, color="#2b7bb9", ha="center")


def wind_zones_from_log(log) -> list:
    zones = log.get("scenario", {}).get("wind_zones", [])
    winds = log.get("realization", {}).get("zone_wind_m_s", {})
    return [(np.array(z["lower"]), np.array(z["upper"]), winds[z["name"]], z["name"])
            for z in zones if z["name"] in winds]


class RaceView:
    def __init__(self, track, *, live=False, attitude=None, wind_zones=()):
        """attitude: optional (times_s, euler_deg) arrays; adds a pose panel.
        wind_zones: optional (lower, upper, wind_m_s, name) tuples, drawn as boxes with the zone's wind."""
        import matplotlib.pyplot as plt
        self.plt, self.live = plt, live
        self.attitude = None
        if attitude is None:
            self.figure = plt.figure(figsize=(10, 6))
            self.axes = self.figure.add_subplot(111, projection="3d")
        else:
            self.figure = plt.figure(figsize=(15, 6.5))
            grid = self.figure.add_gridspec(2, 2, width_ratios=(1.8, 1), height_ratios=(1.25, 1),
                                            wspace=0.06, hspace=0.22)
            self.axes = self.figure.add_subplot(grid[:, 0], projection="3d")
            self.attitude = AttitudePanel(self.figure, grid[0, 1], grid[1, 1], *attitude)
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
        for zone_lo, zone_hi, wind, name in wind_zones:
            draw_wind_zone(ax, np.clip(zone_lo, lo, hi), np.clip(zone_hi, lo, hi), np.asarray(wind), name)
        ax.view_init(elev=27, azim=-65)
        ax.set_title("Course overview", fontsize=11, pad=5)
        ax.legend(loc="upper left", fontsize=9)
        if self.attitude is None:
            self.figure.subplots_adjust(left=0.02, right=0.96, bottom=0.03, top=0.87)
        else:
            self.figure.subplots_adjust(left=0.01, right=0.98, bottom=0.09, top=0.88)
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
        if self.attitude is not None:
            self.attitude.draw(self.last_item["time_s"], rotation)
        passed = self.last_item["next_gate"]
        for index, line in enumerate(self.gates):
            line.set_color("#438c55" if index < passed else "#276fa4" if index == passed else "#9ca7b0")
        lap = f" | lap {self.last_item['lap'] + 1}" if "lap" in self.last_item else ""
        self.title.set_text(f"Quadrotor racing | t = {self.last_item['time_s']:.2f} s{lap} | next gate: {passed}")

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
    samples = log["samples"]
    attitude = (np.array([item["time_s"] for item in samples]),
                euler_deg([item["quaternion_xyzw"] for item in samples]))
    view = RaceView(track_from_dict(log["config"]["track"]), attitude=attitude, wind_zones=wind_zones_from_log(log))
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
            return view.line, *view.drone.artists, *view.attitude.artists, view.title

        animation = FuncAnimation(view.figure, frame, frames=indices, interval=1000 / 12, blit=False)
        if output:
            Path(output).parent.mkdir(parents=True, exist_ok=True)
            animation.save(output, writer=PillowWriter(fps=12))
        if show:
            plt.show()
        return log["summary"]
    finally:
        view.close()
