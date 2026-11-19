"""Static picture of a track: top view, 3D view and altitude profile."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Rectangle
import numpy as np

from .api import Box, Track

GATE_COLOR = "#253744"
PATH_CMAP = "viridis"


def _course_points(track: Track) -> np.ndarray:
    """Start, then every gate center, then gate 1 again: the leg flown on each later lap."""
    centers = [g.center for g in track.gates]
    return np.array([track.start, *centers, centers[0]], dtype=float)


def _gate_frame(gate) -> tuple[np.ndarray, np.ndarray]:
    """Unit pass direction (gate +x) and unit opening direction (gate +y)."""
    c, s = np.cos(gate.yaw_rad), np.sin(gate.yaw_rad)
    return np.array([c, s, 0.0]), np.array([-s, c, 0.0])


def _colored_path(points: np.ndarray, norm, linewidth=2.2) -> LineCollection:
    segments = np.stack((points[:-1, :2], points[1:, :2]), axis=1)
    lines = LineCollection(segments, cmap=PATH_CMAP, norm=norm, linewidth=linewidth, zorder=2)
    lines.set_array(0.5 * (points[:-1, 2] + points[1:, 2]))
    return lines


def _draw_top(ax, track: Track, points: np.ndarray, norm):
    lo, hi = np.array(track.bounds.lower), np.array(track.bounds.upper)
    ax.add_patch(Rectangle(lo[:2], *(hi - lo)[:2], fill=False, linestyle="--", color="#9aa5ad",
                           linewidth=1.0))
    for obstacle in track.obstacles:
        if isinstance(obstacle, Box):
            o_lo, o_hi = np.array(obstacle.lower), np.array(obstacle.upper)
            ax.add_patch(Rectangle(o_lo[:2], *(o_hi - o_lo)[:2], color="#ba5d48", alpha=0.5, zorder=1))
    ax.plot(*points[:2, :2].T, color="#9aa5ad", linestyle=":", linewidth=1.4, zorder=1)
    path = _colored_path(points[1:], norm)
    ax.add_collection(path)
    placed = [np.array(g.center[:2]) for g in track.gates]
    for index, gate in enumerate(track.gates):
        forward, across = _gate_frame(gate)
        center = np.array(gate.center)
        ends = np.array([center - across * gate.width_m / 2, center + across * gate.width_m / 2])
        ax.plot(*ends[:, :2].T, color=GATE_COLOR, linewidth=4, solid_capstyle="butt", zorder=3)
        ax.annotate("", xy=center[:2] + 0.9 * forward[:2], xytext=center[:2],
                    arrowprops=dict(arrowstyle="-|>", color=GATE_COLOR, lw=1.2), zorder=3)
        # Beside the frame (off the path), on whichever side is farther from other gates and labels.
        sides = [center[:2] + sign * (gate.width_m / 2 + 0.9) * across[:2] for sign in (1, -1)]
        label = max(sides, key=lambda p: min(np.linalg.norm(p - q) for q in placed))
        placed.append(label)
        ax.text(*label, str(index + 1), ha="center", va="center", fontsize=9, fontweight="bold",
                color=GATE_COLOR, zorder=4,
                bbox=dict(boxstyle="circle,pad=0.2", facecolor="white", edgecolor=GATE_COLOR, linewidth=0.8))
    ax.plot(*track.start[:2], marker="*", markersize=14, color="#d4483b", zorder=5, linestyle="none")
    margin = 1.0
    ax.set(xlim=(lo[0] - margin, hi[0] + margin), ylim=(lo[1] - margin, hi[1] + margin),
           xlabel="x [m]", ylabel="y [m]", title="Top view (★ start, arrows: pass direction, dashed: arena)")
    ax.set_aspect("equal")
    ax.grid(True, color="#e3e7ea", linewidth=0.6)
    return path


def _draw_3d(ax, track: Track, points: np.ndarray, norm):
    cmap = plt.get_cmap(PATH_CMAP)
    ax.plot(*points[:2].T, color="#9aa5ad", linestyle=":", linewidth=1.2)
    for a, b in zip(points[1:-1], points[2:]):
        ax.plot(*np.stack((a, b)).T, color=cmap(norm(0.5 * (a[2] + b[2]))), linewidth=2)
    for index, gate in enumerate(track.gates):
        forward, across = _gate_frame(gate)
        up = np.array([0.0, 0.0, 1.0])
        w, h = gate.width_m / 2, gate.height_m / 2
        corners = np.array([-w * across - h * up, w * across - h * up, w * across + h * up,
                            -w * across + h * up, -w * across - h * up]) + gate.center
        ax.plot(*corners.T, color=GATE_COLOR, linewidth=1.8)
        ax.text(*(np.array(gate.center) + (0, 0, h + 0.3)), str(index + 1), fontsize=8, ha="center",
                color=GATE_COLOR)
    for obstacle in track.obstacles:
        if isinstance(obstacle, Box):
            o_lo, o_hi = np.array(obstacle.lower), np.array(obstacle.upper)
            ax.bar3d(*o_lo, *(o_hi - o_lo), color="#ba5d48", alpha=0.5)
    ax.scatter(*track.start, marker="*", s=120, color="#d4483b", depthshade=False)
    lo, hi = np.array(track.bounds.lower), np.array(track.bounds.upper)
    ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), zlim=(lo[2], hi[2]),
           xlabel="x [m]", ylabel="y [m]", zlabel="z [m]", title="3D view (height ×2)")
    ax.set_zticks(np.linspace(lo[2], hi[2], 3))
    ax.set_box_aspect(np.maximum((hi - lo) * (1, 1, 2), 0.1))
    ax.view_init(elev=28, azim=-60)


def _draw_profile(ax, track: Track, points: np.ndarray):
    distance = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))))
    lo, hi = track.bounds.lower[2], track.bounds.upper[2]
    ax.axhspan(lo - 1, lo, color="#d9d2c5")
    ax.axhline(hi, color="#9aa5ad", linestyle="--", linewidth=1.0)
    ax.plot(distance[:2], points[:2, 2], color="#9aa5ad", linestyle=":", linewidth=1.4)
    ax.plot(distance[1:], points[1:, 2], color="#23738b", linewidth=1.8)
    gate_distance = distance[1:-1]
    for index, (d, gate) in enumerate(zip(gate_distance, track.gates)):
        z, h = gate.center[2], gate.height_m / 2
        ax.plot([d, d], [z - h, z + h], color=GATE_COLOR, linewidth=4, solid_capstyle="butt")
        ax.text(d, z + h + 0.15, str(index + 1), ha="center", va="bottom", fontsize=8, color=GATE_COLOR)
    ax.plot(distance[-1], points[-1, 2], marker="o", color=GATE_COLOR, markersize=4)
    ax.plot(0.0, points[0, 2], marker="*", markersize=12, color="#d4483b", linestyle="none")
    ax.set(xlim=(-1, distance[-1] + 1), ylim=(lo - 0.3, hi + 0.5), xlabel="distance along straight legs [m]",
           ylabel="z [m]", title=f"Altitude profile (bars: gate openings; one lap ≈ {distance[-1] - distance[1]:.0f} m)")
    ax.grid(True, color="#e3e7ea", linewidth=0.6)


def plot_track(track: Track):
    """Figure of a track: top view with pass directions, 3D view and altitude profile."""
    points = _course_points(track)
    norm = plt.Normalize(track.bounds.lower[2], track.bounds.upper[2])
    figure = plt.figure(figsize=(15, 9.5))
    grid = figure.add_gridspec(2, 2, width_ratios=(1.2, 1), height_ratios=(1.5, 1), wspace=0.12, hspace=0.28)
    top = figure.add_subplot(grid[0, 0])
    path = _draw_top(top, track, points, norm)
    figure.colorbar(path, ax=top, shrink=0.8, pad=0.02, label="leg altitude z [m]")
    _draw_3d(figure.add_subplot(grid[0, 1], projection="3d"), track, points, norm)
    _draw_profile(figure.add_subplot(grid[1, :]), track, points)
    figure.suptitle(f"{track.name}: {len(track.gates)} gates, {len(track.obstacles)} obstacle(s)",
                    fontsize=15, y=0.97)
    return figure


def save_track_plot(track: Track, output: str | Path) -> Path:
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure = plot_track(track)
    figure.savefig(output, dpi=120, bbox_inches="tight")
    plt.close(figure)
    return output
