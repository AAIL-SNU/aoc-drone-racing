"""Pure geometric rules, independent of the dynamics library and renderer."""

from dataclasses import dataclass
from functools import singledispatch
import numpy as np

from .api import Box, Gate, Track


def gate_local(point, gate: Gate) -> np.ndarray:
    c, s = np.cos(gate.yaw_rad), np.sin(gate.yaw_rad)
    delta = np.asarray(point) - gate.center
    return np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1], delta[2]])


def crossing_fraction(start, end, gate: Gate, radius: float, *, forward=True):
    """Fraction of a signed plane crossing through the sphere-eroded opening."""
    a, b = gate_local(start, gate), gate_local(end, gate)
    crosses = a[0] < 0 <= b[0] if forward else a[0] > 0 >= b[0]
    if not crosses:
        return None
    alpha = -a[0] / (b[0] - a[0])
    hit = a + alpha * (b - a)
    if abs(hit[1]) < gate.width_m / 2 - radius and abs(hit[2]) < gate.height_m / 2 - radius:
        return float(alpha)
    return None


def segment_box_fraction(start, end, box: Box, radius: float = 0.0):
    """Slab intersection with a radius-expanded AABB (conservative sphere sweep)."""
    a, b = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
    lo, hi = np.asarray(box.lower) - radius, np.asarray(box.upper) + radius
    enter, leave = 0.0, 1.0
    for x, d, low, high in zip(a, b - a, lo, hi):
        if abs(d) < 1e-14:
            if x < low or x > high:
                return None
        else:
            u, v = sorted(((low - x) / d, (high - x) / d))
            enter, leave = max(enter, u), min(leave, v)
            if enter > leave:
                return None
    return float(enter)


@singledispatch
def obstacle_fraction(obstacle, start, end, radius: float):
    """First contact fraction of the radius-swept segment with an obstacle, or None."""
    raise TypeError(f"No collision rule for obstacle type {type(obstacle).__name__}")


@obstacle_fraction.register
def _(obstacle: Box, start, end, radius: float):
    return segment_box_fraction(start, end, obstacle, radius)


def frame_boxes(gate: Gate) -> tuple[Box, ...]:
    """Four frame bars in gate-local coordinates; width/height describe the opening."""
    w, h, f = gate.width_m / 2, gate.height_m / 2, gate.frame_m
    return (
        Box((-f / 2, -w - f, -h - f), (f / 2, -w, h + f)),
        Box((-f / 2, w, -h - f), (f / 2, w + f, h + f)),
        Box((-f / 2, -w, -h - f), (f / 2, w, -h)),
        Box((-f / 2, -w, h), (f / 2, w, h + f)),
    )


@dataclass(frozen=True)
class RuleEvent:
    kind: str
    fraction: float
    gate: int | None = None


class RaceRules:
    def __init__(self, track: Track, radius: float):
        self.track, self.radius = track, radius
        self.next_gate = 0

    def inspect(self, start, end) -> list[RuleEvent]:
        """Order crossings and contacts; later events cannot overtake a collision."""
        candidates = []
        for index, gate in enumerate(self.track.gates):
            a, b = gate_local(start, gate), gate_local(end, gate)
            for box in frame_boxes(gate):
                alpha = segment_box_fraction(a, b, box, self.radius)
                if alpha is not None:
                    candidates.append(RuleEvent("collision", alpha, index))
            for forward, kind in ((True, "forward"), (False, "reverse_gate")):
                alpha = crossing_fraction(start, end, gate, self.radius, forward=forward)
                if alpha is not None:
                    candidates.append(RuleEvent(kind, alpha, index))
        for obstacle in self.track.obstacles:
            alpha = obstacle_fraction(obstacle, start, end, self.radius)
            if alpha is not None:
                candidates.append(RuleEvent("collision", alpha))
        lo = np.asarray(self.track.bounds.lower) + self.radius
        hi = np.asarray(self.track.bounds.upper) - self.radius
        a, b = np.asarray(start), np.asarray(end)
        if np.any(a <= lo) or np.any(a >= hi):
            candidates.append(RuleEvent("out_of_bounds", 0.0))
        elif np.any(b <= lo) or np.any(b >= hi):
            fractions = [(bound - a[j]) / (b[j] - a[j])
                         for j in range(3) for bound in (lo[j], hi[j])
                         if b[j] != a[j] and 0 <= (bound - a[j]) / (b[j] - a[j]) <= 1]
            candidates.append(RuleEvent("out_of_bounds", min(fractions)))
        events = []
        for event in sorted(candidates, key=lambda e: (e.fraction, e.kind != "collision")):
            if event.kind == "forward":
                if event.gate == self.next_gate:
                    events.append(RuleEvent("gate_passed", event.fraction, event.gate))
                    self.next_gate += 1
                    if self.next_gate == len(self.track.gates):
                        events.append(RuleEvent("finished", event.fraction, event.gate))
                        break
                elif event.gate > self.next_gate:
                    events.append(RuleEvent("skipped_gate", event.fraction, event.gate))
                    break
            elif event.kind == "reverse_gate":
                # Reversing a previously completed gate is allowed, but never awards progress.
                if event.gate >= self.next_gate:
                    events.append(event)
                    break
            else:
                events.append(event)
                break
        return events
