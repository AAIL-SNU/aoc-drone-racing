"""Pure geometric rules, independent of the dynamics library and renderer."""

from dataclasses import dataclass
from functools import singledispatch
import numpy as np

from .api import Box, Gate, Track


def gate_local(point, gate: Gate) -> np.ndarray:
    c, s = np.cos(gate.yaw_rad), np.sin(gate.yaw_rad)
    delta = np.asarray(point) - gate.center
    return np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1], delta[2]])


def plane_crossing(start, end, gate: Gate, *, forward=True):
    """(fraction, gate-local hit point) of a signed crossing of the gate plane, or None."""
    a, b = gate_local(start, gate), gate_local(end, gate)
    crosses = a[0] < 0 <= b[0] if forward else a[0] > 0 >= b[0]
    if not crosses:
        return None
    alpha = -a[0] / (b[0] - a[0])
    return float(alpha), a + alpha * (b - a)


def inside_opening(hit, gate: Gate, radius: float, margin: float = 0.0) -> bool:
    return abs(hit[1]) < gate.width_m / 2 - radius + margin and abs(hit[2]) < gate.height_m / 2 - radius + margin


def crossing_fraction(start, end, gate: Gate, radius: float, *, forward=True):
    """Fraction of a signed plane crossing through the sphere-eroded opening."""
    crossing = plane_crossing(start, end, gate, forward=forward)
    if crossing is not None and inside_opening(crossing[1], gate, radius):
        return crossing[0]
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
    lap: int | None = None


# Crossing the next gate's plane forward, outside the opening but at most this far
# beyond its edge, resolves that gate as missed. A gate flown around entirely is
# resolved as missed when the drone passes a later gate.
MISS_WINDOW_M = 2.0
TERMINAL = {"collision", "out_of_bounds", "finished"}


class RaceRules:
    """Gate order over several laps.

    Event kinds: ``gate_passed`` and ``gate_missed`` (each resolves one gate),
    ``lap_completed``, ``wrong_way`` (an opening crossed backwards; scored as an
    infraction, never as progress), and the terminal ``finished``, ``collision`` and
    ``out_of_bounds``. Every event carries the lap it happened in.
    """

    def __init__(self, track: Track, radius: float, laps: int = 1):
        self.track, self.radius, self.laps = track, radius, laps
        self.count = len(track.gates)
        self.progress = 0          # gates resolved (passed or missed) since the start

    @property
    def next_gate(self) -> int:
        return self.progress % self.count

    @property
    def lap(self) -> int:
        return min(self.progress // self.count, self.laps - 1)

    @property
    def finished(self) -> bool:
        return self.progress >= self.laps * self.count

    def _candidates(self, start, end):
        candidates = []
        for index, gate in enumerate(self.track.gates):
            a, b = gate_local(start, gate), gate_local(end, gate)
            for box in frame_boxes(gate):
                alpha = segment_box_fraction(a, b, box, self.radius)
                if alpha is not None:
                    candidates.append(RuleEvent("collision", alpha, index))
            forward = plane_crossing(start, end, gate, forward=True)
            if forward is not None:
                if inside_opening(forward[1], gate, self.radius):
                    candidates.append(RuleEvent("forward", forward[0], index))
                elif inside_opening(forward[1], gate, self.radius, MISS_WINDOW_M):
                    candidates.append(RuleEvent("beside", forward[0], index))
            backward = plane_crossing(start, end, gate, forward=False)
            if backward is not None and inside_opening(backward[1], gate, self.radius):
                candidates.append(RuleEvent("wrong_way", backward[0], index))
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
        return sorted(candidates, key=lambda e: (e.fraction, e.kind != "collision"))

    def _resolve(self, kind: str, fraction: float, events: list) -> None:
        lap = self.lap
        events.append(RuleEvent(kind, fraction, self.next_gate, lap))
        self.progress += 1
        if self.progress % self.count == 0:
            events.append(RuleEvent("lap_completed", fraction, None, lap))
        if self.finished:
            events.append(RuleEvent("finished", fraction, None, lap))

    def inspect(self, start, end) -> list[RuleEvent]:
        """Events of one straight motion segment, in order; nothing follows a terminal event."""
        events = []
        for event in self._candidates(start, end):
            if event.kind in ("collision", "out_of_bounds"):
                events.append(RuleEvent(event.kind, event.fraction, event.gate, self.lap))
            elif event.kind == "wrong_way":
                events.append(RuleEvent("wrong_way", event.fraction, event.gate, self.lap))
            elif event.kind == "beside" and event.gate == self.next_gate:
                self._resolve("gate_missed", event.fraction, events)
            elif event.kind == "forward":
                ahead = (event.gate - self.next_gate) % self.count
                if ahead > self.count // 2:
                    continue          # a gate resolved recently, flown through again
                for _ in range(ahead):
                    if not self.finished:
                        self._resolve("gate_missed", event.fraction, events)
                if not self.finished:
                    self._resolve("gate_passed", event.fraction, events)
            if events and events[-1].kind in TERMINAL:
                break
        return events
