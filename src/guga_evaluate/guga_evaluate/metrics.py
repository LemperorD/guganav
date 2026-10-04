"""Pure metric helpers used by the ROS evaluation node."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Optional, Sequence, Tuple


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def percentile(values: Sequence[float], percent: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = max(0.0, min(100.0, percent)) * (len(ordered) - 1) / 100.0
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


@dataclass
class SeriesStats:
    """Bounded scalar samples with convenient summary statistics."""

    max_samples: int = 1_000_000
    values: list[float] = field(default_factory=list)

    def add(self, value: float) -> None:
        if math.isfinite(value) and len(self.values) < self.max_samples:
            self.values.append(float(value))

    def summary(self) -> dict:
        if not self.values:
            return {"count": 0}
        count = len(self.values)
        mean = sum(self.values) / count
        rmse = math.sqrt(sum(value * value for value in self.values) / count)
        return {
            "count": count,
            "mean": mean,
            "rmse": rmse,
            "min": min(self.values),
            "max": max(self.values),
            "p50": percentile(self.values, 50.0),
            "p95": percentile(self.values, 95.0),
            "p99": percentile(self.values, 99.0),
        }


@dataclass
class TopicTiming:
    count: int = 0
    first_receipt: Optional[float] = None
    last_receipt: Optional[float] = None
    previous_receipt: Optional[float] = None
    intervals: SeriesStats = field(default_factory=SeriesStats)
    latencies: SeriesStats = field(default_factory=SeriesStats)

    def observe(self, receipt_time: float,
                header_time: Optional[float] = None) -> None:
        self.count += 1
        if self.first_receipt is None:
            self.first_receipt = receipt_time
        if self.previous_receipt is not None:
            interval = receipt_time - self.previous_receipt
            if interval >= 0.0:
                self.intervals.add(interval)
        self.previous_receipt = receipt_time
        self.last_receipt = receipt_time
        if header_time is not None and header_time > 0.0:
            latency = receipt_time - header_time
            # Different clock domains can produce nonsense. Keep only plausible
            # non-negative samples.
            if 0.0 <= latency <= 60.0:
                self.latencies.add(latency)

    def summary(self) -> dict:
        duration = 0.0
        if self.first_receipt is not None and self.last_receipt is not None:
            duration = max(0.0, self.last_receipt - self.first_receipt)
        frequency = (self.count - 1) / duration if duration > 0.0 else 0.0
        return {
            "count": self.count,
            "duration_sec": duration,
            "effective_hz": frequency,
            "interval_sec": self.intervals.summary(),
            "latency_sec": self.latencies.summary(),
        }


def path_length(points: Sequence[Tuple[float, float]]) -> float:
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
    )


def path_curvatures(points: Sequence[Tuple[float, float]]) -> list[float]:
    """Return unsigned three-point curvature samples."""
    curvatures: list[float] = []
    for first, middle, last in zip(points, points[1:], points[2:]):
        a = math.hypot(middle[0] - first[0], middle[1] - first[1])
        b = math.hypot(last[0] - middle[0], last[1] - middle[1])
        c = math.hypot(last[0] - first[0], last[1] - first[1])
        denominator = a * b * c
        if denominator <= 1e-12:
            continue
        cross = abs(
            (middle[0] - first[0]) * (last[1] - first[1])
            - (middle[1] - first[1]) * (last[0] - first[0])
        )
        curvatures.append(2.0 * cross / denominator)
    return curvatures


def nearest_path_error(
    x: float,
    y: float,
    yaw: float,
    points: Sequence[Tuple[float, float]],
) -> Optional[Tuple[float, float, float]]:
    """Return cross-track error, heading error and normalized progress."""
    if len(points) < 2:
        return None

    total_length = path_length(points)
    traversed = 0.0
    best_distance = math.inf
    best_heading_error = 0.0
    best_progress_distance = 0.0

    for first, last in zip(points, points[1:]):
        dx = last[0] - first[0]
        dy = last[1] - first[1]
        length_sq = dx * dx + dy * dy
        segment_length = math.sqrt(length_sq)
        if length_sq <= 1e-12:
            continue
        projection = ((x - first[0]) * dx + (y - first[1]) * dy) / length_sq
        projection = max(0.0, min(1.0, projection))
        nearest_x = first[0] + projection * dx
        nearest_y = first[1] + projection * dy
        distance = math.hypot(x - nearest_x, y - nearest_y)
        if distance < best_distance:
            best_distance = distance
            path_heading = math.atan2(dy, dx)
            best_heading_error = wrap_angle(yaw - path_heading)
            best_progress_distance = traversed + projection * segment_length
        traversed += segment_length

    if not math.isfinite(best_distance):
        return None
    progress = best_progress_distance / total_length if total_length > 0.0 else 0.0
    return best_distance, best_heading_error, progress


def points_from_xy(items: Iterable[object]) -> list[Tuple[float, float]]:
    return [(float(item[0]), float(item[1])) for item in items]
