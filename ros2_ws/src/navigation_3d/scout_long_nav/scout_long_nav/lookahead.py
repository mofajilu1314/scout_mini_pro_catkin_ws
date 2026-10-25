# -*- coding: utf-8 -*-
"""Pure helpers for Arena look-ahead and goal bridging (offline-testable)."""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

Point2 = Tuple[float, float]


def path_length(points: Sequence[Point2]) -> float:
    total = 0.0
    for i in range(1, len(points)):
        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]
        total += math.hypot(dx, dy)
    return total


def closest_point_on_polyline(
    points: Sequence[Point2], x: float, y: float
) -> Tuple[int, float, Point2]:
    """Return (seg_index, t in [0,1], closest_xy) on polyline."""
    if not points:
        raise ValueError("empty path")
    if len(points) == 1:
        return 0, 0.0, (points[0][0], points[0][1])

    best_d2 = float("inf")
    best = (0, 0.0, (points[0][0], points[0][1]))
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        vx, vy = x1 - x0, y1 - y0
        seg_len2 = vx * vx + vy * vy
        if seg_len2 < 1e-12:
            t = 0.0
            cx, cy = x0, y0
        else:
            t = max(0.0, min(1.0, ((x - x0) * vx + (y - y0) * vy) / seg_len2))
            cx = x0 + t * vx
            cy = y0 + t * vy
        d2 = (x - cx) ** 2 + (y - cy) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best = (i, t, (cx, cy))
    return best


def lookahead_point(
    points: Sequence[Point2],
    x: float,
    y: float,
    lookahead_m: float,
) -> Optional[Point2]:
    """
    Arena-style spatial horizon: from the closest point on the path, advance
    approximately lookahead_m along the polyline. If remaining length is shorter,
    return the final waypoint.
    """
    if not points:
        return None
    if len(points) == 1:
        return (points[0][0], points[0][1])

    seg_i, t, _ = closest_point_on_polyline(points, x, y)
    # Remaining distance from projection to end
    x0, y0 = points[seg_i]
    x1, y1 = points[seg_i + 1]
    cx = x0 + t * (x1 - x0)
    cy = y0 + t * (y1 - y0)

    remaining = lookahead_m
    # First partial segment
    first_len = math.hypot(x1 - cx, y1 - cy)
    if first_len >= remaining or seg_i == len(points) - 2 and first_len >= remaining - 1e-9:
        if first_len < 1e-12:
            return (x1, y1)
        ratio = remaining / first_len
        return (cx + ratio * (x1 - cx), cy + ratio * (y1 - cy))

    remaining -= first_len
    for i in range(seg_i + 1, len(points) - 1):
        ax, ay = points[i]
        bx, by = points[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        if seg >= remaining:
            if seg < 1e-12:
                return (bx, by)
            ratio = remaining / seg
            return (ax + ratio * (bx - ax), ay + ratio * (by - ay))
        remaining -= seg
    return (points[-1][0], points[-1][1])


def lookahead_waypoints(
    points: Sequence[Point2],
    x: float,
    y: float,
    lookahead_m: float,
    spacing_m: float = 1.0,
) -> List[Point2]:
    """Sample points from closest projection along path up to lookahead horizon.

    Includes start (closest on path) and the final lookahead point. For Arena viz.
    """
    goal = lookahead_point(points, x, y, lookahead_m)
    if goal is None or not points:
        return []
    if len(points) == 1:
        return [(float(points[0][0]), float(points[0][1]))]

    seg_i, t, _ = closest_point_on_polyline(points, x, y)
    x0, y0 = points[seg_i]
    x1, y1 = points[seg_i + 1]
    cx = x0 + t * (x1 - x0)
    cy = y0 + t * (y1 - y0)

    chain: List[Point2] = [(cx, cy), (x1, y1)]
    for i in range(seg_i + 1, len(points) - 1):
        chain.append(points[i + 1])

    spacing = max(float(spacing_m), 0.2)
    horizon = max(float(lookahead_m), 0.0)
    samples: List[Point2] = [(cx, cy)]
    dist_from_start = 0.0
    next_sample = spacing

    for i in range(len(chain) - 1):
        ax, ay = chain[i]
        bx, by = chain[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        if seg < 1e-12:
            continue
        usable = min(seg, horizon - dist_from_start)
        if usable <= 0.0:
            break
        while next_sample <= dist_from_start + usable + 1e-9:
            along = next_sample - dist_from_start
            r = along / seg
            samples.append((ax + r * (bx - ax), ay + r * (by - ay)))
            next_sample += spacing
        dist_from_start += usable
        if dist_from_start >= horizon - 1e-9:
            break

    gx, gy = float(goal[0]), float(goal[1])
    if math.hypot(samples[-1][0] - gx, samples[-1][1] - gy) > 0.15:
        samples.append((gx, gy))
    else:
        samples[-1] = (gx, gy)
    return samples


def pose_to_point_xy(x: float, y: float) -> Point2:
    return (float(x), float(y))


def polyline_from_xy(xs: Sequence[float], ys: Sequence[float]) -> List[Point2]:
    if len(xs) != len(ys):
        raise ValueError("xs/ys length mismatch")
    return [(float(a), float(b)) for a, b in zip(xs, ys)]
