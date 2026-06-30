from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


Point = Tuple[float, float]


@dataclass
class ScaleResult:
    pixels: float
    inches: float
    inches_per_pixel: float


def calculate_scale(point_a: Point, point_b: Point, known_inches: float) -> ScaleResult:
    """Calculate image scale from two image points and a known real distance."""

    if known_inches <= 0:
        raise ValueError("known_inches must be greater than zero")

    dx = point_b[0] - point_a[0]
    dy = point_b[1] - point_a[1]
    pixel_distance = math.hypot(dx, dy)

    if pixel_distance <= 0:
        raise ValueError("scale points must not be identical")

    return ScaleResult(
        pixels=pixel_distance,
        inches=known_inches,
        inches_per_pixel=known_inches / pixel_distance,
    )
