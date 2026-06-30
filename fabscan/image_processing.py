from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import cv2
import numpy as np


@dataclass
class FoundContour:
    """A contour found in the thresholded image."""

    points: np.ndarray  # Shape: (N, 2), pixel coordinates
    area: float
    layer: str
    parent_index: int


@dataclass
class ProcessedImage:
    """Intermediate processing results used by the UI preview."""

    threshold_image: np.ndarray
    contours: List[FoundContour]


def threshold_image(
    image_bgr: np.ndarray,
    threshold_value: int = 127,
    blur_size: int = 3,
    invert: bool = False,
) -> np.ndarray:
    """Convert a BGR image to a clean black/white threshold image.

    OpenCV contour detection works best when the part/profile is white on a black
    background. Use `invert=True` when the part shows up dark on a light background.
    """

    if image_bgr is None:
        raise ValueError("image_bgr cannot be None")

    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)

    # Gaussian blur needs an odd kernel size. A value of 0/1 means no useful blur.
    blur_size = max(1, int(blur_size))
    if blur_size % 2 == 0:
        blur_size += 1

    if blur_size > 1:
        gray = cv2.GaussianBlur(gray, (blur_size, blur_size), 0)

    mode = cv2.THRESH_BINARY_INV if invert else cv2.THRESH_BINARY
    _, binary = cv2.threshold(gray, int(threshold_value), 255, mode)
    return binary


def find_contours(
    image_bgr: np.ndarray,
    threshold_value: int = 127,
    blur_size: int = 3,
    invert: bool = False,
    min_area: float = 100.0,
    simplify_percent: float = 0.20,
) -> ProcessedImage:
    """Find simplified contours suitable for a rough DXF export.

    `simplify_percent` is the epsilon percentage of each contour perimeter used by
    cv2.approxPolyDP. Higher values make fewer points and rougher geometry.
    """

    binary = threshold_image(
        image_bgr=image_bgr,
        threshold_value=threshold_value,
        blur_size=blur_size,
        invert=invert,
    )

    contours, hierarchy = cv2.findContours(binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    found: List[FoundContour] = []

    if hierarchy is None:
        return ProcessedImage(threshold_image=binary, contours=found)

    hierarchy = hierarchy[0]
    for index, contour in enumerate(contours):
        area = float(abs(cv2.contourArea(contour)))
        if area < float(min_area):
            continue

        perimeter = cv2.arcLength(contour, True)
        epsilon = perimeter * (float(simplify_percent) / 100.0)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # OpenCV stores contours as (N, 1, 2). Flatten to (N, 2).
        points = approx.reshape(-1, 2).astype(float)
        if len(points) < 3:
            continue

        parent_index = int(hierarchy[index][3])
        layer = "OUTSIDE" if parent_index == -1 else "INSIDE"

        found.append(
            FoundContour(
                points=points,
                area=area,
                layer=layer,
                parent_index=parent_index,
            )
        )

    # Largest contours first. This makes the UI status easier to understand.
    found.sort(key=lambda c: c.area, reverse=True)
    return ProcessedImage(threshold_image=binary, contours=found)
