from __future__ import annotations

from pathlib import Path
from typing import Iterable, Tuple

import ezdxf
import numpy as np

from fabscan.image_processing import FoundContour


def image_points_to_dxf_points(
    points: np.ndarray,
    scale_inches_per_pixel: float,
    image_height_pixels: int,
) -> Iterable[Tuple[float, float]]:
    """Convert image pixel coordinates to DXF inch coordinates.

    Image coordinates have Y increasing downward. DXF/CAD coordinates usually have
    Y increasing upward, so we flip Y during export.
    """

    for x_px, y_px in points:
        x_in = float(x_px) * scale_inches_per_pixel
        y_in = float(image_height_pixels - y_px) * scale_inches_per_pixel
        yield (x_in, y_in)


def export_contours_to_dxf(
    contours: list[FoundContour],
    output_path: str | Path,
    scale_inches_per_pixel: float,
    image_height_pixels: int,
) -> Path:
    """Export contours to a simple polyline DXF."""

    if scale_inches_per_pixel <= 0:
        raise ValueError("scale_inches_per_pixel must be greater than zero")

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    doc = ezdxf.new("R2010")
    doc.units = ezdxf.units.IN
    msp = doc.modelspace()

    for layer_name in ("OUTSIDE", "INSIDE", "REFERENCE"):
        if layer_name not in doc.layers:
            doc.layers.new(name=layer_name)

    for contour in contours:
        dxf_points = list(
            image_points_to_dxf_points(
                points=contour.points,
                scale_inches_per_pixel=scale_inches_per_pixel,
                image_height_pixels=image_height_pixels,
            )
        )
        if len(dxf_points) >= 3:
            msp.add_lwpolyline(dxf_points, close=True, dxfattribs={"layer": contour.layer})

    doc.saveas(output_path)
    return output_path
