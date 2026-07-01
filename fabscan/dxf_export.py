from __future__ import annotations

from pathlib import Path
from typing import Iterable, Literal, Optional, Tuple

import ezdxf
import numpy as np

from fabscan.image_processing import FoundContour


ExportOriginMode = Literal["preserve", "lower_left", "center"]


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


def _validate_export_options(
    scale_inches_per_pixel: float,
    origin_mode: ExportOriginMode,
    margin_inches: float,
) -> None:
    if scale_inches_per_pixel <= 0:
        raise ValueError("scale_inches_per_pixel must be greater than zero")

    if origin_mode not in ("preserve", "lower_left", "center"):
        raise ValueError(f"Unsupported origin_mode: {origin_mode}")

    if margin_inches < 0:
        raise ValueError("margin_inches must be zero or greater")


def _bbox_from_point_groups(
    point_groups: Iterable[list[Tuple[float, float]]],
) -> Optional[Tuple[float, float, float, float, float, float]]:
    all_points = [point for group in point_groups for point in group]
    if not all_points:
        return None

    xs = [point[0] for point in all_points]
    ys = [point[1] for point in all_points]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)
    return min_x, min_y, max_x, max_y, max_x - min_x, max_y - min_y


def contours_to_dxf_point_groups(
    contours: list[FoundContour],
    scale_inches_per_pixel: float,
    image_height_pixels: int,
    origin_mode: ExportOriginMode = "preserve",
    margin_inches: float = 0.0,
) -> list[tuple[FoundContour, list[Tuple[float, float]]]]:
    """Convert enabled contours into DXF point groups with optional origin move.

    origin_mode values:
    - preserve: keep the image-derived CAD position.
    - lower_left: move enabled geometry lower-left bbox to margin,margin.
    - center: move enabled geometry bbox center to 0,0. Margin is ignored.
    """

    _validate_export_options(scale_inches_per_pixel, origin_mode, margin_inches)

    converted: list[tuple[FoundContour, list[Tuple[float, float]]]] = []
    for contour in contours:
        if not contour.enabled:
            continue

        dxf_points = list(
            image_points_to_dxf_points(
                points=contour.points,
                scale_inches_per_pixel=scale_inches_per_pixel,
                image_height_pixels=image_height_pixels,
            )
        )
        if len(dxf_points) >= 3:
            converted.append((contour, dxf_points))

    bbox = _bbox_from_point_groups(points for _contour, points in converted)
    if bbox is None:
        return converted

    min_x, min_y, max_x, max_y, _width, _height = bbox
    offset_x = 0.0
    offset_y = 0.0

    if origin_mode == "lower_left":
        offset_x = margin_inches - min_x
        offset_y = margin_inches - min_y
    elif origin_mode == "center":
        offset_x = -((min_x + max_x) / 2.0)
        offset_y = -((min_y + max_y) / 2.0)

    if offset_x == 0.0 and offset_y == 0.0:
        return converted

    shifted: list[tuple[FoundContour, list[Tuple[float, float]]]] = []
    for contour, points in converted:
        shifted.append((contour, [(x + offset_x, y + offset_y) for x, y in points]))

    return shifted


def get_export_bbox_for_contours(
    contours: list[FoundContour],
    scale_inches_per_pixel: float,
    image_height_pixels: int,
    origin_mode: ExportOriginMode = "preserve",
    margin_inches: float = 0.0,
) -> Optional[Tuple[float, float, float, float, float, float]]:
    """Return DXF output bbox as min_x, min_y, max_x, max_y, width, height in inches."""

    converted = contours_to_dxf_point_groups(
        contours=contours,
        scale_inches_per_pixel=scale_inches_per_pixel,
        image_height_pixels=image_height_pixels,
        origin_mode=origin_mode,
        margin_inches=margin_inches,
    )
    return _bbox_from_point_groups(points for _contour, points in converted)


def export_contours_to_dxf(
    contours: list[FoundContour],
    output_path: str | Path,
    scale_inches_per_pixel: float,
    image_height_pixels: int,
    origin_mode: ExportOriginMode = "preserve",
    margin_inches: float = 0.0,
) -> Path:
    """Export enabled contours to a simple polyline DXF."""

    _validate_export_options(scale_inches_per_pixel, origin_mode, margin_inches)

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    doc = ezdxf.new("R2010")
    doc.units = ezdxf.units.IN
    msp = doc.modelspace()

    for layer_name in ("OUTSIDE", "INSIDE", "REFERENCE"):
        if layer_name not in doc.layers:
            doc.layers.new(name=layer_name)

    converted = contours_to_dxf_point_groups(
        contours=contours,
        scale_inches_per_pixel=scale_inches_per_pixel,
        image_height_pixels=image_height_pixels,
        origin_mode=origin_mode,
        margin_inches=margin_inches,
    )

    for contour, dxf_points in converted:
        msp.add_lwpolyline(dxf_points, close=True, dxfattribs={"layer": contour.layer})

    doc.saveas(output_path)
    return output_path


def export_trace_points_to_dxf(
    points: list[Tuple[float, float, float]] | list[Tuple[float, float]],
    output_path: str | Path,
    close: bool = True,
    layer_name: str = "TRACE",
) -> Path:
    """Export manually captured CNC XY trace points to a simple DXF polyline.

    The points are expected to already be in machine/work units. FabScan sets
    the DXF units to inches because the target plasma workflow is inch-based.
    Z is accepted for display/capture history but ignored for 2D DXF export.
    """

    return export_trace_groups_to_dxf([points], output_path=output_path, close=close, layer_name=layer_name)


def _normalize_angle_degrees(angle: float) -> float:
    return float(angle) % 360.0


def _entity_type(entity: object | None) -> str:
    if isinstance(entity, dict):
        return str(entity.get("type", "polyline"))
    return "polyline"


def _xy_point(point: Tuple[float, float, float] | Tuple[float, float]) -> Tuple[float, float]:
    return (float(point[0]), float(point[1]))


def _add_trace_entity_to_modelspace(
    msp,
    group: list[Tuple[float, float, float]] | list[Tuple[float, float]],
    entity: object | None,
    close: bool,
    layer_name: str,
) -> str:
    """Add one manual trace group to the modelspace and return an entity type label.

    Supported native entities are LINE, CIRCLE, and ARC. Anything unsupported falls
    back to the previous safe LWPOLYLINE behavior.
    """

    entity_kind = _entity_type(entity)
    attribs = {"layer": layer_name}

    if entity_kind == "line" and isinstance(entity, dict):
        start_point = entity.get("start")
        end_point = entity.get("end")
        if start_point is None or end_point is None:
            xy_points = [_xy_point(point) for point in group]
            start_point, end_point = xy_points[0], xy_points[-1]
        msp.add_line(_xy_point(start_point), _xy_point(end_point), dxfattribs=attribs)
        return "LINE"

    if entity_kind == "circle" and isinstance(entity, dict):
        center = entity.get("center")
        radius = float(entity.get("radius", 0.0))
        if center is None or radius <= 0.0:
            raise ValueError("Invalid circle trace entity")
        msp.add_circle(_xy_point(center), radius, dxfattribs=attribs)
        return "CIRCLE"

    if entity_kind == "arc" and isinstance(entity, dict):
        center = entity.get("center")
        radius = float(entity.get("radius", 0.0))
        if center is None or radius <= 0.0:
            raise ValueError("Invalid arc trace entity")
        start_angle = _normalize_angle_degrees(float(entity.get("start_angle", 0.0)))
        end_angle = _normalize_angle_degrees(float(entity.get("end_angle", 0.0)))
        msp.add_arc(
            center=_xy_point(center),
            radius=radius,
            start_angle=start_angle,
            end_angle=end_angle,
            dxfattribs=attribs,
        )
        return "ARC"

    xy_points = [_xy_point(point) for point in group]
    msp.add_lwpolyline(xy_points, close=bool(close), dxfattribs=attribs)
    return "LWPOLYLINE"


def export_trace_groups_to_dxf(
    trace_groups: list[list[Tuple[float, float, float]]] | list[list[Tuple[float, float]]],
    output_path: str | Path,
    close: bool = True,
    layer_name: str = "TRACE",
    trace_entities: Optional[list[object | None]] = None,
) -> Path:
    """Export manually captured CNC XY traces as DXF geometry.

    Normal trace groups become separate LWPOLYLINE entities. Assisted trace tools
    can provide native entity metadata so fitted lines, circles, and arcs export
    as real DXF LINE/CIRCLE/ARC entities instead of sampled polylines.
    Z is accepted for capture history but ignored for 2D DXF export.
    """

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    indexed_groups = [(index, group) for index, group in enumerate(trace_groups) if group]
    if not indexed_groups:
        raise ValueError("At least one trace group is required for DXF export")

    for export_index, (group_index, group) in enumerate(indexed_groups, start=1):
        entity = trace_entities[group_index] if trace_entities is not None and group_index < len(trace_entities) else None
        entity_kind = _entity_type(entity)

        if entity_kind in ("circle",):
            if len(group) < 3:
                raise ValueError(f"Trace {export_index} circle needs at least three captured points")
            continue
        if entity_kind in ("arc",):
            if len(group) < 2:
                raise ValueError(f"Trace {export_index} arc needs at least two captured points")
            continue
        if entity_kind in ("line",):
            if len(group) < 2:
                raise ValueError(f"Trace {export_index} line needs at least two captured points")
            continue

        if len(group) < 2:
            raise ValueError(f"Trace {export_index} needs at least two points")
        if close and len(group) < 3:
            raise ValueError(f"Closed trace {export_index} needs at least three points")

    doc = ezdxf.new("R2010")
    doc.units = ezdxf.units.IN
    msp = doc.modelspace()

    if layer_name not in doc.layers:
        doc.layers.new(name=layer_name)

    for _group_index, group in indexed_groups:
        entity = trace_entities[_group_index] if trace_entities is not None and _group_index < len(trace_entities) else None
        _add_trace_entity_to_modelspace(msp, group, entity, close=close, layer_name=layer_name)

    doc.saveas(output_path)
    return output_path
