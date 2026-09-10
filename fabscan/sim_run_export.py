from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, is_dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable, Sequence

import cv2
import ezdxf
import numpy as np

INTEGRATION_VERSION = "0.6.0-dev-m5.6"


def _jsonable(value: Any) -> Any:
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
        return None if math.isnan(value) else ("inf" if value > 0 else "-inf")
    return value


def _cad_xy(points: Iterable[Sequence[float]], sheet_height_in: float) -> list[tuple[float, float]]:
    """Convert SVG/simulator sheet coordinates (+Y down) to CAD coordinates (+Y up)."""
    return [(float(p[0]), float(sheet_height_in) - float(p[1])) for p in points]


def _ordered_truth_centerline(points: np.ndarray, ppi: int) -> tuple[list[tuple[float, float]], bool]:
    """Order the precomputed one-pixel SVG centerline into one path/cycle.

    The M5 truth assets contain a single non-branching 8-connected centerline for
    each A-G profile. A-E have two degree-1 endpoints; F/G are closed degree-2
    cycles. Ordering it here gives CAD a real polyline instead of thousands of
    unrelated POINT entities.
    """
    if len(points) < 2:
        return [(float(p[0]), float(p[1])) for p in points], False

    grid = {(int(round(float(x) * ppi)), int(round(float(y) * ppi))) for x, y in points}
    adjacency: dict[tuple[int, int], list[tuple[int, int]]] = {}
    for node in grid:
        x, y = node
        nbrs = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nxt = (x + dx, y + dy)
                if nxt in grid:
                    nbrs.append(nxt)
        adjacency[node] = nbrs

    endpoints = [node for node, nbrs in adjacency.items() if len(nbrs) == 1]
    closed = len(endpoints) == 0
    start = min(endpoints) if endpoints else min(grid)

    ordered = [start]
    previous: tuple[int, int] | None = None
    current = start
    visited_edges: set[frozenset[tuple[int, int]]] = set()
    max_edges = max(1, len(grid) + 8)

    for _ in range(max_edges):
        choices = []
        for nxt in adjacency[current]:
            edge = frozenset((current, nxt))
            if edge not in visited_edges:
                choices.append(nxt)
        if not choices:
            break
        if previous is not None and len(choices) > 1:
            non_back = [n for n in choices if n != previous]
            if non_back:
                choices = non_back
        nxt = choices[0]
        visited_edges.add(frozenset((current, nxt)))
        previous, current = current, nxt
        if closed and current == start:
            break
        ordered.append(current)

    path = [(x / float(ppi), y / float(ppi)) for x, y in ordered]
    return path, closed


def _simplify(points: Sequence[tuple[float, float]], closed: bool, tolerance_in: float = 0.00075) -> list[tuple[float, float]]:
    if len(points) <= 2:
        return list(points)
    arr = np.asarray(points, dtype=np.float32).reshape((-1, 1, 2))
    approx = cv2.approxPolyDP(arr, epsilon=float(tolerance_in), closed=bool(closed))
    simplified = [(float(p[0][0]), float(p[0][1])) for p in approx]
    if len(simplified) < 2:
        return list(points)
    return simplified


def _add_polyline(msp, points: Sequence[tuple[float, float]], layer: str, *, close: bool = False) -> None:
    if len(points) >= 2:
        msp.add_lwpolyline(points, close=bool(close), dxfattribs={"layer": layer})


def _write_combined_dxf(
    output_path: Path,
    *,
    sheet_height_in: float,
    truth_points_sheet: Sequence[tuple[float, float]],
    truth_closed: bool,
    camera_points_sheet: Sequence[tuple[float, float]],
    belief_points_sheet: Sequence[tuple[float, float]],
) -> None:
    doc = ezdxf.new("R2010")
    doc.units = ezdxf.units.IN
    msp = doc.modelspace()

    layer_specs = (
        ("SVG_TRUTH", 8),
        ("CAMERA_PATH", 1),
        ("M4_BELIEF", 4),
    )
    for name, color in layer_specs:
        if name not in doc.layers:
            doc.layers.new(name=name, dxfattribs={"color": color})

    truth_cad = _cad_xy(truth_points_sheet, sheet_height_in)
    camera_cad = _cad_xy(camera_points_sheet, sheet_height_in)
    belief_cad = _cad_xy(belief_points_sheet, sheet_height_in)
    _add_polyline(msp, truth_cad, "SVG_TRUTH", close=truth_closed)
    _add_polyline(msp, camera_cad, "CAMERA_PATH", close=False)
    _add_polyline(msp, belief_cad, "M4_BELIEF", close=False)

    doc.saveas(output_path)


def _write_xy_csv(path: Path, points_sheet: Sequence[tuple[float, float]], sheet_height_in: float) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["index", "sheet_x_in", "sheet_y_in", "cad_x_in", "cad_y_in"])
        for i, (x, y) in enumerate(points_sheet):
            writer.writerow([i, f"{x:.9f}", f"{y:.9f}", f"{x:.9f}", f"{sheet_height_in-y:.9f}"])


def _write_diagnostic_csv(path: Path, samples: Sequence[Any]) -> None:
    rows = [asdict(s) if is_dataclass(s) else dict(s) for s in samples]
    if not rows:
        path.write_text("step\n", encoding="utf-8")
        return
    fields = list(rows[0].keys())
    extra = [
        "perception_position_error_in",
        "estimator_position_error_in",
        "planner_target_cross_track_error_in",
        "controller_tracking_error_in",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields + extra)
        writer.writeheader()
        for row in rows:
            def dist(ax, ay, bx, by):
                if None in (ax, ay, bx, by):
                    return None
                return math.hypot(float(ax)-float(bx), float(ay)-float(by))
            out = dict(row)
            out[extra[0]] = row.get("measured_cross_track_error")
            if out[extra[0]] is None:
                out[extra[0]] = dist(row.get("measured_x"), row.get("measured_y"), row.get("truth_x"), row.get("truth_y"))
            out[extra[1]] = row.get("estimate_cross_track_error")
            if out[extra[1]] is None:
                out[extra[1]] = dist(row.get("estimate_x"), row.get("estimate_y"), row.get("truth_x"), row.get("truth_y"))
            out[extra[2]] = dist(row.get("planner_x"), row.get("planner_y"), row.get("planner_truth_x"), row.get("planner_truth_y"))
            out[extra[3]] = dist(row.get("actual_x"), row.get("actual_y"), row.get("planner_x"), row.get("planner_y"))
            writer.writerow(out)


def _summary_text(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    def fmt(group: str, key: str = "rms", suffix: str = " in") -> str:
        value = summary.get(group, {}).get(key)
        return "—" if value is None else f"{float(value):.6f}{suffix}"
    lines = [
        "FabScan M5.6 Simulated Run Diagnostic Summary",
        "",
        f"Shape: {manifest.get('shape', '—')}",
        f"Steps: {manifest.get('sim_step_count', 0)}",
        f"Travel: {manifest.get('total_distance_in', 0.0):.6f} in",
        f"Stop reason: {manifest.get('stop_reason') or 'manual save / run still open'}",
        f"Camera realism: {manifest.get('camera_realism_percent', 0.0):.1f}%  seed {manifest.get('camera_realism_seed', 0)}",
        f"Observation delay: {manifest.get('observation_delay_ms', 0.0):.1f} ms",
        f"Machine dynamics: {'enabled' if manifest.get('acceleration_model') else 'disabled'}",
        f"Configured accel X/Y: {manifest.get('machine_accel_x_in_s2', 0.0):.1f} / {manifest.get('machine_accel_y_in_s2', 0.0):.1f} in/s^2",
        f"Configured velocity cap: {manifest.get('machine_max_velocity_ipm', 0.0):.1f} IPM",
        f"Servo period: {manifest.get('machine_servo_period_ms', 0.0):.1f} ms",
        f"Virtual run time: {manifest.get('virtual_time_s', 0.0):.6f} s",
        "",
        f"Perception profile X-track RMS: {fmt('perception_position_error_in')}",
        f"Perception profile X-track max: {fmt('perception_position_error_in', 'max')}",
        f"Estimator profile X-track RMS:  {fmt('estimator_position_error_in')}",
        f"Estimator profile X-track max:  {fmt('estimator_position_error_in', 'max')}",
        f"Estimator tangent RMS:   {fmt('estimator_tangent_error_deg', suffix=' deg')}",
        f"Planner target RMS:      {fmt('planner_truth_error_in')}",
        f"Planner target max:      {fmt('planner_truth_error_in', 'max')}",
        f"Controller tracking RMS: {fmt('controller_tracking_error_in')}",
        f"Actual machine path RMS: {fmt('actual_machine_cross_track_error_in')}",
        f"Actual machine path max: {fmt('actual_machine_cross_track_error_in', 'max')}",
        f"Velocity tracking RMS:   {fmt('velocity_tracking_error_ipm', suffix=' IPM')}",
        f"Acceleration-limited moves: {summary.get('acceleration_limited_events', 0)}",
        f"Velocity-limited moves:     {summary.get('velocity_limited_events', 0)}",
        "",
        f"Reverse events: {summary.get('reverse_events', 0)}",
        f"Safety interventions: {len(summary.get('safety_events', []))}",
        f"Likely layer: {summary.get('likely_layer', '—')}",
        "",
        "Diagnosis:",
    ]
    lines.extend(f"- {item}" for item in summary.get("diagnosis", []))
    return "\n".join(lines) + "\n"


def export_simulated_run(
    *,
    output_root: Path | str,
    shape: str,
    world: Any,
    planner: Any,
    diagnostics: Any,
    planner_history: Sequence[dict[str, Any]],
    source_replay: Path | str | None,
    sim_step_count: int,
    stop_reason: str,
    integration_version: str,
    settings: dict[str, Any] | None = None,
    calibration: dict[str, Any] | None = None,
    camera_realism: dict[str, Any] | None = None,
    timing_model: dict[str, Any] | None = None,
) -> Path:
    """Save one M5.6 virtual run as DXF + machine-readable diagnostics."""
    output_root = Path(output_root).expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = output_root / f"{stamp}_shape_{str(shape).upper()}"
    suffix = 2
    while run_dir.exists():
        run_dir = output_root / f"{stamp}_shape_{str(shape).upper()}_{suffix}"
        suffix += 1
    run_dir.mkdir(parents=True)

    camera_path = list(getattr(planner, "camera_history", []) or [])
    samples = list(getattr(diagnostics, "samples", []) or [])
    belief_path = [
        (float(s.estimate_x), float(s.estimate_y))
        for s in samples
        if getattr(s, "estimate_x", None) is not None and getattr(s, "estimate_y", None) is not None
    ]
    truth_raw, truth_closed = _ordered_truth_centerline(world.components[str(shape).upper()], int(world.world_ppi))
    truth_path = _simplify(truth_raw, truth_closed)
    summary = diagnostics.summarize()

    manifest = {
        "format": 1,
        "fabscan_integration_version": integration_version,
        "exporter_version": INTEGRATION_VERSION,
        "created_at": datetime.now().astimezone().isoformat(),
        "shape": str(shape).upper(),
        "source_svg": str(getattr(world, "svg_path", "TestImage.svg")),
        "source_replay": str(source_replay) if source_replay is not None else None,
        "coordinate_note": "CSV includes SVG/simulator sheet coordinates (+Y down) and CAD coordinates (+Y up). DXF uses CAD +Y up and inches.",
        "sheet_width_in": float(world.width_in),
        "sheet_height_in": float(world.height_in),
        "truth_centerline_ppi": int(world.world_ppi),
        "truth_polyline_points_raw": len(truth_raw),
        "truth_polyline_points_dxf": len(truth_path),
        "sim_step_count": int(sim_step_count),
        "camera_path_points": len(camera_path),
        "belief_path_points": len(belief_path),
        "total_distance_in": float(getattr(planner, "total_distance", 0.0) or 0.0),
        "stop_reason": str(stop_reason or ""),
        "start_pose_sheet_in": list(camera_path[0]) if camera_path else None,
        "end_pose_sheet_in": list(camera_path[-1]) if camera_path else None,
        "dxf_layers": ["SVG_TRUTH", "CAMERA_PATH", "M4_BELIEF"],
        "camera_realism_percent": float((camera_realism or {}).get("percent", 0.0)),
        "camera_realism_seed": int((camera_realism or {}).get("seed", 0)),
        "camera_realism_source": ((camera_realism or {}).get("profile", {}) or {}).get("source"),
        "camera_realism_frames_measured": int(((camera_realism or {}).get("profile", {}) or {}).get("frames_measured", 0)),
        "observation_delay_ms": float((timing_model or {}).get("observation_delay_ms", 0.0)),
        "virtual_time_s": float((timing_model or {}).get("virtual_time_s", 0.0)),
        "last_observation_time_s": float((timing_model or {}).get("last_observation_time_s", 0.0)),
        "last_observation_lag_distance_in": float((timing_model or {}).get("last_observation_lag_distance_in", 0.0)),
        "timing_time_basis": (timing_model or {}).get("time_basis"),
        "acceleration_model": bool((timing_model or {}).get("acceleration_model", False)),
        "controller_lag_model": bool((timing_model or {}).get("controller_lag_model", False)),
        "machine_accel_x_in_s2": float(((timing_model or {}).get("machine_dynamics", {}) or {}).get("accel_x_in_s2", 0.0)),
        "machine_accel_y_in_s2": float(((timing_model or {}).get("machine_dynamics", {}) or {}).get("accel_y_in_s2", 0.0)),
        "machine_max_velocity_ipm": float(((timing_model or {}).get("machine_dynamics", {}) or {}).get("max_velocity_ipm", 0.0)),
        "machine_servo_period_ms": float(((timing_model or {}).get("machine_dynamics", {}) or {}).get("servo_period_ms", 0.0)),
        "machine_current_velocity_ipm": float((timing_model or {}).get("current_velocity_ipm", 0.0)),
    }

    (run_dir / "manifest.json").write_text(json.dumps(_jsonable(manifest), indent=2), encoding="utf-8")
    (run_dir / "diagnostics.json").write_text(json.dumps(_jsonable(summary), indent=2), encoding="utf-8")
    (run_dir / "diagnostics.txt").write_text(_summary_text(summary, manifest), encoding="utf-8")
    (run_dir / "settings.json").write_text(json.dumps(_jsonable(settings or {}), indent=2), encoding="utf-8")
    (run_dir / "calibration.json").write_text(json.dumps(_jsonable(calibration or {}), indent=2), encoding="utf-8")
    (run_dir / "camera_realism.json").write_text(json.dumps(_jsonable(camera_realism or {}), indent=2), encoding="utf-8")
    (run_dir / "timing_model.json").write_text(json.dumps(_jsonable(timing_model or {}), indent=2), encoding="utf-8")
    (run_dir / "machine_dynamics.json").write_text(json.dumps(_jsonable(((timing_model or {}).get("machine_dynamics", {}) or {})), indent=2), encoding="utf-8")

    _write_xy_csv(run_dir / "camera_path.csv", camera_path, float(world.height_in))
    _write_xy_csv(run_dir / "m4_belief_path.csv", belief_path, float(world.height_in))
    _write_xy_csv(run_dir / "svg_truth_reference.csv", truth_path, float(world.height_in))
    _write_diagnostic_csv(run_dir / "diagnostic_samples.csv", samples)

    with (run_dir / "planner_steps.jsonl").open("w", encoding="utf-8") as handle:
        for item in planner_history:
            handle.write(json.dumps(_jsonable(item), separators=(",", ":")) + "\n")

    _write_combined_dxf(
        run_dir / "simulated_trace_layers.dxf",
        sheet_height_in=float(world.height_in),
        truth_points_sheet=truth_path,
        truth_closed=truth_closed,
        camera_points_sheet=camera_path,
        belief_points_sheet=belief_path,
    )

    return run_dir
