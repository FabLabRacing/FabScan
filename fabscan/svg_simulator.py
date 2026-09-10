from __future__ import annotations

import json
import math
import threading
import tkinter as tk
from dataclasses import asdict, dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Any, Optional

import numpy as np
import cv2
from PIL import Image, ImageTk

from fabscan.path_belief import BeliefState, PathBeliefEstimator
from fabscan.replay_rerun import ExistingPlannerRerunner
from fabscan.virtual_planner import ConnectedPathPlanner, PlannerResult
from fabscan.sim_run_export import export_simulated_run
from fabscan.camera_realism import RealCameraModel
from fabscan.machine_dynamics import XYMachineDynamics

INTEGRATION_VERSION = "0.6.0-dev-m5.6"


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else np.asarray([1.0, 0.0])


def _angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def _axis_angle_error_degrees(a: float, b: float) -> float:
    """Smallest angular error between two unoriented line tangents."""
    diff = abs(((float(a) - float(b) + 180.0) % 360.0) - 180.0)
    return min(diff, abs(180.0 - diff))


def _align_axis_angle_to_reference(angle: float, reference: float) -> float:
    candidates = (float(angle), float(angle) + 180.0, float(angle) - 180.0)
    return min(candidates, key=lambda value: abs(((value - reference + 180.0) % 360.0) - 180.0))


@dataclass(frozen=True)
class TruthSample:
    query_x: float
    query_y: float
    truth_x: float
    truth_y: float
    distance: float
    tangent_x: float
    tangent_y: float
    tangent_degrees: float
    curvature: float
    implied_radius: float
    shape: str


@dataclass
class DiagnosticSample:
    step: int
    truth_x: float
    truth_y: float
    truth_tangent_deg: float
    truth_curvature: float
    measured_x: Optional[float] = None
    measured_y: Optional[float] = None
    measured_tangent_deg: Optional[float] = None
    measured_cross_track_error: Optional[float] = None
    measured_truth_tangent_deg: Optional[float] = None
    estimate_x: Optional[float] = None
    estimate_y: Optional[float] = None
    estimate_tangent_deg: Optional[float] = None
    estimate_curvature: Optional[float] = None
    estimate_cross_track_error: Optional[float] = None
    estimate_truth_tangent_deg: Optional[float] = None
    planner_x: Optional[float] = None
    planner_y: Optional[float] = None
    planner_truth_x: Optional[float] = None
    planner_truth_y: Optional[float] = None
    actual_x: Optional[float] = None
    actual_y: Optional[float] = None
    confidence: Optional[float] = None
    usable_lookahead: Optional[float] = None
    dynamics_enabled: bool = False
    requested_velocity_ipm: Optional[float] = None
    actual_velocity_ipm: Optional[float] = None
    acceleration_limited: bool = False
    velocity_limited: bool = False
    actual_cross_track_error: Optional[float] = None
    decision_reason: str = ""


class PostRunDiagnostics:
    """Ground-truth-aware post-run diagnostics for future M5+ runs.

    The separation is deliberate: estimator error, planner error, and controller
    error are reported independently so FabScan can eventually say *which layer*
    most likely caused a bad run rather than just saying the run was bad.
    """

    def __init__(self) -> None:
        self.samples: list[DiagnosticSample] = []
        self.reverse_events = 0
        self.low_lookahead_accel_events = 0
        self.safety_events: list[str] = []

    def add(self, sample: DiagnosticSample) -> None:
        self.samples.append(sample)

    def summarize(self) -> dict[str, Any]:
        def pos_error(ax, ay, bx, by):
            if None in (ax, ay, bx, by):
                return None
            return math.hypot(float(ax) - float(bx), float(ay) - float(by))

        perception_pos: list[float] = []
        perception_tangent: list[float] = []
        est_pos: list[float] = []
        planner_truth: list[float] = []
        controller: list[float] = []
        tangent: list[float] = []
        curvature: list[float] = []
        velocity_tracking: list[float] = []
        actual_cross_track: list[float] = []
        acceleration_limited_events = 0
        velocity_limited_events = 0
        for sample in self.samples:
            value = sample.measured_cross_track_error
            if value is None:
                value = pos_error(sample.measured_x, sample.measured_y, sample.truth_x, sample.truth_y)
            if value is not None:
                perception_pos.append(float(value))
            if sample.measured_tangent_deg is not None:
                tangent_truth = sample.measured_truth_tangent_deg if sample.measured_truth_tangent_deg is not None else sample.truth_tangent_deg
                perception_tangent.append(abs(math.degrees(_angle_diff(
                    math.radians(float(sample.measured_tangent_deg)),
                    math.radians(float(tangent_truth)),
                ))))
            value = sample.estimate_cross_track_error
            if value is None:
                value = pos_error(sample.estimate_x, sample.estimate_y, sample.truth_x, sample.truth_y)
            if value is not None:
                est_pos.append(float(value))
            value = pos_error(sample.planner_x, sample.planner_y, sample.planner_truth_x, sample.planner_truth_y)
            if value is not None:
                planner_truth.append(value)
            value = pos_error(sample.actual_x, sample.actual_y, sample.planner_x, sample.planner_y)
            if value is not None:
                controller.append(value)
            if sample.estimate_tangent_deg is not None:
                tangent_truth = sample.estimate_truth_tangent_deg if sample.estimate_truth_tangent_deg is not None else sample.truth_tangent_deg
                tangent.append(abs(math.degrees(_angle_diff(
                    math.radians(float(sample.estimate_tangent_deg)),
                    math.radians(float(tangent_truth)),
                ))))
            if sample.estimate_curvature is not None:
                curvature.append(abs(float(sample.estimate_curvature) - float(sample.truth_curvature)))
            if sample.requested_velocity_ipm is not None and sample.actual_velocity_ipm is not None:
                velocity_tracking.append(abs(float(sample.requested_velocity_ipm) - float(sample.actual_velocity_ipm)))
            if sample.actual_cross_track_error is not None:
                actual_cross_track.append(float(sample.actual_cross_track_error))
            if sample.acceleration_limited:
                acceleration_limited_events += 1
            if sample.velocity_limited:
                velocity_limited_events += 1

        def stats(values: list[float]) -> dict[str, Any]:
            if not values:
                return {"count": 0, "mean": None, "rms": None, "max": None}
            arr = np.asarray(values, dtype=float)
            return {
                "count": len(values),
                "mean": float(arr.mean()),
                "rms": float(np.sqrt(np.mean(arr * arr))),
                "max": float(arr.max()),
            }

        perception_stats = stats(perception_pos)
        estimator_stats = stats(est_pos)
        planner_stats = stats(planner_truth)
        controller_stats = stats(controller)
        diagnosis: list[str] = []
        likely_layer = "no dominant failure layer identified"
        p_rms = perception_stats.get("rms")
        e_rms = estimator_stats.get("rms")
        pl_rms = planner_stats.get("rms")
        c_rms = controller_stats.get("rms")
        if p_rms is not None and p_rms > 0.008:
            likely_layer = "perception / path association"
            diagnosis.append("Fresh camera measurements have material cross-track error from the SVG profile.")
        if e_rms is not None and p_rms is not None and e_rms > max(0.006, p_rms * 1.6):
            likely_layer = "state estimator"
            diagnosis.append("Estimator error is substantially larger than the incoming measurement error.")
        if pl_rms is not None and pl_rms > 0.008 and (e_rms is None or pl_rms > e_rms * 1.5):
            likely_layer = "planner"
            diagnosis.append("Planned virtual targets diverge more than the state estimate itself.")
        if c_rms is not None and c_rms > 0.002:
            if acceleration_limited_events:
                likely_layer = "machine acceleration / motion dynamics"
                diagnosis.append("Approved targets are not reached within one planner interval because the configured acceleration/deceleration limit is active; inspect actual cross-track error before treating this as a controller fault.")
            else:
                likely_layer = "controller / executor"
                diagnosis.append("Executed motion does not track the approved planner target.")
        if self.safety_events:
            diagnosis.append(f"Safety gate intervened {len(self.safety_events)} time(s); inspect the first intervention before tuning around it.")
        if not diagnosis:
            diagnosis.append("No layer exceeds the current diagnostic thresholds; compare geometry-specific peaks next.")

        return {
            "samples": len(self.samples),
            "perception_position_error_in": perception_stats,
            "perception_tangent_error_deg": stats(perception_tangent),
            "estimator_position_error_in": estimator_stats,
            "estimator_tangent_error_deg": stats(tangent),
            "estimator_curvature_error_1_per_in": stats(curvature),
            "planner_truth_error_in": planner_stats,
            "controller_tracking_error_in": controller_stats,
            "actual_machine_cross_track_error_in": stats(actual_cross_track),
            "velocity_tracking_error_ipm": stats(velocity_tracking),
            "acceleration_limited_events": acceleration_limited_events,
            "velocity_limited_events": velocity_limited_events,
            "reverse_events": self.reverse_events,
            "low_lookahead_accel_events": self.low_lookahead_accel_events,
            "safety_events": list(self.safety_events),
            "likely_layer": likely_layer,
            "diagnosis": diagnosis,
        }



class SVGGroundTruthWorld:
    """Virtual sheet derived from the exact TestImage.svg supplied for FabScan.

    Runtime uses a bundled 600-pixel/inch raster and precomputed stroke centerlines.
    No CairoSVG/scikit-image/scipy dependency is required on the plasma PC.
    Position ground truth is therefore quantized at 1/600 inch (~0.0017 inch).
    """

    SHAPES = tuple("ABCDEFG")

    def __init__(self, asset_dir: Path | str) -> None:
        root = Path(asset_dir).expanduser().resolve()
        self.asset_dir = root
        self.svg_path = root / "TestImage.svg"
        self.raster_path = root / "TestImage_600ppi.png"
        self.truth_path = root / "TestImage_truth.npz"
        for path in (self.svg_path, self.raster_path, self.truth_path):
            if not path.is_file():
                raise ValueError(f"Missing Simulator asset: {path}")
        data = np.load(self.truth_path)
        self.world_ppi = int(data["ppi"][0])
        self.width_in = float(data["width_in"][0])
        self.height_in = float(data["height_in"][0])
        self.components = {name: np.asarray(data[name], dtype=float) for name in self.SHAPES}
        self.rgb = np.asarray(Image.open(self.raster_path).convert("RGB"))
        self._camera_delta_cache: dict[tuple[Any, ...], np.ndarray] = {}

    def nearest(self, x: float, y: float, shape: str) -> TruthSample:
        shape = shape.upper()
        points = self.components[shape]
        q = np.asarray([x, y], dtype=float)
        dist2 = np.sum((points - q) ** 2, axis=1)
        index = int(np.argmin(dist2))
        p = points[index]
        distance = float(math.sqrt(float(dist2[index])))

        # PCA around the nearest stroke-center point gives a robust local tangent.
        local_dist2 = np.sum((points - p) ** 2, axis=1)
        nearest = np.argpartition(local_dist2, min(60, len(points) - 1))[: min(61, len(points))]
        neighborhood = points[nearest]
        mean = neighborhood.mean(axis=0)
        centered = neighborhood - mean
        covariance = centered.T @ centered
        values, vectors = np.linalg.eigh(covariance)
        tangent = _unit(vectors[:, int(np.argmax(values))])
        normal = np.asarray([-tangent[1], tangent[0]])
        u = centered @ tangent
        v = centered @ normal
        curvature = 0.0
        if len(u) >= 9 and float(np.ptp(u)) > 0.04:
            try:
                a, b, _c = np.polyfit(u, v, 2)
                curvature = float(2 * a / ((1 + b * b) ** 1.5))
            except Exception:
                curvature = 0.0
        angle = math.degrees(math.atan2(float(tangent[1]), float(tangent[0])))
        radius = (1.0 / abs(curvature)) if abs(curvature) > 1e-6 else math.inf
        return TruthSample(
            query_x=float(x), query_y=float(y), truth_x=float(p[0]), truth_y=float(p[1]),
            distance=distance, tangent_x=float(tangent[0]), tangent_y=float(tangent[1]),
            tangent_degrees=angle, curvature=curvature, implied_radius=radius, shape=shape,
        )

    def render_camera(self, x: float, y: float, width_px: int = 800, height_px: int = 600, camera_ppi: float = 635.0) -> Image.Image:
        # M5 starts with sheet-aligned camera axes. Rotation can be introduced later
        # once planner/estimator integration needs the full real-camera transform.
        fov_w = width_px / camera_ppi
        fov_h = height_px / camera_ppi
        x0 = x - fov_w / 2.0
        y0 = y - fov_h / 2.0
        scale = self.world_ppi
        left = round(x0 * scale)
        top = round(y0 * scale)
        right = round((x0 + fov_w) * scale)
        bottom = round((y0 + fov_h) * scale)
        out = np.full((max(1, bottom - top), max(1, right - left), 3), 255, dtype=np.uint8)
        src_l = max(0, left); src_t = max(0, top)
        src_r = min(self.rgb.shape[1], right); src_b = min(self.rgb.shape[0], bottom)
        if src_r > src_l and src_b > src_t:
            dst_l = src_l - left; dst_t = src_t - top
            out[dst_t:dst_t + (src_b - src_t), dst_l:dst_l + (src_r - src_l)] = self.rgb[src_t:src_b, src_l:src_r]
        return Image.fromarray(out).resize((width_px, height_px), Image.Resampling.LANCZOS)

    def render_camera_calibrated(
        self,
        x: float,
        y: float,
        calibration: dict[str, Any],
        width_px: int = 800,
        height_px: int = 600,
    ) -> Image.Image:
        """Render an already-transformed camera view using the real saved calibration.

        The calibration matrix records how a fixed feature moves in pixels when
        the camera moves in machine coordinates. Therefore a physical feature
        offset from camera center maps to the negative of that response.
        """
        inv = np.asarray(calibration.get("matrix_pixel_to_machine"), dtype=float)
        if inv.shape != (2, 2):
            raise ValueError("Calibration does not contain a 2x2 matrix_pixel_to_machine")
        cache_key = (width_px, height_px, *[round(float(v), 12) for v in inv.ravel()])
        world_delta = self._camera_delta_cache.get(cache_key)
        if world_delta is None:
            yy, xx = np.mgrid[0:height_px, 0:width_px]
            pixel_delta = np.stack([xx - width_px / 2.0, yy - height_px / 2.0], axis=-1).astype(np.float32)
            world_delta = -(pixel_delta @ inv.astype(np.float32).T)
            self._camera_delta_cache[cache_key] = world_delta
        map_x = ((float(x) + world_delta[..., 0]) * self.world_ppi).astype(np.float32)
        map_y = ((float(y) + world_delta[..., 1]) * self.world_ppi).astype(np.float32)
        out = cv2.remap(
            self.rgb,
            map_x,
            map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(255, 255, 255),
        )
        return Image.fromarray(out)

    def shape_center(self, shape: str) -> tuple[float, float]:
        points = self.components[shape.upper()]
        centroid = points.mean(axis=0)
        # Put the virtual camera on the profile, near the visual center of the
        # shape, rather than at the polygon centroid (which may be far from G).
        index = int(np.argmin(np.sum((points - centroid) ** 2, axis=1)))
        point = points[index]
        return float(point[0]), float(point[1])


@dataclass(frozen=True)
class VirtualFrameAnalysis:
    step: int
    camera_x: float
    camera_y: float
    truth: TruthSample
    raw_found: bool
    raw_pixel_error_x: float
    raw_pixel_error_y: float
    raw_angle_degrees: float
    raw_confidence: float
    raw_point_count: int
    measured_x: Optional[float]
    measured_y: Optional[float]
    measured_angle_degrees: Optional[float]
    measurement_position_error: Optional[float]
    measurement_tangent_error_degrees: Optional[float]
    belief: BeliefState
    estimator_position_error: Optional[float]
    estimator_tangent_error_degrees: Optional[float]
    estimator_curvature_magnitude_error: Optional[float]
    diagnostics_summary: dict[str, Any]


class VirtualPerceptionEstimatorBridge:
    """Connect an arbitrary M5 virtual camera frame to real FabScan perception + M4 belief.

    The replay run is used only as a detector-settings source. No historical frame,
    historical detection, planner result, or LinuxCNC position participates in the
    analysis. Virtual machine/world coordinates are sheet coordinates in inches.
    """

    def __init__(self, dialog_cls: type, run_dir: Path | str, camera_ppi: float) -> None:
        self.dialog_cls = dialog_cls
        self.run_dir = Path(run_dir).expanduser().resolve()
        self.camera_ppi = float(camera_ppi)
        self.step_index = 0
        self.previous_camera: Optional[tuple[float, float]] = None
        self.diagnostics = PostRunDiagnostics()
        self._build()

    @staticmethod
    def _synthetic_calibration(camera_ppi: float) -> dict[str, Any]:
        ppi = max(1.0, float(camera_ppi))
        # The live calibration matrix maps camera motion to observed pixel motion.
        # In the sheet-aligned virtual world, moving the camera +X/+Y makes a fixed
        # feature move -X/-Y in the image, hence the negative diagonal matrix.
        return {
            "valid": True,
            "coordinate_mode_label": "Virtual SVG sheet coordinates",
            "camera_width": 800,
            "camera_height": 600,
            "matrix_pixel_to_machine": [[-1.0 / ppi, 0.0], [0.0, -1.0 / ppi]],
            "matrix_machine_to_pixel": [[-ppi, 0.0], [0.0, -ppi]],
            "pixels_per_unit_x": ppi,
            "pixels_per_unit_y": ppi,
        }

    def _build(self) -> None:
        # Reuse M3's already-tested safe headless CameraCalibrationDialog harness.
        # It creates no LinuxCNC command object and its jog sender is a no-motion sink.
        self.rerunner = ExistingPlannerRerunner(self.dialog_cls, self.run_dir)
        self.harness = self.rerunner._harness
        self.settings = dict(self.rerunner.settings)
        self.calibration = dict(self.rerunner.calibration)
        self.harness.active_calibration = dict(self.calibration)
        # Virtual frames are already camera-transformed/sheet-aligned. We therefore
        # call detect_line_in_frame() directly and intentionally bypass rotate/flip.
        self.estimator = PathBeliefEstimator(self.calibration, self.settings)

    def reset(self, camera_ppi: Optional[float] = None) -> None:
        if camera_ppi is not None:
            self.camera_ppi = float(camera_ppi)
        self.step_index = 0
        self.previous_camera = None
        self.diagnostics = PostRunDiagnostics()
        self._build()

    @staticmethod
    def _line_payload(line: Any, move_x: float, move_y: float) -> dict[str, Any]:
        return {
            "result": "found" if bool(line.found) else "not_found",
            "pixel_error_x": float(getattr(line, "pixel_error_x", 0.0)),
            "pixel_error_y": float(getattr(line, "pixel_error_y", 0.0)),
            "angle_degrees": float(getattr(line, "angle_degrees", 0.0)),
            "confidence": float(getattr(line, "confidence", 0.0)),
            "point_count": int(getattr(line, "point_count", 0)),
            "span_px": float(getattr(line, "span_px", 0.0)),
            "width_px": float(getattr(line, "width_px", 0.0)),
            "search_px": int(getattr(line, "search_px", 0)),
            "move_x": float(move_x),
            "move_y": float(move_y),
        }

    def world_to_pixel(self, camera_x: float, camera_y: float, world_x: float, world_y: float, width_px: int = 800, height_px: int = 600) -> tuple[float, float]:
        matrix = np.asarray(self.calibration.get("matrix_machine_to_pixel"), dtype=float)
        delta = np.asarray([float(world_x) - float(camera_x), float(world_y) - float(camera_y)], dtype=float)
        pixel_delta = -(matrix @ delta)
        return width_px / 2.0 + float(pixel_delta[0]), height_px / 2.0 + float(pixel_delta[1])

    def analyze(
        self,
        frame_bgr: np.ndarray,
        *,
        camera_x: float,
        camera_y: float,
        truth: TruthSample,
    ) -> VirtualFrameAnalysis:
        h = self.harness
        height, width = frame_bgr.shape[:2]
        previous_state = self.estimator.states[-1] if self.estimator.states else None
        if previous_state is not None and previous_state.measurement_valid:
            h._follow_heading_unit = (float(previous_state.tangent_x), float(previous_state.tangent_y))

        h.current_frame_bgr = frame_bgr
        raw = h.detect_line_in_frame(frame_bgr)
        stable = raw
        if raw.found:
            stable = h._stabilize_follow_line(raw, frame_w=width, frame_h=height)
        h.current_line = stable

        if self.previous_camera is None:
            move_x = move_y = 0.0
        else:
            move_x = float(camera_x) - float(self.previous_camera[0])
            move_y = float(camera_y) - float(self.previous_camera[1])

        planner_output = self._line_payload(stable, move_x, move_y) if stable.found else {}
        step_id = self.step_index + 1
        step = {
            "step_id": step_id,
            "step_label": f"Virtual estimator {step_id}",
            "position_before": {"x": float(camera_x), "y": float(camera_y), "z": 0.0},
            "delayed_position": {"x": float(camera_x), "y": float(camera_y), "z": 0.0, "source": "Simulator delayed virtual camera"},
            "position_after": {"x": float(camera_x), "y": float(camera_y), "z": 0.0},
            "planner_output": planner_output,
        }
        belief = self.estimator.update(step, frame_bgr, self.step_index)

        measured_x: Optional[float] = None
        measured_y: Optional[float] = None
        measured_angle: Optional[float] = None
        measurement_position_error: Optional[float] = None
        measurement_tangent_error: Optional[float] = None
        if stable.found:
            pixel_error = np.asarray([float(stable.pixel_error_x), float(stable.pixel_error_y)], dtype=float)
            inv = np.asarray(self.calibration["matrix_pixel_to_machine"], dtype=float)
            feature_offset = -(inv @ pixel_error)
            measured_x = float(camera_x) + float(feature_offset[0])
            measured_y = float(camera_y) + float(feature_offset[1])
            pixel_angle = math.radians(float(stable.angle_degrees))
            pixel_tangent = np.asarray([math.cos(pixel_angle), math.sin(pixel_angle)], dtype=float)
            world_tangent = -(inv @ pixel_tangent)
            world_norm = float(np.linalg.norm(world_tangent))
            if world_norm > 1e-12:
                world_tangent = world_tangent / world_norm
            measured_angle = math.degrees(math.atan2(float(world_tangent[1]), float(world_tangent[0])))
            measurement_position_error = math.hypot(measured_x - truth.truth_x, measured_y - truth.truth_y)
            measurement_tangent_error = _axis_angle_error_degrees(measured_angle, truth.tangent_degrees)

        estimator_position_error: Optional[float] = None
        estimator_tangent_error: Optional[float] = None
        estimator_curvature_error: Optional[float] = None
        if belief.measurement_valid:
            estimator_position_error = math.hypot(belief.estimate_x - truth.truth_x, belief.estimate_y - truth.truth_y)
            estimator_tangent_error = _axis_angle_error_degrees(belief.tangent_degrees, truth.tangent_degrees)
            estimator_curvature_error = abs(abs(float(belief.curvature)) - abs(float(truth.curvature)))
            aligned_truth_tangent = _align_axis_angle_to_reference(truth.tangent_degrees, belief.tangent_degrees)
            self.diagnostics.add(DiagnosticSample(
                step=step_id,
                truth_x=truth.truth_x,
                truth_y=truth.truth_y,
                truth_tangent_deg=aligned_truth_tangent,
                truth_curvature=abs(float(truth.curvature)),
                measured_x=measured_x,
                measured_y=measured_y,
                measured_tangent_deg=(_align_axis_angle_to_reference(measured_angle, aligned_truth_tangent) if measured_angle is not None else None),
                estimate_x=belief.estimate_x,
                estimate_y=belief.estimate_y,
                estimate_tangent_deg=belief.tangent_degrees,
                estimate_curvature=abs(float(belief.curvature)),
                confidence=belief.belief_confidence,
                usable_lookahead=belief.usable_lookahead,
                decision_reason=belief.reason,
            ))

        self.previous_camera = (float(camera_x), float(camera_y))
        self.step_index += 1
        return VirtualFrameAnalysis(
            step=step_id,
            camera_x=float(camera_x),
            camera_y=float(camera_y),
            truth=truth,
            raw_found=bool(raw.found),
            raw_pixel_error_x=float(getattr(raw, "pixel_error_x", 0.0)),
            raw_pixel_error_y=float(getattr(raw, "pixel_error_y", 0.0)),
            raw_angle_degrees=float(getattr(raw, "angle_degrees", 0.0)),
            raw_confidence=float(getattr(raw, "confidence", 0.0)),
            raw_point_count=int(getattr(raw, "point_count", 0)),
            measured_x=measured_x,
            measured_y=measured_y,
            measured_angle_degrees=measured_angle,
            measurement_position_error=measurement_position_error,
            measurement_tangent_error_degrees=measurement_tangent_error,
            belief=belief,
            estimator_position_error=estimator_position_error,
            estimator_tangent_error_degrees=estimator_tangent_error,
            estimator_curvature_magnitude_error=estimator_curvature_error,
            diagnostics_summary=self.diagnostics.summarize(),
        )

    def record_planner_diagnostics(
        self,
        planner: PlannerResult,
        *,
        planner_truth: TruthSample,
        actual_x: Optional[float],
        actual_y: Optional[float],
    ) -> None:
        if not self.diagnostics.samples:
            return
        sample = self.diagnostics.samples[-1]
        if planner.accepted:
            sample.planner_x = float(planner.approved_x)
            sample.planner_y = float(planner.approved_y)
            sample.planner_truth_x = float(planner_truth.truth_x)
            sample.planner_truth_y = float(planner_truth.truth_y)
            sample.actual_x = actual_x
            sample.actual_y = actual_y
        sample.decision_reason = planner.decision_reason
        if planner.safety_action != "PASS":
            event = f"step {sample.step}: {planner.safety_action} — {planner.decision_reason}"
            self.diagnostics.safety_events.append(event)
            if "reverse" in planner.decision_reason.lower():
                self.diagnostics.reverse_events += 1



class SVGSimulatorWindow(tk.Toplevel):
    def __init__(
        self,
        parent: Optional[tk.Misc] = None,
        asset_dir: Optional[Path | str] = None,
        dialog_cls: Optional[type] = None,
        run_dir: Optional[Path | str] = None,
    ) -> None:
        super().__init__(parent)
        self.title("FabScan Simulator — SVG Ground Truth")
        # Keep the initial window inside the usable display area. The simulator
        # used to request 900 px of height unconditionally, which can place the
        # lower controls below the desktop work area on 768/800/900 px displays.
        screen_w = max(900, int(self.winfo_screenwidth()))
        screen_h = max(650, int(self.winfo_screenheight()))
        initial_w = min(1450, max(1050, screen_w - 80))
        initial_h = min(900, max(650, screen_h - 100))
        self.geometry(f"{initial_w}x{initial_h}")
        self.minsize(min(1100, initial_w), min(650, initial_h))
        self.asset_dir = Path(asset_dir or (Path(__file__).resolve().parents[1] / "simulator_assets"))
        self.dialog_cls = dialog_cls
        self.run_dir = Path(run_dir).expanduser().resolve() if run_dir is not None else None
        self.bridge: Optional[VirtualPerceptionEstimatorBridge] = None
        self.analysis: Optional[VirtualFrameAnalysis] = None
        self.analysis_pose: Optional[tuple[float, float, str, float, float, int]] = None
        self.analysis_render_rgb: Optional[np.ndarray] = None
        self.world: Optional[SVGGroundTruthWorld] = None
        self.camera_model: Optional[RealCameraModel] = None
        self.photo_sheet: Optional[ImageTk.PhotoImage] = None
        self.photo_cam: Optional[ImageTk.PhotoImage] = None
        self.shape_var = tk.StringVar(value="G")
        self.x_var = tk.DoubleVar(value=4.25)
        self.y_var = tk.DoubleVar(value=8.0)
        self.ppi_var = tk.DoubleVar(value=635.0)
        self.realism_percent_var = tk.DoubleVar(value=100.0)
        self.realism_seed_var = tk.IntVar(value=5400)
        self.observation_delay_ms_var = tk.DoubleVar(value=120.0)
        self.delay_model_text = tk.StringVar(value="Observation delay: 120 ms; timestamped pose history")
        self.dynamics_enabled_var = tk.BooleanVar(value=True)
        self.accel_x_var = tk.DoubleVar(value=60.0)
        self.accel_y_var = tk.DoubleVar(value=60.0)
        self.max_velocity_ipm_var = tk.DoubleVar(value=700.0)
        self.servo_period_ms_var = tk.DoubleVar(value=4.0)
        self.dynamics_text = tk.StringVar(value="FabLabPlasma dynamics: X/Y 60 in/s², 700 IPM cap, 4 ms servo period")
        self.camera_model_text = tk.StringVar(value="Real-camera model not loaded")
        self.status_var = tk.StringVar(value="Building SVG-derived ground truth…")
        self.truth_text = tk.StringVar(value="Ground truth not loaded")
        self.analysis_text = tk.StringVar(value="Simulator estimator/planner not initialized")
        self.planner_text = tk.StringVar(value="Planner has not run yet")
        self.reverse_initial_var = tk.BooleanVar(value=False)
        self.max_auto_steps_var = tk.IntVar(value=300)
        self.auto_delay_ms_var = tk.IntVar(value=35)
        self.planner: Optional[ConnectedPathPlanner] = None
        self.last_plan: Optional[PlannerResult] = None
        self.plan_pose: Optional[tuple[float, float]] = None
        self.sim_running = False
        self.sim_after_id: Optional[str] = None
        self.sim_step_count = 0
        self.sim_time_s = 0.0
        self.motion_history: list[tuple[float, float, float]] = []
        self.last_observation_pose: Optional[tuple[float, float]] = None
        self.last_observation_time_s = 0.0
        self.last_observation_lag_distance = 0.0
        self.last_move_duration_s = 0.0
        self.last_dynamics_result = None
        self.machine_dynamics = XYMachineDynamics(
            accel_x_in_s2=self.accel_x_var.get(), accel_y_in_s2=self.accel_y_var.get(),
            max_velocity_ipm=self.max_velocity_ipm_var.get(), servo_period_ms=self.servo_period_ms_var.get(),
        )
        self.sim_start_pose: Optional[tuple[float, float]] = None
        self.sim_stop_reason = ""
        self.planner_history: list[dict[str, Any]] = []
        self.auto_save_var = tk.BooleanVar(value=True)
        self.last_saved_run_dir: Optional[Path] = None
        self._auto_saved_signature: Optional[tuple[str, int, str]] = None
        self._build_ui()
        self.after(80, self.load_world)

    def _build_ui(self) -> None:
        """Build the simulator as a working tool rather than a milestone test bench.

        M5.x accumulated controls vertically as each realism layer was added. That
        was fine during development, but it made the simulator awkward to operate.
        M6.3 keeps all model behavior unchanged and reorganizes the same controls
        into Run / Environment / Diagnostics tabs with a resizable camera split.
        """
        top = ttk.Frame(self, padding=(8, 7)); top.pack(fill=tk.X)
        ttk.Label(top, text="SVG Ground-Truth Simulator", font=("TkDefaultFont", 10, "bold")).pack(side=tk.LEFT)
        ttk.Label(top, text="Shape").pack(side=tk.LEFT, padx=(22, 4))
        combo = ttk.Combobox(top, textvariable=self.shape_var, values=list("ABCDEFG"), width=4, state="readonly")
        combo.pack(side=tk.LEFT)
        combo.bind("<<ComboboxSelected>>", lambda _e: self.center_shape())
        ttk.Button(top, text="Center Shape", command=self.center_shape).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(top, text="Exact TestImage.svg truth; planner never receives SVG truth", foreground="#555555").pack(side=tk.RIGHT)

        pane = ttk.Panedwindow(self, orient=tk.HORIZONTAL)
        pane.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        left = ttk.Frame(pane)
        right = ttk.Frame(pane, width=540)
        pane.add(left, weight=3)
        pane.add(right, weight=2)

        lf = ttk.LabelFrame(left, text="SVG Sheet / Virtual Machine Path — click to place camera", padding=5)
        lf.pack(fill=tk.BOTH, expand=True)
        self.sheet = tk.Canvas(lf, bg="#dddddd", highlightthickness=0)
        self.sheet.pack(fill=tk.BOTH, expand=True)
        self.sheet.bind("<Button-1>", self.click_sheet)
        self.sheet.bind("<Configure>", lambda _e: self.draw_sheet())

        # Right side is vertically resizable: camera on top, task-oriented tabs below.
        right_pane = ttk.Panedwindow(right, orient=tk.VERTICAL)
        right_pane.pack(fill=tk.BOTH, expand=True)
        camera_host = ttk.Frame(right_pane)
        controls_host = ttk.Frame(right_pane)
        right_pane.add(camera_host, weight=3)
        right_pane.add(controls_host, weight=2)

        rf = ttk.LabelFrame(camera_host, text="Virtual 800×600 Camera / Planner Overlay", padding=5)
        rf.pack(fill=tk.BOTH, expand=True)
        self.cam = tk.Canvas(rf, bg="#222222", height=320, highlightthickness=0)
        self.cam.pack(fill=tk.BOTH, expand=True)
        self.cam.bind("<Configure>", lambda _e: self.draw_camera())

        notebook = ttk.Notebook(controls_host)
        notebook.pack(fill=tk.BOTH, expand=True, pady=(6, 0))
        run_tab = ttk.Frame(notebook, padding=7)
        env_tab = ttk.Frame(notebook, padding=7)
        diag_tab = ttk.Frame(notebook, padding=7)
        notebook.add(run_tab, text="Run")
        notebook.add(env_tab, text="Environment")
        notebook.add(diag_tab, text="Diagnostics")
        self.sim_notebook = notebook

        # ---- Run tab ----
        pose = ttk.LabelFrame(run_tab, text="Virtual Camera Pose", padding=7)
        pose.pack(fill=tk.X)
        row = ttk.Frame(pose); row.pack(fill=tk.X)
        for label, var in (("X in", self.x_var), ("Y in", self.y_var), ("px/in", self.ppi_var)):
            ttk.Label(row, text=label).pack(side=tk.LEFT, padx=(0, 3))
            entry = ttk.Entry(row, textvariable=var, width=8)
            entry.pack(side=tk.LEFT, padx=(0, 9))
            entry.bind("<Return>", lambda _e: self.refresh())
        ttk.Button(row, text="Refresh", command=self.refresh).pack(side=tk.LEFT)
        nav = ttk.Frame(pose); nav.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(nav, text='Nudge .030"').pack(side=tk.LEFT, padx=(0, 5))
        for label, dx, dy in (("←", -0.03, 0), ("→", 0.03, 0), ("↑", 0, -0.03), ("↓", 0, 0.03)):
            ttk.Button(nav, text=label, width=3, command=lambda a=dx, b=dy: self.nudge(a, b)).pack(side=tk.LEFT, padx=2)

        follow_controls = ttk.LabelFrame(run_tab, text="Virtual Follow", padding=7)
        follow_controls.pack(fill=tk.BOTH, expand=True, pady=(7, 0))
        row1 = ttk.Frame(follow_controls); row1.pack(fill=tk.X)
        self.follow_step_button = ttk.Button(row1, text="Follow Step", command=self.follow_step_once)
        self.follow_step_button.pack(side=tk.LEFT)
        self.run_button = ttk.Button(row1, text="Run Auto Follow", command=self.start_auto_follow)
        self.run_button.pack(side=tk.LEFT, padx=(6, 0))
        self.pause_button = ttk.Button(row1, text="Pause", command=self.pause_auto_follow)
        self.pause_button.pack(side=tk.LEFT, padx=(6, 0))
        ttk.Checkbutton(row1, text="Reverse start", variable=self.reverse_initial_var).pack(side=tk.LEFT, padx=(10, 0))

        row2 = ttk.Frame(follow_controls); row2.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(row2, text="Max steps").pack(side=tk.LEFT)
        ttk.Entry(row2, textvariable=self.max_auto_steps_var, width=6).pack(side=tk.LEFT, padx=(3, 10))
        ttk.Label(row2, text="UI delay ms").pack(side=tk.LEFT)
        ttk.Entry(row2, textvariable=self.auto_delay_ms_var, width=6).pack(side=tk.LEFT, padx=(3, 10))
        ttk.Button(row2, text="Save Run / DXF", command=self.save_simulated_run).pack(side=tk.LEFT)
        ttk.Checkbutton(row2, text="Auto-save", variable=self.auto_save_var).pack(side=tk.LEFT, padx=(8, 0))

        ttk.Separator(follow_controls, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)
        ttk.Label(follow_controls, textvariable=self.planner_text, justify=tk.LEFT,
                  font=("TkFixedFont", 9), wraplength=500).pack(anchor=tk.W, fill=tk.X)

        # ---- Environment tab ----
        realism = ttk.LabelFrame(env_tab, text="Camera Appearance", padding=7)
        realism.pack(fill=tk.X)
        rrow = ttk.Frame(realism); rrow.pack(fill=tk.X)
        ttk.Label(rrow, text="Realism %").pack(side=tk.LEFT)
        ttk.Entry(rrow, textvariable=self.realism_percent_var, width=7).pack(side=tk.LEFT, padx=(3, 10))
        ttk.Label(rrow, text="Seed").pack(side=tk.LEFT)
        ttk.Entry(rrow, textvariable=self.realism_seed_var, width=8).pack(side=tk.LEFT, padx=(3, 10))
        for label, value in (("Perfect", 0.0), ("Measured", 100.0), ("Stress 150", 150.0), ("Stress 200", 200.0)):
            ttk.Button(rrow, text=label, command=lambda v=value: self.set_realism(v)).pack(side=tk.LEFT, padx=2)
        ttk.Label(realism, textvariable=self.camera_model_text, justify=tk.LEFT, wraplength=505).pack(anchor=tk.W, pady=(5, 0))

        delay_box = ttk.LabelFrame(env_tab, text="Observation Timing", padding=7)
        delay_box.pack(fill=tk.X, pady=(7, 0))
        drow = ttk.Frame(delay_box); drow.pack(fill=tk.X)
        ttk.Label(drow, text="Delay ms").pack(side=tk.LEFT)
        delay_entry = ttk.Entry(drow, textvariable=self.observation_delay_ms_var, width=7)
        delay_entry.pack(side=tk.LEFT, padx=(3, 10))
        delay_entry.bind("<Return>", lambda _e: self.set_observation_delay(self.observation_delay_ms_var.get()))
        for label, value in (("0", 0.0), ("Measured 120", 120.0), ("Stress 200", 200.0)):
            ttk.Button(drow, text=label, command=lambda v=value: self.set_observation_delay(v)).pack(side=tk.LEFT, padx=2)
        ttk.Label(delay_box, textvariable=self.delay_model_text, justify=tk.LEFT, wraplength=505).pack(anchor=tk.W, pady=(5, 0))

        dynamics_box = ttk.LabelFrame(env_tab, text="Machine Dynamics", padding=7)
        dynamics_box.pack(fill=tk.X, pady=(7, 0))
        mrow = ttk.Frame(dynamics_box); mrow.pack(fill=tk.X)
        ttk.Checkbutton(mrow, text="Enable", variable=self.dynamics_enabled_var, command=self.reset_estimator).pack(side=tk.LEFT)
        ttk.Button(mrow, text="FabLabPlasma Defaults", command=self._set_fablabplasma_dynamics).pack(side=tk.LEFT, padx=(8, 0))
        mrow2 = ttk.Frame(dynamics_box); mrow2.pack(fill=tk.X, pady=(5, 0))
        for label, var, width in (("X accel", self.accel_x_var, 7), ("Y accel", self.accel_y_var, 7), ("Max IPM", self.max_velocity_ipm_var, 7), ("Servo ms", self.servo_period_ms_var, 6)):
            ttk.Label(mrow2, text=label).pack(side=tk.LEFT, padx=(0, 3))
            ent = ttk.Entry(mrow2, textvariable=var, width=width)
            ent.pack(side=tk.LEFT, padx=(0, 8))
            ent.bind("<Return>", lambda _e: self.reset_estimator())
        ttk.Label(dynamics_box, textvariable=self.dynamics_text, justify=tk.LEFT, wraplength=505).pack(anchor=tk.W, pady=(5, 0))
        ttk.Label(env_tab, text="Model scope: camera appearance + timestamped observation delay + X/Y acceleration limits. Following error, backlash, vibration, and motion blur are intentionally not invented.",
                  justify=tk.LEFT, wraplength=505, foreground="#555555").pack(anchor=tk.W, pady=(8, 0))

        # ---- Diagnostics tab ----
        actions = ttk.Frame(diag_tab); actions.pack(fill=tk.X)
        self.analyze_button = ttk.Button(actions, text="Analyze Current Frame", command=self.analyze_virtual_frame)
        self.analyze_button.pack(side=tk.LEFT)
        ttk.Button(actions, text="Reset Follow State", command=self.reset_estimator).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(actions, text="Post-Run Summary…", command=self.show_estimator_summary).pack(side=tk.LEFT, padx=(6, 0))

        # Diagnostics can be long. Keep only this tab scrollable instead of making
        # every simulator control live in one giant scrolling column.
        diag_host = ttk.Frame(diag_tab); diag_host.pack(fill=tk.BOTH, expand=True, pady=(7, 0))
        diag_canvas = tk.Canvas(diag_host, highlightthickness=0, borderwidth=0)
        diag_scroll = ttk.Scrollbar(diag_host, orient=tk.VERTICAL, command=diag_canvas.yview)
        diag_canvas.configure(yscrollcommand=diag_scroll.set)
        diag_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        diag_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        diag_body = ttk.Frame(diag_canvas)
        diag_window = diag_canvas.create_window((0, 0), window=diag_body, anchor=tk.NW)
        diag_body.bind("<Configure>", lambda _e: diag_canvas.configure(scrollregion=diag_canvas.bbox("all")))
        diag_canvas.bind("<Configure>", lambda e: diag_canvas.itemconfigure(diag_window, width=max(1, e.width)))

        truth_box = ttk.LabelFrame(diag_body, text="SVG Truth / Current Pose", padding=7)
        truth_box.pack(fill=tk.X)
        ttk.Label(truth_box, textvariable=self.truth_text, justify=tk.LEFT, font=("TkFixedFont", 9), wraplength=500).pack(anchor=tk.W)
        analysis_box = ttk.LabelFrame(diag_body, text="Perception / Belief", padding=7)
        analysis_box.pack(fill=tk.X, pady=(7, 0))
        ttk.Label(analysis_box, textvariable=self.analysis_text, justify=tk.LEFT, font=("TkFixedFont", 9), wraplength=500).pack(anchor=tk.W)
        note = ttk.LabelFrame(diag_body, text="Safety Boundary", padding=7)
        note.pack(fill=tk.X, pady=(7, 0))
        ttk.Label(note, justify=tk.LEFT, wraplength=500,
            text="The connected-path planner has motion authority only over the virtual camera. It sees rendered camera evidence and persistent belief, never SVG truth. Every move remains bounded by the separate reversal/length safety gate; no LinuxCNC command API is used.").pack(anchor=tk.W)

        def _diag_wheel(event) -> str:
            delta = int(getattr(event, "delta", 0))
            if delta:
                diag_canvas.yview_scroll((-1 if delta > 0 else 1) * 3, "units")
            return "break"
        diag_canvas.bind("<MouseWheel>", _diag_wheel)
        diag_body.bind("<MouseWheel>", _diag_wheel)

        ttk.Label(self, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W, padding=4).pack(fill=tk.X, side=tk.BOTTOM)

    def load_world(self) -> None:
        try:
            self.world = SVGGroundTruthWorld(self.asset_dir)
            self.camera_model = RealCameraModel(self.asset_dir)
            self.camera_model_text.set(self.camera_model.summary())
        except Exception as exc:
            messagebox.showerror("Simulator could not load", str(exc), parent=self)
            self.status_var.set(f"Ground-truth load failed: {exc}")
            return
        if self.dialog_cls is not None and self.run_dir is not None:
            try:
                self.bridge = VirtualPerceptionEstimatorBridge(self.dialog_cls, self.run_dir, self.ppi_var.get())
                self.planner = ConnectedPathPlanner(self.bridge.calibration, self.bridge.settings)
                self.analysis_text.set(f"Estimator/planner ready. Detector settings source: {self.run_dir.name}\nPlace the virtual camera, then Analyze or start Virtual Follow.")
            except Exception as exc:
                self.bridge = None
                self.analysis_text.set(f"Estimator bridge failed to initialize: {type(exc).__name__}: {exc}")
                self.analyze_button.configure(state=tk.DISABLED)
        else:
            self.analyze_button.configure(state=tk.DISABLED)
            self.analysis_text.set("No replay/settings source or CameraCalibrationDialog class was supplied. Ground-truth camera remains usable, but Simulator perception/planning is disabled.")
        self.status_var.set(
            f"Ground truth ready: {self.world.width_in:.3f} × {self.world.height_in:.3f} in, A–G centerlines sampled at 1/{self.world.world_ppi} in. Simulator uses the selected replay's saved camera calibration plus an independently adjustable delayed-observation model."
        )
        self.center_shape()

    def center_shape(self) -> None:
        if self.world is None: return
        self.pause_auto_follow()
        x, y = self.world.shape_center(self.shape_var.get()); self.x_var.set(x); self.y_var.set(y)
        self.reset_estimator(silent=True)
        self._invalidate_analysis()
        self.refresh()

    def nudge(self, dx: float, dy: float) -> None:
        self.pause_auto_follow()
        self.x_var.set(self.x_var.get() + dx); self.y_var.set(self.y_var.get() + dy)
        if self.planner is not None:
            self.planner.record_executed_pose(self.x_var.get(), self.y_var.get())
        self._invalidate_analysis()
        self.refresh()

    def refresh(self) -> None:
        self._invalidate_analysis_if_pose_changed()
        self.draw_sheet(); self.draw_camera()

    def click_sheet(self, event: tk.Event) -> None:
        if self.world is None: return
        cw = max(1, self.sheet.winfo_width()); ch = max(1, self.sheet.winfo_height())
        scale = min(cw / self.world.width_in, ch / self.world.height_in)
        ox = (cw - self.world.width_in * scale) / 2; oy = (ch - self.world.height_in * scale) / 2
        x = (event.x - ox) / scale; y = (event.y - oy) / scale
        if 0 <= x <= self.world.width_in and 0 <= y <= self.world.height_in:
            self.pause_auto_follow()
            self.x_var.set(x); self.y_var.set(y)
            # Clicking the sheet is treated as a teleport/setup action, not a
            # plausible motion sample. Start a clean belief/planner at that pose.
            self.reset_estimator(silent=True)
            self._invalidate_analysis(); self.refresh()

    def _invalidate_analysis(self) -> None:
        self.analysis = None
        self.analysis_pose = None
        self.analysis_render_rgb = None

    def _invalidate_analysis_if_pose_changed(self) -> None:
        if self.analysis_pose is None:
            return
        pose = (float(self.x_var.get()), float(self.y_var.get()), self.shape_var.get(), float(self.ppi_var.get()), float(self.realism_percent_var.get()), int(self.realism_seed_var.get()), float(self.observation_delay_ms_var.get()))
        if any(abs(float(a) - float(b)) > 1e-9 for a, b in zip(pose[:2], self.analysis_pose[:2])) or pose[2:7] != self.analysis_pose[2:7]:
            self._invalidate_analysis()

    def set_realism(self, value: float) -> None:
        self.realism_percent_var.set(float(value))
        # Do not mix estimator history generated under different sensor models.
        self.reset_estimator(silent=True)
        self.refresh()
        self.status_var.set(f"Simulator camera realism set to {float(value):.0f}%; follow state reset for a clean comparison.")

    def set_observation_delay(self, value: float) -> None:
        value = max(0.0, float(value))
        self.observation_delay_ms_var.set(value)
        self.delay_model_text.set(
            f"Observation delay: {value:.0f} ms; virtual time {self.sim_time_s:.3f} s; "
            f"last lag {self.last_observation_lag_distance:.4f} in"
        )
        # Timing history is part of estimator state. Never splice two delay models
        # into one run; restart at the current virtual pose for a clean comparison.
        self.reset_estimator(silent=True)
        self.refresh()
        self.status_var.set(f"Simulator observation delay set to {value:.0f} ms; follow state reset.")

    def _set_fablabplasma_dynamics(self) -> None:
        self.accel_x_var.set(60.0)
        self.accel_y_var.set(60.0)
        self.max_velocity_ipm_var.set(700.0)
        self.servo_period_ms_var.set(4.0)
        self.dynamics_enabled_var.set(True)
        self.reset_estimator(silent=True)
        self.refresh()
        self.status_var.set("Simulator machine dynamics reset to FabLabPlasma.ini values; follow state reset.")

    def _rebuild_machine_dynamics(self) -> None:
        self.machine_dynamics = XYMachineDynamics(
            accel_x_in_s2=max(0.001, float(self.accel_x_var.get())),
            accel_y_in_s2=max(0.001, float(self.accel_y_var.get())),
            max_velocity_ipm=max(1.0, float(self.max_velocity_ipm_var.get())),
            servo_period_ms=max(0.1, float(self.servo_period_ms_var.get())),
        )
        self.last_dynamics_result = None
        enabled = bool(self.dynamics_enabled_var.get())
        self.dynamics_text.set(
            f"{'ENABLED' if enabled else 'DISABLED'} — X {self.machine_dynamics.accel_x_in_s2:.1f} / Y {self.machine_dynamics.accel_y_in_s2:.1f} in/s², "
            f"{self.machine_dynamics.max_velocity_ipm:.0f} IPM cap, {self.machine_dynamics.servo_period_ms:.1f} ms servo; current speed 0.0 IPM"
        )

    def _reset_timing_history(self) -> None:
        self.sim_time_s = 0.0
        x = float(self.x_var.get()); y = float(self.y_var.get())
        self.motion_history = [(0.0, x, y)]
        self.last_observation_pose = (x, y)
        self.last_observation_time_s = 0.0
        self.last_observation_lag_distance = 0.0
        self.last_move_duration_s = 0.0
        self.delay_model_text.set(
            f"Observation delay: {float(self.observation_delay_ms_var.get()):.0f} ms; virtual time 0.000 s; last lag 0.0000 in"
        )

    def _interpolated_pose(self, query_time_s: float) -> tuple[float, float]:
        if not self.motion_history:
            return float(self.x_var.get()), float(self.y_var.get())
        t = max(0.0, float(query_time_s))
        if t <= self.motion_history[0][0]:
            return float(self.motion_history[0][1]), float(self.motion_history[0][2])
        if t >= self.motion_history[-1][0]:
            return float(self.motion_history[-1][1]), float(self.motion_history[-1][2])
        for i in range(1, len(self.motion_history)):
            t1, x1, y1 = self.motion_history[i]
            if t <= t1:
                t0, x0, y0 = self.motion_history[i - 1]
                span = max(1e-12, float(t1) - float(t0))
                u = max(0.0, min(1.0, (t - float(t0)) / span))
                return float(x0 + (x1 - x0) * u), float(y0 + (y1 - y0) * u)
        return float(self.motion_history[-1][1]), float(self.motion_history[-1][2])

    def _observation_pose(self) -> tuple[float, float, float, float]:
        delay_s = max(0.0, float(self.observation_delay_ms_var.get())) / 1000.0
        observation_time = max(0.0, float(self.sim_time_s) - delay_s)
        ox, oy = self._interpolated_pose(observation_time)
        lag = math.hypot(float(self.x_var.get()) - ox, float(self.y_var.get()) - oy)
        self.last_observation_pose = (ox, oy)
        self.last_observation_time_s = observation_time
        self.last_observation_lag_distance = lag
        self.delay_model_text.set(
            f"Observation delay: {float(self.observation_delay_ms_var.get()):.0f} ms; "
            f"virtual time {self.sim_time_s:.3f} s; frame time {observation_time:.3f} s; lag {lag:.4f} in"
        )
        return ox, oy, observation_time, lag

    def _render_virtual_rgb(
        self,
        *,
        for_analysis: bool = False,
        camera_x: Optional[float] = None,
        camera_y: Optional[float] = None,
        frame_index: Optional[int] = None,
    ) -> np.ndarray:
        if self.world is None:
            raise RuntimeError("ground-truth world is not ready")
        cx = float(self.x_var.get() if camera_x is None else camera_x)
        cy = float(self.y_var.get() if camera_y is None else camera_y)
        if self.bridge is not None:
            image = self.world.render_camera_calibrated(cx, cy, self.bridge.calibration, 800, 600)
        else:
            image = self.world.render_camera(cx, cy, 800, 600, self.ppi_var.get())
        rgb = np.asarray(image.convert("RGB"))
        if self.camera_model is not None:
            if frame_index is None:
                frame_index = (self.bridge.step_index + 1) if self.bridge is not None else (self.sim_step_count + 1)
            rgb = self.camera_model.apply(
                rgb, realism_percent=self.realism_percent_var.get(), frame_index=int(frame_index),
                seed=self.realism_seed_var.get(), camera_x=cx, camera_y=cy,
            )
        if for_analysis:
            self.analysis_render_rgb = np.asarray(rgb, dtype=np.uint8).copy()
        return np.asarray(rgb, dtype=np.uint8)

    def reset_estimator(self, silent: bool = False) -> None:
        self.pause_auto_follow()
        self._invalidate_analysis()
        self.last_plan = None
        self.plan_pose = None
        self.sim_step_count = 0
        self.sim_time_s = 0.0
        self.motion_history: list[tuple[float, float, float]] = []
        self.last_observation_pose: Optional[tuple[float, float]] = None
        self.last_observation_time_s = 0.0
        self.last_observation_lag_distance = 0.0
        self.last_move_duration_s = 0.0
        self.sim_start_pose = None
        self.sim_stop_reason = ""
        self.planner_history.clear()
        self._rebuild_machine_dynamics()
        self._reset_timing_history()
        self.last_saved_run_dir = None
        self._auto_saved_signature = None
        if self.bridge is None:
            if not silent:
                self.status_var.set("Simulator estimator bridge is not available.")
            return
        try:
            self.bridge.reset(self.ppi_var.get())
            if self.planner is None:
                self.planner = ConnectedPathPlanner(self.bridge.calibration, self.bridge.settings)
            self.planner.reset(reverse_initial=bool(self.reverse_initial_var.get()))
            self.planner.record_executed_pose(self.x_var.get(), self.y_var.get())
            self.sim_start_pose = (float(self.x_var.get()), float(self.y_var.get()))
            self.analysis_text.set(
                f"Follow state reset. Detector settings source: {self.bridge.run_dir.name}\n"
                "Analyze manually, use Follow Step, or start Auto Follow."
            )
            self.planner_text.set("Planner reset; waiting for first analyzed frame.")
            if not silent:
                self.status_var.set("Simulator estimator + planner + timing/dynamics history cleared at the current virtual-camera position.")
            self.refresh()
        except Exception as exc:
            self.status_var.set(f"Follow-state reset failed: {exc}")
            if not silent:
                messagebox.showerror("Simulator follow-state reset", f"{type(exc).__name__}: {exc}", parent=self)


    def _analyze_current_frame(self) -> tuple[np.ndarray, VirtualFrameAnalysis]:
        if self.world is None or self.bridge is None:
            raise RuntimeError("Simulator perception/estimator bridge is not ready")
        if abs(float(self.bridge.camera_ppi) - float(self.ppi_var.get())) > 1e-9:
            self.reset_estimator(silent=True)
        obs_x, obs_y, obs_time, lag = self._observation_pose()
        rgb = self._render_virtual_rgb(
            for_analysis=True, camera_x=obs_x, camera_y=obs_y, frame_index=self.bridge.step_index + 1
        )
        frame_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        truth = self.world.nearest(obs_x, obs_y, self.shape_var.get())
        result = self.bridge.analyze(
            frame_bgr, camera_x=obs_x, camera_y=obs_y, truth=truth
        )
        # Simulator diagnostics are delay-aware and geometry-aware: compare the
        # measured/estimated world point to the nearest SVG profile at that point,
        # not to the profile point nearest the *current* controller pose. This
        # removes along-path separation caused by a deliberate observation delay.
        if self.bridge.diagnostics.samples:
            sample = self.bridge.diagnostics.samples[-1]
            if result.measured_x is not None and result.measured_y is not None:
                mt = self.world.nearest(result.measured_x, result.measured_y, self.shape_var.get())
                sample.measured_cross_track_error = float(mt.distance)
                sample.measured_truth_tangent_deg = _align_axis_angle_to_reference(mt.tangent_degrees, result.measured_angle_degrees if result.measured_angle_degrees is not None else mt.tangent_degrees)
            if result.belief.measurement_valid:
                et = self.world.nearest(result.belief.estimate_x, result.belief.estimate_y, self.shape_var.get())
                sample.estimate_cross_track_error = float(et.distance)
                sample.estimate_truth_tangent_deg = _align_axis_angle_to_reference(et.tangent_degrees, result.belief.tangent_degrees)
        self.analysis = result
        self.analysis_pose = (
            float(self.x_var.get()), float(self.y_var.get()), self.shape_var.get(), float(self.ppi_var.get()),
            float(self.realism_percent_var.get()), int(self.realism_seed_var.get()),
            float(self.observation_delay_ms_var.get()), float(obs_x), float(obs_y), float(obs_time), float(lag),
        )
        self._update_analysis_text(result)
        return frame_bgr, result

    def analyze_virtual_frame(self) -> None:
        try:
            _frame, result = self._analyze_current_frame()
            self.draw_camera()
            self.status_var.set(
                f"Simulator virtual step {result.step}: delayed SVG frame analyzed by real FabScan perception; M4 belief updated. Planner not executed."
            )
        except Exception as exc:
            self.status_var.set(f"Virtual analysis failed: {exc}")
            messagebox.showerror("Analyze Virtual Frame", f"{type(exc).__name__}: {exc}", parent=self)

    def _ensure_planner_ready(self) -> None:
        if self.bridge is None:
            raise RuntimeError("perception bridge is not ready")
        if self.planner is None:
            self.planner = ConnectedPathPlanner(self.bridge.calibration, self.bridge.settings)
            self.planner.reset(reverse_initial=bool(self.reverse_initial_var.get()))
            self.planner.record_executed_pose(self.x_var.get(), self.y_var.get())
            self.sim_start_pose = (float(self.x_var.get()), float(self.y_var.get()))

    def _update_planner_text(self, plan: PlannerResult) -> None:
        state = "MOVE" if plan.accepted else "HOLD"
        self.planner_text.set(
            f"Planner:               {state}   safety {plan.safety_action}\n"
            f"Move:                  X {plan.move_x:+.4f}  Y {plan.move_y:+.4f}  len {plan.move_length:.4f} in\n"
            f"Connected look-ahead:  {plan.usable_lookahead:.4f} in   steering {plan.steering_lookahead:.4f} in\n"
            f"Visible path turn:     {plan.path_turn_degrees:.1f}°   candidates {plan.candidate_count}\n"
            f"Planner confidence:    {plan.planner_confidence:.1f}%\n"
            f"Requested velocity:    {plan.recommended_velocity:.1f} IPM\n"
            f"Actual machine speed:  {self.machine_dynamics.speed_ipm:.1f} IPM  dynamics {'ON' if self.dynamics_enabled_var.get() else 'OFF'}\n"
            f"Reason: {plan.decision_reason}"
        )

    def follow_step_once(self, *, from_auto: bool = False) -> bool:
        if self.world is None or self.bridge is None:
            self.status_var.set("Simulator simulator/perception bridge is not ready.")
            return False
        try:
            self._ensure_planner_ready()
            frame_bgr, result = self._analyze_current_frame()
            assert self.planner is not None
            frame_camera_x = float(result.camera_x)
            frame_camera_y = float(result.camera_y)
            plan = self.planner.plan(
                frame_bgr, camera_x=self.x_var.get(), camera_y=self.y_var.get(), belief=result.belief,
                frame_camera_x=frame_camera_x, frame_camera_y=frame_camera_y,
            )
            self.last_plan = plan
            self.plan_pose = (float(self.x_var.get()), float(self.y_var.get()))
            self._update_planner_text(plan)
            self.sim_step_count += 1
            plan_record = plan.to_dict()
            plan_record["sim_step"] = int(self.sim_step_count)
            plan_record["shape"] = self.shape_var.get()
            plan_record["camera_realism_percent"] = float(self.realism_percent_var.get())
            plan_record["camera_realism_seed"] = int(self.realism_seed_var.get())
            plan_record["observation_delay_ms"] = float(self.observation_delay_ms_var.get())
            plan_record["sim_time_s_before_move"] = float(self.sim_time_s)
            plan_record["observation_time_s"] = float(self.last_observation_time_s)
            plan_record["observation_pose"] = [float(frame_camera_x), float(frame_camera_y)]
            plan_record["current_pose"] = [float(self.x_var.get()), float(self.y_var.get())]
            plan_record["observation_lag_distance_in"] = float(self.last_observation_lag_distance)
            plan_record["belief"] = result.belief.to_dict() if hasattr(result.belief, "to_dict") else {}
            self.planner_history.append(plan_record)

            if not plan.accepted:
                current_truth = self.world.nearest(self.x_var.get(), self.y_var.get(), self.shape_var.get())
                self.bridge.record_planner_diagnostics(plan, planner_truth=current_truth, actual_x=None, actual_y=None)
                self.sim_stop_reason = plan.decision_reason
                self.pause_auto_follow()
                self.draw_sheet(); self.draw_camera()
                self.status_var.set(f"Simulator virtual follow HOLD at step {self.sim_step_count}: {plan.decision_reason}")
                self._maybe_auto_save()
                return False

            target_truth = self.world.nearest(plan.approved_x, plan.approved_y, self.shape_var.get())
            speed_ipm = max(1.0, min(float(plan.recommended_velocity), float(self.max_velocity_ipm_var.get())))
            # One planner interval is the time the ideal step would consume at the
            # requested velocity. With dynamics enabled, the virtual machine may
            # travel less/more than the safety-approved geometric target because
            # velocity can only change at the configured LinuxCNC acceleration.
            move_duration_s = max(
                float(self.machine_dynamics.servo_period_s),
                60.0 * float(plan.move_length) / max(speed_ipm, 1e-9),
            )
            start_time = float(self.sim_time_s)
            current_x = float(self.x_var.get()); current_y = float(self.y_var.get())
            if bool(self.dynamics_enabled_var.get()):
                dyn = self.machine_dynamics.advance(
                    x=current_x, y=current_y, direction_x=plan.move_x, direction_y=plan.move_y,
                    requested_speed_ipm=speed_ipm, duration_s=move_duration_s,
                )
                actual_x, actual_y = float(dyn.end_x), float(dyn.end_y)
                self.last_dynamics_result = dyn
                for rel_t, px, py, _vx, _vy in self.machine_dynamics.last_samples[1:]:
                    self.motion_history.append((start_time + float(rel_t), float(px), float(py)))
                plan_record["machine_dynamics"] = dyn.to_dict()
            else:
                actual_x, actual_y = float(plan.approved_x), float(plan.approved_y)
                self.machine_dynamics.vx = float(plan.move_x / max(plan.move_length, 1e-12)) * (speed_ipm / 60.0)
                self.machine_dynamics.vy = float(plan.move_y / max(plan.move_length, 1e-12)) * (speed_ipm / 60.0)
                self.motion_history.append((start_time + move_duration_s, actual_x, actual_y))
                plan_record["machine_dynamics"] = {"enabled": False, "actual_equals_planner_target": True}
            self.last_move_duration_s = move_duration_s
            self.sim_time_s = start_time + move_duration_s
            self.x_var.set(actual_x); self.y_var.set(actual_y)
            self.planner.record_executed_pose(actual_x, actual_y)
            self.bridge.record_planner_diagnostics(
                plan, planner_truth=target_truth, actual_x=actual_x, actual_y=actual_y
            )
            actual_truth = self.world.nearest(actual_x, actual_y, self.shape_var.get())
            if self.bridge.diagnostics.samples:
                ds = self.bridge.diagnostics.samples[-1]
                ds.dynamics_enabled = bool(self.dynamics_enabled_var.get())
                ds.requested_velocity_ipm = float(speed_ipm)
                ds.actual_velocity_ipm = float(self.machine_dynamics.speed_ipm)
                ds.actual_cross_track_error = float(actual_truth.distance)
                if self.last_dynamics_result is not None:
                    ds.acceleration_limited = bool(self.last_dynamics_result.accel_limited)
                    ds.velocity_limited = bool(self.last_dynamics_result.velocity_limited)
            plan_record["recommended_velocity_ipm"] = speed_ipm
            plan_record["actual_velocity_ipm_after"] = float(self.machine_dynamics.speed_ipm)
            plan_record["actual_pose_after"] = [actual_x, actual_y]
            plan_record["actual_cross_track_error_in"] = float(actual_truth.distance)
            plan_record["move_duration_s"] = move_duration_s
            plan_record["sim_time_s_after_move"] = float(self.sim_time_s)
            if self.last_dynamics_result is not None:
                self.dynamics_text.set(
                    f"ENABLED — requested {speed_ipm:.1f} IPM; actual {self.machine_dynamics.speed_ipm:.1f} IPM; "
                    f"X/Y accel {self.machine_dynamics.accel_x_in_s2:.1f}/{self.machine_dynamics.accel_y_in_s2:.1f} in/s²; "
                    f"tracking {math.hypot(actual_x-plan.approved_x, actual_y-plan.approved_y):.4f} in"
                )

            # Non-ground-truth loop closure: compare the actual virtual machine pose
            # with its own start after meaningful travel.
            if self.sim_start_pose is None:
                self.sim_start_pose = self.planner.camera_history[0] if self.planner.camera_history else (plan.camera_x, plan.camera_y)
            if self.planner.total_distance > 0.75 and self.sim_start_pose is not None:
                closure = math.hypot(actual_x - self.sim_start_pose[0], actual_y - self.sim_start_pose[1])
                if closure < 0.045:
                    self.sim_stop_reason = f"loop closure: returned within {closure:.4f} in of start after {self.planner.total_distance:.3f} in"
                    self.pause_auto_follow()

            self._invalidate_analysis()
            self.refresh()
            if self.sim_stop_reason:
                self.status_var.set(f"Simulator virtual follow complete: {self.sim_stop_reason}")
                self._maybe_auto_save()
                return False
            self.status_var.set(
                f"Simulator virtual follow step {self.sim_step_count}: moved {plan.move_length:.4f} in; "
                f"look-ahead {plan.usable_lookahead:.3f} in; lag {self.last_observation_lag_distance:.4f} in; "
                f"planner confidence {plan.planner_confidence:.1f}%."
            )
            return True
        except Exception as exc:
            self.pause_auto_follow()
            self.sim_stop_reason = f"runtime error: {type(exc).__name__}: {exc}"
            self.status_var.set(f"Virtual follow failed: {exc}")
            self._maybe_auto_save()
            if not from_auto:
                messagebox.showerror("Simulator Virtual Follow", f"{type(exc).__name__}: {exc}", parent=self)
            return False

    def start_auto_follow(self) -> None:
        if self.sim_running:
            return
        if self.planner is None or self.sim_step_count == 0:
            self.reset_estimator(silent=True)
        self.sim_stop_reason = ""
        self.sim_running = True
        self.run_button.configure(state=tk.DISABLED)
        self.status_var.set("Simulator automatic virtual follow running…")
        self._auto_follow_tick()

    def _auto_follow_tick(self) -> None:
        if not self.sim_running:
            return
        max_steps = max(1, int(self.max_auto_steps_var.get()))
        if self.sim_step_count >= max_steps:
            self.sim_stop_reason = f"max-step limit {max_steps} reached"
            self.pause_auto_follow()
            self.status_var.set(f"Simulator virtual follow stopped: {self.sim_stop_reason}")
            self._maybe_auto_save()
            return
        keep_going = self.follow_step_once(from_auto=True)
        if not keep_going or not self.sim_running:
            return
        delay = max(1, int(self.auto_delay_ms_var.get()))
        self.sim_after_id = self.after(delay, self._auto_follow_tick)

    def pause_auto_follow(self) -> None:
        self.sim_running = False
        if self.sim_after_id is not None:
            try:
                self.after_cancel(self.sim_after_id)
            except Exception:
                pass
            self.sim_after_id = None
        if hasattr(self, "run_button"):
            self.run_button.configure(state=tk.NORMAL)


    def _default_simulated_runs_root(self) -> Path:
        if self.run_dir is not None and self.run_dir.parent.name == "replays":
            return self.run_dir.parent.parent / "simulated_runs"
        if self.run_dir is not None:
            return self.run_dir.parent / "simulated_runs"
        return Path.home() / "FabScan_simulated_runs"

    def _maybe_auto_save(self) -> None:
        if not bool(self.auto_save_var.get()) or self.sim_step_count <= 0 or not self.sim_stop_reason:
            return
        signature = (self.shape_var.get(), int(self.sim_step_count), str(self.sim_stop_reason))
        if signature == self._auto_saved_signature:
            return
        try:
            self.save_simulated_run(auto=True)
            self._auto_saved_signature = signature
        except Exception as exc:
            self.status_var.set(f"Simulator auto-save failed: {exc}")

    def save_simulated_run(self, auto: bool = False) -> Optional[Path]:
        if self.world is None or self.bridge is None or self.planner is None:
            if not auto:
                messagebox.showinfo("Save Simulated Run", "Run/estimator data is not available yet.", parent=self)
            return None
        if self.sim_step_count <= 0 or len(self.planner.camera_history) < 2:
            if not auto:
                messagebox.showinfo("Save Simulated Run", "Run at least one Follow Step before saving.", parent=self)
            return None
        root = self._default_simulated_runs_root()
        run_dir = export_simulated_run(
            output_root=root,
            shape=self.shape_var.get(),
            world=self.world,
            planner=self.planner,
            diagnostics=self.bridge.diagnostics,
            planner_history=self.planner_history,
            source_replay=self.run_dir,
            sim_step_count=self.sim_step_count,
            stop_reason=self.sim_stop_reason,
            integration_version=INTEGRATION_VERSION,
            settings=self.bridge.settings,
            calibration=self.bridge.calibration,
            camera_realism={
                "percent": float(self.realism_percent_var.get()),
                "seed": int(self.realism_seed_var.get()),
                "profile": dict(self.camera_model.profile) if self.camera_model is not None else {},
            },
            timing_model={
                "observation_delay_ms": float(self.observation_delay_ms_var.get()),
                "virtual_time_s": float(self.sim_time_s),
                "last_observation_time_s": float(self.last_observation_time_s),
                "last_observation_lag_distance_in": float(self.last_observation_lag_distance),
                "time_basis": "planner interval from requested IPM; pose integrated at configured servo period",
                "acceleration_model": bool(self.dynamics_enabled_var.get()),
                "controller_lag_model": False,
                "machine_dynamics": self.machine_dynamics.config_dict(),
                "current_velocity_ipm": float(self.machine_dynamics.speed_ipm),
            },
        )
        self.last_saved_run_dir = run_dir
        self.status_var.set(f"Simulator simulated run saved: {run_dir}")
        if not auto:
            messagebox.showinfo(
                "Simulator simulated run saved",
                "Saved run package with CAD-ready Y-up DXF layers:\n"
                "  SVG_TRUTH\n  CAMERA_PATH\n  M4_BELIEF\n\n"
                f"Folder:\n{run_dir}",
                parent=self,
            )
        return run_dir

    @staticmethod
    def _fmt_optional(value: Optional[float], fmt: str = ".5f", suffix: str = "") -> str:
        return "—" if value is None else f"{value:{fmt}}{suffix}"

    def _update_analysis_text(self, result: VirtualFrameAnalysis) -> None:
        b = result.belief
        measured_xy = "—" if result.measured_x is None else f"X {result.measured_x:.4f}  Y {result.measured_y:.4f}"
        estimate_xy = "—" if not b.measurement_valid else f"X {b.estimate_x:.4f}  Y {b.estimate_y:.4f}"
        radius = "∞" if b.implied_radius <= 0.0 else f"{b.implied_radius:.3f} in"
        self.analysis_text.set(
            f"Virtual step:          {result.step}\n"
            f"Virtual time:          {self.sim_time_s:.4f} s  frame time {self.last_observation_time_s:.4f} s\n"
            f"Observation delay:     {float(self.observation_delay_ms_var.get()):.0f} ms  spatial lag {self.last_observation_lag_distance:.5f} in\n"
            f"Raw detector:          {'FOUND' if result.raw_found else 'NOT FOUND'}  conf {result.raw_confidence:.1f}%  points {result.raw_point_count}\n"
            f"Raw pixel error:       X {result.raw_pixel_error_x:+.2f}  Y {result.raw_pixel_error_y:+.2f} px  image angle {result.raw_angle_degrees:+.2f}°\n"
            f"Measured profile:      {measured_xy}\n"
            f"Measured world tangent:{self._fmt_optional(result.measured_angle_degrees, '.2f', '°')}\n"
            f"Truth profile:         X {result.truth.truth_x:.4f}  Y {result.truth.truth_y:.4f}\n"
            f"Measurement pos error: {self._fmt_optional(result.measurement_position_error)} in\n"
            f"Measurement tan error: {self._fmt_optional(result.measurement_tangent_error_degrees, '.2f', '°')} (axis)\n"
            f"M4 estimate:           {estimate_xy}\n"
            f"Estimator pos error:   {self._fmt_optional(result.estimator_position_error)} in\n"
            f"Estimator tan error:   {self._fmt_optional(result.estimator_tangent_error_degrees, '.2f', '°')} (axis)\n"
            f"Estimator curvature:   {b.curvature:+.5f} 1/in  radius {radius}\n"
            f"|curvature| error*:    {self._fmt_optional(result.estimator_curvature_magnitude_error, '.5f')} 1/in\n"
            f"Usable look-ahead:     {b.usable_lookahead:.4f} in\n"
            f"Prediction horizon:    {b.prediction_horizon:.4f} in\n"
            f"Belief confidence:     {b.belief_confidence:.1f}%\n"
            f"Innovation:            pos {b.position_innovation:.5f} in  heading {b.heading_innovation_degrees:+.2f}°\n"
            f"*truth curvature is a local centerline fit; magnitude is compared because tangent sign is not directional yet."
        )

    def show_estimator_summary(self) -> None:
        if self.bridge is None:
            messagebox.showinfo("Simulator diagnostic summary", "Estimator bridge is not available.", parent=self)
            return
        summary = self.bridge.diagnostics.summarize()
        perception = summary.get("perception_position_error_in", {})
        pos = summary.get("estimator_position_error_in", {})
        tangent = summary.get("estimator_tangent_error_deg", {})
        planner = summary.get("planner_truth_error_in", {})
        controller = summary.get("controller_tracking_error_in", {})
        diagnosis = summary.get("diagnosis", [])
        text = (
            f"Analyzed samples: {summary.get('samples', 0)}\n"
            f"Virtual follow steps: {self.sim_step_count}\n"
            f"Stop reason: {self.sim_stop_reason or '—'}\n"
            f"Observation delay: {float(self.observation_delay_ms_var.get()):.0f} ms\n"
            f"Virtual time: {self.sim_time_s:.4f} s   Last spatial lag: {self.last_observation_lag_distance:.5f} in\n\n"
            f"Perception position error\n"
            f"  RMS: {self._fmt_optional(perception.get('rms'))} in   Max: {self._fmt_optional(perception.get('max'))} in\n\n"
            f"Estimator position error\n"
            f"  RMS: {self._fmt_optional(pos.get('rms'))} in   Max: {self._fmt_optional(pos.get('max'))} in\n"
            f"Estimator tangent RMS: {self._fmt_optional(tangent.get('rms'), '.3f')}°\n\n"
            f"Planner target cross-track error\n"
            f"  RMS: {self._fmt_optional(planner.get('rms'))} in   Max: {self._fmt_optional(planner.get('max'))} in\n\n"
            f"Controller tracking error\n"
            f"  RMS: {self._fmt_optional(controller.get('rms'))} in   Max: {self._fmt_optional(controller.get('max'))} in\n\n"
            f"Reverse safety events: {summary.get('reverse_events', 0)}\n"
            f"Safety interventions: {len(summary.get('safety_events', []))}\n\n"
            f"Likely layer: {summary.get('likely_layer', '—')}\n" +
            "\n".join(f"• {line}" for line in diagnosis)
        )
        messagebox.showinfo("Simulator post-run self-diagnostic summary", text, parent=self)


    def draw_sheet(self) -> None:
        c = self.sheet; c.delete("all")
        if self.world is None: return
        cw = max(100, c.winfo_width()); ch = max(100, c.winfo_height())
        scale = min(cw / self.world.width_in, ch / self.world.height_in)
        width = max(1, int(self.world.width_in * scale)); height = max(1, int(self.world.height_in * scale))
        image = Image.fromarray(self.world.rgb).resize((width, height), Image.Resampling.LANCZOS)
        self.photo_sheet = ImageTk.PhotoImage(image); ox = (cw - width) / 2; oy = (ch - height) / 2
        c.create_image(ox, oy, image=self.photo_sheet, anchor=tk.NW)
        px_per_in = self.ppi_var.get(); fov_w = 800 / px_per_in * scale; fov_h = 600 / px_per_in * scale
        cx = ox + self.x_var.get() * scale; cy = oy + self.y_var.get() * scale
        c.create_rectangle(cx - fov_w / 2, cy - fov_h / 2, cx + fov_w / 2, cy + fov_h / 2, outline="#ff4040", width=2)
        c.create_line(cx - 8, cy, cx + 8, cy, fill="#00a0ff", width=2); c.create_line(cx, cy - 8, cx, cy + 8, fill="#00a0ff", width=2)
        if self.last_observation_pose is not None and float(self.observation_delay_ms_var.get()) > 0.0:
            ocx = ox + float(self.last_observation_pose[0]) * scale
            ocy = oy + float(self.last_observation_pose[1]) * scale
            c.create_line(ocx - 7, ocy, ocx + 7, ocy, fill="#ff9d20", width=2)
            c.create_line(ocx, ocy - 7, ocx, ocy + 7, fill="#ff9d20", width=2)
            c.create_line(ocx, ocy, cx, cy, fill="#ff9d20", dash=(4, 3), width=1)

        # Simulator planner/executor history. These overlays are display-only and use
        # sheet coordinates; the planner itself never receives SVG ground truth.
        if self.planner is not None and len(self.planner.camera_history) >= 2:
            coords: list[float] = []
            for px, py in self.planner.camera_history:
                coords.extend([ox + px * scale, oy + py * scale])
            c.create_line(*coords, fill="#0077cc", width=2)
        if self.last_plan is not None:
            if len(self.last_plan.local_path_points) >= 2:
                coords = []
                for px, py in self.last_plan.local_path_points:
                    coords.extend([ox + px * scale, oy + py * scale])
                c.create_line(*coords, fill="#d5a000", width=3)
            tx = ox + self.last_plan.approved_x * scale; ty = oy + self.last_plan.approved_y * scale
            c.create_oval(tx-5, ty-5, tx+5, ty+5, outline="#d84cff", width=3)

    def draw_camera(self) -> None:
        c = self.cam; c.delete("all")
        if self.world is None: return
        if self.analysis_render_rgb is not None and self.analysis is not None:
            rgb = self.analysis_render_rgb
            frame_x = float(self.analysis.camera_x); frame_y = float(self.analysis.camera_y)
        else:
            frame_x = float(self.x_var.get()); frame_y = float(self.y_var.get())
            rgb = self._render_virtual_rgb(for_analysis=False, camera_x=frame_x, camera_y=frame_y)
        image = Image.fromarray(rgb)
        cw = max(100, c.winfo_width()); ch = max(100, c.winfo_height()); scale = min(cw / image.width, ch / image.height)
        display = image.resize((max(1, int(image.width * scale)), max(1, int(image.height * scale))), Image.Resampling.LANCZOS)
        self.photo_cam = ImageTk.PhotoImage(display); c.create_image(cw / 2, ch / 2, image=self.photo_cam, anchor=tk.CENTER)
        arm = 12; c.create_line(cw / 2 - arm, ch / 2, cw / 2 + arm, ch / 2, fill="#00d8ff", width=2); c.create_line(cw / 2, ch / 2 - arm, cw / 2, ch / 2 + arm, fill="#00d8ff", width=2)
        truth = self.world.nearest(frame_x, frame_y, self.shape_var.get())
        # Ground-truth tangent overlay at the closest profile point.
        if self.bridge is not None:
            tpx, tpy = self.bridge.world_to_pixel(frame_x, frame_y, truth.truth_x, truth.truth_y)
            truth_px = cw / 2 + (tpx - 400.0) * scale
            truth_py = ch / 2 + (tpy - 300.0) * scale
            tipx, tipy = self.bridge.world_to_pixel(
                frame_x, frame_y,
                truth.truth_x + truth.tangent_x * 0.10, truth.truth_y + truth.tangent_y * 0.10
            )
            tvx = (tipx - tpx) * scale; tvy = (tipy - tpy) * scale
            n = max(1e-9, math.hypot(tvx, tvy)); tvx = tvx / n * 55 * scale; tvy = tvy / n * 55 * scale
        else:
            truth_px = cw / 2 + (truth.truth_x - frame_x) * self.ppi_var.get() * scale
            truth_py = ch / 2 + (truth.truth_y - frame_y) * self.ppi_var.get() * scale
            tvx = truth.tangent_x * 55 * scale; tvy = truth.tangent_y * 55 * scale
        c.create_line(truth_px - tvx, truth_py - tvy, truth_px + tvx, truth_py + tvy, fill="#2ac45a", width=3)
        c.create_oval(truth_px-5, truth_py-5, truth_px+5, truth_py+5, outline="#2ac45a", width=2)

        if self.analysis is not None and self.analysis_pose is not None:
            a = self.analysis
            # Raw detector (orange) and stabilized measurement (cyan dot).
            if a.raw_found:
                rx = cw / 2 + a.raw_pixel_error_x * scale
                ry = ch / 2 + a.raw_pixel_error_y * scale
                ang = math.radians(a.raw_angle_degrees)
                rvx, rvy = math.cos(ang), math.sin(ang)
                c.create_line(rx-rvx*70*scale, ry-rvy*70*scale, rx+rvx*70*scale, ry+rvy*70*scale, fill="#ff8c20", width=2)
                c.create_oval(rx-4, ry-4, rx+4, ry+4, outline="#ff8c20", width=2)
            if a.measured_x is not None and a.measured_y is not None:
                mpx, mpy = self.bridge.world_to_pixel(frame_x, frame_y, a.measured_x, a.measured_y)
                mx = cw / 2 + (mpx - 400.0) * scale
                my = ch / 2 + (mpy - 300.0) * scale
                c.create_oval(mx-5, my-5, mx+5, my+5, outline="#00d8ff", width=3)
            if a.belief.measurement_valid:
                epx, epy = self.bridge.world_to_pixel(frame_x, frame_y, a.belief.estimate_x, a.belief.estimate_y)
                ex = cw / 2 + (epx - 400.0) * scale
                ey = ch / 2 + (epy - 300.0) * scale
                c.create_line(ex-7, ey, ex+7, ey, fill="#d84cff", width=3); c.create_line(ex, ey-7, ex, ey+7, fill="#d84cff", width=3)
                pred = []
                for px, py in a.belief.predicted_points:
                    ppx, ppy = self.bridge.world_to_pixel(frame_x, frame_y, px, py)
                    pred.extend([cw/2 + (ppx-400.0)*scale, ch/2 + (ppy-300.0)*scale])
                if len(pred) >= 4:
                    c.create_line(*pred, fill="#d5a000", width=3)
        radius = "∞" if math.isinf(truth.implied_radius) else f"{truth.implied_radius:.3f} in"
        self.truth_text.set(
            f"Selected profile:     {truth.shape}\n"
            f"Frame/observation:    X {truth.query_x:.4f}  Y {truth.query_y:.4f} in\n"
            f"Current controller:   X {float(self.x_var.get()):.4f}  Y {float(self.y_var.get()):.4f} in\n"
            f"Observation lag:      {self.last_observation_lag_distance:.5f} in  ({float(self.observation_delay_ms_var.get()):.0f} ms)\n"
            f"Nearest SVG center:   X {truth.truth_x:.4f}  Y {truth.truth_y:.4f} in\n"
            f"True cross-track:     {truth.distance:.5f} in\n"
            f"Local tangent:        {truth.tangent_degrees:.2f}°\n"
            f"Local curvature*:     {truth.curvature:+.5f} 1/in\n"
            f"Implied radius*:      {radius}\n"
            f"*curvature is a local fit to the 1/600-in SVG-derived centerline"
        )


def launch_svg_simulator(
    parent: Optional[tk.Misc] = None,
    asset_dir: Optional[Path | str] = None,
    *,
    dialog_cls: Optional[type] = None,
    run_dir: Optional[Path | str] = None,
) -> SVGSimulatorWindow:
    return SVGSimulatorWindow(parent=parent, asset_dir=asset_dir, dialog_cls=dialog_cls, run_dir=run_dir)


if __name__ == "__main__":
    root = tk.Tk(); root.withdraw(); window = SVGSimulatorWindow(root)
    window.protocol("WM_DELETE_WINDOW", root.destroy); root.mainloop()
