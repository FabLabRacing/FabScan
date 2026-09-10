from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np

INTEGRATION_VERSION = "0.6.0-dev-m4"


def _float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _text(value: Any, default: str = "") -> str:
    return default if value is None else str(value)


def _wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _unit(vector: np.ndarray, fallback: tuple[float, float] = (1.0, 0.0)) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-12:
        return np.asarray(fallback, dtype=float)
    return np.asarray(vector, dtype=float) / norm


def _arc_advance(x: float, y: float, heading: float, curvature: float, distance: float) -> tuple[float, float, float]:
    if abs(curvature) < 1e-6:
        return (
            x + math.cos(heading) * distance,
            y + math.sin(heading) * distance,
            heading,
        )
    next_heading = heading + curvature * distance
    next_x = x + (math.sin(next_heading) - math.sin(heading)) / curvature
    next_y = y + (-math.cos(next_heading) + math.cos(heading)) / curvature
    return next_x, next_y, next_heading


@dataclass(frozen=True)
class ContextResult:
    found: bool
    seed_component_area_px: int = 0
    other_geometry_count: int = 0
    usable_lookahead: float = 0.0
    max_forward_extent: float = 0.0
    forward_support_ratio: float = 0.0
    seed_x_px: float = 0.0
    seed_y_px: float = 0.0
    component_contour: tuple[tuple[int, int], ...] = ()
    reason: str = ""

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["component_contour"] = [list(point) for point in self.component_contour]
        return data


@dataclass(frozen=True)
class BeliefState:
    step_id: int
    step_index: int
    measurement_valid: bool
    measurement_x: float
    measurement_y: float
    measured_heading_degrees: float
    measured_confidence: float
    estimate_x: float
    estimate_y: float
    tangent_x: float
    tangent_y: float
    tangent_degrees: float
    raw_curvature: float
    curvature: float
    curvature_trend: float
    implied_radius: float
    usable_lookahead: float
    prediction_horizon: float
    belief_confidence: float
    uncertainty: float
    position_innovation: float
    heading_innovation_degrees: float
    context: ContextResult
    recent_measurements: tuple[tuple[float, float], ...]
    predicted_points: tuple[tuple[float, float], ...]
    reason: str

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["context"] = self.context.to_dict()
        data["recent_measurements"] = [list(point) for point in self.recent_measurements]
        data["predicted_points"] = [list(point) for point in self.predicted_points]
        return data


@dataclass
class _Measurement:
    step_id: int
    x: float
    y: float
    heading: float
    confidence: float
    travel_s: float


class PathBeliefEstimator:
    """Read-only world-space path-belief scaffold.

    This estimator intentionally has no motion API. It consumes the same recorded
    evidence used by replay and produces a persistent state estimate plus a short
    constant-curvature prediction for visualization and later planner work.
    """

    def __init__(self, calibration: dict[str, Any], settings: dict[str, Any]) -> None:
        matrix = calibration.get("matrix_pixel_to_machine")
        if not isinstance(matrix, list) or len(matrix) != 2:
            raise ValueError("Replay calibration does not contain matrix_pixel_to_machine")
        self.pixel_to_machine = np.asarray(matrix, dtype=float)
        if self.pixel_to_machine.shape != (2, 2):
            raise ValueError("matrix_pixel_to_machine must be 2x2")
        self.settings = dict(settings)
        self.threshold = _int(settings.get("threshold_var", settings.get("threshold", 90)), 90)
        self.min_confidence = _float(
            settings.get("follow_min_confidence_var", settings.get("follow_min_confidence", 45.0)),
            45.0,
        )
        self._measurements: list[_Measurement] = []
        self._states: list[BeliefState] = []
        self._last_machine_position: Optional[np.ndarray] = None
        self._travel_s = 0.0
        self._filtered_curvature_history: list[tuple[float, float]] = []

    @property
    def states(self) -> tuple[BeliefState, ...]:
        return tuple(self._states)

    def _machine_base(self, step: dict[str, Any]) -> np.ndarray:
        delayed = step.get("delayed_position")
        if isinstance(delayed, dict) and delayed.get("x") not in (None, "") and delayed.get("y") not in (None, ""):
            return np.asarray([_float(delayed.get("x")), _float(delayed.get("y"))], dtype=float)
        before = step.get("position_before")
        if isinstance(before, dict):
            return np.asarray([_float(before.get("x")), _float(before.get("y"))], dtype=float)
        return np.zeros(2, dtype=float)

    def _measurement_from_step(self, step: dict[str, Any], frame_shape: tuple[int, int]) -> Optional[tuple[np.ndarray, float, float, np.ndarray]]:
        planner = step.get("planner_output")
        if not isinstance(planner, dict):
            return None
        if planner.get("pixel_error_x") in (None, "") or planner.get("pixel_error_y") in (None, ""):
            return None
        confidence = _float(planner.get("confidence"))
        base = self._machine_base(step)
        pixel_error = np.asarray(
            [_float(planner.get("pixel_error_x")), _float(planner.get("pixel_error_y"))],
            dtype=float,
        )
        # Calibration records how a fixed feature moves in image space when the
        # camera moves. A physical feature offset from camera center therefore has
        # the opposite sign of that camera-motion transform.
        feature_offset = -(self.pixel_to_machine @ pixel_error)
        point = base + feature_offset

        angle = math.radians(_float(planner.get("angle_degrees")))
        pixel_tangent = np.asarray([math.cos(angle), math.sin(angle)], dtype=float)
        tangent = _unit(-(self.pixel_to_machine @ pixel_tangent))
        move = np.asarray([_float(planner.get("move_x")), _float(planner.get("move_y"))], dtype=float)
        if float(np.linalg.norm(move)) > 1e-6:
            if float(np.dot(tangent, move)) < 0.0:
                tangent = -tangent
        elif self._measurements:
            previous = np.asarray(
                [math.cos(self._measurements[-1].heading), math.sin(self._measurements[-1].heading)],
                dtype=float,
            )
            if float(np.dot(tangent, previous)) < 0.0:
                tangent = -tangent
        heading = math.atan2(float(tangent[1]), float(tangent[0]))
        if self._measurements:
            previous_heading = self._measurements[-1].heading
            heading = previous_heading + _wrap_pi(heading - previous_heading)
        frame_h, frame_w = frame_shape
        seed = np.asarray(
            [frame_w / 2.0 + pixel_error[0], frame_h / 2.0 + pixel_error[1]],
            dtype=float,
        )
        return point, heading, confidence, seed

    def _context_from_frame(
        self,
        frame_bgr: np.ndarray,
        seed: np.ndarray,
        tangent_world: np.ndarray,
    ) -> ContextResult:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        mask = (gray < int(self.threshold)).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), dtype=np.uint8))
        count, labels, stats, _centroids = cv2.connectedComponentsWithStats(mask, 8)
        height, width = gray.shape[:2]

        component_label: Optional[int] = None
        nearest_seed = seed.copy()
        for radius in (8, 16, 32, 64, 96):
            x0 = max(0, int(seed[0]) - radius)
            x1 = min(width, int(seed[0]) + radius + 1)
            y0 = max(0, int(seed[1]) - radius)
            y1 = min(height, int(seed[1]) + radius + 1)
            yy, xx = np.nonzero(mask[y0:y1, x0:x1])
            if len(xx) == 0:
                continue
            points = np.column_stack([xx + x0, yy + y0]).astype(float)
            nearest_index = int(np.argmin(np.sum((points - seed) ** 2, axis=1)))
            nearest_seed = points[nearest_index]
            component_label = int(labels[int(nearest_seed[1]), int(nearest_seed[0])])
            if component_label > 0:
                break
        if not component_label:
            return ContextResult(found=False, reason="no dark component near current profile")

        area = int(stats[component_label, cv2.CC_STAT_AREA])
        if area < 50:
            return ContextResult(found=False, seed_component_area_px=area, reason="seed component too small")

        yy, xx = np.nonzero(labels == component_label)
        pixels = np.column_stack([xx, yy]).astype(float)
        pixel_delta = pixels - seed
        world_delta = -(self.pixel_to_machine @ pixel_delta.T).T
        forward_projection = world_delta @ tangent_world
        positive = forward_projection[forward_projection > 0.0]
        if len(positive):
            usable = float(np.percentile(positive, 95.0))
            max_forward = float(np.max(positive))
        else:
            usable = 0.0
            max_forward = 0.0
        forward_ratio = float(np.count_nonzero(forward_projection > 0.0) / max(1, len(forward_projection)))

        other_geometry_count = 0
        for label in range(1, count):
            if label == component_label:
                continue
            if int(stats[label, cv2.CC_STAT_AREA]) >= 250:
                other_geometry_count += 1

        component_mask = np.zeros_like(mask)
        component_mask[labels == component_label] = 255
        contours, _hierarchy = cv2.findContours(component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour: tuple[tuple[int, int], ...] = ()
        if contours:
            largest = max(contours, key=cv2.contourArea)
            epsilon = max(1.0, 0.003 * cv2.arcLength(largest, True))
            simplified = cv2.approxPolyDP(largest, epsilon, True)
            contour = tuple((int(p[0][0]), int(p[0][1])) for p in simplified[:300])

        return ContextResult(
            found=True,
            seed_component_area_px=area,
            other_geometry_count=other_geometry_count,
            usable_lookahead=max(0.0, usable),
            max_forward_extent=max(0.0, max_forward),
            forward_support_ratio=forward_ratio,
            seed_x_px=float(nearest_seed[0]),
            seed_y_px=float(nearest_seed[1]),
            component_contour=contour,
            reason="connected component seeded from current profile",
        )

    def _raw_curvature(self) -> float:
        if len(self._measurements) < 4:
            return 0.0
        newest_s = self._measurements[-1].travel_s
        selected = [m for m in self._measurements if newest_s - m.travel_s <= 0.30]
        if len(selected) < 4:
            return 0.0
        s = np.asarray([m.travel_s for m in selected], dtype=float)
        heading = np.asarray([m.heading for m in selected], dtype=float)
        weights = np.asarray([max(0.1, min(1.0, m.confidence / 100.0)) for m in selected], dtype=float)
        mean_s = float(np.average(s, weights=weights))
        mean_h = float(np.average(heading, weights=weights))
        denominator = float(np.sum(weights * (s - mean_s) ** 2))
        if denominator <= 1e-10:
            return 0.0
        return float(np.sum(weights * (s - mean_s) * (heading - mean_h)) / denominator)

    def _curvature_trend(self) -> float:
        if len(self._filtered_curvature_history) < 4:
            return 0.0
        newest_s = self._filtered_curvature_history[-1][0]
        selected = [(s, k) for s, k in self._filtered_curvature_history if newest_s - s <= 0.30]
        if len(selected) < 4:
            return 0.0
        s = np.asarray([value[0] for value in selected], dtype=float)
        k = np.asarray([value[1] for value in selected], dtype=float)
        s0 = float(np.mean(s))
        k0 = float(np.mean(k))
        denominator = float(np.sum((s - s0) ** 2))
        if denominator <= 1e-10:
            return 0.0
        return float(np.sum((s - s0) * (k - k0)) / denominator)

    def _recent_points(self, distance: float = 0.40) -> tuple[tuple[float, float], ...]:
        if not self._measurements:
            return ()
        newest_s = self._measurements[-1].travel_s
        points = [
            (m.x, m.y)
            for m in self._measurements
            if newest_s - m.travel_s <= distance
        ]
        return tuple(points[-24:])

    @staticmethod
    def _prediction_points(x: float, y: float, heading: float, curvature: float, horizon: float) -> tuple[tuple[float, float], ...]:
        if horizon <= 1e-6:
            return ((x, y),)
        count = 12
        points: list[tuple[float, float]] = []
        for index in range(count + 1):
            distance = horizon * index / count
            px, py, _ph = _arc_advance(x, y, heading, curvature, distance)
            points.append((px, py))
        return tuple(points)

    def update(self, step: dict[str, Any], frame_bgr: np.ndarray, step_index: int) -> BeliefState:
        frame_h, frame_w = frame_bgr.shape[:2]
        measurement = self._measurement_from_step(step, (frame_h, frame_w))
        step_id = _int(step.get("step_id"), step_index + 1)
        machine_position = self._machine_base(step)
        if self._last_machine_position is not None:
            self._travel_s += float(np.linalg.norm(machine_position - self._last_machine_position))
        self._last_machine_position = machine_position

        if measurement is None:
            if self._states:
                previous = self._states[-1]
                context = ContextResult(found=False, reason="no profile measurement for this step")
                state = BeliefState(
                    step_id=step_id,
                    step_index=step_index,
                    measurement_valid=False,
                    measurement_x=previous.measurement_x,
                    measurement_y=previous.measurement_y,
                    measured_heading_degrees=previous.measured_heading_degrees,
                    measured_confidence=0.0,
                    estimate_x=previous.estimate_x,
                    estimate_y=previous.estimate_y,
                    tangent_x=previous.tangent_x,
                    tangent_y=previous.tangent_y,
                    tangent_degrees=previous.tangent_degrees,
                    raw_curvature=previous.raw_curvature,
                    curvature=previous.curvature,
                    curvature_trend=previous.curvature_trend,
                    implied_radius=previous.implied_radius,
                    usable_lookahead=0.0,
                    prediction_horizon=0.0,
                    belief_confidence=0.0,
                    uncertainty=1.0,
                    position_innovation=0.0,
                    heading_innovation_degrees=0.0,
                    context=context,
                    recent_measurements=self._recent_points(),
                    predicted_points=((previous.estimate_x, previous.estimate_y),),
                    reason="held previous belief because the frame produced no usable measurement",
                )
            else:
                context = ContextResult(found=False, reason="no profile measurement for this step")
                state = BeliefState(
                    step_id=step_id,
                    step_index=step_index,
                    measurement_valid=False,
                    measurement_x=0.0,
                    measurement_y=0.0,
                    measured_heading_degrees=0.0,
                    measured_confidence=0.0,
                    estimate_x=0.0,
                    estimate_y=0.0,
                    tangent_x=1.0,
                    tangent_y=0.0,
                    tangent_degrees=0.0,
                    raw_curvature=0.0,
                    curvature=0.0,
                    curvature_trend=0.0,
                    implied_radius=0.0,
                    usable_lookahead=0.0,
                    prediction_horizon=0.0,
                    belief_confidence=0.0,
                    uncertainty=1.0,
                    position_innovation=0.0,
                    heading_innovation_degrees=0.0,
                    context=context,
                    recent_measurements=(),
                    predicted_points=(),
                    reason="no initial profile measurement",
                )
            self._states.append(state)
            return state

        measured_point, measured_heading, confidence, seed = measurement
        self._measurements.append(
            _Measurement(
                step_id=step_id,
                x=float(measured_point[0]),
                y=float(measured_point[1]),
                heading=measured_heading,
                confidence=confidence,
                travel_s=self._travel_s,
            )
        )
        raw_curvature = self._raw_curvature()

        if not self._states or not self._states[-1].measurement_valid:
            estimate = measured_point.copy()
            estimate_heading = measured_heading
            filtered_curvature = raw_curvature
            position_innovation = 0.0
            heading_innovation = 0.0
        else:
            previous = self._states[-1]
            previous_heading = math.radians(previous.tangent_degrees)
            ds = max(0.0, self._travel_s - self._measurements[-2].travel_s) if len(self._measurements) >= 2 else 0.0
            pred_x, pred_y, pred_heading = _arc_advance(
                previous.estimate_x,
                previous.estimate_y,
                previous_heading,
                previous.curvature,
                ds,
            )
            predicted = np.asarray([pred_x, pred_y], dtype=float)
            innovation_vector = measured_point - predicted
            position_innovation = float(np.linalg.norm(innovation_vector))
            heading_innovation = _wrap_pi(measured_heading - pred_heading)

            quality = max(0.0, min(1.0, (confidence - self.min_confidence) / max(1.0, 100.0 - self.min_confidence)))
            position_gain = 0.35 + 0.35 * quality
            heading_gain = 0.28 + 0.42 * quality
            curvature_gain = 0.20 + 0.35 * quality
            estimate = predicted + innovation_vector * position_gain
            estimate_heading = pred_heading + heading_innovation * heading_gain
            filtered_curvature = previous.curvature + (raw_curvature - previous.curvature) * curvature_gain

        tangent = np.asarray([math.cos(estimate_heading), math.sin(estimate_heading)], dtype=float)
        context = self._context_from_frame(frame_bgr, seed, tangent)
        usable_lookahead = context.usable_lookahead if context.found else 0.0

        # Dynamic prediction horizon: visible connected support is the hard upper
        # bound. Curvature and uncertainty shorten how far the belief extrapolates.
        curvature_limit = 0.30 / (1.0 + 0.55 * abs(filtered_curvature))
        quality = max(0.0, min(1.0, (confidence - self.min_confidence) / max(1.0, 100.0 - self.min_confidence)))
        confidence_limit = 0.08 + 0.22 * quality
        prediction_horizon = min(usable_lookahead, curvature_limit, confidence_limit)
        if usable_lookahead > 0.0:
            prediction_horizon = max(min(0.05, usable_lookahead), prediction_horizon)

        # Confidence is intentionally interpretable rather than tuned as a final
        # safety probability. Penalize disagreement with prediction and lack of
        # connected context while preserving the detector's measurement quality.
        position_penalty = min(35.0, position_innovation / 0.020 * 12.0) if position_innovation > 0.0 else 0.0
        heading_penalty = min(35.0, abs(math.degrees(heading_innovation)) / 45.0 * 18.0) if 'heading_innovation' in locals() else 0.0
        context_bonus = min(8.0, usable_lookahead / 0.25 * 8.0) if context.found else -12.0
        belief_confidence = max(0.0, min(100.0, confidence - position_penalty - heading_penalty + context_bonus))
        uncertainty = max(0.0, min(1.0, 1.0 - belief_confidence / 100.0))

        self._filtered_curvature_history.append((self._travel_s, filtered_curvature))
        curvature_trend = self._curvature_trend()
        implied_radius = 0.0 if abs(filtered_curvature) < 0.05 else 1.0 / abs(filtered_curvature)
        predicted_points = self._prediction_points(
            float(estimate[0]),
            float(estimate[1]),
            estimate_heading,
            filtered_curvature,
            prediction_horizon,
        )

        reason_parts = ["updated previous belief with current profile measurement"]
        if context.found:
            reason_parts.append(f"{usable_lookahead:.3f} in connected context visible ahead")
            if context.other_geometry_count:
                reason_parts.append(f"{context.other_geometry_count} other dark component(s) visible")
        else:
            reason_parts.append("no connected far-context support")
        if abs(filtered_curvature) < 0.08:
            reason_parts.append("path belief is approximately straight")
        else:
            direction = "CCW" if filtered_curvature > 0.0 else "CW"
            reason_parts.append(f"path belief is curving {direction}")

        state = BeliefState(
            step_id=step_id,
            step_index=step_index,
            measurement_valid=True,
            measurement_x=float(measured_point[0]),
            measurement_y=float(measured_point[1]),
            measured_heading_degrees=math.degrees(measured_heading),
            measured_confidence=confidence,
            estimate_x=float(estimate[0]),
            estimate_y=float(estimate[1]),
            tangent_x=float(tangent[0]),
            tangent_y=float(tangent[1]),
            tangent_degrees=math.degrees(estimate_heading),
            raw_curvature=raw_curvature,
            curvature=filtered_curvature,
            curvature_trend=curvature_trend,
            implied_radius=implied_radius,
            usable_lookahead=usable_lookahead,
            prediction_horizon=prediction_horizon,
            belief_confidence=belief_confidence,
            uncertainty=uncertainty,
            position_innovation=position_innovation,
            heading_innovation_degrees=math.degrees(heading_innovation) if 'heading_innovation' in locals() else 0.0,
            context=context,
            recent_measurements=self._recent_points(),
            predicted_points=predicted_points,
            reason="; ".join(reason_parts),
        )
        self._states.append(state)
        return state


class ReplayBeliefAnalyzer:
    def __init__(self, run_dir: Path | str) -> None:
        self.run_dir = Path(run_dir).expanduser().resolve()
        self.calibration = json.loads((self.run_dir / "calibration.json").read_text(encoding="utf-8"))
        self.settings = json.loads((self.run_dir / "settings.json").read_text(encoding="utf-8"))
        self.steps: list[dict[str, Any]] = []
        with (self.run_dir / "steps.jsonl").open("r", encoding="utf-8") as handle:
            for raw in handle:
                if raw.strip():
                    self.steps.append(json.loads(raw))
        self.estimator = PathBeliefEstimator(self.calibration, self.settings)

    def run_all(self, progress: Optional[Any] = None) -> list[BeliefState]:
        states: list[BeliefState] = []
        total = len(self.steps)
        for index, step in enumerate(self.steps):
            relative = _text(step.get("decision_frame"))
            frame_path = (self.run_dir / relative).resolve()
            try:
                frame_path.relative_to(self.run_dir)
            except ValueError as exc:
                raise ValueError(f"Replay frame escapes run folder: {relative}") from exc
            frame = cv2.imread(str(frame_path), cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError(f"Could not read replay frame: {frame_path}")
            states.append(self.estimator.update(step, frame, index))
            if progress is not None:
                progress(index + 1, total)
        return states

    @staticmethod
    def summary(states: list[BeliefState]) -> dict[str, Any]:
        valid = [state for state in states if state.measurement_valid]
        curved = [state for state in valid if abs(state.curvature) >= 0.08]
        return {
            "steps": len(states),
            "valid_measurements": len(valid),
            "mean_belief_confidence": (sum(s.belief_confidence for s in valid) / len(valid)) if valid else 0.0,
            "mean_usable_lookahead": (sum(s.usable_lookahead for s in valid) / len(valid)) if valid else 0.0,
            "median_abs_curvature": float(np.median([abs(s.curvature) for s in valid])) if valid else 0.0,
            "median_implied_radius_when_curved": float(np.median([s.implied_radius for s in curved if s.implied_radius > 0.0])) if curved else 0.0,
            "max_abs_heading_innovation_degrees": max((abs(s.heading_innovation_degrees) for s in valid), default=0.0),
            "max_position_innovation": max((s.position_innovation for s in valid), default=0.0),
        }
