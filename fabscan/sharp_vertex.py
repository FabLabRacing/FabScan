"""Sharp vertices on an ordered boundary, with explicit execution-owned state.

All geometry is in machine inches. This module has no camera or motion API.
Thresholds describe supported geometry, not a relaxation of the boundary gate.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Optional

import numpy as np


ARRIVAL_TOLERANCE = 0.003
EXIT_DISTANCE = 0.060
LEG_SUPPORT = 0.080
VERTEX_FIELDS = (
    "vertex_association_source", "vertex_association_x", "vertex_association_y", "vertex_association_reason",
    "vertex_incoming_support_span", "vertex_outgoing_support_start",
    "vertex_incoming_support_max_px", "vertex_outgoing_support_max_px",
    "vertex_state", "vertex_id", "vertex_event", "vertex_previous_state",
    "vertex_reason", "vertex_x", "vertex_y", "vertex_distance",
    "vertex_incoming_x", "vertex_incoming_y", "vertex_outgoing_x", "vertex_outgoing_y",
    "vertex_turn_degrees", "vertex_fit_error", "vertex_confirmations",
    "vertex_observation_id", "vertex_exit_x", "vertex_exit_y", "vertex_exit_distance",
)


def _angle(a: np.ndarray, b: np.ndarray) -> float:
    return math.degrees(math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0))))


def _fit(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    center = np.mean(points, axis=0)
    _, _, vh = np.linalg.svd(points - center, full_matrices=False)
    direction = vh[0]
    if np.dot(direction, points[-1] - points[0]) < 0:
        direction = -direction
    normal = np.asarray([-direction[1], direction[0]])
    error = float(np.max(np.abs((points - center) @ normal)))
    return center, direction, error


@dataclass(frozen=True)
class SharpVertex:
    point: tuple[float, float]
    incoming: tuple[float, float]
    outgoing: tuple[float, float]
    turn_degrees: float
    fit_error: float


def find_sharp_vertex(path: np.ndarray, *, pixel_size: float) -> tuple[Optional[SharpVertex], str]:
    """Fit two supported straight legs, rejecting distributed curvature and fuzz.

    Each leg needs ~0.06 inch of support outside a narrow vertex neighborhood.
    A round arc can fit two tangents, but their intersection is off the observed
    path and/or the direction changes within each leg. Neither is a vertex.
    """
    if len(path) < 8:
        return None, "insufficient connected path for two supported legs"
    distances = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    if distances[-1] < 2 * LEG_SUPPORT:
        return None, "insufficient connected path for two supported legs"
    sample_s = np.arange(0.0, distances[-1], 0.005)
    points = np.column_stack([np.interp(sample_s, distances, path[:, i]) for i in (0, 1)])
    segments = np.diff(path, axis=0)
    segment_norm2 = np.sum(segments * segments, axis=1)
    # The cap prevents coarse calibration from turning a radius into a vertex.
    tolerance = min(0.004, max(0.0015, 1.5 * pixel_size))
    best: Optional[SharpVertex] = None
    best_quality = math.inf
    first_vertex_s: Optional[float] = None
    reason = "no localized sharp turn with two straight supported legs"
    for i in range(16, len(points) - 16):
        before = points[i-16:i-2]
        after = points[i+3:i+17]
        c0, incoming, err0 = _fit(before)
        c1, outgoing, err1 = _fit(after)
        turn = _angle(incoming, outgoing)
        if not 55.0 <= turn <= 135.0:
            continue
        if max(err0, err1) > tolerance:
            reason = "turn is curved or irregular: straight-leg residual too large"
            continue
        # Straightness must persist across each leg, not just fit in aggregate.
        if any(_angle(_fit(leg[:7])[1], _fit(leg[7:])[1]) > 12.0 for leg in (before, after)):
            reason = "turn is distributed along a radius rather than localized"
            continue
        along = np.linalg.solve(np.column_stack([incoming, -outgoing]), c1 - c0)
        vertex = c0 + along[0] * incoming
        if np.linalg.norm(vertex - points[i]) > 0.018:
            continue
        # Distance to the original connected boundary, not a resampled chord
        # that can shave off a corner depending on subpixel/sample phase.
        projection = np.clip(np.sum((vertex - path[:-1]) * segments, axis=1)
                             / np.maximum(segment_norm2, 1e-20), 0.0, 1.0)
        error = float(np.min(np.linalg.norm(path[:-1] + projection[:, None] * segments - vertex, axis=1)))
        if error > tolerance * 1.5:
            reason = "two tangents intersect off the observed boundary (rounded turn)"
            continue
        quality = max(err0, err1) + error
        if first_vertex_s is not None and sample_s[i] > first_vertex_s + 0.030:
            break  # Never skip an earlier supported vertex for a cleaner later fit.
        if first_vertex_s is None:
            first_vertex_s = float(sample_s[i])
        if quality < best_quality:
            best_quality = quality
            best = SharpVertex(tuple(vertex), tuple(incoming), tuple(outgoing), turn, max(err0, err1))
    return best, "two straight connected legs meet at a sharp vertex" if best else reason


class VertexCommitment:
    """Confirmation in observations; arrival/release only in executed poses."""

    def __init__(self) -> None:
        self.state = "FOLLOW"
        self.vertex: Optional[SharpVertex] = None
        self.confirmations = 0
        self.observation_id: Any = None
        self.vertex_id = 0
        self.reason = "ordinary boundary following"
        self.last_vertex: Optional[np.ndarray] = None
        self.events: list[dict[str, Any]] = []

    @property
    def active(self) -> bool:
        return self.state in {"APPROACH", "OUTGOING"}

    def fields(self, camera: np.ndarray) -> dict[str, Any]:
        fields: dict[str, Any] = {
            "vertex_state": self.state, "vertex_id": self.vertex_id,
            "vertex_reason": self.reason, "vertex_confirmations": self.confirmations,
            "vertex_observation_id": self.observation_id,
        }
        if self.vertex is not None:
            v = self.vertex
            point = np.asarray(v.point)
            exit_point = point + np.asarray(v.outgoing) * EXIT_DISTANCE
            fields.update({
                "vertex_x": v.point[0], "vertex_y": v.point[1],
                "vertex_distance": float(np.linalg.norm(point - camera)),
                "vertex_incoming_x": v.incoming[0], "vertex_incoming_y": v.incoming[1],
                "vertex_outgoing_x": v.outgoing[0], "vertex_outgoing_y": v.outgoing[1],
                "vertex_turn_degrees": v.turn_degrees, "vertex_fit_error": v.fit_error,
                "vertex_exit_x": float(exit_point[0]), "vertex_exit_y": float(exit_point[1]),
                "vertex_exit_distance": float(np.linalg.norm(exit_point - camera)),
            })
        return fields

    def _event(self, event: str, previous: str, camera: np.ndarray) -> None:
        self.events.append({**self.fields(camera), "vertex_event": event, "vertex_previous_state": previous})

    def drain_events(self) -> list[dict[str, Any]]:
        events, self.events = self.events, []
        return events

    def reject_candidate(self, reason: str, camera: np.ndarray) -> None:
        if self.state == "CANDIDATE":
            self.state = "FOLLOW"
            self.reason = reason
            self._event("CANDIDATE_REJECTED", "CANDIDATE", camera)
            self.vertex = None
            self.confirmations = 0
        else:
            self.reason = reason

    def observe(self, vertex: Optional[SharpVertex], *, camera: np.ndarray,
                observation_id: Any, pixel_size: float, max_cross_track: float) -> None:
        if self.active:
            return  # Never refit or blend a committed target.
        if vertex is None:
            self.reject_candidate("no credible sharp vertex in this observation", camera)
            return
        point = np.asarray(vertex.point)
        incoming = np.asarray(vertex.incoming)
        delta = point - camera
        along = float(np.dot(delta, incoming))
        cross = float(np.linalg.norm(delta - along * incoming))
        if along < ARRIVAL_TOLERANCE or along > 0.22 or cross > max_cross_track:
            self.reject_candidate("vertex outside supported forward approach corridor", camera)
            return
        if self.last_vertex is not None and np.linalg.norm(point - self.last_vertex) < 0.04:
            self.reject_candidate("previous vertex already completed", camera)
            return
        consistent = self.vertex is not None and (
            np.linalg.norm(point - self.vertex.point) <= min(0.010, max(0.006, 3 * pixel_size))
            and _angle(incoming, np.asarray(self.vertex.incoming)) <= 12.0
            and _angle(np.asarray(vertex.outgoing), np.asarray(self.vertex.outgoing)) <= 12.0
        )
        if not consistent:
            self.reject_candidate("vertex changed before confirmation", camera)
            self.vertex = vertex
            self.confirmations = 1
            self.observation_id = observation_id
            self.state = "CANDIDATE"
            self.reason = "sharp vertex needs confirmation in a second distinct frame"
            self._event("CANDIDATE", "FOLLOW", camera)
        elif observation_id != self.observation_id:
            self.confirmations += 1
            self.observation_id = observation_id
            self.vertex = vertex  # Latest fit becomes immutable at commitment.
            self.vertex_id += 1
            self.state = "APPROACH"
            self.reason = "vertex confirmed; fixed machine target and outgoing leg committed"
            self._event("COMMITTED", "CANDIDATE", camera)

    def record_executed_pose(self, camera: np.ndarray) -> Optional[np.ndarray]:
        if not self.active or self.vertex is None:
            return None
        point = np.asarray(self.vertex.point)
        outgoing = np.asarray(self.vertex.outgoing)
        if self.state == "APPROACH" and np.linalg.norm(camera - point) <= ARRIVAL_TOLERANCE:
            self.reason = "reported executed pose reached the committed vertex"
            self._event("VERTEX_REACHED", "APPROACH", camera)
            self.state = "OUTGOING"
            self.reason = "outgoing leg latched; incoming heading memory retired"
            self._event("OUTGOING_LATCHED", "APPROACH", camera)
        if self.state == "OUTGOING":
            exit_point = point + outgoing * EXIT_DISTANCE
            if np.linalg.norm(camera - exit_point) <= ARRIVAL_TOLERANCE:
                self.state = "FOLLOW"
                self.reason = "executed pose established outgoing leg; ordinary following resumed"
                self.last_vertex = point.copy()
                self._event("RESUMED", "OUTGOING", camera)
                self.vertex = None
                self.confirmations = 0
            return outgoing
        return np.asarray(self.vertex.incoming)

    def hold(self, reason: str, camera: np.ndarray) -> None:
        # Keep commitment for diagnostics; a new physical run creates fresh state.
        if self.active:
            self.reason = reason
            self._event("HOLD", self.state, camera)
        else:
            self.reject_candidate(reason, camera)
