from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Optional

import cv2
import numpy as np

from fabscan.path_belief import BeliefState

INTEGRATION_VERSION = "0.6.0-dev-m5.6"


def _float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _unit(v: np.ndarray, fallback: tuple[float, float] = (1.0, 0.0)) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n <= 1e-12:
        return np.asarray(fallback, dtype=float)
    return np.asarray(v, dtype=float) / n


def _angle_between_degrees(a: np.ndarray, b: np.ndarray) -> float:
    aa = _unit(a); bb = _unit(b)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(aa, bb))))))


def _skeletonize(mask: np.ndarray) -> np.ndarray:
    """Zhang-Suen thinning implemented with NumPy only.

    OpenCV's erosion-difference "morphological skeleton" can fragment a curved,
    anti-aliased 40-pixel stroke into many disconnected islands. Zhang-Suen keeps
    one-pixel connectivity, which is what the look-ahead graph needs.
    """
    img = (mask > 0).astype(np.uint8)
    if img.shape[0] < 3 or img.shape[1] < 3:
        return (img * 255).astype(np.uint8)
    # Never allow np.roll wraparound to create a false edge connection.
    img[[0, -1], :] = 0; img[:, [0, -1]] = 0

    def neighbors(a: np.ndarray):
        p2 = np.roll(a,  1, axis=0)                         # north
        p3 = np.roll(np.roll(a, 1, axis=0), -1, axis=1)  # north-east
        p4 = np.roll(a, -1, axis=1)                        # east
        p5 = np.roll(np.roll(a,-1, axis=0), -1, axis=1)  # south-east
        p6 = np.roll(a, -1, axis=0)                        # south
        p7 = np.roll(np.roll(a,-1, axis=0),  1, axis=1)  # south-west
        p8 = np.roll(a,  1, axis=1)                        # west
        p9 = np.roll(np.roll(a, 1, axis=0),  1, axis=1)  # north-west
        return p2,p3,p4,p5,p6,p7,p8,p9

    for _ in range(80):
        changed = False
        p = neighbors(img); b = sum(p)
        seq = p + (p[0],)
        transitions = sum(((seq[i] == 0) & (seq[i+1] == 1)).astype(np.uint8) for i in range(8))
        interior = np.zeros_like(img, dtype=bool); interior[1:-1,1:-1] = True
        mark = (img == 1) & interior & (b >= 2) & (b <= 6) & (transitions == 1) \
               & ((p[0] * p[2] * p[4]) == 0) & ((p[2] * p[4] * p[6]) == 0)
        if np.any(mark):
            img[mark] = 0; changed = True

        p = neighbors(img); b = sum(p); seq = p + (p[0],)
        transitions = sum(((seq[i] == 0) & (seq[i+1] == 1)).astype(np.uint8) for i in range(8))
        mark = (img == 1) & interior & (b >= 2) & (b <= 6) & (transitions == 1) \
               & ((p[0] * p[2] * p[6]) == 0) & ((p[0] * p[4] * p[6]) == 0)
        if np.any(mark):
            img[mark] = 0; changed = True
        if not changed:
            break
    return (img * 255).astype(np.uint8)


@dataclass(frozen=True)
class CandidatePath:
    points_world: tuple[tuple[float, float], ...]
    points_pixel: tuple[tuple[int, int], ...]
    arc_length: float
    start_alignment: float
    end_alignment: float
    turn_degrees: float
    score: float
    reason: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class PlannerResult:
    accepted: bool
    safety_action: str
    camera_x: float
    camera_y: float
    proposed_x: float
    proposed_y: float
    approved_x: float
    approved_y: float
    move_x: float
    move_y: float
    move_length: float
    forward_step: float
    cross_track_correction: float
    local_path_points: tuple[tuple[float, float], ...]
    current_tangent_x: float
    current_tangent_y: float
    predicted_tangent_x: float
    predicted_tangent_y: float
    curvature: float
    usable_lookahead: float
    steering_lookahead: float
    recommended_velocity: float
    planner_confidence: float
    path_turn_degrees: float
    candidate_count: int
    rejected_candidates: tuple[str, ...]
    decision_reason: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class ConnectedPathPlanner:
    """First belief-based virtual planner.

    The planner never receives SVG ground truth. It consumes only the rendered
    camera frame, calibration/settings, current camera pose, and the persistent
    M4 belief. It extracts the connected stroke centerline ahead, evaluates both
    possible path directions, and returns a rich trajectory/velocity result.

    The executor remains separate: M5.2 uses one bounded virtual step from this
    result. No LinuxCNC command API is present here.
    """

    def __init__(self, calibration: dict[str, Any], settings: dict[str, Any]) -> None:
        self.calibration = dict(calibration)
        self.settings = dict(settings)
        self.pixel_to_machine = np.asarray(self.calibration["matrix_pixel_to_machine"], dtype=float)
        self.machine_to_pixel = np.asarray(self.calibration["matrix_machine_to_pixel"], dtype=float)
        self.threshold = int(_float(settings.get("threshold_var", settings.get("threshold", 90)), 90))
        self.follow_step = max(0.005, _float(settings.get("follow_step_var", settings.get("follow_step", 0.03)), 0.03))
        self.max_correct = max(0.0, _float(settings.get("follow_max_correct_var", settings.get("follow_max_correct", 0.012)), 0.012))
        self.base_feed = max(1.0, _float(settings.get("follow_feed_var", settings.get("follow_feed", 100.0)), 100.0))
        self.min_confidence = _float(settings.get("follow_min_confidence_var", settings.get("follow_min_confidence", 45.0)), 45.0)
        self.travel_unit: Optional[np.ndarray] = None
        self.initial_reverse = False
        self.camera_history: list[tuple[float, float]] = []
        self.total_distance = 0.0
        self._last_camera: Optional[np.ndarray] = None

    def reset(self, *, reverse_initial: bool = False) -> None:
        self.travel_unit = None
        self.initial_reverse = bool(reverse_initial)
        self.camera_history.clear()
        self.total_distance = 0.0
        self._last_camera = None

    def record_executed_pose(self, x: float, y: float) -> None:
        point = np.asarray([float(x), float(y)], dtype=float)
        if self._last_camera is not None:
            delta = point - self._last_camera
            distance = float(np.linalg.norm(delta))
            if distance > 1e-9:
                self.total_distance += distance
        self._last_camera = point
        self.camera_history.append((float(x), float(y)))
        if len(self.camera_history) > 2000:
            self.camera_history = self.camera_history[-2000:]

        # Established travel is a trailing path chord, not the most recent bump.
        # A correction-heavy move can point mostly sideways and must not become
        # the global "forward" direction used by the next candidate/safety check.
        if len(self.camera_history) >= 2:
            current = np.asarray(self.camera_history[-1], dtype=float)
            previous = current
            chosen = np.asarray(self.camera_history[-2], dtype=float)
            accumulated = 0.0
            for historical in reversed(self.camera_history[:-1]):
                historical_p = np.asarray(historical, dtype=float)
                accumulated += float(np.linalg.norm(previous - historical_p))
                chosen = historical_p
                previous = historical_p
                if accumulated >= 0.10:
                    break
            chord = current - chosen
            if float(np.linalg.norm(chord)) > 1e-9:
                self.travel_unit = _unit(
                    chord,
                    tuple(self.travel_unit) if self.travel_unit is not None else (1.0, 0.0),
                )

    def _world_to_pixel_delta(self, world_delta: np.ndarray) -> np.ndarray:
        return -(self.machine_to_pixel @ np.asarray(world_delta, dtype=float))

    def _pixel_to_world(self, camera: np.ndarray, px: float, py: float, width: int, height: int) -> np.ndarray:
        pixel_delta = np.asarray([float(px) - width / 2.0, float(py) - height / 2.0], dtype=float)
        return camera - (self.pixel_to_machine @ pixel_delta)

    def _seed_component(self, frame_bgr: np.ndarray, seed_px: np.ndarray) -> tuple[Optional[np.ndarray], str]:
        """Associate the belief seed with the nearest *usable* dark component.

        M6.1 chose the nearest dark pixel first and only then checked whether that
        pixel's connected component was large enough. A tiny scratch/speck could
        therefore monopolize every expanding search radius and hide the real
        profile immediately beside it. M6.2 filters out undersized components
        before choosing the nearest qualifying pixel/component.
        """
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        mask = np.where(gray < self.threshold, 255, 0).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), dtype=np.uint8))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        h, w = mask.shape
        chosen: Optional[int] = None
        chosen_distance = math.inf
        chosen_area = 0

        # Work from all dark pixels once. The radius loop preserves the old local
        # association envelope, while area filtering happens before nearest-point
        # selection so rejected noise cannot starve a valid neighboring profile.
        yy_all, xx_all = np.nonzero(mask)
        if len(xx_all):
            labels_all = labels[yy_all, xx_all]
            areas_all = stats[labels_all, cv2.CC_STAT_AREA]
            dx_all = xx_all.astype(float) - float(seed_px[0])
            dy_all = yy_all.astype(float) - float(seed_px[1])
            dist2_all = dx_all * dx_all + dy_all * dy_all

            for radius in (8, 16, 32, 64, 96, 140):
                inside = (
                    (np.abs(dx_all) <= float(radius))
                    & (np.abs(dy_all) <= float(radius))
                    & (labels_all > 0)
                    & (areas_all >= 50)
                )
                if not np.any(inside):
                    continue
                candidate_indices = np.flatnonzero(inside)
                local = int(candidate_indices[int(np.argmin(dist2_all[candidate_indices]))])
                chosen = int(labels_all[local])
                chosen_distance = math.sqrt(float(dist2_all[local]))
                chosen_area = int(stats[chosen, cv2.CC_STAT_AREA])
                break

        if chosen is None:
            return None, "no qualifying connected dark component near estimated profile"
        component = np.zeros_like(mask)
        component[labels == chosen] = 255
        return component, (
            f"connected component anchored to current belief; area {chosen_area} px; "
            f"seed distance {chosen_distance:.2f} px"
        )

    @staticmethod
    def _adjacency(skel: np.ndarray) -> tuple[np.ndarray, dict[int, list[int]]]:
        yy, xx = np.nonzero(skel)
        points = np.column_stack([xx, yy]).astype(np.int32)
        key_to_index = {int(y) * skel.shape[1] + int(x): i for i, (x, y) in enumerate(points)}
        adjacency: dict[int, list[int]] = {i: [] for i in range(len(points))}
        w = skel.shape[1]
        for i, (x, y) in enumerate(points):
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    j = key_to_index.get((int(y) + dy) * w + (int(x) + dx))
                    if j is not None:
                        adjacency[i].append(j)
        return points, adjacency

    @staticmethod
    def _dijkstra(points: np.ndarray, adjacency: dict[int, list[int]], start: int, max_cost_px: float) -> tuple[np.ndarray, np.ndarray]:
        import heapq
        n = len(points)
        dist = np.full(n, np.inf, dtype=float)
        parent = np.full(n, -1, dtype=np.int32)
        dist[start] = 0.0
        heap: list[tuple[float, int]] = [(0.0, start)]
        while heap:
            cost, node = heapq.heappop(heap)
            if cost != dist[node] or cost > max_cost_px:
                continue
            x0, y0 = points[node]
            for nxt in adjacency[node]:
                x1, y1 = points[nxt]
                step = math.hypot(float(x1 - x0), float(y1 - y0))
                nc = cost + step
                if nc < dist[nxt] and nc <= max_cost_px:
                    dist[nxt] = nc
                    parent[nxt] = node
                    heapq.heappush(heap, (nc, nxt))
        return dist, parent

    @staticmethod
    def _reconstruct(parent: np.ndarray, endpoint: int) -> list[int]:
        path = [int(endpoint)]
        seen = {int(endpoint)}
        node = int(endpoint)
        while parent[node] >= 0:
            node = int(parent[node])
            if node in seen:
                break
            path.append(node); seen.add(node)
        path.reverse()
        return path

    @staticmethod
    def _resample_polyline(points: np.ndarray, spacing: float) -> tuple[np.ndarray, np.ndarray]:
        if len(points) < 2:
            return points, np.zeros(len(points), dtype=float)
        seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        total = float(s[-1])
        if total <= 1e-9:
            return points[:1], np.asarray([0.0])
        sample_s = np.arange(0.0, total + spacing * 0.5, spacing)
        if sample_s[-1] < total:
            sample_s = np.append(sample_s, total)
        x = np.interp(sample_s, s, points[:, 0])
        y = np.interp(sample_s, s, points[:, 1])
        return np.column_stack([x, y]), sample_s

    @staticmethod
    def _point_at_s(points: np.ndarray, s: np.ndarray, target_s: float) -> np.ndarray:
        if len(points) == 0:
            return np.zeros(2, dtype=float)
        if len(points) == 1 or target_s <= 0.0:
            return points[0].copy()
        target_s = min(float(target_s), float(s[-1]))
        x = float(np.interp(target_s, s, points[:, 0]))
        y = float(np.interp(target_s, s, points[:, 1]))
        return np.asarray([x, y], dtype=float)

    def _candidate_paths(
        self,
        component: np.ndarray,
        camera: np.ndarray,
        seed_px: np.ndarray,
        reference_tangent_world: np.ndarray,
    ) -> tuple[list[CandidatePath], list[str]]:
        skeleton = _skeletonize(component)
        points_px, adjacency = self._adjacency(skeleton)
        rejects: list[str] = []
        if len(points_px) < 8:
            return [], ["connected component skeleton was too small"]
        start = int(np.argmin(np.sum((points_px.astype(float) - seed_px) ** 2, axis=1)))
        # Convert 0.45 in to a conservative pixel/geodesic budget using the two axes.
        px_per_in = max(20.0, 0.5 * (np.linalg.norm(self.machine_to_pixel[:, 0]) + np.linalg.norm(self.machine_to_pixel[:, 1])))
        dist_px, parent = self._dijkstra(points_px, adjacency, start, 0.45 * px_per_in)
        finite = np.flatnonzero(np.isfinite(dist_px))
        if len(finite) < 8:
            return [], ["not enough connected skeleton ahead"]

        ref_px = _unit(self._world_to_pixel_delta(reference_tangent_world))
        # Consider endpoints close to the farthest reachable geodesic distance in
        # both branches. Dijkstra naturally supplies both directions on a circle.
        max_d = float(np.max(dist_px[finite]))
        threshold = max(0.055 * px_per_in, min(max_d * 0.65, 0.28 * px_per_in))
        far_endpoints = {int(i) for i in finite if dist_px[i] >= threshold}
        # Also preserve the end of each visible branch even when one side is much
        # shorter than the other. This matters on curves near the camera FOV edge
        # and near real open-profile endpoints; otherwise the long opposite branch
        # can hide the valid forward continuation.
        min_branch = 0.040 * px_per_in
        branch_endpoints: set[int] = set()
        for i in finite:
            if dist_px[i] < min_branch:
                continue
            finite_neighbors = [j for j in adjacency[int(i)] if np.isfinite(dist_px[j])]
            if len(finite_neighbors) <= 1 or all(dist_px[j] <= dist_px[i] + 1e-9 for j in finite_neighbors):
                branch_endpoints.add(int(i))
        # Branch ends are deliberately evaluated first. A dense set of cutoff
        # samples from the long branch must never crowd the shorter forward branch
        # out of the endpoint budget.
        endpoint_pool = sorted(branch_endpoints, key=lambda i: float(dist_px[i]), reverse=True)
        endpoint_pool += [i for i in sorted(far_endpoints, key=lambda i: float(dist_px[i]), reverse=True) if i not in branch_endpoints]
        endpoints: list[int] = []
        for node in endpoint_pool:
            p = points_px[node].astype(float)
            # Keep endpoints from both visible branches. A hard top-10-by-distance
            # cap can accidentally keep ten samples from the long *backward* branch
            # and discard the shorter forward branch on a curve.
            if all(float(np.linalg.norm(p - points_px[e])) > 14.0 for e in endpoints):
                endpoints.append(node)
            if len(endpoints) >= 60:
                break

        candidates: list[CandidatePath] = []
        h, w = component.shape
        for endpoint in endpoints:
            ids = self._reconstruct(parent, endpoint)
            if len(ids) < 5:
                continue
            pixel_path = points_px[ids].astype(float)
            world_path = np.asarray([self._pixel_to_world(camera, p[0], p[1], w, h) for p in pixel_path], dtype=float)
            world_path, s = self._resample_polyline(world_path, 0.010)
            if len(world_path) < 4 or float(s[-1]) < 0.045:
                continue
            initial_target = self._point_at_s(world_path, s, min(0.045, float(s[-1])))
            initial_vec = _unit(initial_target - world_path[0])
            start_alignment = float(np.dot(initial_vec, reference_tangent_world))
            # Do not let a candidate whose first meaningful motion goes backward
            # compete with the forward continuation.
            if start_alignment < -0.10:
                rejects.append(f"candidate rejected: starts backward ({start_alignment:+.2f})")
                continue
            end_start = self._point_at_s(world_path, s, max(0.0, float(s[-1]) - min(0.06, float(s[-1]) * 0.25)))
            end_vec = _unit(world_path[-1] - end_start)
            predicted = np.asarray(reference_tangent_world, dtype=float)
            end_alignment = float(np.dot(end_vec, predicted))
            turn = _angle_between_degrees(initial_vec, end_vec)
            length_score = min(1.0, float(s[-1]) / 0.25)
            alignment_score = max(0.0, min(1.0, (start_alignment + 1.0) * 0.5))
            # Favor long visible continuations and initial continuity. Do not
            # heavily penalize future turns; look-ahead exists specifically so a
            # real corner/radius can be seen before we arrive at it.
            score = 0.58 * length_score + 0.34 * alignment_score + 0.08 * max(0.0, end_alignment)
            candidates.append(CandidatePath(
                points_world=tuple((float(p[0]), float(p[1])) for p in world_path),
                points_pixel=tuple((int(p[0]), int(p[1])) for p in pixel_path[::max(1, len(pixel_path)//120)]),
                arc_length=float(s[-1]),
                start_alignment=start_alignment,
                end_alignment=end_alignment,
                turn_degrees=turn,
                score=float(score),
                reason=f"connected skeleton path {s[-1]:.3f} in; start alignment {start_alignment:+.2f}; visible turn {turn:.1f}°",
            ))
        candidates.sort(key=lambda c: c.score, reverse=True)
        return candidates, rejects

    def plan(
        self,
        frame_bgr: np.ndarray,
        *,
        camera_x: float,
        camera_y: float,
        belief: BeliefState,
        frame_camera_x: Optional[float] = None,
        frame_camera_y: Optional[float] = None,
    ) -> PlannerResult:
        # M5.6 separates the current controller pose from the pose that produced
        # the delayed camera frame. Pixel/world association must use the frame
        # pose, while execution/safety remain anchored to the current pose.
        camera = np.asarray([float(camera_x), float(camera_y)], dtype=float)
        frame_camera = np.asarray([
            float(camera_x if frame_camera_x is None else frame_camera_x),
            float(camera_y if frame_camera_y is None else frame_camera_y),
        ], dtype=float)
        zero = (float(camera_x), float(camera_y))
        if not belief.measurement_valid:
            return self._refusal(camera, belief, "no valid profile measurement", ())
        if belief.measured_confidence < self.min_confidence:
            return self._refusal(camera, belief, f"measurement confidence {belief.measured_confidence:.1f}% below minimum", ())

        belief_tangent = _unit(np.asarray([belief.tangent_x, belief.tangent_y], dtype=float))

        h, w = frame_bgr.shape[:2]
        # Near-field precision anchors path association to the *current* measured
        # profile location. The persistent estimate supplies memory/heading, but is
        # deliberately not allowed to drag the connected-path seed behind a curve.
        seed_world = np.asarray([belief.measurement_x, belief.measurement_y], dtype=float)
        seed_delta_px = self._world_to_pixel_delta(seed_world - frame_camera)
        seed_px = np.asarray([w / 2.0 + seed_delta_px[0], h / 2.0 + seed_delta_px[1]], dtype=float)
        component, component_reason = self._seed_component(frame_bgr, seed_px)
        if component is None:
            return self._refusal(camera, belief, component_reason, ())

        if self.travel_unit is None:
            # The first camera frame gives an unoriented tangent. Examine both
            # connected directions and use visible path support to pick the initial
            # travel direction. The UI's Reverse option deliberately chooses the
            # other branch when both are available.
            pos_candidates, pos_rejects = self._candidate_paths(component, frame_camera, seed_px, belief_tangent)
            neg_candidates, neg_rejects = self._candidate_paths(component, frame_camera, seed_px, -belief_tangent)
            choices: list[tuple[float, np.ndarray, CandidatePath, list[str]]] = []
            if pos_candidates:
                choices.append((pos_candidates[0].score, belief_tangent, pos_candidates[0], pos_rejects))
            if neg_candidates:
                choices.append((neg_candidates[0].score, -belief_tangent, neg_candidates[0], neg_rejects))
            if not choices:
                return self._refusal(camera, belief, "no connected centerline candidate in either initial direction", tuple((pos_rejects + neg_rejects)[-8:]))
            choices.sort(key=lambda item: item[0], reverse=True)
            selected = choices[-1] if self.initial_reverse and len(choices) > 1 else choices[0]
            _score, reference, best, rejects = selected
            # Keep all same-direction alternatives for the confidence/uniqueness
            # calculation below.
            candidates = pos_candidates if float(np.dot(reference, belief_tangent)) >= 0.0 else neg_candidates
        else:
            reference = _unit(self.travel_unit)
            if float(np.dot(belief_tangent, reference)) < 0.0:
                belief_tangent = -belief_tangent
            # Blend memory with the current state estimate. Travel direction wins
            # enough to stop one odd frame from instantly flipping the planner.
            reference = _unit(reference * 0.68 + belief_tangent * 0.32)
            candidates, rejects = self._candidate_paths(component, frame_camera, seed_px, reference)
            if not candidates:
                return self._refusal(camera, belief, "no forward connected centerline candidate", tuple(rejects[-8:]))
            best = candidates[0]
        path = np.asarray(best.points_world, dtype=float)
        path, s = self._resample_polyline(path, 0.005)
        usable = min(float(best.arc_length), max(0.0, float(belief.usable_lookahead))) if belief.usable_lookahead > 0 else float(best.arc_length)
        # Dynamic steering look-ahead is a description of what we trust, not a
        # direct jump target. The executor still consumes only the first segment.
        turn_factor = max(0.35, 1.0 - min(1.0, best.turn_degrees / 100.0) * 0.55)
        confidence_factor = max(0.30, min(1.0, belief.belief_confidence / 80.0))
        steering_lookahead = min(usable, 0.20 * turn_factor * (0.70 + 0.30 * confidence_factor))
        steering_lookahead = max(min(0.05, usable), steering_lookahead) if usable > 0 else 0.0

        # Future tangent comes from the connected path at the steering horizon.
        future_p0 = self._point_at_s(path, s, max(0.0, steering_lookahead - 0.025))
        future_p1 = self._point_at_s(path, s, min(float(s[-1]), steering_lookahead + 0.025))
        predicted_tangent = _unit(future_p1 - future_p0, tuple(reference))
        if float(np.dot(predicted_tangent, reference)) < -0.25 and best.turn_degrees < 70.0:
            predicted_tangent = -predicted_tangent

        # Dynamic physical step: normal 0.030 on clear paths, smaller when the
        # visible continuation is short, confidence is low, or a large turn is near.
        step_scale = 1.0
        if usable < 0.10:
            step_scale *= max(0.40, usable / 0.10)
        if belief.belief_confidence < 55.0:
            step_scale *= max(0.45, belief.belief_confidence / 55.0)
        if best.turn_degrees > 35.0:
            step_scale *= max(0.45, 1.0 - min(90.0, best.turn_degrees) / 140.0)
        forward_step = max(0.008, self.follow_step * step_scale)

        # The estimator may intentionally lag the physical camera slightly. Anchor
        # execution at the point on the extracted path nearest the *current camera*,
        # then consume one forward segment from there. This keeps estimator memory
        # from turning into a backward controller command.
        anchor_index = int(np.argmin(np.sum((path - camera) ** 2, axis=1)))
        anchor_s = float(s[anchor_index])
        cross_track = float(np.linalg.norm(path[anchor_index] - camera))
        remaining = max(0.0, float(s[-1]) - anchor_s)
        if remaining < 0.004:
            return self._refusal(camera, belief, "connected path ends at the current camera position", tuple(rejects[-8:]))
        path_target = self._point_at_s(path, s, anchor_s + min(forward_step, remaining))
        proposed = path_target
        move = proposed - camera
        move_len = float(np.linalg.norm(move))

        # Planner confidence combines belief quality, available connected path,
        # candidate uniqueness, and whether the chosen path starts forward.
        path_factor = min(1.0, usable / 0.18)
        unique_gap = (best.score - candidates[1].score) if len(candidates) > 1 else 0.25
        uniqueness = max(0.25, min(1.0, 0.5 + unique_gap * 2.0))
        planner_confidence = max(0.0, min(100.0,
            0.50 * belief.belief_confidence + 30.0 * path_factor + 20.0 * uniqueness
        ))
        curvature_factor = 1.0 / (1.0 + abs(float(belief.curvature)) * 0.18)
        turn_speed = max(0.30, 1.0 - min(100.0, best.turn_degrees) / 130.0)
        recommended_velocity = self.base_feed * max(0.15, min(1.0,
            (planner_confidence / 100.0) * curvature_factor * turn_speed
        ))

        # ---- independent hard safety gate ----
        max_move = self.follow_step + min(self.max_correct, 0.015)
        safety_action = "PASS"
        approved = proposed.copy()
        if move_len > max_move and move_len > 1e-9:
            approved = camera + move / move_len * max_move
            safety_action = f"CLAMP move to {max_move:.3f} in"
            move = approved - camera
            move_len = float(np.linalg.norm(move))

        # Reversal safety applies to *path progress*, not the total camera move.
        # The total move may legitimately point sideways while correcting cross-track
        # error.  Treating that correction vector as travel caused the first real G
        # test to HOLD after a correction-heavy step even though the connected path
        # ahead remained valid.
        path_forward = path_target - path[anchor_index]
        path_forward_len = float(np.linalg.norm(path_forward))
        if self.travel_unit is not None and path_forward_len > 1e-9:
            forward_dot = float(np.dot(_unit(path_forward), _unit(self.travel_unit)))
            if forward_dot < -0.20:
                return self._refusal(camera, belief,
                    f"SAFETY REFUSAL: connected path progress reverses established travel (dot {forward_dot:+.2f})",
                    tuple(rejects[-8:] + [best.reason]))

        if move_len < 0.003:
            return self._refusal(camera, belief, "SAFETY HOLD: proposed move collapsed below 0.003 in", tuple(rejects[-8:]))

        reason = (
            f"selected connected centerline candidate; {best.reason}; "
            f"dynamic steering look-ahead {steering_lookahead:.3f} in; "
            f"execute first {move_len:.3f} in only; safety {safety_action}"
        )
        return PlannerResult(
            accepted=True,
            safety_action=safety_action,
            camera_x=float(camera[0]), camera_y=float(camera[1]),
            proposed_x=float(proposed[0]), proposed_y=float(proposed[1]),
            approved_x=float(approved[0]), approved_y=float(approved[1]),
            move_x=float(move[0]), move_y=float(move[1]), move_length=move_len,
            forward_step=forward_step, cross_track_correction=cross_track,
            local_path_points=tuple((float(p[0]), float(p[1])) for p in path),
            current_tangent_x=float(reference[0]), current_tangent_y=float(reference[1]),
            predicted_tangent_x=float(predicted_tangent[0]), predicted_tangent_y=float(predicted_tangent[1]),
            curvature=float(belief.curvature), usable_lookahead=usable,
            steering_lookahead=steering_lookahead, recommended_velocity=recommended_velocity,
            planner_confidence=planner_confidence, path_turn_degrees=best.turn_degrees,
            candidate_count=len(candidates), rejected_candidates=tuple(rejects[-8:]),
            decision_reason=reason,
        )

    def _refusal(self, camera: np.ndarray, belief: BeliefState, reason: str, rejects: tuple[str, ...]) -> PlannerResult:
        reference = self.travel_unit if self.travel_unit is not None else np.asarray([belief.tangent_x, belief.tangent_y], dtype=float)
        reference = _unit(reference)
        return PlannerResult(
            accepted=False, safety_action="HOLD", camera_x=float(camera[0]), camera_y=float(camera[1]),
            proposed_x=float(camera[0]), proposed_y=float(camera[1]), approved_x=float(camera[0]), approved_y=float(camera[1]),
            move_x=0.0, move_y=0.0, move_length=0.0, forward_step=0.0, cross_track_correction=0.0,
            local_path_points=(), current_tangent_x=float(reference[0]), current_tangent_y=float(reference[1]),
            predicted_tangent_x=float(reference[0]), predicted_tangent_y=float(reference[1]),
            curvature=float(belief.curvature), usable_lookahead=float(belief.usable_lookahead), steering_lookahead=0.0,
            recommended_velocity=0.0, planner_confidence=0.0, path_turn_degrees=0.0, candidate_count=0,
            rejected_candidates=rejects, decision_reason=reason,
        )
