from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
import math
from typing import Any, Optional

import cv2
import numpy as np

from fabscan.path_belief import BeliefState, BoundaryObservation, ContextResult
from fabscan.sharp_vertex import (
    ARRIVAL_TOLERANCE, EXIT_DISTANCE, LEG_SUPPORT, CORNER_LEG_START, CORNER_RADIUS,
    VertexCommitment, find_sharp_vertex,
)

INTEGRATION_VERSION = "0.6.0-alpha.1-m6.4.3"


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
    boundary_points_world: tuple[tuple[float, float], ...] = ()

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
    motion_mode: str = "boundary_follow"
    vertex_diagnostics: dict[str, Any] = field(default_factory=dict)

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
        line_mode = str(settings.get("line_mode_var", settings.get("line_mode", "Line center")) or "Line center").strip()
        self.line_mode = line_mode if line_mode in {"Line center", "Edge near center"} else "Line center"
        self.line_search_px = max(40, min(1000, int(round(_float(
            settings.get("line_search_px_var", settings.get("line_search_px", 220)), 220
        )))))
        self.travel_unit: Optional[np.ndarray] = None
        self.initial_reverse = False
        self.camera_history: list[tuple[float, float]] = []
        self.total_distance = 0.0
        self._last_camera: Optional[np.ndarray] = None
        self.vertex = VertexCommitment()
        self.pixel_size = float(np.linalg.norm(self.pixel_to_machine, ord=2))

    def reset(self, *, reverse_initial: bool = False) -> None:
        self.travel_unit = None
        self.initial_reverse = bool(reverse_initial)
        self.camera_history.clear()
        self.total_distance = 0.0
        self._last_camera = None
        self.vertex = VertexCommitment()

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

        previous_vertex_state = self.vertex.state
        locked_heading = self.vertex.record_executed_pose(point)
        if locked_heading is not None:
            self.travel_unit = locked_heading
            if self.vertex.state != previous_vertex_state:
                # The incoming trailing chord must not vote on outgoing travel.
                self.camera_history = [(float(x), float(y))]
            return

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

    def _path_representation(
        self,
        component: np.ndarray,
        seed_px: np.ndarray,
    ) -> tuple[np.ndarray, str, str]:
        """Choose connected-path geometry without changing the user's Line/Edge intent.

        Line mode keeps the established skeleton-centerline behavior. In Edge mode,
        a large/deep filled component is always treated as a boundary problem:
        use a substantial nearby contour, or HOLD rather than silently falling back
        to the filled region's medial-axis skeleton.
        """
        h, w = component.shape
        area_px = int(np.count_nonzero(component))
        frame_area = max(1, int(h) * int(w))
        area_fraction = float(area_px) / float(frame_area)

        max_interior_radius_px = 0.0
        if area_px > 0:
            distance = cv2.distanceTransform(component, cv2.DIST_L2, 3)
            if distance.size:
                max_interior_radius_px = float(np.max(distance))

        # At 800x600, a ~40 px regression stroke has an interior radius near
        # 20 px. Filled tabletop/part regions are normally much deeper/larger.
        deep_region_limit_px = max(28.0, 0.06 * float(min(h, w)))
        region_like = (
            area_fraction >= 0.12
            or max_interior_radius_px >= deep_region_limit_px
        )
        edge_mode = self.line_mode == "Edge near center"

        if region_like and edge_mode:
            contours, _hierarchy = cv2.findContours(
                component.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE
            )
            px_per_in = max(
                20.0,
                0.5 * (
                    np.linalg.norm(self.machine_to_pixel[:, 0])
                    + np.linalg.norm(self.machine_to_pixel[:, 1])
                ),
            )
            min_contour_perimeter_px = max(20.0, 0.06 * px_per_in)
            chosen = None
            chosen_distance = math.inf
            chosen_length = 0
            sx, sy = float(seed_px[0]), float(seed_px[1])
            seed_point = np.asarray([sx, sy], dtype=float)

            for contour in contours:
                pts = contour.reshape(-1, 2)
                if len(pts) < 8:
                    continue
                if float(cv2.arcLength(contour, True)) < min_contour_perimeter_px:
                    continue
                delta = pts.astype(float) - seed_point
                distance = math.sqrt(float(np.min(np.sum(delta * delta, axis=1))))
                if (
                    distance < chosen_distance - 1e-9
                    or (abs(distance - chosen_distance) <= 1e-9 and len(pts) > chosen_length)
                ):
                    chosen = contour
                    chosen_distance = distance
                    chosen_length = len(pts)

            # Edge mode may legitimately put the measured seed farther from the
            # threshold contour at a sharp vertex. Give it a bounded portion of the
            # user's search window, but never reinterpret the filled region as a
            # centerline if the boundary is not credible.
            boundary_seed_limit_px = max(
                15.0,
                min(80.0, 0.10 * float(self.line_search_px)),
            )
            if chosen is not None and chosen_distance <= boundary_seed_limit_px:
                boundary = np.zeros_like(component)
                cv2.drawContours(boundary, [chosen], -1, 255, 1)
                return boundary, "boundary", (
                    f"Edge mode boundary selected: component {area_fraction * 100.0:.1f}% of frame, "
                    f"max interior radius {max_interior_radius_px:.1f} px, "
                    f"seed-to-boundary {chosen_distance:.2f} px "
                    f"(limit {boundary_seed_limit_px:.2f} px)"
                )

            if chosen is None:
                reason = (
                    f"Edge mode requires a boundary for region-like component "
                    f"({area_fraction * 100.0:.1f}% of frame, max interior radius "
                    f"{max_interior_radius_px:.1f} px), but no substantial contour was found; "
                    f"centerline fallback disabled"
                )
            else:
                reason = (
                    f"Edge mode requires a boundary for region-like component; nearest substantial "
                    f"boundary is {chosen_distance:.2f} px from seed, beyond "
                    f"{boundary_seed_limit_px:.2f} px limit; centerline fallback disabled"
                )
            return np.zeros_like(component), "boundary", reason

        skeleton = _skeletonize(component)
        if region_like and not edge_mode:
            mode_note = "; Line mode keeps centerline representation"
        elif edge_mode:
            mode_note = "; Edge mode component is not region-like, retaining existing centerline path"
        else:
            mode_note = ""
        return skeleton, "centerline", (
            f"centerline retained: component {area_fraction * 100.0:.1f}% of frame, "
            f"max interior radius {max_interior_radius_px:.1f} px{mode_note}"
        )

    def _candidate_paths(
        self,
        path_mask: np.ndarray,
        camera: np.ndarray,
        seed_px: np.ndarray,
        reference_tangent_world: np.ndarray,
        path_kind: str,
    ) -> tuple[list[CandidatePath], list[str]]:
        points_px, adjacency = self._adjacency(path_mask)
        rejects: list[str] = []
        if len(points_px) < 8:
            return [], [f"connected {path_kind} path was too small"]
        start = int(np.argmin(np.sum((points_px.astype(float) - seed_px) ** 2, axis=1)))
        # Convert 0.45 in to a conservative pixel/geodesic budget using the two axes.
        px_per_in = max(20.0, 0.5 * (np.linalg.norm(self.machine_to_pixel[:, 0]) + np.linalg.norm(self.machine_to_pixel[:, 1])))
        dist_px, parent = self._dijkstra(points_px, adjacency, start, 0.45 * px_per_in)
        finite = np.flatnonzero(np.isfinite(dist_px))
        if len(finite) < 8:
            return [], [f"not enough connected {path_kind} path ahead"]

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
        h, w = path_mask.shape
        for endpoint in endpoints:
            ids = self._reconstruct(parent, endpoint)
            if len(ids) < 5:
                continue
            pixel_path = points_px[ids].astype(float)
            world_path = np.asarray([self._pixel_to_world(camera, p[0], p[1], w, h) for p in pixel_path], dtype=float)
            boundary_points = tuple(tuple(p) for p in world_path) if path_kind == "boundary" else ()
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
                reason=f"connected {path_kind} path {s[-1]:.3f} in; start alignment {start_alignment:+.2f}; visible turn {turn:.1f}°",
                boundary_points_world=boundary_points,
            ))
        candidates.sort(key=lambda candidate: candidate.score, reverse=True)
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
        observation_id: Any = None,
    ) -> PlannerResult:
        self._association_diagnostics = {}
        self._observation_id = belief.step_id if observation_id is None else observation_id
        plan = self._plan(
            frame_bgr, camera_x=camera_x, camera_y=camera_y, belief=belief,
            frame_camera_x=frame_camera_x, frame_camera_y=frame_camera_y,
        )
        camera = np.asarray([camera_x, camera_y], dtype=float)
        if not plan.accepted:
            self.vertex.hold(plan.decision_reason, camera)
        return replace(plan, vertex_diagnostics={**self.vertex.fields(camera), **self._association_diagnostics},
                       motion_mode=("vertex_approach" if self.vertex.state == "APPROACH" else
                                    "vertex_outgoing" if self.vertex.state == "OUTGOING" else plan.motion_mode))

    def _committed_corner_support(self, path_mask: np.ndarray, frame_camera: np.ndarray) -> str:
        """Validate a local connected transition, not an exact tip pixel.

        The fitting model excludes the central .015 inch of each straight leg.
        Require those leg endpoints to be joined by observed boundary INSIDE a
        .018-inch world-space disk. No dilation, gap bridging or target update.
        """
        vertex = self.vertex.vertex
        assert vertex is not None
        point = np.asarray(vertex.point)
        h, w = path_mask.shape
        center = np.asarray([w / 2, h / 2]) - self.machine_to_pixel @ (point - frame_camera)
        radius_px = int(np.ceil(np.linalg.norm(self.machine_to_pixel, ord=2) * CORNER_RADIUS)) + 2
        x0, y0 = np.floor(center - radius_px).astype(int)
        x1, y1 = np.ceil(center + radius_px).astype(int) + 1
        if x0 < 1 or y0 < 1 or x1 >= w or y1 >= h:
            return "committed corner neighborhood left the camera view"
        yy, xx = np.mgrid[y0:y1, x0:x1]
        pixels = np.stack((xx, yy), axis=-1)
        world = frame_camera - (pixels - [w / 2, h / 2]) @ self.pixel_to_machine.T
        local = ((np.linalg.norm(world - point, axis=-1) <= CORNER_RADIUS)
                 & (path_mask[y0:y1, x0:x1] > 0)).astype(np.uint8)
        self._association_diagnostics.update(vertex_corner_radius=CORNER_RADIUS, vertex_corner_connected=False)
        if not np.any(local):
            return "committed corner lost its local boundary transition"
        ys, xs = np.nonzero(local)
        boundary_px = np.column_stack((xs + x0, ys + y0))
        self._association_diagnostics["vertex_tip_distance_px"] = float(np.min(np.linalg.norm(boundary_px-center, axis=1)))
        _, labels = cv2.connectedComponents(local, connectivity=8)
        endpoint_labels = []
        for direction, sign in ((vertex.incoming, -1), (vertex.outgoing, 1)):
            endpoint = point + sign * CORNER_LEG_START * np.asarray(direction)
            endpoint_px = np.asarray([w / 2, h / 2]) - self.machine_to_pixel @ (endpoint - frame_camera)
            distances = np.linalg.norm(boundary_px - endpoint_px, axis=1)
            nearest = int(np.argmin(distances))
            if distances[nearest] > 4.0:
                return "committed corner lost boundary support at a transition endpoint"
            endpoint_labels.append(labels[ys[nearest], xs[nearest]])
        if endpoint_labels[0] != endpoint_labels[1]:
            return "committed corner boundary transition is disconnected"
        self._association_diagnostics["vertex_corner_connected"] = True
        return ""

    def _validate_committed_boundary(self, camera: np.ndarray, frame_camera: np.ndarray,
                                     path_mask: np.ndarray) -> str:
        vertex = self.vertex.vertex
        assert vertex is not None
        point = np.asarray(vertex.point)
        incoming = np.asarray(vertex.incoming)
        outgoing = np.asarray(vertex.outgoing)
        h, w = path_mask.shape
        distance_image = cv2.distanceTransform(255 - path_mask, cv2.DIST_L2, 3)
        approaching = self.vertex.state == "APPROACH"
        direction = incoming if approaching else outgoing
        progress = float(np.dot(camera - point, direction))
        lateral = float(np.linalg.norm(camera - point - progress * direction))
        if (approaching and not -0.220 - ARRIVAL_TOLERANCE <= progress <= ARRIVAL_TOLERANCE) or (
                not approaching and not -ARRIVAL_TOLERANCE <= progress <= EXIT_DISTANCE + ARRIVAL_TOLERANCE):
            return "camera outside committed leg extent"
        if lateral > self.max_correct + 0.015:
            return "camera left committed leg corridor"
        # Retire only support behind the actual controller pose, keeping one
        # arrival-tolerance overlap. Check the ENTIRE remaining approach (which
        # can exceed LEG_SUPPORT), then the outgoing leg. Sample at <= .005 in.
        incoming_span = max(ARRIVAL_TOLERANCE, -progress + ARRIVAL_TOLERANCE) if approaching else 0.0
        outgoing_start = 0.0 if approaching else max(0.0, progress - ARRIVAL_TOLERANCE)
        self._association_diagnostics.update(
            vertex_incoming_support_span=incoming_span,
            vertex_outgoing_support_start=outgoing_start,
        )
        ranges = (("incoming", incoming, -1, CORNER_LEG_START, max(CORNER_LEG_START, incoming_span)),
                  ("outgoing", outgoing, 1, max(CORNER_LEG_START, outgoing_start), LEG_SUPPORT))
        for name, leg, sign, low, high in ranges:
            # The central corner is validated as a connected transition below;
            # the 4-pixel straight-leg bound starts outside that neighborhood.
            if name == "incoming" and not approaching:
                continue
            offsets = np.linspace(low, high, max(2, int(np.ceil((high-low)/0.005))+1))
            samples = point + sign * offsets[:, None] * leg
            pixels = np.asarray([w / 2, h / 2]) - (self.machine_to_pixel @ (samples - frame_camera).T).T
            pixels = np.rint(pixels).astype(int)
            if np.any(pixels < 1) or np.any(pixels[:, 0] >= w-1) or np.any(pixels[:, 1] >= h-1):
                return "committed vertex legs left the camera view"
            support = distance_image[pixels[:, 1], pixels[:, 0]]
            residual = float(np.max(support))
            self._association_diagnostics[f"vertex_{name}_support_max_px"] = residual
            if residual > 4.0:
                return f"committed vertex lost boundary support ({name} remaining support residual {residual:.2f} px)"

        corner_reason = self._committed_corner_support(path_mask, frame_camera)
        if corner_reason:
            return corner_reason

        return ""

    def committed_observation(self, frame_bgr: np.ndarray, *, camera: np.ndarray,
                              frame_camera: np.ndarray, confidence: float) -> tuple[Optional[BoundaryObservation], str]:
        """Observe the currently confirmed leg only after fresh geometry validation."""
        self._association_diagnostics = {}
        if not self.vertex.active:
            return None, "no committed leg"
        if not math.isfinite(confidence) or confidence < self.min_confidence:
            return None, "committed observation has insufficient detector confidence"
        vertex = self.vertex.vertex
        point = np.asarray(vertex.point)
        approaching = self.vertex.state == "APPROACH"
        leg = np.asarray(vertex.incoming if approaching else vertex.outgoing)
        h, w = frame_bgr.shape[:2]
        projection = float(np.dot(frame_camera - point, leg))
        # Measure on a supported straight segment, outside the uncertain tip.
        along = np.clip(projection, -0.220, -CORNER_LEG_START) if approaching else np.clip(projection, CORNER_LEG_START, LEG_SUPPORT)
        anchor = point + along * leg
        seed_px = np.array([w / 2, h / 2]) - self.machine_to_pixel @ (anchor - frame_camera)
        component, reason = self._seed_component(frame_bgr, seed_px)
        if component is None:
            return None, reason
        mask, kind, reason = self._path_representation(component, seed_px)
        if kind != "boundary" or not np.any(mask):
            return None, reason
        reason = self._validate_committed_boundary(camera, frame_camera, mask)
        if reason:
            return None, reason
        yy, xx = np.nonzero(mask)
        pixels = np.column_stack((xx, yy))
        distances = np.linalg.norm(pixels - seed_px, axis=1)
        nearest = int(np.argmin(distances))
        if distances[nearest] > 4.0:
            return None, "committed observation lost straight-leg support"
        observed = frame_camera - self.pixel_to_machine @ (pixels[nearest] - [w / 2, h / 2])
        normal = np.array([-leg[1], leg[0]])
        # The location comes from the observed boundary's lateral offset. The
        # confirmed straight-leg model supplies direction and along-leg station.
        measured = point + projection * leg + np.dot(observed - anchor, normal) * normal
        source = "committed_incoming" if approaching else "committed_outgoing"
        context = ContextResult(found=True, usable_lookahead=LEG_SUPPORT,
                                seed_x_px=float(pixels[nearest, 0]), seed_y_px=float(pixels[nearest, 1]),
                                reason="fresh committed straight leg and connected corner revalidated")
        return BoundaryObservation(tuple(measured), tuple(leg), float(confidence), context,
                                   (self.vertex.vertex_id, self.vertex.state), source), ""

    def _committed_plan(self, camera: np.ndarray, frame_camera: np.ndarray,
                        belief: BeliefState, path_mask: np.ndarray) -> PlannerResult:
        """Consume one bounded segment of a fixed vertex/exit target.

        Revalidate the untraversed approach and outgoing support on this frame's
        gated boundary. Executed position, not detector drift or planned motion,
        retires incoming support. The frozen corner transition is always checked.
        """
        reason = self._validate_committed_boundary(camera, frame_camera, path_mask)
        if reason:
            return self._refusal(camera, belief, reason, ())
        vertex = self.vertex.vertex
        point = np.asarray(vertex.point)
        incoming = np.asarray(vertex.incoming)
        outgoing = np.asarray(vertex.outgoing)
        exit_point = point + outgoing * EXIT_DISTANCE
        approaching = self.vertex.state == "APPROACH"
        target = point if approaching else exit_point
        direction = incoming if approaching else outgoing
        delta = target - camera
        distance = float(np.linalg.norm(delta))
        if float(np.dot(delta, direction)) < -ARRIVAL_TOLERANCE:
            return self._refusal(camera, belief, "committed target was passed without confirmed arrival", ())
        if distance < ARRIVAL_TOLERANCE:
            return self._refusal(camera, belief, "committed target awaits executed-pose confirmation", ())
        # Do not leave a sub-minimum final step. A short final move still ends at
        # the vertex; no step may extend through it onto the outgoing leg.
        step = min(self.follow_step, distance)
        if 0.0 < distance - step < ARRIVAL_TOLERANCE and distance >= 2 * ARRIVAL_TOLERANCE:
            step = distance / 2.0
        move = delta * (step / distance)
        approved = camera + move
        confidence = min(float(belief.measured_confidence), float(belief.belief_confidence))
        mode = "vertex_approach" if approaching else "vertex_outgoing"
        reason = (
            f"{mode}: fixed vertex X{point[0]:.6f} Y{point[1]:.6f}; "
            f"remaining target distance {distance:.6f} in; remaining approach and outgoing boundary support revalidated"
        )
        return PlannerResult(
            accepted=True, safety_action="PASS", camera_x=float(camera[0]), camera_y=float(camera[1]),
            proposed_x=float(approved[0]), proposed_y=float(approved[1]),
            approved_x=float(approved[0]), approved_y=float(approved[1]),
            move_x=float(move[0]), move_y=float(move[1]), move_length=step,
            forward_step=step, cross_track_correction=0.0,
            local_path_points=tuple(tuple(p) for p in (camera, target)),
            current_tangent_x=float(direction[0]), current_tangent_y=float(direction[1]),
            predicted_tangent_x=float(outgoing[0]), predicted_tangent_y=float(outgoing[1]),
            curvature=0.0, usable_lookahead=LEG_SUPPORT, steering_lookahead=distance,
            recommended_velocity=self.base_feed * 0.3, planner_confidence=confidence,
            path_turn_degrees=vertex.turn_degrees, candidate_count=1, rejected_candidates=(),
            decision_reason=reason, motion_mode=mode,
        )

    def _plan(
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
            return self._refusal(camera, belief, belief.reason if self.vertex.active else "no valid profile measurement", ())
        if belief.measured_confidence < self.min_confidence:
            return self._refusal(camera, belief, f"measurement confidence {belief.measured_confidence:.1f}% below minimum", ())

        belief_tangent = _unit(np.asarray([belief.tangent_x, belief.tangent_y], dtype=float))

        h, w = frame_bgr.shape[:2]
        # Near-field precision anchors path association to the *current* measured
        # profile location. The persistent estimate supplies memory/heading, but is
        # deliberately not allowed to drag the connected-path seed behind a curve.
        seed_world = np.asarray([belief.measurement_x, belief.measurement_y], dtype=float)
        source = "detector"
        if self.vertex.active:
            vertex = self.vertex.vertex
            point = np.asarray(vertex.point)
            approaching = self.vertex.state == "APPROACH"
            leg = np.asarray(vertex.incoming if approaching else vertex.outgoing)
            # Associate in the frame's coordinate system, not the possibly newer
            # execution pose. This selects a component, never a new target.
            low, high = (-0.220, 0.0) if approaching else (0.0, EXIT_DISTANCE)
            seed_world = point + np.clip(np.dot(frame_camera - point, leg), low, high) * leg
            source = "committed_incoming" if approaching else "committed_outgoing"
        self._association_diagnostics.update(vertex_association_source=source,
            vertex_association_x=float(seed_world[0]), vertex_association_y=float(seed_world[1]))
        seed_delta_px = self._world_to_pixel_delta(seed_world - frame_camera)
        seed_px = np.asarray([w / 2.0 + seed_delta_px[0], h / 2.0 + seed_delta_px[1]], dtype=float)
        component, component_reason = self._seed_component(frame_bgr, seed_px)
        if component is None:
            return self._refusal(camera, belief, component_reason, ())
        path_mask, path_kind, extraction_reason = self._path_representation(component, seed_px)
        self._association_diagnostics["vertex_association_reason"] = extraction_reason
        if not np.any(path_mask):
            return self._refusal(camera, belief, extraction_reason, ())

        if self.vertex.active:
            if path_kind != "boundary":
                return self._refusal(camera, belief, "committed vertex requires the same boundary representation", ())
            return self._committed_plan(camera, frame_camera, belief, path_mask)

        if self.travel_unit is None:
            # The first camera frame gives an unoriented tangent. Examine both
            # connected directions and use visible path support to pick the initial
            # travel direction. The UI's Reverse option deliberately chooses the
            # other branch when both are available.
            pos_candidates, pos_rejects = self._candidate_paths(path_mask, frame_camera, seed_px, belief_tangent, path_kind)
            neg_candidates, neg_rejects = self._candidate_paths(path_mask, frame_camera, seed_px, -belief_tangent, path_kind)
            choices: list[tuple[float, np.ndarray, CandidatePath, list[str]]] = []
            if pos_candidates:
                choices.append((pos_candidates[0].score, belief_tangent, pos_candidates[0], pos_rejects))
            if neg_candidates:
                choices.append((neg_candidates[0].score, -belief_tangent, neg_candidates[0], neg_rejects))
            if not choices:
                return self._refusal(camera, belief, "no connected path candidate in either initial direction", tuple((pos_rejects + neg_rejects)[-8:]))
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
            candidates, rejects = self._candidate_paths(path_mask, frame_camera, seed_px, reference, path_kind)
            if not candidates:
                return self._refusal(camera, belief, "no forward connected path candidate", tuple(rejects[-8:]))
            best = candidates[0]
        path = np.asarray(best.points_world, dtype=float)
        if path_kind == "boundary":
            vertex, vertex_reason = find_sharp_vertex(np.asarray(best.boundary_points_world), pixel_size=self.pixel_size)
            self.vertex.observe(
                vertex, camera=camera, observation_id=self._observation_id,
                pixel_size=self.pixel_size, max_cross_track=self.max_correct + 0.015,
            )
            if vertex is None:
                self.vertex.reason = vertex_reason
            if self.vertex.active:
                self.travel_unit = np.asarray(self.vertex.vertex.incoming)
                return self._committed_plan(camera, frame_camera, belief, path_mask)
            if self.vertex.state == "CANDIDATE" and self.vertex.vertex is not None:
                if np.linalg.norm(np.asarray(self.vertex.vertex.point) - camera) < self.follow_step + ARRIVAL_TOLERANCE:
                    return self._refusal(camera, belief, "sharp vertex too close to cross before fresh-frame confirmation", ())
        else:
            self.vertex.reject_candidate("sharp-vertex commitment is limited to boundary paths", camera)
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
            f"selected connected {path_kind} candidate; {component_reason}; {extraction_reason}; "
            f"{best.reason}; dynamic steering look-ahead {steering_lookahead:.3f} in; "
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
