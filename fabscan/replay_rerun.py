from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import importlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Callable, Optional

import cv2

from fabscan.linuxcnc_status import LinuxCNCPositionStatus


INTEGRATION_VERSION = "0.6.0-dev-m3"


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


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"Expected JSON object in {path}")
    return value


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, raw in enumerate(handle, start=1):
            text = raw.strip()
            if not text:
                continue
            try:
                value = json.loads(text)
            except json.JSONDecodeError as exc:
                raise ValueError(f"Invalid JSON in {path.name} line {line_number}: {exc}") from exc
            if not isinstance(value, dict):
                raise ValueError(f"Expected object in {path.name} line {line_number}")
            rows.append(value)
    return rows


class _ValueVar:
    """Tiny Tk-variable stand-in for the headless calibration harness."""

    def __init__(self, value: Any = None) -> None:
        self._value = value

    def get(self) -> Any:
        return self._value

    def set(self, value: Any) -> None:
        self._value = value


class _ReplayStatusReader:
    """Read-only LinuxCNC status substitute.

    Each replay step supplies the recorded pre-move and post-move positions.
    No LinuxCNC module, command object, or jog API is used.
    """

    def __init__(self) -> None:
        self._statuses: list[LinuxCNCPositionStatus] = []
        self._last = self._make_status({})

    @staticmethod
    def _make_status(position: dict[str, Any]) -> LinuxCNCPositionStatus:
        xyz = (
            _float(position.get("x")),
            _float(position.get("y")),
            _float(position.get("z")),
        )
        return LinuxCNCPositionStatus(
            available=True,
            connected=True,
            error=None,
            task_state="ON",
            interp_state="IDLE",
            task_mode="MANUAL",
            homed_text="XYZ",
            all_xyz_homed=True,
            machine_position=xyz,
            work_position=xyz,
        )

    def set_step(self, before: dict[str, Any], after: dict[str, Any]) -> None:
        self._statuses = [self._make_status(before), self._make_status(after)]
        self._last = self._statuses[0]

    def read_status(self) -> LinuxCNCPositionStatus:
        if self._statuses:
            self._last = self._statuses.pop(0)
        return self._last


@dataclass(frozen=True)
class RerunStepResult:
    step_id: int
    step_label: str
    historical_terminal: str
    fresh_terminal: str
    historical_result: str
    fresh_result: str
    historical: dict[str, Any]
    fresh: dict[str, Any]
    fresh_detection: dict[str, Any]
    decision_match: bool
    heading_state_match: bool
    move_delta_x: float
    move_delta_y: float
    move_delta_length: float
    target_delta: float
    angle_delta_degrees: float
    confidence_delta: float
    classification: str
    notes: list[str]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class RerunReport:
    run_dir: str
    integration_version: str
    step_count: int
    decision_matches: int
    close_matches: int
    different_steps: int
    max_move_delta: float
    mean_move_delta: float
    max_target_delta: float
    max_angle_delta_degrees: float
    max_confidence_delta: float
    results: list[RerunStepResult]

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["results"] = [result.to_dict() for result in self.results]
        return data

    def summary_lines(self) -> list[str]:
        return [
            f"Run:                  {self.run_dir}",
            f"Steps rerun:          {self.step_count}",
            f"Decision matches:     {self.decision_matches}/{self.step_count}",
            f"Close comparisons:    {self.close_matches}/{self.step_count}",
            f"Different steps:      {self.different_steps}",
            f"Max move-vector delta:{self.max_move_delta:.6f}",
            f"Mean move delta:      {self.mean_move_delta:.6f}",
            f"Max target delta:     {self.max_target_delta:.6f}",
            f"Max angle delta:      {self.max_angle_delta_degrees:.3f} deg",
            f"Max confidence delta: {self.max_confidence_delta:.3f}%",
        ]


class ExistingPlannerRerunner:
    """Run the installed FabScan detector/planner against recorded evidence.

    The real CameraCalibrationDialog class supplies the perception, continuity,
    filtering, corner, delayed-target, virtual-target, and move-planning code.
    A headless instance receives recorded frames and positions. Motion sending,
    waits, dialogs, preview updates, and LinuxCNC access are replaced with safe
    read-only stand-ins.
    """

    _VARIABLE_DEFAULTS: dict[str, Any] = {
        "camera_index_var": 0,
        "camera_width_var": 800,
        "camera_height_var": 600,
        "camera_stream_max_fps_var": 30.0,
        "camera_preview_max_fps_var": 1.0,
        "linuxcnc_safe_preview_var": False,
        "profile_preview_var": False,
        "rotate_var": "0",
        "flip_x_var": False,
        "flip_y_var": False,
        "fine_rotation_var": 0.0,
        "threshold_var": 90,
        "show_dot_marker_var": False,
        "show_mask_var": False,
        "move_distance_var": 0.100,
        "feed_var": 5.0,
        "jog_step_var": 0.010,
        "center_max_move_var": 0.100,
        "line_mode_var": "Line center",
        "line_search_px_var": "220",
        "show_line_preview_var": True,
        "follow_step_var": 0.050,
        "follow_feed_var": 5.0,
        "follow_settle_ms_var": 150,
        "follow_max_heading_change_var": 70.0,
        "follow_corner_pause_var": False,
        "follow_corner_angle_var": "55",
        "follow_corner_assist_var": False,
        "follow_corner_lookahead_steps_var": 5,
        "follow_max_correct_var": 0.050,
        "follow_deadband_var": 0.003,
        "follow_gain_var": 0.50,
        "follow_stabilize_var": True,
        "follow_filter_offset_alpha_var": 0.35,
        "follow_filter_angle_alpha_var": 0.25,
        "follow_sanity_angle_var": 30.0,
        "follow_min_confidence_var": 45.0,
        "follow_direction_var": "Y+",
        "follow_capture_point_var": False,
        "follow_enabled_var": True,
        "follow_repeat_count_var": 1,
        "follow_timeline_log_var": False,
        "follow_use_delayed_position_var": False,
        "follow_position_delay_ms_var": 120,
        "follow_virtual_target_var": False,
        "follow_virtual_min_progress_pct_var": 70.0,
        "follow_record_run_var": False,
    }

    def __init__(self, dialog_cls: type, run_dir: Path | str) -> None:
        self.dialog_cls = dialog_cls
        self.run_dir = Path(run_dir).expanduser().resolve()
        self.manifest = _load_json(self.run_dir / "manifest.json")
        self.settings = _load_json(self.run_dir / "settings.json")
        self.calibration = _load_json(self.run_dir / "calibration.json")
        self.steps = _load_jsonl(self.run_dir / "steps.jsonl")
        events = _load_jsonl(self.run_dir / "events.jsonl")
        self.events_by_step: dict[int, list[dict[str, Any]]] = {}
        for event in events:
            self.events_by_step.setdefault(_int(event.get("step_id")), []).append(event)

        module = sys.modules.get(dialog_cls.__module__)
        if module is None:
            module = importlib.import_module(dialog_cls.__module__)
        self.LineDetection = getattr(module, "LineDetection")
        self.DotDetection = getattr(module, "DotDetection")
        self._harness = self._make_harness()
        self._context: dict[str, Any] = {}
        self._captured_events: list[dict[str, Any]] = []
        self._install_safe_hooks()

    def _make_harness(self) -> Any:
        h = self.dialog_cls.__new__(self.dialog_cls)
        h.linuxcnc_reader = _ReplayStatusReader()
        h.coordinate_mode_label = _text(
            self.calibration.get("coordinate_mode_label"), "Machine coordinates"
        )
        h.result = None
        h.cap = None
        h.current_frame_bgr = None
        h.current_frame_sequence = 0
        h.current_frame_timestamp = 0.0
        h.current_dot = self.DotDetection(False)
        h.current_line = self.LineDetection(False)
        h.preview_display_width = 700
        h.preview_display_height = 420
        h.after_job = None
        h._tk_preview = None
        h._closing = False
        h._motion_active = False
        h._manual_jog_active = False
        h._follow_stop_requested = False
        h._follow_heading_unit = None
        h._follow_last_move_unit = None
        h._follow_filtered_err = None
        h._follow_filtered_vec_px = None
        h._follow_recent_turn_degrees = []
        h._follow_recent_raw_angle_degrees = []
        h._last_follow_sanity_action = ""
        h._last_follow_sanity_reason = ""
        h._last_follow_sanity_change_degrees = None
        h._follow_corner_resume_once = False
        h._follow_corner_assist_consumed = False
        h._follow_corner_assist_cooldown_steps = 0
        h._follow_corner_assist_clear_frames = 0
        h._follow_corner_assist_clear_frame_limit = 2
        h._last_camera_sequence = 0
        h._last_preview_sequence = 0
        h._next_preview_due = 0.0
        h._preview_count = 0
        h._timeline_log_path = None
        h._timeline_log_file = None
        h._timeline_csv = None
        h._timeline_step_counter = 0
        h._follow_run_counter = 0
        h._active_follow_run_id = 0
        h._active_follow_run_step = 0
        h._position_history = []
        h._position_history_window_s = 8.0
        h._follow_virtual_target_xy = None
        h._follow_not_found_retry_count = 0
        h._follow_not_found_retry_timeout_s = 0.0
        h._last_follow_detection_sequence = 0
        h._last_follow_detection_result = ""
        h.trace_capture_callback = None
        h.active_calibration = dict(self.calibration)
        h._last_valid_line_search_px = _int(
            self.settings.get("line_search_px_var", self.settings.get("line_search_px", 220)),
            220,
        )

        for name, default in self._VARIABLE_DEFAULTS.items():
            value = self.settings.get(name, default)
            h.__dict__[name] = _ValueVar(value)
        h.dot_status_var = _ValueVar("")
        h.line_status_var = _ValueVar("")
        h.cal_status_var = _ValueVar("")
        h.transform_status_var = _ValueVar("")
        return h

    def _install_safe_hooks(self) -> None:
        h = self._harness
        h.update = lambda: None
        h._show_current_frame = lambda: None
        h._freshen_follow_frame_if_stale = lambda **_kwargs: None
        h._wait_and_pump_camera = lambda _seconds: None
        h._send_correction_jogs = lambda *_args, **_kwargs: True
        h._timeline_log = self._capture_event
        h._detect_line_for_follow_with_retries = self._detect_for_replay
        h._delayed_position_plan_fields = self._recorded_delayed_position

    def _capture_event(self, event: str, **kwargs: Any) -> None:
        self._captured_events.append({"event": str(event), **kwargs})

    def _event(self, step_id: int, name: str) -> Optional[dict[str, Any]]:
        for event in self.events_by_step.get(step_id, []):
            if _text(event.get("event")).upper() == name.upper():
                return event
        return None

    def _line_from_payload(self, payload: dict[str, Any]) -> Any:
        frame_w = _int(self._context.get("frame_w"), 800)
        frame_h = _int(self._context.get("frame_h"), 600)
        angle = _float(payload.get("angle_degrees"))
        vx = math.cos(math.radians(angle))
        vy = math.sin(math.radians(angle))
        err_x = _float(payload.get("pixel_error_x"))
        err_y = _float(payload.get("pixel_error_y"))
        found = _text(payload.get("result")).lower() == "found"
        return self.LineDetection(
            found=found,
            mode=_text(self._harness.line_mode_var.get(), "Line center"),
            x=(frame_w / 2.0) + err_x,
            y=(frame_h / 2.0) + err_y,
            vx=vx,
            vy=vy,
            pixel_error_x=err_x,
            pixel_error_y=err_y,
            angle_degrees=angle,
            confidence=_float(payload.get("confidence")),
            span_px=_float(payload.get("span_px")),
            width_px=_float(payload.get("width_px")),
            point_count=_int(payload.get("point_count")),
            search_px=_int(payload.get("search_px")),
            corner_candidate=bool(payload.get("corner_candidate")),
            corner_angle_degrees=_float(payload.get("corner_angle_degrees")),
            corner_strength=_float(payload.get("corner_strength")),
            corner_hough_segment_count=_int(payload.get("corner_hough_segment_count")),
            corner_cluster_count=_int(payload.get("corner_cluster_count")),
            corner_primary_angle_degrees=_float(payload.get("corner_primary_angle_degrees")),
            corner_secondary_angle_degrees=_float(payload.get("corner_secondary_angle_degrees")),
            corner_third_angle_degrees=_float(payload.get("corner_third_angle_degrees")),
            corner_primary_length_px=_float(payload.get("corner_primary_length_px")),
            corner_secondary_length_px=_float(payload.get("corner_secondary_length_px")),
            corner_third_length_px=_float(payload.get("corner_third_length_px")),
            corner_reject_reason=_text(payload.get("corner_reject_reason")),
            corner_intersection_valid=bool(payload.get("corner_intersection_valid")),
            corner_intersection_x=_float(payload.get("corner_intersection_x_px"), frame_w / 2.0),
            corner_intersection_y=_float(payload.get("corner_intersection_y_px"), frame_h / 2.0),
            message=_text(payload.get("reason"), "Line/edge found"),
        )

    def _detect_for_replay(
        self,
        *,
        step_id: int,
        step_label: str,
        phase: str,
        base_event: str,
    ) -> tuple[Any, float]:
        if phase == "pre_move":
            line = self._harness.detect_line()
        else:
            event = self._context.get("post_event")
            payload = event.get("payload", {}) if isinstance(event, dict) else {}
            line = (
                self._line_from_payload(payload)
                if payload
                else self.LineDetection(False, message="Recorded post detection is unavailable")
            )
        self._capture_event(
            base_event,
            step_id=step_id,
            step_label=step_label,
            result="found" if line.found else "not_found",
            reason=line.message,
            **self._harness._timeline_line_fields(line),
        )
        return line, 0.0

    def _recorded_delayed_position(
        self,
        *,
        current_x: float,
        current_y: float,
        frame_move_x: float,
        frame_move_y: float,
    ) -> tuple[float, float, dict[str, Any]]:
        step = self._context.get("step", {})
        delayed = step.get("delayed_position", {}) if isinstance(step, dict) else {}
        use_delayed = bool(self._harness.follow_use_delayed_position_var.get())
        base_x = _float(delayed.get("x"), current_x) if use_delayed else current_x
        base_y = _float(delayed.get("y"), current_y) if use_delayed else current_y
        fields = {
            "use_delayed_position": use_delayed,
            "position_delay_ms": _int(self._harness.follow_position_delay_ms_var.get()),
            "frame_move_x": f"{frame_move_x:.6f}",
            "frame_move_y": f"{frame_move_y:.6f}",
            "target_base_x": f"{base_x:.6f}",
            "target_base_y": f"{base_y:.6f}",
            "command_adjust_x": f"{base_x - current_x:.6f}",
            "command_adjust_y": f"{base_y - current_y:.6f}",
            "delayed_position_found": bool(use_delayed),
            "delayed_position_source": _text(delayed.get("source"), "recorded replay"),
            "delayed_position_age_ms": _text(delayed.get("age_ms")),
            "delayed_position_error_ms": "0.000",
            "delayed_x": f"{base_x:.6f}",
            "delayed_y": f"{base_y:.6f}",
        }
        return base_x, base_y, fields

    @staticmethod
    def _terminal_event(events: list[dict[str, Any]]) -> Optional[dict[str, Any]]:
        for name in ("MOVE_PLAN", "STEP_REFUSED", "MOVE_FAILED", "STEP_COMPLETE"):
            for event in events:
                if _text(event.get("event")).upper() == name:
                    return event
        return None

    @staticmethod
    def _classify(
        decision_match: bool,
        move_delta: float,
        target_delta: float,
        angle_delta: float,
    ) -> str:
        if not decision_match:
            return "DIFFERENT"
        if move_delta <= 0.001 and target_delta <= 0.001 and angle_delta <= 3.0:
            return "MATCH"
        if move_delta <= 0.003 and target_delta <= 0.003 and angle_delta <= 8.0:
            return "CLOSE"
        return "DIFFERENT"

    def run_all(
        self,
        progress: Optional[Callable[[int, int], None]] = None,
    ) -> RerunReport:
        h = self._harness
        h._begin_follow_run("Follow N Replay", requested_steps=len(self.steps), reset_latch=True)
        results: list[RerunStepResult] = []

        for index, step in enumerate(self.steps, start=1):
            self._captured_events = []
            step_id = _int(step.get("step_id"), index)
            frame_relative = _text(step.get("decision_frame"))
            frame_path = (self.run_dir / frame_relative).resolve()
            try:
                frame_path.relative_to(self.run_dir)
            except ValueError as exc:
                raise ValueError(f"Replay frame escapes run folder: {frame_relative}") from exc
            frame = cv2.imread(str(frame_path), cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError(f"Could not read replay frame: {frame_path}")

            historical = step.get("planner_output", {})
            if not isinstance(historical, dict):
                historical = {}
            h.current_frame_bgr = frame
            h.current_frame_sequence = _int(historical.get("frame_sequence"), index)
            h.current_frame_timestamp = _float(historical.get("frame_timestamp_s"), float(index))
            frame_h, frame_w = frame.shape[:2]
            self._context = {
                "step": step,
                "frame_w": frame_w,
                "frame_h": frame_h,
                "post_event": self._event(step_id, "POST_DETECTION"),
            }

            post_final = self._event(step_id, "POST_DETECTION_FINAL")
            post_payload = post_final.get("payload", {}) if isinstance(post_final, dict) else {}
            position_after = step.get("position_after", {})
            if not isinstance(position_after, dict):
                position_after = {}
            post_position = {
                "x": post_payload.get("post_x", position_after.get("x", 0.0)),
                "y": post_payload.get("post_y", position_after.get("y", 0.0)),
                "z": 0.0,
            }
            position_before = step.get("position_before", {})
            if not isinstance(position_before, dict):
                position_before = {}
            h.linuxcnc_reader.set_step(position_before, post_position)

            ok = h._follow_line_step_impl(
                step_label=_text(step.get("step_label"), f"Follow {index}/{len(self.steps)}"),
                show_dialogs=False,
            )
            terminal = self._terminal_event(self._captured_events)
            fresh = dict(terminal or {})
            fresh_terminal = _text(fresh.pop("event", "STEP_REFUSED" if not ok else ""))
            detection_event = next(
                (
                    event
                    for event in self._captured_events
                    if _text(event.get("event")).upper() == "DETECTION"
                ),
                {},
            )
            fresh_detection = dict(detection_event)
            fresh_detection.pop("event", None)

            historical_terminal = _text(step.get("terminal_event"))
            historical_result = _text(step.get("terminal_result"))
            fresh_result = _text(fresh.get("result"), "planned" if fresh_terminal == "MOVE_PLAN" else "")
            decision_match = fresh_terminal == historical_terminal
            if historical_terminal == "MOVE_PLAN" and fresh_terminal == "MOVE_PLAN":
                decision_match = True

            move_dx = _float(fresh.get("move_x")) - _float(historical.get("move_x"))
            move_dy = _float(fresh.get("move_y")) - _float(historical.get("move_y"))
            move_delta = math.hypot(move_dx, move_dy)
            target_delta = math.hypot(
                _float(fresh.get("target_x")) - _float(historical.get("target_x")),
                _float(fresh.get("target_y")) - _float(historical.get("target_y")),
            )
            angle_delta = abs(
                _float(fresh.get("angle_degrees", fresh_detection.get("angle_degrees")))
                - _float(historical.get("angle_degrees"))
            )
            confidence_delta = abs(
                _float(fresh.get("confidence", fresh_detection.get("confidence")))
                - _float(historical.get("confidence"))
            )
            heading_match = _text(fresh.get("heading_state")) == _text(historical.get("heading_state"))
            classification = self._classify(decision_match, move_delta, target_delta, angle_delta)
            notes: list[str] = []
            if not decision_match:
                notes.append(f"historical {historical_terminal or '—'} vs fresh {fresh_terminal or '—'}")
            if not heading_match:
                notes.append("heading-state text differs")
            if move_delta > 0.001:
                notes.append(f"move-vector delta {move_delta:.6f}")
            if angle_delta > 3.0:
                notes.append(f"fit-angle delta {angle_delta:.3f}°")

            results.append(
                RerunStepResult(
                    step_id=step_id,
                    step_label=_text(step.get("step_label")),
                    historical_terminal=historical_terminal,
                    fresh_terminal=fresh_terminal,
                    historical_result=historical_result,
                    fresh_result=fresh_result,
                    historical=dict(historical),
                    fresh=fresh,
                    fresh_detection=fresh_detection,
                    decision_match=decision_match,
                    heading_state_match=heading_match,
                    move_delta_x=move_dx,
                    move_delta_y=move_dy,
                    move_delta_length=move_delta,
                    target_delta=target_delta,
                    angle_delta_degrees=angle_delta,
                    confidence_delta=confidence_delta,
                    classification=classification,
                    notes=notes,
                )
            )
            if progress is not None:
                progress(index, len(self.steps))

        move_deltas = [result.move_delta_length for result in results]
        decision_matches = sum(1 for result in results if result.decision_match)
        close_matches = sum(1 for result in results if result.classification in {"MATCH", "CLOSE"})
        different = sum(1 for result in results if result.classification == "DIFFERENT")
        return RerunReport(
            run_dir=str(self.run_dir),
            integration_version=INTEGRATION_VERSION,
            step_count=len(results),
            decision_matches=decision_matches,
            close_matches=close_matches,
            different_steps=different,
            max_move_delta=max(move_deltas, default=0.0),
            mean_move_delta=(sum(move_deltas) / len(move_deltas)) if move_deltas else 0.0,
            max_target_delta=max((result.target_delta for result in results), default=0.0),
            max_angle_delta_degrees=max(
                (result.angle_delta_degrees for result in results), default=0.0
            ),
            max_confidence_delta=max(
                (result.confidence_delta for result in results), default=0.0
            ),
            results=results,
        )


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Rerun the installed FabScan detector/planner against a recorded Follow run"
    )
    parser.add_argument("run_dir", type=Path)
    parser.add_argument(
        "--dialog-module",
        default="fabscan.camera_calibration",
        help="module containing CameraCalibrationDialog",
    )
    parser.add_argument("--dialog-class", default="CameraCalibrationDialog")
    parser.add_argument("--json", type=Path, help="optional comparison-report output")
    args = parser.parse_args(argv)

    module = importlib.import_module(args.dialog_module)
    dialog_cls = getattr(module, args.dialog_class)
    report = ExistingPlannerRerunner(dialog_cls, args.run_dir).run_all()
    print("\n".join(report.summary_lines()))
    if args.json is not None:
        args.json.write_text(json.dumps(report.to_dict(), indent=2), encoding="utf-8")
        print(f"Report written:        {args.json.resolve()}")
    return 0 if report.different_steps == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
