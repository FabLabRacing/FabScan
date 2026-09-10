from __future__ import annotations

from dataclasses import dataclass
import math
import time
import traceback
from pathlib import Path
from typing import Any, Optional

import numpy as np
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from fabscan.path_belief import BeliefState, PathBeliefEstimator
from fabscan.virtual_planner import ConnectedPathPlanner, PlannerResult

INTEGRATION_VERSION = "0.6.0-alpha.1-m6.3.1"


def _float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError, tk.TclError):
        return default


def _unit(x: float, y: float) -> Optional[tuple[float, float]]:
    n = math.hypot(float(x), float(y))
    if n <= 1e-12:
        return None
    return float(x) / n, float(y) / n


def _heading_degrees(x: float, y: float) -> Optional[float]:
    if math.hypot(float(x), float(y)) <= 1e-12:
        return None
    return math.degrees(math.atan2(float(y), float(x)))


def _heading_change_degrees(prev_x: float, prev_y: float, x: float, y: float) -> Optional[float]:
    a = _heading_degrees(prev_x, prev_y)
    b = _heading_degrees(x, y)
    if a is None or b is None:
        return None
    delta = (b - a + 180.0) % 360.0 - 180.0
    return abs(delta)


def _point_at_arc_length(path: np.ndarray, s: np.ndarray, target_s: float) -> np.ndarray:
    if len(path) == 0:
        return np.zeros(2, dtype=float)
    if len(path) == 1 or target_s <= 0.0:
        return path[0].copy()
    if target_s >= float(s[-1]):
        return path[-1].copy()
    idx = int(np.searchsorted(s, float(target_s), side="right"))
    idx = max(1, min(idx, len(path) - 1))
    s0 = float(s[idx - 1]); s1 = float(s[idx])
    if s1 - s0 <= 1e-12:
        return path[idx].copy()
    t = (float(target_s) - s0) / (s1 - s0)
    return path[idx - 1] * (1.0 - t) + path[idx] * t


def _controller_move_from_plan(dialog: Any, plan: PlannerResult, current_x: float, current_y: float) -> tuple[float, float, dict[str, Any]]:
    """Turn the planned path into one physical step.

    The planner describes a forward connected trajectory.  The physical controller
    separately applies the proven FabScan side-correction tuning so a large
    cross-track observation cannot consume an entire Follow Step or become the
    next definition of "forward".
    """
    path = np.asarray(plan.local_path_points, dtype=float)
    camera = np.asarray([float(current_x), float(current_y)], dtype=float)
    if path.ndim != 2 or path.shape[0] < 2 or path.shape[1] != 2:
        return float(plan.move_x), float(plan.move_y), {
            "controller_mode": "planner_move_fallback",
            "controller_forward_x": f"{plan.move_x:.6f}",
            "controller_forward_y": f"{plan.move_y:.6f}",
            "controller_raw_cross_track_x": "0.000000",
            "controller_raw_cross_track_y": "0.000000",
            "controller_raw_cross_track_len": "0.000000",
            "controller_correct_x": "0.000000",
            "controller_correct_y": "0.000000",
            "controller_correct_len": "0.000000",
            "controller_correction_state": "path unavailable; planner fallback",
            "controller_correction_limited": False,
        }

    segments = np.linalg.norm(np.diff(path, axis=0), axis=1)
    s = np.concatenate(([0.0], np.cumsum(segments)))
    anchor_index = int(np.argmin(np.sum((path - camera) ** 2, axis=1)))
    anchor = path[anchor_index]
    anchor_s = float(s[anchor_index])
    remaining = max(0.0, float(s[-1]) - anchor_s)
    forward_distance = min(max(0.0, float(plan.forward_step)), remaining)
    forward_target = _point_at_arc_length(path, s, anchor_s + forward_distance)
    forward = forward_target - anchor

    raw_correction = anchor - camera
    deadband = float(dialog._get_follow_deadband())
    gain = float(dialog._get_follow_gain())
    max_correct = float(dialog._get_follow_max_correct())
    correct_x, correct_y, limited, state, raw_len, tuned_len = dialog._apply_follow_correction_tuning(
        float(raw_correction[0]),
        float(raw_correction[1]),
        deadband=deadband,
        gain=gain,
        max_correct=max_correct,
    )
    move_x = float(forward[0]) + float(correct_x)
    move_y = float(forward[1]) + float(correct_y)
    fields = {
        "controller_mode": "path_progress_plus_tuned_cross_track",
        "controller_anchor_x": f"{anchor[0]:.6f}",
        "controller_anchor_y": f"{anchor[1]:.6f}",
        "controller_forward_x": f"{forward[0]:.6f}",
        "controller_forward_y": f"{forward[1]:.6f}",
        "controller_forward_len": f"{math.hypot(float(forward[0]), float(forward[1])):.6f}",
        "controller_raw_cross_track_x": f"{raw_correction[0]:.6f}",
        "controller_raw_cross_track_y": f"{raw_correction[1]:.6f}",
        "controller_raw_cross_track_len": f"{raw_len:.6f}",
        "controller_correct_x": f"{correct_x:.6f}",
        "controller_correct_y": f"{correct_y:.6f}",
        "controller_correct_len": f"{tuned_len:.6f}",
        "controller_correction_state": str(state),
        "controller_correction_limited": bool(limited),
        "controller_deadband": f"{deadband:.6f}",
        "controller_gain": f"{gain:.3f}",
        "controller_max_correct": f"{max_correct:.6f}",
    }
    return move_x, move_y, fields


def _settings_snapshot(dialog: Any) -> dict[str, Any]:
    """Capture the small settings surface consumed by M4/M5 planner code."""
    out: dict[str, Any] = {}
    for name in (
        "threshold_var",
        "follow_step_var",
        "follow_max_correct_var",
        "follow_feed_var",
        "follow_min_confidence_var",
        "line_search_px_var",
        "follow_filter_offset_alpha_var",
        "follow_filter_angle_alpha_var",
    ):
        var = getattr(dialog, name, None)
        getter = getattr(var, "get", None)
        if callable(getter):
            try:
                out[name] = getter()
            except Exception:
                pass
    # Explicit aliases make the planner robust to future UI-var renames.
    out["follow_step"] = float(dialog._get_follow_step())
    out["follow_max_correct"] = float(dialog._get_follow_max_correct())
    out["follow_feed"] = float(dialog._get_follow_feed())
    out["follow_min_confidence"] = float(dialog._get_follow_min_confidence())
    out["threshold"] = int(dialog._get_threshold())
    return out


def _line_payload(line: Any, move_x: float, move_y: float) -> dict[str, Any]:
    return {
        "result": "found" if bool(getattr(line, "found", False)) else "not_found",
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


@dataclass
class RealBeliefSession:
    estimator: PathBeliefEstimator
    planner: ConnectedPathPlanner
    calibration: dict[str, Any]
    settings: dict[str, Any]
    step_index: int = 0
    last_actual_x: Optional[float] = None
    last_actual_y: Optional[float] = None
    last_move_x: float = 0.0
    last_move_y: float = 0.0
    last_command_x: float = 0.0
    last_command_y: float = 0.0


def _find_follow_controls_parent(dialog: Any) -> Any:
    def walk(widget: Any) -> Any:
        try:
            children = widget.winfo_children()
        except Exception:
            return None
        for child in children:
            try:
                if str(child.cget("text")) == "Follow N":
                    return child.master
            except Exception:
                pass
            found = walk(child)
            if found is not None:
                return found
        return None
    return walk(dialog)


def _add_ui(dialog: Any) -> None:
    parent = _find_follow_controls_parent(dialog)
    if parent is None:
        raise RuntimeError("M6 could not locate the existing Follow controls")

    max_row = -1
    for child in parent.winfo_children():
        try:
            info = child.grid_info()
            max_row = max(max_row, int(info.get("row", -1)))
        except Exception:
            pass
    row = max_row + 1

    ttk.Separator(parent, orient=tk.HORIZONTAL).grid(
        row=row, column=0, columnspan=2, sticky="ew", pady=(10, 6)
    )
    row += 1
    ttk.Label(parent, text="M6.3 Real Belief Follow", font=("TkDefaultFont", 9, "bold")).grid(
        row=row, column=0, columnspan=2, sticky=tk.W
    )
    row += 1

    dialog.m6_real_arm_var = tk.BooleanVar(value=False)
    dialog.m6_real_max_move_var = tk.DoubleVar(value=0.035)
    dialog.m6_real_min_conf_var = tk.DoubleVar(value=45.0)
    dialog.m6_real_min_lookahead_var = tk.DoubleVar(value=0.045)
    dialog.m6_live_overlay_var = tk.BooleanVar(value=True)

    ttk.Checkbutton(
        parent,
        text="ARM real M6 motion",
        variable=dialog.m6_real_arm_var,
    ).grid(row=row, column=0, columnspan=2, sticky=tk.W)
    row += 1
    ttk.Checkbutton(
        parent,
        text="Live planner overlay",
        variable=dialog.m6_live_overlay_var,
        command=dialog._show_current_frame,
    ).grid(row=row, column=0, columnspan=2, sticky=tk.W)
    row += 1

    ttk.Button(
        parent,
        text="Open SVG Simulator…",
        command=dialog.m6_open_svg_simulator,
    ).grid(row=row, column=0, columnspan=2, sticky="ew", pady=(5, 1))
    row += 1

    ttk.Label(parent, text="Hard max move").grid(row=row, column=0, sticky=tk.W, pady=(4, 0))
    ttk.Entry(parent, textvariable=dialog.m6_real_max_move_var, width=8).grid(
        row=row, column=1, sticky="ew", padx=(4, 0), pady=(4, 0)
    )
    row += 1
    ttk.Label(parent, text="Min planner conf").grid(row=row, column=0, sticky=tk.W, pady=(4, 0))
    ttk.Entry(parent, textvariable=dialog.m6_real_min_conf_var, width=8).grid(
        row=row, column=1, sticky="ew", padx=(4, 0), pady=(4, 0)
    )
    row += 1
    ttk.Label(parent, text="Min look-ahead").grid(row=row, column=0, sticky=tk.W, pady=(4, 0))
    ttk.Entry(parent, textvariable=dialog.m6_real_min_lookahead_var, width=8).grid(
        row=row, column=1, sticky="ew", padx=(4, 0), pady=(4, 0)
    )
    row += 1

    ttk.Button(parent, text="M6.3 Dry Plan (NO MOVE)", command=dialog.m6_dry_plan).grid(
        row=row, column=0, columnspan=2, sticky="ew", pady=(7, 0)
    )
    row += 1
    ttk.Button(parent, text="M6.3 Belief Step", command=dialog.m6_follow_single_step).grid(
        row=row, column=0, columnspan=2, sticky="ew", pady=(4, 0)
    )
    row += 1
    ttk.Button(parent, text="M6.3 Belief N", command=dialog.m6_follow_multiple_steps).grid(
        row=row, column=0, columnspan=2, sticky="ew", pady=(4, 0)
    )
    row += 1
    ttk.Label(
        parent,
        text="Recorder is forced ON. Old Corner Assist / Virtual Target are bypassed.\n"
             "Recommended IPM is logged only; each real step is one coordinated X/Y move at Follow Feed.",
        wraplength=245,
        justify=tk.LEFT,
    ).grid(row=row, column=0, columnspan=2, sticky=tk.W, pady=(5, 0))



def m6_open_svg_simulator(dialog: Any) -> None:
    """Open the SVG simulator directly from Camera Calibration.

    The simulator still needs one recorded Follow run as its source for the
    saved detector settings and camera calibration.  M6.3.1 removes the old
    requirement to open the replay viewer first: choose the run folder here,
    then launch the simulator directly.
    """
    project_root = Path(__file__).resolve().parents[1]
    replay_root = project_root / "replays"
    initial_dir = replay_root if replay_root.is_dir() else project_root
    selected = filedialog.askdirectory(
        parent=dialog,
        title="Select FabScan replay/settings source for SVG Simulator",
        initialdir=str(initial_dir),
        mustexist=True,
    )
    if not selected:
        return

    try:
        from fabscan.svg_simulator import launch_svg_simulator

        window = launch_svg_simulator(
            parent=dialog,
            asset_dir=project_root / "simulator_assets",
            dialog_cls=type(dialog),
            run_dir=Path(selected),
        )
        # Keep an explicit reference for Tk lifetime/debugging and make the
        # most-recent source visible to the operator.
        dialog._m6_simulator_window = window
        dialog._m6_simulator_source = str(selected)
        try:
            dialog.cal_status_var.set(
                f"SVG Simulator opened using settings/calibration from {Path(selected).name}."
            )
        except Exception:
            pass
    except Exception as exc:
        messagebox.showerror(
            "SVG Simulator",
            f"Could not open SVG Simulator:\n\n{type(exc).__name__}: {exc}",
            parent=dialog,
        )


def _set_live_overlay_state(
    dialog: Any,
    *,
    step_label: str,
    current_x: float,
    current_y: float,
    belief: Optional[BeliefState] = None,
    plan: Optional[PlannerResult] = None,
    move_x: float = 0.0,
    move_y: float = 0.0,
    controller_fields: Optional[dict[str, Any]] = None,
    status: str = "IDLE",
    reason: str = "",
) -> None:
    """Store display-only M6 geometry for the normal camera preview.

    This deliberately does no perception or planning work. The preview renderer
    consumes only values already produced by the live M6 decision cycle.
    """
    state: dict[str, Any] = {
        "step_label": str(step_label),
        "camera_x": float(current_x),
        "camera_y": float(current_y),
        "status": str(status),
        "reason": str(reason),
        "move_x": float(move_x),
        "move_y": float(move_y),
        "controller_fields": dict(controller_fields or {}),
    }
    if belief is not None:
        state["belief"] = {
            "measurement_valid": bool(belief.measurement_valid),
            "measurement_x": float(belief.measurement_x),
            "measurement_y": float(belief.measurement_y),
            "estimate_x": float(belief.estimate_x),
            "estimate_y": float(belief.estimate_y),
            "predicted_points": tuple((float(x), float(y)) for x, y in belief.predicted_points),
            "belief_confidence": float(belief.belief_confidence),
        }
    if plan is not None:
        state["plan"] = {
            "accepted": bool(plan.accepted),
            "local_path_points": tuple((float(x), float(y)) for x, y in plan.local_path_points),
            "approved_x": float(plan.approved_x),
            "approved_y": float(plan.approved_y),
            "planner_confidence": float(plan.planner_confidence),
            "usable_lookahead": float(plan.usable_lookahead),
            "steering_lookahead": float(plan.steering_lookahead),
            "recommended_velocity": float(plan.recommended_velocity),
            "path_turn_degrees": float(plan.path_turn_degrees),
            "candidate_count": int(plan.candidate_count),
            "safety_action": str(plan.safety_action),
        }
    dialog._m6_live_overlay_state = state


def _m6_draw_live_overlay(dialog: Any, draw: Any, scale: float, source_w: int, source_h: int) -> None:
    """Draw the already-computed M6 plan on the normal live preview."""
    try:
        if not bool(getattr(dialog, "m6_live_overlay_var").get()):
            return
    except Exception:
        return
    state = getattr(dialog, "_m6_live_overlay_state", None)
    if not isinstance(state, dict):
        return
    calibration = dialog._validate_calibration(getattr(dialog, "active_calibration", None))
    if calibration is None:
        return
    try:
        matrix = np.asarray(calibration["matrix_machine_to_pixel"], dtype=float)
        camera = np.asarray([float(state["camera_x"]), float(state["camera_y"])], dtype=float)
    except Exception:
        return

    def world_to_display(x: float, y: float) -> tuple[float, float]:
        # Same sign convention used by ConnectedPathPlanner._world_to_pixel_delta.
        delta_px = -(matrix @ (np.asarray([float(x), float(y)], dtype=float) - camera))
        return (source_w * 0.5 + float(delta_px[0])) * scale, (source_h * 0.5 + float(delta_px[1])) * scale

    # Simulator-matching palette: cyan measurement, magenta belief, gold path,
    # green commanded progress, orange lateral correction/final target.
    belief = state.get("belief") or {}
    plan = state.get("plan") or {}
    try:
        if belief.get("measurement_valid"):
            mx, my = world_to_display(belief["measurement_x"], belief["measurement_y"]); r = 5
            draw.ellipse((mx-r, my-r, mx+r, my+r), outline=(0,216,255), width=3)
        if "estimate_x" in belief:
            ex, ey = world_to_display(belief["estimate_x"], belief["estimate_y"]); r = 7
            draw.line((ex-r, ey, ex+r, ey), fill=(216,76,255), width=3)
            draw.line((ex, ey-r, ex, ey+r), fill=(216,76,255), width=3)
        pred = list(belief.get("predicted_points") or ())
        if len(pred) >= 2:
            pts = [world_to_display(x, y) for x, y in pred]
            draw.line(pts, fill=(190,145,0), width=2)
    except Exception:
        pass

    path = list(plan.get("local_path_points") or ())
    if len(path) >= 2:
        try:
            pts = [world_to_display(x, y) for x, y in path]
            draw.line(pts, fill=(213,160,0), width=3)
        except Exception:
            pass

    cx, cy = world_to_display(float(state["camera_x"]), float(state["camera_y"]))
    cf = state.get("controller_fields") or {}
    fx = _float(cf.get("controller_forward_x"), 0.0)
    fy = _float(cf.get("controller_forward_y"), 0.0)
    forward_x = float(state["camera_x"]) + fx
    forward_y = float(state["camera_y"]) + fy
    fpx, fpy = world_to_display(forward_x, forward_y)
    target_x = float(state["camera_x"]) + _float(state.get("move_x"), 0.0)
    target_y = float(state["camera_y"]) + _float(state.get("move_y"), 0.0)
    tpx, tpy = world_to_display(target_x, target_y)
    if math.hypot(fx, fy) > 1e-9:
        draw.line((cx, cy, fpx, fpy), fill=(46,204,86), width=4)
    if math.hypot(tpx-fpx, tpy-fpy) > 1.0:
        draw.line((fpx, fpy, tpx, tpy), fill=(255,140,32), width=3)
    rr = 6
    draw.ellipse((tpx-rr, tpy-rr, tpx+rr, tpy+rr), outline=(46,204,86), width=3)

    # Compact status card. This is display-only and intentionally avoids any
    # extra detector/planner invocation in the preview thread.
    status = str(state.get("status", ""))
    reason = str(state.get("reason", ""))
    conf = _float(plan.get("planner_confidence"), 0.0)
    look = _float(plan.get("usable_lookahead"), 0.0)
    rec = _float(plan.get("recommended_velocity"), 0.0)
    move_len = math.hypot(_float(state.get("move_x")), _float(state.get("move_y")))
    corr = _float(cf.get("controller_correct_len"), 0.0)
    forward = math.hypot(fx, fy)
    turn = _float(plan.get("path_turn_degrees"), 0.0)
    lines = [
        f"M6.3 {status}   conf {conf:.0f}%   look {look:.3f} in",
        f"move {move_len:.4f}   forward {forward:.4f}   correction {corr:.4f} in",
        f"turn {turn:.1f} deg   rec {rec:.0f} IPM   candidates {int(plan.get('candidate_count', 0) or 0)}",
    ]
    if reason and status.upper().startswith("HOLD"):
        lines.append(reason[:92])
    x0, y0 = 10, 10
    line_h = 16
    card_w = min(int(source_w * scale) - 20, 610)
    card_h = line_h * len(lines) + 10
    draw.rectangle((x0-4, y0-4, x0+card_w, y0+card_h), fill=(0,0,0))
    for i, text in enumerate(lines):
        fill = (255,110,110) if (i == 0 and status.upper().startswith("HOLD")) else (255,255,255)
        draw.text((x0, y0 + i*line_h), text, fill=fill)

def _finish_recording(dialog: Any, *, status: str, reason: str) -> None:
    recorder = getattr(dialog, "_run_recorder", None)
    if recorder is None or not getattr(recorder, "active", False):
        return
    try:
        path = recorder.finish_run(status=status, reason=reason, wait=False)
        if path is not None:
            dialog._recording_last_path = path
    except Exception:
        traceback.print_exc()


def _m6_preflight(dialog: Any, *, require_arm: bool, show_dialogs: bool = True) -> Optional[dict[str, Any]]:
    def refuse(title: str, message: str) -> None:
        dialog.cal_status_var.set(message)
        if show_dialogs:
            messagebox.showinfo(title, message, parent=dialog)

    if require_arm and not bool(dialog.m6_real_arm_var.get()):
        refuse("M6 not armed", "Check ARM real M6 motion before allowing physical movement.")
        return None
    if getattr(dialog, "_motion_active", False) or getattr(dialog, "_manual_jog_active", False):
        refuse("Motion active", "Wait for the current motion to finish or press STOP Move.")
        return None
    if not bool(dialog.follow_enabled_var.get()):
        refuse("Follow disabled", "Check Enable follow before using M6 Belief Follow.")
        return None
    calibration = dialog._validate_calibration(getattr(dialog, "active_calibration", None))
    if calibration is None or "matrix_machine_to_pixel" not in calibration:
        refuse("No calibration", "M6 requires the current valid 2-D camera calibration.")
        return None
    if getattr(dialog, "current_frame_bgr", None) is None:
        refuse("No camera frame", "No camera frame is available yet.")
        return None
    recorder = getattr(dialog, "_run_recorder", None)
    if recorder is None or not hasattr(dialog, "follow_record_run_var"):
        refuse("Recorder required", "M6 physical tests require the M1 Follow Run Recorder.")
        return None
    # Physical M6 always records. Do this before _begin_follow_run so the recorder
    # wrapper starts the evidence package automatically.
    dialog.follow_record_run_var.set(True)
    return dict(calibration)


def _new_session(dialog: Any, calibration: dict[str, Any]) -> RealBeliefSession:
    settings = _settings_snapshot(dialog)
    estimator = PathBeliefEstimator(calibration, settings)
    planner = ConnectedPathPlanner(calibration, settings)
    planner.reset(reverse_initial=False)
    direction = dialog._get_follow_direction_preference()
    dx, dy = dialog._axis_vector_from_follow_direction(direction)
    planner.travel_unit = np.asarray([float(dx), float(dy)], dtype=float)

    status = dialog.linuxcnc_reader.read_status()
    if status.connected:
        x, y, _z = dialog._active_position(status)
        planner.record_executed_pose(x, y)
        last_x, last_y = float(x), float(y)
    else:
        last_x = last_y = None
    return RealBeliefSession(
        estimator=estimator,
        planner=planner,
        calibration=dict(calibration),
        settings=settings,
        last_actual_x=last_x,
        last_actual_y=last_y,
    )


def _frame_pose(dialog: Any, current_x: float, current_y: float) -> tuple[float, float, dict[str, Any]]:
    use_delayed = bool(dialog.follow_use_delayed_position_var.get())
    delay_ms = int(dialog._get_follow_position_delay_ms())
    fields: dict[str, Any] = {
        "use_delayed_position": use_delayed,
        "position_delay_ms": delay_ms,
        "delayed_position_found": False,
        "delayed_position_source": "current_position",
        "delayed_x": f"{current_x:.6f}",
        "delayed_y": f"{current_y:.6f}",
    }
    if not use_delayed:
        return current_x, current_y, fields
    frame_ts = float(getattr(dialog, "current_frame_timestamp", 0.0) or 0.0)
    if frame_ts <= 0.0:
        fields["delayed_position_source"] = "no frame timestamp; current position fallback"
        return current_x, current_y, fields
    target_time = frame_ts - delay_ms / 1000.0
    sample = dialog._lookup_position_history(target_time)
    if sample is None:
        fields["delayed_position_source"] = "no position history; current position fallback"
        return current_x, current_y, fields
    fields.update({
        "delayed_position_found": True,
        "delayed_position_source": str(sample.source),
        "delayed_position_age_ms": f"{(time.monotonic() - sample.timestamp_s) * 1000.0:.3f}",
        "delayed_position_error_ms": f"{(sample.timestamp_s - target_time) * 1000.0:.3f}",
        "delayed_x": f"{sample.x:.6f}",
        "delayed_y": f"{sample.y:.6f}",
    })
    return float(sample.x), float(sample.y), fields


def _build_belief(
    dialog: Any,
    session: RealBeliefSession,
    *,
    step_id: int,
    step_label: str,
    line: Any,
    transformed_frame: np.ndarray,
    current_x: float,
    current_y: float,
    frame_x: float,
    frame_y: float,
) -> BeliefState:
    # The first belief update has no previously executed move yet.  Seed the
    # line-orientation hint from the same Start dir already given to the planner.
    # Without this, the undirected line fit can initialize the estimator tangent
    # 180 degrees opposite to travel; step 2 then looks like a huge heading/position
    # innovation and falsely collapses planner confidence.
    orient_x = float(session.last_move_x)
    orient_y = float(session.last_move_y)
    if math.hypot(orient_x, orient_y) <= 1e-9 and session.planner.travel_unit is not None:
        orient_x = float(session.planner.travel_unit[0])
        orient_y = float(session.planner.travel_unit[1])
    payload = _line_payload(line, orient_x, orient_y)
    step = {
        "step_id": int(step_id),
        "step_label": str(step_label),
        "position_before": {"x": float(current_x), "y": float(current_y), "z": 0.0},
        "delayed_position": {"x": float(frame_x), "y": float(frame_y), "z": 0.0, "source": "M6 live frame pose"},
        "position_after": {"x": float(current_x), "y": float(current_y), "z": 0.0},
        "planner_output": payload,
    }
    state = session.estimator.update(step, transformed_frame, session.step_index)
    session.step_index += 1
    return state


def _log_belief(dialog: Any, *, step_id: int, step_label: str, belief: BeliefState, delay_fields: dict[str, Any]) -> None:
    dialog._timeline_log(
        "M6_BELIEF",
        step_id=step_id,
        step_label=step_label,
        result="valid" if belief.measurement_valid else "invalid",
        measurement_x=f"{belief.measurement_x:.6f}",
        measurement_y=f"{belief.measurement_y:.6f}",
        estimate_x=f"{belief.estimate_x:.6f}",
        estimate_y=f"{belief.estimate_y:.6f}",
        tangent_x=f"{belief.tangent_x:.6f}",
        tangent_y=f"{belief.tangent_y:.6f}",
        tangent_degrees=f"{belief.tangent_degrees:.3f}",
        curvature=f"{belief.curvature:.6f}",
        curvature_trend=f"{belief.curvature_trend:.6f}",
        implied_radius=f"{belief.implied_radius:.6f}",
        usable_lookahead=f"{belief.usable_lookahead:.6f}",
        prediction_horizon=f"{belief.prediction_horizon:.6f}",
        belief_confidence=f"{belief.belief_confidence:.3f}",
        position_innovation=f"{belief.position_innovation:.6f}",
        heading_innovation_degrees=f"{belief.heading_innovation_degrees:.3f}",
        reason=belief.reason,
        **delay_fields,
    )


def _plan_payload(plan: PlannerResult, *, current_x: float, current_y: float, move_x: float, move_y: float,
                  target_x: float, target_y: float, physical_safety: str) -> dict[str, Any]:
    return {
        "result": "move" if plan.accepted else "hold",
        "reason": plan.decision_reason,
        "planner_version": INTEGRATION_VERSION,
        "planner_type": "connected_path_belief",
        "current_x": f"{current_x:.6f}",
        "current_y": f"{current_y:.6f}",
        "target_x": f"{target_x:.6f}",
        "target_y": f"{target_y:.6f}",
        "move_x": f"{move_x:.6f}",
        "move_y": f"{move_y:.6f}",
        "move_len": f"{math.hypot(move_x, move_y):.6f}",
        "planner_proposed_x": f"{plan.proposed_x:.6f}",
        "planner_proposed_y": f"{plan.proposed_y:.6f}",
        "planner_approved_x": f"{plan.approved_x:.6f}",
        "planner_approved_y": f"{plan.approved_y:.6f}",
        "planner_safety_action": plan.safety_action,
        "physical_safety_action": physical_safety,
        "forward_step": f"{plan.forward_step:.6f}",
        "cross_track_correction": f"{plan.cross_track_correction:.6f}",
        "current_tangent_x": f"{plan.current_tangent_x:.6f}",
        "current_tangent_y": f"{plan.current_tangent_y:.6f}",
        "predicted_tangent_x": f"{plan.predicted_tangent_x:.6f}",
        "predicted_tangent_y": f"{plan.predicted_tangent_y:.6f}",
        "curvature": f"{plan.curvature:.6f}",
        "usable_lookahead": f"{plan.usable_lookahead:.6f}",
        "steering_lookahead": f"{plan.steering_lookahead:.6f}",
        "recommended_velocity": f"{plan.recommended_velocity:.3f}",
        "planner_confidence": f"{plan.planner_confidence:.3f}",
        "path_turn_degrees": f"{plan.path_turn_degrees:.3f}",
        "candidate_count": int(plan.candidate_count),
        "rejected_candidates": list(plan.rejected_candidates),
    }


def _execute_coordinated_xy_step(
    dialog: Any,
    *,
    step_id: int,
    step_label: str,
    move_x: float,
    move_y: float,
    target_x: float,
    target_y: float,
    feed: float,
    coordinate_mode: str,
) -> tuple[bool, dict[str, Any]]:
    """Execute one bounded M6 step as a single coordinated LinuxCNC XY move.

    M6.1 sent X and Y as two independent incremental jogs.  That made every
    diagonal planner vector into an X-then-Y dogleg and produced a visible jerk
    at the axis handoff.  M6.2 sends one guarded MDI G1 X/Y vector, waits for the
    endpoint and interpreter IDLE, then explicitly returns LinuxCNC to MANUAL
    before the next camera/planner cycle.
    """
    move_len = math.hypot(float(move_x), float(move_y))
    start_status = dialog.linuxcnc_reader.read_status()
    fields: dict[str, Any] = {
        "executor_mode": "coordinated_xy_mdi_g1",
        "executor_start_mode": str(getattr(start_status, "task_mode", "Unknown")),
        "executor_target_x": f"{target_x:.6f}",
        "executor_target_y": f"{target_y:.6f}",
        "executor_move_x": f"{move_x:.6f}",
        "executor_move_y": f"{move_y:.6f}",
        "executor_move_len": f"{move_len:.6f}",
        "executor_feed_ipm": f"{feed:.3f}",
        "executor_coordinate_mode": str(coordinate_mode),
        "executor_position_wait_ok": False,
        "executor_idle_wait_ok": False,
        "executor_manual_restore_ok": False,
        "executor_mdi_command": "",
    }

    dialog._timeline_log(
        "COORD_XY_START",
        step_id=step_id,
        step_label=step_label,
        result="begin",
        **fields,
    )
    started = time.monotonic()
    result = dialog.linuxcnc_reader.controlled_xy_move(target_x, target_y, feed, coordinate_mode)
    fields["executor_mdi_command"] = str(getattr(result, "mdi_command", ""))
    if not result.success:
        fields["executor_result"] = "command_refused"
        fields["executor_message"] = str(result.message)
        dialog._timeline_log(
            "COORD_XY_FAILED", step_id=step_id, step_label=step_label,
            result="failed", reason=result.message,
            duration_ms=f"{(time.monotonic() - started) * 1000.0:.3f}",
            **fields,
        )
        dialog.cal_status_var.set(result.message)
        return False, fields

    # Keep pumping the camera and position history while LinuxCNC traverses the
    # short vector, then require interpreter IDLE before changing task mode.
    position_ok = bool(dialog._wait_for_position_near(target_x, target_y, coordinate_mode, move_len))
    fields["executor_position_wait_ok"] = position_ok
    idle_ok = bool(dialog._wait_for_idle(6.0))
    fields["executor_idle_wait_ok"] = idle_ok

    if not position_ok or not idle_ok:
        abort = dialog.linuxcnc_reader.abort_motion()
        fields["executor_abort_sent"] = bool(abort.success)
        fields["executor_abort_message"] = str(abort.message)
        # Best effort: give an abort a short chance to return the interpreter to IDLE.
        if not idle_ok:
            idle_ok = bool(dialog._wait_for_idle(2.0))
            fields["executor_idle_wait_ok_after_abort"] = idle_ok

    manual = dialog.linuxcnc_reader.set_manual_mode() if idle_ok else None
    if manual is not None:
        fields["executor_manual_restore_ok"] = bool(manual.success)
        fields["executor_manual_restore_message"] = str(manual.message)
        fields["executor_end_mode"] = str(getattr(manual.status, "task_mode", "Unknown")) if manual.status is not None else "Unknown"
    else:
        fields["executor_manual_restore_message"] = "not attempted because interpreter was not IDLE"
        fields["executor_end_mode"] = "Unknown"

    ok = bool(position_ok and idle_ok and manual is not None and manual.success)
    fields["executor_result"] = "ok" if ok else "failed"
    dialog._timeline_log(
        "COORD_XY_DONE" if ok else "COORD_XY_FAILED",
        step_id=step_id,
        step_label=step_label,
        result="ok" if ok else "failed",
        reason="coordinated X/Y move complete and MANUAL restored" if ok else "coordinated X/Y move did not complete cleanly",
        duration_ms=f"{(time.monotonic() - started) * 1000.0:.3f}",
        **fields,
    )
    if not ok:
        if not position_ok:
            dialog.cal_status_var.set("M6 coordinated X/Y move did not reach the requested endpoint.")
        elif not idle_ok:
            dialog.cal_status_var.set("M6 coordinated X/Y move did not return LinuxCNC to IDLE.")
        else:
            dialog.cal_status_var.set(str(manual.message))
    return ok, fields


def _m6_step_impl(dialog: Any, *, session: RealBeliefSession, step_label: str, show_dialogs: bool, dry_run: bool = False) -> bool:
    step_start = time.monotonic()
    dialog._timeline_step_counter += 1
    step_id = int(dialog._timeline_step_counter)
    dialog._active_follow_run_step += 1
    dialog._timeline_log(
        "STEP_START",
        step_id=step_id,
        step_label=step_label,
        result="begin",
        planner_version=INTEGRATION_VERSION,
        m6_real_belief=True,
        dry_run=bool(dry_run),
        start_direction=dialog._get_follow_direction_preference(),
        hard_max_move=f"{_float(dialog.m6_real_max_move_var.get(), 0.035):.6f}",
        min_planner_confidence=f"{_float(dialog.m6_real_min_conf_var.get(), 45.0):.3f}",
        min_lookahead=f"{_float(dialog.m6_real_min_lookahead_var.get(), 0.045):.6f}",
    )

    if dialog._follow_stop_requested:
        dialog._timeline_log("STEP_STOPPED", step_id=step_id, step_label=step_label, result="stopped", reason="STOP requested")
        return False

    calibration = dialog._validate_calibration(getattr(dialog, "active_calibration", None))
    if calibration is None:
        dialog._timeline_log("STEP_REFUSED", step_id=step_id, step_label=step_label, result="refused", reason="calibration became invalid")
        return False

    dialog._freshen_follow_frame_if_stale(step_id=step_id, step_label=step_label)
    status = dialog.linuxcnc_reader.read_status()
    dialog._record_position_history(status, source="m6_step_status")
    if not dialog._status_ok_for_calibration(status):
        reason = status.error or dialog._status_not_ready_message(status)
        dialog._timeline_log("STEP_REFUSED", step_id=step_id, step_label=step_label, result="refused", reason=reason)
        if show_dialogs:
            messagebox.showerror("LinuxCNC not ready", reason, parent=dialog)
        dialog.cal_status_var.set(reason)
        return False
    current_x, current_y, _z = dialog._active_position(status)

    # Keep the line stabilizer oriented from the persistent M4 belief rather than
    # the legacy corner/progress state.
    if session.estimator.states:
        previous = session.estimator.states[-1]
        if previous.measurement_valid:
            dialog._follow_heading_unit = (float(previous.tangent_x), float(previous.tangent_y))

    line, detection_ms = dialog._detect_line_for_follow_with_retries(
        step_id=step_id,
        step_label=step_label,
        phase="pre_move",
        base_event="DETECTION_M6",
    )
    if not line.found:
        reason = f"M6 HOLD: {line.message} after fresh-frame retry"
        dialog.current_line = line
        dialog._timeline_log("STEP_REFUSED", step_id=step_id, step_label=step_label, result="hold", reason=reason)
        dialog.cal_status_var.set(reason)
        dialog._show_current_frame()
        return False

    transformed = dialog.get_transformed_frame_bgr(dialog.current_frame_bgr)
    frame_h, frame_w = transformed.shape[:2]
    line = dialog._stabilize_follow_line(line, frame_w=frame_w, frame_h=frame_h)
    dialog.current_line = line

    min_detector_conf = float(dialog._get_follow_min_confidence())
    if float(line.confidence) < min_detector_conf:
        reason = f"M6 HOLD: detector confidence {line.confidence:.1f}% below {min_detector_conf:.1f}%"
        dialog._timeline_log("STEP_REFUSED", step_id=step_id, step_label=step_label, result="hold", reason=reason, **dialog._timeline_line_fields(line))
        dialog.cal_status_var.set(reason)
        dialog._show_current_frame()
        return False

    frame_x, frame_y, delay_fields = _frame_pose(dialog, float(current_x), float(current_y))
    belief = _build_belief(
        dialog,
        session,
        step_id=step_id,
        step_label=step_label,
        line=line,
        transformed_frame=transformed,
        current_x=float(current_x),
        current_y=float(current_y),
        frame_x=float(frame_x),
        frame_y=float(frame_y),
    )
    _log_belief(dialog, step_id=step_id, step_label=step_label, belief=belief, delay_fields=delay_fields)

    plan = session.planner.plan(
        transformed,
        camera_x=float(current_x),
        camera_y=float(current_y),
        belief=belief,
        frame_camera_x=float(frame_x),
        frame_camera_y=float(frame_y),
    )
    if not plan.accepted:
        reason = f"M6 HOLD: {plan.decision_reason}"
        _set_live_overlay_state(
            dialog, step_label=step_label, current_x=float(current_x), current_y=float(current_y),
            belief=belief, plan=plan, status="HOLD", reason=reason,
        )
        dialog._timeline_log("STEP_REFUSED", step_id=step_id, step_label=step_label, result="hold", reason=reason, planner_confidence=f"{plan.planner_confidence:.3f}")
        dialog.cal_status_var.set(reason)
        dialog._show_current_frame()
        return False

    min_plan_conf = max(0.0, min(100.0, _float(dialog.m6_real_min_conf_var.get(), 45.0)))
    min_lookahead = max(0.0, min(0.50, _float(dialog.m6_real_min_lookahead_var.get(), 0.045)))
    if plan.planner_confidence < min_plan_conf:
        reason = f"M6 HOLD: planner confidence {plan.planner_confidence:.1f}% below {min_plan_conf:.1f}%"
        _set_live_overlay_state(dialog, step_label=step_label, current_x=float(current_x), current_y=float(current_y), belief=belief, plan=plan, status="HOLD", reason=reason)
        dialog._timeline_log("STEP_REFUSED", step_id=step_id, step_label=step_label, result="hold", reason=reason)
        dialog.cal_status_var.set(reason)
        dialog._show_current_frame()
        return False
    if plan.usable_lookahead < min_lookahead:
        reason = f"M6 HOLD: usable look-ahead {plan.usable_lookahead:.4f} in below {min_lookahead:.4f} in"
        _set_live_overlay_state(dialog, step_label=step_label, current_x=float(current_x), current_y=float(current_y), belief=belief, plan=plan, status="HOLD", reason=reason)
        dialog._timeline_log("STEP_REFUSED", step_id=step_id, step_label=step_label, result="hold", reason=reason)
        dialog.cal_status_var.set(reason)
        dialog._show_current_frame()
        return False

    move_x, move_y, controller_fields = _controller_move_from_plan(
        dialog, plan, float(current_x), float(current_y)
    )
    physical_safety = "PASS"

    # Second independent physical envelope outside the planner. This is purposely
    # stricter than the simulator's internal max and defaults to 0.035 in.
    hard_max = max(0.005, min(0.100, _float(dialog.m6_real_max_move_var.get(), 0.035)))
    move_len = math.hypot(move_x, move_y)
    if move_len > hard_max and move_len > 1e-12:
        scale = hard_max / move_len
        move_x *= scale
        move_y *= scale
        move_len = hard_max
        physical_safety = f"CLAMP to M6 hard max {hard_max:.3f} in"

    # Independent physical reversal check uses the planner's *forward path
    # progress*, not the total camera move.  The total move contains a bounded
    # lateral correction and may legitimately swing from one side of the profile
    # to the other.
    progress_unit = _unit(
        _float(controller_fields.get("controller_forward_x"), 0.0),
        _float(controller_fields.get("controller_forward_y"), 0.0),
    )
    established = session.planner.travel_unit
    if progress_unit is not None and established is not None:
        forward_dot = progress_unit[0] * float(established[0]) + progress_unit[1] * float(established[1])
        if forward_dot < -0.20:
            reason = f"M6 PHYSICAL SAFETY HOLD: connected path progress reverses established travel (dot {forward_dot:+.2f})"
            _set_live_overlay_state(
                dialog, step_label=step_label, current_x=float(current_x), current_y=float(current_y),
                belief=belief, plan=plan, move_x=move_x, move_y=move_y,
                controller_fields=controller_fields, status="HOLD", reason=reason,
            )
            dialog._timeline_log(
                "STEP_REFUSED", step_id=step_id, step_label=step_label, result="hold", reason=reason,
                **controller_fields,
            )
            dialog.cal_status_var.set(reason)
            dialog._show_current_frame()
            return False

    if move_len < 0.003:
        reason = f"M6 PHYSICAL SAFETY HOLD: approved move collapsed to {move_len:.4f} in"
        _set_live_overlay_state(
            dialog, step_label=step_label, current_x=float(current_x), current_y=float(current_y),
            belief=belief, plan=plan, move_x=move_x, move_y=move_y,
            controller_fields=controller_fields, status="HOLD", reason=reason,
        )
        dialog._timeline_log("STEP_REFUSED", step_id=step_id, step_label=step_label, result="hold", reason=reason)
        dialog.cal_status_var.set(reason)
        dialog._show_current_frame()
        return False

    target_x = float(current_x) + move_x
    target_y = float(current_y) + move_y
    payload = _plan_payload(
        plan,
        current_x=float(current_x), current_y=float(current_y),
        move_x=move_x, move_y=move_y,
        target_x=target_x, target_y=target_y,
        physical_safety=physical_safety,
    )
    _set_live_overlay_state(
        dialog, step_label=step_label, current_x=float(current_x), current_y=float(current_y),
        belief=belief, plan=plan, move_x=move_x, move_y=move_y,
        controller_fields=controller_fields, status="PLAN", reason=physical_safety,
    )
    command_heading = _heading_degrees(move_x, move_y)
    command_heading_change = _heading_change_degrees(
        session.last_command_x, session.last_command_y, move_x, move_y
    )
    payload.update({
        "command_heading_degrees": f"{command_heading:.3f}" if command_heading is not None else "",
        "command_heading_change_degrees": (
            f"{command_heading_change:.3f}" if command_heading_change is not None else ""
        ),
        "previous_command_x": f"{session.last_command_x:.6f}",
        "previous_command_y": f"{session.last_command_y:.6f}",
    })
    payload.update(delay_fields)
    payload.update(controller_fields)
    payload.update(dialog._timeline_line_fields(line))
    dialog._timeline_log("MOVE_PLAN", step_id=step_id, step_label=step_label, **payload)

    status_text = (
        f"{step_label}: M6 plan X{move_x:+.4f} Y{move_y:+.4f} ({move_len:.4f} in), "
        f"look-ahead {plan.usable_lookahead:.3f} in, plan conf {plan.planner_confidence:.0f}%, "
        f"recommended {plan.recommended_velocity:.0f} IPM [logged only], "
        f"side correction {controller_fields.get('controller_correct_len', '0.000000')} in, safety {physical_safety}."
    )
    dialog.cal_status_var.set(status_text)
    dialog.update()

    if dry_run:
        dialog._timeline_log("STEP_COMPLETE", step_id=step_id, step_label=step_label, result="dry_plan", reason=status_text)
        dialog._show_current_frame()
        return True

    feed = float(dialog._get_follow_feed())
    coordinate_mode = dialog.coordinate_mode_label
    dialog._manual_jog_active = True
    try:
        send_start = time.monotonic()
        ok, executor_fields = _execute_coordinated_xy_step(
            dialog,
            step_id=step_id,
            step_label=step_label,
            move_x=move_x,
            move_y=move_y,
            target_x=target_x,
            target_y=target_y,
            feed=feed,
            coordinate_mode=coordinate_mode,
        )
        if not ok:
            dialog._timeline_log(
                "MOVE_FAILED", step_id=step_id, step_label=step_label,
                result="failed", reason=dialog.cal_status_var.get(), **executor_fields,
            )
            return False
        session.last_command_x = float(move_x)
        session.last_command_y = float(move_y)
        dialog._timeline_log(
            "MOVE_SENT", step_id=step_id, step_label=step_label, result="ok",
            duration_ms=f"{(time.monotonic()-send_start)*1000.0:.3f}",
            move_x=f"{move_x:.6f}", move_y=f"{move_y:.6f}", move_len=f"{move_len:.6f}",
            command_heading_degrees=f"{command_heading:.3f}" if command_heading is not None else "",
            command_heading_change_degrees=(
                f"{command_heading_change:.3f}" if command_heading_change is not None else ""
            ),
            requested_feed_ipm=f"{feed:.3f}", recommended_velocity_ipm=f"{plan.recommended_velocity:.3f}",
            **executor_fields,
        )

        settle_ms = int(dialog._get_follow_settle_ms())
        settle_start = time.monotonic()
        dialog._wait_and_pump_camera(settle_ms / 1000.0)
        dialog._timeline_log("SETTLE_DONE", step_id=step_id, step_label=step_label, result="ok", duration_ms=f"{(time.monotonic()-settle_start)*1000.0:.3f}")

        post_status = dialog.linuxcnc_reader.read_status()
        dialog._record_position_history(post_status, source="m6_post_move")
        if not post_status.connected:
            reason = post_status.error or "LinuxCNC disconnected after M6 move"
            dialog._timeline_log("STEP_FAILED", step_id=step_id, step_label=step_label, result="failed", reason=reason)
            dialog.cal_status_var.set(reason)
            return False
        post_x, post_y, _post_z = dialog._active_position(post_status)
        actual_dx = float(post_x) - float(current_x)
        actual_dy = float(post_y) - float(current_y)
        actual_heading = _heading_degrees(actual_dx, actual_dy)
        actual_heading_change = _heading_change_degrees(
            session.last_move_x, session.last_move_y, actual_dx, actual_dy
        )
        session.last_move_x = actual_dx
        session.last_move_y = actual_dy
        session.last_actual_x = float(post_x)
        session.last_actual_y = float(post_y)
        session.planner.record_executed_pose(float(post_x), float(post_y))
        if isinstance(getattr(dialog, "_m6_live_overlay_state", None), dict):
            dialog._m6_live_overlay_state["camera_x"] = float(post_x)
            dialog._m6_live_overlay_state["camera_y"] = float(post_y)
            dialog._m6_live_overlay_state["status"] = "COMPLETE"

        if bool(dialog.follow_capture_point_var.get()) and dialog.trace_capture_callback is not None:
            dialog.trace_capture_callback()

        target_error = math.hypot(float(post_x) - target_x, float(post_y) - target_y)
        dialog._timeline_log(
            "POST_DETECTION_FINAL",
            step_id=step_id, step_label=step_label, result="position_only",
            post_x=f"{float(post_x):.6f}", post_y=f"{float(post_y):.6f}",
            target_x=f"{target_x:.6f}", target_y=f"{target_y:.6f}",
            target_error=f"{target_error:.6f}",
        )
        dialog._timeline_log(
            "STEP_COMPLETE", step_id=step_id, step_label=step_label, result="ok",
            duration_ms=f"{(time.monotonic()-step_start)*1000.0:.3f}",
            move_x=f"{actual_dx:.6f}", move_y=f"{actual_dy:.6f}",
            move_len=f"{math.hypot(actual_dx, actual_dy):.6f}",
            actual_heading_degrees=f"{actual_heading:.3f}" if actual_heading is not None else "",
            actual_heading_change_degrees=(
                f"{actual_heading_change:.3f}" if actual_heading_change is not None else ""
            ),
            executor_mode="coordinated_xy_mdi_g1",
            target_x=f"{target_x:.6f}", target_y=f"{target_y:.6f}",
            post_x=f"{float(post_x):.6f}", post_y=f"{float(post_y):.6f}",
        )
        dialog.cal_status_var.set(
            f"{step_label} complete: actual X{actual_dx:+.4f} Y{actual_dy:+.4f}; "
            f"plan conf {plan.planner_confidence:.0f}%, look-ahead {plan.usable_lookahead:.3f} in."
        )
        dialog._show_current_frame()
        return True
    finally:
        dialog._manual_jog_active = False


def _begin_physical_run(dialog: Any, *, label: str, requested_steps: int) -> Optional[RealBeliefSession]:
    calibration = _m6_preflight(dialog, require_arm=True, show_dialogs=True)
    if calibration is None:
        return None
    try:
        dialog._reset_follow_filter()
        dialog._clear_follow_heading()
    except Exception:
        pass
    session = _new_session(dialog, calibration)
    dialog._m6_session = session
    dialog._follow_stop_requested = False
    dialog._begin_follow_run(label, requested_steps=requested_steps, reset_latch=True)
    dialog._timeline_log(
        "M6_RUN_CONFIG",
        result="begin",
        planner_version=INTEGRATION_VERSION,
        start_direction=dialog._get_follow_direction_preference(),
        hard_max_move=f"{_float(dialog.m6_real_max_move_var.get(), 0.035):.6f}",
        min_planner_confidence=f"{_float(dialog.m6_real_min_conf_var.get(), 45.0):.3f}",
        min_lookahead=f"{_float(dialog.m6_real_min_lookahead_var.get(), 0.045):.6f}",
        real_feed_ipm=f"{dialog._get_follow_feed():.3f}",
        recommended_velocity_used_for_motion=False,
        corner_assist_bypassed=True,
        legacy_virtual_target_bypassed=True,
        recorder_forced_on=True,
        physical_executor="coordinated_xy_mdi_g1",
        returns_to_manual_between_steps=True,
        step_and_settle_mode=True,
    )
    return session


def m6_dry_plan(dialog: Any) -> None:
    calibration = _m6_preflight(dialog, require_arm=False, show_dialogs=True)
    if calibration is None:
        return
    try:
        dialog._reset_follow_filter()
        dialog._clear_follow_heading()
    except Exception:
        pass
    session = _new_session(dialog, calibration)
    # Dry plan is intentionally not started as a Follow run and does not move.
    dialog._follow_stop_requested = False
    ok = _m6_step_impl(dialog, session=session, step_label="M6 Dry Plan", show_dialogs=True, dry_run=True)
    if ok:
        dialog.cal_status_var.set(dialog.cal_status_var.get() + " NO MOTION SENT.")


def m6_follow_single_step(dialog: Any) -> None:
    session = _begin_physical_run(dialog, label="M6 Belief Step", requested_steps=1)
    if session is None:
        return
    status = "incomplete"
    reason = "M6 Belief Step did not complete"
    try:
        ok = _m6_step_impl(dialog, session=session, step_label="M6 Belief Step", show_dialogs=True, dry_run=False)
        status = "complete" if ok else "incomplete"
        reason = dialog.cal_status_var.get()
        dialog._timeline_log("RUN_END", result=status, reason=reason, planner_version=INTEGRATION_VERSION)
    except Exception as exc:
        status = "exception"
        reason = repr(exc)
        dialog.cal_status_var.set(f"M6 exception: {exc}")
        dialog._timeline_log("RUN_END", result="exception", reason=reason, planner_version=INTEGRATION_VERSION)
        traceback.print_exc()
        messagebox.showerror("M6 Belief Step", f"{type(exc).__name__}: {exc}", parent=dialog)
    finally:
        _finish_recording(dialog, status=status, reason=reason)


def m6_follow_multiple_steps(dialog: Any) -> None:
    count = int(dialog._get_follow_repeat_count())
    if count <= 1:
        m6_follow_single_step(dialog)
        return
    session = _begin_physical_run(dialog, label="M6 Belief N", requested_steps=count)
    if session is None:
        return
    completed = 0
    status = "incomplete"
    reason = ""
    try:
        dialog.cal_status_var.set(f"M6 Belief N starting: {count} requested bounded steps.")
        dialog.update()
        for index in range(1, count + 1):
            if dialog._follow_stop_requested:
                break
            ok = _m6_step_impl(
                dialog,
                session=session,
                step_label=f"M6 {index}/{count}",
                show_dialogs=False,
                dry_run=False,
            )
            if not ok:
                break
            completed += 1
        if dialog._follow_stop_requested:
            status = "stopped"
            reason = f"M6 Belief N stopped by user after {completed}/{count} completed steps."
        elif completed >= count:
            status = "complete"
            reason = f"M6 Belief N complete: {completed}/{count} steps completed."
        else:
            status = "incomplete"
            reason = f"M6 Belief N HOLD after {completed}/{count} completed steps. {dialog.cal_status_var.get()}"
        dialog.cal_status_var.set(reason)
        dialog._timeline_log("RUN_END", result=status, reason=reason, planner_version=INTEGRATION_VERSION)
    except Exception as exc:
        status = "exception"
        reason = repr(exc)
        dialog.cal_status_var.set(f"M6 exception after {completed} steps: {exc}")
        dialog._timeline_log("RUN_END", result="exception", reason=reason, planner_version=INTEGRATION_VERSION)
        traceback.print_exc()
        messagebox.showerror("M6 Belief N", f"{type(exc).__name__}: {exc}", parent=dialog)
    finally:
        _finish_recording(dialog, status=status, reason=reason)
        dialog._show_current_frame()


def install_real_belief_follow(dialog_cls: type) -> None:
    if getattr(dialog_cls, "_fabscan_m6_real_belief_installed", False):
        return
    required = (
        "__init__", "_begin_follow_run", "_timeline_log", "_wait_for_position_near", "_wait_for_idle",
        "_detect_line_for_follow_with_retries", "_validate_calibration",
    )
    missing = [name for name in required if not hasattr(dialog_cls, name)]
    if missing:
        raise RuntimeError(f"M6 cannot attach; missing CameraCalibrationDialog methods: {', '.join(missing)}")

    original_init = dialog_cls.__init__

    def wrapped_init(self: Any, *args: Any, **kwargs: Any) -> None:
        original_init(self, *args, **kwargs)
        self._m6_session = None
        self._m6_live_overlay_state = None
        _add_ui(self)
        try:
            current = str(self.title())
            if "M6 Real Belief" not in current:
                self.title(f"{current} + M6 Real Belief")
        except Exception:
            pass

    dialog_cls.__init__ = wrapped_init
    dialog_cls.m6_open_svg_simulator = m6_open_svg_simulator
    dialog_cls.m6_dry_plan = m6_dry_plan
    dialog_cls.m6_follow_single_step = m6_follow_single_step
    dialog_cls.m6_follow_multiple_steps = m6_follow_multiple_steps
    dialog_cls._m6_draw_live_overlay = _m6_draw_live_overlay
    dialog_cls._fabscan_m6_real_belief_installed = True
