from __future__ import annotations

from dataclasses import dataclass, replace
from datetime import datetime
import csv
import math
from pathlib import Path
import time
from typing import Any, Callable, Optional

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageTk
import tkinter as tk
from tkinter import messagebox, ttk

from fabscan.camera_device import (
    DEFAULT_CAMERA_HEIGHT,
    DEFAULT_CAMERA_PREVIEW_MAX_FPS,
    DEFAULT_CAMERA_STREAM_MAX_FPS,
    DEFAULT_CAMERA_WIDTH,
    CameraStream,
    open_camera_capture,
    parse_preset_label,
    preset_labels,
    size_to_preset_label,
)
from fabscan.linuxcnc_status import LinuxCNCPositionStatus, LinuxCNCStatusReader


ROTATE_VALUES = (0, 90, 180, 270)


@dataclass
class DotDetection:
    found: bool
    x: float = 0.0
    y: float = 0.0
    area: float = 0.0
    confidence: float = 0.0
    message: str = "No dot found"


@dataclass
class LineDetection:
    found: bool
    mode: str = "Line center"
    x: float = 0.0
    y: float = 0.0
    vx: float = 1.0
    vy: float = 0.0
    pixel_error_x: float = 0.0
    pixel_error_y: float = 0.0
    angle_degrees: float = 0.0
    confidence: float = 0.0
    span_px: float = 0.0
    width_px: float = 0.0
    point_count: int = 0
    search_px: int = 0
    stabilized: bool = False
    filtered_pixel_error_x: float = 0.0
    filtered_pixel_error_y: float = 0.0
    filtered_angle_degrees: float = 0.0
    continuity_message: str = ""
    corner_candidate: bool = False
    corner_angle_degrees: float = 0.0
    corner_strength: float = 0.0
    corner_message: str = ""
    corner_primary_vx: float = 0.0
    corner_primary_vy: float = 0.0
    corner_secondary_vx: float = 0.0
    corner_secondary_vy: float = 0.0
    corner_primary_x: float = 0.0
    corner_primary_y: float = 0.0
    corner_secondary_x: float = 0.0
    corner_secondary_y: float = 0.0
    corner_intersection_x: float = 0.0
    corner_intersection_y: float = 0.0
    corner_intersection_valid: bool = False
    message: str = "No line/edge found"


@dataclass
class CornerMetrics:
    candidate: bool = False
    angle_degrees: float = 0.0
    strength: float = 0.0
    message: str = ""
    primary_x: float = 0.0
    primary_y: float = 0.0
    primary_vx: float = 0.0
    primary_vy: float = 0.0
    secondary_x: float = 0.0
    secondary_y: float = 0.0
    secondary_vx: float = 0.0
    secondary_vy: float = 0.0
    intersection_x: float = 0.0
    intersection_y: float = 0.0
    intersection_valid: bool = False


@dataclass
class CornerAssistMove:
    ok: bool
    move_x: float = 0.0
    move_y: float = 0.0
    heading: Optional[tuple[float, float]] = None
    heading_state: str = ""
    message: str = ""
    distance: float = 0.0
    max_distance: float = 0.0


@dataclass
class PositionHistorySample:
    timestamp_s: float
    x: float
    y: float
    z: float
    source: str = ""


@dataclass
class CameraCalibrationDialogResult:
    camera_index: int
    requested_width: int
    requested_height: int
    camera_stream_max_fps: float
    camera_preview_max_fps: float
    linuxcnc_safe_preview: bool
    profile_preview: bool
    rotate_degrees: int
    flip_x: bool
    flip_y: bool
    fine_rotation_degrees: float
    threshold: int
    show_dot_marker: bool
    show_mask: bool
    move_distance: float
    feed_units_per_min: float
    jog_step: float
    center_max_move: float
    line_mode: str
    line_search_px: int
    show_line_preview: bool
    follow_step: float
    follow_feed_units_per_min: float
    follow_settle_ms: int
    follow_max_heading_change_degrees: float
    follow_corner_pause_enabled: bool
    follow_corner_angle_degrees: float
    follow_corner_assist_enabled: bool
    follow_corner_lookahead_steps: int
    follow_max_correct: float
    follow_deadband: float
    follow_gain: float
    follow_stabilize_enabled: bool
    follow_filter_offset_alpha: float
    follow_filter_angle_alpha: float
    follow_sanity_angle_degrees: float
    follow_min_confidence: float
    follow_direction: str
    follow_capture_point: bool
    follow_enabled: bool
    follow_repeat_count: int
    follow_timeline_log_enabled: bool
    follow_use_delayed_position: bool
    follow_position_delay_ms: int
    follow_virtual_target_enabled: bool
    follow_virtual_min_progress_pct: float
    calibration: Optional[dict[str, Any]] = None


class CameraCalibrationDialog(tk.Toplevel):
    """Camera/machine calibration helper for FabScan.

    This dialog handles the Scanything-style camera/machine calibration and
    bounded camera-assisted line/edge following. The final DXF is still based
    on LinuxCNC position. The camera is only used as the steering eye.
    """

    def __init__(
        self,
        parent: tk.Misc,
        *,
        linuxcnc_reader: LinuxCNCStatusReader,
        coordinate_mode_label: str,
        camera_index: int = 0,
        camera_width: int = DEFAULT_CAMERA_WIDTH,
        camera_height: int = DEFAULT_CAMERA_HEIGHT,
        camera_stream_max_fps: float = DEFAULT_CAMERA_STREAM_MAX_FPS,
        camera_preview_max_fps: float = DEFAULT_CAMERA_PREVIEW_MAX_FPS,
        linuxcnc_safe_preview: bool = False,
        profile_preview: bool = False,
        rotate_degrees: int = 0,
        flip_x: bool = False,
        flip_y: bool = False,
        fine_rotation_degrees: float = 0.0,
        threshold: int = 90,
        show_dot_marker: bool = True,
        move_distance: float = 0.100,
        feed_per_minute: float = 5.0,
        jog_step: float = 0.010,
        center_max_move: float = 0.100,
        line_mode: str = "Line center",
        line_search_px: int = 220,
        show_line_preview: bool = True,
        show_mask: bool = False,
        follow_step: float = 0.050,
        follow_feed_units_per_min: float = 5.0,
        follow_settle_ms: int = 150,
        follow_max_heading_change_degrees: float = 70.0,
        follow_corner_pause_enabled: bool = True,
        follow_corner_angle_degrees: float = 55.0,
        follow_corner_assist_enabled: bool = True,
        follow_corner_lookahead_steps: int = 5,
        follow_max_correct: float = 0.050,
        follow_deadband: float = 0.003,
        follow_gain: float = 0.50,
        follow_stabilize_enabled: bool = True,
        follow_filter_offset_alpha: float = 0.35,
        follow_filter_angle_alpha: float = 0.25,
        follow_sanity_angle_degrees: float = 30.0,
        follow_min_confidence: float = 45.0,
        follow_direction: str = "Y+",
        follow_capture_point: bool = False,
        follow_enabled: bool = False,
        follow_repeat_count: int = 5,
        follow_timeline_log_enabled: bool = False,
        follow_use_delayed_position: bool = False,
        follow_position_delay_ms: int = 120,
        follow_virtual_target_enabled: bool = False,
        follow_virtual_min_progress_pct: float = 70.0,
        existing_calibration: Optional[dict[str, Any]] = None,
        trace_capture_callback: Optional[Callable[[], None]] = None,
    ) -> None:
        super().__init__(parent)
        self.title("FabScan Camera Calibration Lite - v0.5.3.3")
        self.minsize(1080, 650)
        # Give the dialog an explicit starting size so Tk does not keep
        # recomputing the top-level size as live preview/status content changes.
        self.geometry("1240x760")
        self.transient(parent)

        self.linuxcnc_reader = linuxcnc_reader
        self.coordinate_mode_label = coordinate_mode_label or "Work coordinates"
        self.result: Optional[CameraCalibrationDialogResult] = None
        self.cap: Optional[CameraStream] = None
        self.current_frame_bgr: Optional[np.ndarray] = None
        self.current_frame_sequence: int = 0
        self.current_frame_timestamp: float = 0.0
        self.current_dot: DotDetection = DotDetection(False)
        self.current_line: LineDetection = LineDetection(False)
        # Fixed live-preview box. Without this, the PhotoImage size can change
        # during line/edge preview and Tk resizes the whole calibration window.
        self.preview_display_width = 700
        self.preview_display_height = 420
        self.after_job: Optional[str] = None
        self._tk_preview: Optional[ImageTk.PhotoImage] = None
        self._closing = False
        self._motion_active = False
        self._manual_jog_active = False
        self._follow_stop_requested = False
        # Direction latch for line/edge following. A detected line has no arrow,
        # so the fitted tangent can flip 180 degrees from one frame to the next.
        # Once a follow direction is established, keep subsequent steps moving
        # along the same machine-space heading unless the user changes settings.
        self._follow_heading_unit: Optional[tuple[float, float]] = None
        # Last successful commanded move direction. This is a second continuity
        # guard for closed/curved shapes: even if a fitted line axis is valid,
        # the actual commanded travel direction should not suddenly reverse and
        # make the machine dither back and forth on the same feature.
        self._follow_last_move_unit: Optional[tuple[float, float]] = None
        # Lightweight follow stabilization state. This deliberately starts
        # simpler than optical flow/Kalman/PID: reject impossible heading jumps,
        # prefer contours near the previous feature, then EMA-filter the accepted
        # pixel offset and tangent. Lucas-Kanade/Kalman/P+D can sit on top of
        # this later once the basic detector is less jumpy.
        self._follow_filtered_err: Optional[tuple[float, float]] = None
        self._follow_filtered_vec_px: Optional[tuple[float, float]] = None
        # When corner pause stops at an intentional corner, the user can pick
        # the next Start dir and click Find Line/Edge. The next follow move is
        # allowed to bypass corner pause once so it can step out of the
        # ambiguous L-shaped ROI instead of immediately pausing again.
        self._follow_corner_resume_once = False
        self._last_camera_sequence = 0
        self._last_preview_sequence = 0
        self._next_preview_due = 0.0
        self._preview_count = 0
        self._timeline_log_path: Optional[Path] = None
        self._timeline_log_file: Optional[Any] = None
        self._timeline_csv: Optional[csv.DictWriter] = None
        self._timeline_step_counter = 0
        self._follow_run_counter = 0
        self._active_follow_run_id = 0
        self._active_follow_run_step = 0
        self._position_history: list[PositionHistorySample] = []
        self._position_history_window_s = 8.0
        # Stage 2D experimental virtual-target state. This remains a bounded
        # position-step shim, not a velocity-jog/continuous controller. The
        # virtual target is reset at each new Follow run and kept inside the
        # normal Follow Step + Max correct limits.
        self._follow_virtual_target_xy: Optional[tuple[float, float]] = None
        # v0.5.3.3 transient-not-found recovery. With Capture FPS much higher
        # than Preview FPS, one bad/undrawn frame can otherwise stop Follow N even
        # when the operator still sees the last good overlay on screen. Keep the
        # retry count small and wait only for fresh frames so this stays safe and
        # bounded.
        self._follow_not_found_retry_count = 2
        self._follow_not_found_retry_timeout_s = 0.20
        self._last_follow_detection_sequence = 0
        self._last_follow_detection_result = ""
        self.trace_capture_callback = trace_capture_callback
        self.active_calibration: Optional[dict[str, Any]] = self._validate_calibration(existing_calibration)

        if int(rotate_degrees) not in ROTATE_VALUES:
            rotate_degrees = 0

        self.camera_index_var = tk.IntVar(value=max(0, int(camera_index)))
        self.camera_width_var = tk.IntVar(value=max(0, int(camera_width)))
        self.camera_height_var = tk.IntVar(value=max(0, int(camera_height)))
        self.camera_preset_var = tk.StringVar(value=size_to_preset_label(camera_width, camera_height))
        self.camera_stream_max_fps_var = tk.DoubleVar(value=self._clamp_camera_stream_max_fps(camera_stream_max_fps))
        self.camera_preview_max_fps_var = tk.DoubleVar(value=self._clamp_camera_preview_max_fps(camera_preview_max_fps))
        self.linuxcnc_safe_preview_var = tk.BooleanVar(value=bool(linuxcnc_safe_preview))
        self.profile_preview_var = tk.BooleanVar(value=bool(profile_preview))
        self.rotate_var = tk.StringVar(value=str(int(rotate_degrees)))
        self.flip_x_var = tk.BooleanVar(value=bool(flip_x))
        self.flip_y_var = tk.BooleanVar(value=bool(flip_y))
        self.fine_rotation_var = tk.DoubleVar(value=self._clamp_fine_rotation(fine_rotation_degrees))
        self.threshold_var = tk.IntVar(value=max(0, min(255, int(threshold))))
        self.show_dot_marker_var = tk.BooleanVar(value=bool(show_dot_marker))
        self.show_mask_var = tk.BooleanVar(value=bool(show_mask))
        self.move_distance_var = tk.DoubleVar(value=max(0.001, float(move_distance)))
        self.feed_var = tk.DoubleVar(value=max(0.1, float(feed_per_minute)))
        self.jog_step_var = tk.DoubleVar(value=max(0.001, float(jog_step)))
        self.center_max_move_var = tk.DoubleVar(value=max(0.001, float(center_max_move)))
        self.line_mode_var = tk.StringVar(value=self._normalize_line_mode(line_mode))
        self._last_valid_line_search_px = self._clamp_line_search_px(line_search_px)
        self.line_search_px_var = tk.StringVar(value=str(self._last_valid_line_search_px))
        self.show_line_preview_var = tk.BooleanVar(value=bool(show_line_preview))
        self.follow_step_var = tk.DoubleVar(value=max(0.001, float(follow_step)))
        self.follow_feed_var = tk.DoubleVar(value=max(0.1, float(follow_feed_units_per_min)))
        self.follow_settle_ms_var = tk.IntVar(value=self._clamp_follow_settle_ms(follow_settle_ms))
        self.follow_max_heading_change_var = tk.DoubleVar(value=self._clamp_follow_max_heading_change(follow_max_heading_change_degrees))
        self.follow_corner_pause_var = tk.BooleanVar(value=bool(follow_corner_pause_enabled))
        self.follow_corner_angle_var = tk.StringVar(value=f"{self._clamp_follow_corner_angle(follow_corner_angle_degrees):g}")
        self.follow_corner_assist_var = tk.BooleanVar(value=bool(follow_corner_assist_enabled))
        self.follow_corner_lookahead_steps_var = tk.IntVar(value=self._clamp_follow_corner_lookahead_steps(follow_corner_lookahead_steps))
        self.follow_max_correct_var = tk.DoubleVar(value=max(0.0, float(follow_max_correct)))
        self.follow_deadband_var = tk.DoubleVar(value=self._clamp_follow_deadband(follow_deadband))
        self.follow_gain_var = tk.DoubleVar(value=self._clamp_follow_gain(follow_gain))
        self.follow_stabilize_var = tk.BooleanVar(value=bool(follow_stabilize_enabled))
        self.follow_filter_offset_alpha_var = tk.DoubleVar(value=self._clamp_follow_filter_alpha(follow_filter_offset_alpha, default=0.35))
        self.follow_filter_angle_alpha_var = tk.DoubleVar(value=self._clamp_follow_filter_alpha(follow_filter_angle_alpha, default=0.25))
        self.follow_sanity_angle_var = tk.DoubleVar(value=self._clamp_follow_sanity_angle(follow_sanity_angle_degrees))
        self.follow_min_confidence_var = tk.DoubleVar(value=max(0.0, min(100.0, float(follow_min_confidence))))
        self.follow_direction_var = tk.StringVar(value=self._normalize_follow_direction(follow_direction))
        self.follow_capture_point_var = tk.BooleanVar(value=bool(follow_capture_point))
        self.follow_enabled_var = tk.BooleanVar(value=bool(follow_enabled))
        self.follow_repeat_count_var = tk.IntVar(value=max(1, min(9999, int(follow_repeat_count))))
        self.follow_timeline_log_var = tk.BooleanVar(value=bool(follow_timeline_log_enabled))
        self.follow_use_delayed_position_var = tk.BooleanVar(value=bool(follow_use_delayed_position))
        self.follow_position_delay_ms_var = tk.IntVar(value=self._clamp_follow_position_delay_ms(follow_position_delay_ms))
        self.follow_virtual_target_var = tk.BooleanVar(value=bool(follow_virtual_target_enabled))
        self.follow_virtual_min_progress_pct_var = tk.DoubleVar(value=self._clamp_follow_virtual_min_progress_pct(follow_virtual_min_progress_pct))
        self.dot_status_var = tk.StringVar(value="Dot: —")
        self.line_status_var = tk.StringVar(value="Line/edge: —")
        self.cal_status_var = tk.StringVar(value="Open camera, center the calibration dot, then click Find Dot.")
        self.transform_status_var = tk.StringVar(value="Calibration: not run")

        self._build_ui()
        if self.active_calibration:
            self._show_calibration_summary(self.active_calibration, loaded=True)
        self._register_traces()
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.bind("<Escape>", lambda _event: self.close())
        self.after(100, self.open_camera)

    def _build_ui(self) -> None:
        """Build a compact calibration/following layout.

        v0.5.6 proved the single-step following logic, but the calibration
        window used too much vertical space above the live preview. This layout
        keeps the camera/setup controls short, moves the live status beside the
        video, and puts the jog/follow controls next to the preview where they
        are easier to use while watching the camera.
        """

        footer = ttk.Label(
            self,
            text=(
                "Calibration, dot-centering, single-step follow, and screen jogs use guarded X/Y incremental jogs through LinuxCNC MANUAL mode. "
                "Torch/plasma should stay disabled. The camera steers; LinuxCNC remains the ruler."
            ),
            anchor=tk.W,
            padding=(8, 0, 8, 8),
        )
        footer.pack(side=tk.BOTTOM, fill=tk.X)

        root = ttk.Frame(self, padding=8)
        root.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        top = ttk.Frame(root)
        top.pack(side=tk.TOP, fill=tk.X)

        camera = ttk.LabelFrame(top, text="Camera / View", padding=6)
        camera.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 8))

        ttk.Label(camera, text="Index").grid(row=0, column=0, sticky=tk.W)
        ttk.Spinbox(camera, from_=0, to=10, textvariable=self.camera_index_var, width=4).grid(
            row=0, column=1, sticky=tk.W, padx=(4, 10)
        )
        ttk.Label(camera, text="W").grid(row=0, column=2, sticky=tk.W)
        ttk.Entry(camera, textvariable=self.camera_width_var, width=6).grid(row=0, column=3, sticky=tk.W, padx=(4, 10))
        ttk.Label(camera, text="H").grid(row=0, column=4, sticky=tk.W)
        ttk.Entry(camera, textvariable=self.camera_height_var, width=6).grid(row=0, column=5, sticky=tk.W, padx=(4, 10))
        ttk.Label(camera, text="Preset").grid(row=0, column=6, sticky=tk.W)
        preset_combo = ttk.Combobox(
            camera,
            textvariable=self.camera_preset_var,
            values=preset_labels(),
            width=11,
            state="readonly",
        )
        preset_combo.grid(row=0, column=7, sticky=tk.W, padx=(4, 10))
        preset_combo.bind("<<ComboboxSelected>>", lambda _event: self.apply_resolution_preset())
        ttk.Label(camera, text="Capture FPS").grid(row=0, column=8, sticky=tk.W)
        ttk.Entry(camera, textvariable=self.camera_stream_max_fps_var, width=5).grid(row=0, column=9, sticky=tk.W, padx=(4, 10))
        ttk.Label(camera, text="Preview FPS").grid(row=0, column=10, sticky=tk.W)
        ttk.Entry(camera, textvariable=self.camera_preview_max_fps_var, width=5).grid(row=0, column=11, sticky=tk.W, padx=(4, 10))
        ttk.Button(camera, text="Open / Restart", command=self.open_camera).grid(row=0, column=12, sticky=tk.W, padx=(0, 6))
        ttk.Button(camera, text="Close", command=self.close).grid(row=0, column=13, sticky=tk.W)

        ttk.Checkbutton(camera, text="LinuxCNC Safe", variable=self.linuxcnc_safe_preview_var, command=self.apply_linuxcnc_safe_preview).grid(row=1, column=8, columnspan=2, sticky=tk.W, pady=(6, 0))
        ttk.Checkbutton(camera, text="Profile", variable=self.profile_preview_var).grid(row=1, column=10, columnspan=2, sticky=tk.W, pady=(6, 0))

        ttk.Label(camera, text="Rotate").grid(row=1, column=0, sticky=tk.W, pady=(6, 0))
        ttk.Combobox(
            camera,
            textvariable=self.rotate_var,
            values=tuple(str(value) for value in ROTATE_VALUES),
            width=4,
            state="readonly",
        ).grid(row=1, column=1, sticky=tk.W, padx=(4, 10), pady=(6, 0))
        ttk.Checkbutton(camera, text="Flip X", variable=self.flip_x_var).grid(row=1, column=2, columnspan=2, sticky=tk.W, pady=(6, 0))
        ttk.Checkbutton(camera, text="Flip Y", variable=self.flip_y_var).grid(row=1, column=4, columnspan=2, sticky=tk.W, pady=(6, 0))
        ttk.Label(camera, text="Fine").grid(row=1, column=6, sticky=tk.E, pady=(6, 0))
        ttk.Scale(
            camera,
            from_=-10.0,
            to=10.0,
            variable=self.fine_rotation_var,
            command=lambda _value: self._show_current_frame(),
        ).grid(row=1, column=7, columnspan=2, sticky="ew", padx=(4, 8), pady=(6, 0))
        self.fine_rotation_label = ttk.Label(camera, width=7)
        self.fine_rotation_label.grid(row=1, column=9, sticky=tk.W, pady=(6, 0))
        camera.columnconfigure(7, weight=1)
        self._update_fine_rotation_label()

        vision = ttk.LabelFrame(top, text="Dot / Mask", padding=6)
        vision.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        ttk.Label(vision, text="Threshold").grid(row=0, column=0, sticky=tk.W)
        ttk.Scale(
            vision,
            from_=0,
            to=255,
            variable=self.threshold_var,
            command=lambda _value: self._show_current_frame(),
            length=110,
        ).grid(row=0, column=1, sticky="ew", padx=(4, 6))
        self.threshold_label = ttk.Label(vision, width=4)
        self.threshold_label.grid(row=0, column=2, sticky=tk.W)
        ttk.Checkbutton(vision, text="Mask", variable=self.show_mask_var, command=self._show_current_frame).grid(
            row=1, column=0, sticky=tk.W, pady=(4, 0)
        )
        ttk.Checkbutton(vision, text="Dot marker", variable=self.show_dot_marker_var, command=self._show_current_frame).grid(
            row=2, column=0, sticky=tk.W, pady=(4, 0)
        )
        ttk.Button(vision, text="Find Dot", command=self.find_dot_once).grid(row=1, column=1, columnspan=2, sticky="ew", pady=(4, 0))
        vision.columnconfigure(1, weight=1)

        motion = ttk.LabelFrame(top, text="Calibration", padding=6)
        motion.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Label(motion, text="Move").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(motion, textvariable=self.move_distance_var, width=7).grid(row=0, column=1, sticky=tk.W, padx=(4, 8))
        ttk.Label(motion, text="Feed").grid(row=0, column=2, sticky=tk.W)
        ttk.Entry(motion, textvariable=self.feed_var, width=6).grid(row=0, column=3, sticky=tk.W, padx=(4, 0))
        ttk.Button(motion, text="Run Calibration", command=self.run_calibration).grid(
            row=1, column=0, columnspan=3, sticky="ew", pady=(6, 0), padx=(0, 4)
        )
        ttk.Button(motion, text="STOP", command=self.stop_motion).grid(row=1, column=3, sticky="ew", pady=(6, 0))

        main = ttk.Frame(root)
        main.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=(8, 0))

        left_panel = ttk.Frame(main, width=285)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        left_panel.pack_propagate(False)

        status_frame = ttk.LabelFrame(left_panel, text="Status", padding=6, height=270)
        status_frame.pack(side=tk.TOP, fill=tk.X)
        # Keep the live status area fixed-height so long Line center updates do
        # not resize the dialog, but make it scrollable so no information is
        # clipped when a line wraps more than expected.
        status_frame.pack_propagate(False)
        status_scroll = ttk.Scrollbar(status_frame, orient=tk.VERTICAL)
        status_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.status_text = tk.Text(
            status_frame,
            height=14,
            width=34,
            wrap=tk.WORD,
            yscrollcommand=status_scroll.set,
            relief=tk.FLAT,
            borderwidth=0,
            highlightthickness=0,
            padx=2,
            pady=2,
        )
        self.status_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.status_text.configure(state=tk.DISABLED)
        status_scroll.configure(command=self.status_text.yview)
        self._refresh_status_text()

        line_tools = ttk.LabelFrame(left_panel, text="Line / Edge Preview", padding=6)
        line_tools.pack(side=tk.TOP, fill=tk.X, pady=(8, 0))
        ttk.Label(line_tools, text="Mode").grid(row=0, column=0, sticky=tk.W)
        ttk.Combobox(
            line_tools,
            textvariable=self.line_mode_var,
            values=("Line center", "Edge near center"),
            width=17,
            state="readonly",
        ).grid(row=0, column=1, columnspan=2, sticky="ew", padx=(4, 0))
        ttk.Label(line_tools, text="Search px").grid(row=1, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(line_tools, textvariable=self.line_search_px_var, width=7).grid(row=1, column=1, sticky=tk.W, padx=(4, 0), pady=(5, 0))
        ttk.Checkbutton(
            line_tools,
            text="Overlay",
            variable=self.show_line_preview_var,
            command=self._show_current_frame,
        ).grid(row=1, column=2, sticky=tk.W, pady=(5, 0))
        ttk.Button(line_tools, text="Find Line / Edge", command=self.find_line_once).grid(
            row=2, column=0, columnspan=3, sticky="ew", pady=(6, 0)
        )
        line_tools.columnconfigure(1, weight=1)

        preview_frame = ttk.LabelFrame(main, text="Live Preview", padding=4)
        preview_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.preview_box = ttk.Frame(
            preview_frame,
            width=self.preview_display_width,
            height=self.preview_display_height,
        )
        self.preview_box.pack(side=tk.TOP, expand=True)
        self.preview_box.pack_propagate(False)
        self.preview_label = ttk.Label(self.preview_box, anchor=tk.CENTER)
        self.preview_label.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        right_shell = ttk.Frame(main, width=300)
        right_shell.pack(side=tk.LEFT, fill=tk.Y, padx=(8, 0))
        right_shell.pack_propagate(False)

        # v0.5.17 added enough follow/corner controls that the old fixed-height
        # right panel could hide the Follow Step / Follow N / STOP controls on
        # shorter screens. Keep the right-side control stack fixed-width, but
        # make it vertically scrollable so all controls remain reachable.
        right_canvas = tk.Canvas(right_shell, highlightthickness=0, borderwidth=0)
        right_scroll = ttk.Scrollbar(right_shell, orient=tk.VERTICAL, command=right_canvas.yview)
        right_canvas.configure(yscrollcommand=right_scroll.set)
        right_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        right_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right_panel = ttk.Frame(right_canvas)
        right_window = right_canvas.create_window((0, 0), window=right_panel, anchor=tk.NW)

        def _update_right_scroll_region(_event: tk.Event) -> None:
            try:
                right_canvas.configure(scrollregion=right_canvas.bbox("all"))
            except tk.TclError:
                pass

        def _resize_right_panel(event: tk.Event) -> None:
            try:
                right_canvas.itemconfigure(right_window, width=event.width)
            except tk.TclError:
                pass

        def _on_right_panel_mousewheel(event: tk.Event) -> str:
            # Support common mouse-wheel events on Linux and Windows.
            try:
                if getattr(event, "num", None) == 4:
                    right_canvas.yview_scroll(-1, "units")
                elif getattr(event, "num", None) == 5:
                    right_canvas.yview_scroll(1, "units")
                elif event.delta:
                    right_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
            except tk.TclError:
                pass
            return "break"

        def _bind_right_panel_mousewheel(_event: tk.Event) -> None:
            right_canvas.bind_all("<MouseWheel>", _on_right_panel_mousewheel)
            right_canvas.bind_all("<Button-4>", _on_right_panel_mousewheel)
            right_canvas.bind_all("<Button-5>", _on_right_panel_mousewheel)

        def _unbind_right_panel_mousewheel(_event: tk.Event) -> None:
            right_canvas.unbind_all("<MouseWheel>")
            right_canvas.unbind_all("<Button-4>")
            right_canvas.unbind_all("<Button-5>")

        right_panel.bind("<Configure>", _update_right_scroll_region)
        right_canvas.bind("<Configure>", _resize_right_panel)
        right_canvas.bind("<Enter>", _bind_right_panel_mousewheel)
        right_canvas.bind("<Leave>", _unbind_right_panel_mousewheel)
        right_panel.bind("<Enter>", _bind_right_panel_mousewheel)
        right_panel.bind("<Leave>", _unbind_right_panel_mousewheel)

        jog = ttk.LabelFrame(right_panel, text="Dot Center Jog", padding=6)
        jog.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(jog, text="Step").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(jog, textvariable=self.jog_step_var, width=8).grid(row=0, column=1, columnspan=3, sticky="ew", padx=(4, 0))

        step_row = ttk.Frame(jog)
        step_row.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(4, 6))
        for label, value in ((".001", 0.001), (".005", 0.005), (".010", 0.010), (".050", 0.050), (".100", 0.100)):
            ttk.Button(step_row, text=label, width=5, command=lambda v=value: self.jog_step_var.set(v)).pack(
                side=tk.LEFT, padx=(0, 2)
            )

        ttk.Button(jog, text="Y+", command=lambda: self.manual_jog("Y", +1)).grid(row=2, column=1, columnspan=2, sticky="ew", pady=(0, 2))
        ttk.Button(jog, text="X-", command=lambda: self.manual_jog("X", -1)).grid(row=3, column=0, sticky="ew", padx=(0, 2))
        ttk.Button(jog, text="Find", command=self.find_dot_once).grid(row=3, column=1, columnspan=2, sticky="ew", padx=(0, 2))
        ttk.Button(jog, text="X+", command=lambda: self.manual_jog("X", +1)).grid(row=3, column=3, sticky="ew")
        ttk.Button(jog, text="Y-", command=lambda: self.manual_jog("Y", -1)).grid(row=4, column=1, columnspan=2, sticky="ew", pady=(2, 0))

        ttk.Separator(jog, orient=tk.HORIZONTAL).grid(row=5, column=0, columnspan=4, sticky="ew", pady=(8, 6))
        ttk.Label(jog, text="Max center").grid(row=6, column=0, columnspan=2, sticky=tk.W)
        ttk.Entry(jog, textvariable=self.center_max_move_var, width=8).grid(row=6, column=2, columnspan=2, sticky="ew")
        ttk.Button(jog, text="Center Dot", command=self.center_dot_using_calibration).grid(
            row=7, column=0, columnspan=4, sticky="ew", pady=(6, 0)
        )
        for col in range(4):
            jog.columnconfigure(col, weight=1)

        follow_tools = ttk.LabelFrame(right_panel, text="Single-Step Follow", padding=6)
        follow_tools.pack(side=tk.TOP, fill=tk.X, pady=(8, 0))
        ttk.Checkbutton(follow_tools, text="Enable follow", variable=self.follow_enabled_var).grid(row=0, column=0, columnspan=2, sticky=tk.W)
        ttk.Label(follow_tools, text="Step").grid(row=1, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(follow_tools, textvariable=self.follow_step_var, width=8).grid(row=1, column=1, sticky="ew", padx=(4, 0), pady=(5, 0))
        ttk.Label(follow_tools, text="Feed").grid(row=2, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(follow_tools, textvariable=self.follow_feed_var, width=8).grid(row=2, column=1, sticky="ew", padx=(4, 0), pady=(5, 0))
        ttk.Label(follow_tools, text="Settle ms").grid(row=3, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(follow_tools, textvariable=self.follow_settle_ms_var, width=8).grid(row=3, column=1, sticky="ew", padx=(4, 0), pady=(5, 0))
        ttk.Label(follow_tools, text="Max turn°").grid(row=4, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(follow_tools, textvariable=self.follow_max_heading_change_var, width=8).grid(row=4, column=1, sticky="ew", padx=(4, 0), pady=(5, 0))
        ttk.Checkbutton(
            follow_tools,
            text="Corner pause",
            variable=self.follow_corner_pause_var,
        ).grid(row=5, column=0, columnspan=2, sticky=tk.W, pady=(5, 0))
        ttk.Label(follow_tools, text="Corner°").grid(row=6, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(follow_tools, textvariable=self.follow_corner_angle_var, width=8).grid(row=6, column=1, sticky="ew", padx=(4, 0), pady=(5, 0))
        ttk.Checkbutton(
            follow_tools,
            text="Corner assist",
            variable=self.follow_corner_assist_var,
        ).grid(row=7, column=0, columnspan=2, sticky=tk.W, pady=(5, 0))
        ttk.Label(follow_tools, text="Lookahead").grid(row=8, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(follow_tools, textvariable=self.follow_corner_lookahead_steps_var, width=8).grid(
            row=8, column=1, sticky="ew", padx=(4, 0), pady=(5, 0)
        )
        ttk.Label(follow_tools, text="Max correct").grid(row=9, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(follow_tools, textvariable=self.follow_max_correct_var, width=8).grid(row=9, column=1, sticky="ew", padx=(4, 0), pady=(5, 0))
        ttk.Label(follow_tools, text="Deadband").grid(row=10, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(follow_tools, textvariable=self.follow_deadband_var, width=8).grid(row=10, column=1, sticky="ew", padx=(4, 0), pady=(5, 0))
        ttk.Label(follow_tools, text="Gain").grid(row=11, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(follow_tools, textvariable=self.follow_gain_var, width=8).grid(row=11, column=1, sticky="ew", padx=(4, 0), pady=(5, 0))
        ttk.Checkbutton(
            follow_tools,
            text="Stabilize",
            variable=self.follow_stabilize_var,
        ).grid(row=12, column=0, columnspan=2, sticky=tk.W, pady=(5, 0))
        ttk.Label(follow_tools, text="Err α").grid(row=13, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(follow_tools, textvariable=self.follow_filter_offset_alpha_var, width=8).grid(row=13, column=1, sticky="ew", padx=(4, 0), pady=(5, 0))
        ttk.Label(follow_tools, text="Ang α").grid(row=14, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(follow_tools, textvariable=self.follow_filter_angle_alpha_var, width=8).grid(row=14, column=1, sticky="ew", padx=(4, 0), pady=(5, 0))
        ttk.Label(follow_tools, text="Sanity°").grid(row=15, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(follow_tools, textvariable=self.follow_sanity_angle_var, width=8).grid(row=15, column=1, sticky="ew", padx=(4, 0), pady=(5, 0))
        ttk.Label(follow_tools, text="Min conf").grid(row=16, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(follow_tools, textvariable=self.follow_min_confidence_var, width=8).grid(row=16, column=1, sticky="ew", padx=(4, 0), pady=(5, 0))
        ttk.Label(follow_tools, text="Count").grid(row=17, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(follow_tools, textvariable=self.follow_repeat_count_var, width=8).grid(row=17, column=1, sticky="ew", padx=(4, 0), pady=(5, 0))
        ttk.Label(follow_tools, text="Start dir").grid(row=18, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Combobox(
            follow_tools,
            textvariable=self.follow_direction_var,
            values=("X+", "X-", "Y+", "Y-"),
            width=9,
            state="readonly",
        ).grid(row=18, column=1, sticky="ew", padx=(4, 0), pady=(5, 0))
        ttk.Checkbutton(
            follow_tools,
            text="Capture after move",
            variable=self.follow_capture_point_var,
        ).grid(row=19, column=0, columnspan=2, sticky=tk.W, pady=(5, 0))
        ttk.Checkbutton(
            follow_tools,
            text="Timeline log",
            variable=self.follow_timeline_log_var,
            command=self._on_timeline_log_toggle,
        ).grid(row=20, column=0, columnspan=2, sticky=tk.W, pady=(5, 0))
        ttk.Checkbutton(
            follow_tools,
            text="Use delayed pos",
            variable=self.follow_use_delayed_position_var,
        ).grid(row=21, column=0, columnspan=2, sticky=tk.W, pady=(5, 0))
        ttk.Label(follow_tools, text="Delay ms").grid(row=22, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(follow_tools, textvariable=self.follow_position_delay_ms_var, width=8).grid(
            row=22, column=1, sticky="ew", padx=(4, 0), pady=(5, 0)
        )
        ttk.Checkbutton(
            follow_tools,
            text="Virtual target",
            variable=self.follow_virtual_target_var,
        ).grid(row=23, column=0, columnspan=2, sticky=tk.W, pady=(5, 0))
        ttk.Label(follow_tools, text="Min prog %").grid(row=24, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(follow_tools, textvariable=self.follow_virtual_min_progress_pct_var, width=8).grid(
            row=24, column=1, sticky="ew", padx=(4, 0), pady=(5, 0)
        )
        ttk.Button(follow_tools, text="Follow Step", command=self.follow_line_single_step).grid(
            row=25, column=0, columnspan=2, sticky="ew", pady=(8, 0)
        )
        ttk.Button(follow_tools, text="Follow N", command=self.follow_line_multiple_steps).grid(
            row=26, column=0, columnspan=2, sticky="ew", pady=(4, 0)
        )
        ttk.Button(follow_tools, text="STOP Move", command=self.stop_motion).grid(row=27, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        follow_tools.columnconfigure(1, weight=1)

    def _on_timeline_log_toggle(self) -> None:
        """Open or close the Stage 2 timeline logger from the Follow panel."""

        if self._timeline_log_enabled():
            path = self._open_timeline_log()
            if path is not None:
                self.cal_status_var.set(f"Timeline log enabled: {path}")
        else:
            path = self._timeline_log_path
            self._close_timeline_log()
            if path is not None:
                self.cal_status_var.set(f"Timeline log closed: {path}")

    def _timeline_log_enabled(self) -> bool:
        try:
            return bool(self.follow_timeline_log_var.get())
        except tk.TclError:
            return False

    @staticmethod
    def _timeline_fields() -> tuple[str, ...]:
        return (
            "timestamp_iso",
            "monotonic_s",
            "event",
            "step_id",
            "step_label",
            "result",
            "reason",
            "run_id",
            "run_step",
            "frame_sequence",
            "frame_timestamp_s",
            "frame_age_ms",
            "capture_fps",
            "preview_fps",
            "use_delayed_position",
            "position_delay_ms",
            "use_virtual_target",
            "virtual_min_progress_pct",
            "frame_target_time_s",
            "delayed_position_found",
            "delayed_position_source",
            "delayed_position_age_ms",
            "delayed_position_error_ms",
            "delayed_x",
            "delayed_y",
            "target_base_x",
            "target_base_y",
            "frame_move_x",
            "frame_move_y",
            "command_adjust_x",
            "command_adjust_y",
            "virtual_state",
            "virtual_desired_x",
            "virtual_desired_y",
            "virtual_target_x",
            "virtual_target_y",
            "virtual_raw_parallel",
            "virtual_raw_perp",
            "virtual_final_parallel",
            "virtual_final_perp",
            "virtual_min_forward",
            "virtual_side_limit",
            "detection_phase",
            "retry_attempt",
            "retry_wait_ms",
            "fresh_frame",
            "linuxcnc_read_ms",
            "status_ok",
            "task_state",
            "task_mode",
            "interp_state",
            "start_x",
            "start_y",
            "post_x",
            "post_y",
            "target_x",
            "target_y",
            "move_x",
            "move_y",
            "move_len",
            "tangent_x",
            "tangent_y",
            "correct_x",
            "correct_y",
            "raw_correct_len",
            "applied_correct_len",
            "confidence",
            "min_confidence",
            "pixel_error_x",
            "pixel_error_y",
            "angle_degrees",
            "span_px",
            "width_px",
            "point_count",
            "search_px",
            "heading_state",
            "heading_change_degrees",
            "progress_dot",
            "correction_state",
            "corner_candidate",
            "corner_angle_degrees",
            "corner_strength",
            "duration_ms",
        )

    def _open_timeline_log(self) -> Optional[Path]:
        if self._timeline_csv is not None and self._timeline_log_path is not None:
            return self._timeline_log_path
        try:
            log_dir = Path.home() / "FabScan Logs"
            log_dir.mkdir(parents=True, exist_ok=True)
            path = log_dir / f"fabscan_timeline_{datetime.now().strftime('%Y-%m-%d_%H%M%S')}.csv"
            file = path.open("w", newline="", encoding="utf-8")
            writer = csv.DictWriter(file, fieldnames=self._timeline_fields(), extrasaction="ignore")
            writer.writeheader()
            self._timeline_log_file = file
            self._timeline_csv = writer
            self._timeline_log_path = path
            self._timeline_log(
                "SESSION_START",
                result="ok",
                capture_fps=self._get_camera_stream_max_fps(),
                preview_fps=self._get_camera_preview_max_fps(),
                use_delayed_position=bool(self.follow_use_delayed_position_var.get()),
                position_delay_ms=self._get_follow_position_delay_ms(),
                use_virtual_target=bool(self.follow_virtual_target_var.get()),
                virtual_min_progress_pct=f"{self._get_follow_virtual_min_progress_pct():.3f}",
            )
            return path
        except OSError as exc:
            self._timeline_log_file = None
            self._timeline_csv = None
            self._timeline_log_path = None
            try:
                self.follow_timeline_log_var.set(False)
            except tk.TclError:
                pass
            self.cal_status_var.set(f"Timeline log could not open: {exc}")
            return None

    def _close_timeline_log(self) -> None:
        file = self._timeline_log_file
        self._timeline_csv = None
        self._timeline_log_file = None
        if file is not None:
            try:
                file.close()
            except OSError:
                pass

    def _timeline_frame_fields(self) -> dict[str, Any]:
        now = time.monotonic()
        frame_ts = float(getattr(self, "current_frame_timestamp", 0.0) or 0.0)
        return {
            "frame_sequence": int(getattr(self, "current_frame_sequence", 0) or 0),
            "frame_timestamp_s": f"{frame_ts:.6f}" if frame_ts > 0.0 else "",
            "frame_age_ms": f"{(now - frame_ts) * 1000.0:.3f}" if frame_ts > 0.0 else "",
        }

    @staticmethod
    def _timeline_line_fields(line: Optional[LineDetection]) -> dict[str, Any]:
        if line is None:
            return {}
        return {
            "confidence": f"{float(line.confidence):.3f}",
            "pixel_error_x": f"{float(line.pixel_error_x):.3f}",
            "pixel_error_y": f"{float(line.pixel_error_y):.3f}",
            "angle_degrees": f"{float(line.angle_degrees):.3f}",
            "span_px": f"{float(line.span_px):.3f}",
            "width_px": f"{float(line.width_px):.3f}",
            "point_count": int(line.point_count),
            "search_px": int(line.search_px),
            "corner_candidate": bool(line.corner_candidate),
            "corner_angle_degrees": f"{float(line.corner_angle_degrees):.3f}",
            "corner_strength": f"{float(line.corner_strength):.3f}",
        }

    def _current_frame_age_s(self) -> Optional[float]:
        frame_ts = float(getattr(self, "current_frame_timestamp", 0.0) or 0.0)
        if frame_ts <= 0.0:
            return None
        return max(0.0, time.monotonic() - frame_ts)

    def _max_follow_frame_age_s(self) -> float:
        """Accept a wider age window at low capture rates, tighter at 30 fps."""

        fps = max(0.1, self._get_camera_stream_max_fps())
        return max(0.25, min(0.75, 2.5 / fps))

    def _wait_for_fresh_camera_frame(
        self,
        *,
        previous_sequence: int,
        timeout_s: float,
    ) -> tuple[bool, float]:
        """Pump the camera until a newer frame is available or timeout expires."""

        start = time.monotonic()
        end = start + max(0.0, float(timeout_s))
        while time.monotonic() < end and not self._closing:
            self.update()
            self._pump_camera_frame(display=False)
            if int(getattr(self, "current_frame_sequence", 0) or 0) > int(previous_sequence):
                return True, time.monotonic() - start
            time.sleep(0.01)
        return False, time.monotonic() - start

    def _freshen_follow_frame_if_stale(self, *, step_id: int, step_label: str) -> None:
        """Avoid starting a follow decision from a stale paused/preview frame."""

        if self.current_frame_bgr is None:
            return
        age = self._current_frame_age_s()
        max_age = self._max_follow_frame_age_s()
        if age is not None and age <= max_age:
            return

        previous_sequence = int(getattr(self, "current_frame_sequence", 0) or 0)
        refreshed, waited_s = self._wait_for_fresh_camera_frame(
            previous_sequence=previous_sequence,
            timeout_s=self._follow_not_found_retry_timeout_s,
        )
        self._timeline_log(
            "FRAME_FRESHEN",
            step_id=step_id,
            step_label=step_label,
            result="fresh" if refreshed else "stale",
            reason=f"frame age exceeded {max_age * 1000.0:.0f} ms" if age is not None else "frame timestamp unavailable",
            retry_wait_ms=f"{waited_s * 1000.0:.3f}",
            fresh_frame=bool(refreshed),
        )

    def _detect_line_for_follow_with_retries(
        self,
        *,
        step_id: int,
        step_label: str,
        phase: str,
        base_event: str,
    ) -> tuple[LineDetection, float]:
        """Detect a line/edge and retry transient not-found frames safely.

        v0.5.3.3 intentionally retries only complete not-found detections. It
        does not bypass confidence, sanity, progress lock, or motion limits. The
        retry waits for a newer camera frame first, so it can recover from a bad
        frame without re-processing the same failed image over and over.
        """

        attempts = max(0, int(self._follow_not_found_retry_count))
        total_detection_ms = 0.0
        detection_start = time.monotonic()
        line = self.detect_line()
        detection_ms = (time.monotonic() - detection_start) * 1000.0
        total_detection_ms += detection_ms
        self._timeline_log(
            base_event,
            step_id=step_id,
            step_label=step_label,
            result="found" if line.found else "not_found",
            reason=line.message,
            detection_phase=phase,
            retry_attempt=0,
            duration_ms=f"{detection_ms:.3f}",
            **self._timeline_line_fields(line),
        )

        if line.found:
            self._last_follow_detection_sequence = int(getattr(self, "current_frame_sequence", 0) or 0)
            self._last_follow_detection_result = "found"
            return line, total_detection_ms

        for attempt in range(1, attempts + 1):
            previous_sequence = int(getattr(self, "current_frame_sequence", 0) or 0)
            refreshed, waited_s = self._wait_for_fresh_camera_frame(
                previous_sequence=previous_sequence,
                timeout_s=self._follow_not_found_retry_timeout_s,
            )
            self._timeline_log(
                f"{base_event}_RETRY_WAIT",
                step_id=step_id,
                step_label=step_label,
                result="fresh" if refreshed else "timeout",
                reason=line.message,
                detection_phase=phase,
                retry_attempt=attempt,
                retry_wait_ms=f"{waited_s * 1000.0:.3f}",
                fresh_frame=bool(refreshed),
            )
            if not refreshed:
                continue

            retry_start = time.monotonic()
            retry_line = self.detect_line()
            retry_ms = (time.monotonic() - retry_start) * 1000.0
            total_detection_ms += retry_ms
            self._timeline_log(
                f"{base_event}_RETRY",
                step_id=step_id,
                step_label=step_label,
                result="found" if retry_line.found else "not_found",
                reason=retry_line.message,
                detection_phase=phase,
                retry_attempt=attempt,
                duration_ms=f"{retry_ms:.3f}",
                **self._timeline_line_fields(retry_line),
            )
            line = retry_line
            if line.found:
                self._last_follow_detection_sequence = int(getattr(self, "current_frame_sequence", 0) or 0)
                self._last_follow_detection_result = f"found after retry {attempt}"
                return line, total_detection_ms

        self._last_follow_detection_sequence = int(getattr(self, "current_frame_sequence", 0) or 0)
        self._last_follow_detection_result = "not_found"
        return line, total_detection_ms

    def _timeline_log(self, event: str, **kwargs: Any) -> None:
        """Write one Stage 2 timeline event.

        This is intentionally low-rate: it logs Follow Step / Follow N decisions
        and jog timing, not every preview frame. That keeps the RT-safe preview
        gains from v0.5.26 intact while giving us a time-aligned flight recorder.
        """

        if not self._timeline_log_enabled():
            return
        if self._timeline_csv is None:
            if self._open_timeline_log() is None:
                return
        writer = self._timeline_csv
        file = self._timeline_log_file
        if writer is None or file is None:
            return
        row: dict[str, Any] = {field: "" for field in self._timeline_fields()}
        row.update(
            {
                "timestamp_iso": datetime.now().isoformat(timespec="milliseconds"),
                "monotonic_s": f"{time.monotonic():.6f}",
                "event": event,
                "run_id": int(getattr(self, "_active_follow_run_id", 0) or 0),
                "run_step": int(getattr(self, "_active_follow_run_step", 0) or 0),
            }
        )
        row.update(self._timeline_frame_fields())
        for key, value in kwargs.items():
            if key in row:
                row[key] = value
        try:
            writer.writerow(row)
            file.flush()
        except OSError as exc:
            try:
                self.follow_timeline_log_var.set(False)
            except tk.TclError:
                pass
            self.cal_status_var.set(f"Timeline log disabled after write error: {exc}")
            self._close_timeline_log()

    def _register_traces(self) -> None:
        watched_vars: tuple[tk.Variable, ...] = (
            self.rotate_var,
            self.flip_x_var,
            self.flip_y_var,
            self.fine_rotation_var,
            self.threshold_var,
            self.show_dot_marker_var,
            self.show_mask_var,
            self.line_mode_var,
            self.line_search_px_var,
            self.show_line_preview_var,
        )
        for variable in watched_vars:
            variable.trace_add("write", lambda *_args: self._on_preview_setting_changed())
        status_vars: tuple[tk.Variable, ...] = (
            self.dot_status_var,
            self.cal_status_var,
            self.transform_status_var,
            self.line_status_var,
        )
        for variable in status_vars:
            variable.trace_add("write", lambda *_args: self._refresh_status_text())
        self.follow_direction_var.trace_add("write", lambda *_args: self._clear_follow_heading(clear_corner_resume=False))
        self.follow_stabilize_var.trace_add("write", lambda *_args: self._reset_follow_filter())

    def _refresh_status_text(self) -> None:
        status_text = getattr(self, "status_text", None)
        if status_text is None:
            return
        text = "\n\n".join(
            (
                str(self.dot_status_var.get()),
                str(self.cal_status_var.get()),
                str(self.transform_status_var.get()),
                str(self.line_status_var.get()),
            )
        )
        try:
            first, last = status_text.yview()
            auto_scroll = last >= 0.98
            status_text.configure(state=tk.NORMAL)
            status_text.delete("1.0", tk.END)
            status_text.insert("1.0", text)
            status_text.configure(state=tk.DISABLED)
            if auto_scroll:
                status_text.see("end-1c")
            else:
                # Preserve the user's scroll position while live status text
                # changes. This stops the scrollbar from snapping back to the
                # bottom while reading older/top status lines.
                status_text.yview_moveto(first)
        except tk.TclError:
            return

    def _clear_follow_heading(self, *, clear_corner_resume: bool = True) -> None:
        self._follow_heading_unit = None
        self._follow_last_move_unit = None
        self._reset_follow_filter()
        if clear_corner_resume:
            self._follow_corner_resume_once = False

    def _reset_follow_filter(self) -> None:
        self._follow_filtered_err = None
        self._follow_filtered_vec_px = None

    def _on_preview_setting_changed(self) -> None:
        # Any camera/threshold/search change can alter the fitted tangent. Start
        # a fresh follow latch after the user deliberately changes detection setup.
        self._clear_follow_heading()
        self._normalize_fine_rotation_var()
        self._update_fine_rotation_label()
        self._update_threshold_label()
        self._show_current_frame()

    def _clamp_fine_rotation(self, value: float) -> float:
        try:
            angle = float(value)
        except (TypeError, ValueError):
            angle = 0.0
        return max(-10.0, min(10.0, angle))

    def _normalize_fine_rotation_var(self) -> None:
        try:
            current = float(self.fine_rotation_var.get())
        except (tk.TclError, TypeError, ValueError):
            current = 0.0
        clamped = self._clamp_fine_rotation(current)
        if abs(clamped - current) > 1e-9:
            self.fine_rotation_var.set(clamped)

    def _update_fine_rotation_label(self) -> None:
        self.fine_rotation_label.configure(text=f"{self._get_fine_rotation_degrees():+.1f}°")

    def _update_threshold_label(self) -> None:
        self.threshold_label.configure(text=str(self._get_threshold()))

    def _clamp_camera_stream_max_fps(self, value: object) -> float:
        try:
            fps = float(value)
        except (tk.TclError, TypeError, ValueError):
            fps = DEFAULT_CAMERA_STREAM_MAX_FPS
        return max(1.0, min(60.0, fps))

    def _get_camera_stream_max_fps(self) -> float:
        fps = self._clamp_camera_stream_max_fps(self.camera_stream_max_fps_var.get())
        try:
            if abs(float(self.camera_stream_max_fps_var.get()) - fps) > 1e-9:
                self.camera_stream_max_fps_var.set(fps)
        except (tk.TclError, TypeError, ValueError):
            self.camera_stream_max_fps_var.set(fps)
        return fps

    def _clamp_camera_preview_max_fps(self, value: object) -> float:
        try:
            fps = float(value)
        except (tk.TclError, TypeError, ValueError):
            fps = DEFAULT_CAMERA_PREVIEW_MAX_FPS
        return max(0.2, min(60.0, fps))

    def _get_camera_preview_max_fps(self) -> float:
        fps = self._clamp_camera_preview_max_fps(self.camera_preview_max_fps_var.get())
        try:
            if abs(float(self.camera_preview_max_fps_var.get()) - fps) > 1e-9:
                self.camera_preview_max_fps_var.set(fps)
        except (tk.TclError, TypeError, ValueError):
            self.camera_preview_max_fps_var.set(fps)
        return fps

    def apply_linuxcnc_safe_preview(self) -> None:
        if not bool(self.linuxcnc_safe_preview_var.get()):
            return
        self.camera_stream_max_fps_var.set(5.0)
        self.camera_preview_max_fps_var.set(1.0)
        self.show_line_preview_var.set(False)
        self.show_dot_marker_var.set(False)
        self.show_mask_var.set(False)
        self.follow_corner_pause_var.set(False)
        self.follow_corner_assist_var.set(False)
        self.cal_status_var.set("LinuxCNC Safe preview: capture 5 fps, display 1 fps, line/dot/corner preview work off.")

    def _profile_enabled(self) -> bool:
        try:
            return bool(self.profile_preview_var.get())
        except tk.TclError:
            return False

    def _get_requested_size(self) -> tuple[int, int]:
        try:
            width = int(self.camera_width_var.get())
        except (tk.TclError, TypeError, ValueError):
            width = 0
        try:
            height = int(self.camera_height_var.get())
        except (tk.TclError, TypeError, ValueError):
            height = 0
        return max(0, width), max(0, height)

    def apply_resolution_preset(self) -> None:
        preset = parse_preset_label(self.camera_preset_var.get())
        if preset is None:
            return
        width, height = preset
        self.camera_width_var.set(width)
        self.camera_height_var.set(height)
        self.cal_status_var.set(f"Preset selected: {width} x {height}. Click Open / Restart to apply.")

    def _get_camera_index(self) -> int:
        try:
            return max(0, int(self.camera_index_var.get()))
        except (tk.TclError, TypeError, ValueError):
            return 0

    def _get_rotate_degrees(self) -> int:
        try:
            rotate_degrees = int(self.rotate_var.get())
        except (tk.TclError, TypeError, ValueError):
            rotate_degrees = 0
        if rotate_degrees not in ROTATE_VALUES:
            rotate_degrees = 0
        return rotate_degrees

    def _get_fine_rotation_degrees(self) -> float:
        try:
            return self._clamp_fine_rotation(float(self.fine_rotation_var.get()))
        except (tk.TclError, TypeError, ValueError):
            return 0.0

    def _get_threshold(self) -> int:
        try:
            return max(0, min(255, int(round(float(self.threshold_var.get())))))
        except (tk.TclError, TypeError, ValueError):
            return 90

    def _get_move_distance(self) -> float:
        try:
            move = abs(float(self.move_distance_var.get()))
        except (tk.TclError, TypeError, ValueError):
            move = 0.100
        return max(0.001, min(1.000, move))

    def _get_feed(self) -> float:
        try:
            feed = abs(float(self.feed_var.get()))
        except (tk.TclError, TypeError, ValueError):
            feed = 5.0
        return max(0.1, min(120.0, feed))

    def _get_jog_step(self) -> float:
        try:
            step = abs(float(self.jog_step_var.get()))
        except (tk.TclError, TypeError, ValueError):
            step = 0.010
        return max(0.001, min(1.000, step))

    def _get_center_max_move(self) -> float:
        try:
            move = abs(float(self.center_max_move_var.get()))
        except (tk.TclError, TypeError, ValueError):
            move = 0.100
        return max(0.001, min(1.000, move))

    def _get_follow_step(self) -> float:
        try:
            step = abs(float(self.follow_step_var.get()))
        except (tk.TclError, TypeError, ValueError):
            step = 0.050
        return max(0.001, min(1.000, step))

    def _get_follow_feed(self) -> float:
        try:
            feed = abs(float(self.follow_feed_var.get()))
        except (tk.TclError, TypeError, ValueError):
            feed = 5.0
        feed = max(0.1, min(120.0, feed))
        self.follow_feed_var.set(feed)
        return feed

    def _clamp_follow_settle_ms(self, value: object) -> int:
        try:
            settle_ms = int(round(float(value)))
        except (tk.TclError, TypeError, ValueError):
            settle_ms = 150
        return max(0, min(2000, settle_ms))

    def _get_follow_settle_ms(self) -> int:
        settle_ms = self._clamp_follow_settle_ms(self.follow_settle_ms_var.get())
        try:
            if int(self.follow_settle_ms_var.get()) != settle_ms:
                self.follow_settle_ms_var.set(settle_ms)
        except (tk.TclError, TypeError, ValueError):
            self.follow_settle_ms_var.set(settle_ms)
        return settle_ms

    def _clamp_follow_position_delay_ms(self, value: object) -> int:
        try:
            delay_ms = int(round(float(value)))
        except (tk.TclError, TypeError, ValueError):
            delay_ms = 120
        return max(0, min(1000, delay_ms))

    def _get_follow_position_delay_ms(self) -> int:
        delay_ms = self._clamp_follow_position_delay_ms(self.follow_position_delay_ms_var.get())
        try:
            if int(self.follow_position_delay_ms_var.get()) != delay_ms:
                self.follow_position_delay_ms_var.set(delay_ms)
        except (tk.TclError, TypeError, ValueError):
            self.follow_position_delay_ms_var.set(delay_ms)
        return delay_ms

    def _clamp_follow_virtual_min_progress_pct(self, value: object) -> float:
        try:
            pct = abs(float(value))
        except (tk.TclError, TypeError, ValueError):
            pct = 70.0
        return max(0.0, min(100.0, pct))

    def _get_follow_virtual_min_progress_pct(self) -> float:
        pct = self._clamp_follow_virtual_min_progress_pct(self.follow_virtual_min_progress_pct_var.get())
        try:
            if abs(float(self.follow_virtual_min_progress_pct_var.get()) - pct) > 1e-9:
                self.follow_virtual_min_progress_pct_var.set(pct)
        except (tk.TclError, TypeError, ValueError):
            self.follow_virtual_min_progress_pct_var.set(pct)
        return pct

    def _get_follow_virtual_target_enabled(self) -> bool:
        try:
            return bool(self.follow_virtual_target_var.get())
        except tk.TclError:
            return False

    def _clamp_parallel_perp_move(
        self,
        *,
        move_x: float,
        move_y: float,
        heading: tuple[float, float],
        min_forward: float,
        max_forward: float,
        max_side: float,
    ) -> tuple[float, float, float, float, float, float]:
        unit = self._normalize_unit_vector(heading[0], heading[1])
        if unit is None:
            return move_x, move_y, 0.0, 0.0, 0.0, 0.0
        hx, hy = unit
        nx, ny = -hy, hx
        raw_parallel = (move_x * hx) + (move_y * hy)
        raw_perp = (move_x * nx) + (move_y * ny)
        final_parallel = max(min_forward, min(max_forward, raw_parallel))
        final_perp = max(-max_side, min(max_side, raw_perp))
        return (
            (final_parallel * hx) + (final_perp * nx),
            (final_parallel * hy) + (final_perp * ny),
            raw_parallel,
            raw_perp,
            final_parallel,
            final_perp,
        )

    def _clamp_follow_max_heading_change(self, value: object) -> float:
        try:
            angle = abs(float(value))
        except (tk.TclError, TypeError, ValueError):
            angle = 70.0
        return max(0.0, min(180.0, angle))

    def _get_follow_max_heading_change(self) -> float:
        angle = self._clamp_follow_max_heading_change(self.follow_max_heading_change_var.get())
        try:
            if abs(float(self.follow_max_heading_change_var.get()) - angle) > 1e-9:
                self.follow_max_heading_change_var.set(angle)
        except (tk.TclError, TypeError, ValueError):
            self.follow_max_heading_change_var.set(angle)
        return angle

    def _clamp_follow_corner_angle(self, value: object) -> float:
        try:
            angle = abs(float(value))
        except (tk.TclError, TypeError, ValueError):
            angle = 55.0
        return max(10.0, min(135.0, angle))

    def _get_follow_corner_angle(self, *, normalize: bool = False) -> float:
        # The Corner° entry is intentionally a StringVar so a user can edit it
        # normally. During live overlay updates it may temporarily contain
        # values like "" or "5"; do not clamp/write those back while the user
        # is typing, or the entry jumps to 10/135 and becomes nearly unusable.
        angle = self._clamp_follow_corner_angle(self.follow_corner_angle_var.get())
        if normalize:
            try:
                self.follow_corner_angle_var.set(f"{angle:g}")
            except tk.TclError:
                pass
        return angle

    def _clamp_follow_corner_lookahead_steps(self, value: object) -> int:
        try:
            steps = int(round(float(value)))
        except (tk.TclError, TypeError, ValueError):
            steps = 5
        return max(1, min(10, steps))

    def _get_follow_corner_lookahead_steps(self) -> int:
        steps = self._clamp_follow_corner_lookahead_steps(self.follow_corner_lookahead_steps_var.get())
        try:
            if int(self.follow_corner_lookahead_steps_var.get()) != steps:
                self.follow_corner_lookahead_steps_var.set(steps)
        except (tk.TclError, TypeError, ValueError):
            self.follow_corner_lookahead_steps_var.set(steps)
        return steps

    def _get_follow_max_correct(self) -> float:
        try:
            move = abs(float(self.follow_max_correct_var.get()))
        except (tk.TclError, TypeError, ValueError):
            move = 0.050
        return max(0.0, min(1.000, move))

    def _clamp_follow_deadband(self, value: object) -> float:
        try:
            deadband = abs(float(value))
        except (tk.TclError, TypeError, ValueError):
            deadband = 0.003
        return max(0.0, min(1.000, deadband))

    def _get_follow_deadband(self) -> float:
        deadband = self._clamp_follow_deadband(self.follow_deadband_var.get())
        try:
            if abs(float(self.follow_deadband_var.get()) - deadband) > 1e-9:
                self.follow_deadband_var.set(deadband)
        except (tk.TclError, TypeError, ValueError):
            self.follow_deadband_var.set(deadband)
        return deadband

    def _clamp_follow_gain(self, value: object) -> float:
        try:
            gain = float(value)
        except (tk.TclError, TypeError, ValueError):
            gain = 0.50
        return max(0.0, min(2.0, gain))

    def _get_follow_gain(self) -> float:
        gain = self._clamp_follow_gain(self.follow_gain_var.get())
        try:
            if abs(float(self.follow_gain_var.get()) - gain) > 1e-9:
                self.follow_gain_var.set(gain)
        except (tk.TclError, TypeError, ValueError):
            self.follow_gain_var.set(gain)
        return gain

    def _get_follow_stabilize_enabled(self) -> bool:
        try:
            return bool(self.follow_stabilize_var.get())
        except tk.TclError:
            return True

    def _clamp_follow_filter_alpha(self, value: object, *, default: float) -> float:
        try:
            alpha = float(value)
        except (tk.TclError, TypeError, ValueError):
            alpha = default
        # 0 disables update, 1 follows the latest frame exactly. Keep a useful
        # range so a typo cannot make the filter explode.
        return max(0.0, min(1.0, alpha))

    def _get_follow_filter_offset_alpha(self) -> float:
        alpha = self._clamp_follow_filter_alpha(self.follow_filter_offset_alpha_var.get(), default=0.35)
        try:
            if abs(float(self.follow_filter_offset_alpha_var.get()) - alpha) > 1e-9:
                self.follow_filter_offset_alpha_var.set(alpha)
        except (tk.TclError, TypeError, ValueError):
            self.follow_filter_offset_alpha_var.set(alpha)
        return alpha

    def _get_follow_filter_angle_alpha(self) -> float:
        alpha = self._clamp_follow_filter_alpha(self.follow_filter_angle_alpha_var.get(), default=0.25)
        try:
            if abs(float(self.follow_filter_angle_alpha_var.get()) - alpha) > 1e-9:
                self.follow_filter_angle_alpha_var.set(alpha)
        except (tk.TclError, TypeError, ValueError):
            self.follow_filter_angle_alpha_var.set(alpha)
        return alpha

    def _clamp_follow_sanity_angle(self, value: object) -> float:
        try:
            angle = abs(float(value))
        except (tk.TclError, TypeError, ValueError):
            angle = 30.0
        # 0 disables sanity reject; 90 is already very loose for same-feature
        # following and avoids fighting intentional corner handling.
        return max(0.0, min(90.0, angle))

    def _get_follow_sanity_angle(self, *, normalize: bool = False) -> float:
        angle = self._clamp_follow_sanity_angle(self.follow_sanity_angle_var.get())
        if normalize:
            try:
                if abs(float(self.follow_sanity_angle_var.get()) - angle) > 1e-9:
                    self.follow_sanity_angle_var.set(angle)
            except (tk.TclError, TypeError, ValueError):
                self.follow_sanity_angle_var.set(angle)
        return angle

    @staticmethod
    def _apply_follow_correction_tuning(
        move_x: float,
        move_y: float,
        *,
        deadband: float,
        gain: float,
        max_correct: float,
    ) -> tuple[float, float, bool, str, float, float]:
        """Apply deadband, gain, and hard limit to a side-correction vector.

        Returns tuned_x, tuned_y, limited, state_text, raw_len, tuned_len. The
        deadband is in machine units and is removed from the raw correction
        magnitude before gain is applied. This keeps tiny camera/detection noise
        from causing edge-to-edge hunting while still nudging back toward center
        when the offset becomes meaningful.
        """

        raw_len = math.hypot(move_x, move_y)
        if raw_len < 1e-12:
            return 0.0, 0.0, False, "no side correction", raw_len, 0.0
        if raw_len <= deadband:
            return 0.0, 0.0, False, "inside deadband", raw_len, 0.0

        desired_len = (raw_len - deadband) * gain
        if desired_len <= 0.0 or max_correct <= 0.0:
            return 0.0, 0.0, raw_len > deadband, "gain/limit zeroed", raw_len, 0.0

        limited = desired_len > max_correct
        tuned_len = min(desired_len, max_correct)
        scale = tuned_len / raw_len
        return move_x * scale, move_y * scale, limited, "tuned", raw_len, tuned_len

    def _get_follow_min_confidence(self) -> float:
        try:
            confidence = float(self.follow_min_confidence_var.get())
        except (tk.TclError, TypeError, ValueError):
            confidence = 45.0
        return max(0.0, min(100.0, confidence))

    def _get_follow_repeat_count(self) -> int:
        try:
            count = int(self.follow_repeat_count_var.get())
        except (tk.TclError, TypeError, ValueError):
            count = 5
        count = max(1, min(9999, count))
        self.follow_repeat_count_var.set(count)
        return count

    def _normalize_follow_direction(self, value: object) -> str:
        text = str(value or "Y+").strip().upper().replace(" ", "")
        if text in {"X+", "+X"}:
            return "X+"
        if text in {"X-", "-X"}:
            return "X-"
        if text in {"Y+", "+Y"}:
            return "Y+"
        if text in {"Y-", "-Y"}:
            return "Y-"
        # v0.5.16 and older used Forward/Reverse. Convert old saved values to
        # a deterministic machine-axis latch instead of carrying the ambiguous
        # wording forward. The user can switch to X+/X-/Y- for the next test.
        if text in {"FORWARD", "REVERSE"}:
            return "Y+"
        return "Y+"

    def _get_follow_direction_preference(self) -> str:
        raw_direction = self.follow_direction_var.get()
        direction = self._normalize_follow_direction(raw_direction)
        # Do not write the normalized value back when it is already unchanged.
        # Tk variable traces fire even for same-value writes on some systems;
        # v0.5.23 accidentally cleared the follow/progress latch every step by
        # calling set() here while reading Start dir. Only normalize legacy or
        # malformed values that actually differ.
        if str(raw_direction or "").strip() != direction:
            self.follow_direction_var.set(direction)
        return direction

    @staticmethod
    def _axis_vector_from_follow_direction(direction: str) -> tuple[float, float]:
        if direction == "X+":
            return 1.0, 0.0
        if direction == "X-":
            return -1.0, 0.0
        if direction == "Y-":
            return 0.0, -1.0
        return 0.0, 1.0

    def _normalize_line_mode(self, value: object) -> str:
        text = str(value or "Line center").strip()
        if text in {"Line center", "Edge near center"}:
            return text
        return "Line center"

    def _get_line_mode(self) -> str:
        return self._normalize_line_mode(self.line_mode_var.get())

    def _clamp_line_search_px(self, value: object) -> int:
        try:
            pixels = int(round(float(value)))
        except (tk.TclError, TypeError, ValueError):
            pixels = 220
        return max(40, min(1000, pixels))

    def _get_line_search_px(self, *, normalize: bool = False) -> int:
        try:
            raw = str(self.line_search_px_var.get()).strip()
        except (tk.TclError, TypeError, ValueError):
            raw = ""

        if raw == "":
            pixels = getattr(self, "_last_valid_line_search_px", 220)
        else:
            try:
                pixels = max(40, min(1000, int(round(float(raw)))))
                self._last_valid_line_search_px = pixels
            except (TypeError, ValueError):
                pixels = getattr(self, "_last_valid_line_search_px", 220)

        if normalize:
            text = str(pixels)
            try:
                if self.line_search_px_var.get() != text:
                    self.line_search_px_var.set(text)
            except tk.TclError:
                self.line_search_px_var.set(text)
        return pixels

    def open_camera(self) -> None:
        self.release_camera()
        self.current_frame_bgr = None
        self.current_frame_sequence = 0
        self.current_frame_timestamp = 0.0
        self.current_dot = DotDetection(False)
        self.current_line = LineDetection(False)
        self._update_threshold_label()

        index = self._get_camera_index()
        width, height = self._get_requested_size()
        self.camera_preset_var.set(size_to_preset_label(width, height))

        cap, info, error = open_camera_capture(index, width, height)
        if cap is None:
            self.cap = None
            self.cal_status_var.set(f"Camera {index} did not open. {error}")
            return

        if bool(self.linuxcnc_safe_preview_var.get()):
            try:
                cv2.setNumThreads(1)
            except Exception:
                pass
        fps_cap = self._get_camera_stream_max_fps()
        preview_fps = self._get_camera_preview_max_fps()
        self._last_camera_sequence = 0
        self._last_preview_sequence = 0
        self._next_preview_due = 0.0
        self._preview_count = 0
        stream = CameraStream(cap, info, max_fps=fps_cap)
        stream.start()
        self.cap = stream

        status = (
            f"Camera {index} open via {info.backend_name}. Requested {info.requested_size_text}; "
            f"actual {info.actual_size_text}; format {info.fourcc}; read cap {fps_cap:g} fps; "
            f"preview cap {preview_fps:g} fps."
        )
        if info.warning:
            status += f" Warning: {info.warning}"
        self.cal_status_var.set(status)
        self._schedule_next_frame()

    def release_camera(self) -> None:
        if self.after_job is not None:
            try:
                self.after_cancel(self.after_job)
            except tk.TclError:
                pass
            self.after_job = None
        if self.cap is not None:
            self.cap.close()
            self.cap = None

    def _schedule_next_frame(self) -> None:
        if self._closing:
            return
        delay_ms = 50
        if self._next_preview_due > 0.0:
            delay_ms = max(20, int(max(0.0, self._next_preview_due - time.monotonic()) * 1000.0))
        self.after_job = self.after(delay_ms, self._update_preview)

    def _update_preview(self) -> None:
        self.after_job = None
        if self._closing or self.cap is None:
            return
        now = time.monotonic()
        if self._next_preview_due > 0.0 and now < self._next_preview_due:
            self._schedule_next_frame()
            return
        self._next_preview_due = now + (1.0 / self._get_camera_preview_max_fps())
        self._pump_camera_frame(display=True)
        if self.cap is not None and not self._closing:
            self._schedule_next_frame()

    def _pump_camera_frame(self, *, display: bool = False) -> bool:
        if self._closing or self.cap is None:
            return False
        t0 = time.perf_counter()
        result = self.cap.get_latest_frame_with_id()
        t_get = time.perf_counter()
        if result is not None:
            frame, sequence, timestamp = result
            if sequence == self._last_camera_sequence and not display:
                return False
            self._last_camera_sequence = sequence
            self.current_frame_bgr = frame
            self.current_frame_sequence = int(sequence)
            self.current_frame_timestamp = float(timestamp)
            if display and sequence != self._last_preview_sequence:
                self._last_preview_sequence = sequence
                self._show_frame(frame)
                self._preview_count += 1
                if self._profile_enabled():
                    stats = self.cap.stats()
                    print(
                        "FabScan calibration preview "
                        f"seq={sequence} get_ms={(t_get - t0) * 1000.0:.1f} "
                        f"total_ms={(time.perf_counter() - t0) * 1000.0:.1f} "
                        f"cap_fps={stats.get('read_fps', 0.0):.2f} preview_count={self._preview_count} "
                        f"dropped={stats.get('dropped_count', 0.0):.0f}",
                        flush=True,
                    )
            return True
        message = self.cap.status_message()
        if message:
            self.cal_status_var.set(message)
        return False

    def _show_current_frame(self) -> None:
        if self.current_frame_bgr is not None:
            self._show_frame(self.current_frame_bgr)

    def get_transformed_frame_bgr(self, frame_bgr: np.ndarray) -> np.ndarray:
        frame = frame_bgr
        rotate_degrees = self._get_rotate_degrees()
        if rotate_degrees == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif rotate_degrees == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif rotate_degrees == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        if bool(self.flip_x_var.get()):
            frame = cv2.flip(frame, 1)
        if bool(self.flip_y_var.get()):
            frame = cv2.flip(frame, 0)

        fine_degrees = self._get_fine_rotation_degrees()
        if abs(fine_degrees) > 0.0001:
            h, w = frame.shape[:2]
            center = (w / 2.0, h / 2.0)
            matrix = cv2.getRotationMatrix2D(center, fine_degrees, 1.0)
            frame = cv2.warpAffine(
                frame,
                matrix,
                (w, h),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_REPLICATE,
            )
        return frame

    def _make_mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        # Black dot/ring on white paper. Dark pixels become white in the mask.
        _, mask = cv2.threshold(gray, self._get_threshold(), 255, cv2.THRESH_BINARY_INV)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        return mask

    def detect_dot(self) -> DotDetection:
        if self.current_frame_bgr is None:
            return DotDetection(False, message="No camera frame yet")
        frame = self.get_transformed_frame_bgr(self.current_frame_bgr)
        return self.detect_dot_in_frame(frame)

    def detect_dot_in_frame(self, frame_bgr: np.ndarray) -> DotDetection:
        mask = self._make_mask(frame_bgr)
        contours, _hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        h, w = mask.shape[:2]
        center_x = w / 2.0
        center_y = h / 2.0
        diagonal = max(1.0, math.hypot(w, h))

        best: Optional[DotDetection] = None
        best_score = -1.0
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < 10.0 or area > float(w * h) * 0.35:
                continue
            moments = cv2.moments(contour)
            if abs(moments.get("m00", 0.0)) < 1e-9:
                continue
            x = float(moments["m10"] / moments["m00"])
            y = float(moments["m01"] / moments["m00"])
            perimeter = float(cv2.arcLength(contour, True))
            circularity = 0.0
            if perimeter > 1e-9:
                circularity = max(0.0, min(1.0, 4.0 * math.pi * area / (perimeter * perimeter)))
            distance_score = max(0.0, 1.0 - (math.hypot(x - center_x, y - center_y) / (diagonal * 0.50)))
            area_score = min(1.0, math.sqrt(area) / 120.0)
            score = (0.55 * distance_score) + (0.25 * circularity) + (0.20 * area_score)
            if score > best_score:
                best_score = score
                best = DotDetection(
                    found=True,
                    x=x,
                    y=y,
                    area=area,
                    confidence=max(0.0, min(100.0, score * 100.0)),
                    message="Dot found",
                )

        if best is None:
            return DotDetection(False, message="No dark calibration target found. Adjust threshold/light or recenter dot.")
        return best

    def _show_frame(self, frame_bgr: np.ndarray) -> None:
        profile = self._profile_enabled()
        t0 = time.perf_counter()
        transformed = self.get_transformed_frame_bgr(frame_bgr)
        t_transform = time.perf_counter()
        if bool(self.show_dot_marker_var.get()):
            self.current_dot = self.detect_dot_in_frame(transformed)
        else:
            self.current_dot = DotDetection(False, message="Dot preview disabled")
        t_dot = time.perf_counter()
        if bool(self.show_line_preview_var.get()):
            self.current_line = self.detect_line_in_frame(transformed)
        else:
            self.current_line = LineDetection(False, message="Line preview disabled")
        t_line = time.perf_counter()

        if bool(self.show_mask_var.get()):
            mask = self._make_mask(transformed)
            preview_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        else:
            preview_rgb = cv2.cvtColor(transformed, cv2.COLOR_BGR2RGB)

        t_mask = time.perf_counter()
        pil_image = Image.fromarray(preview_rgb)
        original_w, original_h = pil_image.size
        # Scale against a fixed preview box instead of the label's current
        # requested size. This prevents the live image from changing the dialog
        # geometry frame-to-frame.
        max_w = max(1, int(self.preview_display_width))
        max_h = max(1, int(self.preview_display_height))
        scale = min(max_w / original_w, max_h / original_h, 1.0)
        new_w = max(1, int(original_w * scale))
        new_h = max(1, int(original_h * scale))
        if (new_w, new_h) != pil_image.size:
            pil_image = pil_image.resize((new_w, new_h), Image.Resampling.LANCZOS)

        t_resize = time.perf_counter()
        self._draw_overlay(pil_image, scale, original_w, original_h)
        t_overlay = time.perf_counter()
        if self._closing:
            return
        self._tk_preview = ImageTk.PhotoImage(pil_image, master=self)
        t_imagetk = time.perf_counter()
        try:
            self.preview_label.configure(image=self._tk_preview)
        except tk.TclError:
            return
        self._update_dot_status(original_w, original_h)
        self._update_line_status(original_w, original_h)
        if profile:
            t_end = time.perf_counter()
            print(
                "FabScan calibration frame "
                f"transform_ms={(t_transform - t0) * 1000.0:.1f} "
                f"dot_ms={(t_dot - t_transform) * 1000.0:.1f} "
                f"line_ms={(t_line - t_dot) * 1000.0:.1f} "
                f"mask_rgb_ms={(t_mask - t_line) * 1000.0:.1f} "
                f"resize_ms={(t_resize - t_mask) * 1000.0:.1f} "
                f"overlay_ms={(t_overlay - t_resize) * 1000.0:.1f} "
                f"imagetk_ms={(t_imagetk - t_overlay) * 1000.0:.1f} "
                f"total_ms={(t_end - t0) * 1000.0:.1f}",
                flush=True,
            )

    def _draw_overlay(self, pil_image: Image.Image, scale: float, source_w: int, source_h: int) -> None:
        draw = ImageDraw.Draw(pil_image)
        w, h = pil_image.size
        cx = w // 2
        cy = h // 2
        outline = (0, 0, 0)
        cross = (255, 230, 0)
        found = (0, 255, 0)
        missing = (255, 80, 80)

        # Keep the yellow calibration crosshair visible all the time. It is
        # the camera center reference used by both dot centering and edge/line
        # follow, and it is useful even when the dot marker overlay is hidden.
        draw.line((cx, 0, cx, h), fill=outline, width=5)
        draw.line((0, cy, w, cy), fill=outline, width=5)
        draw.line((cx, 0, cx, h), fill=cross, width=2)
        draw.line((0, cy, w, cy), fill=cross, width=2)

        r = 7
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), outline=outline, width=4)
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), outline=cross, width=2)

        if bool(self.show_dot_marker_var.get()):
            if self.current_dot.found:
                dx = int(round(self.current_dot.x * scale))
                dy = int(round(self.current_dot.y * scale))
                rr = 12
                draw.ellipse((dx - rr, dy - rr, dx + rr, dy + rr), outline=outline, width=5)
                draw.ellipse((dx - rr, dy - rr, dx + rr, dy + rr), outline=found, width=3)
                draw.line((dx - 18, dy, dx + 18, dy), fill=found, width=2)
                draw.line((dx, dy - 18, dx, dy + 18), fill=found, width=2)
            else:
                draw.text((10, 10), "DOT NOT FOUND", fill=missing)

        if bool(self.show_line_preview_var.get()):
            self._draw_line_overlay(draw, scale, source_w, source_h)

        source_text = f"Source {source_w}x{source_h}  Threshold {self._get_threshold()}"
        frame_age = self._current_frame_age_s()
        if frame_age is not None:
            source_text += f"  Frame age {frame_age * 1000.0:.0f} ms"
        draw.text((10, h - 24), source_text, fill=(255, 255, 255))
        preview_fps = self._get_camera_preview_max_fps()
        capture_fps = self._get_camera_stream_max_fps()
        if preview_fps < max(1.1, capture_fps * 0.50):
            draw.text(
                (10, h - 44),
                "LOW PREVIEW RATE: overlay is sampled; follow may use newer frames",
                fill=(255, 255, 255),
            )

    def _update_dot_status(self, frame_w: int, frame_h: int) -> None:
        if self.current_dot.found:
            err_x = self.current_dot.x - (frame_w / 2.0)
            err_y = self.current_dot.y - (frame_h / 2.0)
            self.dot_status_var.set(
                f"Dot: X {self.current_dot.x:.1f} Y {self.current_dot.y:.1f} | "
                f"offset from center X {err_x:+.1f}px Y {err_y:+.1f}px | "
                f"area {self.current_dot.area:.0f} | confidence {self.current_dot.confidence:.0f}%"
            )
        else:
            self.dot_status_var.set(f"Dot: not found - {self.current_dot.message}")

    def _search_box_bounds(self, frame_w: int, frame_h: int) -> tuple[int, int, int, int]:
        size = self._get_line_search_px()
        half = max(20, size // 2)
        cx = frame_w // 2
        cy = frame_h // 2
        x1 = max(0, cx - half)
        y1 = max(0, cy - half)
        x2 = min(frame_w, cx + half)
        y2 = min(frame_h, cy + half)
        return x1, y1, x2, y2

    @staticmethod
    def _angle_difference_degrees(a: float, b: float) -> float:
        diff = abs((a - b) % 180.0)
        if diff > 90.0:
            diff = 180.0 - diff
        return diff

    @staticmethod
    def _line_intersection_px(
        p1: tuple[float, float],
        d1: tuple[float, float],
        p2: tuple[float, float],
        d2: tuple[float, float],
    ) -> tuple[bool, float, float]:
        denom = (d1[0] * d2[1]) - (d1[1] * d2[0])
        if abs(denom) < 1e-6:
            return False, 0.0, 0.0
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        t = ((dx * d2[1]) - (dy * d2[0])) / denom
        x = p1[0] + (t * d1[0])
        y = p1[1] + (t * d1[1])
        if not math.isfinite(x) or not math.isfinite(y):
            return False, 0.0, 0.0
        return True, x, y

    def _fit_points_line_metrics(
        self,
        points_global: np.ndarray,
        *,
        center_x: float,
        center_y: float,
    ) -> Optional[dict[str, float]]:
        if points_global is None or len(points_global) < 5:
            return None
        fit = cv2.fitLine(points_global.reshape(-1, 1, 2), cv2.DIST_L2, 0, 0.01, 0.01)
        vx = float(fit[0][0])
        vy = float(fit[1][0])
        x0 = float(fit[2][0])
        y0 = float(fit[3][0])
        length = math.hypot(vx, vy)
        if length < 1e-9:
            return None
        vx /= length
        vy /= length
        if vx < 0.0 or (abs(vx) < 1e-9 and vy < 0.0):
            vx = -vx
            vy = -vy
        t = ((center_x - x0) * vx) + ((center_y - y0) * vy)
        closest_x = x0 + (t * vx)
        closest_y = y0 + (t * vy)
        err_x = closest_x - center_x
        err_y = closest_y - center_y
        return {
            "vx": vx,
            "vy": vy,
            "x0": x0,
            "y0": y0,
            "closest_x": closest_x,
            "closest_y": closest_y,
            "err_x": err_x,
            "err_y": err_y,
        }

    def _score_with_follow_continuity(
        self,
        base_score: float,
        points_global: np.ndarray,
        *,
        center_x: float,
        center_y: float,
        search_px: int,
    ) -> tuple[float, str]:
        """Prefer the same feature instead of re-electing from scratch.

        The old detector picked the best contour in each frame mostly by area,
        span, or distance to the search-box center. This adjustment is only
        active after a prior accepted detection/latch exists; it penalizes
        sudden offset jumps and heading disagreement so the contour choice has
        memory without needing optical flow yet.
        """

        if not self._get_follow_stabilize_enabled():
            return base_score, ""
        has_offset_memory = self._follow_filtered_err is not None
        has_angle_memory = self._follow_heading_unit is not None or self._follow_filtered_vec_px is not None
        if not has_offset_memory and not has_angle_memory:
            return base_score, ""

        metrics = self._fit_points_line_metrics(points_global, center_x=center_x, center_y=center_y)
        if metrics is None:
            return base_score, ""

        penalty_fraction = 0.0
        notes: list[str] = []
        if has_offset_memory and self._follow_filtered_err is not None:
            jump = math.hypot(
                float(metrics["err_x"]) - self._follow_filtered_err[0],
                float(metrics["err_y"]) - self._follow_filtered_err[1],
            )
            offset_window = max(8.0, float(search_px) * 0.25)
            offset_fraction = min(1.0, jump / offset_window)
            penalty_fraction += 0.35 * offset_fraction
            if jump >= 4.0:
                notes.append(f"offset continuity {jump:.0f}px")

        if has_angle_memory:
            angle_diff: Optional[float] = None
            machine_vec = self._machine_vector_from_pixel_vector(float(metrics["vx"]), float(metrics["vy"]))
            if self._follow_heading_unit is not None and machine_vec is not None:
                unit = self._normalize_unit_vector(machine_vec[0], machine_vec[1])
                if unit is not None:
                    angle_diff = min(
                        self._angle_between_unit_vectors(self._follow_heading_unit, unit),
                        self._angle_between_unit_vectors(self._follow_heading_unit, (-unit[0], -unit[1])),
                    )
            elif self._follow_filtered_vec_px is not None:
                unit = self._normalize_unit_vector(float(metrics["vx"]), float(metrics["vy"]))
                if unit is not None:
                    angle_diff = min(
                        self._angle_between_unit_vectors(self._follow_filtered_vec_px, unit),
                        self._angle_between_unit_vectors(self._follow_filtered_vec_px, (-unit[0], -unit[1])),
                    )
            if angle_diff is not None:
                sanity = max(5.0, self._get_follow_sanity_angle() or 30.0)
                angle_fraction = min(1.0, angle_diff / sanity)
                penalty_fraction += 0.50 * angle_fraction
                if angle_diff >= 5.0:
                    notes.append(f"heading continuity {angle_diff:.0f}°")

        penalty_fraction = max(0.0, min(0.85, penalty_fraction))
        return base_score * (1.0 - penalty_fraction), "; ".join(notes[:2])

    def _corner_detection_enabled(self) -> bool:
        try:
            # Corner Assist is only evaluated from the corner-pause path, so if
            # Corner pause is off there is no reason to spend CPU on Hough
            # corner/intersection detection during every preview/follow frame.
            return bool(self.follow_corner_pause_var.get())
        except tk.TclError:
            return False

    def _line_corner_metrics(
        self,
        roi: np.ndarray,
        *,
        search_px: int,
        corner_angle_min: float,
    ) -> CornerMetrics:
        """Detect whether the ROI contains two strong line directions.

        A sharp printed corner makes the current single-line fit average both
        legs into a rounded/diagonal result. This detector uses Hough line
        segments in the same search ROI to notice when there is a meaningful
        secondary direction. v0.5.21 also fits both Hough-direction clusters so
        the two line equations can be intersected for Corner Assist.
        """

        if roi.size <= 0:
            return CornerMetrics()

        edges = cv2.Canny(roi, 50, 150)
        min_line_length = max(18, int(round(search_px * 0.12)))
        max_line_gap = max(5, int(round(search_px * 0.04)))
        threshold = max(10, int(round(search_px * 0.07)))
        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180.0,
            threshold=threshold,
            minLineLength=min_line_length,
            maxLineGap=max_line_gap,
        )
        if lines is None:
            return CornerMetrics()

        segments: list[tuple[float, float, float, float, float, float]] = []
        for line in lines.reshape(-1, 4):
            x1, y1, x2, y2 = [float(v) for v in line]
            length = math.hypot(x2 - x1, y2 - y1)
            if length < float(min_line_length):
                continue
            angle = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180.0
            segments.append((angle, length, x1, y1, x2, y2))
        if len(segments) < 2:
            return CornerMetrics()

        clusters: list[dict[str, Any]] = []
        for angle, length, sx1, sy1, sx2, sy2 in sorted(segments, key=lambda item: item[1], reverse=True):
            best_index: Optional[int] = None
            best_diff = 180.0
            for idx, cluster in enumerate(clusters):
                diff = self._angle_difference_degrees(angle, float(cluster["angle"]))
                if diff < best_diff:
                    best_diff = diff
                    best_index = idx
            if best_index is not None and best_diff <= 15.0:
                cluster = clusters[best_index]
                new_weight = float(cluster["length"]) + length
                # Weighted circular mean in doubled-angle space so 0/180 wraps.
                c = float(cluster["cos2"]) + (math.cos(math.radians(angle * 2.0)) * length)
                sn = float(cluster["sin2"]) + (math.sin(math.radians(angle * 2.0)) * length)
                cluster["cos2"] = c
                cluster["sin2"] = sn
                cluster["length"] = new_weight
                cluster["angle"] = (math.degrees(math.atan2(sn, c)) / 2.0) % 180.0
                cluster["count"] = float(cluster["count"]) + 1.0
                cluster["points"].extend([(sx1, sy1), (sx2, sy2)])
            else:
                clusters.append(
                    {
                        "angle": angle,
                        "length": length,
                        "cos2": math.cos(math.radians(angle * 2.0)) * length,
                        "sin2": math.sin(math.radians(angle * 2.0)) * length,
                        "count": 1.0,
                        "points": [(sx1, sy1), (sx2, sy2)],
                    }
                )

        if len(clusters) < 2:
            return CornerMetrics()

        clusters.sort(key=lambda item: float(item["length"]), reverse=True)
        primary = clusters[0]
        best_secondary: Optional[dict[str, Any]] = None
        best_separation = 0.0
        for secondary in clusters[1:]:
            separation = self._angle_difference_degrees(float(primary["angle"]), float(secondary["angle"]))
            if separation > best_separation:
                best_separation = separation
                best_secondary = secondary

        if best_secondary is None:
            return CornerMetrics()

        def fit_cluster(cluster: dict[str, Any]) -> Optional[tuple[float, float, float, float]]:
            pts = np.asarray(cluster["points"], dtype=np.float32)
            if pts.shape[0] < 2:
                return None
            fit = cv2.fitLine(pts.reshape(-1, 1, 2), cv2.DIST_L2, 0, 0.01, 0.01)
            vx = float(fit[0][0])
            vy = float(fit[1][0])
            x0 = float(fit[2][0])
            y0 = float(fit[3][0])
            length = math.hypot(vx, vy)
            if length < 1e-9:
                return None
            vx /= length
            vy /= length
            if vx < 0.0 or (abs(vx) < 1e-9 and vy < 0.0):
                vx = -vx
                vy = -vy
            return x0, y0, vx, vy

        primary_fit = fit_cluster(primary)
        secondary_fit = fit_cluster(best_secondary)
        if primary_fit is None or secondary_fit is None:
            return CornerMetrics()

        p_x, p_y, p_vx, p_vy = primary_fit
        s_x, s_y, s_vx, s_vy = secondary_fit
        valid_intersection, ix, iy = self._line_intersection_px((p_x, p_y), (p_vx, p_vy), (s_x, s_y), (s_vx, s_vy))
        if valid_intersection:
            margin = max(8.0, float(search_px) * 0.20)
            roi_h, roi_w = roi.shape[:2]
            valid_intersection = (-margin <= ix <= float(roi_w) + margin and -margin <= iy <= float(roi_h) + margin)

        strength = float(best_secondary["length"]) / max(1.0, float(primary["length"]))
        min_secondary_length = max(24.0, float(search_px) * 0.18)
        candidate = (
            best_separation >= corner_angle_min
            and best_separation <= 135.0
            and float(best_secondary["length"]) >= min_secondary_length
            and strength >= 0.25
        )
        message = (
            f"secondary line {best_separation:.0f}° from primary, strength {strength:.2f}"
            if float(best_secondary["length"]) >= min_secondary_length
            else ""
        )
        if candidate and valid_intersection:
            message = f"{message}, intersection fitted"
        elif candidate:
            message = f"{message}, no safe intersection"

        return CornerMetrics(
            candidate=bool(candidate),
            angle_degrees=float(best_separation),
            strength=float(strength),
            message=message,
            primary_x=float(p_x),
            primary_y=float(p_y),
            primary_vx=float(p_vx),
            primary_vy=float(p_vy),
            secondary_x=float(s_x),
            secondary_y=float(s_y),
            secondary_vx=float(s_vx),
            secondary_vy=float(s_vy),
            intersection_x=float(ix),
            intersection_y=float(iy),
            intersection_valid=bool(valid_intersection),
        )

    def detect_line(self) -> LineDetection:
        if self.current_frame_bgr is None:
            return LineDetection(False, mode=self._get_line_mode(), message="No camera frame yet")
        frame = self.get_transformed_frame_bgr(self.current_frame_bgr)
        return self.detect_line_in_frame(frame)

    def detect_line_in_frame(self, frame_bgr: np.ndarray) -> LineDetection:
        mode = self._get_line_mode()
        mask = self._make_mask(frame_bgr)
        h, w = mask.shape[:2]
        x1, y1, x2, y2 = self._search_box_bounds(w, h)
        roi = mask[y1:y2, x1:x2]
        if roi.size <= 0:
            return LineDetection(False, mode=mode, message="Line search box is empty")

        corner_metrics = CornerMetrics()
        if self._corner_detection_enabled():
            corner_metrics = self._line_corner_metrics(
                roi,
                search_px=self._get_line_search_px(),
                corner_angle_min=self._get_follow_corner_angle(normalize=False),
            )

        points: Optional[np.ndarray] = None
        score_hint = 0.0
        continuity_message = ""
        if mode == "Edge near center":
            edges = cv2.Canny(roi, 50, 150)
            contours, _hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            local_cx = (x2 - x1) / 2.0
            local_cy = (y2 - y1) / 2.0
            best_contour: Optional[np.ndarray] = None
            best_score = -1.0
            best_continuity = ""
            for contour in contours:
                if len(contour) < 8:
                    continue
                pts = contour.reshape(-1, 2).astype(np.float32)
                min_dist = float(np.min(np.hypot(pts[:, 0] - local_cx, pts[:, 1] - local_cy)))
                span_x = float(np.max(pts[:, 0]) - np.min(pts[:, 0])) if len(pts) else 0.0
                span_y = float(np.max(pts[:, 1]) - np.min(pts[:, 1])) if len(pts) else 0.0
                span = math.hypot(span_x, span_y)
                score = span - (0.45 * min_dist)
                pts_global = pts.copy()
                pts_global[:, 0] += float(x1)
                pts_global[:, 1] += float(y1)
                score, note = self._score_with_follow_continuity(
                    score,
                    pts_global,
                    center_x=w / 2.0,
                    center_y=h / 2.0,
                    search_px=self._get_line_search_px(),
                )
                if score > best_score:
                    best_score = score
                    best_contour = contour
                    best_continuity = note
            if best_contour is not None:
                points = best_contour.reshape(-1, 2).astype(np.float32)
                score_hint = max(0.0, best_score)
                continuity_message = best_continuity
        else:
            contours, _hierarchy = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            roi_area = float(max(1, roi.shape[0] * roi.shape[1]))
            local_cx = (x2 - x1) / 2.0
            local_cy = (y2 - y1) / 2.0
            best_contour = None
            best_score = -1.0
            best_continuity = ""
            for contour in contours:
                area = float(cv2.contourArea(contour))
                if area < 8.0 or area > roi_area * 0.90:
                    continue
                pts = contour.reshape(-1, 2).astype(np.float32)
                if len(pts) < 5:
                    continue
                mean_x = float(np.mean(pts[:, 0]))
                mean_y = float(np.mean(pts[:, 1]))
                distance_score = max(0.0, self._get_line_search_px() * 0.75 - math.hypot(mean_x - local_cx, mean_y - local_cy))
                score = area + (2.0 * distance_score)
                pts_global = pts.copy()
                pts_global[:, 0] += float(x1)
                pts_global[:, 1] += float(y1)
                score, note = self._score_with_follow_continuity(
                    score,
                    pts_global,
                    center_x=w / 2.0,
                    center_y=h / 2.0,
                    search_px=self._get_line_search_px(),
                )
                if score > best_score:
                    best_score = score
                    best_contour = contour
                    best_continuity = note
            if best_contour is not None:
                points = best_contour.reshape(-1, 2).astype(np.float32)
                score_hint = max(0.0, best_score)
                continuity_message = best_continuity

        if points is None or len(points) < 5:
            return LineDetection(False, mode=mode, message="No usable line/edge found in search box")

        points_global = points.copy()
        points_global[:, 0] += float(x1)
        points_global[:, 1] += float(y1)
        fit = cv2.fitLine(points_global.reshape(-1, 1, 2), cv2.DIST_L2, 0, 0.01, 0.01)
        vx = float(fit[0][0])
        vy = float(fit[1][0])
        x0 = float(fit[2][0])
        y0 = float(fit[3][0])
        length = math.hypot(vx, vy)
        if length < 1e-9:
            return LineDetection(False, mode=mode, message="Line fit failed")
        vx /= length
        vy /= length
        # cv2.fitLine direction is mathematically sign-ambiguous. Force a
        # stable screen direction so repeated Follow Step clicks do not randomly
        # alternate between forward and reverse.
        if vx < 0.0 or (abs(vx) < 1e-9 and vy < 0.0):
            vx = -vx
            vy = -vy

        center_x = w / 2.0
        center_y = h / 2.0
        t = ((center_x - x0) * vx) + ((center_y - y0) * vy)
        closest_x = x0 + (t * vx)
        closest_y = y0 + (t * vy)
        err_x = closest_x - center_x
        err_y = closest_y - center_y

        projections = ((points_global[:, 0] - x0) * vx) + ((points_global[:, 1] - y0) * vy)
        span = float(np.max(projections) - np.min(projections)) if len(projections) else 0.0
        perpendicular = np.abs((points_global[:, 0] - x0) * (-vy) + (points_global[:, 1] - y0) * vx)
        width_est = float(np.percentile(perpendicular, 90)) if len(perpendicular) else 0.0
        aspect_score = 0.0
        if width_est > 1e-6:
            aspect_score = min(1.0, span / (width_est * 8.0))
        span_score = min(1.0, span / max(1.0, self._get_line_search_px() * 0.55))
        center_score = max(0.0, 1.0 - (math.hypot(err_x, err_y) / max(1.0, self._get_line_search_px() * 0.50)))
        point_score = min(1.0, math.sqrt(len(points_global)) / 35.0)
        raw_conf = (0.35 * span_score) + (0.25 * aspect_score) + (0.25 * point_score) + (0.15 * center_score)
        if score_hint <= 0.0:
            raw_conf *= 0.85
        confidence = max(0.0, min(100.0, raw_conf * 100.0))
        angle = math.degrees(math.atan2(vy, vx))
        return LineDetection(
            found=True,
            mode=mode,
            x=closest_x,
            y=closest_y,
            vx=vx,
            vy=vy,
            pixel_error_x=err_x,
            pixel_error_y=err_y,
            angle_degrees=angle,
            confidence=confidence,
            span_px=span,
            width_px=width_est * 2.0,
            point_count=int(len(points_global)),
            search_px=self._get_line_search_px(),
            continuity_message=continuity_message,
            corner_candidate=corner_metrics.candidate,
            corner_angle_degrees=corner_metrics.angle_degrees,
            corner_strength=corner_metrics.strength,
            corner_message=corner_metrics.message,
            corner_primary_vx=corner_metrics.primary_vx,
            corner_primary_vy=corner_metrics.primary_vy,
            corner_secondary_vx=corner_metrics.secondary_vx,
            corner_secondary_vy=corner_metrics.secondary_vy,
            corner_primary_x=corner_metrics.primary_x + float(x1),
            corner_primary_y=corner_metrics.primary_y + float(y1),
            corner_secondary_x=corner_metrics.secondary_x + float(x1),
            corner_secondary_y=corner_metrics.secondary_y + float(y1),
            corner_intersection_x=corner_metrics.intersection_x + float(x1),
            corner_intersection_y=corner_metrics.intersection_y + float(y1),
            corner_intersection_valid=corner_metrics.intersection_valid,
            message="Line/edge found",
        )

    def _draw_line_overlay(self, draw: ImageDraw.ImageDraw, scale: float, source_w: int, source_h: int) -> None:
        x1, y1, x2, y2 = self._search_box_bounds(source_w, source_h)
        sx1 = int(round(x1 * scale))
        sy1 = int(round(y1 * scale))
        sx2 = int(round(x2 * scale))
        sy2 = int(round(y2 * scale))
        outline = (0, 0, 0)
        box_color = (0, 190, 255)
        line_color = (255, 80, 255)
        point_color = (0, 255, 255)
        draw.rectangle((sx1, sy1, sx2, sy2), outline=outline, width=4)
        draw.rectangle((sx1, sy1, sx2, sy2), outline=box_color, width=2)

        if not self.current_line.found:
            return

        cx = int(round((source_w / 2.0) * scale))
        cy = int(round((source_h / 2.0) * scale))
        px = int(round(self.current_line.x * scale))
        py = int(round(self.current_line.y * scale))
        span = max(source_w, source_h)
        lx1 = int(round((self.current_line.x - self.current_line.vx * span) * scale))
        ly1 = int(round((self.current_line.y - self.current_line.vy * span) * scale))
        lx2 = int(round((self.current_line.x + self.current_line.vx * span) * scale))
        ly2 = int(round((self.current_line.y + self.current_line.vy * span) * scale))
        draw.line((lx1, ly1, lx2, ly2), fill=outline, width=6)
        draw.line((lx1, ly1, lx2, ly2), fill=line_color, width=3)
        draw.line((cx, cy, px, py), fill=outline, width=5)
        draw.line((cx, cy, px, py), fill=point_color, width=2)
        rr = 8
        draw.ellipse((px - rr, py - rr, px + rr, py + rr), outline=outline, width=4)
        draw.ellipse((px - rr, py - rr, px + rr, py + rr), outline=point_color, width=2)
        if self.current_line.corner_intersection_valid:
            ix = int(round(self.current_line.corner_intersection_x * scale))
            iy = int(round(self.current_line.corner_intersection_y * scale))
            cr = 7
            corner_color = (255, 200, 0)
            draw.line((ix - cr, iy, ix + cr, iy), fill=outline, width=5)
            draw.line((ix, iy - cr, ix, iy + cr), fill=outline, width=5)
            draw.line((ix - cr, iy, ix + cr, iy), fill=corner_color, width=2)
            draw.line((ix, iy - cr, ix, iy + cr), fill=corner_color, width=2)

    def _update_line_status(self, frame_w: int, frame_h: int) -> None:
        if not bool(self.show_line_preview_var.get()):
            self.line_status_var.set("Line/edge: preview disabled")
            return
        line = self.current_line
        if not line.found:
            self.line_status_var.set(f"Line/edge: not found - {line.message}")
            return

        correction = self._machine_correction_from_pixel_error(line.pixel_error_x, line.pixel_error_y)
        if correction is None:
            correction_text = "calibration not available"
        else:
            move_x, move_y = correction
            correction_text = f"suggested correction X{move_x:+.4f} Y{move_y:+.4f}"
        quality_text = (
            f"span {line.span_px:.0f}px | width {line.width_px:.1f}px | "
            f"points {line.point_count} | search {line.search_px}px"
        )
        if line.corner_message:
            corner_label = "corner" if line.corner_candidate else "2nd dir"
            quality_text += f" | {corner_label} {line.corner_angle_degrees:.0f}°/{line.corner_strength:.2f}"
            if line.corner_intersection_valid:
                quality_text += (
                    f" | ix {line.corner_intersection_x - (frame_w / 2.0):+.0f}px "
                    f"iy {line.corner_intersection_y - (frame_h / 2.0):+.0f}px"
                )
        if line.continuity_message:
            quality_text += f" | {line.continuity_message}"
        self.line_status_var.set(
            f"{line.mode}: offset X{line.pixel_error_x:+.1f}px Y{line.pixel_error_y:+.1f}px | "
            f"angle {line.angle_degrees:+.1f}° | confidence {line.confidence:.0f}% | "
            f"{quality_text} | {correction_text}"
        )

    @staticmethod
    def _normalize_unit_vector(x: float, y: float) -> Optional[tuple[float, float]]:
        length = math.hypot(x, y)
        if length < 1e-9 or not math.isfinite(length):
            return None
        return x / length, y / length

    def _stabilize_follow_line(self, line: LineDetection, *, frame_w: int, frame_h: int) -> LineDetection:
        """EMA-filter the accepted follow detection.

        This is intentionally a modest first stage before trying Lucas-Kanade,
        Kalman, or PID. The detector still decides what the line is; this layer
        only stops one noisy frame from immediately becoming motion.
        """

        if not self._get_follow_stabilize_enabled() or not line.found:
            return line

        raw_err = (float(line.pixel_error_x), float(line.pixel_error_y))
        raw_vec = self._normalize_unit_vector(float(line.vx), float(line.vy))
        if raw_vec is None:
            return line
        raw_vec, signed_to_latch = self._orient_pixel_vector_to_follow_heading(raw_vec)

        offset_alpha = self._get_follow_filter_offset_alpha()
        angle_alpha = self._get_follow_filter_angle_alpha()

        previous_err = self._follow_filtered_err
        if previous_err is None:
            filtered_err = raw_err
        else:
            filtered_err = (
                previous_err[0] + ((raw_err[0] - previous_err[0]) * offset_alpha),
                previous_err[1] + ((raw_err[1] - previous_err[1]) * offset_alpha),
            )

        previous_vec = self._follow_filtered_vec_px
        if previous_vec is None:
            filtered_vec = raw_vec
        else:
            # Fit-line vectors are sign-ambiguous. Flip the raw vector before
            # averaging so +179/-1 style cases do not cancel to zero.
            dot = (previous_vec[0] * raw_vec[0]) + (previous_vec[1] * raw_vec[1])
            if dot < 0.0:
                raw_vec = (-raw_vec[0], -raw_vec[1])
            candidate = (
                previous_vec[0] + ((raw_vec[0] - previous_vec[0]) * angle_alpha),
                previous_vec[1] + ((raw_vec[1] - previous_vec[1]) * angle_alpha),
            )
            normalized = self._normalize_unit_vector(candidate[0], candidate[1])
            filtered_vec = normalized if normalized is not None else raw_vec

        self._follow_filtered_err = filtered_err
        self._follow_filtered_vec_px = filtered_vec

        closest_x = (frame_w / 2.0) + filtered_err[0]
        closest_y = (frame_h / 2.0) + filtered_err[1]
        filtered_angle = math.degrees(math.atan2(filtered_vec[1], filtered_vec[0]))
        continuity_bits = ["filtered"]
        if signed_to_latch:
            continuity_bits.append("signed to latch")
        if line.continuity_message:
            continuity_bits.append(line.continuity_message)
        continuity = "; ".join(continuity_bits)
        return replace(
            line,
            x=closest_x,
            y=closest_y,
            vx=filtered_vec[0],
            vy=filtered_vec[1],
            pixel_error_x=filtered_err[0],
            pixel_error_y=filtered_err[1],
            angle_degrees=filtered_angle,
            stabilized=True,
            filtered_pixel_error_x=filtered_err[0],
            filtered_pixel_error_y=filtered_err[1],
            filtered_angle_degrees=filtered_angle,
            continuity_message=continuity,
        )

    def _heading_change_from_line_to_latch(self, line: LineDetection) -> Optional[float]:
        previous = self._follow_heading_unit
        if previous is None or not line.found:
            return None
        candidates = self._machine_heading_candidates_from_line(line, include_corner_directions=False)
        best: Optional[float] = None
        for unit_x, unit_y, _label in candidates:
            # Lines are not arrows. Compare both signs and keep the nearest
            # direction to the current machine-space latch.
            diff = min(
                self._angle_between_unit_vectors(previous, (unit_x, unit_y)),
                self._angle_between_unit_vectors(previous, (-unit_x, -unit_y)),
            )
            if best is None or diff < best:
                best = diff
        return best

    def _follow_detection_sanity_ok(self, line: LineDetection, *, step_label: str) -> bool:
        if not self._get_follow_stabilize_enabled():
            return True
        sanity_angle = self._get_follow_sanity_angle()
        if sanity_angle <= 0.0 or self._follow_heading_unit is None or line.corner_candidate:
            return True
        heading_change = self._heading_change_from_line_to_latch(line)
        if heading_change is None or heading_change <= sanity_angle:
            return True
        self.cal_status_var.set(
            f"{step_label} refused: detection heading jumped {heading_change:.0f}° from the latched heading; "
            f"sanity limit is {sanity_angle:.0f}°. Treating this frame as not-found instead of following a possible wrong feature."
        )
        return False

    def _machine_correction_from_pixel_error(self, err_x: float, err_y: float) -> Optional[tuple[float, float]]:
        calibration = self._validate_calibration(self.active_calibration)
        if calibration is None:
            return None
        try:
            inv = calibration["matrix_pixel_to_machine"]
            move_x = float(inv[0][0]) * (-err_x) + float(inv[0][1]) * (-err_y)
            move_y = float(inv[1][0]) * (-err_x) + float(inv[1][1]) * (-err_y)
        except Exception:  # noqa: BLE001
            return None
        if not math.isfinite(move_x) or not math.isfinite(move_y):
            return None
        return move_x, move_y

    def _machine_vector_from_pixel_vector(self, px_x: float, px_y: float) -> Optional[tuple[float, float]]:
        calibration = self._validate_calibration(self.active_calibration)
        if calibration is None:
            return None
        try:
            inv = calibration["matrix_pixel_to_machine"]
            move_x = float(inv[0][0]) * float(px_x) + float(inv[0][1]) * float(px_y)
            move_y = float(inv[1][0]) * float(px_x) + float(inv[1][1]) * float(px_y)
        except Exception:  # noqa: BLE001
            return None
        if not math.isfinite(move_x) or not math.isfinite(move_y):
            return None
        return move_x, move_y

    def _pixel_vector_from_machine_vector(self, machine_x: float, machine_y: float) -> Optional[tuple[float, float]]:
        calibration = self._validate_calibration(self.active_calibration)
        if calibration is None:
            return None
        try:
            matrix = calibration["matrix_machine_to_pixel"]
            px_x = float(matrix[0][0]) * float(machine_x) + float(matrix[0][1]) * float(machine_y)
            px_y = float(matrix[1][0]) * float(machine_x) + float(matrix[1][1]) * float(machine_y)
        except Exception:  # noqa: BLE001
            return None
        if not math.isfinite(px_x) or not math.isfinite(px_y):
            return None
        return px_x, px_y

    def _orient_pixel_vector_to_follow_heading(
        self,
        pixel_vec: tuple[float, float],
    ) -> tuple[tuple[float, float], bool]:
        """Flip a sign-ambiguous fitted pixel tangent to match travel direction.

        cv2.fitLine returns an axis, not an arrow. v0.5.24 resolves that
        ambiguity as early as possible by comparing the candidate pixel vector
        in machine space against the currently latched travel heading. This is
        especially important on circles/radii where a 180-degree sign flip can
        look like a valid line fit but makes the commanded motion reverse.
        """

        heading = self._follow_heading_unit
        if heading is None:
            return pixel_vec, False
        machine_vec = self._machine_vector_from_pixel_vector(pixel_vec[0], pixel_vec[1])
        if machine_vec is None:
            return pixel_vec, False
        machine_unit = self._normalize_unit_vector(machine_vec[0], machine_vec[1])
        heading_unit = self._normalize_unit_vector(heading[0], heading[1])
        if machine_unit is None or heading_unit is None:
            return pixel_vec, False
        dot = (machine_unit[0] * heading_unit[0]) + (machine_unit[1] * heading_unit[1])
        if dot < 0.0:
            return (-pixel_vec[0], -pixel_vec[1]), True
        return pixel_vec, False

    def _limit_move_vector(self, move_x: float, move_y: float, max_len: float) -> tuple[float, float, bool]:
        length = math.hypot(move_x, move_y)
        if max_len <= 0.0:
            return 0.0, 0.0, length > 0.0
        if length > max_len:
            scale = max_len / length
            return move_x * scale, move_y * scale, True
        return move_x, move_y, False

    def find_line_once(self) -> None:
        # Treat Find Line/Edge as the user's explicit setup step for a new follow
        # direction. The next Follow Step will establish a fresh heading from the
        # selected X+/X-/Y+/Y- Start dir, then later steps will latch to it.
        self._clear_follow_heading()
        if self.current_frame_bgr is None:
            messagebox.showinfo("No camera frame", "No camera frame is available yet.", parent=self)
            return
        transformed = self.get_transformed_frame_bgr(self.current_frame_bgr)
        frame_h, frame_w = transformed.shape[:2]
        line = self.detect_line_in_frame(transformed)
        if line.found:
            # Find Line/Edge is the user's explicit re-lock operation. Seed the
            # EMA filter from this accepted detection so the first move starts
            # from a stable known feature instead of carrying stale history.
            self._reset_follow_filter()
            line = self._stabilize_follow_line(line, frame_w=frame_w, frame_h=frame_h)
        self.current_line = line
        if line.found:
            corner_text = ""
            if line.corner_message:
                corner_text = f" Corner: {line.corner_message}."
            resume_text = ""
            if bool(self.follow_corner_pause_var.get()) and line.corner_candidate:
                self._follow_corner_resume_once = True
                resume_text = " Next follow move will bypass corner pause once; choose the next Start dir before moving."
            self.cal_status_var.set(
                f"{line.mode} found. Pixel offset X{line.pixel_error_x:+.1f} Y{line.pixel_error_y:+.1f}; "
                f"confidence {line.confidence:.0f}%.{corner_text}{resume_text}"
            )
        else:
            self._follow_corner_resume_once = False
            self.cal_status_var.set(line.message)
        self._show_current_frame()

    def find_dot_once(self) -> None:
        if self.current_frame_bgr is None:
            messagebox.showinfo("No camera frame", "No camera frame is available yet.", parent=self)
            return
        dot = self.detect_dot()
        self.current_dot = dot
        if dot.found:
            self.cal_status_var.set(
                f"Dot found at X {dot.x:.1f} Y {dot.y:.1f}. Center it reasonably, then Run Calibration."
            )
        else:
            self.cal_status_var.set(dot.message)
        self._show_current_frame()

    def _validate_calibration(self, calibration: Optional[dict[str, Any]]) -> Optional[dict[str, Any]]:
        if not isinstance(calibration, dict):
            return None
        if not calibration.get("valid"):
            return None
        matrix = calibration.get("matrix_pixel_to_machine")
        try:
            if len(matrix) != 2 or len(matrix[0]) != 2 or len(matrix[1]) != 2:
                return None
            values = [float(matrix[0][0]), float(matrix[0][1]), float(matrix[1][0]), float(matrix[1][1])]
            if not all(math.isfinite(value) for value in values):
                return None
        except Exception:  # noqa: BLE001
            return None
        return calibration

    def manual_jog(self, axis: str, direction: int) -> None:
        if self._motion_active:
            messagebox.showinfo("Calibration active", "Wait for calibration to finish or press STOP Move.", parent=self)
            return
        if self._manual_jog_active:
            return

        axis = axis.upper().strip()
        if axis not in {"X", "Y"}:
            return
        direction = 1 if direction >= 0 else -1
        step = self._get_jog_step()
        feed = self._get_feed()
        coordinate_mode = self.coordinate_mode_label

        status = self.linuxcnc_reader.read_status()
        if not self._status_ok_for_calibration(status):
            messagebox.showerror("LinuxCNC not ready", status.error or self._status_not_ready_message(status), parent=self)
            return
        start_x, start_y, _z = self._active_position(status)
        target_x = start_x + (step * direction if axis == "X" else 0.0)
        target_y = start_y + (step * direction if axis == "Y" else 0.0)

        self._manual_jog_active = True
        try:
            jog = self.linuxcnc_reader.incremental_jog(axis, direction, step, feed)
            if not jog.success:
                messagebox.showerror("Jog failed", jog.message, parent=self)
                self.cal_status_var.set(jog.message)
                return

            label = f"{axis}{'+' if direction > 0 else '-'}"
            self.cal_status_var.set(f"Manual calibration jog: {label} {step:.4f} at {feed:.1f} units/min.")
            self._wait_for_position_near(target_x, target_y, coordinate_mode, step)
            self._wait_and_pump_camera(0.12)
            self._show_current_frame()
        finally:
            self._manual_jog_active = False

    def run_calibration(self) -> None:
        if self._motion_active:
            messagebox.showinfo("Motion already active", "FabScan is already running a calibration move.", parent=self)
            return
        if self.current_frame_bgr is None:
            messagebox.showinfo("No camera frame", "Open the camera and find the dot first.", parent=self)
            return

        move_distance = self._get_move_distance()
        feed = self._get_feed()
        coordinate_mode = self.coordinate_mode_label

        start_dot = self.detect_dot()
        if not start_dot.found:
            messagebox.showinfo("Dot not found", start_dot.message, parent=self)
            return

        status = self.linuxcnc_reader.read_status()
        if not self._status_ok_for_calibration(status):
            messagebox.showerror("LinuxCNC not ready", status.error or self._status_not_ready_message(status), parent=self)
            return
        start_x, start_y, _z = self._active_position(status)

        self.cal_status_var.set(
            f"Calibration starting: {coordinate_mode}, move {move_distance:.4f}, feed {feed:.1f} units/min."
        )

        self._motion_active = True
        try:
            self.cal_status_var.set("Calibration: jogging X+ and looking for the same dot...")
            x_dot = self._move_find_dot_return(
                target_x=start_x + move_distance,
                target_y=start_y,
                return_x=start_x,
                return_y=start_y,
                feed=feed,
                coordinate_mode=coordinate_mode,
                label="X+",
            )
            if x_dot is None:
                return

            self.cal_status_var.set("Calibration: jogging Y+ and looking for the same dot...")
            y_dot = self._move_find_dot_return(
                target_x=start_x,
                target_y=start_y + move_distance,
                return_x=start_x,
                return_y=start_y,
                feed=feed,
                coordinate_mode=coordinate_mode,
                label="Y+",
            )
            if y_dot is None:
                return

            try:
                calibration = self._build_calibration_result(
                    start_dot=start_dot,
                    x_dot=x_dot,
                    y_dot=y_dot,
                    move_distance=move_distance,
                    feed=feed,
                    coordinate_mode=coordinate_mode,
                    start_x=start_x,
                    start_y=start_y,
                )
            except RuntimeError as exc:
                self.cal_status_var.set(f"Calibration failed: {exc}")
                messagebox.showerror("Calibration failed", str(exc), parent=self)
                return
            self.active_calibration = calibration
            self.result = self._make_result(calibration=calibration)
            self._show_calibration_summary(calibration)
        finally:
            self._motion_active = False
        self._manual_jog_active = False

    def _move_find_dot_return(
        self,
        *,
        target_x: float,
        target_y: float,
        return_x: float,
        return_y: float,
        feed: float,
        coordinate_mode: str,
        label: str,
    ) -> Optional[DotDetection]:
        """Jog one calibration axis, find the dot, then jog back.

        v0.5.0 used repeated MDI G1 moves here. That worked for some moves but
        could race QtPlasmaC/LinuxCNC mode changes and trigger "Must be in MDI
        mode to issue MDI command." Calibration is a relative motion test, so
        the already-proven MANUAL-mode incremental jog path is a better fit.
        """

        axis, direction, distance = self._axis_direction_distance(
            target_x=target_x,
            target_y=target_y,
            return_x=return_x,
            return_y=return_y,
        )
        if axis is None:
            self.cal_status_var.set("Calibration failed: no X/Y calibration move was requested.")
            return None

        jog = self.linuxcnc_reader.incremental_jog(axis, direction, distance, feed)
        if not jog.success:
            messagebox.showerror("Calibration jog failed", jog.message, parent=self)
            self.cal_status_var.set(jog.message)
            return None

        self.cal_status_var.set(f"Calibration: {label} jog sent. Waiting for position to settle...")
        if not self._wait_for_position_near(target_x, target_y, coordinate_mode, distance):
            self.cal_status_var.set(f"Calibration failed: LinuxCNC did not reach/settle after {label} jog.")
            self._return_to_start(axis, -direction, distance, feed, return_x, return_y, coordinate_mode)
            return None

        self._wait_and_pump_camera(0.35)
        dot = self.detect_dot()
        if not dot.found:
            self.cal_status_var.set(f"Calibration failed: dot left frame or was not found after {label} jog.")
            messagebox.showerror(
                "Dot lost",
                (
                    f"Dot was not found after the {label} calibration jog.\n\n"
                    "Use a smaller calibration move, better lighting, or re-center the dot. "
                    "FabScan will try to jog back to the start point."
                ),
                parent=self,
            )
            self._return_to_start(axis, -direction, distance, feed, return_x, return_y, coordinate_mode)
            return None

        self.cal_status_var.set(f"Calibration: dot found after {label}. Jogging back to start...")
        if not self._return_to_start(axis, -direction, distance, feed, return_x, return_y, coordinate_mode):
            return None
        return dot

    def _axis_direction_distance(
        self,
        *,
        target_x: float,
        target_y: float,
        return_x: float,
        return_y: float,
    ) -> tuple[Optional[str], int, float]:
        dx = float(target_x) - float(return_x)
        dy = float(target_y) - float(return_y)
        if abs(dx) >= abs(dy) and abs(dx) > 1e-9:
            return "X", (1 if dx >= 0.0 else -1), abs(dx)
        if abs(dy) > 1e-9:
            return "Y", (1 if dy >= 0.0 else -1), abs(dy)
        return None, 1, 0.0

    def _return_to_start(
        self,
        axis: str,
        direction: int,
        distance: float,
        feed: float,
        start_x: float,
        start_y: float,
        coordinate_mode: str,
    ) -> bool:
        jog = self.linuxcnc_reader.incremental_jog(axis, direction, distance, feed)
        if not jog.success:
            messagebox.showerror("Return jog failed", jog.message, parent=self)
            self.cal_status_var.set(jog.message)
            return False
        if not self._wait_for_position_near(start_x, start_y, coordinate_mode, distance):
            self.cal_status_var.set("Return jog was sent, but FabScan did not see the expected start position settle.")
            return False
        return True

    def _record_position_history(self, status: LinuxCNCPositionStatus, *, source: str, timestamp_s: Optional[float] = None) -> Optional[PositionHistorySample]:
        if not getattr(status, "connected", False):
            return None
        try:
            x, y, z = self._active_position(status)
        except Exception:
            return None
        now = time.monotonic() if timestamp_s is None else float(timestamp_s)
        sample = PositionHistorySample(now, float(x), float(y), float(z), source)
        self._position_history.append(sample)
        cutoff = now - self._position_history_window_s
        while self._position_history and self._position_history[0].timestamp_s < cutoff:
            self._position_history.pop(0)
        return sample

    def _lookup_position_history(self, target_time_s: float) -> Optional[PositionHistorySample]:
        samples = self._position_history
        if not samples:
            return None
        if target_time_s <= samples[0].timestamp_s:
            return samples[0]
        if target_time_s >= samples[-1].timestamp_s:
            return samples[-1]

        prev = samples[0]
        for nxt in samples[1:]:
            if nxt.timestamp_s >= target_time_s:
                span = nxt.timestamp_s - prev.timestamp_s
                if span <= 1e-9:
                    return prev
                ratio = (target_time_s - prev.timestamp_s) / span
                return PositionHistorySample(
                    target_time_s,
                    prev.x + ((nxt.x - prev.x) * ratio),
                    prev.y + ((nxt.y - prev.y) * ratio),
                    prev.z + ((nxt.z - prev.z) * ratio),
                    f"interp:{prev.source}->{nxt.source}",
                )
            prev = nxt
        return samples[-1]

    def _delayed_position_plan_fields(
        self,
        *,
        current_x: float,
        current_y: float,
        frame_move_x: float,
        frame_move_y: float,
    ) -> tuple[float, float, dict[str, Any]]:
        delay_ms = self._get_follow_position_delay_ms()
        use_delayed = bool(self.follow_use_delayed_position_var.get())
        fields: dict[str, Any] = {
            "use_delayed_position": use_delayed,
            "position_delay_ms": delay_ms,
            "frame_move_x": f"{frame_move_x:.6f}",
            "frame_move_y": f"{frame_move_y:.6f}",
            "target_base_x": f"{current_x:.6f}",
            "target_base_y": f"{current_y:.6f}",
            "command_adjust_x": "0.000000",
            "command_adjust_y": "0.000000",
        }
        if not use_delayed:
            return current_x, current_y, fields

        frame_ts = float(getattr(self, "current_frame_timestamp", 0.0) or 0.0)
        if frame_ts <= 0.0:
            fields["delayed_position_found"] = False
            fields["delayed_position_source"] = "no frame timestamp"
            return current_x, current_y, fields

        target_time = frame_ts - (delay_ms / 1000.0)
        fields["frame_target_time_s"] = f"{target_time:.6f}"
        sample = self._lookup_position_history(target_time)
        if sample is None:
            fields["delayed_position_found"] = False
            fields["delayed_position_source"] = "no position history"
            return current_x, current_y, fields

        now = time.monotonic()
        fields.update(
            {
                "delayed_position_found": True,
                "delayed_position_source": sample.source,
                "delayed_position_age_ms": f"{(now - sample.timestamp_s) * 1000.0:.3f}",
                "delayed_position_error_ms": f"{(sample.timestamp_s - target_time) * 1000.0:.3f}",
                "delayed_x": f"{sample.x:.6f}",
                "delayed_y": f"{sample.y:.6f}",
                "target_base_x": f"{sample.x:.6f}",
                "target_base_y": f"{sample.y:.6f}",
                "command_adjust_x": f"{sample.x - current_x:.6f}",
                "command_adjust_y": f"{sample.y - current_y:.6f}",
            }
        )
        return sample.x, sample.y, fields

    def _virtual_target_default_fields(self) -> dict[str, Any]:
        return {
            "use_virtual_target": bool(self._get_follow_virtual_target_enabled()),
            "virtual_min_progress_pct": f"{self._get_follow_virtual_min_progress_pct():.3f}",
        }

    def _apply_follow_virtual_target(
        self,
        *,
        current_x: float,
        current_y: float,
        desired_target_x: float,
        desired_target_y: float,
        heading: tuple[float, float],
        follow_step: float,
        max_correct: float,
    ) -> tuple[float, float, float, float, dict[str, Any]]:
        """Stage 2D-lite virtual-target command shaping.

        This does not send velocity jogs and does not allow an unbounded
        controller to chase the camera. It keeps a small virtual target in the
        same coordinate space as LinuxCNC, advances that target toward the
        camera-derived desired target, then clamps the real command into
        tangent/perpendicular components. The important first safety rule is
        that delay compensation may help steering, but it may not cancel most
        of the forward progress.
        """

        fields = self._virtual_target_default_fields()
        if not self._get_follow_virtual_target_enabled():
            fields["virtual_state"] = "disabled"
            return desired_target_x, desired_target_y, desired_target_x - current_x, desired_target_y - current_y, fields

        unit = self._normalize_unit_vector(heading[0], heading[1])
        if unit is None:
            fields["virtual_state"] = "no heading"
            return desired_target_x, desired_target_y, desired_target_x - current_x, desired_target_y - current_y, fields

        prev_target = self._follow_virtual_target_xy
        if prev_target is None:
            prev_target = (current_x, current_y)

        desired_delta_x = desired_target_x - prev_target[0]
        desired_delta_y = desired_target_y - prev_target[1]
        max_target_advance = max(0.001, follow_step + max_correct)
        target_delta_x, target_delta_y, target_limited = self._limit_move_vector(
            desired_delta_x,
            desired_delta_y,
            max_target_advance,
        )
        virtual_x = prev_target[0] + target_delta_x
        virtual_y = prev_target[1] + target_delta_y

        raw_move_x = virtual_x - current_x
        raw_move_y = virtual_y - current_y
        min_forward = follow_step * (self._get_follow_virtual_min_progress_pct() / 100.0)
        max_forward = max(0.001, follow_step + max_correct)
        max_side = max(0.0, max_correct)
        move_x, move_y, raw_parallel, raw_perp, final_parallel, final_perp = self._clamp_parallel_perp_move(
            move_x=raw_move_x,
            move_y=raw_move_y,
            heading=unit,
            min_forward=min_forward,
            max_forward=max_forward,
            max_side=max_side,
        )
        move_x, move_y, total_limited = self._limit_move_vector(move_x, move_y, max_forward)
        # If total length limiting scaled the vector, report the actual final
        # tangent/side components that will be commanded.
        hx, hy = unit
        nx, ny = -hy, hx
        final_parallel = (move_x * hx) + (move_y * hy)
        final_perp = (move_x * nx) + (move_y * ny)
        target_x = current_x + move_x
        target_y = current_y + move_y
        self._follow_virtual_target_xy = (target_x, target_y)

        state_bits = ["active"]
        if target_limited:
            state_bits.append("target limited")
        if raw_parallel < min_forward:
            state_bits.append("forward floor")
        if abs(raw_perp) > max_side:
            state_bits.append("side limited")
        if total_limited:
            state_bits.append("total limited")

        fields.update(
            {
                "virtual_state": "; ".join(state_bits),
                "virtual_desired_x": f"{desired_target_x:.6f}",
                "virtual_desired_y": f"{desired_target_y:.6f}",
                "virtual_target_x": f"{target_x:.6f}",
                "virtual_target_y": f"{target_y:.6f}",
                "virtual_raw_parallel": f"{raw_parallel:.6f}",
                "virtual_raw_perp": f"{raw_perp:.6f}",
                "virtual_final_parallel": f"{final_parallel:.6f}",
                "virtual_final_perp": f"{final_perp:.6f}",
                "virtual_min_forward": f"{min_forward:.6f}",
                "virtual_side_limit": f"{max_side:.6f}",
            }
        )
        return target_x, target_y, move_x, move_y, fields

    def _wait_for_position_near(
        self,
        target_x: float,
        target_y: float,
        coordinate_mode: str,
        move_distance: float,
    ) -> bool:
        timeout_seconds = max(6.0, (abs(move_distance) / max(0.001, self._get_feed())) * 60.0 * 4.0 + 2.0)
        tolerance = max(0.001, abs(move_distance) * 0.03)
        end_time = time.monotonic() + timeout_seconds
        stable_count = 0
        last_message = ""
        while time.monotonic() < end_time:
            self.update()
            self._pump_camera_frame()
            status = self.linuxcnc_reader.read_status()
            self._record_position_history(status, source="wait_position")
            if status.connected:
                x, y, _z = self._active_position(status)
                error = math.hypot(float(x) - float(target_x), float(y) - float(target_y))
                last_message = (
                    f"pos X{x:.4f} Y{y:.4f}, target X{target_x:.4f} Y{target_y:.4f}, "
                    f"error {error:.5f}, mode {status.task_mode}"
                )
                if error <= tolerance:
                    stable_count += 1
                    if stable_count >= 4:
                        return True
                else:
                    stable_count = 0
            else:
                last_message = status.error or "LinuxCNC not connected"
                stable_count = 0
            time.sleep(0.05)
        self.cal_status_var.set(f"Timed out waiting for calibration jog to settle ({last_message}).")
        return False

    def _wait_for_idle(self, timeout_seconds: float) -> bool:
        end_time = time.monotonic() + timeout_seconds
        last_message = ""
        while time.monotonic() < end_time:
            self.update()
            status = self.linuxcnc_reader.read_status()
            self._record_position_history(status, source="wait_idle")
            if status.connected:
                last_message = f"state {status.task_state}, mode {status.task_mode}, interp {status.interp_state}"
                if status.interp_state == "IDLE":
                    return True
            else:
                last_message = status.error or "LinuxCNC not connected"
            time.sleep(0.05)
        self.cal_status_var.set(f"Timed out waiting for LinuxCNC IDLE ({last_message}).")
        return False

    def _wait_and_pump_camera(self, seconds: float) -> None:
        end_time = time.monotonic() + seconds
        while time.monotonic() < end_time:
            self.update()
            self._pump_camera_frame()
            time.sleep(0.03)

    def _status_ok_for_calibration(self, status: LinuxCNCPositionStatus) -> bool:
        if not status.available or not status.connected:
            return False
        if status.task_state != "ON":
            return False
        if status.interp_state != "IDLE":
            return False
        if status.task_mode != "MANUAL":
            return False
        if not status.all_xyz_homed:
            return False
        return True

    def _status_not_ready_message(self, status: LinuxCNCPositionStatus) -> str:
        if not status.available or not status.connected:
            return status.error or "FabScan is not connected to LinuxCNC."
        if status.task_state != "ON":
            return f"LinuxCNC task state must be ON. Current state: {status.task_state}."
        if status.interp_state != "IDLE":
            return f"LinuxCNC interpreter must be IDLE. Current state: {status.interp_state}."
        if status.task_mode != "MANUAL":
            return (
                "Camera calibration now uses MANUAL-mode incremental jogs, not MDI moves. "
                f"Current mode: {status.task_mode}. Switch QtPlasmaC/LinuxCNC to manual/jog mode."
            )
        if not status.all_xyz_homed:
            return f"X/Y/Z must be homed. Current homed state: {status.homed_text}."
        return "LinuxCNC is not ready for calibration."

    def _active_position(self, status: LinuxCNCPositionStatus) -> tuple[float, float, float]:
        if self.coordinate_mode_label == "Machine coordinates":
            return status.machine_position
        return status.work_position

    def _build_calibration_result(
        self,
        *,
        start_dot: DotDetection,
        x_dot: DotDetection,
        y_dot: DotDetection,
        move_distance: float,
        feed: float,
        coordinate_mode: str,
        start_x: float,
        start_y: float,
    ) -> dict[str, Any]:
        x_response = (x_dot.x - start_dot.x, x_dot.y - start_dot.y)
        y_response = (y_dot.x - start_dot.x, y_dot.y - start_dot.y)

        # Matrix maps machine move [dx, dy] to pixel move [du, dv].
        a = x_response[0] / move_distance
        b = y_response[0] / move_distance
        c = x_response[1] / move_distance
        d = y_response[1] / move_distance
        det = (a * d) - (b * c)
        if abs(det) < 1e-9:
            raise RuntimeError("Calibration matrix is singular. Increase calibration move or improve dot detection.")

        inv = [[d / det, -b / det], [-c / det, a / det]]
        px_per_unit_x = math.hypot(*x_response) / move_distance
        px_per_unit_y = math.hypot(*y_response) / move_distance
        x_angle = math.degrees(math.atan2(x_response[1], x_response[0]))
        y_angle = math.degrees(math.atan2(y_response[1], y_response[0]))

        return {
            "valid": True,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "camera_index": self._get_camera_index(),
            "camera_width": self._get_requested_size()[0],
            "camera_height": self._get_requested_size()[1],
            "rotate_degrees": self._get_rotate_degrees(),
            "flip_x": bool(self.flip_x_var.get()),
            "flip_y": bool(self.flip_y_var.get()),
            "fine_rotation_degrees": self._get_fine_rotation_degrees(),
            "threshold": self._get_threshold(),
            "coordinate_mode_label": coordinate_mode,
            "move_distance": move_distance,
            "feed_units_per_min": feed,
            "start_position": [start_x, start_y],
            "start_dot_px": [start_dot.x, start_dot.y],
            "x_plus_dot_px": [x_dot.x, x_dot.y],
            "y_plus_dot_px": [y_dot.x, y_dot.y],
            "x_plus_pixel_response": [x_response[0], x_response[1]],
            "y_plus_pixel_response": [y_response[0], y_response[1]],
            "matrix_machine_to_pixel": [[a, b], [c, d]],
            "matrix_pixel_to_machine": inv,
            "pixels_per_unit_x": px_per_unit_x,
            "pixels_per_unit_y": px_per_unit_y,
            "x_response_angle_degrees": x_angle,
            "y_response_angle_degrees": y_angle,
            "determinant": det,
        }

    def _show_calibration_summary(self, calibration: dict[str, Any], *, loaded: bool = False) -> None:
        prefix = "Calibration loaded" if loaded else "Calibration valid"
        self.transform_status_var.set(
            f"{prefix}: "
            f"X {calibration['pixels_per_unit_x']:.1f} px/unit, "
            f"Y {calibration['pixels_per_unit_y']:.1f} px/unit, "
            f"X angle {calibration['x_response_angle_degrees']:+.1f}°, "
            f"Y angle {calibration['y_response_angle_degrees']:+.1f}°."
        )
        if loaded:
            self.cal_status_var.set("Saved calibration loaded. Use Center Dot to test the camera/machine transform.")
        else:
            self.cal_status_var.set("Calibration complete. Use Center Dot to test it, or close this window to return to FabScan.")

    def _machine_heading_candidates_from_line(
        self,
        line: LineDetection,
        *,
        include_corner_directions: bool = False,
    ) -> list[tuple[float, float, str]]:
        """Return normalized machine-space heading candidates for a line detection.

        Normal following uses the single cv2.fitLine tangent. At a corner, the
        single fit may be an averaged diagonal. When the user has explicitly
        clicked Find Line/Edge after a corner pause, include the Hough primary
        and secondary directions so the selected Start dir can choose the next
        leg instead of the averaged line.
        """

        pixel_candidates: list[tuple[float, float, str]] = [(line.vx, line.vy, "line fit")]
        if include_corner_directions:
            for vx, vy, label in (
                (line.corner_primary_vx, line.corner_primary_vy, "corner primary"),
                (line.corner_secondary_vx, line.corner_secondary_vy, "corner secondary"),
            ):
                if math.hypot(vx, vy) > 1e-9:
                    # Avoid duplicate candidates that are essentially the same
                    # direction as one already listed.
                    duplicate = False
                    cand_angle = math.degrees(math.atan2(vy, vx)) % 180.0
                    for existing_vx, existing_vy, _existing_label in pixel_candidates:
                        existing_angle = math.degrees(math.atan2(existing_vy, existing_vx)) % 180.0
                        if self._angle_difference_degrees(cand_angle, existing_angle) < 10.0:
                            duplicate = True
                            break
                    if not duplicate:
                        pixel_candidates.append((vx, vy, label))

        machine_candidates: list[tuple[float, float, str]] = []
        for px_x, px_y, label in pixel_candidates:
            tangent = self._machine_vector_from_pixel_vector(px_x, px_y)
            if tangent is None:
                continue
            length = math.hypot(tangent[0], tangent[1])
            if length < 1e-9:
                continue
            machine_candidates.append((tangent[0] / length, tangent[1] / length, label))
        return machine_candidates

    def _choose_latched_follow_heading_from_candidates(
        self,
        candidates: list[tuple[float, float, str]],
        direction_preference: str,
        *,
        ignore_previous: bool = False,
    ) -> tuple[Optional[tuple[float, float]], str]:
        """Resolve the 180-degree ambiguity from one or more heading candidates."""

        if not candidates:
            return None, "no tangent"

        previous = self._follow_last_move_unit or self._follow_heading_unit
        if previous is not None and not ignore_previous:
            prev_len = math.hypot(previous[0], previous[1])
            if prev_len > 1e-9:
                prev_x = previous[0] / prev_len
                prev_y = previous[1] / prev_len
                best: Optional[tuple[float, float, str, float]] = None
                for unit_x, unit_y, label in candidates:
                    dot = (unit_x * prev_x) + (unit_y * prev_y)
                    if dot < 0.0:
                        unit_x = -unit_x
                        unit_y = -unit_y
                        dot = -dot
                    if best is None or dot > best[3]:
                        best = (unit_x, unit_y, label, dot)
                if best is not None:
                    return (best[0], best[1]), f"latched {best[2]}"

        # First step after Find Line/Edge or a setting change: honor the user's
        # deterministic machine-axis start direction. This is less random than
        # Forward/Reverse because the selected sign is tied to X+/X-/Y+/Y-.
        target_x, target_y = self._axis_vector_from_follow_direction(direction_preference)
        best: Optional[tuple[float, float, str, float]] = None
        for unit_x, unit_y, label in candidates:
            projection = (unit_x * target_x) + (unit_y * target_y)
            if projection < 0.0:
                unit_x = -unit_x
                unit_y = -unit_y
                projection = -projection
            if best is None or projection > best[3]:
                best = (unit_x, unit_y, label, projection)
        if best is None:
            return None, "no tangent"
        weak_text = " weak-axis" if best[3] < 0.15 else ""
        return (best[0], best[1]), f"new {direction_preference} {best[2]}{weak_text} heading"

    def _corner_assist_move_from_line(
        self,
        line: LineDetection,
        *,
        frame_w: int,
        frame_h: int,
        follow_step: float,
        direction_preference: str,
    ) -> CornerAssistMove:
        """Calculate a conservative move to the detected corner intersection.

        This is the first real corner-follow assist. When the search ROI sees
        two strong line directions, drive the camera/crosshair to the fitted
        line intersection instead of asking the user to step through the corner
        manually. The move is limited to a short lookahead window so a bad
        intersection cannot command a large blind move.
        """

        if not line.corner_intersection_valid:
            return CornerAssistMove(False, message="no safe line intersection")

        heading_candidates = self._machine_heading_candidates_from_line(line, include_corner_directions=True)
        outgoing_heading, heading_state = self._choose_latched_follow_heading_from_candidates(
            heading_candidates,
            direction_preference,
            ignore_previous=True,
        )
        if outgoing_heading is None:
            return CornerAssistMove(False, message="could not latch outgoing corner direction")

        err_x = line.corner_intersection_x - (frame_w / 2.0)
        err_y = line.corner_intersection_y - (frame_h / 2.0)
        move = self._machine_correction_from_pixel_error(err_x, err_y)
        if move is None:
            return CornerAssistMove(False, message="saved calibration could not convert corner offset")

        move_x, move_y = move
        distance = math.hypot(move_x, move_y)
        if not math.isfinite(distance):
            return CornerAssistMove(False, message="corner move was not finite")

        lookahead_steps = self._get_follow_corner_lookahead_steps()
        max_distance = max(follow_step, follow_step * float(lookahead_steps))
        if distance > max_distance:
            return CornerAssistMove(
                False,
                message=f"corner is {distance:.4f} away; lookahead limit is {max_distance:.4f}",
                distance=distance,
                max_distance=max_distance,
            )
        if distance < 0.0005:
            return CornerAssistMove(
                False,
                message="already at/too near the calculated corner",
                heading=outgoing_heading,
                heading_state=heading_state,
                distance=distance,
                max_distance=max_distance,
            )

        return CornerAssistMove(
            True,
            move_x=move_x,
            move_y=move_y,
            heading=outgoing_heading,
            heading_state=heading_state,
            message=(
                f"intersection X{line.corner_intersection_x - (frame_w / 2.0):+.0f}px "
                f"Y{line.corner_intersection_y - (frame_h / 2.0):+.0f}px"
            ),
            distance=distance,
            max_distance=max_distance,
        )

    @staticmethod
    def _angle_between_unit_vectors(a: tuple[float, float], b: tuple[float, float]) -> float:
        dot = (a[0] * b[0]) + (a[1] * b[1])
        dot = max(-1.0, min(1.0, dot))
        return math.degrees(math.acos(dot))

    def _follow_heading_change_degrees(self, heading: tuple[float, float]) -> Optional[float]:
        previous = self._follow_heading_unit
        if previous is None:
            return None
        prev_len = math.hypot(previous[0], previous[1])
        head_len = math.hypot(heading[0], heading[1])
        if prev_len < 1e-9 or head_len < 1e-9:
            return None
        prev_unit = (previous[0] / prev_len, previous[1] / prev_len)
        head_unit = (heading[0] / head_len, heading[1] / head_len)
        return self._angle_between_unit_vectors(prev_unit, head_unit)

    def _begin_follow_run(self, label: str, *, requested_steps: int, reset_latch: bool) -> None:
        self._follow_run_counter += 1
        self._active_follow_run_id = self._follow_run_counter
        self._active_follow_run_step = 0
        if reset_latch:
            self._clear_follow_heading()
        else:
            self._follow_virtual_target_xy = None
        self._timeline_log(
            "RUN_START",
            result="begin",
            reason=label,
            use_delayed_position=bool(self.follow_use_delayed_position_var.get()),
            position_delay_ms=self._get_follow_position_delay_ms(),
            use_virtual_target=bool(self.follow_virtual_target_var.get()),
            virtual_min_progress_pct=f"{self._get_follow_virtual_min_progress_pct():.3f}",
            capture_fps=f"{self._get_camera_stream_max_fps():.3f}",
            preview_fps=f"{self._get_camera_preview_max_fps():.3f}",
            move_len=f"{self._get_follow_step():.6f}",
            min_confidence=f"{self._get_follow_min_confidence():.3f}",
            applied_correct_len=f"{self._get_follow_max_correct():.6f}",
        )

    def follow_line_single_step(self) -> None:
        """Move one bounded step along the detected line/edge."""

        self._begin_follow_run("Follow Step", requested_steps=1, reset_latch=False)
        self._follow_stop_requested = False
        self._follow_line_step_impl(step_label="Follow Step", show_dialogs=True)

    def follow_line_multiple_steps(self) -> None:
        """Run a bounded number of single follow steps, stopping on trouble.

        This is still not continuous/free-running following. The user chooses a
        count, and FabScan performs that many already-bounded single-step
        moves. Each step re-detects the line/edge and stops if confidence drops,
        the target is lost, LinuxCNC is not ready, or STOP Move is pressed.
        """

        if self._motion_active:
            messagebox.showinfo("Motion active", "Wait for the current motion to finish or press STOP Move.", parent=self)
            return
        if self._manual_jog_active:
            return
        if not bool(self.follow_enabled_var.get()):
            messagebox.showinfo("Follow disabled", "Check Enable follow before using Follow N.", parent=self)
            return

        count = self._get_follow_repeat_count()
        if count <= 1:
            self.follow_line_single_step()
            return

        self._begin_follow_run("Follow N", requested_steps=count, reset_latch=True)
        self._follow_stop_requested = False
        completed = 0
        self.cal_status_var.set(f"Follow N starting: {count} requested steps.")
        self.update()

        for index in range(1, count + 1):
            if self._follow_stop_requested:
                break
            ok = self._follow_line_step_impl(step_label=f"Follow {index}/{count}", show_dialogs=False)
            if not ok:
                break
            completed += 1

        last = self.cal_status_var.get()
        if self._follow_stop_requested:
            self.cal_status_var.set(f"Follow N stopped by user after {completed}/{count} completed steps.")
        elif completed >= count:
            self.cal_status_var.set(f"Follow N complete: {completed}/{count} steps completed.")
        else:
            self.cal_status_var.set(f"Follow N stopped after {completed}/{count} completed steps. {last}")
        self._timeline_log(
            "RUN_END",
            result="stopped" if self._follow_stop_requested else ("complete" if completed >= count else "incomplete"),
            reason=self.cal_status_var.get(),
        )
        self._show_current_frame()

    def _follow_line_step_impl(self, *, step_label: str, show_dialogs: bool) -> bool:
        """Shared implementation for one camera-derived line/edge follow step."""

        step_start_time = time.monotonic()
        self._timeline_step_counter += 1
        step_id = self._timeline_step_counter
        self._active_follow_run_step += 1
        self._timeline_log(
            "STEP_START",
            step_id=step_id,
            step_label=step_label,
            result="begin",
            capture_fps=f"{self._get_camera_stream_max_fps():.3f}",
            preview_fps=f"{self._get_camera_preview_max_fps():.3f}",
            use_delayed_position=bool(self.follow_use_delayed_position_var.get()),
            position_delay_ms=self._get_follow_position_delay_ms(),
            use_virtual_target=bool(self.follow_virtual_target_var.get()),
            virtual_min_progress_pct=f"{self._get_follow_virtual_min_progress_pct():.3f}",
        )

        if self._motion_active:
            self._timeline_log("STEP_REFUSED", step_id=step_id, step_label=step_label, result="refused", reason="motion active")
            if show_dialogs:
                messagebox.showinfo("Motion active", "Wait for the current motion to finish or press STOP Move.", parent=self)
            return False
        if self._manual_jog_active:
            self._timeline_log("STEP_REFUSED", step_id=step_id, step_label=step_label, result="refused", reason="manual jog active")
            return False
        if not bool(self.follow_enabled_var.get()):
            self._timeline_log("STEP_REFUSED", step_id=step_id, step_label=step_label, result="refused", reason="follow disabled")
            if show_dialogs:
                messagebox.showinfo("Follow disabled", "Check Enable follow before using Follow Step.", parent=self)
            return False

        calibration = self._validate_calibration(self.active_calibration)
        if calibration is None:
            self._timeline_log("STEP_REFUSED", step_id=step_id, step_label=step_label, result="refused", reason="no valid calibration")
            if show_dialogs:
                messagebox.showinfo("No calibration", "Run calibration first, then use Follow Step.", parent=self)
            self.cal_status_var.set("Follow refused: no valid camera calibration.")
            return False
        if self.current_frame_bgr is None:
            self._timeline_log("STEP_REFUSED", step_id=step_id, step_label=step_label, result="refused", reason="no camera frame")
            if show_dialogs:
                messagebox.showinfo("No camera frame", "No camera frame is available yet.", parent=self)
            self.cal_status_var.set("Follow refused: no camera frame is available.")
            return False

        self._freshen_follow_frame_if_stale(step_id=step_id, step_label=step_label)
        transformed_for_size = self.get_transformed_frame_bgr(self.current_frame_bgr)
        frame_h, frame_w = transformed_for_size.shape[:2]

        status_read_start = time.monotonic()
        status = self.linuxcnc_reader.read_status()
        status_read_ms = (time.monotonic() - status_read_start) * 1000.0
        self._record_position_history(status, source="step_status")
        status_ok = self._status_ok_for_calibration(status)
        start_status_x = ""
        start_status_y = ""
        if status.connected:
            try:
                start_status_x, start_status_y, _status_z = self._active_position(status)
            except Exception:
                start_status_x = ""
                start_status_y = ""
        self._timeline_log(
            "STATUS_READ",
            step_id=step_id,
            step_label=step_label,
            linuxcnc_read_ms=f"{status_read_ms:.3f}",
            status_ok=bool(status_ok),
            task_state=status.task_state,
            task_mode=status.task_mode,
            interp_state=status.interp_state,
            start_x=f"{float(start_status_x):.6f}" if start_status_x != "" else "",
            start_y=f"{float(start_status_y):.6f}" if start_status_y != "" else "",
        )
        if not status_ok:
            message = status.error or self._status_not_ready_message(status)
            self._timeline_log(
                "STEP_REFUSED",
                step_id=step_id,
                step_label=step_label,
                result="refused",
                reason=message,
                linuxcnc_read_ms=f"{status_read_ms:.3f}",
                status_ok=False,
                task_state=status.task_state,
                task_mode=status.task_mode,
                interp_state=status.interp_state,
            )
            if show_dialogs:
                messagebox.showerror("LinuxCNC not ready", message, parent=self)
            self.cal_status_var.set(message)
            return False

        line, detection_ms = self._detect_line_for_follow_with_retries(
            step_id=step_id,
            step_label=step_label,
            phase="pre_move",
            base_event="DETECTION",
        )
        if not line.found:
            self.current_line = line
            reason = f"{line.message} after fresh-frame retry"
            self._timeline_log("STEP_REFUSED", step_id=step_id, step_label=step_label, result="refused", reason=reason)
            self.cal_status_var.set(reason)
            if show_dialogs:
                messagebox.showinfo("Line/edge not found", reason, parent=self)
            self._show_current_frame()
            return False

        if not self._follow_detection_sanity_ok(line, step_label=step_label):
            self.current_line = line
            self._timeline_log(
                "STEP_REFUSED",
                step_id=step_id,
                step_label=step_label,
                result="refused",
                reason=self.cal_status_var.get(),
                **self._timeline_line_fields(line),
            )
            self._show_current_frame()
            return False

        line = self._stabilize_follow_line(line, frame_w=frame_w, frame_h=frame_h)
        self.current_line = line

        min_confidence = self._get_follow_min_confidence()
        if line.confidence < min_confidence:
            reason = f"confidence {line.confidence:.0f}% is below minimum {min_confidence:.0f}%"
            self._timeline_log(
                "STEP_REFUSED",
                step_id=step_id,
                step_label=step_label,
                result="refused",
                reason=reason,
                min_confidence=f"{min_confidence:.3f}",
                **self._timeline_line_fields(line),
            )
            self.cal_status_var.set(f"{step_label} refused: {reason}.")
            self._show_current_frame()
            return False

        follow_step = self._get_follow_step()
        direction_preference = self._get_follow_direction_preference()

        correction = self._machine_correction_from_pixel_error(line.pixel_error_x, line.pixel_error_y)
        if correction is None:
            self._timeline_log("STEP_REFUSED", step_id=step_id, step_label=step_label, result="refused", reason="saved calibration could not be used for line following")
            if show_dialogs:
                messagebox.showerror("Bad calibration", "Saved calibration could not be used for line following.", parent=self)
            self.cal_status_var.set("Follow failed: saved calibration could not be used for line following.")
            return False

        corner_resume = bool(self._follow_corner_resume_once and line.corner_candidate)
        corner_assist: Optional[CornerAssistMove] = None
        if bool(self.follow_corner_pause_var.get()) and line.corner_candidate and not corner_resume:
            if bool(self.follow_corner_assist_var.get()):
                assist = self._corner_assist_move_from_line(
                    line,
                    frame_w=frame_w,
                    frame_h=frame_h,
                    follow_step=follow_step,
                    direction_preference=direction_preference,
                )
                if assist.ok:
                    corner_assist = assist
                else:
                    corner_angle = self._get_follow_corner_angle(normalize=True)
                    self._follow_corner_resume_once = True
                    self.cal_status_var.set(
                        f"{step_label} paused: corner seen, but Corner Assist refused "
                        f"({assist.message}; threshold {corner_angle:.0f}°). "
                        "Choose/check Start dir, adjust Lookahead/Search if needed, or click Find Line/Edge then Follow Step."
                    )
                    self._show_current_frame()
                    return False
            else:
                corner_angle = self._get_follow_corner_angle(normalize=True)
                self._follow_corner_resume_once = True
                self.cal_status_var.set(
                    f"{step_label} paused: possible corner/intersection detected "
                    f"({line.corner_message}; threshold {corner_angle:.0f}°). "
                    "Choose the next Start dir, then click Find Line/Edge. The next follow move will bypass corner pause once."
                )
                self._show_current_frame()
                return False

        # Consume the one-shot corner resume after confidence/status checks. If
        # the frame still contains an L-shaped corner, include the primary and
        # secondary Hough directions so Start dir can choose the outgoing leg
        # instead of the averaged diagonal line fit.
        self._follow_corner_resume_once = False

        max_correct = self._get_follow_max_correct()
        deadband = self._get_follow_deadband()
        gain = self._get_follow_gain()
        total_limited = False
        correction_limited = False

        if corner_assist is not None:
            heading = corner_assist.heading
            if heading is None:
                self.cal_status_var.set(f"{step_label} failed: corner assist had no outgoing heading.")
                self._show_current_frame()
                return False
            heading_state = f"corner assist {corner_assist.heading_state}"
            heading_change = self._follow_heading_change_degrees(heading)
            tangent_x = 0.0
            tangent_y = 0.0
            correct_x = 0.0
            correct_y = 0.0
            correction_state = "corner assist"
            raw_correct_len = 0.0
            applied_correct_len = 0.0
            move_x = corner_assist.move_x
            move_y = corner_assist.move_y
            move_len = math.hypot(move_x, move_y)
        else:
            heading_candidates = self._machine_heading_candidates_from_line(
                line,
                include_corner_directions=corner_resume,
            )
            heading, heading_state = self._choose_latched_follow_heading_from_candidates(
                heading_candidates,
                direction_preference,
            )
            if heading is None:
                self.cal_status_var.set(f"{step_label} failed: detected line direction could not be latched.")
                self._show_current_frame()
                return False

            heading_change = self._follow_heading_change_degrees(heading)
            max_heading_change = self._get_follow_max_heading_change()
            if heading_change is not None and max_heading_change > 0.0 and heading_change > max_heading_change:
                self.cal_status_var.set(
                    f"{step_label} stopped: heading changed {heading_change:.0f}°; max turn is {max_heading_change:.0f}°. "
                    "Use Find Line/Edge to reset the latch if this is an intentional corner."
                )
                self._show_current_frame()
                return False

            tangent_x = follow_step * heading[0]
            tangent_y = follow_step * heading[1]

            if corner_resume:
                # The line-center correction for an L-shaped corner is often based
                # on an averaged/diagonal fit. For the first resume step, move along
                # the selected outgoing leg only, then let the next fresh frame apply
                # normal correction once the corner is no longer dominating the ROI.
                correct_x = 0.0
                correct_y = 0.0
                correction_state = "corner resume"
                raw_correct_len = math.hypot(correction[0], correction[1])
                applied_correct_len = 0.0
            else:
                correct_x, correct_y, correction_limited, correction_state, raw_correct_len, applied_correct_len = (
                    self._apply_follow_correction_tuning(
                        correction[0],
                        correction[1],
                        deadband=deadband,
                        gain=gain,
                        max_correct=max_correct,
                    )
                )
            move_x = tangent_x + correct_x
            move_y = tangent_y + correct_y
            move_len = math.hypot(move_x, move_y)
            max_total = max(0.001, follow_step + max_correct)
            move_x, move_y, total_limited = self._limit_move_vector(move_x, move_y, max_total)
            move_len = math.hypot(move_x, move_y)

        start_x, start_y, _z = self._active_position(status)
        frame_move_x = move_x
        frame_move_y = move_y
        target_base_x, target_base_y, delay_fields = self._delayed_position_plan_fields(
            current_x=start_x,
            current_y=start_y,
            frame_move_x=frame_move_x,
            frame_move_y=frame_move_y,
        )
        target_x = target_base_x + frame_move_x
        target_y = target_base_y + frame_move_y
        move_x = target_x - start_x
        move_y = target_y - start_y
        delay_limited = False
        max_total_for_delay = max(0.001, follow_step + max_correct)
        move_x, move_y, delay_limited = self._limit_move_vector(move_x, move_y, max_total_for_delay)
        if delay_limited:
            target_x = start_x + move_x
            target_y = start_y + move_y
            prior_state = correction_state
            correction_state = f"{prior_state}; delay command limited" if prior_state else "delay command limited"

        virtual_fields = self._virtual_target_default_fields()
        if corner_assist is None and self._get_follow_virtual_target_enabled():
            target_x, target_y, move_x, move_y, virtual_fields = self._apply_follow_virtual_target(
                current_x=start_x,
                current_y=start_y,
                desired_target_x=target_x,
                desired_target_y=target_y,
                heading=heading,
                follow_step=follow_step,
                max_correct=max_correct,
            )
            prior_state = correction_state
            correction_state = f"{prior_state}; virtual target" if prior_state else "virtual target"
        move_len = math.hypot(move_x, move_y)

        progress_dot: Optional[float] = None
        progress_lock_refused = False
        previous_move = self._follow_last_move_unit or self._follow_heading_unit
        if corner_assist is None and previous_move is not None and move_len >= 0.0005:
            move_unit = self._normalize_unit_vector(move_x, move_y)
            previous_unit = self._normalize_unit_vector(previous_move[0], previous_move[1])
            if move_unit is not None and previous_unit is not None:
                progress_dot = (move_unit[0] * previous_unit[0]) + (move_unit[1] * previous_unit[1])
                # Hard progress lock: do not send a move that points backward
                # relative to the last successful commanded move. v0.5.23 tried
                # to recover silently with tangent-only fallback, but E/F test
                # traces showed alternating +X/-X points still reached the DXF.
                # Refuse visibly instead so a real reverse requires Find Line/Edge
                # or a deliberate Start dir/latch reset.
                if progress_dot < -0.05:
                    progress_lock_refused = True

        if progress_lock_refused:
            dot_text = f"{progress_dot:+.2f}" if progress_dot is not None else "unknown"
            reason = f"progress lock blocked reverse move (dot {dot_text})"
            self._timeline_log(
                "STEP_REFUSED",
                step_id=step_id,
                step_label=step_label,
                result="refused",
                reason=reason,
                progress_dot=f"{progress_dot:.6f}" if progress_dot is not None else "",
                move_x=f"{move_x:.6f}",
                move_y=f"{move_y:.6f}",
                move_len=f"{move_len:.6f}",
                **delay_fields,
                **virtual_fields,
                **self._timeline_line_fields(line),
            )
            self.cal_status_var.set(
                f"{step_label} refused: progress lock blocked a reverse move "
                f"(dot {dot_text}). Use Find Line/Edge to reset the latch if this reversal is intentional."
            )
            self._show_current_frame()
            return False

        if move_len < 0.0005:
            self._timeline_log(
                "STEP_REFUSED",
                step_id=step_id,
                step_label=step_label,
                result="refused",
                reason="calculated move is tiny",
                move_x=f"{move_x:.6f}",
                move_y=f"{move_y:.6f}",
                move_len=f"{move_len:.6f}",
                **delay_fields,
                **virtual_fields,
                **self._timeline_line_fields(line),
            )
            self.cal_status_var.set(f"{step_label}: calculated move is tiny. No move sent.")
            self._show_current_frame()
            return False

        max_heading_change = self._get_follow_max_heading_change()
        feed = self._get_follow_feed()
        settle_ms = self._get_follow_settle_ms()
        coordinate_mode = self.coordinate_mode_label

        self._timeline_log(
            "MOVE_PLAN",
            step_id=step_id,
            step_label=step_label,
            result="planned",
            status_ok=True,
            start_x=f"{start_x:.6f}",
            start_y=f"{start_y:.6f}",
            target_x=f"{target_x:.6f}",
            target_y=f"{target_y:.6f}",
            move_x=f"{move_x:.6f}",
            move_y=f"{move_y:.6f}",
            move_len=f"{move_len:.6f}",
            tangent_x=f"{tangent_x:.6f}",
            tangent_y=f"{tangent_y:.6f}",
            correct_x=f"{correct_x:.6f}",
            correct_y=f"{correct_y:.6f}",
            raw_correct_len=f"{raw_correct_len:.6f}",
            applied_correct_len=f"{applied_correct_len:.6f}",
            min_confidence=f"{min_confidence:.3f}",
            heading_state=heading_state,
            heading_change_degrees=f"{heading_change:.3f}" if heading_change is not None else "",
            progress_dot=f"{progress_dot:.6f}" if progress_dot is not None else "",
            correction_state=correction_state,
            **delay_fields,
            **virtual_fields,
            **self._timeline_line_fields(line),
        )

        self._manual_jog_active = True
        try:
            limit_bits = []
            if progress_dot is not None:
                limit_bits.append(f"progress {progress_dot:+.2f}")
            if correction_limited:
                limit_bits.append("side correction limited")
            elif correction_state == "inside deadband":
                limit_bits.append("inside deadband")
            elif correction_state == "corner resume":
                limit_bits.append("corner resume: side correction skipped")
            elif correction_state == "corner assist":
                lookahead_steps = self._get_follow_corner_lookahead_steps()
                limit_bits.append(f"corner assist: {corner_assist.distance:.4f}/{corner_assist.max_distance:.4f}, {lookahead_steps} step lookahead")
            if total_limited:
                limit_bits.append("total move limited")
            if self._get_follow_virtual_target_enabled() and corner_assist is None:
                state = str(virtual_fields.get("virtual_state", "virtual target"))
                limit_bits.append(f"virtual target: {state}")
            limit_text = f" ({', '.join(limit_bits)})" if limit_bits else ""
            turn_text = ""
            if heading_change is not None and max_heading_change > 0.0:
                turn_text = f"turn {heading_change:.0f}°/{max_heading_change:.0f}°, "
            if correction_state == "corner assist":
                move_detail = (
                    f"corner {corner_assist.message}, move X{move_x:+.4f} Y{move_y:+.4f}."
                    if corner_assist is not None
                    else f"move X{move_x:+.4f} Y{move_y:+.4f}."
                )
            else:
                move_detail = (
                    f"tangent X{tangent_x:+.4f} Y{tangent_y:+.4f}, "
                    f"correct raw {raw_correct_len:.4f} -> applied {applied_correct_len:.4f} "
                    f"(dead {deadband:.4f}, gain {gain:.2f}) X{correct_x:+.4f} Y{correct_y:+.4f}, "
                    f"total X{move_x:+.4f} Y{move_y:+.4f}."
                )
            self.cal_status_var.set(
                f"{step_label}{limit_text}: F{feed:.1f}, settle {settle_ms} ms, {heading_state}, "
                f"{turn_text}{move_detail}"
            )
            self.update()
            send_start = time.monotonic()
            if not self._send_correction_jogs(
                move_x,
                move_y,
                target_x,
                target_y,
                feed,
                coordinate_mode,
                timeline_step_id=step_id,
                timeline_step_label=step_label,
            ):
                self._timeline_log(
                    "MOVE_FAILED",
                    step_id=step_id,
                    step_label=step_label,
                    result="failed",
                    reason=self.cal_status_var.get(),
                    duration_ms=f"{(time.monotonic() - send_start) * 1000.0:.3f}",
                    move_x=f"{move_x:.6f}",
                    move_y=f"{move_y:.6f}",
                    move_len=f"{move_len:.6f}",
                )
                return False
            self._timeline_log(
                "MOVE_SENT",
                step_id=step_id,
                step_label=step_label,
                result="ok",
                duration_ms=f"{(time.monotonic() - send_start) * 1000.0:.3f}",
                move_x=f"{move_x:.6f}",
                move_y=f"{move_y:.6f}",
                move_len=f"{move_len:.6f}",
            )

            # Motion succeeded. Latch the machine-space heading used for this
            # step so the next detection cannot flip 180 degrees. For Corner
            # Assist, this is the selected outgoing leg, not the incoming leg.
            self._follow_heading_unit = heading
            move_unit = self._normalize_unit_vector(move_x, move_y)
            if move_unit is not None:
                self._follow_last_move_unit = move_unit
            settle_start = time.monotonic()
            self._wait_and_pump_camera(settle_ms / 1000.0)
            self._timeline_log(
                "SETTLE_DONE",
                step_id=step_id,
                step_label=step_label,
                result="ok",
                duration_ms=f"{(time.monotonic() - settle_start) * 1000.0:.3f}",
            )
            new_line, post_detection_ms = self._detect_line_for_follow_with_retries(
                step_id=step_id,
                step_label=step_label,
                phase="post_move",
                base_event="POST_DETECTION",
            )
            if correction_state == "corner assist":
                # The old-leg filter would fight the new outgoing leg. Re-seed
                # after the corner move if the next leg is visible.
                self._reset_follow_filter()
            if new_line.found:
                new_line = self._stabilize_follow_line(new_line, frame_w=frame_w, frame_h=frame_h)
            self.current_line = new_line
            post_status = self.linuxcnc_reader.read_status()
            self._record_position_history(post_status, source="post_detection")
            post_x = ""
            post_y = ""
            if post_status.connected:
                try:
                    post_x, post_y, _post_z = self._active_position(post_status)
                except Exception:
                    post_x = ""
                    post_y = ""
            self._timeline_log(
                "POST_DETECTION_FINAL",
                step_id=step_id,
                step_label=step_label,
                result="found" if new_line.found else "not_found",
                reason=new_line.message,
                post_x=f"{float(post_x):.6f}" if post_x != "" else "",
                post_y=f"{float(post_y):.6f}" if post_y != "" else "",
                target_x=f"{target_x:.6f}",
                target_y=f"{target_y:.6f}",
                duration_ms=f"{post_detection_ms:.3f}",
                **self._timeline_line_fields(new_line),
            )
            capture_text = ""
            if bool(self.follow_capture_point_var.get()) and self.trace_capture_callback is not None:
                self.trace_capture_callback()
                capture_text = " Captured current position to the active trace."
            if new_line.found:
                self.cal_status_var.set(
                    f"{step_label} complete. New offset X{new_line.pixel_error_x:+.1f}px "
                    f"Y{new_line.pixel_error_y:+.1f}px, confidence {new_line.confidence:.0f}%." + capture_text
                )
            else:
                self.cal_status_var.set(f"{step_label} complete, but the line/edge was not found afterward." + capture_text)
            self._timeline_log(
                "STEP_COMPLETE",
                step_id=step_id,
                step_label=step_label,
                result="ok",
                duration_ms=f"{(time.monotonic() - step_start_time) * 1000.0:.3f}",
                move_x=f"{move_x:.6f}",
                move_y=f"{move_y:.6f}",
                move_len=f"{move_len:.6f}",
                target_x=f"{target_x:.6f}",
                target_y=f"{target_y:.6f}",
            )
            self._show_current_frame()
            return True
        finally:
            self._manual_jog_active = False

    def center_dot_using_calibration(self) -> None:
        if self._motion_active:
            messagebox.showinfo("Calibration active", "Wait for calibration to finish or press STOP Move.", parent=self)
            return
        if self._manual_jog_active:
            return

        calibration = self._validate_calibration(self.active_calibration)
        if calibration is None:
            messagebox.showinfo("No calibration", "Run calibration first, then use Center Dot.", parent=self)
            return
        if self.current_frame_bgr is None:
            messagebox.showinfo("No camera frame", "No camera frame is available yet.", parent=self)
            return

        status = self.linuxcnc_reader.read_status()
        if not self._status_ok_for_calibration(status):
            messagebox.showerror("LinuxCNC not ready", status.error or self._status_not_ready_message(status), parent=self)
            return

        transformed = self.get_transformed_frame_bgr(self.current_frame_bgr)
        frame_h, frame_w = transformed.shape[:2]
        dot = self.detect_dot_in_frame(transformed)
        self.current_dot = dot
        if not dot.found:
            self.cal_status_var.set(dot.message)
            messagebox.showinfo("Dot not found", dot.message, parent=self)
            self._show_current_frame()
            return

        err_x = dot.x - (frame_w / 2.0)
        err_y = dot.y - (frame_h / 2.0)
        pixel_error = math.hypot(err_x, err_y)
        if pixel_error <= 2.0:
            self.cal_status_var.set(f"Center Dot: already centered within {pixel_error:.1f} px.")
            self._show_current_frame()
            return

        try:
            inv = calibration["matrix_pixel_to_machine"]
            # Desired pixel shift is opposite the current dot-to-crosshair error.
            move_x = float(inv[0][0]) * (-err_x) + float(inv[0][1]) * (-err_y)
            move_y = float(inv[1][0]) * (-err_x) + float(inv[1][1]) * (-err_y)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Bad calibration", f"Saved calibration could not be used: {exc}", parent=self)
            return

        if not math.isfinite(move_x) or not math.isfinite(move_y):
            messagebox.showerror("Bad correction", "Calculated dot-centering move was not finite.", parent=self)
            return

        max_move = self._get_center_max_move()
        vector_len = math.hypot(move_x, move_y)
        limited = False
        if vector_len > max_move:
            scale = max_move / vector_len
            move_x *= scale
            move_y *= scale
            vector_len = max_move
            limited = True

        if vector_len < 0.0005:
            self.cal_status_var.set(f"Center Dot: correction is tiny ({vector_len:.5f} units). No move sent.")
            self._show_current_frame()
            return

        start_x, start_y, _z = self._active_position(status)
        target_x = start_x + move_x
        target_y = start_y + move_y
        feed = self._get_feed()
        coordinate_mode = self.coordinate_mode_label

        self._timeline_log(
            "CENTER_DOT_MOVE_PLAN",
            step_label="Center Dot",
            result="planned",
            status_ok=True,
            start_x=f"{start_x:.6f}",
            start_y=f"{start_y:.6f}",
            target_x=f"{target_x:.6f}",
            target_y=f"{target_y:.6f}",
            move_x=f"{move_x:.6f}",
            move_y=f"{move_y:.6f}",
            move_len=f"{vector_len:.6f}",
            pixel_error_x=f"{err_x:.3f}",
            pixel_error_y=f"{err_y:.3f}",
            reason="limited" if limited else "",
        )

        self._manual_jog_active = True
        try:
            limit_text = " limited" if limited else ""
            self.cal_status_var.set(
                f"Center Dot:{limit_text} correction X{move_x:+.4f} Y{move_y:+.4f} "
                f"from pixel error X{err_x:+.1f} Y{err_y:+.1f}."
            )
            if not self._send_correction_jogs(
                move_x,
                move_y,
                target_x,
                target_y,
                feed,
                coordinate_mode,
                timeline_step_label="Center Dot",
            ):
                return
            self._wait_and_pump_camera(0.20)
            new_dot = self.detect_dot()
            self.current_dot = new_dot
            if new_dot.found:
                new_err_x = new_dot.x - (frame_w / 2.0)
                new_err_y = new_dot.y - (frame_h / 2.0)
                self.cal_status_var.set(
                    f"Center Dot complete. New offset X{new_err_x:+.1f}px Y{new_err_y:+.1f}px. "
                    "Click again if you want to sneak up on center."
                )
            else:
                self.cal_status_var.set("Center Dot move complete, but the dot was not found afterward.")
            self._show_current_frame()
        finally:
            self._manual_jog_active = False

    def _send_correction_jogs(
        self,
        move_x: float,
        move_y: float,
        target_x: float,
        target_y: float,
        feed: float,
        coordinate_mode: str,
        *,
        timeline_step_id: int = 0,
        timeline_step_label: str = "",
    ) -> bool:
        start_status = self.linuxcnc_reader.read_status()
        self._record_position_history(start_status, source="jog_start")
        start_x, start_y, _z = self._active_position(start_status)

        if abs(move_x) >= 0.0005:
            direction = 1 if move_x >= 0.0 else -1
            distance = abs(move_x)
            event_start = time.monotonic()
            self._timeline_log(
                "JOG_X_START",
                step_id=timeline_step_id,
                step_label=timeline_step_label,
                result="begin",
                start_x=f"{start_x:.6f}",
                start_y=f"{start_y:.6f}",
                target_x=f"{start_x + move_x:.6f}",
                target_y=f"{start_y:.6f}",
                move_x=f"{move_x:.6f}",
                move_y="0.000000",
                move_len=f"{distance:.6f}",
            )
            jog = self.linuxcnc_reader.incremental_jog("X", direction, distance, feed)
            if not jog.success:
                self._timeline_log(
                    "JOG_X_FAILED",
                    step_id=timeline_step_id,
                    step_label=timeline_step_label,
                    result="failed",
                    reason=jog.message,
                    duration_ms=f"{(time.monotonic() - event_start) * 1000.0:.3f}",
                )
                messagebox.showerror("Center Dot jog failed", jog.message, parent=self)
                self.cal_status_var.set(jog.message)
                return False
            wait_ok = self._wait_for_position_near(start_x + move_x, start_y, coordinate_mode, distance)
            self._timeline_log(
                "JOG_X_DONE",
                step_id=timeline_step_id,
                step_label=timeline_step_label,
                result="ok" if wait_ok else "failed",
                duration_ms=f"{(time.monotonic() - event_start) * 1000.0:.3f}",
                target_x=f"{start_x + move_x:.6f}",
                target_y=f"{start_y:.6f}",
                move_x=f"{move_x:.6f}",
                move_y="0.000000",
                move_len=f"{distance:.6f}",
            )
            if not wait_ok:
                self.cal_status_var.set("Center Dot X correction did not settle as expected.")
                return False

        if abs(move_y) >= 0.0005:
            direction = 1 if move_y >= 0.0 else -1
            distance = abs(move_y)
            event_start = time.monotonic()
            self._timeline_log(
                "JOG_Y_START",
                step_id=timeline_step_id,
                step_label=timeline_step_label,
                result="begin",
                target_x=f"{target_x:.6f}",
                target_y=f"{target_y:.6f}",
                move_x="0.000000",
                move_y=f"{move_y:.6f}",
                move_len=f"{distance:.6f}",
            )
            jog = self.linuxcnc_reader.incremental_jog("Y", direction, distance, feed)
            if not jog.success:
                self._timeline_log(
                    "JOG_Y_FAILED",
                    step_id=timeline_step_id,
                    step_label=timeline_step_label,
                    result="failed",
                    reason=jog.message,
                    duration_ms=f"{(time.monotonic() - event_start) * 1000.0:.3f}",
                )
                messagebox.showerror("Center Dot jog failed", jog.message, parent=self)
                self.cal_status_var.set(jog.message)
                return False
            wait_ok = self._wait_for_position_near(target_x, target_y, coordinate_mode, distance)
            self._timeline_log(
                "JOG_Y_DONE",
                step_id=timeline_step_id,
                step_label=timeline_step_label,
                result="ok" if wait_ok else "failed",
                duration_ms=f"{(time.monotonic() - event_start) * 1000.0:.3f}",
                target_x=f"{target_x:.6f}",
                target_y=f"{target_y:.6f}",
                move_x="0.000000",
                move_y=f"{move_y:.6f}",
                move_len=f"{distance:.6f}",
            )
            if not wait_ok:
                self.cal_status_var.set("Center Dot Y correction did not settle as expected.")
                return False

        return True

    def _make_result(self, calibration: Optional[dict[str, Any]]) -> CameraCalibrationDialogResult:
        width, height = self._get_requested_size()
        return CameraCalibrationDialogResult(
            camera_index=self._get_camera_index(),
            requested_width=width,
            requested_height=height,
            camera_stream_max_fps=self._get_camera_stream_max_fps(),
            camera_preview_max_fps=self._get_camera_preview_max_fps(),
            linuxcnc_safe_preview=bool(self.linuxcnc_safe_preview_var.get()),
            profile_preview=bool(self.profile_preview_var.get()),
            rotate_degrees=self._get_rotate_degrees(),
            flip_x=bool(self.flip_x_var.get()),
            flip_y=bool(self.flip_y_var.get()),
            fine_rotation_degrees=self._get_fine_rotation_degrees(),
            threshold=self._get_threshold(),
            show_dot_marker=bool(self.show_dot_marker_var.get()),
            show_mask=bool(self.show_mask_var.get()),
            move_distance=self._get_move_distance(),
            feed_units_per_min=self._get_feed(),
            jog_step=self._get_jog_step(),
            center_max_move=self._get_center_max_move(),
            line_mode=self._get_line_mode(),
            line_search_px=self._get_line_search_px(normalize=True),
            show_line_preview=bool(self.show_line_preview_var.get()),
            follow_step=self._get_follow_step(),
            follow_feed_units_per_min=self._get_follow_feed(),
            follow_settle_ms=self._get_follow_settle_ms(),
            follow_max_heading_change_degrees=self._get_follow_max_heading_change(),
            follow_corner_pause_enabled=bool(self.follow_corner_pause_var.get()),
            follow_corner_angle_degrees=self._get_follow_corner_angle(normalize=True),
            follow_corner_assist_enabled=bool(self.follow_corner_assist_var.get()),
            follow_corner_lookahead_steps=self._get_follow_corner_lookahead_steps(),
            follow_max_correct=self._get_follow_max_correct(),
            follow_deadband=self._get_follow_deadband(),
            follow_gain=self._get_follow_gain(),
            follow_stabilize_enabled=bool(self.follow_stabilize_var.get()),
            follow_filter_offset_alpha=self._get_follow_filter_offset_alpha(),
            follow_filter_angle_alpha=self._get_follow_filter_angle_alpha(),
            follow_sanity_angle_degrees=self._get_follow_sanity_angle(normalize=True),
            follow_min_confidence=self._get_follow_min_confidence(),
            follow_direction=self._normalize_follow_direction(self.follow_direction_var.get()),
            follow_capture_point=bool(self.follow_capture_point_var.get()),
            follow_enabled=bool(self.follow_enabled_var.get()),
            follow_repeat_count=self._get_follow_repeat_count(),
            follow_timeline_log_enabled=bool(self.follow_timeline_log_var.get()),
            follow_use_delayed_position=bool(self.follow_use_delayed_position_var.get()),
            follow_position_delay_ms=self._get_follow_position_delay_ms(),
            follow_virtual_target_enabled=bool(self.follow_virtual_target_var.get()),
            follow_virtual_min_progress_pct=self._get_follow_virtual_min_progress_pct(),
            calibration=calibration or self.active_calibration,
        )

    def stop_motion(self) -> None:
        self._follow_stop_requested = True
        self._timeline_log("STOP_REQUESTED", result="requested")
        self._clear_follow_heading()
        result = self.linuxcnc_reader.abort_motion()
        self._timeline_log("STOP_DONE", result="ok" if result.success else "failed", reason=result.message)
        self.cal_status_var.set(result.message)

    def close(self) -> None:
        self._closing = True
        if self.result is None:
            self.result = self._make_result(calibration=None)
        self.release_camera()
        self._close_timeline_log()
        try:
            self.destroy()
        except tk.TclError:
            pass
