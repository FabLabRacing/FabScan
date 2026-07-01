from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import math
import time
from typing import Any, Optional

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageTk
import tkinter as tk
from tkinter import messagebox, ttk

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
class CameraCalibrationDialogResult:
    camera_index: int
    requested_width: int
    requested_height: int
    rotate_degrees: int
    flip_x: bool
    flip_y: bool
    fine_rotation_degrees: float
    threshold: int
    show_mask: bool
    move_distance: float
    feed_units_per_min: float
    jog_step: float
    center_max_move: float
    calibration: Optional[dict[str, Any]] = None


class CameraCalibrationDialog(tk.Toplevel):
    """Camera/machine calibration helper for FabScan.

    This is intentionally a "lite" version. It does not follow an edge yet.
    It only proves the core Scanything-style idea:

        known LinuxCNC X/Y move -> observed camera pixel shift

    The final DXF should still be based on LinuxCNC position. This calibration
    is only for later steering corrections while camera-following a line/edge.
    """

    def __init__(
        self,
        parent: tk.Misc,
        *,
        linuxcnc_reader: LinuxCNCStatusReader,
        coordinate_mode_label: str,
        camera_index: int = 0,
        camera_width: int = 1280,
        camera_height: int = 720,
        rotate_degrees: int = 0,
        flip_x: bool = False,
        flip_y: bool = False,
        fine_rotation_degrees: float = 0.0,
        threshold: int = 90,
        move_distance: float = 0.100,
        feed_per_minute: float = 5.0,
        jog_step: float = 0.010,
        center_max_move: float = 0.100,
        show_mask: bool = False,
        existing_calibration: Optional[dict[str, Any]] = None,
    ) -> None:
        super().__init__(parent)
        self.title("FabScan Camera Calibration Lite - v0.5.3")
        self.minsize(1040, 720)
        self.transient(parent)

        self.linuxcnc_reader = linuxcnc_reader
        self.coordinate_mode_label = coordinate_mode_label or "Work coordinates"
        self.result: Optional[CameraCalibrationDialogResult] = None
        self.cap: Optional[cv2.VideoCapture] = None
        self.current_frame_bgr: Optional[np.ndarray] = None
        self.current_dot: DotDetection = DotDetection(False)
        self.after_job: Optional[str] = None
        self._tk_preview: Optional[ImageTk.PhotoImage] = None
        self._motion_active = False
        self._manual_jog_active = False
        self.active_calibration: Optional[dict[str, Any]] = self._validate_calibration(existing_calibration)

        if int(rotate_degrees) not in ROTATE_VALUES:
            rotate_degrees = 0

        self.camera_index_var = tk.IntVar(value=max(0, int(camera_index)))
        self.camera_width_var = tk.IntVar(value=max(0, int(camera_width)))
        self.camera_height_var = tk.IntVar(value=max(0, int(camera_height)))
        self.rotate_var = tk.StringVar(value=str(int(rotate_degrees)))
        self.flip_x_var = tk.BooleanVar(value=bool(flip_x))
        self.flip_y_var = tk.BooleanVar(value=bool(flip_y))
        self.fine_rotation_var = tk.DoubleVar(value=self._clamp_fine_rotation(fine_rotation_degrees))
        self.threshold_var = tk.IntVar(value=max(0, min(255, int(threshold))))
        self.show_mask_var = tk.BooleanVar(value=bool(show_mask))
        self.move_distance_var = tk.DoubleVar(value=max(0.001, float(move_distance)))
        self.feed_var = tk.DoubleVar(value=max(0.1, float(feed_per_minute)))
        self.jog_step_var = tk.DoubleVar(value=max(0.001, float(jog_step)))
        self.center_max_move_var = tk.DoubleVar(value=max(0.001, float(center_max_move)))
        self.dot_status_var = tk.StringVar(value="Dot: —")
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
        top = ttk.Frame(self, padding=8)
        top.pack(side=tk.TOP, fill=tk.X)

        camera = ttk.LabelFrame(top, text="Camera", padding=6)
        camera.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(camera, text="Index").pack(side=tk.LEFT)
        ttk.Spinbox(camera, from_=0, to=10, textvariable=self.camera_index_var, width=5).pack(
            side=tk.LEFT, padx=(4, 12)
        )
        ttk.Label(camera, text="Width").pack(side=tk.LEFT)
        ttk.Entry(camera, textvariable=self.camera_width_var, width=8).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(camera, text="Height").pack(side=tk.LEFT)
        ttk.Entry(camera, textvariable=self.camera_height_var, width=8).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Button(camera, text="Open / Restart Camera", command=self.open_camera).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(camera, text="Close", command=self.close).pack(side=tk.LEFT)

        controls = ttk.Frame(top)
        controls.pack(side=tk.TOP, fill=tk.X, pady=(8, 0))

        orientation = ttk.LabelFrame(controls, text="Orientation", padding=6)
        orientation.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 8))

        ttk.Label(orientation, text="Rotate").grid(row=0, column=0, sticky=tk.W)
        ttk.Combobox(
            orientation,
            textvariable=self.rotate_var,
            values=tuple(str(value) for value in ROTATE_VALUES),
            width=5,
            state="readonly",
        ).grid(row=0, column=1, sticky=tk.W, padx=(4, 10))
        ttk.Label(orientation, text="deg").grid(row=0, column=2, sticky=tk.W)
        ttk.Checkbutton(orientation, text="Flip X", variable=self.flip_x_var).grid(row=0, column=3, sticky=tk.W, padx=(12, 0))
        ttk.Checkbutton(orientation, text="Flip Y", variable=self.flip_y_var).grid(row=0, column=4, sticky=tk.W, padx=(8, 0))

        ttk.Label(orientation, text="Fine rotation").grid(row=1, column=0, sticky=tk.W, pady=(6, 0))
        ttk.Scale(
            orientation,
            from_=-10.0,
            to=10.0,
            variable=self.fine_rotation_var,
            command=lambda _value: self._show_current_frame(),
        ).grid(row=1, column=1, columnspan=4, sticky="ew", padx=(4, 8), pady=(6, 0))
        self.fine_rotation_label = ttk.Label(orientation, width=8)
        self.fine_rotation_label.grid(row=1, column=5, sticky=tk.W, pady=(6, 0))
        orientation.columnconfigure(4, weight=1)
        self._update_fine_rotation_label()

        vision = ttk.LabelFrame(controls, text="Dot / Mask", padding=6)
        vision.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        ttk.Label(vision, text="Threshold").grid(row=0, column=0, sticky=tk.W)
        ttk.Scale(
            vision,
            from_=0,
            to=255,
            variable=self.threshold_var,
            command=lambda _value: self._show_current_frame(),
        ).grid(row=0, column=1, sticky="ew", padx=(4, 6))
        self.threshold_label = ttk.Label(vision, width=4)
        self.threshold_label.grid(row=0, column=2, sticky=tk.W)
        ttk.Checkbutton(vision, text="Mask view", variable=self.show_mask_var, command=self._show_current_frame).grid(
            row=1, column=0, columnspan=3, sticky=tk.W, pady=(4, 0)
        )
        ttk.Button(vision, text="Find Dot", command=self.find_dot_once).grid(row=2, column=0, columnspan=3, sticky="ew", pady=(6, 0))
        vision.columnconfigure(1, weight=1)

        motion = ttk.LabelFrame(controls, text="Calibration Motion", padding=6)
        motion.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Label(motion, text="Move dist").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(motion, textvariable=self.move_distance_var, width=8).grid(row=0, column=1, sticky=tk.W, padx=(4, 8))
        ttk.Label(motion, text="Feed/min").grid(row=1, column=0, sticky=tk.W, pady=(4, 0))
        ttk.Entry(motion, textvariable=self.feed_var, width=8).grid(row=1, column=1, sticky=tk.W, padx=(4, 8), pady=(4, 0))
        ttk.Button(motion, text="Run Calibration", command=self.run_calibration).grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0)
        )
        ttk.Button(motion, text="STOP Move", command=self.stop_motion).grid(row=3, column=0, columnspan=2, sticky="ew", pady=(4, 0))

        jog = ttk.LabelFrame(controls, text="Dot Center Jog", padding=6)
        jog.pack(side=tk.LEFT, fill=tk.Y, padx=(8, 0))
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

        info = ttk.Frame(self, padding=(8, 0, 8, 0))
        info.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(info, textvariable=self.dot_status_var, anchor=tk.W).pack(side=tk.TOP, fill=tk.X)
        ttk.Label(info, textvariable=self.cal_status_var, anchor=tk.W).pack(side=tk.TOP, fill=tk.X)
        ttk.Label(info, textvariable=self.transform_status_var, anchor=tk.W).pack(side=tk.TOP, fill=tk.X)

        self.preview_label = ttk.Label(self, anchor=tk.CENTER)
        self.preview_label.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=8)

        footer = ttk.Label(
            self,
            text=(
                "Calibration, dot-centering, and screen jogs use guarded X/Y incremental jogs through LinuxCNC MANUAL mode. Torch/plasma should stay disabled. "
                "The camera steers; LinuxCNC remains the ruler."
            ),
            anchor=tk.W,
            padding=(8, 0, 8, 8),
        )
        footer.pack(side=tk.BOTTOM, fill=tk.X)

    def _register_traces(self) -> None:
        watched_vars: tuple[tk.Variable, ...] = (
            self.rotate_var,
            self.flip_x_var,
            self.flip_y_var,
            self.fine_rotation_var,
            self.threshold_var,
            self.show_mask_var,
        )
        for variable in watched_vars:
            variable.trace_add("write", lambda *_args: self._on_preview_setting_changed())

    def _on_preview_setting_changed(self) -> None:
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

    def open_camera(self) -> None:
        self.release_camera()
        self.current_frame_bgr = None
        self.current_dot = DotDetection(False)
        self._update_threshold_label()

        index = self._get_camera_index()
        width, height = self._get_requested_size()
        cap = cv2.VideoCapture(index)
        if width > 0:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height > 0:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if not cap.isOpened():
            cap.release()
            self.cap = None
            self.cal_status_var.set(f"Camera {index} did not open. Try another index or check the USB camera.")
            return

        self.cap = cap
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.cal_status_var.set(f"Camera {index} open. Actual frame: {actual_w} x {actual_h}.")
        self._schedule_next_frame()

    def release_camera(self) -> None:
        if self.after_job is not None:
            try:
                self.after_cancel(self.after_job)
            except tk.TclError:
                pass
            self.after_job = None
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def _schedule_next_frame(self) -> None:
        self.after_job = self.after(50, self._update_preview)

    def _update_preview(self) -> None:
        self.after_job = None
        self._pump_camera_frame()
        if self.cap is not None:
            self._schedule_next_frame()

    def _pump_camera_frame(self) -> bool:
        if self.cap is None:
            return False
        ok, frame = self.cap.read()
        if ok and frame is not None:
            self.current_frame_bgr = frame
            self._show_frame(frame)
            return True
        self.cal_status_var.set("Camera read failed. Try Open / Restart Camera.")
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
        transformed = self.get_transformed_frame_bgr(frame_bgr)
        self.current_dot = self.detect_dot_in_frame(transformed)

        if bool(self.show_mask_var.get()):
            mask = self._make_mask(transformed)
            preview_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        else:
            preview_rgb = cv2.cvtColor(transformed, cv2.COLOR_BGR2RGB)

        pil_image = Image.fromarray(preview_rgb)
        original_w, original_h = pil_image.size
        max_w = max(1, self.preview_label.winfo_width() - 20)
        max_h = max(1, self.preview_label.winfo_height() - 20)
        scale = min(max_w / original_w, max_h / original_h, 1.0)
        new_w = max(1, int(original_w * scale))
        new_h = max(1, int(original_h * scale))
        if (new_w, new_h) != pil_image.size:
            pil_image = pil_image.resize((new_w, new_h), Image.Resampling.LANCZOS)

        self._draw_overlay(pil_image, scale, original_w, original_h)
        self._tk_preview = ImageTk.PhotoImage(pil_image)
        self.preview_label.configure(image=self._tk_preview)
        self._update_dot_status(original_w, original_h)

    def _draw_overlay(self, pil_image: Image.Image, scale: float, source_w: int, source_h: int) -> None:
        draw = ImageDraw.Draw(pil_image)
        w, h = pil_image.size
        cx = w // 2
        cy = h // 2
        outline = (0, 0, 0)
        cross = (255, 230, 0)
        found = (0, 255, 0)
        missing = (255, 80, 80)

        draw.line((cx, 0, cx, h), fill=outline, width=5)
        draw.line((0, cy, w, cy), fill=outline, width=5)
        draw.line((cx, 0, cx, h), fill=cross, width=2)
        draw.line((0, cy, w, cy), fill=cross, width=2)

        r = 7
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), outline=outline, width=4)
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), outline=cross, width=2)

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

        draw.text((10, h - 24), f"Source {source_w}x{source_h}  Threshold {self._get_threshold()}", fill=(255, 255, 255))

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

        proceed = messagebox.askyesno(
            "Run camera calibration?",
            (
                "FabScan will command small X/Y incremental jogs through LinuxCNC.\n\n"
                "Put QtPlasmaC/LinuxCNC in MANUAL/JOG mode before continuing.\n"
                f"Coordinate source for displayed/saved positions: {coordinate_mode}\n"
                f"Jog distance: {move_distance:.4f}\n"
                f"Feed: {feed:.1f} units/min\n\n"
                "Keep the torch/plasma disabled and keep your hand near E-stop. Continue?"
            ),
            parent=self,
        )
        if not proceed:
            return

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

        self._manual_jog_active = True
        try:
            limit_text = " limited" if limited else ""
            self.cal_status_var.set(
                f"Center Dot:{limit_text} correction X{move_x:+.4f} Y{move_y:+.4f} "
                f"from pixel error X{err_x:+.1f} Y{err_y:+.1f}."
            )
            if not self._send_correction_jogs(move_x, move_y, target_x, target_y, feed, coordinate_mode):
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
    ) -> bool:
        start_status = self.linuxcnc_reader.read_status()
        start_x, start_y, _z = self._active_position(start_status)

        if abs(move_x) >= 0.0005:
            direction = 1 if move_x >= 0.0 else -1
            distance = abs(move_x)
            jog = self.linuxcnc_reader.incremental_jog("X", direction, distance, feed)
            if not jog.success:
                messagebox.showerror("Center Dot jog failed", jog.message, parent=self)
                self.cal_status_var.set(jog.message)
                return False
            if not self._wait_for_position_near(start_x + move_x, start_y, coordinate_mode, distance):
                self.cal_status_var.set("Center Dot X correction did not settle as expected.")
                return False

        if abs(move_y) >= 0.0005:
            direction = 1 if move_y >= 0.0 else -1
            distance = abs(move_y)
            jog = self.linuxcnc_reader.incremental_jog("Y", direction, distance, feed)
            if not jog.success:
                messagebox.showerror("Center Dot jog failed", jog.message, parent=self)
                self.cal_status_var.set(jog.message)
                return False
            if not self._wait_for_position_near(target_x, target_y, coordinate_mode, distance):
                self.cal_status_var.set("Center Dot Y correction did not settle as expected.")
                return False

        return True

    def _make_result(self, calibration: Optional[dict[str, Any]]) -> CameraCalibrationDialogResult:
        width, height = self._get_requested_size()
        return CameraCalibrationDialogResult(
            camera_index=self._get_camera_index(),
            requested_width=width,
            requested_height=height,
            rotate_degrees=self._get_rotate_degrees(),
            flip_x=bool(self.flip_x_var.get()),
            flip_y=bool(self.flip_y_var.get()),
            fine_rotation_degrees=self._get_fine_rotation_degrees(),
            threshold=self._get_threshold(),
            show_mask=bool(self.show_mask_var.get()),
            move_distance=self._get_move_distance(),
            feed_units_per_min=self._get_feed(),
            jog_step=self._get_jog_step(),
            center_max_move=self._get_center_max_move(),
            calibration=calibration or self.active_calibration,
        )

    def stop_motion(self) -> None:
        result = self.linuxcnc_reader.abort_motion()
        self.cal_status_var.set(result.message)

    def close(self) -> None:
        if self.result is None:
            self.result = self._make_result(calibration=None)
        self.release_camera()
        self.destroy()
