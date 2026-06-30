from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageTk
import tkinter as tk
from tkinter import messagebox, ttk


ROTATE_VALUES = (0, 90, 180, 270)


@dataclass
class CameraCaptureResult:
    """Still frame captured from a camera."""

    frame_bgr: np.ndarray
    camera_index: int
    requested_width: int
    requested_height: int
    rotate_degrees: int = 0
    flip_x: bool = False
    flip_y: bool = False
    fine_rotation_degrees: float = 0.0
    show_crosshair: bool = True
    show_axis_labels: bool = True
    show_grid: bool = False


class CameraCaptureDialog(tk.Toplevel):
    """Small OpenCV camera preview/capture dialog.

    The dialog returns one BGR frame. FabScan then treats that still image the
    same way it treats a loaded PNG/JPG. This keeps the camera feature out of
    the contour/scale/DXF pipeline.

    Orientation transforms are applied to the captured still frame before it is
    returned. Preview overlays are for alignment only and are not baked into
    the captured image.
    """

    def __init__(
        self,
        parent: tk.Misc,
        *,
        camera_index: int = 0,
        camera_width: int = 1280,
        camera_height: int = 720,
        rotate_degrees: int = 0,
        flip_x: bool = False,
        flip_y: bool = False,
        fine_rotation_degrees: float = 0.0,
        show_crosshair: bool = True,
        show_axis_labels: bool = True,
        show_grid: bool = False,
    ) -> None:
        super().__init__(parent)
        self.title("FabScan Camera Capture")
        self.minsize(780, 600)
        self.transient(parent)

        self.result: Optional[CameraCaptureResult] = None
        self.cap: Optional[cv2.VideoCapture] = None
        self.current_frame_bgr: Optional[np.ndarray] = None
        self.after_job: Optional[str] = None
        self._tk_preview: Optional[ImageTk.PhotoImage] = None

        if int(rotate_degrees) not in ROTATE_VALUES:
            rotate_degrees = 0

        self.camera_index_var = tk.IntVar(value=max(0, int(camera_index)))
        self.camera_width_var = tk.IntVar(value=max(0, int(camera_width)))
        self.camera_height_var = tk.IntVar(value=max(0, int(camera_height)))
        self.rotate_var = tk.StringVar(value=str(int(rotate_degrees)))
        self.flip_x_var = tk.BooleanVar(value=bool(flip_x))
        self.flip_y_var = tk.BooleanVar(value=bool(flip_y))
        self.fine_rotation_var = tk.DoubleVar(value=self._clamp_fine_rotation(fine_rotation_degrees))
        self.show_crosshair_var = tk.BooleanVar(value=bool(show_crosshair))
        self.show_axis_labels_var = tk.BooleanVar(value=bool(show_axis_labels))
        self.show_grid_var = tk.BooleanVar(value=bool(show_grid))
        self.status_var = tk.StringVar(value="Open a camera to start preview.")

        self._build_ui()
        self._register_orientation_traces()
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.bind("<Escape>", lambda _event: self.close())
        self.bind("<Return>", lambda _event: self.capture_frame())

        # Start preview shortly after the window paints.
        self.after(100, self.open_camera)

    def _build_ui(self) -> None:
        top = ttk.Frame(self, padding=8)
        top.pack(side=tk.TOP, fill=tk.X)

        connection = ttk.LabelFrame(top, text="Camera", padding=6)
        connection.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(connection, text="Index").pack(side=tk.LEFT)
        index_spin = ttk.Spinbox(
            connection,
            from_=0,
            to=10,
            textvariable=self.camera_index_var,
            width=5,
        )
        index_spin.pack(side=tk.LEFT, padx=(4, 12))

        ttk.Label(connection, text="Width").pack(side=tk.LEFT)
        width_entry = ttk.Entry(connection, textvariable=self.camera_width_var, width=8)
        width_entry.pack(side=tk.LEFT, padx=(4, 12))

        ttk.Label(connection, text="Height").pack(side=tk.LEFT)
        height_entry = ttk.Entry(connection, textvariable=self.camera_height_var, width=8)
        height_entry.pack(side=tk.LEFT, padx=(4, 12))

        ttk.Button(connection, text="Open / Restart Camera", command=self.open_camera).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Button(connection, text="Capture Frame", command=self.capture_frame).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Button(connection, text="Close", command=self.close).pack(side=tk.LEFT)

        aids = ttk.Frame(top)
        aids.pack(side=tk.TOP, fill=tk.X, pady=(8, 0))

        orientation = ttk.LabelFrame(aids, text="Orientation", padding=6)
        orientation.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 8))

        ttk.Label(orientation, text="Rotate").grid(row=0, column=0, sticky=tk.W)
        rotate_combo = ttk.Combobox(
            orientation,
            textvariable=self.rotate_var,
            values=tuple(str(value) for value in ROTATE_VALUES),
            width=5,
            state="readonly",
        )
        rotate_combo.grid(row=0, column=1, sticky=tk.W, padx=(4, 10))
        ttk.Label(orientation, text="deg").grid(row=0, column=2, sticky=tk.W)

        ttk.Checkbutton(orientation, text="Flip X", variable=self.flip_x_var).grid(
            row=0, column=3, sticky=tk.W, padx=(12, 0)
        )
        ttk.Checkbutton(orientation, text="Flip Y", variable=self.flip_y_var).grid(
            row=0, column=4, sticky=tk.W, padx=(8, 0)
        )

        ttk.Label(orientation, text="Fine rotation").grid(row=1, column=0, sticky=tk.W, pady=(6, 0))
        fine_scale = ttk.Scale(
            orientation,
            from_=-10.0,
            to=10.0,
            variable=self.fine_rotation_var,
            command=lambda _value: self._show_current_frame(),
        )
        fine_scale.grid(row=1, column=1, columnspan=4, sticky="ew", padx=(4, 8), pady=(6, 0))
        orientation.columnconfigure(2, weight=1)

        fine_buttons = ttk.Frame(orientation)
        fine_buttons.grid(row=2, column=1, columnspan=4, sticky=tk.W, pady=(4, 0))
        for label, delta in (("-1°", -1.0), ("-0.1°", -0.1), ("0°", 0.0), ("+0.1°", 0.1), ("+1°", 1.0)):
            if delta == 0.0:
                command = self.reset_fine_rotation
            else:
                command = lambda step=delta: self.bump_fine_rotation(step)
            ttk.Button(fine_buttons, text=label, width=6, command=command).pack(side=tk.LEFT, padx=(0, 3))

        self.fine_rotation_label = ttk.Label(orientation, width=8)
        self.fine_rotation_label.grid(row=1, column=5, sticky=tk.W, pady=(6, 0))
        self._update_fine_rotation_label()

        overlay = ttk.LabelFrame(aids, text="Overlay", padding=6)
        overlay.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Checkbutton(overlay, text="Crosshair", variable=self.show_crosshair_var).pack(anchor=tk.W)
        ttk.Checkbutton(overlay, text="Axis labels", variable=self.show_axis_labels_var).pack(anchor=tk.W)
        ttk.Checkbutton(overlay, text="Grid", variable=self.show_grid_var).pack(anchor=tk.W)

        self.preview_label = ttk.Label(self, anchor=tk.CENTER)
        self.preview_label.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        status = ttk.Label(self, textvariable=self.status_var, anchor=tk.W, padding=(8, 0, 8, 8))
        status.pack(side=tk.BOTTOM, fill=tk.X)

    def _register_orientation_traces(self) -> None:
        watched_vars: tuple[tk.Variable, ...] = (
            self.rotate_var,
            self.flip_x_var,
            self.flip_y_var,
            self.fine_rotation_var,
            self.show_crosshair_var,
            self.show_axis_labels_var,
            self.show_grid_var,
        )
        for variable in watched_vars:
            variable.trace_add("write", lambda *_args: self._on_orientation_changed())

    def _on_orientation_changed(self) -> None:
        self._normalize_fine_rotation_var()
        self._update_fine_rotation_label()
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
        angle = self._get_fine_rotation_degrees()
        self.fine_rotation_label.configure(text=f"{angle:+.1f}°")

    def bump_fine_rotation(self, delta_degrees: float) -> None:
        self.fine_rotation_var.set(self._clamp_fine_rotation(self._get_fine_rotation_degrees() + delta_degrees))

    def reset_fine_rotation(self) -> None:
        self.fine_rotation_var.set(0.0)

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

    def open_camera(self) -> None:
        """Open/reopen the selected camera index."""

        self.release_camera()
        self.current_frame_bgr = None

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
            self.status_var.set(
                f"Camera {index} did not open. Try index 1, check permissions, or verify the camera is connected."
            )
            return

        self.cap = cap
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.status_var.set(
            f"Camera {index} open. Actual frame: {actual_w} x {actual_h}. "
            "X+ is right and Y+ is up in the transformed preview."
        )
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
        if self.cap is None:
            return

        ok, frame = self.cap.read()
        if ok and frame is not None:
            self.current_frame_bgr = frame
            self._show_frame(frame)
        else:
            self.status_var.set("Camera read failed. Try Open / Restart Camera.")

        if self.cap is not None:
            self._schedule_next_frame()

    def _show_current_frame(self) -> None:
        if self.current_frame_bgr is not None:
            self._show_frame(self.current_frame_bgr)

    def get_transformed_frame_bgr(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Apply orientation settings to a raw BGR camera frame."""

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

    def _show_frame(self, frame_bgr: np.ndarray) -> None:
        frame_bgr = self.get_transformed_frame_bgr(frame_bgr)
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(frame_rgb)

        max_w = max(1, self.preview_label.winfo_width() - 20)
        max_h = max(1, self.preview_label.winfo_height() - 20)
        img_w, img_h = pil_image.size
        scale = min(max_w / img_w, max_h / img_h, 1.0)
        new_w = max(1, int(img_w * scale))
        new_h = max(1, int(img_h * scale))
        if (new_w, new_h) != pil_image.size:
            pil_image = pil_image.resize((new_w, new_h), Image.Resampling.LANCZOS)

        self._draw_overlay(pil_image)
        self._tk_preview = ImageTk.PhotoImage(pil_image)
        self.preview_label.configure(image=self._tk_preview)

    def _draw_overlay(self, pil_image: Image.Image) -> None:
        """Draw live alignment aids on the preview image only."""

        draw = ImageDraw.Draw(pil_image)
        w, h = pil_image.size
        cx = w // 2
        cy = h // 2

        if bool(self.show_grid_var.get()):
            grid_color = (170, 170, 170)
            for index in range(1, 10):
                x = round(index * w / 10)
                y = round(index * h / 10)
                draw.line((x, 0, x, h), fill=grid_color, width=1)
                draw.line((0, y, w, y), fill=grid_color, width=1)

        if bool(self.show_crosshair_var.get()):
            outline = (0, 0, 0)
            main = (255, 230, 0)
            draw.line((cx, 0, cx, h), fill=outline, width=5)
            draw.line((0, cy, w, cy), fill=outline, width=5)
            draw.line((cx, 0, cx, h), fill=main, width=2)
            draw.line((0, cy, w, cy), fill=main, width=2)
            r = 7
            draw.ellipse((cx - r, cy - r, cx + r, cy + r), outline=outline, width=4)
            draw.ellipse((cx - r, cy - r, cx + r, cy + r), outline=main, width=2)

        if bool(self.show_axis_labels_var.get()):
            self._draw_label(draw, "X+", (w - 34, cy - 24))
            self._draw_label(draw, "X-", (10, cy - 24))
            self._draw_label(draw, "Y+", (cx + 10, 10))
            self._draw_label(draw, "Y-", (cx + 10, h - 30))

    def _draw_label(self, draw: ImageDraw.ImageDraw, text: str, xy: tuple[int, int]) -> None:
        x, y = xy
        outline = (0, 0, 0)
        fill = (255, 255, 255)
        for dx, dy in ((-1, -1), (1, -1), (-1, 1), (1, 1), (0, 0)):
            draw.text((x + dx, y + dy), text, fill=outline)
        draw.text((x, y), text, fill=fill)

    def capture_frame(self) -> None:
        if self.current_frame_bgr is None:
            messagebox.showinfo("No frame", "No camera frame is available yet.", parent=self)
            return

        width, height = self._get_requested_size()
        self.result = CameraCaptureResult(
            frame_bgr=self.get_transformed_frame_bgr(self.current_frame_bgr.copy()),
            camera_index=self._get_camera_index(),
            requested_width=width,
            requested_height=height,
            rotate_degrees=self._get_rotate_degrees(),
            flip_x=bool(self.flip_x_var.get()),
            flip_y=bool(self.flip_y_var.get()),
            fine_rotation_degrees=self._get_fine_rotation_degrees(),
            show_crosshair=bool(self.show_crosshair_var.get()),
            show_axis_labels=bool(self.show_axis_labels_var.get()),
            show_grid=bool(self.show_grid_var.get()),
        )
        self.close()

    def close(self) -> None:
        self.release_camera()
        self.destroy()
