from __future__ import annotations

import math
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageTk
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk

from fabscan.dxf_export import ExportOriginMode, export_contours_to_dxf, get_export_bbox_for_contours
from fabscan.image_processing import FoundContour, ProcessedImage, find_contours
from fabscan.scale_tools import ScaleResult, calculate_scale
from fabscan.settings import get_settings_path, load_settings, save_settings


ImagePoint = Tuple[float, float]


class FabScanApp(tk.Tk):
    """FabScan Ver. 0.1.5 desktop app.

    This intentionally favors simple and debuggable over pretty. The goal is to
    prove the photo/scan -> contours -> scaled DXF workflow, with manual contour
    enable/disable before export.
    """

    def __init__(self) -> None:
        super().__init__()
        self.settings = load_settings()
        self._settings_save_job: Optional[str] = None

        self.title("FabScan Ver. 0.1.5 - Save Last Settings")
        try:
            self.geometry(str(self.settings.get("window_geometry", "1280x820")))
        except tk.TclError:
            self.geometry("1280x820")
        self.minsize(950, 600)

        self.image_path: Optional[Path] = None
        self.image_bgr: Optional[np.ndarray] = None
        self.processed: Optional[ProcessedImage] = None
        self.scale_result: Optional[ScaleResult] = None
        self.scale_points: list[ImagePoint] = []
        self.scale_mode = False
        self.selected_contour_id: Optional[int] = None

        self.display_scale = 1.0
        self.display_offset_x = 0.0
        self.display_offset_y = 0.0
        self._tk_image: Optional[ImageTk.PhotoImage] = None

        origin_label = str(self.settings.get("export_origin_label", "Move lower-left to 0,0"))
        if origin_label not in ("Move lower-left to 0,0", "Preserve image position", "Center on 0,0"):
            origin_label = "Move lower-left to 0,0"

        self.threshold_var = tk.IntVar(value=int(self.settings.get("threshold", 127)))
        self.blur_var = tk.IntVar(value=int(self.settings.get("blur", 3)))
        self.min_area_var = tk.DoubleVar(value=float(self.settings.get("min_area", 1000.0)))
        self.simplify_var = tk.DoubleVar(value=float(self.settings.get("simplify_percent", 0.05)))
        self.invert_var = tk.BooleanVar(value=bool(self.settings.get("invert", False)))
        self.show_threshold_var = tk.BooleanVar(value=bool(self.settings.get("show_threshold", False)))
        self.export_origin_var = tk.StringVar(value=origin_label)
        self.export_margin_var = tk.DoubleVar(value=float(self.settings.get("export_margin_inches", 0.0)))

        self._build_ui()
        self._register_settings_traces()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        toolbar = ttk.Frame(self, padding=8)
        toolbar.pack(side=tk.TOP, fill=tk.X)

        ttk.Button(toolbar, text="Load Image", command=self.load_image).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(toolbar, text="Find Contours", command=self.process_image).pack(side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="Set Scale", command=self.start_scale_mode).pack(side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="Export DXF", command=self.export_dxf).pack(side=tk.LEFT, padx=6)

        ttk.Checkbutton(
            toolbar,
            text="Invert",
            variable=self.invert_var,
            command=self.process_image_if_loaded,
        ).pack(side=tk.LEFT, padx=(18, 6))

        ttk.Checkbutton(
            toolbar,
            text="Show Threshold",
            variable=self.show_threshold_var,
            command=self.redraw_preview,
        ).pack(side=tk.LEFT, padx=6)

        ttk.Label(toolbar, text="Tip: select a contour in the list or click its edge; press T to toggle.").pack(
            side=tk.LEFT, padx=(18, 0)
        )

        controls = ttk.LabelFrame(self, text="Image Cleanup", padding=8)
        controls.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(0, 8))

        self._add_slider(controls, "Threshold", self.threshold_var, 0, 255, 1, 0)
        self._add_slider(controls, "Blur", self.blur_var, 1, 21, 1, 1)
        self._add_slider(controls, "Min Area", self.min_area_var, 0, 50000, 100, 2)
        self._add_slider(controls, "Simplify %", self.simplify_var, 0.01, 1.0, 0.01, 3)

        main = ttk.Frame(self, padding=(8, 0, 8, 8))
        main.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(main, bg="#202020", highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<Configure>", lambda _event: self.redraw_preview())
        self.bind("t", lambda _event: self.toggle_selected_contour())
        self.bind("T", lambda _event: self.toggle_selected_contour())

        side_container = ttk.Frame(main, padding=(8, 0, 0, 0), width=360)
        side_container.pack(side=tk.RIGHT, fill=tk.Y)
        side_container.pack_propagate(False)

        self.side_canvas = tk.Canvas(side_container, highlightthickness=0, borderwidth=0)
        self.side_scrollbar = ttk.Scrollbar(side_container, orient=tk.VERTICAL, command=self.side_canvas.yview)
        self.side_canvas.configure(yscrollcommand=self.side_scrollbar.set)

        self.side_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.side_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        side = ttk.Frame(self.side_canvas)
        self.side_canvas_window = self.side_canvas.create_window((0, 0), window=side, anchor=tk.NW)
        side.bind("<Configure>", lambda _event: self._update_side_scrollregion())
        self.side_canvas.bind("<Configure>", self._on_side_canvas_configure)
        self.side_canvas.bind("<Enter>", lambda _event: self._bind_side_mousewheel())
        self.side_canvas.bind("<Leave>", lambda _event: self._unbind_side_mousewheel())

        contours_frame = ttk.LabelFrame(side, text="Contours", padding=6)
        contours_frame.pack(side=tk.TOP, fill=tk.X)

        columns = ("enabled", "layer", "area", "points")
        self.contour_tree = ttk.Treeview(
            contours_frame,
            columns=columns,
            show="headings",
            height=12,
            selectmode="browse",
        )
        self.contour_tree.heading("enabled", text="On")
        self.contour_tree.heading("layer", text="Layer")
        self.contour_tree.heading("area", text="Area px²")
        self.contour_tree.heading("points", text="Pts")
        self.contour_tree.column("enabled", width=42, anchor=tk.CENTER, stretch=False)
        self.contour_tree.column("layer", width=76, anchor=tk.CENTER, stretch=False)
        self.contour_tree.column("area", width=92, anchor=tk.E, stretch=True)
        self.contour_tree.column("points", width=54, anchor=tk.E, stretch=False)
        self.contour_tree.pack(side=tk.TOP, fill=tk.X)
        self.contour_tree.bind("<<TreeviewSelect>>", self.on_contour_tree_select)
        self.contour_tree.bind("<Double-1>", lambda _event: self.toggle_selected_contour())

        contour_buttons = ttk.Frame(contours_frame)
        contour_buttons.pack(side=tk.TOP, fill=tk.X, pady=(6, 0))
        ttk.Button(contour_buttons, text="Toggle Selected", command=self.toggle_selected_contour).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 3)
        )
        ttk.Button(contour_buttons, text="Enable All", command=self.enable_all_contours).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=3
        )
        ttk.Button(contour_buttons, text="Disable All", command=self.disable_all_contours).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(3, 0)
        )

        ttk.Label(side, text="Measurements", font=("TkDefaultFont", 11, "bold")).pack(anchor=tk.W, pady=(8, 0))
        self.measurement_text = tk.Text(side, height=13, width=38, wrap=tk.WORD)
        self.measurement_text.pack(fill=tk.X, expand=False, pady=(4, 0))
        self.measurement_text.configure(state=tk.DISABLED)

        export_frame = ttk.LabelFrame(side, text="DXF Export", padding=6)
        export_frame.pack(side=tk.TOP, fill=tk.X, pady=(8, 0))

        ttk.Label(export_frame, text="Origin").pack(anchor=tk.W)
        origin_combo = ttk.Combobox(
            export_frame,
            textvariable=self.export_origin_var,
            values=(
                "Move lower-left to 0,0",
                "Preserve image position",
                "Center on 0,0",
            ),
            state="readonly",
        )
        origin_combo.pack(fill=tk.X, pady=(2, 6))
        origin_combo.bind("<<ComboboxSelected>>", lambda _event: self.update_export_options_display())

        margin_row = ttk.Frame(export_frame)
        margin_row.pack(fill=tk.X)
        ttk.Label(margin_row, text="Margin in").pack(side=tk.LEFT)
        margin_entry = ttk.Entry(margin_row, textvariable=self.export_margin_var, width=10)
        margin_entry.pack(side=tk.RIGHT)
        margin_entry.bind("<Return>", lambda _event: self.update_export_options_display())
        margin_entry.bind("<FocusOut>", lambda _event: self.update_export_options_display())

        ttk.Label(
            export_frame,
            text="Margin applies to lower-left origin mode.",
            wraplength=300,
        ).pack(anchor=tk.W, pady=(6, 0))

        ttk.Label(side, text="Status", font=("TkDefaultFont", 11, "bold")).pack(anchor=tk.W, pady=(8, 0))
        self.status_text = tk.Text(side, height=9, width=38, wrap=tk.WORD)
        self.status_text.pack(fill=tk.X, expand=False, pady=(4, 0))
        self.status_text.configure(state=tk.DISABLED)

        self.set_status(
            "Load a clean photo/scan of a flat part.\n\n"
            "Tip: For Ver. 0.1, a high-contrast image with the part separated "
            "from the background will work best."
        )
        self.set_measurements(
            "No contours yet.\n\n"
            "After Find Contours, select a contour to see its bounding box and scaled size."
        )
        self.refresh_contour_list()

    def _update_side_scrollregion(self) -> None:
        """Keep the right-side scrollbar matched to the controls panel height."""

        self.side_canvas.configure(scrollregion=self.side_canvas.bbox("all"))

    def _on_side_canvas_configure(self, event: tk.Event) -> None:
        """Make the scrollable right-side frame track the canvas width."""

        self.side_canvas.itemconfigure(self.side_canvas_window, width=event.width)
        self._update_side_scrollregion()

    def _bind_side_mousewheel(self) -> None:
        self.bind_all("<MouseWheel>", self._on_side_mousewheel)
        self.bind_all("<Button-4>", self._on_side_mousewheel)
        self.bind_all("<Button-5>", self._on_side_mousewheel)

    def _unbind_side_mousewheel(self) -> None:
        self.unbind_all("<MouseWheel>")
        self.unbind_all("<Button-4>")
        self.unbind_all("<Button-5>")

    def _on_side_mousewheel(self, event: tk.Event) -> None:
        if getattr(event, "num", None) == 4:
            self.side_canvas.yview_scroll(-1, "units")
        elif getattr(event, "num", None) == 5:
            self.side_canvas.yview_scroll(1, "units")
        else:
            self.side_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def _add_slider(
        self,
        parent: ttk.Frame,
        label: str,
        variable: tk.Variable,
        from_value: float,
        to_value: float,
        resolution: float,
        column: int,
    ) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=column, sticky="ew", padx=6)
        parent.columnconfigure(column, weight=1)

        ttk.Label(frame, text=label).pack(anchor=tk.W)
        scale = ttk.Scale(
            frame,
            from_=from_value,
            to=to_value,
            variable=variable,
            command=lambda _value: self.redraw_preview(),
        )
        scale.pack(fill=tk.X)

        entry = ttk.Entry(frame, textvariable=variable, width=10)
        entry.pack(anchor=tk.W, pady=(2, 0))

    def _register_settings_traces(self) -> None:
        """Save control/export settings shortly after the user changes them."""

        watched_vars: tuple[tk.Variable, ...] = (
            self.threshold_var,
            self.blur_var,
            self.min_area_var,
            self.simplify_var,
            self.invert_var,
            self.show_threshold_var,
            self.export_origin_var,
            self.export_margin_var,
        )

        for variable in watched_vars:
            variable.trace_add("write", lambda *_args: self.queue_save_settings())

    def safe_int_from_var(self, variable: tk.Variable, default: int) -> int:
        try:
            return int(variable.get())
        except (tk.TclError, TypeError, ValueError):
            return default

    def safe_float_from_var(self, variable: tk.Variable, default: float) -> float:
        try:
            return float(variable.get())
        except (tk.TclError, TypeError, ValueError):
            return default

    def collect_settings(self) -> dict[str, object]:
        """Collect the last-used user settings that should persist between runs."""

        return {
            "window_geometry": self.geometry(),
            "threshold": self.safe_int_from_var(self.threshold_var, 127),
            "blur": self.safe_int_from_var(self.blur_var, 3),
            "min_area": self.safe_float_from_var(self.min_area_var, 1000.0),
            "simplify_percent": self.safe_float_from_var(self.simplify_var, 0.05),
            "invert": bool(self.invert_var.get()),
            "show_threshold": bool(self.show_threshold_var.get()),
            "export_origin_label": str(self.export_origin_var.get()),
            "export_margin_inches": self.safe_float_from_var(self.export_margin_var, 0.0),
            "last_image_dir": str(self.settings.get("last_image_dir", Path.home())),
            "last_export_dir": str(self.settings.get("last_export_dir", Path.cwd() / "exports")),
        }

    def queue_save_settings(self) -> None:
        """Debounce settings saves so slider movement does not write constantly."""

        if self._settings_save_job is not None:
            self.after_cancel(self._settings_save_job)
        self._settings_save_job = self.after(500, self.save_settings_now)

    def save_settings_now(self) -> None:
        self._settings_save_job = None
        self.settings.update(self.collect_settings())
        save_settings(self.settings)

    def on_close(self) -> None:
        if self._settings_save_job is not None:
            self.after_cancel(self._settings_save_job)
            self._settings_save_job = None
        self.save_settings_now()
        self.destroy()

    def set_status(self, text: str) -> None:
        self.status_text.configure(state=tk.NORMAL)
        self.status_text.delete("1.0", tk.END)
        self.status_text.insert(tk.END, text)
        self.status_text.configure(state=tk.DISABLED)

    def set_measurements(self, text: str) -> None:
        self.measurement_text.configure(state=tk.NORMAL)
        self.measurement_text.delete("1.0", tk.END)
        self.measurement_text.insert(tk.END, text)
        self.measurement_text.configure(state=tk.DISABLED)

    def append_status(self, text: str) -> None:
        self.status_text.configure(state=tk.NORMAL)
        self.status_text.insert(tk.END, "\n" + text)
        self.status_text.configure(state=tk.DISABLED)
        self.status_text.see(tk.END)

    def load_image(self) -> None:
        initial_dir = Path(str(self.settings.get("last_image_dir", Path.home())))
        if not initial_dir.exists():
            initial_dir = Path.home()

        path = filedialog.askopenfilename(
            title="Load part image",
            initialdir=str(initial_dir),
            filetypes=(
                ("Image files", "*.png *.jpg *.jpeg *.bmp *.tif *.tiff"),
                ("All files", "*.*"),
            ),
        )
        if not path:
            return

        image = cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None:
            messagebox.showerror("Load failed", "OpenCV could not load that image.")
            return

        self.image_path = Path(path)
        self.settings["last_image_dir"] = str(self.image_path.parent)
        self.queue_save_settings()

        self.image_bgr = image
        self.processed = None
        self.scale_result = None
        self.scale_points = []
        self.scale_mode = False
        self.selected_contour_id = None

        h, w = image.shape[:2]
        self.set_status(f"Loaded:\n{self.image_path.name}\n\nImage size: {w} x {h} px")
        self.refresh_contour_list()
        self.update_measurements()
        self.redraw_preview()

    def process_image_if_loaded(self) -> None:
        if self.image_bgr is not None:
            self.process_image()

    def process_image(self) -> None:
        if self.image_bgr is None:
            messagebox.showinfo("No image", "Load an image first.")
            return

        try:
            self.processed = find_contours(
                image_bgr=self.image_bgr,
                threshold_value=int(self.threshold_var.get()),
                blur_size=int(self.blur_var.get()),
                invert=bool(self.invert_var.get()),
                min_area=float(self.min_area_var.get()),
                simplify_percent=float(self.simplify_var.get()),
            )
        except Exception as exc:  # noqa: BLE001 - show the real problem in the UI
            messagebox.showerror("Processing failed", str(exc))
            return

        self.selected_contour_id = self.processed.contours[0].id if self.processed.contours else None
        self.refresh_contour_list()
        self.update_processing_status()
        self.update_measurements()
        self.redraw_preview()

    def get_contour_bbox(self, contour: FoundContour) -> tuple[float, float, float, float, float, float]:
        """Return min_x, min_y, max_x, max_y, width, height in image pixels."""

        min_x = float(np.min(contour.points[:, 0]))
        max_x = float(np.max(contour.points[:, 0]))
        min_y = float(np.min(contour.points[:, 1]))
        max_y = float(np.max(contour.points[:, 1]))
        return min_x, min_y, max_x, max_y, max_x - min_x, max_y - min_y

    def get_contours_bbox(self, contours: list[FoundContour]) -> Optional[tuple[float, float, float, float, float, float]]:
        """Return one bounding box around a group of contours in image pixels."""

        if not contours:
            return None

        all_points = np.vstack([c.points for c in contours])
        min_x = float(np.min(all_points[:, 0]))
        max_x = float(np.max(all_points[:, 0]))
        min_y = float(np.min(all_points[:, 1]))
        max_y = float(np.max(all_points[:, 1]))
        return min_x, min_y, max_x, max_y, max_x - min_x, max_y - min_y

    def format_px(self, value: float) -> str:
        return f"{value:.1f} px"

    def format_inches(self, pixels: float) -> str:
        if self.scale_result is None:
            return "scale not set"
        return f"{pixels * self.scale_result.inches_per_pixel:.4f} in"

    def format_area_inches(self, area_pixels: float) -> str:
        if self.scale_result is None:
            return "scale not set"
        area = area_pixels * (self.scale_result.inches_per_pixel ** 2)
        return f"{area:.4f} in²"

    def format_export_origin(self) -> str:
        mode = self.get_export_origin_mode()
        if mode == "lower_left":
            margin = self.get_export_margin_inches()
            if margin > 0:
                return f"Lower-left at {margin:.4f}, {margin:.4f}"
            return "Lower-left at 0,0"
        if mode == "center":
            return "Center at 0,0"
        return "Preserve image position"

    def get_export_origin_mode(self) -> ExportOriginMode:
        label = self.export_origin_var.get()
        if label == "Move lower-left to 0,0":
            return "lower_left"
        if label == "Center on 0,0":
            return "center"
        return "preserve"

    def get_export_margin_inches(self) -> float:
        try:
            margin = float(self.export_margin_var.get())
        except (tk.TclError, ValueError):
            return 0.0
        return max(0.0, margin)

    def update_export_options_display(self) -> None:
        self.update_processing_status()
        self.update_measurements()

    def update_measurements(self) -> None:
        """Show selected contour and enabled-export bounding box sanity numbers."""

        if self.processed is None or not self.processed.contours:
            self.set_measurements(
                "No contours yet.\n\n"
                "After Find Contours, select a contour to see its bounding box and scaled size."
            )
            return

        contour = self.get_selected_contour()
        enabled_contours = [c for c in self.processed.contours if c.enabled]
        enabled_bbox = self.get_contours_bbox(enabled_contours)

        lines: list[str] = []

        if contour is None:
            lines.append("Selected: none")
        else:
            min_x, min_y, max_x, max_y, width_px, height_px = self.get_contour_bbox(contour)
            lines.extend(
                [
                    f"Selected contour: {contour.id}",
                    f"Layer: {contour.layer}",
                    f"Enabled: {'Yes' if contour.enabled else 'No'}",
                    f"Points: {len(contour.points)}",
                    f"Area: {contour.area:.0f} px²",
                    f"Area: {self.format_area_inches(contour.area)}",
                    "",
                    f"BBox X: {self.format_px(min_x)} to {self.format_px(max_x)}",
                    f"BBox Y: {self.format_px(min_y)} to {self.format_px(max_y)}",
                    f"Size: {self.format_px(width_px)} x {self.format_px(height_px)}",
                    f"Size: {self.format_inches(width_px)} x {self.format_inches(height_px)}",
                ]
            )

        lines.append("")
        lines.append("Enabled image bbox:")
        if enabled_bbox is None:
            lines.append("No enabled contours")
        else:
            _min_x, _min_y, _max_x, _max_y, width_px, height_px = enabled_bbox
            lines.append(f"Size: {self.format_px(width_px)} x {self.format_px(height_px)}")
            lines.append(f"Size: {self.format_inches(width_px)} x {self.format_inches(height_px)}")

        lines.append("")
        lines.append("DXF output bbox:")
        if enabled_bbox is None:
            lines.append("No enabled contours")
        elif self.scale_result is None or self.image_bgr is None:
            lines.append("Set scale to calculate")
        else:
            image_height = int(self.image_bgr.shape[0])
            dxf_bbox = get_export_bbox_for_contours(
                contours=enabled_contours,
                scale_inches_per_pixel=self.scale_result.inches_per_pixel,
                image_height_pixels=image_height,
                origin_mode=self.get_export_origin_mode(),
                margin_inches=self.get_export_margin_inches(),
            )
            if dxf_bbox is None:
                lines.append("No enabled contours")
            else:
                min_x, min_y, max_x, max_y, width_in, height_in = dxf_bbox
                lines.append(self.format_export_origin())
                lines.append(f"X: {min_x:.4f} to {max_x:.4f} in")
                lines.append(f"Y: {min_y:.4f} to {max_y:.4f} in")
                lines.append(f"Size: {width_in:.4f} x {height_in:.4f} in")

        self.set_measurements("\n".join(lines))

    def update_processing_status(self) -> None:
        if self.processed is None:
            return

        outside = sum(1 for c in self.processed.contours if c.layer == "OUTSIDE")
        inside = sum(1 for c in self.processed.contours if c.layer == "INSIDE")
        enabled = [c for c in self.processed.contours if c.enabled]
        enabled_outside = sum(1 for c in enabled if c.layer == "OUTSIDE")
        enabled_inside = sum(1 for c in enabled if c.layer == "INSIDE")
        total_points = sum(len(c.points) for c in self.processed.contours)
        enabled_points = sum(len(c.points) for c in enabled)

        scale_text = "Not set"
        if self.scale_result is not None:
            scale_text = f"{self.scale_result.inches_per_pixel:.8f} in/px"

        self.set_status(
            f"Contours found: {len(self.processed.contours)}\n"
            f"Outside/Inside: {outside} / {inside}\n"
            f"Enabled: {len(enabled)} ({enabled_outside} OUT, {enabled_inside} IN)\n"
            f"Points: {enabled_points} enabled / {total_points} total\n\n"
            f"Threshold: {int(self.threshold_var.get())}\n"
            f"Blur: {int(self.blur_var.get())}\n"
            f"Min area: {float(self.min_area_var.get()):.1f}\n"
            f"Simplify: {float(self.simplify_var.get()):.2f}%\n\n"
            f"Scale: {scale_text}\n"
            f"DXF origin: {self.format_export_origin()}"
        )

    def start_scale_mode(self) -> None:
        if self.image_bgr is None:
            messagebox.showinfo("No image", "Load an image first.")
            return
        self.scale_mode = True
        self.scale_points = []
        self.scale_result = None
        self.append_status("\nScale mode: click two known points on the image.")
        self.redraw_preview()

    def on_canvas_click(self, event: tk.Event) -> None:
        if self.image_bgr is None:
            return

        image_point = self.canvas_to_image_point(event.x, event.y)
        if image_point is None:
            return

        if self.scale_mode:
            self.handle_scale_click(image_point)
            return

        self.select_nearest_contour(image_point)

    def handle_scale_click(self, image_point: ImagePoint) -> None:
        self.scale_points.append(image_point)
        self.redraw_preview()

        if len(self.scale_points) == 2:
            known = simpledialog.askfloat(
                "Known distance",
                "Enter the real distance between the two points in inches:",
                minvalue=0.0001,
            )
            if known is None:
                self.scale_points = []
                self.scale_mode = False
                self.redraw_preview()
                return

            try:
                self.scale_result = calculate_scale(self.scale_points[0], self.scale_points[1], known)
            except Exception as exc:  # noqa: BLE001
                messagebox.showerror("Scale failed", str(exc))
                self.scale_points = []
                self.scale_mode = False
                self.redraw_preview()
                return

            self.scale_mode = False
            self.append_status(
                f"\nScale set:\n"
                f"Pixels: {self.scale_result.pixels:.3f}\n"
                f"Known distance: {self.scale_result.inches:.4f} in\n"
                f"Scale: {self.scale_result.inches_per_pixel:.8f} in/px"
            )
            if self.processed is not None:
                self.update_processing_status()
                self.update_measurements()
            self.redraw_preview()

    def canvas_to_image_point(self, canvas_x: float, canvas_y: float) -> Optional[ImagePoint]:
        if self.image_bgr is None:
            return None

        h, w = self.image_bgr.shape[:2]
        x = (canvas_x - self.display_offset_x) / self.display_scale
        y = (canvas_y - self.display_offset_y) / self.display_scale

        if x < 0 or y < 0 or x >= w or y >= h:
            return None
        return (x, y)

    def image_to_canvas_point(self, image_x: float, image_y: float) -> Tuple[float, float]:
        return (
            self.display_offset_x + image_x * self.display_scale,
            self.display_offset_y + image_y * self.display_scale,
        )

    def refresh_contour_list(self) -> None:
        for item in self.contour_tree.get_children():
            self.contour_tree.delete(item)

        if self.processed is None:
            return

        for contour in self.processed.contours:
            enabled_mark = "✓" if contour.enabled else ""
            self.contour_tree.insert(
                "",
                tk.END,
                iid=str(contour.id),
                values=(enabled_mark, contour.layer, f"{contour.area:.0f}", len(contour.points)),
            )

        if self.selected_contour_id is not None and str(self.selected_contour_id) in self.contour_tree.get_children():
            self.contour_tree.selection_set(str(self.selected_contour_id))
            self.contour_tree.focus(str(self.selected_contour_id))

    def on_contour_tree_select(self, _event: tk.Event) -> None:
        selection = self.contour_tree.selection()
        if not selection:
            return
        self.selected_contour_id = int(selection[0])
        self.update_measurements()
        self.redraw_preview()

    def get_selected_contour(self) -> Optional[FoundContour]:
        if self.processed is None or self.selected_contour_id is None:
            return None
        for contour in self.processed.contours:
            if contour.id == self.selected_contour_id:
                return contour
        return None

    def toggle_selected_contour(self) -> None:
        contour = self.get_selected_contour()
        if contour is None:
            return
        contour.enabled = not contour.enabled
        self.refresh_contour_list()
        self.update_processing_status()
        self.update_measurements()
        self.redraw_preview()

    def enable_all_contours(self) -> None:
        if self.processed is None:
            return
        for contour in self.processed.contours:
            contour.enabled = True
        self.refresh_contour_list()
        self.update_processing_status()
        self.update_measurements()
        self.redraw_preview()

    def disable_all_contours(self) -> None:
        if self.processed is None:
            return
        for contour in self.processed.contours:
            contour.enabled = False
        self.refresh_contour_list()
        self.update_processing_status()
        self.update_measurements()
        self.redraw_preview()

    def select_nearest_contour(self, image_point: ImagePoint) -> None:
        if self.processed is None or not self.processed.contours:
            return

        click_tolerance_px = max(3.0, 12.0 / max(self.display_scale, 0.001))
        best_id: Optional[int] = None
        best_distance = math.inf

        for contour in self.processed.contours:
            contour_points = contour.points.astype(np.float32).reshape(-1, 1, 2)
            distance = abs(float(cv2.pointPolygonTest(contour_points, image_point, True)))
            if distance < best_distance:
                best_distance = distance
                best_id = contour.id

        if best_id is not None and best_distance <= click_tolerance_px:
            self.selected_contour_id = best_id
            self.refresh_contour_list()
            self.update_measurements()
            self.redraw_preview()

    def export_dxf(self) -> None:
        if self.image_bgr is None:
            messagebox.showinfo("No image", "Load an image first.")
            return

        if self.processed is None or not self.processed.contours:
            self.process_image()
            if self.processed is None or not self.processed.contours:
                messagebox.showinfo("No contours", "No contours found to export.")
                return

        if self.scale_result is None:
            messagebox.showinfo("Scale required", "Set the scale before exporting a DXF.")
            return

        enabled_contours = [c for c in self.processed.contours if c.enabled]
        if not enabled_contours:
            messagebox.showinfo("No enabled contours", "Enable at least one contour before exporting.")
            return

        default_name = "fabscan_export.dxf"
        if self.image_path is not None:
            default_name = self.image_path.stem + ".dxf"

        initial_export_dir = Path(str(self.settings.get("last_export_dir", Path.cwd() / "exports")))
        if not initial_export_dir.exists():
            initial_export_dir.mkdir(parents=True, exist_ok=True)

        path = filedialog.asksaveasfilename(
            title="Export DXF",
            defaultextension=".dxf",
            initialfile=default_name,
            initialdir=str(initial_export_dir),
            filetypes=(("DXF files", "*.dxf"), ("All files", "*.*")),
        )
        if not path:
            return

        self.settings["last_export_dir"] = str(Path(path).parent)
        self.queue_save_settings()

        try:
            image_height = int(self.image_bgr.shape[0])
            output = export_contours_to_dxf(
                contours=enabled_contours,
                output_path=path,
                scale_inches_per_pixel=self.scale_result.inches_per_pixel,
                image_height_pixels=image_height,
                origin_mode=self.get_export_origin_mode(),
                margin_inches=self.get_export_margin_inches(),
            )
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("DXF export failed", str(exc))
            return

        self.append_status(
            f"\nDXF exported:\n{output}\n"
            f"Contours exported: {len(enabled_contours)}\n"
            f"Origin: {self.format_export_origin()}"
        )
        messagebox.showinfo("DXF exported", f"Saved:\n{output}")

    def redraw_preview(self) -> None:
        self.canvas.delete("all")
        if self.image_bgr is None:
            self.canvas.create_text(
                self.canvas.winfo_width() / 2,
                self.canvas.winfo_height() / 2,
                text="Load an image to start",
                fill="white",
                font=("TkDefaultFont", 18),
            )
            return

        display_rgb = self._make_display_rgb()
        pil_image = Image.fromarray(display_rgb)

        canvas_w = max(1, self.canvas.winfo_width())
        canvas_h = max(1, self.canvas.winfo_height())
        img_w, img_h = pil_image.size
        self.display_scale = min(canvas_w / img_w, canvas_h / img_h)
        new_w = max(1, int(img_w * self.display_scale))
        new_h = max(1, int(img_h * self.display_scale))
        self.display_offset_x = (canvas_w - new_w) / 2
        self.display_offset_y = (canvas_h - new_h) / 2

        pil_image = pil_image.resize((new_w, new_h), Image.Resampling.LANCZOS)
        draw = ImageDraw.Draw(pil_image)
        self._draw_overlay(draw)

        self._tk_image = ImageTk.PhotoImage(pil_image)
        self.canvas.create_image(
            self.display_offset_x,
            self.display_offset_y,
            anchor=tk.NW,
            image=self._tk_image,
        )

    def _make_display_rgb(self) -> np.ndarray:
        if self.image_bgr is None:
            raise RuntimeError("No image loaded")

        if self.show_threshold_var.get() and self.processed is not None:
            threshold = self.processed.threshold_image
            return cv2.cvtColor(threshold, cv2.COLOR_GRAY2RGB)

        return cv2.cvtColor(self.image_bgr, cv2.COLOR_BGR2RGB)

    def _draw_overlay(self, draw: ImageDraw.ImageDraw) -> None:
        if self.processed is not None:
            # Draw disabled first so enabled/selected contours stay visually dominant.
            for contour in self.processed.contours:
                if not contour.enabled:
                    self._draw_contour(draw, contour)
            for contour in self.processed.contours:
                if contour.enabled and contour.id != self.selected_contour_id:
                    self._draw_contour(draw, contour)
            for contour in self.processed.contours:
                if contour.enabled and contour.id == self.selected_contour_id:
                    self._draw_contour(draw, contour)

        if self.scale_points:
            scaled_points = [
                (x * self.display_scale, y * self.display_scale) for x, y in self.scale_points
            ]
            for x, y in scaled_points:
                radius = 5
                draw.ellipse((x - radius, y - radius, x + radius, y + radius), outline="yellow", width=2)
            if len(scaled_points) == 2:
                draw.line(scaled_points, fill="yellow", width=2)

    def _draw_contour(self, draw: ImageDraw.ImageDraw, contour: FoundContour) -> None:
        points = [(x * self.display_scale, y * self.display_scale) for x, y in contour.points]
        if len(points) < 2:
            return

        if not contour.enabled:
            color = "#777777"
            width = 1
        elif contour.id == self.selected_contour_id:
            color = "yellow"
            width = 4
        else:
            color = "lime" if contour.layer == "OUTSIDE" else "cyan"
            width = 2

        draw.line(points + [points[0]], fill=color, width=width)


def main() -> None:
    app = FabScanApp()
    app.mainloop()


if __name__ == "__main__":
    main()
