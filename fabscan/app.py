from __future__ import annotations

import math
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageTk
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk

from fabscan.camera_capture import CameraCaptureDialog
from fabscan.dxf_export import ExportOriginMode, export_contours_to_dxf, get_export_bbox_for_contours
from fabscan.image_processing import FoundContour, ProcessedImage, find_contours
from fabscan.scale_tools import ScaleResult, calculate_scale
from fabscan.settings import DEFAULT_SETTINGS, get_settings_path, load_settings, save_settings


ImagePoint = Tuple[float, float]

APP_VERSION = "0.2.3"
APP_TITLE = f"FabScan v{APP_VERSION} - Polish / Stability"


class FabScanApp(tk.Tk):
    """FabScan desktop app.

    This intentionally favors simple and debuggable over pretty. The goal is to
    prove the photo/scan -> contours -> scaled DXF workflow, with manual contour
    enable/disable before export.
    """

    def __init__(self) -> None:
        super().__init__()
        self.settings = load_settings()
        self._settings_save_job: Optional[str] = None

        self.title(APP_TITLE)
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
        self.noise_removal_var = tk.IntVar(value=int(self.settings.get("noise_removal", 0)))
        self.edge_cleanup_var = tk.IntVar(value=int(self.settings.get("edge_cleanup", 0)))
        self.min_area_var = tk.DoubleVar(value=float(self.settings.get("min_area", 1000.0)))
        self.simplify_var = tk.DoubleVar(value=float(self.settings.get("simplify_percent", 0.05)))
        self.invert_var = tk.BooleanVar(value=bool(self.settings.get("invert", False)))
        self.show_threshold_var = tk.BooleanVar(value=bool(self.settings.get("show_threshold", False)))
        self.sanity_expected_width_var = tk.DoubleVar(value=float(self.settings.get("sanity_expected_width_inches", 0.0)))
        self.sanity_expected_height_var = tk.DoubleVar(value=float(self.settings.get("sanity_expected_height_inches", 0.0)))
        self.sanity_tolerance_var = tk.DoubleVar(value=float(self.settings.get("sanity_tolerance_inches", 0.010)))
        self.export_origin_var = tk.StringVar(value=origin_label)
        self.export_margin_var = tk.DoubleVar(value=float(self.settings.get("export_margin_inches", 0.0)))

        contour_filter_label = str(self.settings.get("contour_filter_label", "All"))
        if contour_filter_label not in ("All", "Enabled only", "Disabled only", "OUTSIDE", "INSIDE"):
            contour_filter_label = "All"
        contour_sort_label = str(self.settings.get("contour_sort_label", "Layer + area"))
        if contour_sort_label not in ("Layer + area", "Area largest first", "Area smallest first", "ID", "Points most first"):
            contour_sort_label = "Layer + area"

        self.contour_filter_var = tk.StringVar(value=contour_filter_label)
        self.contour_sort_var = tk.StringVar(value=contour_sort_label)

        self._build_menu()
        self._build_ui()
        self._register_settings_traces()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_menu(self) -> None:
        """Create simple menus for help/about and reset actions."""

        menubar = tk.Menu(self)

        file_menu = tk.Menu(menubar, tearoff=False)
        file_menu.add_command(label="Reset Recommended Defaults", command=self.reset_recommended_defaults)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.on_close)
        menubar.add_cascade(label="File", menu=file_menu)

        help_menu = tk.Menu(menubar, tearoff=False)
        help_menu.add_command(label="Basic Workflow", command=self.show_workflow_help)
        help_menu.add_command(label="About FabScan", command=self.show_about)
        menubar.add_cascade(label="Help", menu=help_menu)

        self.configure(menu=menubar)

    def _build_ui(self) -> None:
        toolbar = ttk.Frame(self, padding=8)
        toolbar.pack(side=tk.TOP, fill=tk.X)

        ttk.Button(toolbar, text="Load Image", command=self.load_image).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(toolbar, text="Camera Capture", command=self.capture_camera_image).pack(side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="Find Contours", command=self.process_image).pack(side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="Set Scale", command=self.start_scale_mode).pack(side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="Export DXF", command=self.export_dxf).pack(side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="Reset Defaults", command=self.reset_recommended_defaults).pack(side=tk.LEFT, padx=(18, 6))
        ttk.Button(toolbar, text="Help", command=self.show_workflow_help).pack(side=tk.LEFT, padx=6)

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

        ttk.Label(
            toolbar,
            text="Tip: use low cleanup values first; high cleanup can change real geometry.",
        ).pack(side=tk.LEFT, padx=(18, 0))

        controls = ttk.LabelFrame(self, text="Image Cleanup", padding=8)
        controls.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(0, 8))

        self._add_slider(controls, "Threshold", self.threshold_var, 0, 255, 1, 0)
        self._add_slider(controls, "Blur", self.blur_var, 1, 21, 1, 1)
        self._add_slider(controls, "Noise Removal", self.noise_removal_var, 0, 5, 1, 2)
        self._add_slider(controls, "Edge Cleanup", self.edge_cleanup_var, 0, 5, 1, 3)
        self._add_slider(controls, "Min Area", self.min_area_var, 0, 50000, 100, 4)
        self._add_slider(controls, "Simplify %", self.simplify_var, 0.01, 1.0, 0.01, 5)

        main = ttk.Frame(self, padding=(8, 0, 8, 8))
        main.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(main, bg="#202020", highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<Configure>", lambda _event: self.redraw_preview())
        self.bind("t", lambda _event: self.toggle_selected_contour())
        self.bind("T", lambda _event: self.toggle_selected_contour())
        self.bind("<Escape>", lambda _event: self.cancel_modes())

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

        contour_view_row = ttk.Frame(contours_frame)
        contour_view_row.pack(side=tk.TOP, fill=tk.X, pady=(0, 6))

        ttk.Label(contour_view_row, text="Show").grid(row=0, column=0, sticky=tk.W)
        filter_combo = ttk.Combobox(
            contour_view_row,
            textvariable=self.contour_filter_var,
            values=("All", "Enabled only", "Disabled only", "OUTSIDE", "INSIDE"),
            state="readonly",
            width=13,
        )
        filter_combo.grid(row=1, column=0, sticky="ew", padx=(0, 4))
        filter_combo.bind("<<ComboboxSelected>>", lambda _event: self.on_contour_view_changed())

        ttk.Label(contour_view_row, text="Sort").grid(row=0, column=1, sticky=tk.W)
        sort_combo = ttk.Combobox(
            contour_view_row,
            textvariable=self.contour_sort_var,
            values=("Layer + area", "Area largest first", "Area smallest first", "ID", "Points most first"),
            state="readonly",
            width=17,
        )
        sort_combo.grid(row=1, column=1, sticky="ew", padx=(4, 0))
        sort_combo.bind("<<ComboboxSelected>>", lambda _event: self.on_contour_view_changed())
        contour_view_row.columnconfigure(0, weight=1)
        contour_view_row.columnconfigure(1, weight=1)

        columns = ("id", "enabled", "layer", "area", "points")
        self.contour_tree = ttk.Treeview(
            contours_frame,
            columns=columns,
            show="headings",
            height=12,
            selectmode="browse",
        )
        self.contour_tree.heading("id", text="ID")
        self.contour_tree.heading("enabled", text="On")
        self.contour_tree.heading("layer", text="Layer")
        self.contour_tree.heading("area", text="Area px²")
        self.contour_tree.heading("points", text="Pts")
        self.contour_tree.column("id", width=36, anchor=tk.E, stretch=False)
        self.contour_tree.column("enabled", width=42, anchor=tk.CENTER, stretch=False)
        self.contour_tree.column("layer", width=76, anchor=tk.CENTER, stretch=False)
        self.contour_tree.column("area", width=84, anchor=tk.E, stretch=True)
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

        visible_buttons = ttk.Frame(contours_frame)
        visible_buttons.pack(side=tk.TOP, fill=tk.X, pady=(6, 0))
        ttk.Button(visible_buttons, text="Enable Visible", command=self.enable_visible_contours).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 3)
        )
        ttk.Button(visible_buttons, text="Disable Visible", command=self.disable_visible_contours).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(3, 0)
        )

        ttk.Label(side, text="Measurements", font=("TkDefaultFont", 11, "bold")).pack(anchor=tk.W, pady=(8, 0))
        self.measurement_text = tk.Text(side, height=13, width=38, wrap=tk.WORD)
        self.measurement_text.pack(fill=tk.X, expand=False, pady=(4, 0))
        self.measurement_text.configure(state=tk.DISABLED)

        sanity_frame = ttk.LabelFrame(side, text="X/Y Sanity Check", padding=6)
        sanity_frame.pack(side=tk.TOP, fill=tk.X, pady=(8, 0))

        sanity_grid = ttk.Frame(sanity_frame)
        sanity_grid.pack(fill=tk.X)

        ttk.Label(sanity_grid, text="Expected W in").grid(row=0, column=0, sticky=tk.W, padx=(0, 4))
        expected_w_entry = ttk.Entry(sanity_grid, textvariable=self.sanity_expected_width_var, width=10)
        expected_w_entry.grid(row=1, column=0, sticky="ew", padx=(0, 4))

        ttk.Label(sanity_grid, text="Expected H in").grid(row=0, column=1, sticky=tk.W, padx=4)
        expected_h_entry = ttk.Entry(sanity_grid, textvariable=self.sanity_expected_height_var, width=10)
        expected_h_entry.grid(row=1, column=1, sticky="ew", padx=4)

        ttk.Label(sanity_grid, text="Tol +/- in").grid(row=0, column=2, sticky=tk.W, padx=(4, 0))
        tolerance_entry = ttk.Entry(sanity_grid, textvariable=self.sanity_tolerance_var, width=10)
        tolerance_entry.grid(row=1, column=2, sticky="ew", padx=(4, 0))

        sanity_grid.columnconfigure(0, weight=1)
        sanity_grid.columnconfigure(1, weight=1)
        sanity_grid.columnconfigure(2, weight=1)

        for entry in (expected_w_entry, expected_h_entry, tolerance_entry):
            entry.bind("<Return>", lambda _event: self.update_measurements())
            entry.bind("<FocusOut>", lambda _event: self.update_measurements())

        ttk.Label(
            sanity_frame,
            text="Compares the enabled contour bounding box against known CNC/part dimensions. It does not change scale.",
            wraplength=300,
        ).pack(anchor=tk.W, pady=(6, 0))

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
            "After Find Contours, select a contour to see its bounding box and scaled size.\n"
            "After scale is set, enter expected X/Y dimensions for a sanity check."
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
            self.noise_removal_var,
            self.edge_cleanup_var,
            self.min_area_var,
            self.simplify_var,
            self.invert_var,
            self.show_threshold_var,
            self.sanity_expected_width_var,
            self.sanity_expected_height_var,
            self.sanity_tolerance_var,
            self.export_origin_var,
            self.export_margin_var,
            self.contour_filter_var,
            self.contour_sort_var,
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
            "noise_removal": self.safe_int_from_var(self.noise_removal_var, 0),
            "edge_cleanup": self.safe_int_from_var(self.edge_cleanup_var, 0),
            "min_area": self.safe_float_from_var(self.min_area_var, 1000.0),
            "simplify_percent": self.safe_float_from_var(self.simplify_var, 0.05),
            "invert": bool(self.invert_var.get()),
            "show_threshold": bool(self.show_threshold_var.get()),
            "sanity_expected_width_inches": self.safe_float_from_var(self.sanity_expected_width_var, 0.0),
            "sanity_expected_height_inches": self.safe_float_from_var(self.sanity_expected_height_var, 0.0),
            "sanity_tolerance_inches": self.safe_float_from_var(self.sanity_tolerance_var, 0.010),
            "export_origin_label": str(self.export_origin_var.get()),
            "export_margin_inches": self.safe_float_from_var(self.export_margin_var, 0.0),
            "contour_filter_label": str(self.contour_filter_var.get()),
            "contour_sort_label": str(self.contour_sort_var.get()),
            "last_image_dir": str(self.settings.get("last_image_dir", Path.home())),
            "last_export_dir": str(self.settings.get("last_export_dir", Path.cwd() / "exports")),
            "last_capture_dir": str(self.settings.get("last_capture_dir", Path.home() / "Pictures" / "FabScan Captures")),
            "camera_index": int(self.settings.get("camera_index", 0)),
            "camera_width": int(self.settings.get("camera_width", 1280)),
            "camera_height": int(self.settings.get("camera_height", 720)),
            "camera_rotate_degrees": int(self.settings.get("camera_rotate_degrees", 0)),
            "camera_flip_x": bool(self.settings.get("camera_flip_x", False)),
            "camera_flip_y": bool(self.settings.get("camera_flip_y", False)),
            "camera_fine_rotation_degrees": float(self.settings.get("camera_fine_rotation_degrees", 0.0)),
            "camera_show_crosshair": bool(self.settings.get("camera_show_crosshair", True)),
            "camera_show_axis_labels": bool(self.settings.get("camera_show_axis_labels", True)),
            "camera_show_grid": bool(self.settings.get("camera_show_grid", False)),
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

    def show_workflow_help(self) -> None:
        """Show the basic FabScan workflow in a small help dialog."""

        messagebox.showinfo(
            "FabScan Basic Workflow",
            "Basic workflow:\n\n"
            "1. Load Image or use Camera Capture.\n"
            "2. Adjust Threshold / Blur / Noise Removal / Edge Cleanup.\n"
            "3. Click Find Contours.\n"
            "4. Enable/disable contours so only wanted geometry exports.\n"
            "5. Click Set Scale, pick two known points, and enter the real distance.\n"
            "6. Use the X/Y Sanity Check against known CNC/part dimensions.\n"
            "7. Export DXF and bring it into SheetCam/CAD for final cleanup.\n\n"
            "Tips:\n"
            "- Keep cleanup values low unless the camera image is ugly.\n"
            "- Use Show Threshold to see what FabScan is actually tracing.\n"
            "- Disabled contours stay visible in gray but do not export.\n"
            "- X+ is right and Y+ is up in the transformed camera preview.",
            parent=self,
        )

    def show_about(self) -> None:
        """Show version/about information."""

        messagebox.showinfo(
            "About FabScan",
            f"FabScan v{APP_VERSION}\n\n"
            "Photo/camera-to-DXF helper for flat plasma parts.\n\n"
            "Design goal: create usable DXF geometry quickly, then let SheetCam/CAD do final cleanup when needed.\n\n"
            f"Settings file:\n{get_settings_path()}",
            parent=self,
        )

    def reset_recommended_defaults(self) -> None:
        """Reset normal tracing/export controls to known-good default values."""

        confirm = messagebox.askyesno(
            "Reset defaults?",
            "Reset the main FabScan tracing/export controls to recommended defaults?\n\n"
            "Camera orientation, camera size, last folders, and window position will be kept.",
            parent=self,
        )
        if not confirm:
            return

        # Main image-processing controls.
        self.threshold_var.set(int(DEFAULT_SETTINGS["threshold"]))
        self.blur_var.set(int(DEFAULT_SETTINGS["blur"]))
        self.noise_removal_var.set(int(DEFAULT_SETTINGS["noise_removal"]))
        self.edge_cleanup_var.set(int(DEFAULT_SETTINGS["edge_cleanup"]))
        self.min_area_var.set(float(DEFAULT_SETTINGS["min_area"]))
        self.simplify_var.set(float(DEFAULT_SETTINGS["simplify_percent"]))
        self.invert_var.set(bool(DEFAULT_SETTINGS["invert"]))
        self.show_threshold_var.set(bool(DEFAULT_SETTINGS["show_threshold"]))

        # Measurement/export/list controls.
        self.sanity_expected_width_var.set(float(DEFAULT_SETTINGS["sanity_expected_width_inches"]))
        self.sanity_expected_height_var.set(float(DEFAULT_SETTINGS["sanity_expected_height_inches"]))
        self.sanity_tolerance_var.set(float(DEFAULT_SETTINGS["sanity_tolerance_inches"]))
        self.export_origin_var.set(str(DEFAULT_SETTINGS["export_origin_label"]))
        self.export_margin_var.set(float(DEFAULT_SETTINGS["export_margin_inches"]))
        self.contour_filter_var.set(str(DEFAULT_SETTINGS["contour_filter_label"]))
        self.contour_sort_var.set(str(DEFAULT_SETTINGS["contour_sort_label"]))

        # Existing contours were created using the old controls, so clear them.
        self.processed = None
        self.selected_contour_id = None
        self.scale_points = []
        self.scale_mode = False

        self.refresh_contour_list()
        self.update_measurements()
        self.redraw_preview()
        self.queue_save_settings()
        self.set_status(
            "Recommended defaults restored.\n\n"
            "Camera orientation/settings were left alone.\n"
            "Click Find Contours to process the current image with the restored settings."
        )

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

        image_path = Path(path)
        self.settings["last_image_dir"] = str(image_path.parent)
        self.queue_save_settings()

        self.set_current_image(
            image_bgr=image,
            image_path=image_path,
            source_text=f"Loaded image:\n{image_path}\n\nNext: adjust cleanup, click Find Contours, then set scale.",
        )

    def capture_camera_image(self) -> None:
        """Capture a still frame from a camera and load it as the working image."""

        camera_index = self.safe_int_from_settings("camera_index", 0)
        camera_width = self.safe_int_from_settings("camera_width", 1280)
        camera_height = self.safe_int_from_settings("camera_height", 720)
        camera_rotate_degrees = self.safe_int_from_settings("camera_rotate_degrees", 0)
        camera_flip_x = self.safe_bool_from_settings("camera_flip_x", False)
        camera_flip_y = self.safe_bool_from_settings("camera_flip_y", False)
        camera_fine_rotation_degrees = self.safe_float_from_settings("camera_fine_rotation_degrees", 0.0)
        camera_show_crosshair = self.safe_bool_from_settings("camera_show_crosshair", True)
        camera_show_axis_labels = self.safe_bool_from_settings("camera_show_axis_labels", True)
        camera_show_grid = self.safe_bool_from_settings("camera_show_grid", False)

        dialog = CameraCaptureDialog(
            self,
            camera_index=camera_index,
            camera_width=camera_width,
            camera_height=camera_height,
            rotate_degrees=camera_rotate_degrees,
            flip_x=camera_flip_x,
            flip_y=camera_flip_y,
            fine_rotation_degrees=camera_fine_rotation_degrees,
            show_crosshair=camera_show_crosshair,
            show_axis_labels=camera_show_axis_labels,
            show_grid=camera_show_grid,
        )
        self.wait_window(dialog)

        if dialog.result is None:
            return

        self.settings["camera_index"] = dialog.result.camera_index
        self.settings["camera_width"] = dialog.result.requested_width
        self.settings["camera_height"] = dialog.result.requested_height
        self.settings["camera_rotate_degrees"] = dialog.result.rotate_degrees
        self.settings["camera_flip_x"] = dialog.result.flip_x
        self.settings["camera_flip_y"] = dialog.result.flip_y
        self.settings["camera_fine_rotation_degrees"] = dialog.result.fine_rotation_degrees
        self.settings["camera_show_crosshair"] = dialog.result.show_crosshair
        self.settings["camera_show_axis_labels"] = dialog.result.show_axis_labels
        self.settings["camera_show_grid"] = dialog.result.show_grid

        capture_dir = Path(
            str(self.settings.get("last_capture_dir", Path.home() / "Pictures" / "FabScan Captures"))
        )
        try:
            capture_dir.mkdir(parents=True, exist_ok=True)
        except OSError:
            capture_dir = Path.home()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = capture_dir / f"fabscan_camera_{timestamp}.png"
        saved = cv2.imwrite(str(output_path), dialog.result.frame_bgr)

        orientation_text = (
            f"Rotate {dialog.result.rotate_degrees}°, "
            f"Flip X {'on' if dialog.result.flip_x else 'off'}, "
            f"Flip Y {'on' if dialog.result.flip_y else 'off'}, "
            f"Fine {dialog.result.fine_rotation_degrees:+.1f}°"
        )

        if saved:
            self.settings["last_capture_dir"] = str(output_path.parent)
            image_path = output_path
            source_text = (
                f"Captured from camera {dialog.result.camera_index}.\n"
                f"Saved PNG:\n{output_path}\n"
                f"Orientation: {orientation_text}\n\n"
                "Next: adjust cleanup, click Find Contours, then set scale."
            )
        else:
            image_path = Path(f"fabscan_camera_{timestamp}.png")
            source_text = (
                f"Captured from camera {dialog.result.camera_index}.\n"
                "Warning: frame was not saved to disk.\n"
                f"Orientation: {orientation_text}\n\n"
                "Next: adjust cleanup, click Find Contours, then set scale."
            )

        self.queue_save_settings()
        self.set_current_image(
            image_bgr=dialog.result.frame_bgr,
            image_path=image_path,
            source_text=source_text,
        )

    def safe_int_from_settings(self, key: str, default: int) -> int:
        try:
            return int(self.settings.get(key, default))
        except (TypeError, ValueError):
            return default

    def safe_float_from_settings(self, key: str, default: float) -> float:
        try:
            return float(self.settings.get(key, default))
        except (TypeError, ValueError):
            return default

    def safe_bool_from_settings(self, key: str, default: bool) -> bool:
        value = self.settings.get(key, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def set_current_image(self, *, image_bgr: np.ndarray, image_path: Path, source_text: str) -> None:
        """Load a BGR image array into the normal FabScan image pipeline."""

        self.image_path = image_path
        self.image_bgr = image_bgr
        self.processed = None
        self.scale_result = None
        self.scale_points = []
        self.scale_mode = False
        self.selected_contour_id = None

        h, w = image_bgr.shape[:2]
        self.set_status(f"{source_text}\n\nImage size: {w} x {h} px")
        self.refresh_contour_list()
        self.update_measurements()
        self.redraw_preview()

    def cancel_modes(self) -> None:
        """Cancel scale-picking mode."""

        if not self.scale_mode:
            return

        self.scale_mode = False
        self.scale_points = []
        self.append_status("\nCurrent pick mode cancelled.")
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
                noise_removal=int(self.noise_removal_var.get()),
                edge_cleanup=int(self.edge_cleanup_var.get()),
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

    def get_sanity_values(self) -> tuple[float, float, float]:
        """Return expected width, expected height, and tolerance in inches."""

        expected_width = max(0.0, self.safe_float_from_var(self.sanity_expected_width_var, 0.0))
        expected_height = max(0.0, self.safe_float_from_var(self.sanity_expected_height_var, 0.0))
        tolerance = max(0.0, self.safe_float_from_var(self.sanity_tolerance_var, 0.010))
        return expected_width, expected_height, tolerance

    def update_export_options_display(self) -> None:
        self.update_processing_status()
        self.update_measurements()

    def update_measurements(self) -> None:
        """Show selected contour and enabled-export bounding box sanity numbers."""

        if self.processed is None or not self.processed.contours:
            self.set_measurements(
                "No contours yet.\n\n"
                "After Find Contours, select a contour to see its bounding box and scaled size.\n"
                "After scale is set, enter expected X/Y dimensions for a sanity check."
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
        lines.append("X/Y sanity check:")
        if enabled_bbox is None:
            lines.append("No enabled contours")
        elif self.scale_result is None:
            lines.append("Set scale first")
        else:
            _min_x, _min_y, _max_x, _max_y, width_px, height_px = enabled_bbox
            measured_width = width_px * self.scale_result.inches_per_pixel
            measured_height = height_px * self.scale_result.inches_per_pixel
            expected_width, expected_height, tolerance = self.get_sanity_values()
            lines.append(f"Measured: {measured_width:.4f} x {measured_height:.4f} in")
            if expected_width <= 0 and expected_height <= 0:
                lines.append("Enter expected W/H to compare")
            else:
                if expected_width > 0:
                    error_x = measured_width - expected_width
                    status_x = "OK" if abs(error_x) <= tolerance else "CHECK"
                    lines.append(f"X expected: {expected_width:.4f} in")
                    lines.append(f"X error: {error_x:+.4f} in [{status_x}]")
                else:
                    lines.append("X expected: not set")

                if expected_height > 0:
                    error_y = measured_height - expected_height
                    status_y = "OK" if abs(error_y) <= tolerance else "CHECK"
                    lines.append(f"Y expected: {expected_height:.4f} in")
                    lines.append(f"Y error: {error_y:+.4f} in [{status_y}]")
                else:
                    lines.append("Y expected: not set")
                lines.append(f"Tolerance: ±{tolerance:.4f} in")

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
        visible_count = len(self.get_visible_contours())

        scale_text = "Not set"
        if self.scale_result is not None:
            scale_text = f"{self.scale_result.inches_per_pixel:.8f} in/px"

        self.set_status(
            f"Contours found: {len(self.processed.contours)}\n"
            f"Outside/Inside: {outside} / {inside}\n"
            f"Enabled: {len(enabled)} ({enabled_outside} OUT, {enabled_inside} IN)\n"
            f"Shown in list: {visible_count}\n"
            f"Points: {enabled_points} enabled / {total_points} total\n\n"
            f"Threshold: {int(self.threshold_var.get())}\n"
            f"Blur: {int(self.blur_var.get())}\n"
            f"Noise removal: {int(self.noise_removal_var.get())}\n"
            f"Edge cleanup: {int(self.edge_cleanup_var.get())}\n"
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

    def on_contour_view_changed(self) -> None:
        """Refresh the contour list after show/sort options change."""

        self.refresh_contour_list()
        self.update_processing_status()
        self.update_measurements()
        self.queue_save_settings()

    def contour_is_visible(self, contour: FoundContour) -> bool:
        """Return whether a contour should be shown in the list filter."""

        filter_label = self.contour_filter_var.get()
        if filter_label == "Enabled only":
            return contour.enabled
        if filter_label == "Disabled only":
            return not contour.enabled
        if filter_label == "OUTSIDE":
            return contour.layer == "OUTSIDE"
        if filter_label == "INSIDE":
            return contour.layer == "INSIDE"
        return True

    def get_visible_contours(self) -> list[FoundContour]:
        """Return contours after applying the list filter and sort mode."""

        if self.processed is None:
            return []

        contours = [contour for contour in self.processed.contours if self.contour_is_visible(contour)]
        sort_label = self.contour_sort_var.get()

        if sort_label == "Area largest first":
            return sorted(contours, key=lambda c: (-c.area, c.id))
        if sort_label == "Area smallest first":
            return sorted(contours, key=lambda c: (c.area, c.id))
        if sort_label == "ID":
            return sorted(contours, key=lambda c: c.id)
        if sort_label == "Points most first":
            return sorted(contours, key=lambda c: (-len(c.points), c.id))

        # Default shop-friendly sort: outside profile first, then inside cutouts,
        # with bigger contours listed before small specks/noise inside each layer.
        return sorted(contours, key=lambda c: (0 if c.layer == "OUTSIDE" else 1, -c.area, c.id))

    def refresh_contour_list(self) -> None:
        for item in self.contour_tree.get_children():
            self.contour_tree.delete(item)

        if self.processed is None:
            return

        visible_contours = self.get_visible_contours()
        visible_ids = {contour.id for contour in visible_contours}

        for contour in visible_contours:
            enabled_mark = "✓" if contour.enabled else ""
            self.contour_tree.insert(
                "",
                tk.END,
                iid=str(contour.id),
                values=(contour.id, enabled_mark, contour.layer, f"{contour.area:.0f}", len(contour.points)),
            )

        if self.selected_contour_id not in visible_ids:
            self.selected_contour_id = visible_contours[0].id if visible_contours else None

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

    def enable_visible_contours(self) -> None:
        """Enable only the contours currently visible in the filtered list."""

        if self.processed is None:
            return
        for contour in self.get_visible_contours():
            contour.enabled = True
        self.refresh_contour_list()
        self.update_processing_status()
        self.update_measurements()
        self.redraw_preview()

    def disable_visible_contours(self) -> None:
        """Disable only the contours currently visible in the filtered list."""

        if self.processed is None:
            return
        for contour in self.get_visible_contours():
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

        enabled_outside = sum(1 for contour in enabled_contours if contour.layer == "OUTSIDE")
        enabled_inside = sum(1 for contour in enabled_contours if contour.layer == "INSIDE")
        enabled_points = sum(len(contour.points) for contour in enabled_contours)

        export_lines = [
            "DXF exported successfully.",
            f"File: {output}",
            f"Contours: {len(enabled_contours)} total ({enabled_outside} OUTSIDE, {enabled_inside} INSIDE)",
            f"Points: {enabled_points}",
            f"Scale: {self.scale_result.inches_per_pixel:.8f} in/px",
            f"Origin: {self.format_export_origin()}",
        ]

        dxf_bbox = get_export_bbox_for_contours(
            contours=enabled_contours,
            scale_inches_per_pixel=self.scale_result.inches_per_pixel,
            image_height_pixels=image_height,
            origin_mode=self.get_export_origin_mode(),
            margin_inches=self.get_export_margin_inches(),
        )
        if dxf_bbox is not None:
            min_x, min_y, max_x, max_y, width_in, height_in = dxf_bbox
            export_lines.extend(
                [
                    f"DXF size: {width_in:.4f} x {height_in:.4f} in",
                    f"DXF X: {min_x:.4f} to {max_x:.4f} in",
                    f"DXF Y: {min_y:.4f} to {max_y:.4f} in",
                ]
            )

        export_lines.append("Next: import the DXF into SheetCam/CAD and run normal cleanup as needed.")
        export_message = "\n".join(export_lines)
        self.append_status("\n" + export_message)
        messagebox.showinfo("DXF exported", export_message)

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
