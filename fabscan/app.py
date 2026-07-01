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
from fabscan.camera_calibration import CameraCalibrationDialog
from fabscan.dxf_export import (
    ExportOriginMode,
    export_contours_to_dxf,
    export_trace_groups_to_dxf,
    get_export_bbox_for_contours,
)
from fabscan.image_processing import FoundContour, ProcessedImage, find_contours
from fabscan.linuxcnc_status import LinuxCNCPositionStatus, LinuxCNCStatusReader
from fabscan.scale_tools import ScaleResult, calculate_scale
from fabscan.settings import DEFAULT_SETTINGS, get_settings_path, load_settings, save_settings


ImagePoint = Tuple[float, float]

APP_VERSION = "0.5.9"
APP_TITLE = f"FabScan v{APP_VERSION} - Follow Direction Latch"


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

        self.linuxcnc_reader = LinuxCNCStatusReader()
        self.latest_linuxcnc_status: Optional[LinuxCNCPositionStatus] = None
        self.linuxcnc_auto_refresh_job: Optional[str] = None
        self.jog_busy = False
        self.jog_release_job: Optional[str] = None
        self.motion_busy = False
        self.motion_monitor_job: Optional[str] = None
        self.trace_groups: list[list[tuple[float, float, float]]] = [[]]
        self.trace_entities: list[Optional[dict[str, object]]] = [None]
        self.active_trace_index = 0
        self.trace_arc_center: Optional[tuple[float, float, float]] = None

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

        linuxcnc_coord_label = str(self.settings.get("linuxcnc_coord_mode_label", "Work coordinates"))
        if linuxcnc_coord_label not in ("Work coordinates", "Machine coordinates"):
            linuxcnc_coord_label = "Work coordinates"
        self.linuxcnc_coord_mode_var = tk.StringVar(value=linuxcnc_coord_label)
        self.linuxcnc_auto_refresh_var = tk.BooleanVar(value=bool(self.settings.get("linuxcnc_auto_refresh", False)))
        self.trace_closed_var = tk.BooleanVar(value=bool(self.settings.get("trace_closed", True)))
        self.trace_preview_var = tk.BooleanVar(value=bool(self.settings.get("trace_preview", True)))
        self.trace_show_live_position_var = tk.BooleanVar(value=bool(self.settings.get("trace_show_live_position", True)))
        self.trace_show_point_numbers_var = tk.BooleanVar(value=bool(self.settings.get("trace_show_point_numbers", True)))
        self.jog_controls_enabled_var = tk.BooleanVar(value=False)
        self.jog_step_var = tk.DoubleVar(value=float(self.settings.get("jog_step", 0.010)))
        self.jog_feed_var = tk.DoubleVar(value=float(self.settings.get("jog_feed_units_per_min", 10.0)))
        self.controlled_motion_enabled_var = tk.BooleanVar(value=False)
        self.motion_target_x_var = tk.DoubleVar(value=0.0)
        self.motion_target_y_var = tk.DoubleVar(value=0.0)
        self.motion_feed_var = tk.DoubleVar(value=float(self.settings.get("controlled_motion_feed_units_per_min", 20.0)))
        self.motion_status_var = tk.StringVar(value="Controlled motion disabled")
        self.trace_fit_segments_var = tk.IntVar(value=int(self.settings.get("trace_fit_segments", 72)))
        self.jog_status_var = tk.StringVar(value="Jog disabled")
        self.linuxcnc_status_var = tk.StringVar(value="Not polled")
        self.linuxcnc_state_var = tk.StringVar(value="—")
        self.linuxcnc_task_mode_var = tk.StringVar(value="—")
        self.linuxcnc_homed_var = tk.StringVar(value="—")
        self.linuxcnc_x_var = tk.StringVar(value="—")
        self.linuxcnc_y_var = tk.StringVar(value="—")
        self.linuxcnc_z_var = tk.StringVar(value="—")
        self.trace_count_var = tk.StringVar(value="Trace 1: 0 points")
        self.trace_arc_center_var = tk.StringVar(value="Arc center: —")

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
        if self.linuxcnc_auto_refresh_var.get():
            self.schedule_linuxcnc_auto_refresh()

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
        ttk.Button(toolbar, text="Camera Calibrate", command=self.calibrate_camera_lite).pack(side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="Find Contours", command=self.process_image).pack(side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="Set Scale", command=self.start_scale_mode).pack(side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="Export DXF", command=self.export_dxf).pack(side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="Refresh LinuxCNC", command=self.refresh_linuxcnc_status).pack(side=tk.LEFT, padx=6)
        ttk.Checkbutton(
            toolbar,
            text="Trace Preview",
            variable=self.trace_preview_var,
            command=self.on_trace_display_changed,
        ).pack(side=tk.LEFT, padx=(12, 6))
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

        self._build_linuxcnc_trace_panel(side)

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
            "Tip: For image tracing, a high-contrast image with the part separated "
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

    def _build_linuxcnc_trace_panel(self, side: ttk.Frame) -> None:
        """Build the read-only LinuxCNC position and manual trace controls."""

        trace_frame = ttk.LabelFrame(side, text="LinuxCNC / Manual Trace", padding=6)
        trace_frame.pack(side=tk.TOP, fill=tk.X, pady=(8, 0))

        status_grid = ttk.Frame(trace_frame)
        status_grid.pack(fill=tk.X)

        ttk.Label(status_grid, text="Status").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(status_grid, textvariable=self.linuxcnc_status_var).grid(row=0, column=1, sticky=tk.W)
        ttk.Label(status_grid, text="State").grid(row=1, column=0, sticky=tk.W)
        ttk.Label(status_grid, textvariable=self.linuxcnc_state_var).grid(row=1, column=1, sticky=tk.W)
        ttk.Label(status_grid, text="Task mode").grid(row=2, column=0, sticky=tk.W)
        ttk.Label(status_grid, textvariable=self.linuxcnc_task_mode_var).grid(row=2, column=1, sticky=tk.W)
        ttk.Label(status_grid, text="Homed").grid(row=3, column=0, sticky=tk.W)
        ttk.Label(status_grid, textvariable=self.linuxcnc_homed_var).grid(row=3, column=1, sticky=tk.W)
        status_grid.columnconfigure(1, weight=1)

        coord_row = ttk.Frame(trace_frame)
        coord_row.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(coord_row, text="Capture").pack(side=tk.LEFT)
        coord_combo = ttk.Combobox(
            coord_row,
            textvariable=self.linuxcnc_coord_mode_var,
            values=("Work coordinates", "Machine coordinates"),
            state="readonly",
            width=20,
        )
        coord_combo.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(6, 0))
        coord_combo.bind("<<ComboboxSelected>>", lambda _event: self.update_linuxcnc_display())

        position_grid = ttk.Frame(trace_frame)
        position_grid.pack(fill=tk.X, pady=(6, 0))
        for column, label in enumerate(("X", "Y", "Z")):
            ttk.Label(position_grid, text=label).grid(row=0, column=column, sticky=tk.W)
        ttk.Label(position_grid, textvariable=self.linuxcnc_x_var, width=10).grid(row=1, column=0, sticky=tk.W)
        ttk.Label(position_grid, textvariable=self.linuxcnc_y_var, width=10).grid(row=1, column=1, sticky=tk.W)
        ttk.Label(position_grid, textvariable=self.linuxcnc_z_var, width=10).grid(row=1, column=2, sticky=tk.W)
        for column in range(3):
            position_grid.columnconfigure(column, weight=1)

        refresh_row = ttk.Frame(trace_frame)
        refresh_row.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(refresh_row, text="Refresh Position", command=self.refresh_linuxcnc_status).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 3)
        )
        ttk.Checkbutton(
            refresh_row,
            text="Auto",
            variable=self.linuxcnc_auto_refresh_var,
            command=self.on_linuxcnc_auto_refresh_changed,
        ).pack(side=tk.LEFT, padx=(3, 0))

        jog_frame = ttk.LabelFrame(trace_frame, text="FabScan Jog - X/Y Step Only", padding=6)
        jog_frame.pack(fill=tk.X, pady=(8, 0))

        ttk.Checkbutton(
            jog_frame,
            text="Enable jog controls",
            variable=self.jog_controls_enabled_var,
            command=self.on_jog_controls_enabled_changed,
        ).pack(anchor=tk.W)

        jog_settings = ttk.Frame(jog_frame)
        jog_settings.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(jog_settings, text="Step").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(jog_settings, textvariable=self.jog_step_var, width=8).grid(row=0, column=1, sticky=tk.W, padx=(4, 8))
        ttk.Label(jog_settings, text="Feed/min").grid(row=0, column=2, sticky=tk.W)
        ttk.Entry(jog_settings, textvariable=self.jog_feed_var, width=8).grid(row=0, column=3, sticky=tk.W, padx=(4, 0))
        jog_settings.columnconfigure(1, weight=1)
        jog_settings.columnconfigure(3, weight=1)

        jog_step_row = ttk.Frame(jog_frame)
        jog_step_row.pack(fill=tk.X, pady=(4, 0))
        for index, value in enumerate((0.001, 0.005, 0.010, 0.050, 0.100, 0.500)):
            ttk.Button(
                jog_step_row,
                text=(f"{value:.3f}" if value < 0.1 else f"{value:.3f}".rstrip("0").rstrip(".")),
                command=lambda step=value: self.set_jog_step(step),
            ).grid(row=0, column=index, sticky=tk.EW, padx=(0 if index == 0 else 2, 0))
            jog_step_row.columnconfigure(index, weight=1)

        jog_pad = ttk.Frame(jog_frame)
        jog_pad.pack(pady=(6, 0))
        ttk.Button(jog_pad, text="Y+", width=7, command=lambda: self.incremental_jog("Y", +1)).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(jog_pad, text="X-", width=7, command=lambda: self.incremental_jog("X", -1)).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(jog_pad, text="X+", width=7, command=lambda: self.incremental_jog("X", +1)).grid(row=1, column=2, padx=2, pady=2)
        ttk.Button(jog_pad, text="Y-", width=7, command=lambda: self.incremental_jog("Y", -1)).grid(row=2, column=1, padx=2, pady=2)

        ttk.Label(jog_frame, textvariable=self.jog_status_var, wraplength=300).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(
            jog_frame,
            text="Guarded incremental moves only. LinuxCNC must already be in MANUAL mode. No Z, no torch, no continuous jog.",
            wraplength=300,
        ).pack(anchor=tk.W, pady=(2, 0))

        motion_frame = ttk.LabelFrame(trace_frame, text="Controlled Motion - X/Y Point Move", padding=6)
        motion_frame.pack(fill=tk.X, pady=(8, 0))

        ttk.Checkbutton(
            motion_frame,
            text="Enable controlled moves",
            variable=self.controlled_motion_enabled_var,
            command=self.on_controlled_motion_enabled_changed,
        ).pack(anchor=tk.W)

        motion_grid = ttk.Frame(motion_frame)
        motion_grid.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(motion_grid, text="Target X").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(motion_grid, textvariable=self.motion_target_x_var, width=10).grid(row=0, column=1, sticky=tk.EW, padx=(4, 8))
        ttk.Label(motion_grid, text="Target Y").grid(row=0, column=2, sticky=tk.W)
        ttk.Entry(motion_grid, textvariable=self.motion_target_y_var, width=10).grid(row=0, column=3, sticky=tk.EW, padx=(4, 0))
        ttk.Label(motion_grid, text="Feed/min").grid(row=1, column=0, sticky=tk.W, pady=(4, 0))
        ttk.Entry(motion_grid, textvariable=self.motion_feed_var, width=10).grid(row=1, column=1, sticky=tk.EW, padx=(4, 8), pady=(4, 0))
        motion_grid.columnconfigure(1, weight=1)
        motion_grid.columnconfigure(3, weight=1)

        motion_target_row = ttk.Frame(motion_frame)
        motion_target_row.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(motion_target_row, text="Use Current", command=self.set_motion_target_from_current).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 3)
        )
        ttk.Button(motion_target_row, text="Use Selected Pt", command=self.set_motion_target_from_selected_trace_point).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=3
        )

        motion_command_row = ttk.Frame(motion_frame)
        motion_command_row.pack(fill=tk.X, pady=(4, 0))
        ttk.Button(motion_command_row, text="Move to Target", command=self.controlled_move_to_target).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 3)
        )
        ttk.Button(motion_command_row, text="STOP Move", command=self.abort_controlled_motion).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(3, 0)
        )

        ttk.Label(motion_frame, textvariable=self.motion_status_var, wraplength=300).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(
            motion_frame,
            text="One guarded G1 move to X/Y only. No Z and no torch. This is not a replacement for E-stop.",
            wraplength=300,
        ).pack(anchor=tk.W, pady=(2, 0))

        capture_row = ttk.Frame(trace_frame)
        capture_row.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(capture_row, text="Capture Point", command=self.capture_trace_point).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 3)
        )
        ttk.Button(capture_row, text="Start New", command=self.start_new_trace).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=3
        )
        ttk.Button(capture_row, text="Undo", command=self.undo_trace_point).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=3
        )
        ttk.Button(capture_row, text="Clear", command=self.clear_trace_points).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(3, 0)
        )

        columns = ("trace", "n", "x", "y", "z")
        self.trace_tree = ttk.Treeview(
            trace_frame,
            columns=columns,
            show="headings",
            height=7,
            selectmode="browse",
        )
        self.trace_tree.heading("trace", text="Trace")
        self.trace_tree.heading("n", text="#")
        self.trace_tree.heading("x", text="X")
        self.trace_tree.heading("y", text="Y")
        self.trace_tree.heading("z", text="Z")
        self.trace_tree.column("trace", width=48, anchor=tk.E, stretch=False)
        self.trace_tree.column("n", width=34, anchor=tk.E, stretch=False)
        self.trace_tree.column("x", width=70, anchor=tk.E, stretch=True)
        self.trace_tree.column("y", width=70, anchor=tk.E, stretch=True)
        self.trace_tree.column("z", width=70, anchor=tk.E, stretch=True)
        self.trace_tree.pack(fill=tk.X, pady=(6, 0))
        self.trace_tree.bind("<<TreeviewSelect>>", self.on_trace_tree_select)

        nav_frame = ttk.LabelFrame(trace_frame, text="Point / Trace Navigation", padding=6)
        nav_frame.pack(fill=tk.X, pady=(8, 0))

        nav_row1 = ttk.Frame(nav_frame)
        nav_row1.pack(fill=tk.X)
        ttk.Button(nav_row1, text="First Pt", command=self.select_first_trace_point).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2)
        )
        ttk.Button(nav_row1, text="Prev Pt", command=lambda: self.select_relative_trace_point(-1)).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=2
        )
        ttk.Button(nav_row1, text="Next Pt", command=lambda: self.select_relative_trace_point(+1)).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=2
        )
        ttk.Button(nav_row1, text="Last Pt", command=self.select_last_trace_point).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(2, 0)
        )

        nav_row2 = ttk.Frame(nav_frame)
        nav_row2.pack(fill=tk.X, pady=(5, 0))
        ttk.Button(nav_row2, text="Move Pt", command=self.move_to_selected_trace_point).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2)
        )
        ttk.Button(nav_row2, text="Replace", command=self.replace_selected_trace_point_with_current).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=2
        )
        ttk.Button(nav_row2, text="Insert After", command=self.insert_trace_point_after_selected).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=2
        )
        ttk.Button(nav_row2, text="Delete Pt", command=self.delete_selected_trace_point).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(2, 0)
        )

        ttk.Label(
            nav_frame,
            text="Select a point to make its trace active. Move Pt uses the controlled-motion target and safety checks.",
            wraplength=300,
        ).pack(anchor=tk.W, pady=(5, 0))

        trace_footer = ttk.Frame(trace_frame)
        trace_footer.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(trace_footer, textvariable=self.trace_count_var).pack(side=tk.LEFT)
        ttk.Checkbutton(
            trace_footer,
            text="Closed",
            variable=self.trace_closed_var,
            command=self.on_trace_display_changed,
        ).pack(side=tk.RIGHT)

        trace_preview_options = ttk.Frame(trace_frame)
        trace_preview_options.pack(fill=tk.X, pady=(6, 0))
        ttk.Checkbutton(
            trace_preview_options,
            text="Preview",
            variable=self.trace_preview_var,
            command=self.on_trace_display_changed,
        ).pack(side=tk.LEFT)
        ttk.Checkbutton(
            trace_preview_options,
            text="Live pos",
            variable=self.trace_show_live_position_var,
            command=self.on_trace_display_changed,
        ).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Checkbutton(
            trace_preview_options,
            text="Point #s",
            variable=self.trace_show_point_numbers_var,
            command=self.on_trace_display_changed,
        ).pack(side=tk.LEFT, padx=(8, 0))

        fit_frame = ttk.LabelFrame(trace_frame, text="Assisted Trace Tools", padding=6)
        fit_frame.pack(fill=tk.X, pady=(8, 0))

        fit_settings = ttk.Frame(fit_frame)
        fit_settings.pack(fill=tk.X)
        ttk.Label(fit_settings, text="Curve pts").pack(side=tk.LEFT)
        ttk.Entry(fit_settings, textvariable=self.trace_fit_segments_var, width=7).pack(side=tk.LEFT, padx=(6, 0))

        fit_row1 = ttk.Frame(fit_frame)
        fit_row1.pack(fill=tk.X, pady=(5, 0))
        ttk.Button(fit_row1, text="Line Endpoints", command=self.fit_active_trace_line_endpoints).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 3)
        )
        ttk.Button(fit_row1, text="Rect 2 Pts", command=self.fit_active_trace_rectangle).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(3, 0)
        )

        fit_row2 = ttk.Frame(fit_frame)
        fit_row2.pack(fill=tk.X, pady=(5, 0))
        ttk.Button(fit_row2, text="Circle Fit", command=self.fit_active_trace_circle).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 3)
        )
        ttk.Button(fit_row2, text="3 Pt Arc", command=self.fit_active_trace_arc_3_point).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(3, 0)
        )

        fit_row3 = ttk.Frame(fit_frame)
        fit_row3.pack(fill=tk.X, pady=(5, 0))
        ttk.Button(fit_row3, text="Set Center", command=self.set_trace_arc_center_from_current).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 3)
        )
        ttk.Button(fit_row3, text="Center Arc", command=self.fit_active_trace_center_arc).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(3, 3)
        )
        ttk.Button(fit_row3, text="Clear Center", command=self.clear_trace_arc_center).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(3, 0)
        )

        ttk.Label(fit_frame, textvariable=self.trace_arc_center_var, wraplength=300).pack(anchor=tk.W, pady=(5, 0))
        ttk.Label(
            fit_frame,
            text="Circle/arc tools export native DXF geometry. 3 Pt Arc uses exactly 3 points; Center Arc uses stored center + 2 active points.",
            wraplength=300,
        ).pack(anchor=tk.W, pady=(5, 0))

        ttk.Button(trace_frame, text="Export Manual Trace DXF", command=self.export_trace_dxf).pack(
            fill=tk.X, pady=(6, 0)
        )

        ttk.Label(
            trace_frame,
            text="v0.4.3: Native DXF arcs/circles stay intact when you continue tracing from their endpoint.",
            wraplength=300,
        ).pack(anchor=tk.W, pady=(6, 0))


    def ensure_trace_entity_list(self) -> None:
        """Keep trace entity metadata aligned with trace_groups."""

        while len(self.trace_entities) < len(self.trace_groups):
            self.trace_entities.append(None)
        if len(self.trace_entities) > len(self.trace_groups):
            self.trace_entities = self.trace_entities[: len(self.trace_groups)]

    def get_trace_entity(self, trace_index: int) -> Optional[dict[str, object]]:
        self.ensure_trace_entity_list()
        if 0 <= trace_index < len(self.trace_entities):
            return self.trace_entities[trace_index]
        return None

    def set_trace_entity(self, trace_index: int, entity: Optional[dict[str, object]]) -> None:
        self.ensure_trace_entity_list()
        if 0 <= trace_index < len(self.trace_entities):
            self.trace_entities[trace_index] = entity

    def clear_trace_entity(self, trace_index: int, reason: str | None = None) -> None:
        entity = self.get_trace_entity(trace_index)
        if entity is None:
            return
        self.set_trace_entity(trace_index, None)
        if reason:
            self.append_status(f"\nTrace {trace_index + 1} native geometry cleared: {reason}.")

    def get_active_trace_entity_label(self) -> str:
        entity = self.get_trace_entity(self.active_trace_index)
        if not isinstance(entity, dict):
            return "polyline"
        return str(entity.get("label", entity.get("type", "polyline")))

    def angle_degrees_from_center(self, center_x: float, center_y: float, x: float, y: float) -> float:
        return math.degrees(math.atan2(float(y) - center_y, float(x) - center_x)) % 360.0

    def minor_arc_angles_from_center(
        self,
        center: tuple[float, float, float] | tuple[float, float],
        start: tuple[float, float, float] | tuple[float, float],
        end: tuple[float, float, float] | tuple[float, float],
    ) -> tuple[float, float, float, float]:
        """Return DXF CCW start/end angles for the short arc plus start/end radii."""

        center_x = float(center[0])
        center_y = float(center[1])
        start_angle = self.angle_degrees_from_center(center_x, center_y, float(start[0]), float(start[1]))
        end_angle = self.angle_degrees_from_center(center_x, center_y, float(end[0]), float(end[1]))
        ccw_delta = (end_angle - start_angle) % 360.0
        if ccw_delta <= 180.0:
            dxf_start = start_angle
            dxf_end = end_angle
        else:
            # DXF ARC is CCW. Swapping start/end draws the same minor arc geometry.
            dxf_start = end_angle
            dxf_end = start_angle
        start_radius = math.hypot(float(start[0]) - center_x, float(start[1]) - center_y)
        end_radius = math.hypot(float(end[0]) - center_x, float(end[1]) - center_y)
        return dxf_start, dxf_end, start_radius, end_radius

    def trace_entity_display_points(
        self,
        trace_index: int,
        group: list[tuple[float, float, float]],
    ) -> list[tuple[float, float, float]]:
        """Return display points for the trace preview, sampling native entities when needed."""

        entity = self.get_trace_entity(trace_index)
        if not isinstance(entity, dict):
            return list(group)

        entity_kind = str(entity.get("type", "polyline"))
        if entity_kind == "circle":
            center = entity.get("center")
            radius = float(entity.get("radius", 0.0))
            if center is None or radius <= 0.0:
                return list(group)
            cx = float(center[0])
            cy = float(center[1])
            z = float(group[0][2]) if group else 0.0
            segments = self.get_trace_fit_segment_count(minimum=24, maximum=360)
            return [
                (cx + radius * math.cos(2.0 * math.pi * index / segments),
                 cy + radius * math.sin(2.0 * math.pi * index / segments),
                 z)
                for index in range(segments + 1)
            ]

        if entity_kind == "arc":
            center = entity.get("center")
            radius = float(entity.get("radius", 0.0))
            if center is None or radius <= 0.0:
                return list(group)
            cx = float(center[0])
            cy = float(center[1])
            z = float(group[0][2]) if group else 0.0
            start_angle = math.radians(float(entity.get("start_angle", 0.0)))
            end_angle = math.radians(float(entity.get("end_angle", 0.0)))
            sweep = (end_angle - start_angle) % (2.0 * math.pi)
            if math.isclose(sweep, 0.0, abs_tol=1e-9):
                sweep = 2.0 * math.pi
            max_segments = self.get_trace_fit_segment_count(minimum=8, maximum=360)
            actual_segments = max(4, int(math.ceil(max_segments * (sweep / (2.0 * math.pi)))))
            return [
                (cx + radius * math.cos(start_angle + sweep * index / actual_segments),
                 cy + radius * math.sin(start_angle + sweep * index / actual_segments),
                 z)
                for index in range(actual_segments + 1)
            ]

        return list(group)


    def get_active_trace_points(self) -> list[tuple[float, float, float]]:
        """Return the point list for the currently active manual trace."""

        if not self.trace_groups:
            self.trace_groups = [[]]
            self.active_trace_index = 0

        if self.active_trace_index < 0 or self.active_trace_index >= len(self.trace_groups):
            self.active_trace_index = len(self.trace_groups) - 1

        return self.trace_groups[self.active_trace_index]

    def get_nonempty_trace_groups(self) -> list[list[tuple[float, float, float]]]:
        """Return only manual trace groups that contain points."""

        return [group for group in self.trace_groups if group]

    def get_total_trace_point_count(self) -> int:
        """Return the total number of captured manual trace points across all traces."""

        return sum(len(group) for group in self.trace_groups)

    def get_active_trace_label(self) -> str:
        return f"Trace {self.active_trace_index + 1}"

    def start_new_trace(self) -> None:
        """Start a new manual trace group so separate contours do not connect."""

        active_points = self.get_active_trace_points()
        if not active_points:
            self.append_status(f"\n{self.get_active_trace_label()} is empty; capture points or select Clear first.")
            return

        self.trace_groups.append([])
        self.trace_entities.append(None)
        self.active_trace_index = len(self.trace_groups) - 1
        self.refresh_trace_point_list()
        self.append_status(
            f"\nStarted {self.get_active_trace_label()}. "
            "New captured points will be separate from the previous trace."
        )

    def on_linuxcnc_auto_refresh_changed(self) -> None:
        self.queue_save_settings()
        if self.linuxcnc_auto_refresh_var.get():
            self.schedule_linuxcnc_auto_refresh()
        elif self.linuxcnc_auto_refresh_job is not None:
            self.after_cancel(self.linuxcnc_auto_refresh_job)
            self.linuxcnc_auto_refresh_job = None

    def on_trace_display_changed(self) -> None:
        """Refresh the manual trace preview after trace display options change."""

        self.queue_save_settings()
        self.redraw_preview()

    def schedule_linuxcnc_auto_refresh(self) -> None:
        if self.linuxcnc_auto_refresh_job is not None:
            self.after_cancel(self.linuxcnc_auto_refresh_job)
        self.linuxcnc_auto_refresh_job = self.after(750, self._linuxcnc_auto_refresh_tick)

    def _linuxcnc_auto_refresh_tick(self) -> None:
        self.linuxcnc_auto_refresh_job = None
        if not self.linuxcnc_auto_refresh_var.get():
            return
        self.refresh_linuxcnc_status(show_errors=False)
        self.schedule_linuxcnc_auto_refresh()

    def refresh_linuxcnc_status(self, show_errors: bool = True) -> None:
        """Poll LinuxCNC status without sending any motion commands."""

        status = self.linuxcnc_reader.read_status()
        self.latest_linuxcnc_status = status
        self.update_linuxcnc_display()
        self.redraw_preview()

        if show_errors and not status.connected:
            messagebox.showinfo(
                "LinuxCNC not connected",
                status.error or "LinuxCNC status is not available.",
                parent=self,
            )

    def update_linuxcnc_display(self) -> None:
        status = self.latest_linuxcnc_status
        if status is None:
            self.linuxcnc_status_var.set("Not polled")
            self.linuxcnc_state_var.set("—")
            self.linuxcnc_task_mode_var.set("—")
            self.linuxcnc_homed_var.set("—")
            self.linuxcnc_x_var.set("—")
            self.linuxcnc_y_var.set("—")
            self.linuxcnc_z_var.set("—")
            return

        if not status.available:
            self.linuxcnc_status_var.set("Module missing")
            self.linuxcnc_state_var.set("—")
            self.linuxcnc_task_mode_var.set("—")
            self.linuxcnc_homed_var.set("—")
            self.linuxcnc_x_var.set("—")
            self.linuxcnc_y_var.set("—")
            self.linuxcnc_z_var.set("—")
            return

        if not status.connected:
            self.linuxcnc_status_var.set("Not connected")
            self.linuxcnc_state_var.set("—")
            self.linuxcnc_task_mode_var.set("—")
            self.linuxcnc_homed_var.set("—")
            self.linuxcnc_x_var.set("—")
            self.linuxcnc_y_var.set("—")
            self.linuxcnc_z_var.set("—")
            return

        self.linuxcnc_status_var.set("Connected")
        self.linuxcnc_state_var.set(f"{status.task_state} / {status.interp_state}")
        self.linuxcnc_task_mode_var.set(status.task_mode)
        self.linuxcnc_homed_var.set(status.homed_text)

        position = self.get_active_linuxcnc_position(status)
        self.linuxcnc_x_var.set(f"{position[0]:.4f}")
        self.linuxcnc_y_var.set(f"{position[1]:.4f}")
        self.linuxcnc_z_var.set(f"{position[2]:.4f}")

    def get_active_linuxcnc_position(self, status: LinuxCNCPositionStatus) -> tuple[float, float, float]:
        if self.linuxcnc_coord_mode_var.get() == "Machine coordinates":
            return status.machine_position
        return status.work_position

    def on_jog_controls_enabled_changed(self) -> None:
        """Require an explicit per-session acknowledgement before jog buttons move the machine."""

        if bool(self.jog_controls_enabled_var.get()):
            accepted = messagebox.askyesno(
                "Enable FabScan jog controls?",
                "FabScan jog buttons will move the machine in X/Y by the selected step.\n\n"
                "Use only with the torch disabled, the table clear, and your hand near E-stop.\n\n"
                "FabScan will still refuse to jog unless LinuxCNC is ON, IDLE, homed, and already in MANUAL mode.\n\n"
                "Enable jog controls for this session?",
                parent=self,
            )
            if not accepted:
                self.jog_controls_enabled_var.set(False)
                self.jog_status_var.set("Jog disabled")
                return
            self.jog_status_var.set("Jog enabled: X/Y incremental only")
            self.append_status("\nFabScan jog controls enabled for this session.")
        else:
            self.jog_status_var.set("Jog disabled")
            self.append_status("\nFabScan jog controls disabled.")
        self.queue_save_settings()

    def set_jog_step(self, step: float) -> None:
        self.jog_step_var.set(float(step))
        self.jog_status_var.set(f"Jog step set to {step:.4f}")

    def incremental_jog(self, axis: str, direction: int) -> None:
        """Send one guarded X/Y incremental jog through LinuxCNC."""

        if not bool(self.jog_controls_enabled_var.get()):
            self.jog_status_var.set("Jog disabled - check Enable jog controls first")
            return

        if self.jog_busy:
            self.jog_status_var.set("Jog busy - wait for the current step to finish")
            return

        step = self.safe_float_from_var(self.jog_step_var, 0.010)
        feed = self.safe_float_from_var(self.jog_feed_var, 10.0)

        self.jog_busy = True
        self.jog_status_var.set(f"Jogging {axis}{'+' if direction >= 0 else '-'} {abs(step):.4f}...")

        result = self.linuxcnc_reader.incremental_jog(axis, direction, step, feed)
        self.latest_linuxcnc_status = result.status or self.linuxcnc_reader.read_status()
        self.update_linuxcnc_display()
        self.redraw_preview()

        self.jog_status_var.set(result.message)
        self.append_status(f"\n{result.message}")

        if result.success:
            self.schedule_jog_release(step, feed)
        else:
            self.jog_busy = False
            if self.jog_release_job is not None:
                self.after_cancel(self.jog_release_job)
                self.jog_release_job = None
            messagebox.showinfo("FabScan jog refused", result.message, parent=self)

    def schedule_jog_release(self, step: float, feed: float) -> None:
        """Briefly lock out jog buttons so repeated clicks do not overlap step moves."""

        if self.jog_release_job is not None:
            self.after_cancel(self.jog_release_job)

        step = abs(float(step))
        feed = abs(float(feed))
        if feed <= 0.0:
            delay_seconds = 0.35
        else:
            units_per_second = feed / 60.0
            delay_seconds = (step / units_per_second) + 0.25

        delay_seconds = max(0.35, min(delay_seconds, 5.0))
        self.jog_release_job = self.after(int(delay_seconds * 1000), self.release_jog_busy)

    def release_jog_busy(self) -> None:
        """Mark the jog panel ready for another incremental move."""

        self.jog_release_job = None
        self.jog_busy = False
        if bool(self.jog_controls_enabled_var.get()):
            self.jog_status_var.set("Jog ready")
        self.refresh_linuxcnc_status(show_errors=False)

    def on_controlled_motion_enabled_changed(self) -> None:
        """Require explicit acknowledgement before FabScan can send G-code motion."""

        if bool(self.controlled_motion_enabled_var.get()):
            accepted = messagebox.askyesno(
                "Enable FabScan controlled motion?",
                "FabScan will be allowed to send one X/Y G1 move to LinuxCNC using the target fields.\n\n"
                "This can move the table farther than a jog step. Use only with the torch disabled, "
                "the table clear, and your hand near E-stop.\n\n"
                "FabScan will still refuse motion unless LinuxCNC is ON, IDLE, homed, and in MANUAL or MDI mode.\n\n"
                "Enable controlled X/Y moves for this session?",
                parent=self,
            )
            if not accepted:
                self.controlled_motion_enabled_var.set(False)
                self.motion_status_var.set("Controlled motion disabled")
                return
            self.motion_status_var.set("Controlled motion enabled: X/Y G1 only")
            self.append_status("\nFabScan controlled X/Y motion enabled for this session.")
        else:
            self.motion_status_var.set("Controlled motion disabled")
            self.append_status("\nFabScan controlled motion disabled.")
        self.queue_save_settings()

    def set_motion_target_from_current(self) -> None:
        """Copy the current LinuxCNC position into the controlled-motion target."""

        self.refresh_linuxcnc_status(show_errors=False)
        status = self.latest_linuxcnc_status
        if status is None or not status.connected:
            messagebox.showinfo(
                "No LinuxCNC position",
                (status.error if status is not None else None)
                or "Could not read LinuxCNC position. Start LinuxCNC and try Refresh Position.",
                parent=self,
            )
            return

        x, y, _z = self.get_active_linuxcnc_position(status)
        self.motion_target_x_var.set(float(x))
        self.motion_target_y_var.set(float(y))
        self.motion_status_var.set(f"Target set from current position: X{x:.4f} Y{y:.4f}")
        self.queue_save_settings()

    def get_trace_point_iids(self) -> list[str]:
        """Return all point item IDs currently shown in the trace point tree."""

        if not hasattr(self, "trace_tree"):
            return []
        return [str(item) for item in self.trace_tree.get_children("")]

    def parse_trace_iid(self, iid: str) -> Optional[tuple[int, int]]:
        """Parse a trace tree item id into zero-based trace/point indexes."""

        try:
            trace_text, point_text = str(iid).split(":", 1)
            trace_index = int(trace_text)
            point_index = int(point_text)
        except (TypeError, ValueError):
            return None
        if trace_index < 0 or trace_index >= len(self.trace_groups):
            return None
        if point_index < 0 or point_index >= len(self.trace_groups[trace_index]):
            return None
        return trace_index, point_index

    def select_trace_point_iid(self, iid: str) -> bool:
        """Select a trace point in the list and make its trace active."""

        parsed = self.parse_trace_iid(iid)
        if parsed is None or not hasattr(self, "trace_tree"):
            return False
        trace_index, _point_index = parsed
        self.active_trace_index = trace_index
        self.trace_tree.selection_set(iid)
        self.trace_tree.focus(iid)
        self.trace_tree.see(iid)
        self.redraw_preview()
        return True

    def on_trace_tree_select(self, _event: object | None = None) -> None:
        """Make the selected point's trace active and redraw the preview."""

        selected = self.get_selected_trace_point()
        if selected is None:
            return
        trace_index, _point_index, _point = selected
        self.active_trace_index = trace_index
        self.redraw_preview()

    def select_relative_trace_point(self, delta: int) -> None:
        """Select the previous or next point in the trace list."""

        iids = self.get_trace_point_iids()
        if not iids:
            self.append_status("\nNo trace points to select.")
            return

        selection = self.trace_tree.selection() if hasattr(self, "trace_tree") else ()
        if selection and str(selection[0]) in iids:
            current_index = iids.index(str(selection[0]))
        elif delta >= 0:
            current_index = -1
        else:
            current_index = 0

        new_index = (current_index + delta) % len(iids)
        self.select_trace_point_iid(iids[new_index])

    def select_first_trace_point(self) -> None:
        """Select the first point in the selected/active trace, falling back to the first trace point overall."""

        selected = self.get_selected_trace_point()
        target_trace = selected[0] if selected is not None else self.active_trace_index
        group = self.trace_groups[target_trace] if 0 <= target_trace < len(self.trace_groups) else []
        if group:
            self.select_trace_point_iid(f"{target_trace}:0")
            return

        iids = self.get_trace_point_iids()
        if iids:
            self.select_trace_point_iid(iids[0])
        else:
            self.append_status("\nNo trace points to select.")

    def select_last_trace_point(self) -> None:
        """Select the last point in the selected/active trace, falling back to the last point overall."""

        selected = self.get_selected_trace_point()
        target_trace = selected[0] if selected is not None else self.active_trace_index
        group = self.trace_groups[target_trace] if 0 <= target_trace < len(self.trace_groups) else []
        if group:
            self.select_trace_point_iid(f"{target_trace}:{len(group) - 1}")
            return

        iids = self.get_trace_point_iids()
        if iids:
            self.select_trace_point_iid(iids[-1])
        else:
            self.append_status("\nNo trace points to select.")

    def read_current_trace_position_or_warn(self, title: str) -> Optional[tuple[float, float, float]]:
        """Refresh and return the active LinuxCNC position for trace editing."""

        self.refresh_linuxcnc_status(show_errors=False)
        status = self.latest_linuxcnc_status
        if status is None or not status.connected:
            messagebox.showinfo(
                title,
                (status.error if status is not None else None)
                or "Could not read LinuxCNC position. Start LinuxCNC and try Refresh Position.",
                parent=self,
            )
            return None
        return self.get_active_linuxcnc_position(status)

    def move_to_selected_trace_point(self) -> None:
        """Move to the currently selected trace point using the guarded controlled-motion path."""

        if self.get_selected_trace_point() is None:
            messagebox.showinfo("No trace point selected", "Select a captured trace point first.", parent=self)
            return
        self.set_motion_target_from_selected_trace_point()
        self.controlled_move_to_target()

    def replace_selected_trace_point_with_current(self) -> None:
        """Replace the selected trace point with the current LinuxCNC position."""

        selected = self.get_selected_trace_point()
        if selected is None:
            messagebox.showinfo("No trace point selected", "Select a captured trace point first.", parent=self)
            return

        position = self.read_current_trace_position_or_warn("Replace trace point")
        if position is None:
            return

        trace_index, point_index, old_point = selected
        self.clear_trace_entity(trace_index, "point was replaced")
        self.trace_groups[trace_index][point_index] = position
        iid = f"{trace_index}:{point_index}"
        self.active_trace_index = trace_index
        self.refresh_trace_point_list(select_iid=iid)
        self.append_status(
            f"\nReplaced trace {trace_index + 1}.{point_index + 1}: "
            f"old X {old_point[0]:.4f}, Y {old_point[1]:.4f} -> "
            f"new X {position[0]:.4f}, Y {position[1]:.4f}."
        )

    def insert_trace_point_after_selected(self) -> None:
        """Insert the current LinuxCNC position after the selected trace point."""

        selected = self.get_selected_trace_point()
        if selected is None:
            messagebox.showinfo("No trace point selected", "Select a captured trace point first.", parent=self)
            return

        position = self.read_current_trace_position_or_warn("Insert trace point")
        if position is None:
            return

        trace_index, point_index, _old_point = selected
        self.clear_trace_entity(trace_index, "point was inserted")
        insert_index = point_index + 1
        self.trace_groups[trace_index].insert(insert_index, position)
        iid = f"{trace_index}:{insert_index}"
        self.active_trace_index = trace_index
        self.refresh_trace_point_list(select_iid=iid)
        self.append_status(
            f"\nInserted trace {trace_index + 1}.{insert_index + 1}: "
            f"X {position[0]:.4f}, Y {position[1]:.4f}, Z {position[2]:.4f}."
        )

    def delete_selected_trace_point(self) -> None:
        """Delete the selected trace point and keep the rest of the trace groups valid."""

        selected = self.get_selected_trace_point()
        if selected is None:
            messagebox.showinfo("No trace point selected", "Select a captured trace point first.", parent=self)
            return

        trace_index, point_index, point = selected
        self.clear_trace_entity(trace_index, "point was deleted")
        deleted_label = f"trace {trace_index + 1}.{point_index + 1}"
        removed = self.trace_groups[trace_index].pop(point_index)

        if not self.trace_groups[trace_index] and len(self.trace_groups) > 1:
            del self.trace_groups[trace_index]
            if trace_index < len(self.trace_entities):
                del self.trace_entities[trace_index]
            self.active_trace_index = max(0, min(trace_index, len(self.trace_groups) - 1))
            select_iid = None
        else:
            self.active_trace_index = trace_index
            if self.trace_groups[trace_index]:
                new_point_index = min(point_index, len(self.trace_groups[trace_index]) - 1)
                select_iid = f"{trace_index}:{new_point_index}"
            else:
                select_iid = None

        if not self.trace_groups:
            self.trace_groups = [[]]
            self.trace_entities = [None]
            self.active_trace_index = 0
            select_iid = None

        self.refresh_trace_point_list(select_iid=select_iid)
        self.append_status(
            f"\nDeleted {deleted_label}: X {removed[0]:.4f}, Y {removed[1]:.4f}, Z {removed[2]:.4f}."
        )

    def get_selected_trace_point(self) -> Optional[tuple[int, int, tuple[float, float, float]]]:
        """Return selected trace point as (trace_index, point_index, point)."""

        if not hasattr(self, "trace_tree"):
            return None
        selection = self.trace_tree.selection()
        if not selection:
            return None
        iid = str(selection[0])
        try:
            trace_text, point_text = iid.split(":", 1)
            trace_index = int(trace_text)
            point_index = int(point_text)
            point = self.trace_groups[trace_index][point_index]
        except Exception:  # noqa: BLE001
            return None
        return trace_index, point_index, point

    def set_motion_target_from_selected_trace_point(self) -> None:
        """Copy the selected manual-trace point into the controlled-motion target."""

        selected = self.get_selected_trace_point()
        if selected is None:
            messagebox.showinfo("No trace point selected", "Select a captured trace point first.", parent=self)
            return

        trace_index, point_index, point = selected
        self.motion_target_x_var.set(float(point[0]))
        self.motion_target_y_var.set(float(point[1]))
        self.motion_status_var.set(
            f"Target set from trace {trace_index + 1}.{point_index + 1}: X{point[0]:.4f} Y{point[1]:.4f}"
        )
        self.queue_save_settings()

    def controlled_move_to_target(self) -> None:
        """Send one guarded X/Y controlled move to the target fields."""

        if not bool(self.controlled_motion_enabled_var.get()):
            self.motion_status_var.set("Controlled motion disabled - enable it first")
            return
        if self.motion_busy:
            self.motion_status_var.set("Move busy - wait for current move or press STOP Move")
            return

        target_x = self.safe_float_from_var(self.motion_target_x_var, 0.0)
        target_y = self.safe_float_from_var(self.motion_target_y_var, 0.0)
        feed = self.safe_float_from_var(self.motion_feed_var, 20.0)
        coord_label = self.linuxcnc_coord_mode_var.get()

        accepted = messagebox.askyesno(
            "Move machine to target?",
            f"Move X/Y to:\n\n"
            f"X = {target_x:.4f}\n"
            f"Y = {target_y:.4f}\n"
            f"Feed = {feed:.1f} units/min\n"
            f"Coordinates = {coord_label}\n\n"
            "This will command LinuxCNC motion. Keep your hand near E-stop. Continue?",
            parent=self,
        )
        if not accepted:
            self.motion_status_var.set("Controlled move cancelled")
            return

        self.motion_busy = True
        self.motion_status_var.set(f"Starting move to X{target_x:.4f} Y{target_y:.4f}...")
        self.append_status(f"\nStarting controlled move to X{target_x:.4f} Y{target_y:.4f}.")

        result = self.linuxcnc_reader.controlled_xy_move(target_x, target_y, feed, coord_label)
        self.latest_linuxcnc_status = result.status or self.linuxcnc_reader.read_status()
        self.update_linuxcnc_display()
        self.redraw_preview()

        if not result.success:
            self.motion_busy = False
            self.motion_status_var.set(result.message)
            self.append_status(f"\n{result.message}")
            messagebox.showinfo("FabScan controlled move refused", result.message, parent=self)
            return

        self.motion_status_var.set(result.message)
        self.append_status(f"\n{result.message}\nMDI: {result.mdi_command}")
        self.queue_save_settings()
        self.schedule_controlled_motion_monitor()

    def schedule_controlled_motion_monitor(self) -> None:
        if self.motion_monitor_job is not None:
            self.after_cancel(self.motion_monitor_job)
        self.motion_monitor_job = self.after(500, self._controlled_motion_monitor_tick)

    def _controlled_motion_monitor_tick(self) -> None:
        self.motion_monitor_job = None
        status = self.linuxcnc_reader.read_status()
        self.latest_linuxcnc_status = status
        self.update_linuxcnc_display()
        self.redraw_preview()

        if not self.motion_busy:
            return
        if not status.connected:
            self.motion_busy = False
            self.motion_status_var.set(status.error or "LinuxCNC disconnected during controlled move")
            return
        if status.interp_state == "IDLE":
            self.motion_busy = False
            x, y, _z = self.get_active_linuxcnc_position(status)
            self.motion_status_var.set(f"Move complete / idle at X{x:.4f} Y{y:.4f}")
            self.append_status(f"\nControlled move complete / LinuxCNC idle at X{x:.4f} Y{y:.4f}.")
            return

        self.motion_status_var.set(f"Move running... interpreter {status.interp_state}")
        self.schedule_controlled_motion_monitor()

    def abort_controlled_motion(self) -> None:
        """Abort FabScan-requested motion through LinuxCNC."""

        result = self.linuxcnc_reader.abort_motion()
        self.latest_linuxcnc_status = result.status or self.linuxcnc_reader.read_status()
        self.update_linuxcnc_display()
        self.redraw_preview()

        if self.motion_monitor_job is not None:
            self.after_cancel(self.motion_monitor_job)
            self.motion_monitor_job = None
        self.motion_busy = False
        self.motion_status_var.set(result.message)
        self.append_status(f"\n{result.message}")
        if not result.success:
            messagebox.showinfo("FabScan stop/abort", result.message, parent=self)

    def capture_trace_point(self) -> None:
        """Capture the current LinuxCNC X/Y/Z position into the active manual trace.

        If the active trace has already been fitted to a native DXF entity,
        preserve that entity and automatically start a continuation trace from
        its last defining point. This lets a user create, for example, a native
        arc and then capture the next tangent line point without converting the
        arc back into a two-point polyline.
        """

        self.refresh_linuxcnc_status(show_errors=False)
        status = self.latest_linuxcnc_status
        if status is None or not status.connected:
            messagebox.showinfo(
                "No LinuxCNC position",
                (status.error if status is not None else None)
                or "Could not read LinuxCNC position. Start LinuxCNC and try Refresh Position.",
                parent=self,
            )
            return

        position = self.get_active_linuxcnc_position(status)
        active_points = self.get_active_trace_points()
        active_entity = self.get_trace_entity(self.active_trace_index)

        if active_points and isinstance(active_entity, dict):
            # Do not destroy a native LINE/ARC/CIRCLE just because the user is
            # continuing the trace. Start a new trace seeded from the previous
            # endpoint, then append the newly captured point. The separate DXF
            # entities will still touch if the endpoint coordinates match.
            seed_point = active_points[-1]
            previous_label = self.get_active_trace_label()
            self.trace_groups.append([seed_point, position])
            self.trace_entities.append(None)
            self.active_trace_index = len(self.trace_groups) - 1
            self.refresh_trace_point_list(select_iid=f"{self.active_trace_index}:1")
            self.append_status(
                f"\nPreserved {previous_label} native {active_entity.get('label', active_entity.get('type', 'entity'))}. "
                f"Started {self.get_active_trace_label()} from the fitted endpoint and captured point 2: "
                f"X {position[0]:.4f}, Y {position[1]:.4f}, Z {position[2]:.4f} "
                f"({self.linuxcnc_coord_mode_var.get()})"
            )
            return

        active_points.append(position)
        self.refresh_trace_point_list(select_iid=f"{self.active_trace_index}:{len(active_points) - 1}")
        self.append_status(
            f"\nCaptured {self.get_active_trace_label()} point {len(active_points)}: "
            f"X {position[0]:.4f}, Y {position[1]:.4f}, Z {position[2]:.4f} "
            f"({self.linuxcnc_coord_mode_var.get()})"
        )

    def undo_trace_point(self) -> None:
        active_points = self.get_active_trace_points()
        if not active_points:
            return
        self.clear_trace_entity(self.active_trace_index, "point was removed")
        removed = active_points.pop()
        self.refresh_trace_point_list()
        self.append_status(
            f"\nRemoved {self.get_active_trace_label()} point: "
            f"X {removed[0]:.4f}, Y {removed[1]:.4f}, Z {removed[2]:.4f}"
        )

    def clear_trace_points(self) -> None:
        if self.get_total_trace_point_count() == 0 and len(self.trace_groups) <= 1 and self.trace_arc_center is None:
            return
        if not messagebox.askyesno("Clear trace points?", "Clear all manually captured trace groups?", parent=self):
            return
        self.trace_groups = [[]]
        self.trace_entities = [None]
        self.active_trace_index = 0
        self.trace_arc_center = None
        self.trace_arc_center_var.set("Arc center: —")
        self.refresh_trace_point_list()
        self.append_status("\nManual trace groups cleared.")

    def get_trace_fit_segment_count(self, minimum: int = 4, maximum: int = 360) -> int:
        """Return a bounded segment/point count for generated manual trace curves."""

        try:
            value = int(self.trace_fit_segments_var.get())
        except (tk.TclError, ValueError):
            value = 72
        value = max(minimum, min(value, maximum))
        if value != self.trace_fit_segments_var.get():
            self.trace_fit_segments_var.set(value)
        return value

    def warn_active_trace_points(self, required: int, tool_name: str) -> Optional[list[tuple[float, float, float]]]:
        """Return active trace points or show a friendly message when too few exist."""

        points = self.get_active_trace_points()
        if len(points) < required:
            messagebox.showinfo(
                tool_name,
                f"{tool_name} needs at least {required} point{'s' if required != 1 else ''} in the active trace.",
                parent=self,
            )
            return None
        return points

    def replace_active_trace_points(self, new_points: list[tuple[float, float, float]], message: str) -> None:
        """Replace the active trace with generated/fitted points and refresh the UI."""

        if not new_points:
            return
        self.trace_groups[self.active_trace_index] = new_points
        self.set_trace_entity(self.active_trace_index, None)
        self.refresh_trace_point_list()
        self.append_status("\n" + message)

    def fit_active_trace_line_endpoints(self) -> None:
        """Reduce the active trace to a native DXF line between its first and last point."""

        points = self.warn_active_trace_points(2, "Line Endpoints")
        if points is None:
            return
        new_points = [points[0], points[-1]]
        self.trace_groups[self.active_trace_index] = new_points
        self.set_trace_entity(
            self.active_trace_index,
            {"type": "line", "label": "LINE", "start": new_points[0], "end": new_points[1]},
        )
        self.refresh_trace_point_list()
        self.append_status(
            f"\nLine Endpoints replaced {self.get_active_trace_label()} with a native DXF LINE."
        )

    def fit_active_trace_rectangle(self) -> None:
        """Replace the active trace with an axis-aligned rectangle from two opposite corners."""

        points = self.warn_active_trace_points(2, "Rect 2 Pts")
        if points is None:
            return

        p1 = points[0]
        p2 = points[1]
        x1, y1, z1 = p1
        x2, y2, z2 = p2
        if math.isclose(x1, x2) or math.isclose(y1, y2):
            messagebox.showinfo(
                "Rect 2 Pts",
                "The first two points need different X and Y values to make a rectangle.",
                parent=self,
            )
            return

        z_mid = (float(z1) + float(z2)) / 2.0
        new_points = [
            (float(x1), float(y1), float(z1)),
            (float(x2), float(y1), z_mid),
            (float(x2), float(y2), float(z2)),
            (float(x1), float(y2), z_mid),
        ]
        self.trace_closed_var.set(True)
        self.trace_groups[self.active_trace_index] = new_points
        self.set_trace_entity(self.active_trace_index, {"type": "rect", "label": "RECT"})
        self.refresh_trace_point_list()
        self.append_status(
            f"\nRect 2 Pts replaced {self.get_active_trace_label()} with a 4-corner rectangle. Closed trace enabled."
        )

    def fit_active_trace_circle(self) -> None:
        """Fit a native DXF circle to the active trace points."""

        points = self.warn_active_trace_points(3, "Circle Fit")
        if points is None:
            return

        circle = self.calculate_circle_fit(points, tool_name="Circle Fit")
        if circle is None:
            return
        center_x, center_y, radius = circle

        self.set_trace_entity(
            self.active_trace_index,
            {
                "type": "circle",
                "label": "CIRCLE",
                "center": (center_x, center_y, 0.0),
                "radius": radius,
            },
        )
        self.refresh_trace_point_list()
        self.append_status(
            f"\nCircle Fit set {self.get_active_trace_label()} to export as a native DXF CIRCLE. "
            f"Center X {center_x:.4f}, Y {center_y:.4f}, radius {radius:.4f}."
        )

    def fit_active_trace_arc_3_point(self) -> None:
        """Fit a native DXF arc through exactly three active trace points."""

        points = self.warn_active_trace_points(3, "3 Pt Arc")
        if points is None:
            return
        if len(points) != 3:
            messagebox.showinfo(
                "3 Pt Arc",
                "3 Pt Arc uses exactly three points in the active trace: start, point on arc, end.\n\n"
                "Use Start New before capturing the three arc points if this arc is part of a larger shape.",
                parent=self,
            )
            return

        arc_entity = self.calculate_arc_entity_from_3_points(points[0], points[1], points[2], tool_name="3 Pt Arc")
        if arc_entity is None:
            return

        self.set_trace_entity(self.active_trace_index, arc_entity)
        self.refresh_trace_point_list()
        center = arc_entity["center"]
        self.append_status(
            f"\n3 Pt Arc set {self.get_active_trace_label()} to export as a native DXF ARC. "
            f"Center X {center[0]:.4f}, Y {center[1]:.4f}, radius {float(arc_entity['radius']):.4f}."
        )

    # Backward-compatible name for old button/function references.
    def fit_active_trace_arc_last3(self) -> None:
        self.fit_active_trace_arc_3_point()

    def set_trace_arc_center_from_current(self) -> None:
        """Store the current LinuxCNC position as the center for Center Arc."""

        position = self.read_current_trace_position_or_warn("Set Arc Center")
        if position is None:
            return
        self.trace_arc_center = position
        self.trace_arc_center_var.set(f"Arc center: X {position[0]:.4f}  Y {position[1]:.4f}")
        self.append_status(f"\nArc center set to X {position[0]:.4f}, Y {position[1]:.4f}, Z {position[2]:.4f}.")

    def clear_trace_arc_center(self) -> None:
        self.trace_arc_center = None
        self.trace_arc_center_var.set("Arc center: —")
        self.append_status("\nArc center cleared.")

    def fit_active_trace_center_arc(self) -> None:
        """Create a native short DXF arc from stored center plus two active trace points."""

        if self.trace_arc_center is None:
            messagebox.showinfo(
                "Center Arc",
                "Set an arc center first. Jog to the center point and click Set Center.",
                parent=self,
            )
            return

        points = self.warn_active_trace_points(2, "Center Arc")
        if points is None:
            return
        if len(points) != 2:
            messagebox.showinfo(
                "Center Arc",
                "Center Arc uses exactly two points in the active trace: arc start and arc end.\n\n"
                "The stored center is used as the arc center.",
                parent=self,
            )
            return

        center = self.trace_arc_center
        start = points[0]
        end = points[1]
        dxf_start, dxf_end, start_radius, end_radius = self.minor_arc_angles_from_center(center, start, end)
        if start_radius <= 0.0 or end_radius <= 0.0:
            messagebox.showinfo("Center Arc", "Start/end points must not be on top of the center point.", parent=self)
            return

        avg_radius = (start_radius + end_radius) / 2.0
        radius_tolerance = max(0.005, avg_radius * 0.01)
        radius_error = abs(start_radius - end_radius)
        if radius_error > radius_tolerance:
            messagebox.showinfo(
                "Center Arc",
                "Start and end are not the same distance from the stored center.\n\n"
                f"Start radius: {start_radius:.4f}\n"
                f"End radius:   {end_radius:.4f}\n"
                f"Difference:   {radius_error:.4f}\n"
                f"Allowed:      {radius_tolerance:.4f}\n\n"
                "Jog/capture the start and end again, or use 3 Pt Arc instead.",
                parent=self,
            )
            return

        self.set_trace_entity(
            self.active_trace_index,
            {
                "type": "arc",
                "label": "CENTER ARC",
                "center": (float(center[0]), float(center[1]), float(center[2])),
                "radius": avg_radius,
                "start_angle": dxf_start,
                "end_angle": dxf_end,
            },
        )
        self.refresh_trace_point_list()
        self.append_status(
            f"\nCenter Arc set {self.get_active_trace_label()} to export as a native short DXF ARC. "
            f"Center X {center[0]:.4f}, Y {center[1]:.4f}, radius {avg_radius:.4f}."
        )

    def calculate_circle_fit(
        self,
        points: list[tuple[float, float, float]],
        tool_name: str,
    ) -> Optional[tuple[float, float, float]]:
        """Return least-squares circle center/radius for CNC points."""

        xy = np.array([(float(point[0]), float(point[1])) for point in points], dtype=float)
        x = xy[:, 0]
        y = xy[:, 1]
        a = np.column_stack((x, y, np.ones_like(x)))
        b = -(x * x + y * y)

        try:
            d, e, f = np.linalg.lstsq(a, b, rcond=None)[0]
        except np.linalg.LinAlgError as exc:
            messagebox.showinfo(tool_name, f"Could not fit a circle: {exc}", parent=self)
            return None

        center_x = float(-d / 2.0)
        center_y = float(-e / 2.0)
        radius_sq = float((d * d + e * e) / 4.0 - f)
        if radius_sq <= 0.0:
            messagebox.showinfo(tool_name, "Could not fit a valid circle to those points.", parent=self)
            return None

        return center_x, center_y, math.sqrt(radius_sq)

    def calculate_arc_entity_from_3_points(
        self,
        start: tuple[float, float, float],
        mid: tuple[float, float, float],
        end: tuple[float, float, float],
        tool_name: str,
    ) -> Optional[dict[str, object]]:
        """Return native DXF arc metadata through start/mid/end."""

        x1, y1, z1 = start
        x2, y2, z2 = mid
        x3, y3, z3 = end

        determinant = 2.0 * (
            x1 * (y2 - y3)
            + x2 * (y3 - y1)
            + x3 * (y1 - y2)
        )
        if math.isclose(determinant, 0.0, abs_tol=1e-9):
            messagebox.showinfo(tool_name, "The three points are too close to a straight line to fit an arc.", parent=self)
            return None

        ux = (
            (x1 * x1 + y1 * y1) * (y2 - y3)
            + (x2 * x2 + y2 * y2) * (y3 - y1)
            + (x3 * x3 + y3 * y3) * (y1 - y2)
        ) / determinant
        uy = (
            (x1 * x1 + y1 * y1) * (x3 - x2)
            + (x2 * x2 + y2 * y2) * (x1 - x3)
            + (x3 * x3 + y3 * y3) * (x2 - x1)
        ) / determinant

        radius = math.hypot(x1 - ux, y1 - uy)
        if radius <= 0.0:
            messagebox.showinfo(tool_name, "Could not fit a valid arc radius.", parent=self)
            return None

        def angle_of(x_value: float, y_value: float) -> float:
            return math.atan2(y_value - uy, x_value - ux)

        def ccw_delta(a0: float, a1: float) -> float:
            return (a1 - a0) % (2.0 * math.pi)

        start_angle = angle_of(x1, y1)
        mid_angle = angle_of(x2, y2)
        end_angle = angle_of(x3, y3)

        start_to_end_ccw = ccw_delta(start_angle, end_angle)
        start_to_mid_ccw = ccw_delta(start_angle, mid_angle)

        # Choose the arc from start to end that passes through the middle point.
        if start_to_mid_ccw <= start_to_end_ccw:
            arc_sweep = start_to_end_ccw
        else:
            arc_sweep = start_to_end_ccw - (2.0 * math.pi)

        start_degrees = math.degrees(start_angle) % 360.0
        end_degrees = math.degrees(end_angle) % 360.0
        if arc_sweep >= 0.0:
            dxf_start = start_degrees
            dxf_end = end_degrees
        else:
            # DXF ARC is CCW. Swapping start/end draws the same arc geometry.
            dxf_start = end_degrees
            dxf_end = start_degrees

        z_avg = (float(z1) + float(z2) + float(z3)) / 3.0
        return {
            "type": "arc",
            "label": "3 PT ARC",
            "center": (float(ux), float(uy), z_avg),
            "radius": float(radius),
            "start_angle": float(dxf_start),
            "end_angle": float(dxf_end),
        }

    def refresh_trace_point_list(self, select_iid: Optional[str] = None) -> None:
        previous_selection: Optional[str] = None
        if hasattr(self, "trace_tree"):
            selection = self.trace_tree.selection()
            if selection:
                previous_selection = str(selection[0])

        for item in self.trace_tree.get_children():
            self.trace_tree.delete(item)

        if select_iid is None:
            select_iid = previous_selection

        last_iid: Optional[str] = None
        for group_index, group in enumerate(self.trace_groups, start=1):
            if not group:
                continue
            for point_index, point in enumerate(group, start=1):
                iid = f"{group_index - 1}:{point_index - 1}"
                last_iid = iid
                self.trace_tree.insert(
                    "",
                    tk.END,
                    iid=iid,
                    values=(
                        group_index,
                        point_index,
                        f"{point[0]:.4f}",
                        f"{point[1]:.4f}",
                        f"{point[2]:.4f}",
                    ),
                )

        total_points = self.get_total_trace_point_count()
        nonempty_groups = self.get_nonempty_trace_groups()
        active_points = self.get_active_trace_points()
        point_word = "point" if total_points == 1 else "points"
        trace_word = "trace" if len(nonempty_groups) == 1 else "traces"
        active_entity = self.get_active_trace_entity_label()
        self.trace_count_var.set(
            f"{len(nonempty_groups)} {trace_word}, {total_points} {point_word} | "
            f"Active {self.active_trace_index + 1}: {len(active_points)} | {active_entity}"
        )
        tree_iids = set(self.trace_tree.get_children(""))
        if select_iid is not None and select_iid in tree_iids:
            self.select_trace_point_iid(select_iid)
        elif last_iid is not None:
            self.trace_tree.see(last_iid)

        self.redraw_preview()

    def export_trace_dxf(self) -> None:
        groups = self.get_nonempty_trace_groups()
        if not groups:
            messagebox.showinfo("Not enough points", "Capture at least two CNC points before exporting.", parent=self)
            return

        too_short = [index + 1 for index, group in enumerate(self.trace_groups) if group and len(group) < 2]
        if too_short:
            messagebox.showinfo(
                "Not enough points",
                "Each exported trace needs at least two points. "
                f"Check trace(s): {', '.join(str(value) for value in too_short)}",
                parent=self,
            )
            return

        close_trace = bool(self.trace_closed_var.get())
        if close_trace:
            too_short_closed = []
            for index, group in enumerate(self.trace_groups):
                entity = self.get_trace_entity(index)
                native_type = str(entity.get("type", "")) if isinstance(entity, dict) else ""
                if group and native_type not in ("line", "circle", "arc") and len(group) < 3:
                    too_short_closed.append(index + 1)
            if too_short_closed:
                messagebox.showinfo(
                    "Not enough points",
                    "A closed polyline trace needs at least three CNC points. "
                    f"Check trace(s): {', '.join(str(value) for value in too_short_closed)}",
                    parent=self,
                )
                return

        initial_export_dir = Path(str(self.settings.get("last_export_dir", Path.cwd() / "exports")))
        if not initial_export_dir.exists():
            initial_export_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"fabscan_manual_trace_{timestamp}.dxf"

        path = filedialog.asksaveasfilename(
            title="Export Manual Trace DXF",
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
            output = export_trace_groups_to_dxf(
                self.trace_groups,
                output_path=path,
                close=close_trace,
                layer_name="TRACE",
                trace_entities=self.trace_entities,
            )
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Trace DXF export failed", str(exc), parent=self)
            return

        all_points = [point for group in groups for point in group]
        xs = [point[0] for point in all_points]
        ys = [point[1] for point in all_points]
        width = max(xs) - min(xs)
        height = max(ys) - min(ys)
        native_count = sum(
            1
            for index, group in enumerate(self.trace_groups)
            if group and isinstance(self.get_trace_entity(index), dict)
        )
        export_message = (
            "Manual trace DXF exported successfully.\n"
            f"File: {output}\n"
            f"Layer: TRACE\n"
            f"Traces: {len(groups)}\n"
            f"Points: {len(all_points)} defining/captured\n"
            f"Native entities: {native_count}\n"
            f"Closed polylines: {'Yes' if close_trace else 'No'}\n"
            f"Coordinate source: {self.linuxcnc_coord_mode_var.get()}\n"
            f"Combined trace bbox: {width:.4f} x {height:.4f}\n\n"
            "Next: import the DXF into SheetCam/CAD and verify the measured geometry."
        )
        self.append_status("\n" + export_message)
        messagebox.showinfo("Trace DXF exported", export_message, parent=self)

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
            self.linuxcnc_coord_mode_var,
            self.linuxcnc_auto_refresh_var,
            self.trace_closed_var,
            self.trace_preview_var,
            self.trace_show_live_position_var,
            self.trace_show_point_numbers_var,
            self.jog_controls_enabled_var,
            self.jog_step_var,
            self.jog_feed_var,
            self.motion_feed_var,
            self.trace_fit_segments_var,
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
            "linuxcnc_coord_mode_label": str(self.linuxcnc_coord_mode_var.get()),
            "linuxcnc_auto_refresh": bool(self.linuxcnc_auto_refresh_var.get()),
            "trace_closed": bool(self.trace_closed_var.get()),
            "trace_preview": bool(self.trace_preview_var.get()),
            "trace_show_live_position": bool(self.trace_show_live_position_var.get()),
            "trace_show_point_numbers": bool(self.trace_show_point_numbers_var.get()),
            "jog_controls_enabled": False,
            "jog_step": self.safe_float_from_var(self.jog_step_var, 0.010),
            "jog_feed_units_per_min": self.safe_float_from_var(self.jog_feed_var, 10.0),
            "controlled_motion_feed_units_per_min": self.safe_float_from_var(self.motion_feed_var, 20.0),
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
            "camera_calibration_threshold": int(self.settings.get("camera_calibration_threshold", 90)),
            "camera_calibration_move_distance": float(self.settings.get("camera_calibration_move_distance", 0.100)),
            "camera_calibration_feed_units_per_min": float(self.settings.get("camera_calibration_feed_units_per_min", 5.0)),
            "camera_calibration_jog_step": float(self.settings.get("camera_calibration_jog_step", 0.010)),
            "camera_calibration_center_max_move": float(self.settings.get("camera_calibration_center_max_move", 0.100)),
            "camera_calibration_line_mode": str(self.settings.get("camera_calibration_line_mode", "Line center")),
            "camera_calibration_line_search_px": int(self.settings.get("camera_calibration_line_search_px", 220)),
            "camera_calibration_show_line_preview": bool(self.settings.get("camera_calibration_show_line_preview", True)),
            "camera_calibration_show_mask": bool(self.settings.get("camera_calibration_show_mask", False)),
            "camera_follow_step": float(self.settings.get("camera_follow_step", 0.050)),
            "camera_follow_max_correct": float(self.settings.get("camera_follow_max_correct", 0.050)),
            "camera_follow_min_confidence": float(self.settings.get("camera_follow_min_confidence", 45.0)),
            "camera_follow_direction": str(self.settings.get("camera_follow_direction", "Forward")),
            "camera_follow_capture_point": bool(self.settings.get("camera_follow_capture_point", False)),
            "camera_follow_enabled": bool(self.settings.get("camera_follow_enabled", False)),
            "camera_follow_repeat_count": int(self.settings.get("camera_follow_repeat_count", 5)),
            "camera_calibration": self.settings.get("camera_calibration", None),
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
        if self.linuxcnc_auto_refresh_job is not None:
            self.after_cancel(self.linuxcnc_auto_refresh_job)
            self.linuxcnc_auto_refresh_job = None
        if self.motion_monitor_job is not None:
            self.after_cancel(self.motion_monitor_job)
            self.motion_monitor_job = None
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
            "Manual CNC trace workflow:\n"
            "1. Start LinuxCNC normally, home the machine, and keep the torch disabled.\n"
            "2. Use Refresh LinuxCNC to read position. Optionally enable FabScan X/Y step jogs.\n"
            "3. Jog to each point and click Capture Point.\n"
            "4. Watch captured points in the Trace Preview canvas.\n"
            "5. Use Start New when tracing a separate contour, such as a hole inside an outside profile.\n"
            "6. Export Manual Trace DXF when done. Camera Calibration Lite can be opened from the toolbar to learn camera/machine direction before later edge-following work.\n\n"
            "Tips:\n"
            "- Keep cleanup values low unless the camera image is ugly.\n"
            "- Use Show Threshold to see what FabScan is actually tracing.\n"
            "- Disabled contours stay visible in gray but do not export.\n"
            "- X+ is right and Y+ is up in the transformed camera preview and manual trace preview.",
            parent=self,
        )

    def show_about(self) -> None:
        """Show version/about information."""

        messagebox.showinfo(
            "About FabScan",
            f"FabScan v{APP_VERSION}\n\n"
            "Photo/camera/CNC-trace-to-DXF helper for flat plasma parts.\n\n"
            "Design goal: create usable DXF geometry quickly, then let SheetCam/CAD do final cleanup when needed.\n\n"
            "v0.5.3 adds Center Dot to the Camera Calibration Lite screen. After calibration, FabScan can use the saved camera/machine transform to move the machine so the detected dot lands under the crosshair. This proves the calibration can steer before we try edge following.\n\n"
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
        self.trace_preview_var.set(bool(DEFAULT_SETTINGS["trace_preview"]))
        self.trace_show_live_position_var.set(bool(DEFAULT_SETTINGS["trace_show_live_position"]))
        self.trace_show_point_numbers_var.set(bool(DEFAULT_SETTINGS["trace_show_point_numbers"]))
        self.jog_controls_enabled_var.set(False)
        self.jog_busy = False
        if self.jog_release_job is not None:
            self.after_cancel(self.jog_release_job)
            self.jog_release_job = None
        self.jog_step_var.set(float(DEFAULT_SETTINGS["jog_step"]))
        self.jog_feed_var.set(float(DEFAULT_SETTINGS["jog_feed_units_per_min"]))
        self.jog_status_var.set("Jog disabled")
        self.controlled_motion_enabled_var.set(False)
        self.motion_busy = False
        if self.motion_monitor_job is not None:
            self.after_cancel(self.motion_monitor_job)
            self.motion_monitor_job = None
        self.motion_feed_var.set(float(DEFAULT_SETTINGS["controlled_motion_feed_units_per_min"]))
        self.motion_status_var.set("Controlled motion disabled")

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

    def calibrate_camera_lite(self) -> None:
        """Open the camera/machine calibration helper.

        This does not create DXF geometry. It teaches FabScan how camera pixel
        movement relates to LinuxCNC X/Y movement so later v0.5.x edge-following
        can steer with the camera while LinuxCNC remains the position ruler.
        """

        camera_index = self.safe_int_from_settings("camera_index", 0)
        camera_width = self.safe_int_from_settings("camera_width", 1280)
        camera_height = self.safe_int_from_settings("camera_height", 720)
        camera_rotate_degrees = self.safe_int_from_settings("camera_rotate_degrees", 0)
        camera_flip_x = self.safe_bool_from_settings("camera_flip_x", False)
        camera_flip_y = self.safe_bool_from_settings("camera_flip_y", False)
        camera_fine_rotation_degrees = self.safe_float_from_settings("camera_fine_rotation_degrees", 0.0)
        cal_threshold = self.safe_int_from_settings("camera_calibration_threshold", 90)
        cal_move = self.safe_float_from_settings("camera_calibration_move_distance", 0.100)
        cal_feed = self.safe_float_from_settings("camera_calibration_feed_units_per_min", 5.0)
        cal_jog_step = self.safe_float_from_settings("camera_calibration_jog_step", 0.010)
        cal_center_max_move = self.safe_float_from_settings("camera_calibration_center_max_move", 0.100)
        cal_line_mode = str(self.settings.get("camera_calibration_line_mode", "Line center"))
        cal_line_search_px = self.safe_int_from_settings("camera_calibration_line_search_px", 220)
        cal_show_line_preview = self.safe_bool_from_settings("camera_calibration_show_line_preview", True)
        cal_show_mask = self.safe_bool_from_settings("camera_calibration_show_mask", False)
        follow_step = self.safe_float_from_settings("camera_follow_step", 0.050)
        follow_max_correct = self.safe_float_from_settings("camera_follow_max_correct", 0.050)
        follow_min_confidence = self.safe_float_from_settings("camera_follow_min_confidence", 45.0)
        follow_direction = str(self.settings.get("camera_follow_direction", "Forward"))
        follow_capture_point = self.safe_bool_from_settings("camera_follow_capture_point", False)
        follow_enabled = self.safe_bool_from_settings("camera_follow_enabled", False)
        follow_repeat_count = self.safe_int_from_settings("camera_follow_repeat_count", 5)
        existing_calibration = self.settings.get("camera_calibration", None)

        dialog = CameraCalibrationDialog(
            self,
            linuxcnc_reader=self.linuxcnc_reader,
            coordinate_mode_label=str(self.linuxcnc_coord_mode_var.get()),
            camera_index=camera_index,
            camera_width=camera_width,
            camera_height=camera_height,
            rotate_degrees=camera_rotate_degrees,
            flip_x=camera_flip_x,
            flip_y=camera_flip_y,
            fine_rotation_degrees=camera_fine_rotation_degrees,
            threshold=cal_threshold,
            move_distance=cal_move,
            feed_per_minute=cal_feed,
            jog_step=cal_jog_step,
            center_max_move=cal_center_max_move,
            line_mode=cal_line_mode,
            line_search_px=cal_line_search_px,
            show_line_preview=cal_show_line_preview,
            show_mask=cal_show_mask,
            follow_step=follow_step,
            follow_max_correct=follow_max_correct,
            follow_min_confidence=follow_min_confidence,
            follow_direction=follow_direction,
            follow_capture_point=follow_capture_point,
            follow_enabled=follow_enabled,
            follow_repeat_count=follow_repeat_count,
            existing_calibration=existing_calibration,
            trace_capture_callback=self.capture_trace_point,
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
        self.settings["camera_calibration_threshold"] = dialog.result.threshold
        self.settings["camera_calibration_show_mask"] = dialog.result.show_mask
        self.settings["camera_calibration_move_distance"] = dialog.result.move_distance
        self.settings["camera_calibration_feed_units_per_min"] = dialog.result.feed_units_per_min
        self.settings["camera_calibration_jog_step"] = dialog.result.jog_step
        self.settings["camera_calibration_center_max_move"] = dialog.result.center_max_move
        self.settings["camera_calibration_line_mode"] = dialog.result.line_mode
        self.settings["camera_calibration_line_search_px"] = dialog.result.line_search_px
        self.settings["camera_calibration_show_line_preview"] = dialog.result.show_line_preview
        self.settings["camera_follow_step"] = dialog.result.follow_step
        self.settings["camera_follow_max_correct"] = dialog.result.follow_max_correct
        self.settings["camera_follow_min_confidence"] = dialog.result.follow_min_confidence
        self.settings["camera_follow_direction"] = dialog.result.follow_direction
        self.settings["camera_follow_capture_point"] = dialog.result.follow_capture_point
        self.settings["camera_follow_enabled"] = dialog.result.follow_enabled
        self.settings["camera_follow_repeat_count"] = dialog.result.follow_repeat_count

        if dialog.result.calibration:
            self.settings["camera_calibration"] = dialog.result.calibration
            self.append_status(
                "\nCamera calibration complete: "
                f"X {dialog.result.calibration['pixels_per_unit_x']:.1f} px/unit, "
                f"Y {dialog.result.calibration['pixels_per_unit_y']:.1f} px/unit."
            )
        else:
            self.append_status("\nCamera calibration window closed without a completed calibration.")

        self.queue_save_settings()

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
        if self.should_show_trace_preview():
            return

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

        if self.should_show_trace_preview():
            self.draw_trace_preview()
            return

        if self.image_bgr is None:
            self.canvas.create_text(
                self.canvas.winfo_width() / 2,
                self.canvas.winfo_height() / 2,
                text="Load an image or enable Trace Preview",
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

    def should_show_trace_preview(self) -> bool:
        """Return True when the main canvas should show manual trace geometry."""

        if not bool(self.trace_preview_var.get()):
            return False

        # With no image loaded, the trace preview is the useful empty-canvas view.
        if self.image_bgr is None:
            return True

        # If manual trace groups contain points, show the trace preview while the preview
        # toggle is enabled. Uncheck Trace Preview to return to image tracing.
        return self.get_total_trace_point_count() > 0

    def get_live_trace_position(self) -> Optional[tuple[float, float, float]]:
        """Return current LinuxCNC position when it should be shown in the trace preview."""

        if not bool(self.trace_show_live_position_var.get()):
            return None
        status = self.latest_linuxcnc_status
        if status is None or not status.connected:
            return None
        return self.get_active_linuxcnc_position(status)

    def get_trace_plot_points(self) -> list[tuple[float, float, str]]:
        """Return XY points to include when fitting the trace preview."""

        points: list[tuple[float, float, str]] = []
        for group_index, group in enumerate(self.trace_groups):
            for point in self.trace_entity_display_points(group_index, group):
                points.append((float(point[0]), float(point[1]), f"trace_{group_index}"))
        live = self.get_live_trace_position()
        if live is not None:
            points.append((float(live[0]), float(live[1]), "live"))
        return points

    def draw_trace_preview(self) -> None:
        """Draw the manually captured CNC trace on the main canvas."""

        canvas_w = max(1, self.canvas.winfo_width())
        canvas_h = max(1, self.canvas.winfo_height())

        self.canvas.create_rectangle(0, 0, canvas_w, canvas_h, fill="#202020", outline="")
        self.canvas.create_text(
            12,
            12,
            anchor=tk.NW,
            text="Manual Trace Preview",
            fill="white",
            font=("TkDefaultFont", 14, "bold"),
        )

        fit_points = self.get_trace_plot_points()
        if not fit_points:
            self._draw_empty_trace_preview(canvas_w, canvas_h)
            return

        xs = [point[0] for point in fit_points]
        ys = [point[1] for point in fit_points]
        min_x = min(xs)
        max_x = max(xs)
        min_y = min(ys)
        max_y = max(ys)

        width = max_x - min_x
        height = max_y - min_y
        if width < 0.001:
            center = (min_x + max_x) / 2.0
            min_x = center - 0.5
            max_x = center + 0.5
            width = 1.0
        if height < 0.001:
            center = (min_y + max_y) / 2.0
            min_y = center - 0.5
            max_y = center + 0.5
            height = 1.0

        # Add 10 percent world padding, with a small minimum so single-point
        # captures are still legible.
        pad_x = max(width * 0.10, 0.100)
        pad_y = max(height * 0.10, 0.100)
        min_x -= pad_x
        max_x += pad_x
        min_y -= pad_y
        max_y += pad_y
        width = max_x - min_x
        height = max_y - min_y

        margin_left = 58
        margin_right = 24
        margin_top = 56
        margin_bottom = 44
        plot_w = max(1, canvas_w - margin_left - margin_right)
        plot_h = max(1, canvas_h - margin_top - margin_bottom)
        scale = min(plot_w / width, plot_h / height)

        used_w = width * scale
        used_h = height * scale
        plot_left = margin_left + (plot_w - used_w) / 2.0
        plot_top = margin_top + (plot_h - used_h) / 2.0
        plot_right = plot_left + used_w
        plot_bottom = plot_top + used_h

        def world_to_canvas(x: float, y: float) -> tuple[float, float]:
            cx = plot_left + (x - min_x) * scale
            cy = plot_bottom - (y - min_y) * scale
            return cx, cy

        self._draw_trace_grid(plot_left, plot_top, plot_right, plot_bottom)
        self._draw_trace_axes(world_to_canvas, min_x, max_x, min_y, max_y, plot_left, plot_top, plot_right, plot_bottom)

        for group_index, group in enumerate(self.trace_groups, start=1):
            if not group:
                continue

            display_points = self.trace_entity_display_points(group_index - 1, group)
            captured_canvas_points = [world_to_canvas(float(x), float(y)) for x, y, _z in display_points]
            line_fill = "lime" if (group_index - 1) == self.active_trace_index else "#35b6ff"
            point_fill = "yellow" if (group_index - 1) == self.active_trace_index else "#9fd8ff"
            entity = self.get_trace_entity(group_index - 1)
            native_entity = isinstance(entity, dict) and str(entity.get("type", "polyline")) in ("line", "circle", "arc")

            if len(captured_canvas_points) >= 2:
                flat_points = [coord for point in captured_canvas_points for coord in point]
                self.canvas.create_line(*flat_points, fill=line_fill, width=2)
                if bool(self.trace_closed_var.get()) and not native_entity and len(captured_canvas_points) >= 3:
                    self.canvas.create_line(
                        captured_canvas_points[-1][0],
                        captured_canvas_points[-1][1],
                        captured_canvas_points[0][0],
                        captured_canvas_points[0][1],
                        fill=line_fill,
                        width=2,
                    )

            if native_entity:
                label = str(entity.get("label", entity.get("type", ""))) if isinstance(entity, dict) else ""
                if captured_canvas_points:
                    lx, ly = captured_canvas_points[0]
                    self.canvas.create_text(
                        lx + 10,
                        ly + 10,
                        anchor=tk.NW,
                        text=label,
                        fill=line_fill,
                        font=("TkDefaultFont", 9, "bold"),
                    )

            selected_trace_point = self.get_selected_trace_point()
            selected_trace_index = selected_trace_point[0] if selected_trace_point is not None else None
            selected_point_index = selected_trace_point[1] if selected_trace_point is not None else None

            for point_index, (x, y, z) in enumerate(group, start=1):
                cx, cy = world_to_canvas(float(x), float(y))
                is_selected = (
                    selected_trace_index == (group_index - 1)
                    and selected_point_index == (point_index - 1)
                )
                radius = 8 if is_selected else 5
                self.canvas.create_oval(
                    cx - radius,
                    cy - radius,
                    cx + radius,
                    cy + radius,
                    fill="orange" if is_selected else point_fill,
                    outline="white" if is_selected else "black",
                    width=3 if is_selected else 1,
                )
                if bool(self.trace_show_point_numbers_var.get()):
                    self.canvas.create_text(
                        cx + 8,
                        cy - 8,
                        anchor=tk.SW,
                        text=f"{group_index}.{point_index}",
                        fill="white",
                        font=("TkDefaultFont", 9, "bold"),
                    )

        live = self.get_live_trace_position()
        if live is not None:
            lx, ly, lz = live
            cx, cy = world_to_canvas(float(lx), float(ly))
            size = 9
            self.canvas.create_line(cx - size, cy, cx + size, cy, fill="red", width=2)
            self.canvas.create_line(cx, cy - size, cx, cy + size, fill="red", width=2)
            self.canvas.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, outline="red", width=2)
            self.canvas.create_text(
                cx + 12,
                cy + 12,
                anchor=tk.NW,
                text=f"Live X {lx:.4f}  Y {ly:.4f}",
                fill="red",
                font=("TkDefaultFont", 9, "bold"),
            )

        trace_state = "Closed" if bool(self.trace_closed_var.get()) else "Open"
        coord_source = self.linuxcnc_coord_mode_var.get()
        self.canvas.create_text(
            12,
            canvas_h - 12,
            anchor=tk.SW,
            text=(
                f"{len(self.get_nonempty_trace_groups())} traces, {self.get_total_trace_point_count()} points  |  "
                f"Active {self.active_trace_index + 1} ({self.get_active_trace_entity_label()})  |  {trace_state}  |  {coord_source}  |  "
                "Start New creates a separate contour"
            ),
            fill="#dddddd",
            font=("TkDefaultFont", 10),
        )

        self._draw_trace_direction_indicator(canvas_w, canvas_h)

    def _draw_empty_trace_preview(self, canvas_w: int, canvas_h: int) -> None:
        center_x = canvas_w / 2.0
        center_y = canvas_h / 2.0
        self.canvas.create_line(center_x - 80, center_y, center_x + 80, center_y, fill="#555555", width=1)
        self.canvas.create_line(center_x, center_y - 80, center_x, center_y + 80, fill="#555555", width=1)
        self.canvas.create_text(
            center_x,
            center_y + 110,
            text="Jog in LinuxCNC, then click Capture Point.\nCaptured CNC points will plot here.",
            fill="#dddddd",
            font=("TkDefaultFont", 14),
            justify=tk.CENTER,
        )
        self._draw_trace_direction_indicator(canvas_w, canvas_h)

    def _draw_trace_grid(self, left: float, top: float, right: float, bottom: float) -> None:
        self.canvas.create_rectangle(left, top, right, bottom, outline="#666666", width=1)
        for i in range(1, 4):
            x = left + (right - left) * i / 4.0
            y = top + (bottom - top) * i / 4.0
            self.canvas.create_line(x, top, x, bottom, fill="#303030")
            self.canvas.create_line(left, y, right, y, fill="#303030")

    def _draw_trace_axes(
        self,
        world_to_canvas,
        min_x: float,
        max_x: float,
        min_y: float,
        max_y: float,
        left: float,
        top: float,
        right: float,
        bottom: float,
    ) -> None:
        # Draw X=0 and Y=0 axes when they are inside the fitted world range.
        if min_y <= 0.0 <= max_y:
            x1, y0 = world_to_canvas(min_x, 0.0)
            x2, _y0 = world_to_canvas(max_x, 0.0)
            self.canvas.create_line(x1, y0, x2, y0, fill="#777777", dash=(4, 4))
        if min_x <= 0.0 <= max_x:
            x0, y1 = world_to_canvas(0.0, min_y)
            _x0, y2 = world_to_canvas(0.0, max_y)
            self.canvas.create_line(x0, y1, x0, y2, fill="#777777", dash=(4, 4))

        self.canvas.create_text(left, bottom + 6, anchor=tk.NW, text=f"X {min_x:.3f}", fill="#bbbbbb")
        self.canvas.create_text(right, bottom + 6, anchor=tk.NE, text=f"X {max_x:.3f}", fill="#bbbbbb")
        self.canvas.create_text(left - 6, top, anchor=tk.NE, text=f"Y {max_y:.3f}", fill="#bbbbbb")
        self.canvas.create_text(left - 6, bottom, anchor=tk.SE, text=f"Y {min_y:.3f}", fill="#bbbbbb")

    def _draw_trace_direction_indicator(self, canvas_w: int, canvas_h: int) -> None:
        base_x = canvas_w - 86
        base_y = 70
        self.canvas.create_line(base_x, base_y, base_x + 46, base_y, fill="white", width=2, arrow=tk.LAST)
        self.canvas.create_line(base_x, base_y, base_x, base_y - 46, fill="white", width=2, arrow=tk.LAST)
        self.canvas.create_text(base_x + 52, base_y, text="X+", fill="white", anchor=tk.W, font=("TkDefaultFont", 10, "bold"))
        self.canvas.create_text(base_x, base_y - 52, text="Y+", fill="white", anchor=tk.S, font=("TkDefaultFont", 10, "bold"))

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
