from __future__ import annotations

import argparse
import json
import math
import queue
import sys
import threading
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

from fabscan.replay_rerun import ExistingPlannerRerunner, RerunReport, RerunStepResult
from fabscan.path_belief import BeliefState, ReplayBeliefAnalyzer
from fabscan.svg_simulator import launch_svg_simulator

from PIL import Image, ImageDraw, ImageFont, ImageTk
import tkinter as tk
from tkinter import filedialog, messagebox, ttk


REPLAY_FORMAT_VERSION = 1
INTEGRATION_VERSION = "0.6.0-dev-m5.2"


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object in {path}")
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
                raise ValueError(f"Expected an object in {path.name} line {line_number}")
            rows.append(value)
    return rows


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


def _safe_child(root: Path, relative: str) -> Path:
    candidate = (root / relative).resolve()
    try:
        candidate.relative_to(root.resolve())
    except ValueError as exc:
        raise ValueError(f"Replay file escapes run directory: {relative}") from exc
    return candidate


@dataclass(frozen=True)
class ReplayRun:
    run_dir: Path
    manifest: dict[str, Any]
    settings: dict[str, Any]
    calibration: dict[str, Any]
    steps: list[dict[str, Any]]
    events_by_step: dict[str, list[dict[str, Any]]]

    @classmethod
    def load(cls, run_dir: Path | str) -> "ReplayRun":
        root = Path(run_dir).expanduser().resolve()
        if not root.is_dir():
            raise ValueError(f"Replay folder does not exist: {root}")
        required = ("manifest.json", "settings.json", "calibration.json", "steps.jsonl")
        missing = [name for name in required if not (root / name).is_file()]
        if missing:
            raise ValueError(f"Not a FabScan replay folder; missing: {', '.join(missing)}")

        manifest = _load_json(root / "manifest.json")
        version = _int(manifest.get("format_version"), -1)
        if version != REPLAY_FORMAT_VERSION:
            raise ValueError(
                f"Unsupported replay format {version}; this viewer supports {REPLAY_FORMAT_VERSION}"
            )
        steps = _load_jsonl(root / "steps.jsonl")
        if not steps:
            raise ValueError("Replay contains no steps")

        events_by_step: dict[str, list[dict[str, Any]]] = {}
        events_path = root / "events.jsonl"
        if events_path.is_file():
            for event in _load_jsonl(events_path):
                key = _text(event.get("step_id"))
                if key:
                    events_by_step.setdefault(key, []).append(event)

        # Validate every referenced decision frame now, so failures are explicit.
        for index, step in enumerate(steps, start=1):
            relative = _text(step.get("decision_frame"))
            if not relative:
                frames = step.get("frames")
                if isinstance(frames, list) and frames:
                    relative = _text(frames[-1])
            if not relative:
                raise ValueError(f"Step {index} has no recorded decision frame")
            frame_path = _safe_child(root, relative)
            if not frame_path.is_file():
                raise ValueError(f"Step {index} references missing frame: {relative}")

        return cls(
            run_dir=root,
            manifest=manifest,
            settings=_load_json(root / "settings.json"),
            calibration=_load_json(root / "calibration.json"),
            steps=steps,
            events_by_step=events_by_step,
        )

    @property
    def name(self) -> str:
        return _text(self.manifest.get("run_id"), self.run_dir.name)

    def step_frame_path(self, index: int) -> Path:
        step = self.steps[index]
        relative = _text(step.get("decision_frame"))
        if not relative:
            frames = step.get("frames")
            if isinstance(frames, list) and frames:
                relative = _text(frames[-1])
        return _safe_child(self.run_dir, relative)

    def events_for_index(self, index: int) -> list[dict[str, Any]]:
        step_id = _text(self.steps[index].get("step_id"))
        return self.events_by_step.get(step_id, [])

    def trajectory_points(self) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []
        for step in self.steps:
            before = step.get("position_before")
            if isinstance(before, dict) and before.get("x") not in (None, "") and before.get("y") not in (None, ""):
                point = (_float(before.get("x")), _float(before.get("y")))
                if not points or point != points[-1]:
                    points.append(point)
        after = self.steps[-1].get("position_after")
        if isinstance(after, dict) and after.get("x") not in (None, "") and after.get("y") not in (None, ""):
            point = (_float(after.get("x")), _float(after.get("y")))
            if not points or point != points[-1]:
                points.append(point)
        return points


def annotate_frame(run: ReplayRun, index: int, fresh: Optional[RerunStepResult] = None) -> Image.Image:
    step = run.steps[index]
    planner = step.get("planner_output")
    if not isinstance(planner, dict):
        planner = {}

    image = Image.open(run.step_frame_path(index)).convert("RGB")
    draw = ImageDraw.Draw(image)
    width, height = image.size
    center_x = width / 2.0
    center_y = height / 2.0

    # Search ROI used by the historical detector.
    search_px = _int(planner.get("search_px"), _int(run.settings.get("line_search_px"), 0))
    if search_px > 0:
        half = search_px / 2.0
        draw.rectangle(
            (center_x - half, center_y - half, center_x + half, center_y + half),
            outline=(255, 210, 0),
            width=2,
        )

    # Camera center / desired tracking point.
    arm = max(10, min(width, height) // 35)
    draw.line((center_x - arm, center_y, center_x + arm, center_y), fill=(0, 220, 255), width=2)
    draw.line((center_x, center_y - arm, center_x, center_y + arm), fill=(0, 220, 255), width=2)

    # Historical fitted line, reconstructed from recorded closest-point error + angle.
    err_x = _float(planner.get("pixel_error_x"))
    err_y = _float(planner.get("pixel_error_y"))
    detected_x = center_x + err_x
    detected_y = center_y + err_y
    angle = math.radians(_float(planner.get("angle_degrees")))
    vx = math.cos(angle)
    vy = math.sin(angle)
    line_half = max(width, height)
    draw.line(
        (
            detected_x - vx * line_half,
            detected_y - vy * line_half,
            detected_x + vx * line_half,
            detected_y + vy * line_half,
        ),
        fill=(0, 255, 80),
        width=3,
    )
    radius = 5
    draw.ellipse(
        (detected_x - radius, detected_y - radius, detected_x + radius, detected_y + radius),
        fill=(255, 70, 70),
        outline=(255, 255, 255),
        width=1,
    )
    draw.line((center_x, center_y, detected_x, detected_y), fill=(255, 70, 70), width=2)

    # Fresh rerun line from the installed detector/planner. This is drawn
    # separately so historical and rerun perception can be compared directly.
    if fresh is not None and fresh.fresh_terminal:
        fresh_plan = fresh.fresh
        fresh_err_x = _float(fresh_plan.get("pixel_error_x", fresh.fresh_detection.get("pixel_error_x")))
        fresh_err_y = _float(fresh_plan.get("pixel_error_y", fresh.fresh_detection.get("pixel_error_y")))
        fresh_x = center_x + fresh_err_x
        fresh_y = center_y + fresh_err_y
        fresh_angle = math.radians(
            _float(fresh_plan.get("angle_degrees", fresh.fresh_detection.get("angle_degrees")))
        )
        fresh_vx = math.cos(fresh_angle)
        fresh_vy = math.sin(fresh_angle)
        draw.line(
            (
                fresh_x - fresh_vx * line_half,
                fresh_y - fresh_vy * line_half,
                fresh_x + fresh_vx * line_half,
                fresh_y + fresh_vy * line_half,
            ),
            fill=(255, 145, 0),
            width=2,
        )
        fresh_radius = 4
        draw.ellipse(
            (fresh_x - fresh_radius, fresh_y - fresh_radius, fresh_x + fresh_radius, fresh_y + fresh_radius),
            outline=(255, 145, 0),
            width=2,
        )

    # Corner candidate marker if the historical detector reported one.
    if bool(planner.get("corner_candidate")) and bool(planner.get("corner_intersection_valid")):
        cx = _float(planner.get("corner_intersection_x_px"), center_x)
        cy = _float(planner.get("corner_intersection_y_px"), center_y)
        size = 9
        draw.line((cx - size, cy - size, cx + size, cy + size), fill=(255, 0, 255), width=3)
        draw.line((cx - size, cy + size, cx + size, cy - size), fill=(255, 0, 255), width=3)

    # Compact permanent legend. Use default bitmap font for portability.
    confidence = _float(planner.get("confidence"))
    step_id = _text(step.get("step_id"), str(index + 1))
    legend = f"Step {step_id}/{len(run.steps)}   historical conf {confidence:.1f}%   angle {math.degrees(angle):+.1f} deg"
    if fresh is not None:
        legend += f"   rerun {fresh.classification}"
    bbox = draw.textbbox((0, 0), legend, font=ImageFont.load_default())
    pad = 5
    draw.rectangle((6, 6, 6 + (bbox[2] - bbox[0]) + pad * 2, 6 + (bbox[3] - bbox[1]) + pad * 2), fill=(0, 0, 0))
    draw.text((6 + pad, 6 + pad), legend, fill=(255, 255, 255), font=ImageFont.load_default())
    return image


def _fmt_xy(value: Any) -> str:
    if not isinstance(value, dict):
        return "—"
    x = value.get("x")
    y = value.get("y")
    if x in (None, "") or y in (None, ""):
        return "—"
    return f"X {_float(x):.6f}   Y {_float(y):.6f}"


def _step_detail_text(
    run: ReplayRun,
    index: int,
    fresh: Optional[RerunStepResult] = None,
) -> str:
    step = run.steps[index]
    planner = step.get("planner_output")
    if not isinstance(planner, dict):
        planner = {}
    events = run.events_for_index(index)

    rows = [
        f"Run: {run.name}",
        f"Step: {step.get('step_id', index + 1)} / {len(run.steps)}   {step.get('step_label', '')}",
        f"Time: {step.get('first_timestamp_iso', '')}",
        "",
        f"Position before:  {_fmt_xy(step.get('position_before'))}",
        f"Delayed position: {_fmt_xy(step.get('delayed_position'))}",
        f"Position after:   {_fmt_xy(step.get('position_after'))}",
        "",
        "HISTORICAL",
        f"  Result/event:  {_text(step.get('terminal_result'), '—')} / {_text(step.get('terminal_event'), '—')}",
        f"  Reason:        {_text(step.get('terminal_reason'), '—') or '—'}",
        f"  Confidence:    {_float(planner.get('confidence')):.3f}%",
        f"  Points:        {_int(planner.get('point_count'))}",
        f"  Span / width:  {_float(planner.get('span_px')):.3f}px / {_float(planner.get('width_px')):.3f}px",
        f"  Pixel error:   X {_float(planner.get('pixel_error_x')):+.3f}px   Y {_float(planner.get('pixel_error_y')):+.3f}px",
        f"  Fit angle:     {_float(planner.get('angle_degrees')):+.3f} deg",
        f"  Heading:       {_text(planner.get('heading_state'), '—') or '—'}",
        f"  Move:          X {_float(planner.get('move_x')):+.6f}   Y {_float(planner.get('move_y')):+.6f}",
        f"  Target:        X {_float(planner.get('target_x')):.6f}   Y {_float(planner.get('target_y')):.6f}",
        f"  Correction:    X {_float(planner.get('correct_x')):+.6f}   Y {_float(planner.get('correct_y')):+.6f}",
        f"  State:         {_text(planner.get('correction_state'), '—') or '—'}",
    ]

    if fresh is None:
        rows.extend(
            [
                "",
                "RERUN",
                "  Not run yet. Click 'Rerun Existing Planner'.",
            ]
        )
    else:
        fp = fresh.fresh
        fd = fresh.fresh_detection
        rows.extend(
            [
                "",
                f"RERUN — {fresh.classification}",
                f"  Result/event:  {fresh.fresh_result or '—'} / {fresh.fresh_terminal or '—'}",
                f"  Confidence:    {_float(fp.get('confidence', fd.get('confidence'))):.3f}%",
                f"  Points:        {_int(fp.get('point_count', fd.get('point_count')))}",
                f"  Span / width:  {_float(fp.get('span_px', fd.get('span_px'))):.3f}px / {_float(fp.get('width_px', fd.get('width_px'))):.3f}px",
                f"  Pixel error:   X {_float(fp.get('pixel_error_x', fd.get('pixel_error_x'))):+.3f}px   Y {_float(fp.get('pixel_error_y', fd.get('pixel_error_y'))):+.3f}px",
                f"  Fit angle:     {_float(fp.get('angle_degrees', fd.get('angle_degrees'))):+.3f} deg",
                f"  Heading:       {_text(fp.get('heading_state'), '—') or '—'}",
                f"  Move:          X {_float(fp.get('move_x')):+.6f}   Y {_float(fp.get('move_y')):+.6f}",
                f"  Target:        X {_float(fp.get('target_x')):.6f}   Y {_float(fp.get('target_y')):.6f}",
                f"  Correction:    X {_float(fp.get('correct_x')):+.6f}   Y {_float(fp.get('correct_y')):+.6f}",
                f"  State:         {_text(fp.get('correction_state'), '—') or '—'}",
                "",
                "COMPARISON",
                f"  Decision match:    {fresh.decision_match}",
                f"  Heading text match:{' ' if fresh.heading_state_match else ''}{fresh.heading_state_match}",
                f"  Move delta:        X {fresh.move_delta_x:+.6f}   Y {fresh.move_delta_y:+.6f}",
                f"  Vector difference: {fresh.move_delta_length:.6f}",
                f"  Target difference: {fresh.target_delta:.6f}",
                f"  Angle difference:  {fresh.angle_delta_degrees:.3f} deg",
                f"  Confidence diff:   {fresh.confidence_delta:.3f}%",
                f"  Notes:             {', '.join(fresh.notes) if fresh.notes else '—'}",
            ]
        )

    rows.extend(
        [
            "",
            "Recorded events:",
            "  " + " → ".join(_text(event.get("event")) for event in events),
        ]
    )
    return "\n".join(rows)


def _belief_detail_text(state: Optional[BeliefState]) -> str:
    if state is None:
        return "\n\nM4 PATH BELIEF\n  Not built yet. Click 'Build Path Belief'."
    radius = "straight / very large" if state.implied_radius <= 0.0 else f"{state.implied_radius:.3f} in"
    context = state.context
    rows = [
        "",
        "M4 PATH BELIEF (READ-ONLY)",
        f"  Measurement:    X {state.measurement_x:.6f}   Y {state.measurement_y:.6f}",
        f"  Estimate:       X {state.estimate_x:.6f}   Y {state.estimate_y:.6f}",
        f"  Measured tangent:{state.measured_heading_degrees:+.3f} deg",
        f"  Belief tangent: {state.tangent_degrees:+.3f} deg",
        f"  Curvature raw:  {state.raw_curvature:+.4f} 1/in",
        f"  Curvature belief:{state.curvature:+.4f} 1/in",
        f"  Curvature trend:{state.curvature_trend:+.4f} 1/in^2",
        f"  Implied radius: {radius}",
        f"  Usable context: {state.usable_lookahead:.4f} in",
        f"  Predict horizon:{state.prediction_horizon:.4f} in",
        f"  Belief conf:    {state.belief_confidence:.2f}%",
        f"  Uncertainty:    {state.uncertainty:.3f}",
        f"  Pos innovation: {state.position_innovation:.5f} in",
        f"  Head innovation:{state.heading_innovation_degrees:+.3f} deg",
        f"  Connected area: {context.seed_component_area_px} px",
        f"  Other geometry: {context.other_geometry_count}",
        f"  Forward support:{context.forward_support_ratio * 100.0:.1f}%",
        f"  Reason:          {state.reason}",
    ]
    return "\n".join(rows)


def _overlay_belief_on_frame(image: Image.Image, state: Optional[BeliefState]) -> Image.Image:
    if state is None:
        return image
    image = image.copy()
    draw = ImageDraw.Draw(image)
    contour = state.context.component_contour
    if len(contour) >= 2:
        closed = list(contour) + [contour[0]]
        draw.line(closed, fill=(190, 80, 255), width=2)
    if state.context.found:
        sx, sy = state.context.seed_x_px, state.context.seed_y_px
        r = 6
        draw.ellipse((sx-r, sy-r, sx+r, sy+r), outline=(255, 230, 0), width=2)
    label = (
        f"M4 belief {state.belief_confidence:.1f}%  "
        f"k {state.curvature:+.3f}/in  look {state.usable_lookahead:.3f}in  "
        f"predict {state.prediction_horizon:.3f}in"
    )
    font = ImageFont.load_default()
    bbox = draw.textbbox((0, 0), label, font=font)
    y = max(8, image.height - (bbox[3] - bbox[1]) - 18)
    draw.rectangle((6, y-5, 12 + (bbox[2]-bbox[0]), y + (bbox[3]-bbox[1]) + 5), fill=(0,0,0))
    draw.text((9, y), label, fill=(255,255,255), font=font)
    return image


class TrajectoryPopout(tk.Toplevel):
    """Large synchronized machine-space path view with zoom and pan."""

    def __init__(self, owner: "ReplayViewer") -> None:
        super().__init__(owner)
        self.owner = owner
        self.title("FabScan M4 Path Belief - Zoomable Machine-Space View")
        self.geometry("1200x850")
        self.minsize(700, 500)
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self._drag_anchor: Optional[tuple[int, int]] = None
        self._fullscreen = False

        toolbar = ttk.Frame(self, padding=(7, 6))
        toolbar.pack(fill=tk.X)
        ttk.Button(toolbar, text="|<", width=4, command=owner.first_step).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="< Previous", command=owner.previous_step).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(toolbar, text="Next >", command=owner.next_step).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(toolbar, text=">|", width=4, command=owner.last_step).pack(side=tk.LEFT, padx=(4, 10))
        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        ttk.Button(toolbar, text="Fit", command=self.fit).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="−", width=4, command=lambda: self.change_zoom(1 / 1.25)).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(toolbar, text="+", width=4, command=lambda: self.change_zoom(1.25)).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(toolbar, text="Full Screen", command=self.toggle_fullscreen).pack(side=tk.LEFT, padx=(8, 0))
        self.info_var = tk.StringVar(value="")
        ttk.Label(toolbar, textvariable=self.info_var).pack(side=tk.RIGHT)

        self.canvas = tk.Canvas(self, background="white", highlightthickness=0, cursor="fleur")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.refresh())
        self.canvas.bind("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind("<Button-4>", lambda _event: self.change_zoom(1.15))
        self.canvas.bind("<Button-5>", lambda _event: self.change_zoom(1 / 1.15))
        self.canvas.bind("<ButtonPress-1>", self._pan_start)
        self.canvas.bind("<B1-Motion>", self._pan_move)
        self.canvas.bind("<ButtonRelease-1>", self._pan_end)

        self.bind("<Left>", lambda _event: owner.previous_step())
        self.bind("<Right>", lambda _event: owner.next_step())
        self.bind("<Home>", lambda _event: owner.first_step())
        self.bind("<End>", lambda _event: owner.last_step())
        self.bind("<Key-f>", lambda _event: self.fit())
        self.bind("<Key-F>", lambda _event: self.fit())
        self.bind("<F11>", lambda _event: self.toggle_fullscreen())
        self.bind("<Escape>", self._escape)
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.after_idle(self.refresh)

    def refresh(self) -> None:
        try:
            if not self.winfo_exists():
                return
        except tk.TclError:
            return
        run = self.owner.run
        if run is None:
            self.info_var.set("No replay loaded")
        else:
            self.info_var.set(f"Step {self.owner.index + 1}/{len(run.steps)}   Zoom {self.zoom:.2f}x")
        self.owner._render_trajectory_canvas(
            self.canvas,
            zoom=self.zoom,
            pan_x=self.pan_x,
            pan_y=self.pan_y,
            show_help=True,
        )

    def fit(self) -> None:
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.refresh()

    def change_zoom(self, factor: float) -> None:
        self.zoom = max(0.1, min(100.0, self.zoom * factor))
        self.refresh()

    def _on_mousewheel(self, event: Any) -> str:
        factor = 1.15 if event.delta > 0 else 1 / 1.15
        self.change_zoom(factor)
        return "break"

    def _pan_start(self, event: Any) -> None:
        self._drag_anchor = (event.x, event.y)

    def _pan_move(self, event: Any) -> None:
        if self._drag_anchor is None:
            return
        old_x, old_y = self._drag_anchor
        self.pan_x += event.x - old_x
        self.pan_y += event.y - old_y
        self._drag_anchor = (event.x, event.y)
        self.refresh()

    def _pan_end(self, _event: Any) -> None:
        self._drag_anchor = None

    def toggle_fullscreen(self) -> None:
        self._fullscreen = not self._fullscreen
        self.attributes("-fullscreen", self._fullscreen)
        self.refresh()

    def _escape(self, _event: Any) -> None:
        if self._fullscreen:
            self._fullscreen = False
            self.attributes("-fullscreen", False)
            self.refresh()

    def close(self) -> None:
        self.owner._trajectory_popout_closed()
        try:
            self.destroy()
        except tk.TclError:
            pass


class ReplayViewer(tk.Toplevel):
    def __init__(
        self,
        parent: Optional[tk.Misc] = None,
        run_dir: Optional[Path | str] = None,
        dialog_cls: Optional[type] = None,
    ) -> None:
        super().__init__(parent)
        self.title("FabScan Follow Replay Viewer - Milestone 5.1 (Virtual Perception + Path Belief)")
        self.geometry("1450x900")
        self.minsize(1120, 700)
        if parent is not None:
            self.transient(parent)

        self.run: Optional[ReplayRun] = None
        self.dialog_cls = dialog_cls
        self.rerun_report: Optional[RerunReport] = None
        self.belief_states: Optional[list[BeliefState]] = None
        self._belief_queue: queue.Queue[tuple[str, Any]] = queue.Queue()
        self._belief_thread: Optional[threading.Thread] = None
        self._belief_poll_job: Optional[str] = None
        self._rerun_queue: queue.Queue[tuple[str, Any]] = queue.Queue()
        self._rerun_thread: Optional[threading.Thread] = None
        self._rerun_poll_job: Optional[str] = None
        self.index = 0
        self.playing = False
        self._play_job: Optional[str] = None
        self._photo: Optional[ImageTk.PhotoImage] = None
        self._base_annotated: Optional[Image.Image] = None
        self._resize_job: Optional[str] = None
        self._trajectory_popout: Optional["TrajectoryPopout"] = None

        self.speed_ms_var = tk.IntVar(value=500)
        self.step_var = tk.StringVar(value="1")
        self.run_var = tk.StringVar(value="No replay loaded")
        self.status_var = tk.StringVar(value="Replay is read-only: M4 belief analysis and M3 rerun cannot command LinuxCNC.")
        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.bind("<Left>", lambda _event: self.previous_step())
        self.bind("<Right>", lambda _event: self.next_step())
        self.bind("<Home>", lambda _event: self.first_step())
        self.bind("<End>", lambda _event: self.last_step())
        self.bind("<space>", lambda _event: self.toggle_play())

        if run_dir is not None:
            self.load_run(run_dir)

    def _build_ui(self) -> None:
        toolbar = ttk.Frame(self, padding=(8, 7))
        toolbar.pack(fill=tk.X)
        ttk.Button(toolbar, text="Open Replay…", command=self.choose_run).pack(side=tk.LEFT)
        self.rerun_button = ttk.Button(
            toolbar, text="Rerun Existing Planner", command=self.start_rerun
        )
        self.rerun_button.pack(side=tk.LEFT, padx=(6, 0))
        self.belief_button = ttk.Button(
            toolbar, text="Build Path Belief", command=self.start_belief
        )
        self.belief_button.pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            toolbar, text="SVG Simulator / Diagnostics…", command=self.open_svg_simulator
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)
        ttk.Button(toolbar, text="|<", width=4, command=self.first_step).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="< Previous", command=self.previous_step).pack(side=tk.LEFT, padx=(4, 0))
        self.play_button = ttk.Button(toolbar, text="Play", width=8, command=self.toggle_play)
        self.play_button.pack(side=tk.LEFT, padx=4)
        ttk.Button(toolbar, text="Next >", command=self.next_step).pack(side=tk.LEFT)
        ttk.Button(toolbar, text=">|", width=4, command=self.last_step).pack(side=tk.LEFT, padx=(4, 10))
        ttk.Label(toolbar, text="Step:").pack(side=tk.LEFT)
        entry = ttk.Entry(toolbar, textvariable=self.step_var, width=7)
        entry.pack(side=tk.LEFT, padx=(4, 3))
        entry.bind("<Return>", lambda _event: self.jump_to_entry())
        ttk.Button(toolbar, text="Go", width=4, command=self.jump_to_entry).pack(side=tk.LEFT)
        ttk.Label(toolbar, text="Playback:").pack(side=tk.LEFT, padx=(12, 4))
        speed = ttk.Combobox(
            toolbar,
            width=8,
            state="readonly",
            values=("100 ms", "250 ms", "500 ms", "1000 ms"),
        )
        speed.set("500 ms")
        speed.pack(side=tk.LEFT)

        def speed_changed(_event: Any) -> None:
            value = speed.get().split()[0]
            self.speed_ms_var.set(_int(value, 500))

        speed.bind("<<ComboboxSelected>>", speed_changed)
        ttk.Label(toolbar, textvariable=self.run_var).pack(side=tk.RIGHT, padx=(12, 0))

        pane = ttk.Panedwindow(self, orient=tk.HORIZONTAL)
        pane.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 6))

        left = ttk.Frame(pane)
        right = ttk.Frame(pane, width=470)
        pane.add(left, weight=4)
        pane.add(right, weight=2)

        image_group = ttk.LabelFrame(left, text="Recorded decision frame", padding=6)
        image_group.pack(fill=tk.BOTH, expand=True)
        self.image_canvas = tk.Canvas(image_group, background="#151515", highlightthickness=0)
        self.image_canvas.pack(fill=tk.BOTH, expand=True)
        self.image_canvas.bind("<Configure>", self._on_image_resize)

        trajectory_group = ttk.LabelFrame(right, text="Historical path + M4 belief / prediction", padding=5)
        trajectory_group.pack(fill=tk.BOTH, expand=False)
        trajectory_toolbar = ttk.Frame(trajectory_group)
        trajectory_toolbar.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(trajectory_toolbar, text="Machine-space path").pack(side=tk.LEFT)
        ttk.Button(
            trajectory_toolbar,
            text="Pop Out / Zoom…",
            command=self.open_trajectory_popout,
        ).pack(side=tk.RIGHT)
        self.trajectory_canvas = tk.Canvas(
            trajectory_group,
            width=440,
            height=300,
            background="white",
            highlightthickness=1,
            highlightbackground="#888888",
        )
        self.trajectory_canvas.pack(fill=tk.BOTH, expand=True)
        self.trajectory_canvas.bind("<Configure>", lambda _event: self._draw_trajectory())

        details_group = ttk.LabelFrame(right, text="Historical / rerun / M4 belief", padding=5)
        details_group.pack(fill=tk.BOTH, expand=True, pady=(7, 0))
        self.details = tk.Text(details_group, wrap=tk.WORD, width=58, height=25, font=("TkFixedFont", 9))
        scroll = ttk.Scrollbar(details_group, orient=tk.VERTICAL, command=self.details.yview)
        self.details.configure(yscrollcommand=scroll.set)
        self.details.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.details.configure(state=tk.DISABLED)

        status = ttk.Label(self, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W, padding=(6, 3))
        status.pack(fill=tk.X, side=tk.BOTTOM)

    def open_svg_simulator(self) -> None:
        asset_dir = Path(__file__).resolve().parents[1] / "simulator_assets"
        try:
            if self.run is None:
                raise ValueError("Load a replay first so M5.2 can use its detector settings and camera calibration.")
            launch_svg_simulator(
                self,
                asset_dir,
                dialog_cls=self.dialog_cls,
                run_dir=self.run.run_dir,
            )
        except Exception as exc:
            traceback.print_exc()
            messagebox.showerror(
                "SVG Simulator",
                f"Could not open the SVG Simulator:\n\n{type(exc).__name__}: {exc}",
                parent=self,
            )

    def choose_run(self) -> None:
        initial = Path.cwd()
        if self.run is not None:
            initial = self.run.run_dir.parent
        selected = filedialog.askdirectory(
            parent=self,
            title="Select a FabScan replay run folder",
            initialdir=str(initial),
            mustexist=True,
        )
        if selected:
            self.load_run(selected)

    def load_run(self, run_dir: Path | str) -> None:
        self.stop_playback()
        try:
            run = ReplayRun.load(run_dir)
        except Exception as exc:
            messagebox.showerror("Replay could not be opened", str(exc), parent=self)
            self.status_var.set(f"Replay load failed: {exc}")
            return
        self.run = run
        self.rerun_report = None
        self.belief_states = None
        self.index = 0
        self.run_var.set(f"{run.name}   ({len(run.steps)} steps)")
        self.status_var.set(
            f"Loaded {run.run_dir} — Build Path Belief to run the new read-only estimator, or rerun the old planner."
        )
        self.show_current_step()

    def show_current_step(self) -> None:
        run = self.run
        if run is None:
            return
        self.index = max(0, min(self.index, len(run.steps) - 1))
        self.step_var.set(str(self.index + 1))
        fresh = None
        if self.rerun_report is not None and self.index < len(self.rerun_report.results):
            fresh = self.rerun_report.results[self.index]
        belief = None
        if self.belief_states is not None and self.index < len(self.belief_states):
            belief = self.belief_states[self.index]
        try:
            self._base_annotated = _overlay_belief_on_frame(annotate_frame(run, self.index, fresh), belief)
        except Exception as exc:
            self._base_annotated = None
            self.status_var.set(f"Could not render step {self.index + 1}: {exc}")
        self._draw_image()
        self._draw_trajectory()
        if self._trajectory_popout is not None:
            self._trajectory_popout.refresh()
        self.details.configure(state=tk.NORMAL)
        self.details.delete("1.0", tk.END)
        self.details.insert("1.0", _step_detail_text(run, self.index, fresh) + _belief_detail_text(belief))
        self.details.configure(state=tk.DISABLED)

    def _on_image_resize(self, _event: Any) -> None:
        if self._resize_job is not None:
            try:
                self.after_cancel(self._resize_job)
            except tk.TclError:
                pass
        self._resize_job = self.after(60, self._draw_image)

    def _draw_image(self) -> None:
        self._resize_job = None
        image = self._base_annotated
        canvas = self.image_canvas
        canvas.delete("all")
        if image is None:
            canvas.create_text(20, 20, text="Open a replay run to begin.", fill="white", anchor=tk.NW)
            return
        available_w = max(100, canvas.winfo_width() - 12)
        available_h = max(100, canvas.winfo_height() - 12)
        scale = min(available_w / image.width, available_h / image.height)
        display_size = (max(1, int(image.width * scale)), max(1, int(image.height * scale)))
        resized = image.resize(display_size, Image.Resampling.LANCZOS)
        self._photo = ImageTk.PhotoImage(resized)
        canvas.create_image(canvas.winfo_width() / 2, canvas.winfo_height() / 2, image=self._photo, anchor=tk.CENTER)

    def _trajectory_world_data(self) -> tuple[
        list[tuple[float, float]],
        Optional[BeliefState],
        list[tuple[float, float]],
        list[tuple[float, float]],
        list[tuple[float, float]],
    ]:
        run = self.run
        if run is None:
            return [], None, [], [], []
        historical_points = run.trajectory_points()
        belief = None
        belief_history: list[tuple[float, float]] = []
        predicted: list[tuple[float, float]] = []
        recent: list[tuple[float, float]] = []
        if self.belief_states is not None and self.index < len(self.belief_states):
            belief = self.belief_states[self.index]
            belief_history = [
                (state.estimate_x, state.estimate_y)
                for state in self.belief_states[: self.index + 1]
                if state.measurement_valid
            ]
            predicted = list(belief.predicted_points)
            recent = list(belief.recent_measurements)
        return historical_points, belief, belief_history, predicted, recent

    def _render_trajectory_canvas(
        self,
        canvas: tk.Canvas,
        *,
        zoom: float = 1.0,
        pan_x: float = 0.0,
        pan_y: float = 0.0,
        show_help: bool = False,
    ) -> None:
        canvas.delete("all")
        run = self.run
        if run is None:
            canvas.create_text(12, 12, text="No replay loaded", anchor=tk.NW)
            return

        historical_points, belief, belief_history, predicted, recent = self._trajectory_world_data()
        if not historical_points:
            canvas.create_text(12, 12, text="No recorded XY trajectory", anchor=tk.NW)
            return

        all_points = list(historical_points) + belief_history + predicted + recent
        width = max(160, canvas.winfo_width())
        height = max(140, canvas.winfo_height())
        pad = 36.0 if width > 600 else 28.0
        xs = [p[0] for p in all_points]
        ys = [p[1] for p in all_points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        dx = max(max_x - min_x, 1e-6)
        dy = max(max_y - min_y, 1e-6)
        fit_scale = min((width - 2 * pad) / dx, (height - 2 * pad) / dy)
        scale = fit_scale * max(0.05, zoom)
        center_x = (min_x + max_x) / 2.0
        center_y = (min_y + max_y) / 2.0

        def screen(point: tuple[float, float]) -> tuple[float, float]:
            return (
                width / 2.0 + (point[0] - center_x) * scale + pan_x,
                height / 2.0 - (point[1] - center_y) * scale + pan_y,
            )

        def line(points: list[tuple[float, float]], **kwargs: Any) -> None:
            if len(points) < 2:
                return
            coords: list[float] = []
            for point in points:
                coords.extend(screen(point))
            canvas.create_line(*coords, **kwargs)

        # Light machine-space grid.  It intentionally scales with the view, so zooming
        # gives a visual sense of distance without changing any belief data.
        world_per_100px = 100.0 / max(scale, 1e-9)
        grid_candidates = (0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0)
        grid_step = grid_candidates[-1]
        for candidate in grid_candidates:
            if candidate >= world_per_100px:
                grid_step = candidate
                break
        left_world = center_x + (-width / 2.0 - pan_x) / scale
        right_world = center_x + (width / 2.0 - pan_x) / scale
        bottom_world = center_y - (height / 2.0 - pan_y) / scale
        top_world = center_y - (-height / 2.0 - pan_y) / scale
        gx = math.floor(left_world / grid_step) * grid_step
        while gx <= right_world + grid_step:
            sx, _ = screen((gx, center_y))
            canvas.create_line(sx, 0, sx, height, fill="#eeeeee")
            gx += grid_step
        gy = math.floor(bottom_world / grid_step) * grid_step
        while gy <= top_world + grid_step:
            _, sy = screen((center_x, gy))
            canvas.create_line(0, sy, width, sy, fill="#eeeeee")
            gy += grid_step

        line(list(historical_points), fill="#666666", width=2)

        step = run.steps[self.index]
        before = step.get("position_before")
        after = step.get("position_after")
        if isinstance(before, dict) and isinstance(after, dict):
            p0 = (_float(before.get("x")), _float(before.get("y")))
            p1 = (_float(after.get("x")), _float(after.get("y")))
            x0, y0 = screen(p0)
            x1, y1 = screen(p1)
            canvas.create_line(x0, y0, x1, y1, fill="#d62728", width=4, arrow=tk.LAST)

        if self.rerun_report is not None and self.index < len(self.rerun_report.results):
            fresh = self.rerun_report.results[self.index]
            if isinstance(before, dict) and fresh.fresh_terminal == "MOVE_PLAN":
                start = (_float(before.get("x")), _float(before.get("y")))
                target = (
                    _float(fresh.fresh.get("target_x"), start[0]),
                    _float(fresh.fresh.get("target_y"), start[1]),
                )
                x0, y0 = screen(start)
                x1, y1 = screen(target)
                canvas.create_line(x0, y0, x1, y1, fill="#1f77b4", width=3, arrow=tk.LAST)

        if belief is not None:
            line(belief_history, fill="#00a7b5", width=2)
            for point in recent:
                x, y = screen(point)
                canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill="#cc33aa", outline="")
            line(predicted, fill="#d6a600", width=4, arrow=tk.LAST)
            bx, by = screen((belief.estimate_x, belief.estimate_y))
            canvas.create_oval(bx - 6, by - 6, bx + 6, by + 6, fill="#00a7b5", outline="black")
            tangent_end = (
                belief.estimate_x + belief.tangent_x * 0.10,
                belief.estimate_y + belief.tangent_y * 0.10,
            )
            tx, ty = screen(tangent_end)
            canvas.create_line(bx, by, tx, ty, fill="#111111", width=2, arrow=tk.LAST)
            canvas.create_text(
                width - 8,
                28,
                text=f"M4 conf {belief.belief_confidence:.1f}%  k {belief.curvature:+.3f}/in",
                anchor=tk.NE,
                fill="#006b75",
            )

        canvas.create_rectangle(6, 6, 230, 65, fill="white", outline="#bbbbbb")
        canvas.create_text(12, 10, text=f"X {min_x:.4f}…{max_x:.4f}", anchor=tk.NW)
        canvas.create_text(12, 27, text=f"Y {min_y:.4f}…{max_y:.4f}", anchor=tk.NW)
        canvas.create_text(12, 44, text=f"Grid {grid_step:.4f} in   Zoom {zoom:.2f}x", anchor=tk.NW)
        canvas.create_text(width - 8, 8, text=f"Step {self.index + 1}/{len(run.steps)}", anchor=tk.NE)
        if show_help:
            canvas.create_text(
                width - 8,
                height - 8,
                text="Wheel: zoom   Drag: pan   F: fit   F11: full screen   Esc: exit full screen",
                anchor=tk.SE,
                fill="#555555",
            )

    def _draw_trajectory(self) -> None:
        self._render_trajectory_canvas(self.trajectory_canvas)

    def open_trajectory_popout(self) -> None:
        if self._trajectory_popout is not None:
            try:
                if self._trajectory_popout.winfo_exists():
                    self._trajectory_popout.deiconify()
                    self._trajectory_popout.lift()
                    self._trajectory_popout.focus_force()
                    self._trajectory_popout.refresh()
                    return
            except tk.TclError:
                pass
        self._trajectory_popout = TrajectoryPopout(self)

    def _trajectory_popout_closed(self) -> None:
        self._trajectory_popout = None

    def start_belief(self) -> None:
        if self.run is None:
            messagebox.showinfo("No replay loaded", "Open a replay run first.", parent=self)
            return
        if self._belief_thread is not None and self._belief_thread.is_alive():
            return
        self.stop_playback()
        self.belief_states = None
        self.belief_button.configure(state=tk.DISABLED)
        self.status_var.set(f"Building M4 path belief 0/{len(self.run.steps)} steps…")
        run_dir = self.run.run_dir

        def worker() -> None:
            try:
                analyzer = ReplayBeliefAnalyzer(run_dir)
                def progress(done: int, total: int) -> None:
                    self._belief_queue.put(("progress", (done, total)))
                states = analyzer.run_all(progress=progress)
                self._belief_queue.put(("done", (states, analyzer.summary(states))))
            except Exception:
                self._belief_queue.put(("error", traceback.format_exc()))

        self._belief_thread = threading.Thread(
            target=worker,
            name="FabScanPathBeliefM4",
            daemon=True,
        )
        self._belief_thread.start()
        self._poll_belief_queue()

    def _poll_belief_queue(self) -> None:
        self._belief_poll_job = None
        finished = False
        while True:
            try:
                kind, payload = self._belief_queue.get_nowait()
            except queue.Empty:
                break
            if kind == "progress":
                done, total = payload
                self.status_var.set(f"Building M4 path belief {done}/{total} steps…")
            elif kind == "done":
                self.belief_states, summary = payload
                self.belief_button.configure(state=tk.NORMAL)
                self.status_var.set(
                    f"M4 belief complete: {summary['valid_measurements']}/{summary['steps']} measurements; "
                    f"mean look-ahead {summary['mean_usable_lookahead']:.3f} in; "
                    f"mean belief confidence {summary['mean_belief_confidence']:.1f}%."
                )
                self.show_current_step()
                finished = True
            elif kind == "error":
                self.belief_button.configure(state=tk.NORMAL)
                self.status_var.set("M4 belief build failed; see error dialog.")
                messagebox.showerror("Path belief build failed", str(payload), parent=self)
                finished = True
        if not finished and self._belief_thread is not None and self._belief_thread.is_alive():
            self._belief_poll_job = self.after(80, self._poll_belief_queue)
        elif not finished:
            self.belief_button.configure(state=tk.NORMAL)

    def start_rerun(self) -> None:
        if self.run is None:
            messagebox.showinfo("No replay loaded", "Open a replay run first.", parent=self)
            return
        if self.dialog_cls is None:
            messagebox.showerror(
                "Rerun unavailable",
                "The viewer was not opened from a Camera Calibration dialog, so the installed planner class is unavailable.",
                parent=self,
            )
            return
        if self._rerun_thread is not None and self._rerun_thread.is_alive():
            return
        self.stop_playback()
        self.rerun_report = None
        self.rerun_button.configure(state=tk.DISABLED)
        self.status_var.set(f"Rerunning 0/{len(self.run.steps)} steps with the installed detector/planner…")
        run_dir = self.run.run_dir
        dialog_cls = self.dialog_cls

        def worker() -> None:
            try:
                rerunner = ExistingPlannerRerunner(dialog_cls, run_dir)

                def progress(done: int, total: int) -> None:
                    self._rerun_queue.put(("progress", (done, total)))

                report = rerunner.run_all(progress=progress)
                self._rerun_queue.put(("done", report))
            except Exception:
                self._rerun_queue.put(("error", traceback.format_exc()))

        self._rerun_thread = threading.Thread(
            target=worker,
            name="FabScanReplayRerunM3",
            daemon=True,
        )
        self._rerun_thread.start()
        self._poll_rerun_queue()

    def _poll_rerun_queue(self) -> None:
        self._rerun_poll_job = None
        finished = False
        while True:
            try:
                kind, payload = self._rerun_queue.get_nowait()
            except queue.Empty:
                break
            if kind == "progress":
                done, total = payload
                self.status_var.set(f"Rerunning {done}/{total} steps with the installed detector/planner…")
            elif kind == "done":
                self.rerun_report = payload
                report = self.rerun_report
                self.status_var.set(
                    f"Rerun complete: {report.decision_matches}/{report.step_count} decisions match; "
                    f"{report.close_matches}/{report.step_count} within comparison tolerance; "
                    f"max move delta {report.max_move_delta:.6f}."
                )
                self.rerun_button.configure(state=tk.NORMAL)
                self.show_current_step()
                finished = True
            elif kind == "error":
                self.rerun_button.configure(state=tk.NORMAL)
                self.status_var.set("Rerun failed; see error dialog.")
                messagebox.showerror("Planner rerun failed", str(payload), parent=self)
                finished = True
        if not finished and self._rerun_thread is not None and self._rerun_thread.is_alive():
            self._rerun_poll_job = self.after(80, self._poll_rerun_queue)
        elif not finished:
            self.rerun_button.configure(state=tk.NORMAL)

    def first_step(self) -> None:
        if self.run is not None:
            self.index = 0
            self.show_current_step()

    def last_step(self) -> None:
        if self.run is not None:
            self.index = len(self.run.steps) - 1
            self.show_current_step()

    def previous_step(self) -> None:
        if self.run is not None:
            self.index = max(0, self.index - 1)
            self.show_current_step()

    def next_step(self) -> None:
        if self.run is None:
            return
        if self.index >= len(self.run.steps) - 1:
            self.stop_playback()
            return
        self.index += 1
        self.show_current_step()

    def jump_to_entry(self) -> None:
        if self.run is None:
            return
        requested = _int(self.step_var.get(), self.index + 1)
        self.index = max(0, min(len(self.run.steps) - 1, requested - 1))
        self.show_current_step()

    def toggle_play(self) -> None:
        if self.run is None:
            return
        if self.playing:
            self.stop_playback()
        else:
            if self.index >= len(self.run.steps) - 1:
                self.index = 0
            self.playing = True
            self.play_button.configure(text="Pause")
            self._schedule_next()

    def _schedule_next(self) -> None:
        if not self.playing:
            return
        self._play_job = self.after(max(50, self.speed_ms_var.get()), self._play_tick)

    def _play_tick(self) -> None:
        self._play_job = None
        if not self.playing:
            return
        if self.run is None or self.index >= len(self.run.steps) - 1:
            self.stop_playback()
            return
        self.index += 1
        self.show_current_step()
        self._schedule_next()

    def stop_playback(self) -> None:
        self.playing = False
        self.play_button.configure(text="Play")
        if self._play_job is not None:
            try:
                self.after_cancel(self._play_job)
            except tk.TclError:
                pass
            self._play_job = None

    def close(self) -> None:
        self.stop_playback()
        if self._trajectory_popout is not None:
            try:
                self._trajectory_popout.destroy()
            except tk.TclError:
                pass
            self._trajectory_popout = None
        if self._rerun_poll_job is not None:
            try:
                self.after_cancel(self._rerun_poll_job)
            except tk.TclError:
                pass
            self._rerun_poll_job = None
        if self._belief_poll_job is not None:
            try:
                self.after_cancel(self._belief_poll_job)
            except tk.TclError:
                pass
            self._belief_poll_job = None
        try:
            self.destroy()
        except tk.TclError:
            pass


def _project_root_from_module(dialog_cls: type) -> Path:
    module = sys.modules.get(dialog_cls.__module__)
    module_file = Path(getattr(module, "__file__", Path.cwd())).resolve()
    return module_file.parent.parent


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


def open_replay_viewer(parent: tk.Misc, project_root: Optional[Path] = None, dialog_cls: Optional[type] = None) -> Optional[ReplayViewer]:
    root = Path(project_root or Path.cwd()).resolve()
    replay_root = root / "replays"
    selected = filedialog.askdirectory(
        parent=parent,
        title="Select a FabScan replay run folder",
        initialdir=str(replay_root if replay_root.is_dir() else root),
        mustexist=True,
    )
    if not selected:
        return None
    viewer = ReplayViewer(parent, selected, dialog_cls=dialog_cls)
    viewer.grab_set()
    return viewer


def _add_replay_button(dialog: Any, project_root: Path) -> None:
    parent = _find_follow_controls_parent(dialog)
    if parent is None:
        return
    max_row = -1
    for child in parent.winfo_children():
        try:
            info = child.grid_info()
            max_row = max(max_row, int(info.get("row", -1)))
        except Exception:
            pass
    button = ttk.Button(
        parent,
        text="Replay Follow Run…",
        command=lambda: open_replay_viewer(dialog, project_root, type(dialog)),
    )
    button.grid(row=max_row + 1, column=0, columnspan=2, sticky=tk.W, pady=(5, 0))
    dialog._replay_follow_button = button


def install_replay_viewer_support(dialog_cls: type) -> None:
    if getattr(dialog_cls, "_fabscan_replay_viewer_installed", False):
        return
    original_init = dialog_cls.__init__

    def wrapped_init(self: Any, *args: Any, **kwargs: Any) -> None:
        original_init(self, *args, **kwargs)
        try:
            project_root = _project_root_from_module(dialog_cls)
            _add_replay_button(self, project_root)
            current_title = str(self.title())
            if "Replay M4.1" not in current_title:
                current_title = (current_title
                    .replace(" + Replay M2", "")
                    .replace(" + Replay M3", "")
                    .replace(" + Replay M4", ""))
                self.title(f"{current_title} + Replay M4.1")
        except Exception:
            traceback.print_exc()

    dialog_cls.__init__ = wrapped_init
    dialog_cls._fabscan_replay_viewer_installed = True


def _summary(run: ReplayRun) -> str:
    return "\n".join(
        [
            f"Run:        {run.name}",
            f"Folder:     {run.run_dir}",
            f"Format:     {run.manifest.get('format_version')}",
            f"Status:     {run.manifest.get('status')}",
            f"Complete:   {run.manifest.get('complete')}",
            f"Steps:      {len(run.steps)}",
            f"Frames:     {run.manifest.get('frame_count', len(run.steps))}",
            f"Events:     {run.manifest.get('event_count', sum(len(v) for v in run.events_by_step.values()))}",
            f"Start:      {run.manifest.get('start_time_local', '')}",
        ]
    )


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="FabScan Milestone-4 world-space path-belief replay viewer")
    parser.add_argument("run_dir", nargs="?", help="Recorded replay run folder")
    parser.add_argument("--summary", action="store_true", help="Print replay summary without opening Tk")
    parser.add_argument("--export-step", type=int, metavar="N", help="Export one annotated frame without opening Tk")
    parser.add_argument("--output", type=Path, help="Output PNG for --export-step")
    args = parser.parse_args(argv)

    if args.summary or args.export_step is not None:
        if not args.run_dir:
            parser.error("run_dir is required for --summary or --export-step")
        run = ReplayRun.load(args.run_dir)
        if args.summary:
            print(_summary(run))
        if args.export_step is not None:
            index = max(0, min(len(run.steps) - 1, args.export_step - 1))
            output = args.output or Path(f"replay_step_{index + 1:06d}.png")
            annotate_frame(run, index).save(output)
            print(f"Exported:   {output.resolve()}")
        return 0

    root = tk.Tk()
    root.withdraw()
    try:
        from fabscan.camera_calibration import CameraCalibrationDialog as _DialogClass
    except Exception:
        _DialogClass = None
    viewer = ReplayViewer(root, args.run_dir, dialog_cls=_DialogClass)
    viewer.focus_force()
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
