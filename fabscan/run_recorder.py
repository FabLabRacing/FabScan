from __future__ import annotations

import csv
import json
import os
import platform
import queue
import re
import subprocess
import sys
import threading
import time
import traceback
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover - FabScan already depends on OpenCV
    cv2 = None


FORMAT_VERSION = 1
INTEGRATION_VERSION = "0.6.0-dev-m1"


def _utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _safe_slug(value: object, *, fallback: str = "run") -> str:
    text = str(value or "").strip().lower()
    text = re.sub(r"[^a-z0-9._-]+", "_", text)
    text = text.strip("._-")
    return text[:48] or fallback


def _json_safe(value: Any, *, max_depth: int = 8) -> Any:
    """Convert FabScan/Tk/numpy values into deterministic JSON-safe values."""

    if max_depth <= 0:
        return repr(value)
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, bytes):
        return {"type": "bytes", "length": len(value)}
    if is_dataclass(value):
        try:
            return _json_safe(asdict(value), max_depth=max_depth - 1)
        except Exception:
            return repr(value)
    if isinstance(value, dict):
        return {
            str(key): _json_safe(item, max_depth=max_depth - 1)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple, set)):
        return [_json_safe(item, max_depth=max_depth - 1) for item in value]

    # numpy scalar/array support without importing numpy directly.
    if hasattr(value, "item") and callable(getattr(value, "item")):
        try:
            return _json_safe(value.item(), max_depth=max_depth - 1)
        except Exception:
            pass
    if hasattr(value, "tolist") and callable(getattr(value, "tolist")):
        try:
            converted = value.tolist()
            # Avoid accidentally serializing a full camera frame.
            if isinstance(converted, list) and len(converted) > 256:
                return {
                    "type": type(value).__name__,
                    "shape": list(getattr(value, "shape", ())),
                    "dtype": str(getattr(value, "dtype", "")),
                }
            return _json_safe(converted, max_depth=max_depth - 1)
        except Exception:
            pass
    return repr(value)


def _read_git_metadata(project_root: Path) -> dict[str, Any]:
    result: dict[str, Any] = {}
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=project_root,
            check=True,
            capture_output=True,
            text=True,
            timeout=1.0,
        ).stdout.strip()
        result["commit"] = commit
        status = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=project_root,
            check=True,
            capture_output=True,
            text=True,
            timeout=1.0,
        ).stdout.splitlines()
        result["dirty"] = bool(status)
        result["changed_paths"] = status[:100]
    except Exception as exc:
        result["available"] = False
        result["error"] = str(exc)
    return result


class RunRecorder:
    """Asynchronous, loss-aware recorder for FabScan Follow decisions.

    The live/UI thread copies the exact frame and enqueues a compact event. A
    worker thread writes PNG and JSON/CSV files. Queue overflow is never hidden:
    the recording is marked incomplete and the operator is warned through the
    integration status path.
    """

    TIMELINE_FIELDS = (
        "timestamp_iso",
        "monotonic_s",
        "event_index",
        "event",
        "step_id",
        "step_label",
        "result",
        "reason",
        "frame_sequence",
        "frame_timestamp_s",
        "frame_file",
        "payload_json",
    )

    def __init__(self, project_root: Path, *, queue_limit: int = 16) -> None:
        self.project_root = Path(project_root).resolve()
        self.replays_root = self.project_root / "replays"
        self.queue_limit = max(8, int(queue_limit))
        self._queue: Optional[queue.Queue[dict[str, Any]]] = None
        self._thread: Optional[threading.Thread] = None
        self._state_lock = threading.Lock()
        self._active = False
        self._finishing = False
        self._run_dir: Optional[Path] = None
        self._manifest: dict[str, Any] = {}
        self._error = ""
        self._event_counter = 0
        self._enqueued_frames = 0
        self._enqueued_events = 0
        self._last_finished_dir: Optional[Path] = None

    @property
    def active(self) -> bool:
        with self._state_lock:
            return bool(self._active)

    @property
    def error(self) -> str:
        with self._state_lock:
            return self._error

    @property
    def run_dir(self) -> Optional[Path]:
        with self._state_lock:
            return self._run_dir

    @property
    def last_finished_dir(self) -> Optional[Path]:
        with self._state_lock:
            return self._last_finished_dir

    @property
    def backlog(self) -> int:
        work_queue = self._queue
        return int(work_queue.qsize()) if work_queue is not None else 0

    def start_run(
        self,
        *,
        label: str,
        requested_steps: int,
        settings: dict[str, Any],
        calibration: Any,
        metadata: dict[str, Any],
    ) -> Path:
        self.finish_run(status="superseded", reason="new recording started", wait=True)

        local_now = datetime.now().astimezone()
        stamp = local_now.strftime("%Y-%m-%d_%H%M%S")
        run_dir = self.replays_root / f"{stamp}_{_safe_slug(label, fallback='follow')}"
        suffix = 2
        while run_dir.exists():
            run_dir = self.replays_root / f"{stamp}_{_safe_slug(label, fallback='follow')}_{suffix}"
            suffix += 1
        (run_dir / "frames").mkdir(parents=True, exist_ok=False)

        manifest = {
            "format_version": FORMAT_VERSION,
            "integration_version": INTEGRATION_VERSION,
            "run_id": run_dir.name,
            "label": str(label),
            "requested_steps": int(requested_steps),
            "start_time_local": local_now.isoformat(timespec="milliseconds"),
            "start_time_utc": _utc_iso(),
            "project_root": str(self.project_root),
            "python": sys.version,
            "platform": platform.platform(),
            "pid": os.getpid(),
            "git": _read_git_metadata(self.project_root),
            "metadata": _json_safe(metadata),
            "complete": False,
            "status": "recording",
        }
        self._write_json_atomic(run_dir / "manifest.json", manifest)
        self._write_json_atomic(run_dir / "settings.json", _json_safe(settings))
        self._write_json_atomic(run_dir / "calibration.json", _json_safe(calibration))

        work_queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=self.queue_limit)
        worker = threading.Thread(
            target=self._writer_main,
            args=(run_dir, work_queue, manifest),
            name=f"FabScanRunRecorder-{run_dir.name}",
            daemon=True,
        )
        with self._state_lock:
            self._queue = work_queue
            self._thread = worker
            self._active = True
            self._finishing = False
            self._run_dir = run_dir
            self._manifest = manifest
            self._error = ""
            self._event_counter = 0
            self._enqueued_frames = 0
            self._enqueued_events = 0
        worker.start()
        return run_dir

    def record_event(
        self,
        event: str,
        *,
        payload: Optional[dict[str, Any]] = None,
        frame: Any = None,
    ) -> bool:
        with self._state_lock:
            if not self._active or self._finishing or self._queue is None:
                return False
            self._event_counter += 1
            event_index = self._event_counter
            work_queue = self._queue

        safe_payload = _json_safe(payload or {})
        step_id = safe_payload.get("step_id", "") if isinstance(safe_payload, dict) else ""
        item: dict[str, Any] = {
            "kind": "event",
            "event_index": event_index,
            "timestamp_iso": _utc_iso(),
            "monotonic_s": time.monotonic(),
            "event": str(event),
            "payload": safe_payload,
            "step_id": step_id,
            "frame": frame,
        }
        try:
            work_queue.put(item, timeout=0.20)
        except queue.Full:
            self._mark_error(
                f"recorder writer queue reached {self.queue_limit} items; recording marked incomplete"
            )
            return False

        with self._state_lock:
            self._enqueued_events += 1
            if frame is not None:
                self._enqueued_frames += 1
        return True

    def finish_run(
        self,
        *,
        status: str = "complete",
        reason: str = "",
        wait: bool = False,
    ) -> Optional[Path]:
        with self._state_lock:
            if not self._active or self._queue is None:
                return self._last_finished_dir
            if self._finishing:
                worker = self._thread
                run_dir = self._run_dir
            else:
                self._finishing = True
                worker = self._thread
                run_dir = self._run_dir
                work_queue = self._queue
                finish_item = {
                    "kind": "finish",
                    "timestamp_iso": _utc_iso(),
                    "status": str(status),
                    "reason": str(reason),
                    "error": self._error,
                    "enqueued_events": self._enqueued_events,
                    "enqueued_frames": self._enqueued_frames,
                }
                try:
                    work_queue.put(finish_item, timeout=2.0)
                except queue.Full:
                    self._error = (
                        self._error
                        or "recorder queue remained full while finishing; recording may be incomplete"
                    )
        if wait and worker is not None:
            worker.join(timeout=15.0)
        return run_dir

    def close(self) -> None:
        self.finish_run(status="closed", reason="dialog closed", wait=True)

    def _mark_error(self, message: str) -> None:
        with self._state_lock:
            if not self._error:
                self._error = str(message)

    @staticmethod
    def _write_json_atomic(path: Path, value: Any) -> None:
        temp = path.with_suffix(path.suffix + ".tmp")
        temp.write_text(json.dumps(value, indent=2, sort_keys=True), encoding="utf-8")
        temp.replace(path)

    def _writer_main(
        self,
        run_dir: Path,
        work_queue: queue.Queue[dict[str, Any]],
        manifest: dict[str, Any],
    ) -> None:
        events_path = run_dir / "events.jsonl"
        timeline_path = run_dir / "timeline.csv"
        steps_path = run_dir / "steps.jsonl"
        status_path = run_dir / "writer_status.json"
        saved_frames: dict[tuple[str, str], str] = {}
        step_states: dict[str, dict[str, Any]] = {}
        written_events = 0
        written_frames = 0
        finish_item: dict[str, Any] = {}

        try:
            with events_path.open("w", encoding="utf-8", buffering=1) as events_file, timeline_path.open(
                "w", encoding="utf-8", newline="", buffering=1
            ) as timeline_file:
                timeline = csv.DictWriter(timeline_file, fieldnames=self.TIMELINE_FIELDS)
                timeline.writeheader()

                while True:
                    item = work_queue.get()
                    try:
                        if item.get("kind") == "finish":
                            finish_item = item
                            break
                        if item.get("kind") != "event":
                            continue

                        payload = item.get("payload")
                        if not isinstance(payload, dict):
                            payload = {"value": payload}
                        event = str(item.get("event", ""))
                        step_key = self._step_key(item.get("step_id"))
                        frame_file = ""
                        frame = item.get("frame")
                        if frame is not None:
                            frame_sequence = payload.get("frame_sequence", "")
                            frame_token = str(frame_sequence)
                            if not frame_token:
                                frame_token = f"event:{int(item.get('event_index', 0))}"
                            frame_key = (step_key, frame_token)
                            if frame_key in saved_frames:
                                frame_file = saved_frames[frame_key]
                            else:
                                frame_file = self._write_frame(
                                    run_dir,
                                    frame,
                                    step_key=step_key,
                                    frame_sequence=frame_sequence,
                                    event=event,
                                    event_index=int(item.get("event_index", 0)),
                                )
                                if frame_file:
                                    saved_frames[frame_key] = frame_file
                                    written_frames += 1

                        record = {
                            "timestamp_iso": item.get("timestamp_iso", ""),
                            "monotonic_s": item.get("monotonic_s", ""),
                            "event_index": item.get("event_index", ""),
                            "event": event,
                            "step_id": payload.get("step_id", item.get("step_id", "")),
                            "step_label": payload.get("step_label", ""),
                            "frame_file": frame_file,
                            "payload": payload,
                        }
                        events_file.write(json.dumps(record, sort_keys=True) + "\n")
                        timeline.writerow(
                            {
                                "timestamp_iso": record["timestamp_iso"],
                                "monotonic_s": record["monotonic_s"],
                                "event_index": record["event_index"],
                                "event": event,
                                "step_id": record["step_id"],
                                "step_label": record["step_label"],
                                "result": payload.get("result", ""),
                                "reason": payload.get("reason", ""),
                                "frame_sequence": payload.get("frame_sequence", ""),
                                "frame_timestamp_s": payload.get("frame_timestamp_s", ""),
                                "frame_file": frame_file,
                                "payload_json": json.dumps(payload, sort_keys=True),
                            }
                        )
                        written_events += 1
                        if step_key:
                            self._update_step_state(step_states, step_key, record)
                    finally:
                        work_queue.task_done()

            with steps_path.open("w", encoding="utf-8") as steps_file:
                for key in sorted(step_states, key=self._step_sort_key):
                    steps_file.write(json.dumps(step_states[key], sort_keys=True) + "\n")

            final_error = str(finish_item.get("error") or self.error or "")
            complete = not bool(final_error)
            final_manifest = dict(manifest)
            final_manifest.update(
                {
                    "end_time_utc": finish_item.get("timestamp_iso", _utc_iso()),
                    "status": finish_item.get("status", "complete" if complete else "incomplete"),
                    "finish_reason": finish_item.get("reason", ""),
                    "complete": complete,
                    "error": final_error,
                    "event_count": written_events,
                    "frame_count": written_frames,
                    "step_count": len(step_states),
                    "files": {
                        "events": "events.jsonl",
                        "steps": "steps.jsonl",
                        "timeline": "timeline.csv",
                        "settings": "settings.json",
                        "calibration": "calibration.json",
                        "frames": "frames/",
                    },
                }
            )
            self._write_json_atomic(run_dir / "manifest.json", final_manifest)
            self._write_json_atomic(
                status_path,
                {
                    "complete": complete,
                    "error": final_error,
                    "written_events": written_events,
                    "written_frames": written_frames,
                    "steps": len(step_states),
                    "finished_utc": _utc_iso(),
                },
            )
        except Exception:
            error = traceback.format_exc()
            self._mark_error(error)
            try:
                self._write_json_atomic(
                    status_path,
                    {
                        "complete": False,
                        "error": error,
                        "written_events": written_events,
                        "written_frames": written_frames,
                        "finished_utc": _utc_iso(),
                    },
                )
                failed_manifest = dict(manifest)
                failed_manifest.update(
                    {
                        "complete": False,
                        "status": "writer_failed",
                        "error": error,
                        "end_time_utc": _utc_iso(),
                    }
                )
                self._write_json_atomic(run_dir / "manifest.json", failed_manifest)
            except Exception:
                pass
        finally:
            with self._state_lock:
                self._active = False
                self._finishing = False
                self._last_finished_dir = run_dir
                self._run_dir = None
                self._queue = None
                self._thread = None

    @staticmethod
    def _step_key(value: Any) -> str:
        if value in (None, ""):
            return ""
        try:
            return str(int(value))
        except (TypeError, ValueError):
            return str(value)

    @staticmethod
    def _step_sort_key(value: str) -> tuple[int, Any]:
        try:
            return (0, int(value))
        except (TypeError, ValueError):
            return (1, value)

    def _write_frame(
        self,
        run_dir: Path,
        frame: Any,
        *,
        step_key: str,
        frame_sequence: Any,
        event: str,
        event_index: int,
    ) -> str:
        if cv2 is None:
            self._mark_error("OpenCV unavailable in recorder writer; frames were not saved")
            return ""
        try:
            step_part = f"step_{int(step_key):06d}" if step_key else "run"
        except (TypeError, ValueError):
            step_part = f"step_{_safe_slug(step_key, fallback='unknown')}"
        try:
            seq_part = f"seq_{int(frame_sequence):09d}"
        except (TypeError, ValueError):
            seq_part = f"event_{event_index:09d}"
        event_part = _safe_slug(event, fallback="frame")
        relative = Path("frames") / f"{step_part}_{seq_part}_{event_part}.png"
        absolute = run_dir / relative
        ok = cv2.imwrite(
            str(absolute),
            frame,
            [int(cv2.IMWRITE_PNG_COMPRESSION), 1],
        )
        if not ok:
            self._mark_error(f"OpenCV failed to write {absolute}")
            return ""
        return relative.as_posix()

    @staticmethod
    def _update_step_state(
        step_states: dict[str, dict[str, Any]],
        step_key: str,
        record: dict[str, Any],
    ) -> None:
        payload = record.get("payload")
        if not isinstance(payload, dict):
            payload = {}
        state = step_states.setdefault(
            step_key,
            {
                "step_id": payload.get("step_id", step_key),
                "step_label": payload.get("step_label", ""),
                "first_event_index": record.get("event_index", ""),
                "last_event_index": record.get("event_index", ""),
                "first_timestamp_iso": record.get("timestamp_iso", ""),
                "last_timestamp_iso": record.get("timestamp_iso", ""),
                "event_count": 0,
                "event_names": [],
                "frames": [],
                "decision_frame": "",
                "position_before": {},
                "delayed_position": {},
                "planner_output": {},
                "position_after": {},
                "terminal_event": "",
                "terminal_result": "",
                "terminal_reason": "",
            },
        )
        state["last_event_index"] = record.get("event_index", "")
        state["last_timestamp_iso"] = record.get("timestamp_iso", "")
        state["event_count"] = int(state.get("event_count", 0)) + 1
        event = str(record.get("event", ""))
        if event and event not in state["event_names"]:
            state["event_names"].append(event)
        frame_file = str(record.get("frame_file", "") or "")
        if frame_file and frame_file not in state["frames"]:
            state["frames"].append(frame_file)

        if not state["position_before"]:
            x = payload.get("current_x", payload.get("start_x", ""))
            y = payload.get("current_y", payload.get("start_y", ""))
            z = payload.get("current_z", payload.get("start_z", ""))
            if x not in (None, "") and y not in (None, ""):
                state["position_before"] = {"x": x, "y": y, "z": z}
        delayed_x = payload.get("delayed_x", "")
        delayed_y = payload.get("delayed_y", "")
        if delayed_x not in (None, "") and delayed_y not in (None, ""):
            state["delayed_position"] = {
                "x": delayed_x,
                "y": delayed_y,
                "source": payload.get("delayed_position_source", ""),
                "age_ms": payload.get("delayed_position_age_ms", ""),
            }

        event_upper = event.upper()
        if event_upper == "MOVE_PLAN":
            state["planner_output"] = dict(payload)
            state["decision_frame"] = frame_file or (
                state["frames"][-1] if state["frames"] else ""
            )
            state["terminal_event"] = event
            state["terminal_result"] = payload.get("result", "")
            state["terminal_reason"] = payload.get("reason", "")
        elif event_upper in {"STEP_REFUSED", "STEP_FAILED", "STEP_STOPPED"}:
            state["decision_frame"] = frame_file or (
                state["frames"][-1] if state["frames"] else ""
            )
            state["terminal_event"] = event
            state["terminal_result"] = payload.get("result", "")
            state["terminal_reason"] = payload.get("reason", "")

        # Capture the latest known/commanded final XY. The full event stream is
        # retained, so replay can later distinguish command target from feedback.
        after_x = payload.get("post_x", payload.get("target_x", ""))
        after_y = payload.get("post_y", payload.get("target_y", ""))
        if after_x not in (None, "") and after_y not in (None, ""):
            state["position_after"] = {
                "x": after_x,
                "y": after_y,
                "source_event": event,
            }


def _project_root_from_module(dialog_cls: type) -> Path:
    module = sys.modules.get(dialog_cls.__module__)
    module_file = Path(getattr(module, "__file__", Path.cwd())).resolve()
    # camera_calibration.py lives in <project>/fabscan/
    return module_file.parent.parent


def _snapshot_dialog_settings(dialog: Any) -> dict[str, Any]:
    snapshot: dict[str, Any] = {}
    for name, value in vars(dialog).items():
        if not name.endswith("_var"):
            continue
        getter = getattr(value, "get", None)
        if not callable(getter):
            continue
        try:
            snapshot[name] = _json_safe(getter())
        except Exception as exc:
            snapshot[name] = {"error": str(exc)}

    for getter_name in (
        "_get_follow_step",
        "_get_follow_max_correct",
        "_get_follow_min_confidence",
        "_get_follow_position_delay_ms",
        "_get_camera_stream_max_fps",
        "_get_camera_preview_max_fps",
        "_get_line_search_px",
        "_get_follow_repeat_count",
        "_get_follow_max_heading_change",
    ):
        getter = getattr(dialog, getter_name, None)
        if callable(getter):
            try:
                snapshot[getter_name.removeprefix("_get_")] = _json_safe(getter())
            except Exception as exc:
                snapshot[getter_name.removeprefix("_get_")] = {"error": str(exc)}
    return snapshot


def _dialog_metadata(dialog: Any, *, label: str, requested_steps: int) -> dict[str, Any]:
    frame = getattr(dialog, "current_frame_bgr", None)
    frame_shape = list(getattr(frame, "shape", ())) if frame is not None else []
    title = ""
    try:
        title = str(dialog.title())
    except Exception:
        pass
    return {
        "dialog_class": f"{type(dialog).__module__}.{type(dialog).__name__}",
        "dialog_title": title,
        "run_label": label,
        "requested_steps": requested_steps,
        "frame_shape_at_start": frame_shape,
        "camera_index": getattr(dialog, "camera_index", ""),
        "requested_width": getattr(dialog, "requested_width", ""),
        "requested_height": getattr(dialog, "requested_height", ""),
        "initial_frame_sequence": getattr(dialog, "current_frame_sequence", ""),
        "initial_frame_timestamp_s": getattr(dialog, "current_frame_timestamp", ""),
    }


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


def _add_recorder_ui(dialog: Any) -> None:
    try:
        import tkinter as tk
        from tkinter import ttk

        dialog.follow_record_run_var = tk.BooleanVar(value=False)
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
        check = ttk.Checkbutton(
            parent,
            text="Record Follow Run",
            variable=dialog.follow_record_run_var,
        )
        check.grid(row=max_row + 1, column=0, columnspan=2, sticky=tk.W, pady=(7, 0))
        dialog._record_follow_checkbox = check
    except Exception:
        traceback.print_exc()


def _recording_enabled(dialog: Any) -> bool:
    variable = getattr(dialog, "follow_record_run_var", None)
    getter = getattr(variable, "get", None)
    if not callable(getter):
        return False
    try:
        return bool(getter())
    except Exception:
        return False


def _set_dialog_status(dialog: Any, message: str) -> None:
    variable = getattr(dialog, "cal_status_var", None)
    setter = getattr(variable, "set", None)
    if callable(setter):
        try:
            setter(message)
        except Exception:
            pass


def _finish_dialog_recording(dialog: Any, *, status: str, reason: str) -> None:
    recorder: Optional[RunRecorder] = getattr(dialog, "_run_recorder", None)
    if recorder is None or not recorder.active:
        return
    path = recorder.finish_run(status=status, reason=reason, wait=False)
    if path is not None:
        _set_dialog_status(dialog, f"Follow recording queued for write: {path}")


def install_recording_support(dialog_cls: type) -> None:
    """Attach Milestone-1 recording to CameraCalibrationDialog.

    Integration is intentionally method-wrapped rather than planner-edited. The
    old detector/planner/controller code remains the motion authority.
    """

    if getattr(dialog_cls, "_fabscan_run_recorder_installed", False):
        return
    required = ("__init__", "_begin_follow_run", "_timeline_log")
    missing = [name for name in required if not hasattr(dialog_cls, name)]
    if missing:
        raise RuntimeError(f"FabScan recorder cannot attach; missing methods: {', '.join(missing)}")

    original_init = dialog_cls.__init__
    original_begin = dialog_cls._begin_follow_run
    original_timeline = dialog_cls._timeline_log
    original_single = getattr(dialog_cls, "follow_line_single_step", None)
    original_multiple = getattr(dialog_cls, "follow_line_multiple_steps", None)
    original_close = getattr(dialog_cls, "close", None)

    def wrapped_init(self: Any, *args: Any, **kwargs: Any) -> None:
        original_init(self, *args, **kwargs)
        project_root = _project_root_from_module(dialog_cls)
        self._run_recorder = RunRecorder(project_root)
        self._recording_public_call_depth = 0
        self._recording_last_path = None
        _add_recorder_ui(self)
        try:
            current_title = str(self.title())
            if "Recorder M1" not in current_title:
                self.title(f"{current_title} + Recorder M1")
        except Exception:
            pass

    def wrapped_begin(
        self: Any,
        label: str,
        *,
        requested_steps: int,
        reset_latch: bool,
    ) -> Any:
        recorder: Optional[RunRecorder] = getattr(self, "_run_recorder", None)
        if recorder is not None and _recording_enabled(self):
            try:
                path = recorder.start_run(
                    label=label,
                    requested_steps=requested_steps,
                    settings=_snapshot_dialog_settings(self),
                    calibration=getattr(self, "active_calibration", None),
                    metadata=_dialog_metadata(
                        self,
                        label=label,
                        requested_steps=requested_steps,
                    ),
                )
                self._recording_last_path = path
                _set_dialog_status(self, f"Recording Follow run: {path}")
            except Exception as exc:
                _set_dialog_status(self, f"Follow recorder could not start: {exc}")
                traceback.print_exc()
        return original_begin(
            self,
            label,
            requested_steps=requested_steps,
            reset_latch=reset_latch,
        )

    def wrapped_timeline(self: Any, event: str, *args: Any, **kwargs: Any) -> Any:
        original_error: Optional[BaseException] = None
        result: Any = None
        try:
            result = original_timeline(self, event, *args, **kwargs)
        except BaseException as exc:
            original_error = exc

        recorder: Optional[RunRecorder] = getattr(self, "_run_recorder", None)
        if recorder is not None and recorder.active:
            payload = dict(kwargs)
            payload.setdefault("frame_sequence", getattr(self, "current_frame_sequence", ""))
            payload.setdefault("frame_timestamp_s", getattr(self, "current_frame_timestamp", ""))
            payload.setdefault("active_follow_run_id", getattr(self, "_active_follow_run_id", ""))
            payload.setdefault("active_follow_run_step", getattr(self, "_active_follow_run_step", ""))
            if original_error is not None:
                payload["timeline_original_error"] = repr(original_error)
            frame = None
            event_upper = str(event).upper()
            if event_upper.startswith("DETECTION") or event_upper in {
                "FRAME_SELECTED",
                "FRAME_RETRY",
            }:
                current = getattr(self, "current_frame_bgr", None)
                copier = getattr(current, "copy", None)
                if callable(copier):
                    try:
                        frame = copier()
                    except Exception:
                        frame = None
            queued = recorder.record_event(str(event), payload=payload, frame=frame)
            if not queued and recorder.error:
                _set_dialog_status(self, f"Follow recorder stopped: {recorder.error}")

        if original_error is not None:
            raise original_error
        return result

    def wrap_public(original: Any, method_name: str) -> Any:
        if original is None:
            return None

        def wrapped(self: Any, *args: Any, **kwargs: Any) -> Any:
            self._recording_public_call_depth = int(
                getattr(self, "_recording_public_call_depth", 0)
            ) + 1
            status = "complete"
            reason = f"{method_name} returned"
            try:
                return original(self, *args, **kwargs)
            except BaseException as exc:
                status = "exception"
                reason = repr(exc)
                raise
            finally:
                self._recording_public_call_depth = max(
                    0,
                    int(getattr(self, "_recording_public_call_depth", 1)) - 1,
                )
                if self._recording_public_call_depth == 0:
                    _finish_dialog_recording(self, status=status, reason=reason)

        wrapped.__name__ = getattr(original, "__name__", method_name)
        wrapped.__doc__ = getattr(original, "__doc__", None)
        return wrapped

    def wrapped_close(self: Any, *args: Any, **kwargs: Any) -> Any:
        recorder: Optional[RunRecorder] = getattr(self, "_run_recorder", None)
        if recorder is not None:
            recorder.close()
        if original_close is not None:
            return original_close(self, *args, **kwargs)
        return None

    dialog_cls.__init__ = wrapped_init
    dialog_cls._begin_follow_run = wrapped_begin
    dialog_cls._timeline_log = wrapped_timeline
    if original_single is not None:
        dialog_cls.follow_line_single_step = wrap_public(
            original_single, "follow_line_single_step"
        )
    if original_multiple is not None:
        dialog_cls.follow_line_multiple_steps = wrap_public(
            original_multiple, "follow_line_multiple_steps"
        )
    dialog_cls.close = wrapped_close
    dialog_cls._fabscan_run_recorder_installed = True
