from __future__ import annotations

from dataclasses import dataclass
import sys
import threading
import time
from typing import Optional

import cv2
import numpy as np


DEFAULT_CAMERA_WIDTH = 800
DEFAULT_CAMERA_HEIGHT = 600

# Put the known-good microscope-camera size first. Keep 1280x720 last because
# many UVC microscope cameras do not expose that mode even though it is common
# on webcams.
CAMERA_RESOLUTION_PRESETS: tuple[tuple[int, int], ...] = (
    (800, 600),
    (640, 480),
    (1280, 960),
    (1600, 1200),
    (1280, 720),
)


def preset_labels() -> tuple[str, ...]:
    return tuple(f"{width} x {height}" for width, height in CAMERA_RESOLUTION_PRESETS)


def size_to_preset_label(width: int, height: int) -> str:
    return f"{int(width)} x {int(height)}"


def parse_preset_label(label: str) -> Optional[tuple[int, int]]:
    try:
        left, right = str(label).lower().replace("×", "x").split("x", 1)
        width = int(left.strip())
        height = int(right.strip())
    except (AttributeError, TypeError, ValueError):
        return None
    if width <= 0 or height <= 0:
        return None
    return width, height


def _safe_capture_backend() -> tuple[int, str]:
    """Use V4L2 explicitly on Linux, otherwise let OpenCV pick."""

    if sys.platform.startswith("linux") and hasattr(cv2, "CAP_V4L2"):
        return int(cv2.CAP_V4L2), "V4L2"
    return int(cv2.CAP_ANY), "default"


def _fourcc_text(value: float) -> str:
    try:
        code = int(value)
    except (TypeError, ValueError):
        return "unknown"
    chars = []
    for shift in (0, 8, 16, 24):
        char = chr((code >> shift) & 0xFF)
        chars.append(char if char.isprintable() and char != "\x00" else " ")
    text = "".join(chars).strip()
    return text or "unknown"


@dataclass(frozen=True)
class CameraOpenInfo:
    camera_index: int
    requested_width: int
    requested_height: int
    actual_width: int
    actual_height: int
    backend_name: str
    fourcc: str
    warning: str = ""

    @property
    def actual_size_text(self) -> str:
        if self.actual_width > 0 and self.actual_height > 0:
            return f"{self.actual_width} x {self.actual_height}"
        return "unknown"

    @property
    def requested_size_text(self) -> str:
        if self.requested_width > 0 and self.requested_height > 0:
            return f"{self.requested_width} x {self.requested_height}"
        return "camera default"


def open_camera_capture(camera_index: int, requested_width: int, requested_height: int) -> tuple[Optional[cv2.VideoCapture], CameraOpenInfo, str]:
    """Open a camera with conservative Linux/OpenCV settings.

    The returned capture is intentionally not read on the Tkinter thread. Use
    CameraStream below so slow or failing UVC reads cannot freeze the UI.
    """

    index = max(0, int(camera_index))
    width = max(0, int(requested_width))
    height = max(0, int(requested_height))
    backend, backend_name = _safe_capture_backend()

    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened() and backend != int(cv2.CAP_ANY):
        # Some non-Linux or unusual OpenCV builds define CAP_V4L2 but cannot
        # actually use it. Fall back once before declaring the camera missing.
        cap.release()
        cap = cv2.VideoCapture(index)
        backend_name = "default"

    if not cap.isOpened():
        info = CameraOpenInfo(index, width, height, 0, 0, backend_name, "unknown")
        return None, info, "Camera did not open. Try another index, check permissions, or reconnect USB."

    # UVC microscope cameras are usually happier with MJPG at larger sizes.
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    except Exception:
        pass
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    if width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    actual_w = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
    actual_h = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    fourcc = _fourcc_text(cap.get(cv2.CAP_PROP_FOURCC))

    warning = ""
    if width > 0 and height > 0 and actual_w > 0 and actual_h > 0:
        if abs(actual_w - width) > 2 or abs(actual_h - height) > 2:
            warning = (
                f"Requested {width} x {height}, but camera returned {actual_w} x {actual_h}. "
                "Use a supported preset if preview is slow or unreliable."
            )

    info = CameraOpenInfo(index, width, height, actual_w, actual_h, backend_name, fourcc, warning)
    return cap, info, ""


class CameraStream:
    """Background frame reader for Tkinter camera previews.

    OpenCV/V4L2 reads can pause for a long time when a USB camera is asked for
    an unsupported mode or when /dev/video points at a metadata node. Reading in
    a daemon thread keeps Tkinter responsive and lets the UI show a useful
    status instead of looking frozen.
    """

    def __init__(self, cap: cv2.VideoCapture, info: CameraOpenInfo) -> None:
        self.cap = cap
        self.info = info
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_time = 0.0
        self._read_count = 0
        self._fail_count = 0
        self._last_error = "Waiting for first camera frame..."

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._reader_loop, name="FabScanCameraReader", daemon=True)
        self._thread.start()

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                ok, frame = self.cap.read()
            except Exception as exc:
                ok = False
                frame = None
                error = f"Camera read error: {exc}"
            else:
                error = "Camera read failed. Try Open / Restart Camera."

            now = time.monotonic()
            with self._lock:
                if ok and frame is not None:
                    self._latest_frame = frame
                    self._latest_time = now
                    self._read_count += 1
                    self._fail_count = 0
                    self._last_error = ""
                else:
                    self._fail_count += 1
                    self._last_error = error

            if not ok:
                time.sleep(0.05)

    def get_latest_frame(self, *, copy: bool = True) -> Optional[np.ndarray]:
        with self._lock:
            frame = self._latest_frame
            if frame is None:
                return None
            if copy:
                return frame.copy()
            return frame

    def status_message(self) -> str:
        with self._lock:
            read_count = self._read_count
            fail_count = self._fail_count
            last_error = self._last_error
            age = time.monotonic() - self._latest_time if self._latest_time > 0.0 else 0.0

        if read_count == 0:
            if fail_count > 20:
                return (
                    "No frames received. This may be a metadata /dev/video node, "
                    "an unsupported resolution, or a busy/disconnected camera."
                )
            # Keep the open/status line visible while the first frame is still
            # arriving. Only replace it after repeated read failures.
            return ""
        if fail_count > 10:
            return last_error or "Camera frames stopped. Try Open / Restart Camera."
        if age > 2.0:
            return "Camera frame is stale. Try Open / Restart Camera."
        return ""

    def close(self) -> None:
        self._stop_event.set()
        # Do not wait long: the important part is keeping the UI responsive if a
        # driver read is wedged. The daemon thread will exit once read returns.
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.25)
        try:
            self.cap.release()
        except Exception:
            pass
