from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import glob
import importlib
import sys
from typing import Optional, Sequence, Tuple


Position3 = Tuple[float, float, float]


@dataclass
class LinuxCNCPositionStatus:
    """Small, UI-friendly snapshot of LinuxCNC status.

    FabScan uses this read-only. No motion commands live in this module.
    """

    available: bool
    connected: bool
    error: Optional[str]
    task_state: str = "Unknown"
    interp_state: str = "Unknown"
    homed_text: str = "Unknown"
    all_xyz_homed: bool = False
    machine_position: Position3 = (0.0, 0.0, 0.0)
    work_position: Position3 = (0.0, 0.0, 0.0)


class LinuxCNCStatusReader:
    """Read LinuxCNC status safely and optionally from inside a venv.

    The LinuxCNC Python module is commonly installed in the system Python
    dist-packages path. If FabScan is running from a virtual environment, this
    reader tries the normal import first, then tries common system paths.
    """

    def __init__(self) -> None:
        self._linuxcnc = None
        self._stat = None
        self._import_error: Optional[str] = None
        self._load_linuxcnc_module()

    @property
    def module_available(self) -> bool:
        return self._linuxcnc is not None

    def _load_linuxcnc_module(self) -> None:
        try:
            self._linuxcnc = importlib.import_module("linuxcnc")
            return
        except Exception as exc:  # noqa: BLE001
            self._import_error = str(exc)

        candidate_paths: list[str] = []
        candidate_paths.extend(glob.glob("/usr/lib/python3/dist-packages"))
        candidate_paths.extend(glob.glob("/usr/lib/python3*/dist-packages"))
        candidate_paths.extend(glob.glob("/usr/local/lib/python3*/dist-packages"))

        for path_text in candidate_paths:
            path = Path(path_text)
            if not path.exists():
                continue
            if str(path) not in sys.path:
                sys.path.append(str(path))
            try:
                self._linuxcnc = importlib.import_module("linuxcnc")
                self._import_error = None
                return
            except Exception as exc:  # noqa: BLE001
                self._import_error = str(exc)

    def read_status(self) -> LinuxCNCPositionStatus:
        if self._linuxcnc is None:
            return LinuxCNCPositionStatus(
                available=False,
                connected=False,
                error=(
                    "LinuxCNC Python module is not available. "
                    f"Import error: {self._import_error or 'unknown'}"
                ),
            )

        try:
            if self._stat is None:
                self._stat = self._linuxcnc.stat()
            self._stat.poll()
        except Exception as exc:  # noqa: BLE001
            return LinuxCNCPositionStatus(
                available=True,
                connected=False,
                error=f"Could not poll LinuxCNC status: {exc}",
            )

        try:
            machine_position = self._position3(getattr(self._stat, "actual_position", (0.0, 0.0, 0.0)))
            work_position = self._calculate_work_position(machine_position)
            task_state = self._task_state_text(getattr(self._stat, "task_state", None))
            interp_state = self._interp_state_text(getattr(self._stat, "interp_state", None))
            homed_values = list(getattr(self._stat, "homed", []))
            homed_text, all_xyz_homed = self._homed_text(homed_values)
        except Exception as exc:  # noqa: BLE001
            return LinuxCNCPositionStatus(
                available=True,
                connected=False,
                error=f"LinuxCNC status data could not be interpreted: {exc}",
            )

        return LinuxCNCPositionStatus(
            available=True,
            connected=True,
            error=None,
            task_state=task_state,
            interp_state=interp_state,
            homed_text=homed_text,
            all_xyz_homed=all_xyz_homed,
            machine_position=machine_position,
            work_position=work_position,
        )

    def _calculate_work_position(self, machine_position: Position3) -> Position3:
        """Calculate a practical work-coordinate display from LinuxCNC status.

        LinuxCNC reports machine position plus active offsets. For this first
        read-only FabScan trace workflow, subtracting G5x, G92, and tool offsets
        gives the normal displayed work position for basic X/Y capture.
        """

        g5x = self._position3(getattr(self._stat, "g5x_offset", (0.0, 0.0, 0.0)))
        g92 = self._position3(getattr(self._stat, "g92_offset", (0.0, 0.0, 0.0)))
        tool = self._position3(getattr(self._stat, "tool_offset", (0.0, 0.0, 0.0)))
        return (
            machine_position[0] - g5x[0] - g92[0] - tool[0],
            machine_position[1] - g5x[1] - g92[1] - tool[1],
            machine_position[2] - g5x[2] - g92[2] - tool[2],
        )

    @staticmethod
    def _position3(values: Sequence[float]) -> Position3:
        padded = list(values) + [0.0, 0.0, 0.0]
        return (float(padded[0]), float(padded[1]), float(padded[2]))

    def _task_state_text(self, value: object) -> str:
        linuxcnc = self._linuxcnc
        mapping = {
            getattr(linuxcnc, "STATE_ESTOP", object()): "ESTOP",
            getattr(linuxcnc, "STATE_ESTOP_RESET", object()): "ESTOP RESET",
            getattr(linuxcnc, "STATE_OFF", object()): "OFF",
            getattr(linuxcnc, "STATE_ON", object()): "ON",
        }
        return mapping.get(value, str(value))

    def _interp_state_text(self, value: object) -> str:
        linuxcnc = self._linuxcnc
        mapping = {
            getattr(linuxcnc, "INTERP_IDLE", object()): "IDLE",
            getattr(linuxcnc, "INTERP_READING", object()): "READING",
            getattr(linuxcnc, "INTERP_PAUSED", object()): "PAUSED",
            getattr(linuxcnc, "INTERP_WAITING", object()): "WAITING",
        }
        return mapping.get(value, str(value))

    @staticmethod
    def _homed_text(homed_values: list[object]) -> tuple[str, bool]:
        if not homed_values:
            return "Unknown", False

        labels = ("X", "Y", "Z")
        parts: list[str] = []
        all_xyz_homed = True
        for index, label in enumerate(labels):
            if index >= len(homed_values):
                parts.append(f"{label}:?")
                all_xyz_homed = False
                continue
            homed = bool(homed_values[index])
            parts.append(f"{label}:{'Y' if homed else 'N'}")
            all_xyz_homed = all_xyz_homed and homed

        return " ".join(parts), all_xyz_homed
