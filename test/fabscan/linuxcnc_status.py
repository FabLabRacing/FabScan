from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import glob
import importlib
import math
import sys
from typing import Optional, Sequence, Tuple


Position3 = Tuple[float, float, float]


@dataclass
class LinuxCNCPositionStatus:
    """Small, UI-friendly snapshot of LinuxCNC status."""

    available: bool
    connected: bool
    error: Optional[str]
    task_state: str = "Unknown"
    interp_state: str = "Unknown"
    task_mode: str = "Unknown"
    homed_text: str = "Unknown"
    all_xyz_homed: bool = False
    machine_position: Position3 = (0.0, 0.0, 0.0)
    work_position: Position3 = (0.0, 0.0, 0.0)


@dataclass
class LinuxCNCJogResult:
    """Result of one FabScan-requested incremental jog."""

    success: bool
    message: str
    status: Optional[LinuxCNCPositionStatus] = None


@dataclass
class LinuxCNCMotionResult:
    """Result of one FabScan-requested controlled X/Y move."""

    success: bool
    message: str
    status: Optional[LinuxCNCPositionStatus] = None
    mdi_command: str = ""


class LinuxCNCStatusReader:
    """Read LinuxCNC status and send guarded incremental jogs.

    FabScan keeps this deliberately conservative:
    - status polling is always safe/read-only;
    - jog support is X/Y incremental only;
    - every jog checks LinuxCNC state before motion.

    The LinuxCNC Python module is commonly installed in the system Python
    dist-packages path. If FabScan is running from a virtual environment, this
    reader tries the normal import first, then tries common system paths.
    """

    AXIS_TO_INDEX = {"X": 0, "Y": 1}

    def __init__(self) -> None:
        self._linuxcnc = None
        self._stat = None
        self._command = None
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
            task_mode = self._task_mode_text(getattr(self._stat, "task_mode", None))
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
            task_mode=task_mode,
            homed_text=homed_text,
            all_xyz_homed=all_xyz_homed,
            machine_position=machine_position,
            work_position=work_position,
        )

    def incremental_jog(
        self,
        axis: str,
        direction: int,
        step_distance: float,
        feed_per_minute: float,
    ) -> LinuxCNCJogResult:
        """Send one guarded X/Y incremental jog command.

        Parameters are in the active LinuxCNC machine/user units. FabScan labels
        the feed field as units/minute, then converts to units/second before
        calling LinuxCNC's jog API.
        """

        axis = axis.upper().strip()
        if axis not in self.AXIS_TO_INDEX:
            return LinuxCNCJogResult(False, "FabScan v0.3.4 only supports X/Y jog buttons.")

        direction = 1 if direction >= 0 else -1
        step_distance = abs(float(step_distance))
        feed_per_minute = abs(float(feed_per_minute))

        if step_distance <= 0:
            return LinuxCNCJogResult(False, "Jog step must be greater than zero.")
        if step_distance > 1.0:
            return LinuxCNCJogResult(False, "Jog step is limited to 1.000 machine unit per click in FabScan.")
        if feed_per_minute <= 0:
            return LinuxCNCJogResult(False, "Jog feed must be greater than zero.")
        if feed_per_minute > 120.0:
            return LinuxCNCJogResult(False, "Jog feed is limited to 120 units/minute in FabScan.")

        status = self.read_status()
        ok, message = self._ok_for_incremental_jog(status)
        if not ok:
            return LinuxCNCJogResult(False, message, status)

        if self._linuxcnc is None:
            return LinuxCNCJogResult(False, "LinuxCNC Python module is not available.", status)

        try:
            if self._command is None:
                self._command = self._linuxcnc.command()

            # Axis jogging in Cartesian/world mode requires teleop enabled on
            # supported LinuxCNC versions. If unavailable, continue and let the
            # jog command report any error.
            try:
                self._command.teleop_enable(True)
                self._command.wait_complete(1.0)
            except AttributeError:
                pass

            axis_index = self.AXIS_TO_INDEX[axis]
            velocity_per_second = (feed_per_minute / 60.0) * direction
            self._command.jog(
                self._linuxcnc.JOG_INCREMENT,
                False,  # False = axis Cartesian coordinate jog, not joint jog
                axis_index,
                velocity_per_second,
                step_distance,
            )
        except Exception as exc:  # noqa: BLE001
            return LinuxCNCJogResult(False, f"LinuxCNC jog command failed: {exc}", status)

        return LinuxCNCJogResult(
            True,
            f"Jogged {axis}{'+' if direction > 0 else '-'} {step_distance:.4f} at {feed_per_minute:.1f} units/min.",
            self.read_status(),
        )

    def controlled_xy_move(
        self,
        target_x: float,
        target_y: float,
        feed_per_minute: float,
        coordinate_mode_label: str,
    ) -> LinuxCNCMotionResult:
        """Send one guarded X/Y point-to-point move through LinuxCNC MDI.

        FabScan v0.4.0 keeps controlled motion deliberately small:
        - X/Y only;
        - no Z;
        - no torch/plasma commands;
        - no program start;
        - one explicit MDI G1 move at a user-limited feed.
        """

        try:
            target_x = float(target_x)
            target_y = float(target_y)
            feed_per_minute = abs(float(feed_per_minute))
        except Exception:  # noqa: BLE001
            return LinuxCNCMotionResult(False, "Move target/feed values must be numeric.")

        if not math.isfinite(target_x) or not math.isfinite(target_y):
            return LinuxCNCMotionResult(False, "Move target X/Y must be finite numbers.")
        if feed_per_minute <= 0:
            return LinuxCNCMotionResult(False, "Move feed must be greater than zero.")
        if feed_per_minute > 120.0:
            return LinuxCNCMotionResult(False, "Controlled move feed is limited to 120 units/minute in FabScan.")

        status = self.read_status()
        ok, message = self._ok_for_controlled_xy_move(status)
        if not ok:
            return LinuxCNCMotionResult(False, message, status)

        if self._linuxcnc is None:
            return LinuxCNCMotionResult(False, "LinuxCNC Python module is not available.", status)

        coordinate_mode_label = (coordinate_mode_label or "Work coordinates").strip()
        if coordinate_mode_label == "Machine coordinates":
            mdi_command = f"G90 G53 G1 X{target_x:.6f} Y{target_y:.6f} F{feed_per_minute:.3f}"
        else:
            mdi_command = f"G90 G1 X{target_x:.6f} Y{target_y:.6f} F{feed_per_minute:.3f}"

        try:
            if self._command is None:
                self._command = self._linuxcnc.command()

            self._command.mode(self._linuxcnc.MODE_MDI)
            self._command.wait_complete(1.0)
            self._command.mdi(mdi_command)
        except Exception as exc:  # noqa: BLE001
            return LinuxCNCMotionResult(False, f"LinuxCNC controlled move failed: {exc}", status, mdi_command)

        return LinuxCNCMotionResult(
            True,
            f"Started controlled X/Y move to X{target_x:.4f} Y{target_y:.4f} at {feed_per_minute:.1f} units/min.",
            self.read_status(),
            mdi_command,
        )

    def abort_motion(self) -> LinuxCNCMotionResult:
        """Abort the currently running LinuxCNC command from FabScan.

        This is a software abort, not a replacement for the physical E-stop.
        """

        status = self.read_status()
        if not status.available:
            return LinuxCNCMotionResult(False, status.error or "LinuxCNC Python module is not available.", status)
        if not status.connected:
            return LinuxCNCMotionResult(False, status.error or "FabScan is not connected to LinuxCNC.", status)

        if self._linuxcnc is None:
            return LinuxCNCMotionResult(False, "LinuxCNC Python module is not available.", status)

        try:
            if self._command is None:
                self._command = self._linuxcnc.command()
            self._command.abort()
        except Exception as exc:  # noqa: BLE001
            return LinuxCNCMotionResult(False, f"LinuxCNC abort failed: {exc}", status)

        return LinuxCNCMotionResult(True, "FabScan sent LinuxCNC abort command.", self.read_status())

    def _ok_for_incremental_jog(self, status: LinuxCNCPositionStatus) -> tuple[bool, str]:
        if not status.available:
            return False, status.error or "LinuxCNC Python module is not available."
        if not status.connected:
            return False, status.error or "FabScan is not connected to LinuxCNC."
        if status.task_state != "ON":
            return False, f"LinuxCNC task state must be ON before jogging. Current state: {status.task_state}."
        if status.interp_state != "IDLE":
            return False, f"LinuxCNC interpreter must be IDLE before jogging. Current state: {status.interp_state}."
        if status.task_mode != "MANUAL":
            return False, (
                "LinuxCNC task mode must already be MANUAL before FabScan jogs. "
                f"Current mode: {status.task_mode}. Switch LinuxCNC/QtPlasmaC to manual/jog mode."
            )
        if not status.all_xyz_homed:
            return False, f"X/Y/Z must be homed before FabScan jogs. Current homed state: {status.homed_text}."
        return True, "OK"

    def _ok_for_controlled_xy_move(self, status: LinuxCNCPositionStatus) -> tuple[bool, str]:
        if not status.available:
            return False, status.error or "LinuxCNC Python module is not available."
        if not status.connected:
            return False, status.error or "FabScan is not connected to LinuxCNC."
        if status.task_state != "ON":
            return False, f"LinuxCNC task state must be ON before controlled motion. Current state: {status.task_state}."
        if status.interp_state != "IDLE":
            return False, f"LinuxCNC interpreter must be IDLE before controlled motion. Current state: {status.interp_state}."
        if status.task_mode not in ("MANUAL", "MDI"):
            return False, (
                "LinuxCNC task mode must be MANUAL or MDI before controlled motion. "
                f"Current mode: {status.task_mode}."
            )
        if not status.all_xyz_homed:
            return False, f"X/Y/Z must be homed before controlled motion. Current homed state: {status.homed_text}."
        return True, "OK"

    def _calculate_work_position(self, machine_position: Position3) -> Position3:
        """Calculate a practical work-coordinate display from LinuxCNC status.

        LinuxCNC reports machine position plus active offsets. For this FabScan
        trace workflow, subtracting G5x, G92, and tool offsets gives the normal
        displayed work position for basic X/Y capture.
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

    def _task_mode_text(self, value: object) -> str:
        linuxcnc = self._linuxcnc
        mapping = {
            getattr(linuxcnc, "MODE_MANUAL", object()): "MANUAL",
            getattr(linuxcnc, "MODE_MDI", object()): "MDI",
            getattr(linuxcnc, "MODE_AUTO", object()): "AUTO",
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
