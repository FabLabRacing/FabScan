from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any


APP_NAME = "fabscan"
SETTINGS_FILE_NAME = "settings.json"


DEFAULT_SETTINGS: dict[str, Any] = {
    "window_geometry": "1280x820",
    "threshold": 127,
    "blur": 3,
    "noise_removal": 0,
    "edge_cleanup": 0,
    "min_area": 1000.0,
    "simplify_percent": 0.05,
    "invert": False,
    "show_threshold": False,
    "sanity_expected_width_inches": 0.0,
    "sanity_expected_height_inches": 0.0,
    "sanity_tolerance_inches": 0.010,
    "export_origin_label": "Move lower-left to 0,0",
    "export_margin_inches": 0.0,
    "linuxcnc_coord_mode_label": "Work coordinates",
    "linuxcnc_auto_refresh": False,
    "trace_closed": True,
    "trace_preview": True,
    "trace_show_live_position": True,
    "trace_show_point_numbers": True,
    "jog_controls_enabled": False,
    "jog_step": 0.010,
    "jog_feed_units_per_min": 10.0,
    "trace_fit_segments": 72,
    "controlled_motion_feed_units_per_min": 20.0,
    "contour_filter_label": "All",
    "contour_sort_label": "Layer + area",
    "last_image_dir": str(Path.home()),
    "last_export_dir": str(Path.cwd() / "exports"),
    "last_capture_dir": str(Path.home() / "Pictures" / "FabScan Captures"),
    "camera_index": 0,
    "camera_width": 1280,
    "camera_height": 720,
    "camera_rotate_degrees": 0,
    "camera_flip_x": False,
    "camera_flip_y": False,
    "camera_fine_rotation_degrees": 0.0,
    "camera_show_crosshair": True,
    "camera_show_axis_labels": True,
    "camera_show_grid": False,
    "camera_calibration_threshold": 90,
    "camera_calibration_move_distance": 0.100,
    "camera_calibration_feed_units_per_min": 5.0,
    "camera_calibration_jog_step": 0.010,
    "camera_calibration_center_max_move": 0.100,
    "camera_calibration_line_mode": "Line center",
    "camera_calibration_line_search_px": 220,
    "camera_calibration_show_line_preview": True,
    "camera_calibration_show_mask": False,
    "camera_follow_step": 0.050,
    "camera_follow_max_correct": 0.050,
    "camera_follow_min_confidence": 45.0,
    "camera_follow_direction": "Forward",
    "camera_follow_capture_point": False,
    "camera_follow_enabled": False,
    "camera_calibration": None,
}


def get_settings_path() -> Path:
    """Return the per-user FabScan settings file path.

    Linux follows XDG_CONFIG_HOME when present, otherwise ~/.config/fabscan.
    Windows uses APPDATA when present. This keeps settings out of the git repo.
    """

    if os.name == "nt":
        base = Path(os.environ.get("APPDATA", Path.home() / "AppData" / "Roaming"))
    else:
        base = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config"))

    return base / APP_NAME / SETTINGS_FILE_NAME


def load_settings() -> dict[str, Any]:
    """Load saved settings, falling back to defaults if the file is missing/bad."""

    settings = DEFAULT_SETTINGS.copy()
    path = get_settings_path()

    try:
        if path.exists():
            with path.open("r", encoding="utf-8") as file:
                loaded = json.load(file)
            if isinstance(loaded, dict):
                settings.update(loaded)
    except Exception:
        # Bad settings should never stop FabScan from starting.
        return DEFAULT_SETTINGS.copy()

    return settings


def save_settings(settings: dict[str, Any]) -> Path:
    """Write settings as readable JSON and return the saved path."""

    path = get_settings_path()
    path.parent.mkdir(parents=True, exist_ok=True)

    clean_settings = DEFAULT_SETTINGS.copy()
    clean_settings.update(settings)

    with path.open("w", encoding="utf-8") as file:
        json.dump(clean_settings, file, indent=2, sort_keys=True)
        file.write("\n")

    return path
