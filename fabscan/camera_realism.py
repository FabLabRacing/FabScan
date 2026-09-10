from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np

INTEGRATION_VERSION = "0.6.0-dev-m5.4"


class RealCameraModel:
    """Empirical image-evidence model derived from FabScan A-G camera recordings.

    It changes only the rendered camera image. It never sees SVG truth, planner
    state, or future path. 100% is the measured appearance model; >100% scales
    blur/noise/lighting deviations as a deliberate robustness stress test.
    """

    def __init__(self, asset_dir: Path | str) -> None:
        root = Path(asset_dir).expanduser().resolve()
        profile_path = root / "real_camera_profile.json"
        field_path = root / "real_camera_illumination.npz"
        texture_path = root / "real_camera_texture.npz"
        if not profile_path.is_file() or not field_path.is_file() or not texture_path.is_file():
            raise ValueError(f"Missing M5.4 real-camera model assets in {root}")
        self.profile: dict[str, Any] = json.loads(profile_path.read_text(encoding="utf-8"))
        self.illumination = np.asarray(np.load(field_path)["illumination"], dtype=np.float32)
        texture = np.load(texture_path)
        self.low_texture = np.asarray(texture["low"], dtype=np.float32) / 32.0
        self.high_texture = np.asarray(texture["high"], dtype=np.float32) / 32.0
        self.background = float(self.profile["background_gray"])
        self.line = float(self.profile["line_gray"])
        self.frame_brightness_sigma = float(self.profile["frame_brightness_sigma"])
        self.high_frequency_sigma = float(self.profile["high_frequency_sigma"])
        self.low_frequency_sigma = float(self.profile["low_frequency_texture_sigma"])
        self.extra_blur_sigma = float(self.profile["extra_blur_sigma_px"])
        self.frames_measured = int(self.profile.get("frames_measured", 0))
        self.texture_ppi = float(self.profile.get("texture_ppi", 635.0))

    @staticmethod
    def _wrapped_crop(texture: np.ndarray, h: int, w: int, ox: int, oy: int) -> np.ndarray:
        yy = (np.arange(h, dtype=np.int32) + int(oy)) % texture.shape[0]
        xx = (np.arange(w, dtype=np.int32) + int(ox)) % texture.shape[1]
        return texture[np.ix_(yy, xx)]

    def apply(
        self,
        rgb: np.ndarray,
        *,
        realism_percent: float,
        frame_index: int,
        seed: int = 5400,
        camera_x: float = 0.0,
        camera_y: float = 0.0,
    ) -> np.ndarray:
        strength = max(0.0, float(realism_percent) / 100.0)
        if strength <= 1e-9:
            return np.asarray(rgb, dtype=np.uint8).copy()

        arr = np.asarray(rgb, dtype=np.float32)
        gray = cv2.cvtColor(arr.astype(np.uint8), cv2.COLOR_RGB2GRAY).astype(np.float32) if arr.ndim == 3 else arr

        # Keep measured paper/ink levels physically plausible above 100%; only
        # imperfections are amplified beyond the measured model.
        photometric = min(1.0, strength)
        normalized = np.clip(gray / 255.0, 0.0, 1.0)
        realistic_levels = self.line + normalized * (self.background - self.line)
        out = gray * (1.0 - photometric) + realistic_levels * photometric

        blur_sigma = self.extra_blur_sigma * strength
        if blur_sigma > 0.05:
            out = cv2.GaussianBlur(out, (0, 0), blur_sigma)

        h, w = out.shape
        field = self.illumination
        if field.shape != (h, w):
            field = cv2.resize(field, (w, h), interpolation=cv2.INTER_LINEAR)
        out += field * strength

        # Paper texture is spatially tied to sheet coordinates, so revisiting the
        # same virtual location reproduces the same texture instead of inventing a
        # new random paper pattern every frame.
        ox = int(round(float(camera_x) * self.texture_ppi)) + int(seed) * 17
        oy = int(round(float(camera_y) * self.texture_ppi)) + int(seed) * 29
        low = self._wrapped_crop(self.low_texture, h, w, ox, oy)
        high = self._wrapped_crop(self.high_texture, h, w, ox * 3 + 11, oy * 5 + 7)
        out += low * (self.low_frequency_sigma * strength)
        out += high * (self.high_frequency_sigma * strength)

        # Small exposure/illumination drift observed between recorded frames.
        rng = np.random.default_rng(int(seed) + int(frame_index) * 10007)
        out += float(rng.normal(0.0, self.frame_brightness_sigma * strength))

        out = np.clip(out, 0.0, 255.0).astype(np.uint8)
        return np.repeat(out[:, :, None], 3, axis=2)

    def summary(self) -> str:
        p = self.profile
        return (
            f"Measured from {self.frames_measured} real A-G frames: "
            f"paper {self.background:.1f} gray, line {self.line:.1f} gray, "
            f"fixed illumination σ {float(p.get('illumination_std', 0.0)):.1f}, "
            f"HF noise σ {self.high_frequency_sigma:.2f}, edge 10-90% "
            f"{float(p.get('edge_10_90_px', 0.0)):.1f} px."
        )
