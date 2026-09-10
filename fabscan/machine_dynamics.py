from __future__ import annotations

from dataclasses import dataclass, asdict
import math
from typing import Any


@dataclass(frozen=True)
class DynamicsStepResult:
    start_x: float
    start_y: float
    end_x: float
    end_y: float
    start_vx: float
    start_vy: float
    end_vx: float
    end_vy: float
    desired_vx: float
    desired_vy: float
    duration_s: float
    distance_in: float
    speed_start_ipm: float
    speed_end_ipm: float
    speed_desired_ipm: float
    accel_limited: bool
    velocity_limited: bool
    integration_steps: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class XYMachineDynamics:
    """Simple deterministic X/Y velocity dynamics for the FabLabPlasma table.

    This intentionally models only configured LinuxCNC motion limits:
      * per-axis velocity cannot exceed max_velocity_ipm,
      * per-axis velocity can change only by MAX_ACCELERATION,
      * integration runs at the configured servo period.

    It does not model following error, step loss, backlash, flex, vibration,
    servo/stepgen electronics, motion blur, or LinuxCNC look-ahead internals.
    Those remain separate future realism layers.
    """

    def __init__(
        self,
        *,
        accel_x_in_s2: float = 60.0,
        accel_y_in_s2: float = 60.0,
        max_velocity_ipm: float = 700.0,
        servo_period_ms: float = 4.0,
    ) -> None:
        self.accel_x_in_s2 = max(1e-6, float(accel_x_in_s2))
        self.accel_y_in_s2 = max(1e-6, float(accel_y_in_s2))
        self.max_velocity_in_s = max(1e-6, float(max_velocity_ipm) / 60.0)
        self.max_velocity_ipm = float(max_velocity_ipm)
        self.servo_period_s = max(1e-6, float(servo_period_ms) / 1000.0)
        self.servo_period_ms = float(servo_period_ms)
        self.vx = 0.0
        self.vy = 0.0
        self.last_samples: list[tuple[float, float, float, float, float]] = []

    def reset(self) -> None:
        self.vx = 0.0
        self.vy = 0.0
        self.last_samples = []

    @property
    def speed_ipm(self) -> float:
        return 60.0 * math.hypot(self.vx, self.vy)

    def config_dict(self) -> dict[str, Any]:
        return {
            "model": "per-axis velocity vector with acceleration clamp",
            "source": "FabLabPlasma.ini supplied 2026-09-09",
            "accel_x_in_s2": self.accel_x_in_s2,
            "accel_y_in_s2": self.accel_y_in_s2,
            "decel_x_in_s2": self.accel_x_in_s2,
            "decel_y_in_s2": self.accel_y_in_s2,
            "max_velocity_ipm": self.max_velocity_ipm,
            "servo_period_ms": self.servo_period_ms,
            "unmodeled": [
                "following error / controller lag",
                "backlash / flex",
                "vibration",
                "motion blur",
                "step loss",
                "LinuxCNC trajectory look-ahead internals",
            ],
        }

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return lower if value < lower else upper if value > upper else value

    def _integrate_axis(self, velocity: float, desired: float, accel: float, dt: float) -> tuple[float, float, bool]:
        dv = desired - velocity
        max_dv = accel * dt
        applied = self._clamp(dv, -max_dv, max_dv)
        new_velocity = velocity + applied
        displacement = 0.5 * (velocity + new_velocity) * dt
        return new_velocity, displacement, abs(dv) > max_dv + 1e-12

    def advance(
        self,
        *,
        x: float,
        y: float,
        direction_x: float,
        direction_y: float,
        requested_speed_ipm: float,
        duration_s: float,
    ) -> DynamicsStepResult:
        start_x = float(x)
        start_y = float(y)
        start_vx = float(self.vx)
        start_vy = float(self.vy)
        requested_in_s = max(0.0, float(requested_speed_ipm) / 60.0)
        n = math.hypot(float(direction_x), float(direction_y))
        if n <= 1e-12 or requested_in_s <= 1e-12:
            desired_vx = desired_vy = 0.0
        else:
            ux = float(direction_x) / n
            uy = float(direction_y) / n
            desired_vx = ux * requested_in_s
            desired_vy = uy * requested_in_s

        velocity_limited = False
        if abs(desired_vx) > self.max_velocity_in_s:
            desired_vx = self._clamp(desired_vx, -self.max_velocity_in_s, self.max_velocity_in_s)
            velocity_limited = True
        if abs(desired_vy) > self.max_velocity_in_s:
            desired_vy = self._clamp(desired_vy, -self.max_velocity_in_s, self.max_velocity_in_s)
            velocity_limited = True

        remaining = max(0.0, float(duration_s))
        px, py = start_x, start_y
        accel_limited = False
        steps = 0
        elapsed = 0.0
        self.last_samples = [(0.0, px, py, float(self.vx), float(self.vy))]
        while remaining > 1e-12:
            dt = min(self.servo_period_s, remaining)
            new_vx, dx, lim_x = self._integrate_axis(self.vx, desired_vx, self.accel_x_in_s2, dt)
            new_vy, dy, lim_y = self._integrate_axis(self.vy, desired_vy, self.accel_y_in_s2, dt)
            px += dx
            py += dy
            self.vx = new_vx
            self.vy = new_vy
            accel_limited = accel_limited or lim_x or lim_y
            remaining -= dt
            elapsed += dt
            steps += 1
            self.last_samples.append((elapsed, px, py, float(self.vx), float(self.vy)))

        distance = math.hypot(px - start_x, py - start_y)
        return DynamicsStepResult(
            start_x=start_x,
            start_y=start_y,
            end_x=px,
            end_y=py,
            start_vx=start_vx,
            start_vy=start_vy,
            end_vx=float(self.vx),
            end_vy=float(self.vy),
            desired_vx=desired_vx,
            desired_vy=desired_vy,
            duration_s=max(0.0, float(duration_s)),
            distance_in=distance,
            speed_start_ipm=60.0 * math.hypot(start_vx, start_vy),
            speed_end_ipm=60.0 * math.hypot(self.vx, self.vy),
            speed_desired_ipm=60.0 * math.hypot(desired_vx, desired_vy),
            accel_limited=accel_limited,
            velocity_limited=velocity_limited,
            integration_steps=steps,
        )

    def brake_to_stop(self, *, x: float, y: float) -> DynamicsStepResult:
        """Decelerate the current velocity vector to zero using configured limits."""
        start_speed = math.hypot(self.vx, self.vy)
        if start_speed <= 1e-12:
            return self.advance(x=x, y=y, direction_x=0.0, direction_y=0.0, requested_speed_ipm=0.0, duration_s=0.0)
        tx = abs(self.vx) / self.accel_x_in_s2
        ty = abs(self.vy) / self.accel_y_in_s2
        duration = max(tx, ty)
        return self.advance(x=x, y=y, direction_x=0.0, direction_y=0.0, requested_speed_ipm=0.0, duration_s=duration)
