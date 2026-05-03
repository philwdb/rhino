"""P-controller: pose error → (vx, vy, ω)."""

from __future__ import annotations

import math

from rhino.config import NavConfig
from rhino.platforms.base import Pose


def compute_velocity(
    pose: Pose,
    target_x: float,
    target_y: float,
    target_yaw: float | None,
    cfg: NavConfig,
) -> tuple[float, float, float]:
    dx = target_x - pose.x
    dy = target_y - pose.y
    dist = math.hypot(dx, dy)
    desired_yaw = math.atan2(dy, dx)
    heading_err = _wrap(desired_yaw - pose.yaw)

    # Scale forward speed by heading alignment — turn in place when misaligned.
    alignment = max(0.0, math.cos(heading_err))
    vx = min(cfg.kp_linear * dist, cfg.max_linear_vel) * alignment
    omega = max(-cfg.max_angular_vel, min(cfg.max_angular_vel, cfg.kp_angular * heading_err))
    return vx, 0.0, omega


def _wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
