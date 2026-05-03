"""P-controller: pose error → (vx, vy, ω).  Phase 3 implementation."""

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
    # TODO Phase 3: full pure-pursuit / P-controller
    dx = target_x - pose.x
    dy = target_y - pose.y
    dist = math.hypot(dx, dy)
    desired_yaw = math.atan2(dy, dx)
    heading_err = _wrap(desired_yaw - pose.yaw)
    vx = min(cfg.kp_linear * dist, cfg.max_linear_vel)
    omega = max(-cfg.max_angular_vel, min(cfg.max_angular_vel, cfg.kp_angular * heading_err))
    return vx, 0.0, omega


def _wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
