from __future__ import annotations

import asyncio
from dataclasses import dataclass, field

from rhino.platforms.base import CameraFrame, Pose, RobotStatus


@dataclass
class AppState:
    latest_pose: Pose | None = None
    latest_camera: CameraFrame | None = None
    latest_status: RobotStatus = field(default_factory=RobotStatus)
    _lock: asyncio.Lock = field(default_factory=asyncio.Lock, repr=False)
