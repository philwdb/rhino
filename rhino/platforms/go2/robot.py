"""Real Go2 platform via unitree-webrtc-connect-leshy.  Phase 5 implementation."""

from __future__ import annotations

import asyncio
from typing import Any

from rhino.config import RobotConfig
from rhino.platforms.base import CameraFrame, LidarScan, Pose, RobotStatus


class Go2Platform:
    def __init__(self, cfg: RobotConfig) -> None:
        self._cfg = cfg
        self.camera_queue: asyncio.Queue[CameraFrame] = asyncio.Queue(maxsize=2)
        self.lidar_queue: asyncio.Queue[LidarScan] = asyncio.Queue(maxsize=4)
        self.odom_queue: asyncio.Queue[Pose] = asyncio.Queue(maxsize=8)

    async def start(self) -> None:
        raise NotImplementedError("Phase 5: real Go2 connection not yet implemented")

    async def stop(self) -> None:
        pass

    def send_vel(self, vx: float, vy: float, omega: float) -> None:
        raise NotImplementedError

    def send_cmd(self, cmd: str, **kwargs: Any) -> None:
        raise NotImplementedError

    def get_status(self) -> RobotStatus:
        raise NotImplementedError
