from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np


@dataclass
class CameraFrame:
    data: bytes       # JPEG bytes
    width: int
    height: int
    timestamp: float


@dataclass
class LidarScan:
    points: np.ndarray   # shape (N, 3), world-frame xyz
    timestamp: float


@dataclass
class Pose:
    x: float
    y: float
    yaw: float
    timestamp: float


@dataclass
class RobotStatus:
    battery_pct: float = 0.0
    mode: str = "unknown"
    is_standing: bool = False
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0


@dataclass
class POI:
    id: str
    label: str
    x: float
    y: float
    z: float = 0.0
    created_at: float = 0.0


@dataclass
class Goal:
    x: float
    y: float
    yaw: float | None = None


class Platform(Protocol):
    camera_queue: asyncio.Queue[CameraFrame]   # maxsize=2, drops old frames
    lidar_queue: asyncio.Queue[LidarScan]      # maxsize=4
    odom_queue: asyncio.Queue[Pose]            # maxsize=8

    async def start(self) -> None: ...
    async def stop(self) -> None: ...
    def send_vel(self, vx: float, vy: float, omega: float) -> None: ...
    def send_cmd(self, cmd: str, **kwargs: Any) -> None: ...
    def get_status(self) -> RobotStatus: ...
