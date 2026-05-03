"""2D log-odds occupancy grid with raycasting.  Phase 2 implementation."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from rhino.config import MapConfig
from rhino.platforms.base import LidarScan, Pose


class OccupancyMapper:
    def __init__(self, cfg: MapConfig) -> None:
        self._cfg = cfg
        self._grid: NDArray[np.float32] = np.full(
            (cfg.height, cfg.width), 0.5, dtype=np.float32
        )
        self._origin_x = -cfg.width * cfg.resolution / 2
        self._origin_y = -cfg.height * cfg.resolution / 2

    def update(self, scan: LidarScan, pose: Pose) -> None:
        # TODO Phase 2: raycast scan into log-odds grid
        pass

    def get_grid(self) -> NDArray[np.float32]:
        return self._grid.copy()

    @property
    def resolution(self) -> float:
        return self._cfg.resolution

    @property
    def origin(self) -> tuple[float, float]:
        return self._origin_x, self._origin_y
