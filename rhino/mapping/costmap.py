"""Obstacle inflation costmap."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
from scipy.ndimage import binary_dilation  # type: ignore[import-untyped]

from rhino.config import MapConfig
from rhino.mapping.occupancy import OccupancyMapper

_OCCUPIED_THRESH = 0.65


class Costmap:
    def __init__(self, mapper: OccupancyMapper, cfg: MapConfig) -> None:
        self._mapper = mapper
        r = max(1, int(cfg.inflation_radius / cfg.resolution))
        y, x = np.ogrid[-r : r + 1, -r : r + 1]
        self._struct: NDArray[np.bool_] = (x * x + y * y <= r * r)

    def get(self) -> NDArray[np.float32]:
        """Return inflated binary obstacle grid (1.0=blocked, 0.0=clear)."""
        prob = self._mapper.get_grid()
        obstacle = prob > _OCCUPIED_THRESH
        inflated: NDArray[np.bool_] = binary_dilation(obstacle, structure=self._struct)
        return inflated.astype(np.float32)
