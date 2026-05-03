"""Obstacle inflation costmap.  Phase 2 implementation."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
from scipy.ndimage import binary_dilation  # type: ignore[import-untyped]

from rhino.config import MapConfig
from rhino.mapping.occupancy import OccupancyMapper


class Costmap:
    def __init__(self, mapper: OccupancyMapper, cfg: MapConfig) -> None:
        self._mapper = mapper
        self._radius_cells = int(cfg.inflation_radius / cfg.resolution)

    def get(self) -> NDArray[np.float32]:
        # TODO Phase 2: inflate occupied cells
        return self._mapper.get_grid()
