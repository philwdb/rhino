"""BFS frontier explorer — drives toward unmapped regions."""

from __future__ import annotations

import asyncio
import math

import numpy as np
from scipy.ndimage import binary_dilation  # type: ignore[import-untyped]

from rhino.mapping.occupancy import OccupancyMapper
from rhino.navigation.planner import Navigator
from rhino.platforms.base import Goal

_FREE_THRESH = 0.45
_UNKNOWN_LO = 0.45
_UNKNOWN_HI = 0.55
_MIN_DIST = 0.5       # ignore frontiers closer than this
_CLUSTER_SIZE = 0.5   # voxel size for frontier clustering (m)
_KERNEL = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=bool)


class FrontierExplorer:
    def __init__(self, mapper: OccupancyMapper, nav: Navigator) -> None:
        self._mapper = mapper
        self._nav = nav
        self._enabled = False

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled
        if not enabled:
            self._nav.clear_goal()

    @property
    def enabled(self) -> bool:
        return self._enabled

    async def run(self) -> None:
        loop = asyncio.get_running_loop()
        while True:
            await asyncio.sleep(3.0)

            if not self._enabled or self._nav.current_goal is not None:
                continue

            pose = self._nav.current_pose
            if pose is None:
                continue

            frontiers = await loop.run_in_executor(None, self._find_frontiers)
            if not frontiers:
                continue

            # Pick closest frontier cluster to current pose.
            goal_pt = min(frontiers, key=lambda p: math.hypot(p[0] - pose.x, p[1] - pose.y))
            if math.hypot(goal_pt[0] - pose.x, goal_pt[1] - pose.y) > _MIN_DIST:
                self._nav.set_goal(Goal(x=goal_pt[0], y=goal_pt[1]))

    def _find_frontiers(self) -> list[tuple[float, float]]:
        grid = self._mapper.get_grid()

        free = grid < _FREE_THRESH
        unknown = (grid >= _UNKNOWN_LO) & (grid <= _UNKNOWN_HI)

        # Frontier cells: free AND adjacent to unknown.
        frontier = free & binary_dilation(unknown, structure=_KERNEL)

        rows, cols = np.where(frontier)
        if len(rows) == 0:
            return []

        ox, oy = self._mapper.origin
        res = self._mapper.resolution
        pts = np.stack([ox + cols * res, oy + rows * res], axis=1).astype(np.float32)

        # Cluster by voxel downsampling.
        idx = np.floor(pts / _CLUSTER_SIZE).astype(np.int32)
        _, unique = np.unique(idx, axis=0, return_index=True)
        pts = pts[unique]

        return [(float(x), float(y)) for x, y in pts]
