"""A* navigator + P-controller.  Phase 3 implementation."""

from __future__ import annotations

import asyncio
from typing import Any

from rhino.config import NavConfig
from rhino.mapping.occupancy import OccupancyMapper
from rhino.platforms.base import Goal, Platform, Pose


class Navigator:
    def __init__(self, mapper: OccupancyMapper, platform: Platform, cfg: NavConfig) -> None:
        self._mapper = mapper
        self._platform = platform
        self._cfg = cfg
        self._goal: Goal | None = None
        self._pose: Pose | None = None

    def update_pose(self, pose: Pose) -> None:
        self._pose = pose

    def set_goal(self, goal: Goal) -> None:
        self._goal = goal

    def clear_goal(self) -> None:
        self._goal = None
        self._platform.send_vel(0.0, 0.0, 0.0)

    @property
    def current_goal(self) -> Goal | None:
        return self._goal

    @property
    def current_pose(self) -> Pose | None:
        return self._pose

    async def run(self) -> None:
        # TODO Phase 3: replan A* every cfg.replan_interval, run P-controller
        while True:
            await asyncio.sleep(self._cfg.replan_interval)
