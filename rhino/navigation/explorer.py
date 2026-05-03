"""BFS frontier explorer.  Phase 3 implementation."""

from __future__ import annotations

import asyncio

from rhino.mapping.occupancy import OccupancyMapper
from rhino.navigation.planner import Navigator


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
        # TODO Phase 3: BFS frontier detection, post goals to Navigator
        while True:
            await asyncio.sleep(5.0)
