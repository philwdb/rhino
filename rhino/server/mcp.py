"""MCP server — Phase 4 implementation."""

from __future__ import annotations

import asyncio

from rhino.config import ServerConfig
from rhino.navigation.explorer import FrontierExplorer
from rhino.navigation.planner import Navigator
from rhino.platforms.base import Platform
from rhino.server.state import AppState
from rhino.storage import Storage


class McpServer:
    def __init__(
        self,
        platform: Platform,
        nav: Navigator,
        explorer: FrontierExplorer,
        storage: Storage,
        state: AppState,
        cfg: ServerConfig,
    ) -> None:
        self._platform = platform
        self._nav = nav
        self._explorer = explorer
        self._storage = storage
        self._state = state
        self._cfg = cfg

    async def serve(self) -> None:
        # TODO Phase 4: register MCP tools and start server on self._cfg.mcp_port
        # Tools: send_velocity, relative_move, standup, execute_sport,
        #        observe, get_robot_status, navigate_to, get_nav_status,
        #        explore, tag_poi, list_pois, go_to_poi
        await asyncio.sleep(float("inf"))
