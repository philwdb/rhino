"""MCP server — Phase 4 implementation."""

from __future__ import annotations

from mcp.server.fastmcp import FastMCP

from rhino.config import ServerConfig
from rhino.mapping.occupancy import OccupancyMapper
from rhino.navigation.explorer import FrontierExplorer
from rhino.navigation.planner import Navigator
from rhino.platforms.base import Goal, Platform
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
        mapper: OccupancyMapper,
        cfg: ServerConfig,
    ) -> None:
        self._platform = platform
        self._nav = nav
        self._explorer = explorer
        self._storage = storage
        self._state = state
        self._mapper = mapper
        self._cfg = cfg
        self._mcp = self._build_mcp()

    def _build_mcp(self) -> FastMCP:
        mcp = FastMCP(
            "rhino",
            host=self._cfg.host,
            port=self._cfg.mcp_port,
            log_level="WARNING",
        )

        platform = self._platform
        nav = self._nav
        explorer = self._explorer
        storage = self._storage
        state = self._state
        mapper = self._mapper

        @mcp.tool()
        def send_velocity(vx: float, vy: float = 0.0, omega: float = 0.0) -> str:
            """Send a direct velocity command to the robot."""
            platform.send_vel(vx, vy, omega)
            return "ok"

        @mcp.tool()
        def stop() -> str:
            """Stop the robot immediately and cancel navigation."""
            explorer.set_enabled(False)
            nav.clear_goal()
            platform.send_vel(0.0, 0.0, 0.0)
            return "ok"

        @mcp.tool()
        def navigate_to(x: float, y: float, yaw: float | None = None) -> str:
            """Navigate the robot to a world-frame position (metres)."""
            nav.set_goal(Goal(x=x, y=y, yaw=yaw))
            return f"navigating to ({x:.2f}, {y:.2f})"

        @mcp.tool()
        def get_nav_status() -> dict[str, object]:
            """Return current navigation goal and exploration state."""
            g = nav.current_goal
            return {
                "goal": {"x": g.x, "y": g.y, "yaw": g.yaw} if g else None,
                "exploring": explorer.enabled,
                "mode": nav.mode,
            }

        @mcp.tool()
        def explore(enabled: bool) -> str:
            """Enable or disable autonomous frontier exploration."""
            explorer.set_enabled(enabled)
            return "exploring" if enabled else "exploration stopped"

        @mcp.tool()
        def get_robot_status() -> dict[str, object]:
            """Return robot pose and status."""
            pose = state.latest_pose
            status = state.latest_status
            return {
                "pose": (
                    {"x": pose.x, "y": pose.y, "yaw": pose.yaw}
                    if pose else None
                ),
                "battery_pct": status.battery_pct,
                "mode": status.mode,
                "vx": status.vx,
                "vy": status.vy,
                "omega": status.omega,
            }

        @mcp.tool()
        async def tag_poi(label: str, x: float | None = None, y: float | None = None) -> str:
            """Tag a point of interest. Uses current robot pose if x/y omitted."""
            if x is None or y is None:
                pose = state.latest_pose
                if pose is None:
                    return "error: no pose available — provide x and y explicitly"
                x, y = pose.x, pose.y
            poi = await storage.add_poi(label, x, y)
            return f"tagged '{label}' (id={poi.id}) at ({poi.x:.2f}, {poi.y:.2f})"

        @mcp.tool()
        async def list_pois() -> list[dict[str, object]]:
            """List all saved points of interest."""
            pois = await storage.list_pois()
            return [
                {"id": p.id, "label": p.label, "x": p.x, "y": p.y, "z": p.z}
                for p in pois
            ]

        @mcp.tool()
        async def go_to_poi(label_or_id: str) -> str:
            """Navigate to a saved POI by label or ID (partial match on label)."""
            pois = await storage.list_pois()
            match = next(
                (p for p in pois if p.id == label_or_id or label_or_id.lower() in p.label.lower()),
                None,
            )
            if match is None:
                known = ", ".join(p.label for p in pois) or "none"
                return f"error: no POI matching '{label_or_id}'. Known: {known}"
            nav.set_goal(Goal(x=match.x, y=match.y))
            return f"navigating to '{match.label}' at ({match.x:.2f}, {match.y:.2f})"

        @mcp.tool()
        async def delete_poi(label_or_id: str) -> str:
            """Delete a POI by label or ID."""
            pois = await storage.list_pois()
            match = next(
                (p for p in pois if p.id == label_or_id or label_or_id.lower() in p.label.lower()),
                None,
            )
            if match is None:
                return f"error: no POI matching '{label_or_id}'"
            await storage.delete_poi(match.id)
            return f"deleted '{match.label}'"

        @mcp.tool()
        def set_mapping(enabled: bool) -> str:
            """Enable or disable map building. When disabled the loaded map is frozen."""
            mapper.set_mapping(enabled)
            return "mapping enabled" if enabled else "mapping disabled (map frozen)"

        @mcp.tool()
        def save_map() -> str:
            """Save the current occupancy map to disk."""
            mapper.save()
            return "map saved"

        return mcp

    async def serve(self) -> None:
        await self._mcp.run_sse_async()
