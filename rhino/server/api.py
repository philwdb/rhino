"""FastAPI server — REST + SSE."""

from __future__ import annotations

import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from rhino.config import ServerConfig
from rhino.mapping.occupancy import OccupancyMapper
from rhino.navigation.explorer import FrontierExplorer
from rhino.navigation.planner import Navigator
from rhino.platforms.base import Platform
from rhino.server.state import AppState
from rhino.storage import Storage


class VelocityRequest(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0


class CmdRequest(BaseModel):
    command: str


class ApiServer:
    def __init__(
        self,
        state: AppState,
        mapper: OccupancyMapper,
        nav: Navigator,
        explorer: FrontierExplorer,
        platform: Platform,
        storage: Storage,
        cfg: ServerConfig,
    ) -> None:
        self._state = state
        self._mapper = mapper
        self._nav = nav
        self._explorer = explorer
        self._platform = platform
        self._storage = storage
        self._cfg = cfg
        self._app = self._build_app()

    def _build_app(self) -> FastAPI:
        app = FastAPI(title="rhino")
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_methods=["*"],
            allow_headers=["*"],
        )
        platform = self._platform
        state = self._state

        @app.get("/api/health")
        async def health() -> dict[str, str]:
            return {"status": "ok"}

        @app.post("/api/velocity")
        async def velocity(req: VelocityRequest) -> dict[str, str]:
            platform.send_vel(req.vx, req.vy, req.omega)
            return {"status": "ok"}

        @app.post("/api/stop")
        async def stop() -> dict[str, str]:
            platform.send_vel(0.0, 0.0, 0.0)
            return {"status": "ok"}

        @app.post("/api/cmd/{command}")
        async def cmd(command: str) -> dict[str, str]:
            platform.send_cmd(command)
            return {"status": "ok"}

        @app.get("/api/state")
        async def get_state() -> dict[str, object]:
            pose = state.latest_pose
            return {
                "pose": (
                    {"x": pose.x, "y": pose.y, "yaw": pose.yaw}
                    if pose else None
                ),
                "status": {
                    "mode": state.latest_status.mode,
                    "vx": state.latest_status.vx,
                    "vy": state.latest_status.vy,
                    "omega": state.latest_status.omega,
                },
            }

        # TODO Phase 4: /api/events (SSE), /api/map, /api/pov,
        #               /api/navigate, /api/explore,
        #               /api/pois CRUD + navigation

        return app

    async def serve(self) -> None:
        config = uvicorn.Config(
            self._app,
            host=self._cfg.host,
            port=self._cfg.port,
            log_level="warning",
        )
        server = uvicorn.Server(config)
        await server.serve()
