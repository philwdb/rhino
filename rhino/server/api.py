"""FastAPI server — REST + SSE."""

from __future__ import annotations

import io

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, Response
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

        mapper = self._mapper

        @app.get("/api/map")
        async def get_map() -> Response:
            grid = mapper.get_grid()                        # float32 0-1
            img = (255.0 * (1.0 - grid)).astype(np.uint8)  # free=white, occ=black
            img = img[::-1, :]                              # north-up
            _, buf = cv2.imencode(".png", img)
            return Response(content=buf.tobytes(), media_type="image/png")

        @app.get("/", response_class=HTMLResponse)
        async def teleop_ui() -> str:
            return _TELEOP_HTML

        # TODO Phase 4: /api/events (SSE), /api/pov,
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


_TELEOP_HTML = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>rhino teleop</title>
<style>
  body { font-family: monospace; background: #111; color: #eee; margin: 0; padding: 24px; }
  h2   { margin: 0 0 16px; color: #7cf; }
  #map { display: block; image-rendering: pixelated; width: 400px; height: 400px;
         border: 1px solid #444; margin-bottom: 16px; }
  #pose { margin-bottom: 16px; color: #aaa; }
  #keys { display: grid; grid-template-columns: repeat(3, 52px); gap: 4px; margin-bottom: 16px; }
  .key  { background: #222; border: 1px solid #555; border-radius: 4px;
          width: 48px; height: 48px; display: flex; align-items: center;
          justify-content: center; font-size: 18px; transition: background .05s; }
  .key.active { background: #7cf; color: #111; }
  #hint { color: #555; font-size: 12px; }
</style>
</head>
<body>
<h2>rhino teleop</h2>
<img id="map" src="/api/map" alt="map">
<div id="pose">pose: —</div>
<div id="keys">
  <div></div>
  <div class="key" id="kW">W</div>
  <div></div>
  <div class="key" id="kA">A</div>
  <div class="key" id="kS">S</div>
  <div class="key" id="kD">D</div>
</div>
<div id="hint">W/S forward/back &nbsp; A/D rotate &nbsp; SPACE stop</div>
<script>
const VX = 0.4, OMEGA = 0.8;
const pressed = new Set();
const keyMap = { KeyW:'W', KeyS:'S', KeyA:'A', KeyD:'D' };

function sendVel() {
  let vx = 0, omega = 0;
  if (pressed.has('KeyW')) vx += VX;
  if (pressed.has('KeyS')) vx -= VX * 0.6;
  if (pressed.has('KeyA')) omega += OMEGA;
  if (pressed.has('KeyD')) omega -= OMEGA;
  fetch('/api/velocity', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({vx, vy: 0, omega})
  });
}

document.addEventListener('keydown', e => {
  if (e.code === 'Space') { e.preventDefault(); pressed.clear(); sendVel(); return; }
  if (!(e.code in keyMap) || e.repeat) return;
  e.preventDefault();
  pressed.add(e.code);
  document.getElementById('k' + keyMap[e.code])?.classList.add('active');
  sendVel();
});

document.addEventListener('keyup', e => {
  if (!(e.code in keyMap)) return;
  pressed.delete(e.code);
  document.getElementById('k' + keyMap[e.code])?.classList.remove('active');
  sendVel();
});

// Refresh map and pose every second.
setInterval(() => {
  document.getElementById('map').src = '/api/map?' + Date.now();
  fetch('/api/state').then(r => r.json()).then(d => {
    const p = d.pose;
    if (p) document.getElementById('pose').textContent =
      'x: ' + p.x.toFixed(2) + '  y: ' + p.y.toFixed(2) +
      '  yaw: ' + (p.yaw * 180 / Math.PI).toFixed(1) + '°';
  });
}, 1000);
</script>
</body>
</html>"""
