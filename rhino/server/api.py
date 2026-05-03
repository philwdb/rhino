"""FastAPI server — REST + SSE."""

from __future__ import annotations

import math

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
from rhino.navigation.planner import Navigator, PlanMode
from rhino.platforms.base import Goal, Platform
from rhino.server.state import AppState
from rhino.storage import Storage


class VelocityRequest(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0


class NavigateRequest(BaseModel):
    x: float
    y: float
    yaw: float | None = None


class PlanModeRequest(BaseModel):
    mode: PlanMode


class PoiCreateRequest(BaseModel):
    label: str
    x: float | None = None
    y: float | None = None
    z: float = 0.0


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
        nav = self._nav
        explorer = self._explorer
        storage = self._storage

        @app.post("/api/navigate")
        async def navigate(req: NavigateRequest) -> dict[str, str]:
            nav.set_goal(Goal(x=req.x, y=req.y, yaw=req.yaw))
            return {"status": "ok"}

        @app.post("/api/navigate/cancel")
        async def navigate_cancel() -> dict[str, str]:
            explorer.set_enabled(False)
            nav.clear_goal()
            return {"status": "ok"}

        @app.get("/api/navigate/status")
        async def navigate_status() -> dict[str, object]:
            g = nav.current_goal
            return {
                "goal": {"x": g.x, "y": g.y, "yaw": g.yaw} if g else None,
                "exploring": explorer.enabled,
                "mode": nav.mode,
            }

        @app.post("/api/navigate/mode")
        async def set_mode(req: PlanModeRequest) -> dict[str, str]:
            nav.set_mode(req.mode)
            return {"status": "ok", "mode": req.mode}

        @app.post("/api/explore/start")
        async def explore_start() -> dict[str, str]:
            explorer.set_enabled(True)
            return {"status": "ok"}

        @app.post("/api/explore/stop")
        async def explore_stop() -> dict[str, str]:
            explorer.set_enabled(False)
            return {"status": "ok"}

        @app.get("/api/pois")
        async def list_pois() -> list[dict[str, object]]:
            pois = await storage.list_pois()
            return [
                {"id": p.id, "label": p.label, "x": p.x, "y": p.y, "z": p.z}
                for p in pois
            ]

        @app.post("/api/pois")
        async def create_poi(req: PoiCreateRequest) -> dict[str, object]:
            x, y = req.x, req.y
            if x is None or y is None:
                pose = state.latest_pose
                if pose is None:
                    from fastapi import HTTPException
                    raise HTTPException(422, "no pose available — provide x and y")
                x, y = pose.x, pose.y
            poi = await storage.add_poi(req.label, x, y, req.z)
            return {"id": poi.id, "label": poi.label, "x": poi.x, "y": poi.y, "z": poi.z}

        @app.delete("/api/pois/{poi_id}")
        async def delete_poi(poi_id: str) -> dict[str, object]:
            deleted = await storage.delete_poi(poi_id)
            return {"deleted": deleted}

        @app.post("/api/pois/{poi_id}/navigate")
        async def navigate_to_poi(poi_id: str) -> dict[str, object]:
            pois = await storage.list_pois()
            poi = next((p for p in pois if p.id == poi_id), None)
            if poi is None:
                from fastapi import HTTPException
                raise HTTPException(404, f"poi {poi_id!r} not found")
            nav.set_goal(Goal(x=poi.x, y=poi.y))
            return {"status": "ok", "label": poi.label, "x": poi.x, "y": poi.y}

        @app.get("/api/map")
        async def get_map() -> Response:
            grid = mapper.get_grid()                        # float32 0-1
            gray = (255.0 * (1.0 - grid)).astype(np.uint8)  # free=white, occ=black
            gray = gray[::-1, :]                             # north-up
            img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            H, W = img.shape[:2]
            ox, oy = mapper.origin
            res = mapper.resolution

            def w2i(wx: float, wy: float) -> tuple[int, int]:
                c = int((wx - ox) / res)
                r = H - 1 - int((wy - oy) / res)
                return (c, r)

            # Draw A* path as orange polyline.
            path = nav.current_path
            if len(path) >= 2:
                pts = [w2i(x, y) for x, y in path]
                for p1, p2 in zip(pts, pts[1:]):
                    cv2.line(img, p1, p2, (0, 140, 255), 1)

            # Draw robot position: cyan dot + white heading arrow.
            pose = state.latest_pose
            if pose is not None:
                cx, cy = w2i(pose.x, pose.y)
                cv2.circle(img, (cx, cy), 4, (255, 220, 0), -1)
                arrow_len = 8
                ax = int(cx + arrow_len * math.cos(pose.yaw))
                ay = int(cy - arrow_len * math.sin(pose.yaw))  # row decreases northward
                cv2.arrowedLine(img, (cx, cy), (ax, ay), (255, 255, 255), 1, tipLength=0.4)

            _, buf = cv2.imencode(".png", img)
            return Response(content=buf.tobytes(), media_type="image/png")

        @app.get("/", response_class=HTMLResponse)
        async def teleop_ui() -> str:
            return _TELEOP_HTML

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
  body  { font-family: monospace; background: #111; color: #eee; margin: 0; padding: 24px; }
  h2    { margin: 0 0 16px; color: #7cf; }
  #map  { display: block; image-rendering: pixelated; width: 400px; height: 400px;
          border: 1px solid #444; margin-bottom: 12px; cursor: crosshair; }
  #pose { margin-bottom: 12px; color: #aaa; }
  #nav-status { margin-bottom: 12px; color: #fa0; min-height: 1.2em; }
  #keys { display: grid; grid-template-columns: repeat(3, 52px); gap: 4px; margin-bottom: 12px; }
  .key  { background: #222; border: 1px solid #555; border-radius: 4px; width: 48px; height: 48px;
          display: flex; align-items: center; justify-content: center; font-size: 18px; }
  .key.active { background: #7cf; color: #111; }
  #controls { display: flex; gap: 8px; flex-wrap: wrap; margin-bottom: 8px; }
  button { background: #333; color: #eee; border: 1px solid #555; border-radius: 4px;
           padding: 6px 12px; cursor: pointer; font-family: monospace; }
  button:hover { background: #444; }
  button.on { background: #274; border-color: #4a4; }
  #hint { color: #555; font-size: 12px; }
</style>
</head>
<body>
<h2>rhino teleop</h2>
<img id="map" src="/api/map" alt="map" title="Click to navigate here">
<div id="pose">pose: —</div>
<div id="nav-status"></div>
<div id="keys">
  <div></div><div class="key" id="kW">W</div><div></div>
  <div class="key" id="kA">A</div><div class="key" id="kS">S</div><div class="key" id="kD">D</div>
</div>
<div id="controls">
  <button id="btn-stop" onclick="post('/api/stop')">■ Stop</button>
  <button id="btn-cancel" onclick="post('/api/navigate/cancel')">✕ Cancel nav</button>
  <button id="btn-explore" onclick="toggleExplore()">⟳ Explore</button>
  <button id="btn-mode" onclick="toggleMode()">mode: A*</button>
</div>
<div id="hint">W/S forward/back &nbsp; A/D rotate &nbsp; SPACE stop &nbsp; click map to navigate</div>
<script>
const VX = 0.4, OMEGA = 0.8;
const pressed = new Set();
const keyMap = { KeyW:'W', KeyS:'S', KeyA:'A', KeyD:'D' };
let exploring = false;
let planMode = 'astar';

function post(url, body) {
  return fetch(url, { method:'POST',
    headers: body ? {'Content-Type':'application/json'} : {},
    body: body ? JSON.stringify(body) : undefined });
}

function sendVel() {
  let vx = 0, omega = 0;
  if (pressed.has('KeyW')) vx += VX;
  if (pressed.has('KeyS')) vx -= VX * 0.6;
  if (pressed.has('KeyA')) omega += OMEGA;
  if (pressed.has('KeyD')) omega -= OMEGA;
  post('/api/velocity', {vx, vy:0, omega});
}

// Heartbeat at 10 Hz — robot watchdog requires continuous commands or it stops.
setInterval(sendVel, 100);

document.addEventListener('keydown', e => {
  if (e.code === 'Space') { e.preventDefault(); pressed.clear(); sendVel(); return; }
  if (!(e.code in keyMap) || e.repeat) return;
  e.preventDefault();
  pressed.add(e.code);
  document.getElementById('k'+keyMap[e.code])?.classList.add('active');
  sendVel();
});
document.addEventListener('keyup', e => {
  if (!(e.code in keyMap)) return;
  pressed.delete(e.code);
  document.getElementById('k'+keyMap[e.code])?.classList.remove('active');
  sendVel();
});

// Click map → navigate to that world position.
document.getElementById('map').addEventListener('click', e => {
  const img = e.currentTarget;
  const rect = img.getBoundingClientRect();
  const px = (e.clientX - rect.left) / rect.width;   // 0-1
  const py = (e.clientY - rect.top)  / rect.height;  // 0-1 (0=top=north)
  // Map image: 400px = 20m (200 cells × 0.1m), origin at centre.
  const x =  (px - 0.5) * 20;
  const y = -(py - 0.5) * 20;  // flip: top of image = +y
  post('/api/navigate', {x, y}).then(() => {
    document.getElementById('nav-status').textContent = 'navigating → (' + x.toFixed(1) + ', ' + y.toFixed(1) + ')';
  });
});

function toggleExplore() {
  exploring = !exploring;
  post(exploring ? '/api/explore/start' : '/api/explore/stop');
  document.getElementById('btn-explore').classList.toggle('on', exploring);
  document.getElementById('btn-explore').textContent = exploring ? '⟳ Exploring…' : '⟳ Explore';
}

function toggleMode() {
  planMode = planMode === 'astar' ? 'direct' : 'astar';
  post('/api/navigate/mode', {mode: planMode});
  document.getElementById('btn-mode').textContent = 'mode: ' + (planMode === 'astar' ? 'A*' : 'direct');
  document.getElementById('btn-mode').classList.toggle('on', planMode === 'direct');
}

setInterval(() => {
  document.getElementById('map').src = '/api/map?' + Date.now();
  fetch('/api/state').then(r => r.json()).then(d => {
    const p = d.pose;
    if (p) document.getElementById('pose').textContent =
      'x: '+p.x.toFixed(2)+'  y: '+p.y.toFixed(2)+'  yaw: '+(p.yaw*180/Math.PI).toFixed(1)+'°';
  });
  fetch('/api/navigate/status').then(r => r.json()).then(d => {
    if (d.goal) {
      document.getElementById('nav-status').textContent =
        'navigating → ('+d.goal.x.toFixed(1)+', '+d.goal.y.toFixed(1)+')';
    } else if (!exploring) {
      document.getElementById('nav-status').textContent = '';
    }
    if (d.exploring !== exploring) {
      exploring = d.exploring;
      document.getElementById('btn-explore').classList.toggle('on', exploring);
      document.getElementById('btn-explore').textContent = exploring ? '⟳ Exploring…' : '⟳ Explore';
    }
    if (d.mode && d.mode !== planMode) {
      planMode = d.mode;
      document.getElementById('btn-mode').textContent = 'mode: ' + (planMode === 'astar' ? 'A*' : 'direct');
      document.getElementById('btn-mode').classList.toggle('on', planMode === 'direct');
    }
  });
}, 1000);
</script>
</body>
</html>"""
