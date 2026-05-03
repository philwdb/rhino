"""FastAPI server — REST + SSE."""

from __future__ import annotations

import asyncio
import math

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, Response, StreamingResponse
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
            status = state.latest_status
            return {
                "pose": (
                    {"x": pose.x, "y": pose.y, "yaw": pose.yaw}
                    if pose else None
                ),
                "status": {
                    "mode": status.mode,
                    "is_standing": status.is_standing,
                    "battery_pct": status.battery_pct,
                    "vx": status.vx,
                    "vy": status.vy,
                    "omega": status.omega,
                },
                "path": [[x, y] for x, y in nav.current_path],
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

        @app.get("/api/map/raw")
        async def get_map_raw() -> Response:
            grid = mapper.get_grid()
            gray = (255.0 * (1.0 - grid)).astype(np.uint8)
            gray = gray[::-1, :]  # north-up
            _, buf = cv2.imencode(".png", gray)
            return Response(content=buf.tobytes(), media_type="image/png")

        @app.get("/api/map/info")
        async def map_info() -> dict[str, object]:
            ox, oy = mapper.origin
            H, W = mapper.shape
            return {
                "origin_x": ox,
                "origin_y": oy,
                "resolution": mapper.resolution,
                "width": W,
                "height": H,
            }

        @app.get("/api/camera/stream")
        async def camera_stream() -> StreamingResponse:
            async def generate():
                while True:
                    frame = state.latest_camera
                    if frame is not None:
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n"
                            + frame.data
                            + b"\r\n"
                        )
                    await asyncio.sleep(1 / 30)
            return StreamingResponse(
                generate(),
                media_type="multipart/x-mixed-replace;boundary=frame",
            )

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
  body  { font-family: monospace; background: #111; color: #eee; margin: 0; padding: 24px; display: flex; gap: 24px; flex-wrap: wrap; }
  h2    { margin: 0 0 12px; color: #7cf; width: 100%; }
  #left { display: flex; flex-direction: column; }
  #map  { display: block; image-rendering: pixelated; width: 400px; height: 400px;
          border: 1px solid #444; margin-bottom: 10px; cursor: crosshair; }
  #nav-status { color: #fa0; min-height: 1.2em; margin-bottom: 8px; }
  #keys { display: grid; grid-template-columns: repeat(3, 52px); gap: 4px; margin-bottom: 10px; }
  .key  { background: #222; border: 1px solid #555; border-radius: 4px; width: 48px; height: 48px;
          display: flex; align-items: center; justify-content: center; font-size: 18px; }
  .key.active { background: #7cf; color: #111; }
  #controls { display: flex; gap: 8px; flex-wrap: wrap; margin-bottom: 8px; }
  button { background: #333; color: #eee; border: 1px solid #555; border-radius: 4px;
           padding: 6px 12px; cursor: pointer; font-family: monospace; }
  button:hover { background: #444; }
  button.on { background: #274; border-color: #4a4; }
  #hint { color: #555; font-size: 11px; }

  /* debug panel */
  #debug { display: flex; flex-direction: column; gap: 12px; min-width: 220px; }
  .dbg-box { background: #1a1a1a; border: 1px solid #333; border-radius: 6px; padding: 12px; }
  .dbg-title { color: #7cf; font-size: 11px; text-transform: uppercase; letter-spacing: 1px; margin-bottom: 8px; }
  .dbg-row { display: flex; justify-content: space-between; color: #aaa; margin: 3px 0; }
  .dbg-val { color: #ffe; font-weight: bold; }
  .dbg-val.pos { color: #4f4; }
  .dbg-val.neg { color: #f44; }
  #compass { display: block; margin: 8px auto 0; }
</style>
</head>
<body>
<h2>rhino teleop</h2>

<div id="left">
  <img id="map" src="/api/map" alt="map" title="Click to navigate here">
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
</div>

<div id="debug">
  <div class="dbg-box">
    <div class="dbg-title">Pose (odometry)</div>
    <div class="dbg-row"><span>x</span><span class="dbg-val" id="d-x">—</span></div>
    <div class="dbg-row"><span>y</span><span class="dbg-val" id="d-y">—</span></div>
    <div class="dbg-row"><span>yaw</span><span class="dbg-val" id="d-yaw">—</span></div>
    <canvas id="compass" width="80" height="80"></canvas>
    <div style="color:#555;font-size:10px;text-align:center;margin-top:4px">
      W=forward → x↑ or y↑?<br>A=left → yaw↑ or ↓?
    </div>
  </div>
  <div class="dbg-box">
    <div class="dbg-title">Velocity sent</div>
    <div class="dbg-row"><span>vx (forward)</span><span class="dbg-val" id="d-vx">0.00</span></div>
    <div class="dbg-row"><span>omega (turn)</span><span class="dbg-val" id="d-omega">0.00</span></div>
  </div>
  <div class="dbg-box">
    <div class="dbg-title">Navigation</div>
    <div class="dbg-row"><span>goal x</span><span class="dbg-val" id="d-gx">—</span></div>
    <div class="dbg-row"><span>goal y</span><span class="dbg-val" id="d-gy">—</span></div>
    <div class="dbg-row"><span>mode</span><span class="dbg-val" id="d-mode">—</span></div>
    <div class="dbg-row"><span>exploring</span><span class="dbg-val" id="d-exp">—</span></div>
  </div>
</div>

<script>
const VX = 0.4, OMEGA = 0.8;
const pressed = new Set();
const keyMap = { KeyW:'W', KeyS:'S', KeyA:'A', KeyD:'D' };
let exploring = false;
let planMode = 'astar';
let lastVx = 0, lastOmega = 0;

function post(url, body) {
  return fetch(url, { method:'POST',
    headers: body ? {'Content-Type':'application/json'} : {},
    body: body ? JSON.stringify(body) : undefined });
}

function val(id, v, decimals=2) {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = typeof v === 'number' ? v.toFixed(decimals) : v;
  el.className = 'dbg-val' + (typeof v === 'number' && v > 0.005 ? ' pos' : typeof v === 'number' && v < -0.005 ? ' neg' : '');
}

function drawCompass(yawRad) {
  const c = document.getElementById('compass');
  const ctx = c.getContext('2d');
  const cx = 40, cy = 40, r = 32;
  ctx.clearRect(0, 0, 80, 80);
  ctx.strokeStyle = '#333'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, 2*Math.PI); ctx.stroke();
  // N label (top = +y in world = screen up)
  ctx.fillStyle = '#555'; ctx.font = '10px monospace'; ctx.textAlign = 'center';
  ctx.fillText('N(+y)', cx, cy - r - 2);
  ctx.fillText('E(+x)', cx + r + 2, cy + 4);
  // heading arrow — yaw=0 points right (+x), yaw=π/2 points up (+y)
  const ax = cx + r * 0.8 * Math.cos(-yawRad);
  const ay = cy + r * 0.8 * Math.sin(-yawRad);  // canvas y is inverted
  ctx.strokeStyle = '#7cf'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(ax, ay); ctx.stroke();
  ctx.fillStyle = '#7cf';
  ctx.beginPath(); ctx.arc(ax, ay, 3, 0, 2*Math.PI); ctx.fill();
}

function sendVel() {
  let vx = 0, omega = 0;
  if (pressed.has('KeyW')) vx += VX;
  if (pressed.has('KeyS')) vx -= VX * 0.6;
  if (pressed.has('KeyA')) omega += OMEGA;
  if (pressed.has('KeyD')) omega -= OMEGA;
  lastVx = vx; lastOmega = omega;
  val('d-vx', vx); val('d-omega', omega);
  post('/api/velocity', {vx, vy:0, omega});
}

// Heartbeat: re-send only while keys are held so the robot watchdog is satisfied
// during teleop but navigator commands are not overwritten when idle.
setInterval(() => { if (pressed.size > 0) sendVel(); }, 100);

// Stop and clear stuck keys if the window loses focus.
window.addEventListener('blur', () => {
  pressed.clear();
  Object.values(keyMap).forEach(k => document.getElementById('k'+k)?.classList.remove('active'));
  post('/api/velocity', {vx:0, vy:0, omega:0});
  val('d-vx', 0); val('d-omega', 0);
});

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
  const px = (e.clientX - rect.left) / rect.width;
  const py = (e.clientY - rect.top)  / rect.height;
  const x =  (px - 0.5) * 20;
  const y = -(py - 0.5) * 20;
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

// Poll at 5 Hz for responsive debug display.
setInterval(() => {
  document.getElementById('map').src = '/api/map?' + Date.now();
  fetch('/api/state').then(r => r.json()).then(d => {
    const p = d.pose;
    if (p) {
      val('d-x', p.x);
      val('d-y', p.y);
      const deg = p.yaw * 180 / Math.PI;
      document.getElementById('d-yaw').textContent = deg.toFixed(1) + '°';
      document.getElementById('d-yaw').className = 'dbg-val';
      drawCompass(p.yaw);
    }
  });
  fetch('/api/navigate/status').then(r => r.json()).then(d => {
    if (d.goal) {
      document.getElementById('nav-status').textContent =
        'navigating → ('+d.goal.x.toFixed(1)+', '+d.goal.y.toFixed(1)+')';
      val('d-gx', d.goal.x); val('d-gy', d.goal.y);
    } else {
      if (!exploring) document.getElementById('nav-status').textContent = '';
      document.getElementById('d-gx').textContent = '—';
      document.getElementById('d-gy').textContent = '—';
    }
    document.getElementById('d-mode').textContent = d.mode || '—';
    document.getElementById('d-exp').textContent = d.exploring ? 'yes' : 'no';
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
}, 200);
</script>
</body>
</html>"""
