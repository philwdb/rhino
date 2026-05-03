# rhino

A clean robotics OS for the Unitree Go2. Independent of dimos. Rerun-first.

**Rules:** explicit wiring over auto-discovery · asyncio for I/O, threads where forced · one DB, one way to do things · readable beats generalisable.

---

## How it actually works

### What runs on the Go2

The Go2's onboard computer (Orin NX) runs Unitree's proprietary locomotion controller — a black box that handles gait, balance, and leg control. You never touch any of that. On top of it, Unitree runs a **WebRTC bridge** that rhino connects to over the local network (the robot acts as a WiFi access point, or joins your LAN).

The robot publishes three data streams over WebRTC:
- **Camera** — H.264-encoded video at 1280×720 @ 30 fps
- **LiDAR** — raw scan packets (range + angle per beam, projected to base_link frame by the SDK)
- **Low-state** — joint encoder + IMU data at ~50 Hz; the SDK derives odometry from this (leg kinematics + IMU fusion)

The robot subscribes to one command type:
- **Velocity command** — `(vx, vy, ω)` in the robot's body frame; `vx` = forward, `vy` = left, `ω` = yaw rate. The locomotion controller converts this into stepping behaviour internally.
- **Sport commands** — named strings (`"standup"`, `"liedown"`, `"jump"`, etc.) for discrete behaviours.

That's the full interface. You say "walk 0.3 m/s forward and turn 0.1 rad/s left" — the robot decides how to move its legs.

### What runs on the host PC (rhino)

All intelligence runs on the host. The robot is a sensor+actuator platform. The data loop:

```
Go2 (WebRTC)
  │  camera frames, lidar scans, odometry
  ▼
rhino (host PC)
  ├── OccupancyMapper  — raycasts each lidar scan into a 2D log-odds grid
  ├── Navigator        — A* on the costmap → planned path → P-controller → (vx, vy, ω)
  ├── FrontierExplorer — picks unvisited frontiers as navigation goals
  ├── RerunLogger      — streams everything to the Rerun viewer
  └── FastAPI + MCP    — web dashboard + Claude tool access
  │  velocity commands (vx, vy, ω)
  ▼
Go2 locomotion controller → legs move
```

### Odometry and drift

The Go2's odometry is **proprioceptive** — it integrates leg kinematics and IMU readings. It does not use LiDAR or cameras for localisation. In a typical indoor space (≤50 m²) it stays accurate enough for 10–15 minutes of exploration at 0.1 m map resolution. There is no loop-closure SLAM. For longer sessions or larger spaces, drift will cause the map to warp — that's a known limitation of this design (same as dimos).

### The navigation loop in detail

1. `odom_loop` receives a `Pose` from the Go2, updates `state.latest_pose`
2. `lidar_loop` receives a `LidarScan`, reads `state.latest_pose`, calls `mapper.update()` in a thread executor
3. `mapper.update()` raycasts the scan: marks free cells along each beam, occupied at the endpoint; updates the log-odds grid
4. `costmap.py` inflates obstacles by the robot's footprint radius → cost grid for A\*
5. `nav.run()` wakes on a timer (default 1 s), runs A\* in executor from current pose to current goal
6. `controller` reads current pose, finds the nearest look-ahead point on the path, computes position and heading error, outputs `(vx, vy, ω)`
7. `platform.send_vel(vx, vy, ω)` pushes the command over WebRTC to the robot
8. Repeat until `|position_error| < arrival_tolerance`

`FrontierExplorer` detects the boundary between known-free and unknown cells (frontiers), picks the nearest reachable one, and posts it as a `Goal` to `Navigator`. When the robot arrives, `FrontierExplorer` picks the next frontier.

### Simulation

`MujocoGo2` uses the same interface as `Go2Platform`. It launches a Mujoco subprocess (the Go2's physics model, same XML as dimos), communicates via shared memory, and surfaces the same three queues + `send_vel`. The rest of rhino is unaware it's running in sim.

---

## What we keep from dimos

Go2 WebRTC connection · Mujoco subprocess + SHM pattern · 2D log-odds occupancy grid · costmap inflation · A\* replanning · BFS frontier exploration · P-controller velocity tracking · sport commands · MCP server · FastAPI + SSE · React dashboard · Rerun visualisation.

**Dropped:** LCM, Blueprint system, Foxglove, automatic VLM/POI detection, OpenAI, manipulation, drones, recording/replay, Pinocchio/Drake, Textual TUI, 3D voxel mapping.

---

## Threading model

| Component | How it runs |
|---|---|
| SDK callbacks (real robot) | background thread → `call_soon_threadsafe` → asyncio queues |
| Mujoco SHM polling (sim) | 3 daemon threads → `call_soon_threadsafe` → asyncio queues |
| A\* + raycasting | `loop.run_in_executor(None, ...)` — CPU-bound, must not block event loop |
| Rerun SDK | sync calls from async tasks — fast enough not to matter |
| FastAPI + MCP | asyncio, uvicorn |

---

## Directory structure

```
rhino/
├── pyproject.toml               # uv · entry: rhino = "rhino.main:app"
├── rhino/
│   ├── main.py                  # wires all components; nothing else does
│   ├── config.py                # nested dataclasses (RhinoConfig → MapConfig, NavConfig, …)
│   ├── storage.py               # SQLite: manual POIs (shared by api + mcp)
│   ├── platforms/
│   │   ├── base.py              # Platform protocol + CameraFrame, LidarScan, Pose, Goal, RobotStatus, POI
│   │   └── go2/
│   │       ├── robot.py         # Go2Platform — real robot via unitree-webrtc-connect-leshy
│   │       ├── sim/             # MujocoGo2 — subprocess + SHM
│   │       └── skills.py        # standup, liedown, execute_sport
│   ├── mapping/
│   │   ├── occupancy.py         # log-odds 2D grid, dynamic extent, raycasting
│   │   └── costmap.py           # obstacle inflation
│   ├── navigation/
│   │   ├── planner.py           # A* on costmap + P-controller path following
│   │   ├── controller.py        # pose error → (vx, vy, ω)
│   │   └── explorer.py          # BFS frontier detection + loop
│   ├── viz/
│   │   └── rerun.py             # RerunLogger — every rr.log() call lives here
│   └── server/
│       ├── api.py               # FastAPI: REST endpoints + legacy embedded teleop UI
│       ├── mcp.py               # McpServer (mcp SDK, tools registered in __init__)
│       └── state.py             # AppState: latest pose, camera, status
└── web/                         # React + Vite dev frontend (port 5173)
    ├── package.json
    ├── vite.config.ts           # proxies /api → localhost:8000
    └── src/
        ├── App.tsx              # root: state polling, page routing (Dashboard | Plan)
        ├── types.ts             # TypeScript interfaces matching API responses
        ├── styles.css
        └── components/
            ├── Topbar.tsx       # nav tabs, explore/mode/stop actions
            ├── RobotFleet.tsx   # left sidebar: robot cards (add robots here for bimanual)
            ├── MapPane.tsx      # occupancy map + SVG overlays + pan/zoom
            ├── RobotPanel.tsx   # right panel: camera stream, WASD teleop, sport commands
            └── PlanSidebar.tsx  # POI list + click-to-place on map
```

No `perception/` directory. POIs are manually tagged by the user — no automatic detection.

---

## Key interfaces

### Platform protocol

```python
class Platform(Protocol):
    camera_queue: asyncio.Queue[CameraFrame]   # maxsize=2, drops old frames if full
    lidar_queue:  asyncio.Queue[LidarScan]     # maxsize=4
    odom_queue:   asyncio.Queue[Pose]           # maxsize=8

    async def start(self) -> None: ...
    async def stop(self) -> None: ...
    def send_vel(self, vx: float, vy: float, omega: float) -> None: ...
    def send_cmd(self, cmd: str, **kwargs) -> None: ...
    def get_status(self) -> RobotStatus: ...
```

### POI tagging

POIs are created manually — by the user clicking on the map in the web UI, or via an MCP tool. There is no automatic detection.

```python
@dataclass
class POI:
    id: str        # UUID
    label: str
    x: float       # world frame
    y: float
    z: float       # 0.0 for floor-level; kept for Rerun 3D display
    created_at: float
```

**Web UI flow:** user clicks a point on `MapPane` canvas → canvas pixel converts to world (x, y) via the map's origin + resolution → a label dialog appears → `POST /api/pois` → saved to SQLite → SSE `poi_update` event → all clients re-render POI markers on the map. Alternatively, a "Tag here" button tags the robot's current position.

**MCP flow:** `tag_poi(label)` reads `state.latest_pose` and saves it as a POI. Useful for telling Claude "remember this spot".

POIs persist across sessions (SQLite). They are also shown in Rerun as labelled 3D points.

### `main.py` wiring

```python
async def main(cfg: RhinoConfig) -> None:
    platform = MujocoGo2(cfg.sim_cfg) if cfg.sim else Go2Platform(cfg.robot)
    await platform.start()

    storage  = Storage(cfg.storage)
    state    = AppState()
    rerun    = RerunLogger(cfg.rerun)
    mapper   = OccupancyMapper(cfg.map)
    nav      = Navigator(mapper, platform, cfg.nav)
    explorer = FrontierExplorer(mapper, nav)

    asyncio.create_task(camera_loop(platform, rerun, state))
    asyncio.create_task(lidar_loop(platform, mapper, state, rerun))
    asyncio.create_task(odom_loop(platform, nav, state, rerun))
    asyncio.create_task(nav.run())
    asyncio.create_task(explorer.run())

    api = ApiServer(state, mapper, nav, explorer, platform, storage, cfg.server)
    mcp = McpServer(platform, nav, explorer, storage, state, cfg.server)
    try:
        await asyncio.gather(api.serve(), mcp.serve())
    finally:
        await platform.stop()
```

### MCP tools

Registered as closures inside `McpServer.__init__`:

`send_velocity` · `relative_move` · `standup` · `execute_sport` · `observe` (returns latest frame as base64 JPEG) · `get_robot_status` · `navigate_to` (fires background task, returns immediately) · `get_nav_status` · `explore` · `tag_poi(label)` (saves current pose as POI) · `list_pois` · `go_to_poi(id)` (fires background nav task)

### Rerun entity paths

```
world/robot         — Transform3D (robot pose)
world/camera        — Image (BGR→RGB)
world/lidar         — Points3D
world/occupancy     — Image (grayscale grid)
world/costmap       — Image
world/path          — LineStrips3D
world/pois/{id}     — Points3D + label
```

### Web API

```
GET    /api/state               pose, status (battery, mode, is_standing, vx/vy/ω), path
GET    /api/map                 occupancy PNG with robot + path baked in (legacy teleop UI)
GET    /api/map/raw             clean occupancy PNG — used by the React frontend
GET    /api/map/info            {origin_x, origin_y, resolution, width, height}
GET    /api/camera/stream       MJPEG stream at up to 30 fps
POST   /api/navigate            {x, y, yaw?}
POST   /api/navigate/cancel     stop navigation and clear goal
GET    /api/navigate/status     {goal, exploring, mode}
POST   /api/navigate/mode       {mode: "astar" | "direct"}
POST   /api/explore/start       enable frontier exploration
POST   /api/explore/stop        disable frontier exploration
POST   /api/velocity            {vx, vy, omega}
POST   /api/stop                zero velocity + cancel navigation
POST   /api/cmd/{command}       sport command (StandUp, Dance1, FrontFlip, …)
GET    /api/pois                list saved POIs
POST   /api/pois                {label, x?, y?} — x/y defaults to current pose
DELETE /api/pois/{id}           remove POI
POST   /api/pois/{id}/navigate  navigate to POI
GET    /api/health              {"status": "ok"}
```

---

## Dependencies

```toml
unitree-webrtc-connect-leshy = ">=2.0.7"
mujoco = ">=3.3.4"
numpy = ">=1.26"
scipy = ">=1.12"
opencv-python = ">=4.9"
rerun-sdk = ">=0.20.0"
fastapi = ">=0.115"
uvicorn = ">=0.30"
sse-starlette = ">=2.0"
mcp = ">=1.0"
aiosqlite = ">=0.20"
```

No `openai`, no `open3d`, no `aiohttp`, no `dimos-lcm`.

---

## Running

**Requirements:** [uv](https://docs.astral.sh/uv/) · Node.js 18+

```bash
# Backend
uv sync
uv run rhino --sim                          # simulation (MuJoCo)
uv run rhino --robot-ip 192.168.123.161     # real Go2

# Dev frontend (separate terminal) — proxies /api → localhost:8000
cd web
npm install                                 # first time only
npm run dev                                 # http://localhost:5173
```

The legacy single-page teleop UI is still served at `http://localhost:8000` by the backend.

---

## Phases

**Phase 1 — Sim + Rerun:** `MujocoGo2` + sensor loops + `RerunLogger`. Milestone: camera, lidar, and robot pose visible in Rerun from sim.

**Phase 2 — Mapping:** `OccupancyMapper` + `Costmap`. Milestone: map builds as robot moves; steer with `platform.send_vel()` from REPL.

**Phase 3 — Navigation:** `Navigator` + `Controller` + `FrontierExplorer` + `skills.py`. Milestone: autonomous room exploration in sim; goals settable from REPL.

**Phase 4 — Web + MCP + POIs:** `AppState` + `Storage` + `FastAPI` + `McpServer` + React frontend. Milestone: live web dashboard; click map to tag POIs; navigate to them from UI or Claude Desktop.

**Phase 5 — Real robot:** `Go2Platform`. Milestone: Phase 1–4 running on real hardware; `NavConfig` tolerances tuned to observed Go2 behaviour.

---

## Adding a platform (future)

Create `platforms/<name>/robot.py` implementing `Platform` and `platforms/<name>/skills.py`. Add one branch in `main.py`. Nothing else changes.
