import asyncio
from typing import Optional

import typer

from rhino.config import (
    MapConfig,
    NavConfig,
    RerunConfig,
    RhinoConfig,
    RobotConfig,
    ServerConfig,
    SimConfig,
    StorageConfig,
)
from rhino.mapping.occupancy import OccupancyMapper
from rhino.navigation.explorer import FrontierExplorer
from rhino.navigation.planner import Navigator
from rhino.platforms.base import CameraFrame, LidarScan, Platform, Pose
from rhino.server.api import ApiServer
from rhino.server.mcp import McpServer
from rhino.server.state import AppState
from rhino.storage import Storage
from rhino.viz.rerun import RerunLogger

app = typer.Typer(pretty_exceptions_show_locals=False)


async def _camera_loop(platform: Platform, rerun: RerunLogger, state: AppState) -> None:
    while True:
        frame: CameraFrame = await platform.camera_queue.get()
        state.latest_camera = frame
        rerun.log_camera(frame)


async def _lidar_loop(
    platform: Platform,
    mapper: OccupancyMapper,
    state: AppState,
    rerun: RerunLogger,
    loop: asyncio.AbstractEventLoop,
) -> None:
    executor = None
    while True:
        scan: LidarScan = await platform.lidar_queue.get()
        pose = state.latest_pose
        if pose is not None:
            await loop.run_in_executor(executor, mapper.update, scan, pose)
        rerun.log_lidar(scan)


async def _odom_loop(
    platform: Platform,
    nav: Navigator,
    state: AppState,
    rerun: RerunLogger,
) -> None:
    while True:
        pose: Pose = await platform.odom_queue.get()
        state.latest_pose = pose
        nav.update_pose(pose)
        rerun.log_pose(pose)


async def _run(cfg: RhinoConfig) -> None:
    if cfg.sim:
        from rhino.platforms.go2.sim.sim import MujocoGo2
        platform: Platform = MujocoGo2(cfg.sim_cfg)
    else:
        from rhino.platforms.go2.robot import Go2Platform
        platform = Go2Platform(cfg.robot)

    await platform.start()

    storage = Storage(cfg.storage)
    await storage.init()

    state = AppState()
    rerun = RerunLogger(cfg.rerun)
    mapper = OccupancyMapper(cfg.map)
    nav = Navigator(mapper, platform, cfg.nav)
    explorer = FrontierExplorer(mapper, nav)

    loop = asyncio.get_running_loop()
    asyncio.create_task(_camera_loop(platform, rerun, state))
    asyncio.create_task(_lidar_loop(platform, mapper, state, rerun, loop))
    asyncio.create_task(_odom_loop(platform, nav, state, rerun))
    asyncio.create_task(nav.run())
    asyncio.create_task(explorer.run())

    api = ApiServer(state, mapper, nav, explorer, platform, storage, cfg.server)
    mcp = McpServer(platform, nav, explorer, storage, state, cfg.server)
    try:
        await asyncio.gather(api.serve(), mcp.serve())
    finally:
        await platform.stop()
        await storage.close()


@app.command()
def cli(
    sim: bool = typer.Option(False, "--sim", help="Run in simulation mode"),
    robot_ip: Optional[str] = typer.Option(None, "--robot-ip", help="Go2 IP address"),
    viewer: str = typer.Option("none", "--viewer", help="MuJoCo viewer: none | passive"),
    port: int = typer.Option(8000, "--port", help="API server port"),
    mcp_port: int = typer.Option(8001, "--mcp-port", help="MCP server port"),
) -> None:
    if not sim and robot_ip is None:
        typer.echo("Error: provide --sim or --robot-ip", err=True)
        raise typer.Exit(1)

    cfg = RhinoConfig(
        sim=sim,
        sim_cfg=SimConfig(viewer=viewer),
        robot=RobotConfig(ip=robot_ip or "192.168.123.161"),
        map=MapConfig(),
        nav=NavConfig(),
        server=ServerConfig(port=port, mcp_port=mcp_port),
        storage=StorageConfig(),
        rerun=RerunConfig(),
    )
    asyncio.run(_run(cfg))


if __name__ == "__main__":
    app()
