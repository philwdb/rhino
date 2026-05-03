#!/usr/bin/env python3
"""MuJoCo subprocess entry point — launched by MujocoGo2.start().

Receives a pickle+base64 SimProcessConfig and a JSON dict of SHM names.
Runs the physics loop, writing video/odom/lidar to SHM and reading commands.
No dimos dependencies.
"""

import base64
import json
import pickle
import signal
import sys
import time
import xml.etree.ElementTree as ET
from contextlib import AbstractContextManager
from dataclasses import dataclass
from typing import Any

import mujoco
from mujoco import viewer
import numpy as np
from numpy.typing import NDArray

from rhino.platforms.go2.sim.constants import (
    ASSETS_PATH,
    DEPTH_CAMERA_FOV,
    LAUNCHER_PATH,
    LIDAR_FPS,
    LIDAR_VOXEL_SIZE,
    VIDEO_FPS,
    VIDEO_HEIGHT,
    VIDEO_WIDTH,
)
from rhino.platforms.go2.sim.depth_camera import depth_image_to_point_cloud, voxel_downsample
from rhino.platforms.go2.sim.policy import Go1OnnxController
from rhino.platforms.go2.sim.shared_memory import ShmReader


@dataclass
class SimProcessConfig:
    viewer: str = "none"
    start_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)


def _get_assets() -> dict[str, bytes]:
    assets: dict[str, bytes] = {}
    for f in ASSETS_PATH.iterdir():
        if f.suffix in {".xml", ".stl", ".obj", ".png"}:
            assets[f.name] = f.read_bytes()
    return assets


def _build_model_xml() -> str:
    scene_xml = (ASSETS_PATH / "scene_empty.xml").read_text()
    root = ET.fromstring(scene_xml)
    root.set("model", "unitree_go1_scene")
    root.insert(0, ET.Element("include", file="unitree_go1.xml"))
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    map_elem = visual.find("map")
    if map_elem is None:
        map_elem = ET.SubElement(visual, "map")
    map_elem.set("znear", "0.01")
    map_elem.set("zfar", "10000")
    return ET.tostring(root, encoding="unicode")


class _HeadlessViewer(AbstractContextManager["_HeadlessViewer"]):
    def __enter__(self) -> "_HeadlessViewer":
        return self

    def __exit__(self, *_: Any) -> None:
        pass

    def is_running(self) -> bool:
        return True

    def sync(self) -> None:
        pass


def _run(cfg: SimProcessConfig, shm: ShmReader) -> None:
    xml_string = _build_model_xml()
    assets = _get_assets()

    mujoco.set_mjcb_control(None)
    model = mujoco.MjModel.from_xml_string(xml_string, assets=assets)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)

    sim_dt = 0.005
    ctrl_dt = 0.02
    n_substeps = round(ctrl_dt / sim_dt)
    model.opt.timestep = sim_dt

    default_angles: NDArray[Any] = np.array(model.keyframe("home").qpos[7:])
    policy_path = str(ASSETS_PATH / "unitree_go1_policy.onnx")
    policy = Go1OnnxController(
        policy_path=policy_path,
        default_angles=default_angles,
        n_substeps=n_substeps,
        action_scale=0.5,
        ctrl_dt=ctrl_dt,
    )
    mujoco.set_mjcb_control(policy.get_control)

    x0, y0, _ = cfg.start_pos
    data.qpos[0:3] = [x0, y0, 0.35]
    mujoco.mj_forward(model, data)

    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "track")
    lid_front = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "lidar_front_camera")
    lid_left = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "lidar_left_camera")
    lid_right = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "lidar_right_camera")

    shm.signal_ready()

    ctx: AbstractContextManager[Any]
    if cfg.viewer == "none":
        ctx = _HeadlessViewer()
    else:
        ctx = viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)

    size = (VIDEO_WIDTH, VIDEO_HEIGHT)
    rgb_r = mujoco.Renderer(model, height=size[1], width=size[0])
    d_front = mujoco.Renderer(model, height=size[1], width=size[0])
    d_left = mujoco.Renderer(model, height=size[1], width=size[0])
    d_right = mujoco.Renderer(model, height=size[1], width=size[0])
    for r in (d_front, d_left, d_right):
        r.enable_depth_rendering()

    scene_opt = mujoco.MjvOption()
    last_video = 0.0
    last_lidar = 0.0
    video_interval = 1.0 / VIDEO_FPS
    lidar_interval = 1.0 / LIDAR_FPS

    with ctx as v:
        while v.is_running() and not shm.should_stop():
            step_start = time.time()

            cmd = shm.read_command()
            if cmd is not None:
                linear, angular = cmd
                policy.set_command(linear[0], linear[1], angular[2])

            for _ in range(n_substeps):
                mujoco.mj_step(model, data)

            v.sync()

            pos: NDArray[Any] = data.qpos[0:3].copy()
            quat: NDArray[Any] = data.qpos[3:7].copy()
            shm.write_odom(pos, quat, time.time())

            now = time.time()

            if now - last_video >= video_interval:
                rgb_r.update_scene(data, camera=cam_id, scene_option=scene_opt)
                shm.write_video(rgb_r.render())
                last_video = now

            if now - last_lidar >= lidar_interval:
                all_pts: list[NDArray[Any]] = []
                for renderer, cam_id_l in [
                    (d_front, lid_front),
                    (d_left, lid_left),
                    (d_right, lid_right),
                ]:
                    renderer.update_scene(data, camera=cam_id_l, scene_option=scene_opt)
                    depth = renderer.render()
                    cam_pos: NDArray[Any] = data.cam_xpos[cam_id_l].copy()
                    cam_mat: NDArray[Any] = data.cam_xmat[cam_id_l].reshape(3, 3).copy()
                    pts = depth_image_to_point_cloud(depth, cam_pos, cam_mat, DEPTH_CAMERA_FOV)
                    if pts.shape[0] > 0:
                        all_pts.append(pts)

                if all_pts:
                    combined = np.vstack(all_pts)
                    combined = voxel_downsample(combined, LIDAR_VOXEL_SIZE)
                    shm.write_lidar(combined)

                last_lidar = now

            wait = model.opt.timestep - (time.time() - step_start)
            if wait > 0:
                time.sleep(wait)


if __name__ == "__main__":
    cfg: SimProcessConfig = pickle.loads(base64.b64decode(sys.argv[1]))
    shm_names: dict[str, str] = json.loads(sys.argv[2])

    shm = ShmReader(shm_names)

    def _handle_signal(_sig: int, _frame: Any) -> None:
        ctrl = np.ndarray((2,), dtype=np.int32, buffer=shm.shm.control.buf)
        ctrl[1] = 1

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        _run(cfg, shm)
    finally:
        shm.cleanup()
