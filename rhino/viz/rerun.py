"""Rerun logger — all rr.log() calls live here."""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray
import rerun as rr
import rerun.blueprint as rrb

from rhino.config import RerunConfig
from rhino.platforms.base import CameraFrame, LidarScan, Pose


class RerunLogger:
    def __init__(self, cfg: RerunConfig) -> None:
        rr.init(cfg.app_id, spawn=not cfg.connect)
        if cfg.connect:
            rr.connect()
        rr.send_blueprint(
            rrb.Blueprint(
                rrb.Horizontal(
                    rrb.Spatial3DView(name="World", origin="/world"),
                    rrb.Vertical(
                        rrb.Spatial2DView(name="Map", origin="/map"),
                        rrb.Spatial2DView(name="Robot View", origin="/camera"),
                    ),
                ),
                auto_views=False,
            )
        )

    def log_camera(self, frame: CameraFrame) -> None:
        import cv2
        buf = np.frombuffer(frame.data, dtype=np.uint8)
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if bgr is None:
            return
        rr.log("camera/pov", rr.Image(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)))

    def log_lidar(self, scan: LidarScan) -> None:
        if scan.points.shape[0] > 0:
            rr.log("world/lidar", rr.Points3D(scan.points))

    def log_pose(self, pose: Pose) -> None:
        quat_w = math.cos(pose.yaw / 2)
        quat_z = math.sin(pose.yaw / 2)

        rr.log(
            "world/robot",
            rr.Transform3D(
                translation=[pose.x, pose.y, 0.35],
                rotation=rr.Quaternion(xyzw=[0.0, 0.0, quat_z, quat_w]),
            ),
        )
        rr.log(
            "world/robot/body",
            rr.Boxes3D(
                centers=[[0.0, 0.0, 0.0]],
                half_sizes=[[0.325, 0.14, 0.10]],
                colors=[[60, 180, 100, 200]],
            ),
        )
        rr.log(
            "world/robot/heading",
            rr.Arrows3D(
                origins=[[0.0, 0.0, 0.1]],
                vectors=[[0.5, 0.0, 0.0]],
                colors=[[255, 120, 0, 255]],
            ),
        )

    def log_map(
        self,
        grid: NDArray[np.float32],
        origin: tuple[float, float],
        resolution: float,
    ) -> None:
        # Greyscale: free→white(255), unknown→grey(128), occupied→black(0).
        img = (255.0 * (1.0 - grid)).astype(np.uint8)
        # Flip rows so that map north (max y) is at the top of the image.
        rr.log("map/occupancy", rr.Image(img[::-1, :]))
