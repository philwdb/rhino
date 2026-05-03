"""Real Go2 platform via unitree-webrtc-connect-leshy."""

from __future__ import annotations

import asyncio
import math
import time
from typing import Any

import cv2
import numpy as np

from rhino.config import RobotConfig
from rhino.platforms.base import CameraFrame, LidarScan, Pose, RobotStatus


class Go2Platform:
    def __init__(self, cfg: RobotConfig) -> None:
        self._cfg = cfg
        self.camera_queue: asyncio.Queue[CameraFrame] = asyncio.Queue(maxsize=2)
        self.lidar_queue: asyncio.Queue[LidarScan] = asyncio.Queue(maxsize=4)
        self.odom_queue: asyncio.Queue[Pose] = asyncio.Queue(maxsize=8)
        self._status = RobotStatus(battery_pct=0.0, mode="robot")
        self._conn: Any = None
        self._RTC_TOPIC: dict[str, str] = {}
        self._SPORT_CMD: dict[str, int] = {}

    async def start(self) -> None:
        import unitree_webrtc_connect  # noqa: F401 — applies monkey-patches
        from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD, WebRTCConnectionMethod
        from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection

        self._RTC_TOPIC = RTC_TOPIC
        self._SPORT_CMD = SPORT_CMD

        self._conn = UnitreeWebRTCConnection(
            WebRTCConnectionMethod.LocalSTA,
            ip=self._cfg.ip,
        )

        # --- video track callback ---
        async def on_video_track(track: Any) -> None:
            while True:
                try:
                    frame = await asyncio.wait_for(track.recv(), timeout=2.0)
                    img = frame.to_ndarray(format="bgr24")
                    _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    self._enqueue(
                        self.camera_queue,
                        CameraFrame(
                            data=buf.tobytes(),
                            width=img.shape[1],
                            height=img.shape[0],
                            timestamp=time.time(),
                        ),
                    )
                except asyncio.TimeoutError:
                    continue
                except Exception:
                    break

        await self._conn.connect()

        # video attribute is created inside connect() → add callback after
        self._conn.video.add_track_callback(on_video_track)

        # Switch to native (lz4+numpy) decoder so lidar messages have {"points": ndarray}.
        # The default libvoxel decoder returns mesh render data (positions/uvs/indices), not points.
        self._conn.datachannel.set_decoder("native")

        # Enable full lidar stream (disables bandwidth-saving mode).
        await self._conn.datachannel.disableTrafficSaving(True)

        ps = self._conn.datachannel.pub_sub

        _lidar_count = 0
        _odom_count = 0

        # --- lidar subscription ---
        def on_lidar(msg: dict[str, Any]) -> None:
            nonlocal _lidar_count
            _lidar_count += 1
            try:
                points: np.ndarray = msg["data"]["data"]["points"]
                if _lidar_count == 1:
                    print(f"[go2] first lidar msg: {points.shape} points, "
                          f"range x=[{points[:,0].min():.2f},{points[:,0].max():.2f}] "
                          f"y=[{points[:,1].min():.2f},{points[:,1].max():.2f}]")
                self._enqueue(
                    self.lidar_queue,
                    LidarScan(points=points.astype(np.float32), timestamp=time.time()),
                )
            except Exception as e:
                if _lidar_count <= 3:
                    print(f"[go2] lidar parse error (msg #{_lidar_count}): {e}")
                    print(f"[go2] lidar msg keys: {list(msg.keys())}")
                    if "data" in msg:
                        print(f"[go2] lidar msg['data'] keys: {list(msg['data'].keys()) if isinstance(msg['data'], dict) else type(msg['data'])}")

        # --- odometry subscription ---
        def on_odom(msg: dict[str, Any]) -> None:
            nonlocal _odom_count
            _odom_count += 1
            try:
                pose_data = msg["data"]["pose"]
                x: float = pose_data["position"]["x"]
                y: float = pose_data["position"]["y"]
                ox: float = pose_data["orientation"]["x"]
                oy: float = pose_data["orientation"]["y"]
                oz: float = pose_data["orientation"]["z"]
                ow: float = pose_data["orientation"]["w"]
                yaw = math.atan2(2.0 * (ow * oz + ox * oy), 1.0 - 2.0 * (oy * oy + oz * oz))
                if _odom_count == 1:
                    print(f"[go2] first odom: x={x:.3f} y={y:.3f} yaw={yaw:.3f}")
                self._enqueue(
                    self.odom_queue,
                    Pose(x=x, y=y, yaw=yaw, timestamp=time.time()),
                )
            except Exception as e:
                if _odom_count <= 3:
                    print(f"[go2] odom parse error (msg #{_odom_count}): {e}")
                    print(f"[go2] odom msg keys: {list(msg.keys())}")

        # --- low-state subscription (battery + basic status) ---
        def on_lowstate(msg: dict[str, Any]) -> None:
            try:
                data = msg["data"]
                soc = float(data["bms_state"]["soc"])
                self._status = RobotStatus(battery_pct=soc, mode="robot")
            except Exception:
                pass

        ps.subscribe(RTC_TOPIC["ULIDAR_ARRAY"], on_lidar)
        ps.subscribe(RTC_TOPIC["ROBOTODOM"], on_odom)
        ps.subscribe(RTC_TOPIC["LOW_STATE"], on_lowstate)

        self._conn.video.switchVideoChannel(True)

        # Stand up if robot is in StandDown from a previous session.
        # Must use publish_request_new (awaited) so the robot properly acks the
        # sport command before we start sending velocity commands.
        try:
            await asyncio.wait_for(
                ps.publish_request_new(
                    RTC_TOPIC["SPORT_MOD"],
                    {"api_id": SPORT_CMD["RecoveryStand"]},
                ),
                timeout=5.0,
            )
            await asyncio.sleep(2.0)  # let the stand-up motion complete
        except Exception:
            pass  # already standing — RecoveryStand is a no-op in that case

    async def stop(self) -> None:
        if self._conn is not None:
            try:
                ps = self._conn.datachannel.pub_sub
                ps.publish_without_callback(
                    self._RTC_TOPIC["WIRELESS_CONTROLLER"],
                    data={"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0},
                )
                await asyncio.wait_for(
                    ps.publish_request_new(
                        self._RTC_TOPIC["SPORT_MOD"],
                        {"api_id": self._SPORT_CMD["StandDown"]},
                    ),
                    timeout=3.0,
                )
                await asyncio.sleep(2.0)  # let the sit motion complete
            except Exception:
                pass
            await self._conn.disconnect()
            self._conn = None

    def send_vel(self, vx: float, vy: float, omega: float) -> None:
        # WIRELESS_CONTROLLER axes: ly=forward, lx=strafe, rx=yaw
        self._conn.datachannel.pub_sub.publish_without_callback(
            self._RTC_TOPIC["WIRELESS_CONTROLLER"],
            data={"lx": -vy, "ly": vx, "rx": -omega, "ry": 0.0},
        )

    def send_cmd(self, cmd: str, **kwargs: Any) -> None:
        api_id = self._SPORT_CMD.get(cmd)
        if api_id is None:
            raise ValueError(f"Unknown sport command: {cmd!r}")
        options: dict[str, Any] = {"api_id": api_id}
        if kwargs:
            options["parameter"] = kwargs
        asyncio.ensure_future(
            self._conn.datachannel.pub_sub.publish_request_new(
                self._RTC_TOPIC["SPORT_MOD"],
                options,
            )
        )

    def get_status(self) -> RobotStatus:
        return self._status

    @staticmethod
    def _enqueue(q: asyncio.Queue, item: Any) -> None:
        if q.full():
            try:
                q.get_nowait()
            except Exception:
                pass
        try:
            q.put_nowait(item)
        except Exception:
            pass
