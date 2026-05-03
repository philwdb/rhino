"""MujocoGo2 — Platform implementation backed by a MuJoCo subprocess via SHM."""

from __future__ import annotations

import asyncio
import atexit
import base64
import json
import math
import os
import pickle
import subprocess
import sys
import threading
import time
import weakref
from typing import Any

import cv2
import numpy as np
from numpy.typing import NDArray

from rhino.platforms.base import CameraFrame, LidarScan, Pose, RobotStatus
from rhino.platforms.go2.sim.constants import (
    LAUNCHER_PATH,
    LIDAR_FPS,
    ODOM_FREQUENCY,
    VIDEO_FPS,
)
from rhino.platforms.go2.sim.mujoco_process import SimProcessConfig
from rhino.platforms.go2.sim.shared_memory import ShmWriter
from rhino.config import SimConfig


def _quat_to_yaw(w: float, x: float, y: float, z: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class MujocoGo2:
    def __init__(self, cfg: SimConfig) -> None:
        self._cfg = cfg
        self._shm: ShmWriter | None = None
        self._process: subprocess.Popen[bytes] | None = None
        self._threads: list[threading.Thread] = []
        self._stop_events: list[threading.Event] = []
        self._cleaned_up = False
        self._last_video_seq = 0
        self._last_odom_seq = 0
        self._last_lidar_seq = 0
        self._loop: asyncio.AbstractEventLoop | None = None
        self._status = RobotStatus(is_standing=True, mode="sim")

        self.camera_queue: asyncio.Queue[CameraFrame] = asyncio.Queue(maxsize=2)
        self.lidar_queue: asyncio.Queue[LidarScan] = asyncio.Queue(maxsize=4)
        self.odom_queue: asyncio.Queue[Pose] = asyncio.Queue(maxsize=8)

    async def start(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._shm = ShmWriter()

        proc_cfg = SimProcessConfig(
            viewer=self._cfg.viewer,
            start_pos=self._cfg.start_pos,
        )
        cfg_b64 = base64.b64encode(pickle.dumps(proc_cfg)).decode("ascii")
        shm_json = json.dumps(self._shm.shm.to_names())

        env = os.environ.copy()
        if self._cfg.viewer == "none" and sys.platform.startswith("linux"):
            env.setdefault("MUJOCO_GL", "egl")

        try:
            self._process = subprocess.Popen(
                [sys.executable, str(LAUNCHER_PATH), cfg_b64, shm_json],
                env=env,
            )
        except Exception as exc:
            self._shm.cleanup()
            raise RuntimeError(f"Failed to start MuJoCo subprocess: {exc}") from exc

        deadline = time.monotonic() + 120.0
        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                self._shm.cleanup()
                raise RuntimeError(
                    f"MuJoCo process exited early (code {self._process.returncode})"
                )
            if self._shm.is_ready():
                break
            await asyncio.sleep(0.1)
        else:
            self._shm.cleanup()
            raise RuntimeError("MuJoCo process startup timeout")

        weak_self = weakref.ref(self)

        def _atexit(ref: weakref.ref[MujocoGo2] = weak_self) -> None:
            inst = ref()
            if inst is not None:
                asyncio.run(inst.stop()) if not inst._cleaned_up else None

        atexit.register(_atexit)

        self._start_polling_threads()

    async def stop(self) -> None:
        if self._cleaned_up:
            return
        self._cleaned_up = True

        for ev in self._stop_events:
            ev.set()
        for t in self._threads:
            t.join(timeout=2.0)

        if self._shm:
            self._shm.signal_stop()

        if self._process:
            try:
                self._process.terminate()
                self._process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=2)
            except Exception:
                pass
            self._process = None

        if self._shm:
            self._shm.cleanup()
            self._shm = None

        self._threads.clear()
        self._stop_events.clear()

    def send_vel(self, vx: float, vy: float, omega: float) -> None:
        if self._cleaned_up or self._shm is None:
            return
        linear = np.array([vx, vy, 0.0], dtype=np.float32)
        angular = np.array([0.0, 0.0, omega], dtype=np.float32)
        self._shm.write_command(linear, angular)
        self._status.vx = vx
        self._status.vy = vy
        self._status.omega = omega

    def send_cmd(self, cmd: str, **kwargs: Any) -> None:
        pass  # no-op in sim; standup/liedown handled by physics

    def get_status(self) -> RobotStatus:
        return self._status

    def _start_polling_threads(self) -> None:
        assert self._loop is not None

        def _poll_video(stop: threading.Event) -> None:
            interval = 1.0 / VIDEO_FPS
            while not stop.is_set():
                frame, seq = self._shm.read_video() if self._shm else (None, 0)
                if frame is not None and seq > self._last_video_seq:
                    self._last_video_seq = seq
                    cf = _to_camera_frame(frame)
                    if cf is not None:
                        self._enqueue(self.camera_queue, cf)
                time.sleep(interval)

        def _poll_odom(stop: threading.Event) -> None:
            interval = 1.0 / ODOM_FREQUENCY
            while not stop.is_set():
                odom, seq = self._shm.read_odom() if self._shm else (None, 0)
                if odom is not None and seq > self._last_odom_seq:
                    self._last_odom_seq = seq
                    pos, quat_wxyz, ts = odom
                    w, x, y, z = quat_wxyz
                    pose = Pose(
                        x=float(pos[0]),
                        y=float(pos[1]),
                        yaw=_quat_to_yaw(float(w), float(x), float(y), float(z)),
                        timestamp=ts,
                    )
                    self._enqueue(self.odom_queue, pose)
                time.sleep(interval)

        def _poll_lidar(stop: threading.Event) -> None:
            interval = 1.0 / LIDAR_FPS
            while not stop.is_set():
                pts, seq = self._shm.read_lidar() if self._shm else (None, 0)
                if pts is not None and seq > self._last_lidar_seq:
                    self._last_lidar_seq = seq
                    scan = LidarScan(points=pts, timestamp=time.time())
                    self._enqueue(self.lidar_queue, scan)
                time.sleep(interval)

        for target in (_poll_video, _poll_odom, _poll_lidar):
            ev = threading.Event()
            self._stop_events.append(ev)
            t = threading.Thread(target=target, args=(ev,), daemon=True)
            self._threads.append(t)
            t.start()

    def _enqueue(self, q: asyncio.Queue[Any], item: Any) -> None:
        # All queue operations must run on the event loop thread.
        assert self._loop is not None

        def _put() -> None:
            import queue as _q
            if q.full():
                try:
                    q.get_nowait()
                except _q.Empty:
                    pass
            q.put_nowait(item)

        self._loop.call_soon_threadsafe(_put)


def _to_camera_frame(rgb: NDArray[Any]) -> CameraFrame | None:
    try:
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return None
        h, w = rgb.shape[:2]
        return CameraFrame(data=buf.tobytes(), width=w, height=h, timestamp=time.time())
    except Exception:
        return None
