from dataclasses import dataclass
from multiprocessing import resource_tracker
from multiprocessing.shared_memory import SharedMemory
import pickle
from typing import Any

import numpy as np
from numpy.typing import NDArray

from rhino.platforms.go2.sim.constants import VIDEO_HEIGHT, VIDEO_WIDTH

_video_size = VIDEO_WIDTH * VIDEO_HEIGHT * 3
_odom_size = 8 * 8           # pos(3) + quat(4) + ts(1) float64
_cmd_size = 6 * 4            # linear(3) + angular(3) float32
_lidar_size = 1024 * 1024 * 4
_seq_size = 8 * 8            # 8 int64 sequence numbers
_control_size = 2 * 4        # ready_flag + stop_flag int32

_shm_sizes = {
    "video": _video_size,
    "odom": _odom_size,
    "cmd": _cmd_size,
    "lidar": _lidar_size,
    "lidar_len": 4,
    "seq": _seq_size,
    "control": _control_size,
}


def _unregister(shm: SharedMemory) -> SharedMemory:
    try:
        resource_tracker.unregister(shm._name, "shared_memory")  # type: ignore[attr-defined]
    except Exception:
        pass
    return shm


@dataclass(frozen=True)
class ShmSet:
    video: SharedMemory
    odom: SharedMemory
    cmd: SharedMemory
    lidar: SharedMemory
    lidar_len: SharedMemory
    seq: SharedMemory
    control: SharedMemory

    @classmethod
    def from_names(cls, shm_names: dict[str, str]) -> "ShmSet":
        return cls(**{k: _unregister(SharedMemory(name=shm_names[k])) for k in _shm_sizes})

    @classmethod
    def from_sizes(cls) -> "ShmSet":
        return cls(**{k: SharedMemory(create=True, size=_shm_sizes[k]) for k in _shm_sizes})

    def to_names(self) -> dict[str, str]:
        return {k: getattr(self, k).name for k in _shm_sizes}

    def as_list(self) -> list[SharedMemory]:
        return [getattr(self, k) for k in _shm_sizes]


class ShmReader:
    """Used by the subprocess to write data and read commands."""

    def __init__(self, shm_names: dict[str, str]) -> None:
        self.shm = ShmSet.from_names(shm_names)
        self._last_cmd_seq: int = 0

    def signal_ready(self) -> None:
        ctrl: NDArray[Any] = np.ndarray((2,), dtype=np.int32, buffer=self.shm.control.buf)
        ctrl[0] = 1

    def should_stop(self) -> bool:
        ctrl: NDArray[Any] = np.ndarray((2,), dtype=np.int32, buffer=self.shm.control.buf)
        return bool(ctrl[1] == 1)

    def write_video(self, pixels: NDArray[Any]) -> None:
        arr: NDArray[Any] = np.ndarray(
            (VIDEO_HEIGHT, VIDEO_WIDTH, 3), dtype=np.uint8, buffer=self.shm.video.buf
        )
        arr[:] = pixels
        self._increment_seq(0)

    def write_odom(self, pos: NDArray[Any], quat: NDArray[Any], ts: float) -> None:
        arr: NDArray[Any] = np.ndarray((8,), dtype=np.float64, buffer=self.shm.odom.buf)
        arr[0:3] = pos
        arr[3:7] = quat
        arr[7] = ts
        self._increment_seq(2)

    def write_lidar(self, points: NDArray[Any]) -> None:
        data = pickle.dumps(points)
        n = len(data)
        if n > self.shm.lidar.size:
            return
        len_arr: NDArray[Any] = np.ndarray((1,), dtype=np.uint32, buffer=self.shm.lidar_len.buf)
        len_arr[0] = n
        buf: NDArray[Any] = np.ndarray((n,), dtype=np.uint8, buffer=self.shm.lidar.buf)
        buf[:] = np.frombuffer(data, dtype=np.uint8)
        self._increment_seq(4)

    def read_command(self) -> tuple[NDArray[Any], NDArray[Any]] | None:
        seq = self._get_seq(3)
        if seq > self._last_cmd_seq:
            self._last_cmd_seq = seq
            arr: NDArray[Any] = np.ndarray((6,), dtype=np.float32, buffer=self.shm.cmd.buf)
            return arr[0:3].copy(), arr[3:6].copy()
        return None

    def cleanup(self) -> None:
        for shm in self.shm.as_list():
            try:
                shm.close()
            except Exception:
                pass

    def _increment_seq(self, index: int) -> None:
        seq: NDArray[Any] = np.ndarray((8,), dtype=np.int64, buffer=self.shm.seq.buf)
        seq[index] += 1

    def _get_seq(self, index: int) -> int:
        seq: NDArray[Any] = np.ndarray((8,), dtype=np.int64, buffer=self.shm.seq.buf)
        return int(seq[index])


class ShmWriter:
    """Used by the host process to read data and write commands."""

    def __init__(self) -> None:
        self.shm = ShmSet.from_sizes()
        seq: NDArray[Any] = np.ndarray((8,), dtype=np.int64, buffer=self.shm.seq.buf)
        seq[:] = 0
        cmd: NDArray[Any] = np.ndarray((6,), dtype=np.float32, buffer=self.shm.cmd.buf)
        cmd[:] = 0
        ctrl: NDArray[Any] = np.ndarray((2,), dtype=np.int32, buffer=self.shm.control.buf)
        ctrl[:] = 0

    def is_ready(self) -> bool:
        ctrl: NDArray[Any] = np.ndarray((2,), dtype=np.int32, buffer=self.shm.control.buf)
        return bool(ctrl[0] == 1)

    def signal_stop(self) -> None:
        ctrl: NDArray[Any] = np.ndarray((2,), dtype=np.int32, buffer=self.shm.control.buf)
        ctrl[1] = 1

    def read_video(self) -> tuple[NDArray[Any] | None, int]:
        seq = self._get_seq(0)
        if seq > 0:
            arr: NDArray[Any] = np.ndarray(
                (VIDEO_HEIGHT, VIDEO_WIDTH, 3), dtype=np.uint8, buffer=self.shm.video.buf
            )
            return arr.copy(), seq
        return None, 0

    def read_odom(self) -> tuple[tuple[NDArray[Any], NDArray[Any], float] | None, int]:
        seq = self._get_seq(2)
        if seq > 0:
            arr: NDArray[Any] = np.ndarray((8,), dtype=np.float64, buffer=self.shm.odom.buf)
            return (arr[0:3].copy(), arr[3:7].copy(), float(arr[7])), seq
        return None, 0

    def read_lidar(self) -> tuple[NDArray[Any] | None, int]:
        seq = self._get_seq(4)
        if seq > 0:
            len_arr: NDArray[Any] = np.ndarray((1,), dtype=np.uint32, buffer=self.shm.lidar_len.buf)
            n = int(len_arr[0])
            if 0 < n <= self.shm.lidar.size:
                buf: NDArray[Any] = np.ndarray((n,), dtype=np.uint8, buffer=self.shm.lidar.buf)
                try:
                    points: NDArray[Any] = pickle.loads(bytes(buf))
                    return points, seq
                except Exception:
                    pass
        return None, 0

    def write_command(self, linear: NDArray[Any], angular: NDArray[Any]) -> None:
        arr: NDArray[Any] = np.ndarray((6,), dtype=np.float32, buffer=self.shm.cmd.buf)
        arr[0:3] = linear
        arr[3:6] = angular
        self._increment_seq(3)

    def cleanup(self) -> None:
        for shm in self.shm.as_list():
            try:
                shm.unlink()
            except Exception:
                pass
            try:
                shm.close()
            except Exception:
                pass

    def _increment_seq(self, index: int) -> None:
        seq: NDArray[Any] = np.ndarray((8,), dtype=np.int64, buffer=self.shm.seq.buf)
        seq[index] += 1

    def _get_seq(self, index: int) -> int:
        seq: NDArray[Any] = np.ndarray((8,), dtype=np.int64, buffer=self.shm.seq.buf)
        return int(seq[index])
