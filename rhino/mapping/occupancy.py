"""2D log-odds occupancy grid with raycasting."""

from __future__ import annotations

import math
import threading

import numpy as np
from numpy.typing import NDArray

from rhino.config import MapConfig
from rhino.platforms.base import LidarScan, Pose

# Only treat lidar returns in this z-band as obstacles (ignores floor & ceiling).
_OBS_Z_MIN = 0.08
_OBS_Z_MAX = 1.5


class OccupancyMapper:
    def __init__(self, cfg: MapConfig) -> None:
        self._cfg = cfg
        # Internal: log-odds floats.  0.0 = unknown (p=0.5).
        self._grid: NDArray[np.float32] = np.zeros(
            (cfg.height, cfg.width), dtype=np.float32
        )
        self._origin_x = -cfg.width * cfg.resolution / 2.0
        self._origin_y = -cfg.height * cfg.resolution / 2.0
        # Precompute log-odds increments from probability config values.
        self._lo_hit = math.log(cfg.log_odds_hit / (1.0 - cfg.log_odds_hit))
        self._lo_miss = math.log(cfg.log_odds_miss / (1.0 - cfg.log_odds_miss))
        self._lo_min = math.log(cfg.log_odds_min / (1.0 - cfg.log_odds_min))
        self._lo_max = math.log(cfg.log_odds_max / (1.0 - cfg.log_odds_max))
        self._lock = threading.Lock()

    def update(self, scan: LidarScan, pose: Pose) -> None:
        if scan.points.shape[0] == 0:
            return

        H, W = self._grid.shape
        res = self._cfg.resolution

        # Keep only points in the obstacle height band.
        z = scan.points[:, 2]
        pts = scan.points[(z >= _OBS_Z_MIN) & (z <= _OBS_Z_MAX)]
        if pts.shape[0] == 0:
            return

        # Robot cell — skip update if robot is off the map.
        r0 = int((pose.y - self._origin_y) / res)
        c0 = int((pose.x - self._origin_x) / res)
        if not (0 <= r0 < H and 0 <= c0 < W):
            return

        end_c = ((pts[:, 0] - self._origin_x) / res).astype(np.int32)
        end_r = ((pts[:, 1] - self._origin_y) / res).astype(np.int32)

        free_r: list[NDArray[np.int32]] = []
        free_c: list[NDArray[np.int32]] = []
        hit_r: list[int] = []
        hit_c: list[int] = []

        for er, ec in zip(end_r, end_c):
            n = max(abs(int(er) - r0), abs(int(ec) - c0))
            if n == 0:
                continue

            t = np.linspace(0.0, 1.0, n + 1, dtype=np.float32)
            rs = (r0 + t * (er - r0)).astype(np.int32)
            cs = (c0 + t * (ec - c0)).astype(np.int32)

            # All ray cells except the endpoint → free update.
            rr = rs[:-1]
            rc = cs[:-1]
            ok = (rr >= 0) & (rr < H) & (rc >= 0) & (rc < W)
            if ok.any():
                free_r.append(rr[ok])
                free_c.append(rc[ok])

            # Endpoint → occupied update.
            if 0 <= er < H and 0 <= ec < W:
                hit_r.append(int(er))
                hit_c.append(int(ec))

        with self._lock:
            if free_r:
                fr = np.concatenate(free_r)
                fc = np.concatenate(free_c)
                np.add.at(self._grid, (fr, fc), self._lo_miss)

            if hit_r:
                hr = np.array(hit_r, dtype=np.int32)
                hc = np.array(hit_c, dtype=np.int32)
                np.add.at(self._grid, (hr, hc), self._lo_hit)

            np.clip(self._grid, self._lo_min, self._lo_max, out=self._grid)

    def get_grid(self) -> NDArray[np.float32]:
        """Occupancy probability grid: 0=free, 0.5=unknown, 1=occupied."""
        with self._lock:
            return (1.0 / (1.0 + np.exp(-self._grid))).astype(np.float32)

    @property
    def resolution(self) -> float:
        return self._cfg.resolution

    @property
    def origin(self) -> tuple[float, float]:
        """World-frame (x, y) of grid cell (row=0, col=0)."""
        return self._origin_x, self._origin_y

    @property
    def shape(self) -> tuple[int, int]:
        return (self._cfg.height, self._cfg.width)
