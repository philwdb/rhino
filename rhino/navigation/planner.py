"""A* navigator with P-controller path following."""

from __future__ import annotations

import asyncio
import heapq
import math
from collections import deque

import numpy as np
from numpy.typing import NDArray

from rhino.config import NavConfig
from rhino.mapping.costmap import Costmap
from rhino.mapping.occupancy import OccupancyMapper
from rhino.navigation.controller import compute_velocity
from rhino.platforms.base import Goal, Platform, Pose


class Navigator:
    def __init__(
        self,
        mapper: OccupancyMapper,
        costmap: Costmap,
        platform: Platform,
        cfg: NavConfig,
    ) -> None:
        self._mapper = mapper
        self._costmap = costmap
        self._platform = platform
        self._cfg = cfg
        self._goal: Goal | None = None
        self._pose: Pose | None = None

    def update_pose(self, pose: Pose) -> None:
        self._pose = pose

    def set_goal(self, goal: Goal) -> None:
        self._goal = goal

    def clear_goal(self) -> None:
        self._goal = None
        self._platform.send_vel(0.0, 0.0, 0.0)

    @property
    def current_goal(self) -> Goal | None:
        return self._goal

    @property
    def current_pose(self) -> Pose | None:
        return self._pose

    async def run(self) -> None:
        path: list[tuple[float, float]] = []
        last_replan = 0.0
        loop = asyncio.get_running_loop()

        while True:
            await asyncio.sleep(0.1)  # 10 Hz

            goal = self._goal
            pose = self._pose
            if goal is None or pose is None:
                continue

            # Arrival check.
            if math.hypot(goal.x - pose.x, goal.y - pose.y) < self._cfg.arrival_tolerance:
                self._platform.send_vel(0.0, 0.0, 0.0)
                self._goal = None
                path = []
                continue

            now = loop.time()
            if not path or now - last_replan >= self._cfg.replan_interval:
                new_path = await loop.run_in_executor(None, self._plan, pose, goal)
                if new_path is not None:
                    path = new_path
                last_replan = now

            if not path:
                vx, vy, omega = compute_velocity(pose, goal.x, goal.y, goal.yaw, self._cfg)
            else:
                tx, ty = _lookahead(path, pose.x, pose.y, self._cfg.lookahead_distance)
                vx, vy, omega = compute_velocity(pose, tx, ty, None, self._cfg)

            self._platform.send_vel(vx, vy, omega)

    # ------------------------------------------------------------------
    # Internal helpers (run in thread-pool via run_in_executor)
    # ------------------------------------------------------------------

    def _plan(self, pose: Pose, goal: Goal) -> list[tuple[float, float]] | None:
        blocked = self._costmap.get() > 0.5
        H, W = blocked.shape
        res = self._mapper.resolution
        ox, oy = self._mapper.origin

        r0 = max(0, min(H - 1, int((pose.y - oy) / res)))
        c0 = max(0, min(W - 1, int((pose.x - ox) / res)))
        rg = max(0, min(H - 1, int((goal.y - oy) / res)))
        cg = max(0, min(W - 1, int((goal.x - ox) / res)))

        # If goal cell is blocked, BFS outward to nearest free cell.
        if blocked[rg, cg]:
            rg, cg = _nearest_free(blocked, rg, cg)
            if rg is None:
                return None

        if (r0, c0) == (rg, cg):
            return [(goal.x, goal.y)]

        cells = _astar(blocked, (r0, c0), (rg, cg))
        if cells is None:
            return None

        world = [(ox + c * res, oy + r * res) for r, c in cells]
        return _smooth(world, blocked, ox, oy, res)


# ------------------------------------------------------------------
# Module-level functions
# ------------------------------------------------------------------

def _astar(
    blocked: NDArray[np.bool_],
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]] | None:
    H, W = blocked.shape
    g: dict[tuple[int, int], float] = {start: 0.0}
    came: dict[tuple[int, int], tuple[int, int]] = {}
    heap: list[tuple[float, int, tuple[int, int]]] = []
    counter = 0

    def h(r: int, c: int) -> float:
        return math.hypot(r - goal[0], c - goal[1])

    heapq.heappush(heap, (h(*start), counter, start))

    while heap:
        _, _, cur = heapq.heappop(heap)
        if cur == goal:
            path, node = [], cur
            while node in came:
                path.append(node)
                node = came[node]
            path.append(start)
            return path[::-1]

        r, c = cur
        for dr, dc in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
            nr, nc = r + dr, c + dc
            if not (0 <= nr < H and 0 <= nc < W) or blocked[nr, nc]:
                continue
            ng = g[cur] + (1.4142 if dr and dc else 1.0)
            nb = (nr, nc)
            if ng < g.get(nb, float("inf")):
                g[nb] = ng
                came[nb] = cur
                counter += 1
                heapq.heappush(heap, (ng + h(nr, nc), counter, nb))

    return None


def _nearest_free(
    blocked: NDArray[np.bool_], r: int, c: int
) -> tuple[int, int] | tuple[None, None]:
    H, W = blocked.shape
    q: deque[tuple[int, int]] = deque([(r, c)])
    seen = {(r, c)}
    while q:
        cr, cc = q.popleft()
        if not blocked[cr, cc]:
            return cr, cc
        for dr, dc in ((-1,0),(1,0),(0,-1),(0,1)):
            nr, nc = cr + dr, cc + dc
            if 0 <= nr < H and 0 <= nc < W and (nr, nc) not in seen:
                seen.add((nr, nc))
                q.append((nr, nc))
    return None, None


def _smooth(
    path: list[tuple[float, float]],
    blocked: NDArray[np.bool_],
    ox: float,
    oy: float,
    res: float,
) -> list[tuple[float, float]]:
    """Shortcut path segments that have clear line-of-sight."""
    if len(path) <= 2:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _los(path[i], path[j], blocked, ox, oy, res):
            j -= 1
        out.append(path[j])
        i = j
    return out


def _los(
    p1: tuple[float, float],
    p2: tuple[float, float],
    blocked: NDArray[np.bool_],
    ox: float,
    oy: float,
    res: float,
) -> bool:
    n = max(1, int(math.hypot(p2[0] - p1[0], p2[1] - p1[1]) / res * 2))
    H, W = blocked.shape
    for i in range(n + 1):
        t = i / n
        r = int((p1[1] + t * (p2[1] - p1[1]) - oy) / res)
        c = int((p1[0] + t * (p2[0] - p1[0]) - ox) / res)
        if 0 <= r < H and 0 <= c < W and blocked[r, c]:
            return False
    return True


def _lookahead(
    path: list[tuple[float, float]],
    rx: float,
    ry: float,
    dist: float,
) -> tuple[float, float]:
    for x, y in path:
        if math.hypot(x - rx, y - ry) >= dist:
            return x, y
    return path[-1]
