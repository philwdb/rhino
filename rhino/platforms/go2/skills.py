"""High-level skills for Go2.  Phase 3 implementation."""

from __future__ import annotations

import asyncio
import math

from rhino.navigation.planner import Navigator
from rhino.platforms.base import Platform


async def relative_move(
    platform: Platform,
    nav: Navigator,
    dx: float,
    dy: float,
    dyaw: float = 0.0,
) -> None:
    pose = nav.current_pose
    if pose is None:
        raise RuntimeError("No pose available")
    # TODO Phase 3: compute absolute goal and navigate
    raise NotImplementedError


def standup(platform: Platform) -> None:
    platform.send_cmd("standup")


def liedown(platform: Platform) -> None:
    platform.send_cmd("liedown")


def execute_sport(platform: Platform, cmd: str, **kwargs: object) -> None:
    platform.send_cmd(cmd, **kwargs)
