import math
from typing import Any

import numpy as np
from numpy.typing import NDArray

from rhino.platforms.go2.sim.constants import MAX_HEIGHT, MAX_RANGE, MIN_RANGE


def depth_image_to_point_cloud(
    depth_image: NDArray[Any],
    camera_pos: NDArray[Any],
    camera_mat: NDArray[Any],
    fov_degrees: float = 120,
) -> NDArray[Any]:
    h, w = depth_image.shape
    fovy = math.radians(fov_degrees)
    f = h / (2 * math.tan(fovy / 2))
    cx, cy = w / 2.0, h / 2.0

    # Pixel grid
    us, vs = np.meshgrid(
        np.arange(w, dtype=np.float32),
        np.arange(h, dtype=np.float32),
    )
    d = depth_image.astype(np.float32)

    # Back-project with OpenCV pinhole convention: x=right, y=down, z=forward
    X = (us - cx) * d / f
    Y = (vs - cy) * d / f
    Z = d

    # Match dimos post-processing: flip y (up) and z (toward camera)
    Y = -Y
    Z = -Z

    pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)

    # Range / height filter — same thresholds as dimos
    valid = (
        (np.abs(pts[:, 0]) <= MAX_RANGE)
        & (np.abs(pts[:, 1]) <= MAX_HEIGHT)
        & (np.abs(pts[:, 2]) >= MIN_RANGE)
        & (np.abs(pts[:, 2]) <= MAX_RANGE)
    )
    pts = pts[valid]

    if pts.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float32)

    # Rotate to world frame
    world: NDArray[Any] = (camera_mat @ pts.T).T + camera_pos
    return world


def voxel_downsample(points: NDArray[Any], voxel_size: float) -> NDArray[Any]:
    if points.shape[0] == 0:
        return points
    idx = np.floor(points / voxel_size).astype(np.int32)
    _, unique = np.unique(idx, axis=0, return_index=True)
    return points[unique]
