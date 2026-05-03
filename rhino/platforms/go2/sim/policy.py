from typing import Any

import mujoco
import numpy as np
import onnxruntime as ort  # type: ignore[import-untyped]


def _load_session(policy_path: str) -> ort.InferenceSession:
    available = set(ort.get_available_providers())
    for providers in [
        ["TensorrtExecutionProvider", "CUDAExecutionProvider", "CPUExecutionProvider"],
        ["CUDAExecutionProvider", "CPUExecutionProvider"],
        ["CPUExecutionProvider"],
    ]:
        filtered = [p for p in providers if p in available]
        if filtered:
            try:
                return ort.InferenceSession(policy_path, providers=filtered)
            except Exception:
                continue
    return ort.InferenceSession(policy_path, providers=["CPUExecutionProvider"])


class Go1OnnxController:
    def __init__(
        self,
        policy_path: str,
        default_angles: np.ndarray[Any, Any],
        n_substeps: int,
        action_scale: float,
        ctrl_dt: float = 0.02,
    ) -> None:
        self._policy = _load_session(policy_path)
        self._default_angles = default_angles
        self._action_scale = action_scale
        self._last_action = np.zeros_like(default_angles, dtype=np.float32)
        self._n_substeps = n_substeps
        self._counter = 0
        self._command = np.zeros(3, dtype=np.float32)

    def set_command(self, vx: float, vy: float, omega: float) -> None:
        self._command[:] = [vx, vy, omega]

    def get_control(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._counter += 1
        if self._counter % self._n_substeps != 0:
            return
        linvel = data.sensor("local_linvel").data
        gyro = data.sensor("gyro").data
        imu_xmat = data.site_xmat[model.site("imu").id].reshape(3, 3)
        gravity = imu_xmat.T @ np.array([0, 0, -1])
        joint_angles = data.qpos[7:] - self._default_angles
        joint_velocities = data.qvel[6:]
        obs = np.hstack([
            linvel, gyro, gravity,
            joint_angles, joint_velocities,
            self._last_action, self._command,
        ]).astype(np.float32)
        pred = self._policy.run(["continuous_actions"], {"obs": obs.reshape(1, -1)})[0][0]
        self._last_action = pred.copy()
        data.ctrl[:] = pred * self._action_scale + self._default_angles
