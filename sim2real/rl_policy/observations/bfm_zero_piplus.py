from __future__ import annotations

from typing import Any, Sequence

import numpy as np

from sim2real.rl_policy.observations.base import Observation
from sim2real.rl_policy.observations.bfm_zero import (
    _heading_from_quat_wxyz,
    _projected_gravity,
    _quat_angle_axis_humanoidverse_wxyz,
    _quat_from_yaw,
    _quat_to_tan_norm,
)
from sim2real.rl_policy.utils.motion import MotionData
from sim2real.utils.math import (
    quat_conjugate,
    quat_mul,
    quat_rotate_inverse_numpy,
    quat_rotate_numpy,
)

PIPLUS_BFM_STATE_DIM = 50
PIPLUS_BFM_ACTION_DIM = 22
PIPLUS_BFM_HISTORY_DIM = 288
PIPLUS_BFM_Z_DIM = 256
PIPLUS_BFM_PRIVILEGED_STATE_DIM = 388
PIPLUS_BFM_BASE_ANG_VEL_SCALE = 0.25
PIPLUS_BFM_ACTION_OBS_SCALE = 32.0
PIPLUS_BFM_ACTION_OBS_CLIP = 32.0

PIPLUS_BFM_JOINT_NAMES = (
    "r_shoulder_pitch_joint",
    "r_shoulder_roll_joint",
    "r_upper_arm_joint",
    "r_elbow_joint",
    "l_shoulder_pitch_joint",
    "l_shoulder_roll_joint",
    "l_upper_arm_joint",
    "l_elbow_joint",
    "head_yaw_joint",
    "head_pitch_joint",
    "r_hip_pitch_joint",
    "r_hip_roll_joint",
    "r_thigh_joint",
    "r_calf_joint",
    "r_ankle_pitch_joint",
    "r_ankle_roll_joint",
    "l_hip_pitch_joint",
    "l_hip_roll_joint",
    "l_thigh_joint",
    "l_calf_joint",
    "l_ankle_pitch_joint",
    "l_ankle_roll_joint",
)

# This is the source config's IsaacSim body order after fixed-body collapse.
PIPLUS_BFM_BODY_NAMES = (
    "base_link",
    "r_shoulder_pitch_link",
    "r_shoulder_roll_link",
    "r_upper_arm_link",
    "r_elbow_link",
    "l_shoulder_pitch_link",
    "l_shoulder_roll_link",
    "l_upper_arm_link",
    "l_elbow_link",
    "head_yaw_link",
    "head_pitch_link",
    "r_hip_pitch_link",
    "r_hip_roll_link",
    "r_thigh_link",
    "r_calf_link",
    "r_ankle_pitch_link",
    "r_ankle_roll_link",
    "l_hip_pitch_link",
    "l_hip_roll_link",
    "l_thigh_link",
    "l_calf_link",
    "l_ankle_pitch_link",
    "l_ankle_roll_link",
)
PIPLUS_BFM_EXTENDED_BODY_NAMES = ("r_hand_link", "l_hand_link", "head_link")
PIPLUS_BFM_ALL_BODY_NAMES = PIPLUS_BFM_BODY_NAMES + PIPLUS_BFM_EXTENDED_BODY_NAMES
PIPLUS_BFM_HISTORY_KEYS = (
    "actions",
    "base_ang_vel",
    "dof_pos",
    "dof_vel",
    "projected_gravity",
)

_EXTENSIONS = (
    ("r_hand_link", "r_elbow_link", np.asarray([0.0, 0.0, -0.129], dtype=np.float32)),
    ("l_hand_link", "l_elbow_link", np.asarray([0.0, 0.0, -0.129], dtype=np.float32)),
    ("head_link", "head_pitch_link", np.asarray([0.01, 0.0, 0.06], dtype=np.float32)),
)


def _batched(value: np.ndarray) -> np.ndarray:
    return np.asarray(value, dtype=np.float32).reshape(1, -1)


def _scaled_base_ang_vel(value: np.ndarray) -> np.ndarray:
    return (
        np.asarray(value, dtype=np.float32).reshape(3) * PIPLUS_BFM_BASE_ANG_VEL_SCALE
    )


def _action_obs(action: np.ndarray) -> np.ndarray:
    action = np.asarray(action, dtype=np.float32).reshape(-1)
    if action.size != PIPLUS_BFM_ACTION_DIM:
        raise ValueError(
            f"PiPlus BFM-Zero action dim mismatch: {action.size} != {PIPLUS_BFM_ACTION_DIM}"
        )
    return np.clip(
        action * PIPLUS_BFM_ACTION_OBS_SCALE,
        -PIPLUS_BFM_ACTION_OBS_CLIP,
        PIPLUS_BFM_ACTION_OBS_CLIP,
    ).astype(np.float32)


def _projected_gravity_batch(root_quat_wxyz: np.ndarray) -> np.ndarray:
    quat = np.asarray(root_quat_wxyz, dtype=np.float32).reshape(-1, 4)
    gravity = np.zeros((quat.shape[0], 3), dtype=np.float32)
    gravity[:, 2] = -1.0
    return quat_rotate_inverse_numpy(quat, gravity).astype(np.float32)


def _compute_motion_velocities(
    body_pos_w: np.ndarray,
    body_quat_w: np.ndarray,
    joint_pos: np.ndarray,
    *,
    fps: float,
    sigma: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Match HumanoidVerse's motion-lib finite-difference velocity path."""
    body_pos_w = np.asarray(body_pos_w, dtype=np.float32)
    body_quat_w = np.asarray(body_quat_w, dtype=np.float32)
    joint_pos = np.asarray(joint_pos, dtype=np.float32)
    dt = 1.0 / float(fps)

    from scipy.ndimage import gaussian_filter1d

    body_lin_vel_w = gaussian_filter1d(
        np.gradient(body_pos_w, axis=0) / dt, sigma, axis=0, mode="nearest"
    )
    quat_delta = np.zeros_like(body_quat_w)
    if body_quat_w.shape[0] > 1:
        quat_delta[:-1] = quat_mul(body_quat_w[1:], quat_conjugate(body_quat_w[:-1]))
    quat_delta[-1, ..., 0] = 1.0
    angle, axis = _quat_angle_axis_humanoidverse_wxyz(quat_delta)
    body_ang_vel_w = gaussian_filter1d(
        axis * angle[..., None] / dt, sigma, axis=0, mode="nearest"
    )

    joint_vel = np.zeros_like(joint_pos, dtype=np.float32)
    if joint_pos.shape[0] > 1:
        joint_diff = (joint_pos[1:] - joint_pos[:-1]) / dt
        joint_vel[:-1] = joint_diff
        joint_vel[-1] = joint_diff[-1]
    return (
        body_lin_vel_w.astype(np.float32),
        body_ang_vel_w.astype(np.float32),
        joint_vel.astype(np.float32),
    )


def _privileged_state(
    body_pos: np.ndarray,
    body_quat: np.ndarray,
    body_vel: np.ndarray,
    body_ang_vel: np.ndarray,
) -> np.ndarray:
    body_pos = np.asarray(body_pos, dtype=np.float32)
    body_quat = np.asarray(body_quat, dtype=np.float32)
    body_vel = np.asarray(body_vel, dtype=np.float32)
    body_ang_vel = np.asarray(body_ang_vel, dtype=np.float32)

    root_pos = body_pos[0]
    root_quat = body_quat[0]
    heading_inv = _quat_from_yaw(-_heading_from_quat_wxyz(root_quat))
    heading_inv_expand = np.broadcast_to(heading_inv.reshape(1, 4), body_quat.shape)
    local_body_pos = quat_rotate_numpy(
        heading_inv_expand,
        body_pos - root_pos.reshape(1, 3),
    ).reshape(-1)[3:]
    local_body_quat = quat_mul(heading_inv_expand, body_quat)
    local_body_rot = _quat_to_tan_norm(local_body_quat).reshape(-1)
    local_body_vel = quat_rotate_numpy(heading_inv_expand, body_vel).reshape(-1)
    local_body_ang_vel = quat_rotate_numpy(heading_inv_expand, body_ang_vel).reshape(-1)
    value = np.concatenate(
        [
            root_pos[2:3],
            local_body_pos,
            local_body_rot,
            local_body_vel,
            local_body_ang_vel,
        ],
        axis=0,
    ).astype(np.float32)
    if value.size != PIPLUS_BFM_PRIVILEGED_STATE_DIM:
        raise ValueError(
            f"PiPlus BFM-Zero privileged dim mismatch: {value.size} != {PIPLUS_BFM_PRIVILEGED_STATE_DIM}"
        )
    return value


def _append_extended_bodies(
    body_pos: np.ndarray,
    body_quat: np.ndarray,
    source_body_names: Sequence[str],
) -> tuple[np.ndarray, np.ndarray]:
    body_pos = np.asarray(body_pos, dtype=np.float32)
    body_quat = np.asarray(body_quat, dtype=np.float32)
    names = list(source_body_names)
    extra_pos: list[np.ndarray] = []
    extra_quat: list[np.ndarray] = []
    for name, parent_name, offset in _EXTENSIONS:
        if name in names:
            continue
        parent_idx = names.index(parent_name)
        parent_pos = body_pos[:, parent_idx]
        parent_quat = body_quat[:, parent_idx]
        extra_pos.append(
            parent_pos
            + quat_rotate_numpy(parent_quat, np.broadcast_to(offset, parent_pos.shape))
        )
        extra_quat.append(parent_quat.copy())
        names.append(name)
    if extra_pos:
        body_pos = np.concatenate(
            [body_pos, *[value[:, None, :] for value in extra_pos]], axis=1
        )
        body_quat = np.concatenate(
            [body_quat, *[value[:, None, :] for value in extra_quat]], axis=1
        )
    return body_pos.astype(np.float32), body_quat.astype(np.float32)


class _PiPlusJointSelection(Observation):
    def __init__(self, joint_names: Sequence[str] | None = None, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.joint_names = list(joint_names or self.env.policy_joint_names)
        if len(self.joint_names) != PIPLUS_BFM_ACTION_DIM:
            raise ValueError(
                f"PiPlus BFM-Zero expects {PIPLUS_BFM_ACTION_DIM} joints, got {len(self.joint_names)}"
            )
        self._state_joint_indices = [
            self.state_processor.joint_names.index(name) for name in self.joint_names
        ]
        self._default_joint_pos = np.asarray(
            [
                self.env.default_dof_angles[self.env.joint_names_simulation.index(name)]
                for name in self.joint_names
            ],
            dtype=np.float32,
        )

    def _dof_pos(self) -> np.ndarray:
        return (
            np.asarray(
                self.state_processor.joint_pos[self._state_joint_indices],
                dtype=np.float32,
            )
            - self._default_joint_pos
        )

    def _dof_vel(self) -> np.ndarray:
        return np.asarray(
            self.state_processor.joint_vel[self._state_joint_indices], dtype=np.float32
        )


class bfm_zero_piplus_state(_PiPlusJointSelection, namespace="bfm_zero"):
    def compute(self) -> np.ndarray:
        state = np.concatenate(
            [
                self._dof_pos(),
                self._dof_vel(),
                _projected_gravity(self.state_processor.root_quat_w),
                _scaled_base_ang_vel(self.state_processor.root_ang_vel_b),
            ],
            axis=0,
        )
        if state.size != PIPLUS_BFM_STATE_DIM:
            raise ValueError(
                f"PiPlus BFM-Zero state dim mismatch: {state.size} != {PIPLUS_BFM_STATE_DIM}"
            )
        return _batched(state)


class bfm_zero_piplus_last_action(_PiPlusJointSelection, namespace="bfm_zero"):
    def reset(self) -> None:
        self._last_action = np.zeros(PIPLUS_BFM_ACTION_DIM, dtype=np.float32)

    def update(self, data: dict[str, Any]) -> None:
        self._last_action = _action_obs(
            data.get("action", np.zeros(PIPLUS_BFM_ACTION_DIM, dtype=np.float32))
        )

    def compute(self) -> np.ndarray:
        return _batched(
            getattr(
                self, "_last_action", np.zeros(PIPLUS_BFM_ACTION_DIM, dtype=np.float32)
            )
        )


class bfm_zero_piplus_history_actor(_PiPlusJointSelection, namespace="bfm_zero"):
    def __init__(self, history_length: int = 4, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.history_length = int(history_length)
        if self.history_length <= 0:
            raise ValueError("history_length must be positive")
        self._history = {
            "actions": np.zeros(
                (self.history_length, PIPLUS_BFM_ACTION_DIM), dtype=np.float32
            ),
            "base_ang_vel": np.zeros((self.history_length, 3), dtype=np.float32),
            "dof_pos": np.zeros(
                (self.history_length, PIPLUS_BFM_ACTION_DIM), dtype=np.float32
            ),
            "dof_vel": np.zeros(
                (self.history_length, PIPLUS_BFM_ACTION_DIM), dtype=np.float32
            ),
            "projected_gravity": np.zeros((self.history_length, 3), dtype=np.float32),
        }
        self._pending_current: dict[str, np.ndarray] | None = None
        self._pending_written = True

    def reset(self) -> None:
        for value in self._history.values():
            value[:] = 0.0
        self._pending_current = None
        self._pending_written = True

    def update(self, data: dict[str, Any]) -> None:
        self._pending_current = {
            "actions": _action_obs(
                data.get("action", np.zeros(PIPLUS_BFM_ACTION_DIM, dtype=np.float32))
            ),
            "base_ang_vel": _scaled_base_ang_vel(self.state_processor.root_ang_vel_b),
            "dof_pos": self._dof_pos(),
            "dof_vel": self._dof_vel(),
            "projected_gravity": _projected_gravity(self.state_processor.root_quat_w),
        }
        self._pending_written = False

    def _append_pending_current(self) -> None:
        if self._pending_written or self._pending_current is None:
            return
        for key in PIPLUS_BFM_HISTORY_KEYS:
            history = self._history[key]
            history[1:] = history[:-1].copy()
            history[0] = self._pending_current[key]
        self._pending_written = True

    def compute(self) -> np.ndarray:
        history = np.concatenate(
            [self._history[key].reshape(-1) for key in PIPLUS_BFM_HISTORY_KEYS], axis=0
        )
        if history.size != PIPLUS_BFM_HISTORY_DIM:
            raise ValueError(
                f"PiPlus BFM-Zero history dim mismatch: {history.size} != {PIPLUS_BFM_HISTORY_DIM}"
            )
        self._append_pending_current()
        return _batched(history)


class _PiPlusBackwardWindow(Observation):
    obs_key: str

    def __init__(
        self,
        joint_names: Sequence[str] | None = None,
        seq_length: int = 8,
        target_fps: float = 50.0,
        clamp_to_final: bool = True,
        motion_t_offset: int = -1,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self.joint_names = list(joint_names or self.env.policy_joint_names)
        self.seq_length = int(seq_length)
        self.target_fps = float(target_fps)
        self.clamp_to_final = bool(clamp_to_final)
        self.motion_t_offset = int(motion_t_offset)
        self._default_joint_pos = np.asarray(
            [
                self.env.default_dof_angles[self.env.joint_names_simulation.index(name)]
                for name in self.joint_names
            ],
            dtype=np.float32,
        )
        if self.seq_length <= 0:
            raise ValueError("seq_length must be positive")

    def _window(self) -> dict[str, np.ndarray]:
        if not self.clamp_to_final:
            raise ValueError("PiPlus BFM-Zero currently requires clamp_to_final=True")
        required_steps = (
            self.motion_t_offset - 1 + np.arange(self.seq_length + 1, dtype=np.int64)
        )
        available_steps = np.asarray(
            self.env.motion_future_steps, dtype=np.int64
        ).reshape(-1)
        step_to_index = {int(step): idx for idx, step in enumerate(available_steps)}
        missing = [
            int(step) for step in required_steps if int(step) not in step_to_index
        ]
        if missing:
            raise ValueError(
                f"motion.future_steps is missing {missing}; got {available_steps.tolist()}"
            )
        support_indices = np.asarray(
            [step_to_index[int(step)] for step in required_steps], dtype=np.int64
        )

        motion_data: MotionData | None = self.env.motion_data
        if motion_data is None:
            raise ValueError("PiPlus BFM-Zero requires motion_data")
        source_joint_names = list(self.env.motion_joint_names)
        source_body_names = list(self.env.motion_body_names)
        joint_indices = [source_joint_names.index(name) for name in self.joint_names]
        body_indices = [source_body_names.index(name) for name in PIPLUS_BFM_BODY_NAMES]

        dof_pos = np.asarray(
            motion_data.joint_pos[0, support_indices][:, joint_indices],
            dtype=np.float32,
        )
        body_pos = np.asarray(
            motion_data.body_pos_w[0, support_indices][:, body_indices],
            dtype=np.float32,
        )
        body_quat = np.asarray(
            motion_data.body_quat_w[0, support_indices][:, body_indices],
            dtype=np.float32,
        )
        body_pos, body_quat = _append_extended_bodies(
            body_pos, body_quat, PIPLUS_BFM_BODY_NAMES
        )
        body_lin_vel, body_ang_vel, joint_vel = _compute_motion_velocities(
            body_pos,
            body_quat,
            dof_pos,
            fps=self.target_fps,
            sigma=2.0,
        )

        target = slice(1, self.seq_length + 1)
        root_quat = body_quat[target, 0]
        state = np.concatenate(
            [
                dof_pos[target] - self._default_joint_pos,
                joint_vel[target],
                _projected_gravity_batch(root_quat),
                body_ang_vel[target, 0],
            ],
            axis=-1,
        ).astype(np.float32)
        privileged = np.stack(
            [
                _privileged_state(
                    body_pos[index],
                    body_quat[index],
                    body_lin_vel[index],
                    body_ang_vel[index],
                )
                for index in range(1, self.seq_length + 1)
            ],
            axis=0,
        )
        return {
            "state": state,
            "privileged_state": privileged,
            "steps": required_steps[1:],
        }

    def compute(self) -> np.ndarray:
        value = self._window()[self.obs_key]
        expected_dim = (
            PIPLUS_BFM_STATE_DIM
            if self.obs_key == "state"
            else PIPLUS_BFM_PRIVILEGED_STATE_DIM
        )
        if value.shape != (self.seq_length, expected_dim):
            raise ValueError(
                f"PiPlus {self.obs_key} window shape mismatch: {value.shape}"
            )
        return value.astype(np.float32)


class bfm_zero_piplus_encoder_state_future(_PiPlusBackwardWindow, namespace="bfm_zero"):
    obs_key = "state"


class bfm_zero_piplus_privileged_state_future(
    _PiPlusBackwardWindow, namespace="bfm_zero"
):
    obs_key = "privileged_state"


class bfm_zero_piplus_future_window_weight(Observation, namespace="bfm_zero"):
    def __init__(
        self,
        seq_length: int = 8,
        clamp_to_final: bool = True,
        motion_t_offset: int = -1,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self.seq_length = int(seq_length)
        self.clamp_to_final = bool(clamp_to_final)
        self.motion_t_offset = int(motion_t_offset)

    def compute(self) -> np.ndarray:
        if not self.clamp_to_final:
            return np.ones((self.seq_length, 1), dtype=np.float32)
        required_steps = (
            self.motion_t_offset - 1 + np.arange(self.seq_length + 1, dtype=np.int64)
        )
        available_steps = np.asarray(
            self.env.motion_future_steps, dtype=np.int64
        ).reshape(-1)
        step_to_index = {int(step): idx for idx, step in enumerate(available_steps)}
        support_indices = np.asarray(
            [step_to_index[int(step)] for step in required_steps], dtype=np.int64
        )
        motion_data: MotionData | None = self.env.motion_data
        if motion_data is None:
            raise ValueError("PiPlus BFM-Zero requires motion_data")
        source_step = np.asarray(motion_data.step[0], dtype=np.int64)
        output_step = source_step[support_indices][1:]
        output_required = required_steps[1:]
        weights = np.ones(self.seq_length, dtype=np.float32)
        if self.seq_length > 1:
            weights[1:] = np.where(
                (output_step[1:] == output_step[:-1]) & (output_required[1:] > 0),
                0.0,
                1.0,
            )
        return weights.reshape(self.seq_length, 1)
