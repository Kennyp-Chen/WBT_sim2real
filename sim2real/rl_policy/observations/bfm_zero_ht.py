"""BFM-Zero observations for the HT PiPlus-LSE and Hi checkpoints.

The two checkpoints share the FB-Cpr-Aux layout, but their joint/body contracts
are different.  Keeping the dimensions in this module makes an accidental
22/23/25 DoF mix-up fail during policy construction instead of at inference.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from sim2real.config.robots.ht_bfmzero import (
    HI_25DOF_BFM_BODY_NAMES,
    HI_25DOF_JOINT_NAMES,
    PIPLUS_LSE_23DOF_BFM_BODY_NAMES,
    PIPLUS_LSE_23DOF_JOINT_NAMES,
)
from sim2real.rl_policy.observations.base import Observation
from sim2real.rl_policy.observations.bfm_zero import (
    _heading_from_quat_wxyz,
    _projected_gravity,
    _quat_angle_axis_humanoidverse_wxyz,
    _quat_from_yaw,
    _quat_to_tan_norm,
)
from sim2real.rl_policy.utils.motion import MotionData
from sim2real.utils.math import quat_conjugate, quat_mul, quat_rotate_inverse_numpy, quat_rotate_numpy


@dataclass(frozen=True)
class _Spec:
    label: str
    action_dim: int
    state_dim: int
    history_dim: int
    privileged_dim: int
    joint_names: tuple[str, ...]
    body_names: tuple[str, ...]
    extensions: tuple[tuple[str, str, tuple[float, float, float]], ...]
    imu_body_name: str
    base_ang_vel_scale: float = 0.25
    action_obs_scale: float | tuple[float, ...] = 32.0
    action_obs_clip: float | tuple[float, ...] = 32.0


PIPLUS_LSE_23DOF_SPEC = _Spec(
    label="PiPlus-LSE-23DoF",
    action_dim=23,
    state_dim=52,
    history_dim=300,
    privileged_dim=403,
    joint_names=tuple(PIPLUS_LSE_23DOF_JOINT_NAMES),
    body_names=tuple(PIPLUS_LSE_23DOF_BFM_BODY_NAMES),
    extensions=(
        ("r_hand_link", "r_elbow_link", (0.0, 0.0, -0.129)),
        ("l_hand_link", "l_elbow_link", (0.0, 0.0, -0.129)),
        ("head_link", "head_pitch_link", (0.01, 0.0, 0.06)),
    ),
    imu_body_name="waist_yaw_link",
)
HI_25DOF_SPEC = _Spec(
    label="Hi-25DoF",
    action_dim=25,
    state_dim=56,
    history_dim=324,
    privileged_dim=403,
    joint_names=tuple(HI_25DOF_JOINT_NAMES),
    body_names=tuple(HI_25DOF_BFM_BODY_NAMES),
    extensions=(("head_link", "head_pitch_link", (0.01, 0.0, 0.06)),),
    imu_body_name="waist_yaw_link",
    action_obs_scale=(36.0, 36.0, 36.0, 36.0, 36.0, 36.0, 36.0, 36.0, 36.0, 36.0, 36.0, 3.0, 3.0, 36.0, 36.0, 36.0, 36.0, 8.0, 3.0, 36.0, 36.0, 36.0, 36.0, 8.0, 3.0),
    action_obs_clip=(36.0, 36.0, 36.0, 36.0, 36.0, 36.0, 36.0, 36.0, 36.0, 36.0, 36.0, 3.0, 3.0, 36.0, 36.0, 36.0, 36.0, 8.0, 3.0, 36.0, 36.0, 36.0, 36.0, 8.0, 3.0),
)


def _batched(value: np.ndarray) -> np.ndarray:
    return np.asarray(value, dtype=np.float32).reshape(1, -1)


def _action_obs(action: np.ndarray, spec: _Spec) -> np.ndarray:
    action = np.asarray(action, dtype=np.float32).reshape(-1)
    if action.size != spec.action_dim:
        raise ValueError(f"{spec.label} action dim mismatch: {action.size} != {spec.action_dim}")
    scale = np.asarray(spec.action_obs_scale, dtype=np.float32)
    clip = np.asarray(spec.action_obs_clip, dtype=np.float32)
    if scale.ndim == 0:
        scale = np.full(spec.action_dim, float(scale), dtype=np.float32)
    if clip.ndim == 0:
        clip = np.full(spec.action_dim, float(clip), dtype=np.float32)
    if scale.shape != (spec.action_dim,) or clip.shape != (spec.action_dim,):
        raise ValueError(f"{spec.label} action normalization shape mismatch")
    return np.clip(action * scale, -clip, clip).astype(np.float32)


def _projected_gravity_batch(root_quat_wxyz: np.ndarray) -> np.ndarray:
    quat = np.asarray(root_quat_wxyz, dtype=np.float32).reshape(-1, 4)
    gravity = np.zeros((quat.shape[0], 3), dtype=np.float32)
    gravity[:, 2] = -1.0
    return quat_rotate_inverse_numpy(quat, gravity).astype(np.float32)


def _compute_motion_velocities(body_pos_w: np.ndarray, body_quat_w: np.ndarray, joint_pos: np.ndarray, *, fps: float, sigma: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    from scipy.ndimage import gaussian_filter1d

    body_pos_w = np.asarray(body_pos_w, dtype=np.float32)
    body_quat_w = np.asarray(body_quat_w, dtype=np.float32)
    joint_pos = np.asarray(joint_pos, dtype=np.float32)
    dt = 1.0 / float(fps)
    body_lin_vel_w = gaussian_filter1d(np.gradient(body_pos_w, axis=0) / dt, sigma, axis=0, mode="nearest")
    quat_delta = np.zeros_like(body_quat_w)
    if body_quat_w.shape[0] > 1:
        quat_delta[:-1] = quat_mul(body_quat_w[1:], quat_conjugate(body_quat_w[:-1]))
    quat_delta[-1, ..., 0] = 1.0
    angle, axis = _quat_angle_axis_humanoidverse_wxyz(quat_delta)
    body_ang_vel_w = gaussian_filter1d(axis * angle[..., None] / dt, sigma, axis=0, mode="nearest")
    joint_vel = np.zeros_like(joint_pos, dtype=np.float32)
    if joint_pos.shape[0] > 1:
        diff = (joint_pos[1:] - joint_pos[:-1]) / dt
        joint_vel[:-1] = diff
        joint_vel[-1] = diff[-1]
    return body_lin_vel_w.astype(np.float32), body_ang_vel_w.astype(np.float32), joint_vel.astype(np.float32)


def _privileged_state(body_pos: np.ndarray, body_quat: np.ndarray, body_vel: np.ndarray, body_ang_vel: np.ndarray, spec: _Spec) -> np.ndarray:
    body_pos = np.asarray(body_pos, dtype=np.float32)
    body_quat = np.asarray(body_quat, dtype=np.float32)
    body_vel = np.asarray(body_vel, dtype=np.float32)
    body_ang_vel = np.asarray(body_ang_vel, dtype=np.float32)
    root_pos = body_pos[0]
    heading_inv = _quat_from_yaw(-_heading_from_quat_wxyz(body_quat[0]))
    heading_inv_expand = np.broadcast_to(heading_inv.reshape(1, 4), body_quat.shape)
    local_body_pos = quat_rotate_numpy(heading_inv_expand, body_pos - root_pos.reshape(1, 3)).reshape(-1)[3:]
    local_body_quat = quat_mul(heading_inv_expand, body_quat)
    local_body_rot = _quat_to_tan_norm(local_body_quat).reshape(-1)
    local_body_vel = quat_rotate_numpy(heading_inv_expand, body_vel).reshape(-1)
    local_body_ang_vel = quat_rotate_numpy(heading_inv_expand, body_ang_vel).reshape(-1)
    value = np.concatenate([root_pos[2:3], local_body_pos, local_body_rot, local_body_vel, local_body_ang_vel], axis=0).astype(np.float32)
    if value.size != spec.privileged_dim:
        raise ValueError(f"{spec.label} privileged dim mismatch: {value.size} != {spec.privileged_dim}")
    return value


def _append_extended_bodies(body_pos: np.ndarray, body_quat: np.ndarray, source_body_names: Sequence[str], spec: _Spec) -> tuple[np.ndarray, np.ndarray]:
    names = list(source_body_names)
    extra_pos: list[np.ndarray] = []
    extra_quat: list[np.ndarray] = []
    for name, parent_name, offset_values in spec.extensions:
        if name in names:
            continue
        parent_idx = names.index(parent_name)
        offset = np.asarray(offset_values, dtype=np.float32)
        parent_pos = body_pos[:, parent_idx]
        parent_quat = body_quat[:, parent_idx]
        extra_pos.append(parent_pos + quat_rotate_numpy(parent_quat, np.broadcast_to(offset, parent_pos.shape)))
        extra_quat.append(parent_quat.copy())
        names.append(name)
    if extra_pos:
        body_pos = np.concatenate([body_pos, *[value[:, None, :] for value in extra_pos]], axis=1)
        body_quat = np.concatenate([body_quat, *[value[:, None, :] for value in extra_quat]], axis=1)
    return body_pos.astype(np.float32), body_quat.astype(np.float32)


class _HTJointSelection(Observation):
    spec: _Spec

    def __init__(self, joint_names: Sequence[str] | None = None, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.joint_names = list(joint_names or self.env.policy_joint_names)
        if len(self.joint_names) != self.spec.action_dim:
            raise ValueError(f"{self.spec.label} expects {self.spec.action_dim} joints, got {len(self.joint_names)}")
        self._state_joint_indices = [self.state_processor.joint_names.index(name) for name in self.joint_names]
        self._default_joint_pos = np.asarray([self.env.default_dof_angles[self.env.joint_names_simulation.index(name)] for name in self.joint_names], dtype=np.float32)

    def _dof_pos(self) -> np.ndarray:
        return np.asarray(self.state_processor.joint_pos[self._state_joint_indices], dtype=np.float32) - self._default_joint_pos

    def _dof_vel(self) -> np.ndarray:
        return np.asarray(self.state_processor.joint_vel[self._state_joint_indices], dtype=np.float32)


class _HTState(_HTJointSelection):
    def compute(self) -> np.ndarray:
        state = np.concatenate([self._dof_pos(), self._dof_vel(), _projected_gravity(self.state_processor.root_quat_w), np.asarray(self.state_processor.root_ang_vel_b, dtype=np.float32) * self.spec.base_ang_vel_scale])
        if state.size != self.spec.state_dim:
            raise ValueError(f"{self.spec.label} state dim mismatch: {state.size} != {self.spec.state_dim}")
        return _batched(state)


class _HTLastAction(_HTJointSelection):
    def reset(self) -> None:
        self._last_action = np.zeros(self.spec.action_dim, dtype=np.float32)

    def update(self, data: dict[str, Any]) -> None:
        self._last_action = _action_obs(data.get("action", np.zeros(self.spec.action_dim, dtype=np.float32)), self.spec)

    def compute(self) -> np.ndarray:
        return _batched(getattr(self, "_last_action", np.zeros(self.spec.action_dim, dtype=np.float32)))


class _HTHistoryActor(_HTJointSelection):
    def __init__(self, history_length: int = 4, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.history_length = int(history_length)
        if self.history_length <= 0:
            raise ValueError("history_length must be positive")
        self._history = {
            "actions": np.zeros((self.history_length, self.spec.action_dim), dtype=np.float32),
            "base_ang_vel": np.zeros((self.history_length, 3), dtype=np.float32),
            "dof_pos": np.zeros((self.history_length, self.spec.action_dim), dtype=np.float32),
            "dof_vel": np.zeros((self.history_length, self.spec.action_dim), dtype=np.float32),
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
            "actions": _action_obs(data.get("action", np.zeros(self.spec.action_dim, dtype=np.float32)), self.spec),
            "base_ang_vel": np.asarray(self.state_processor.root_ang_vel_b, dtype=np.float32) * self.spec.base_ang_vel_scale,
            "dof_pos": self._dof_pos(),
            "dof_vel": self._dof_vel(),
            "projected_gravity": _projected_gravity(self.state_processor.root_quat_w),
        }
        self._pending_written = False

    def _append_pending_current(self) -> None:
        if self._pending_written or self._pending_current is None:
            return
        for key, history in self._history.items():
            history[1:] = history[:-1].copy()
            history[0] = self._pending_current[key]
        self._pending_written = True

    def compute(self) -> np.ndarray:
        history = np.concatenate([self._history[key].reshape(-1) for key in ("actions", "base_ang_vel", "dof_pos", "dof_vel", "projected_gravity")])
        self._append_pending_current()
        if history.size != self.spec.history_dim:
            raise ValueError(f"{self.spec.label} history dim mismatch: {history.size} != {self.spec.history_dim}")
        return _batched(history)


class _HTBackwardWindow(Observation):
    spec: _Spec
    obs_key: str

    def __init__(self, joint_names: Sequence[str] | None = None, seq_length: int = 8, target_fps: float = 50.0, clamp_to_final: bool = True, motion_t_offset: int = -1, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.joint_names = list(joint_names or self.env.policy_joint_names)
        self.seq_length = int(seq_length)
        self.target_fps = float(target_fps)
        self.clamp_to_final = bool(clamp_to_final)
        self.motion_t_offset = int(motion_t_offset)
        self._default_joint_pos = np.asarray([self.env.default_dof_angles[self.env.joint_names_simulation.index(name)] for name in self.joint_names], dtype=np.float32)
        if self.seq_length <= 0:
            raise ValueError("seq_length must be positive")

    def _window(self) -> dict[str, np.ndarray]:
        if not self.clamp_to_final:
            raise ValueError(f"{self.spec.label} requires clamp_to_final=True")
        required_steps = self.motion_t_offset - 1 + np.arange(self.seq_length + 1, dtype=np.int64)
        available_steps = np.asarray(self.env.motion_future_steps, dtype=np.int64).reshape(-1)
        step_to_index = {int(step): idx for idx, step in enumerate(available_steps)}
        missing = [int(step) for step in required_steps if int(step) not in step_to_index]
        if missing:
            raise ValueError(f"motion.future_steps is missing {missing}; got {available_steps.tolist()}")
        support_indices = np.asarray([step_to_index[int(step)] for step in required_steps], dtype=np.int64)
        motion_data: MotionData | None = self.env.motion_data
        if motion_data is None:
            raise ValueError(f"{self.spec.label} requires motion_data")
        source_joint_names = list(self.env.motion_joint_names)
        source_body_names = list(self.env.motion_body_names)
        joint_indices = [source_joint_names.index(name) for name in self.joint_names]
        body_indices = [source_body_names.index(name) for name in self.spec.body_names]
        dof_pos = np.asarray(motion_data.joint_pos[0, support_indices][:, joint_indices], dtype=np.float32)
        body_pos = np.asarray(motion_data.body_pos_w[0, support_indices][:, body_indices], dtype=np.float32)
        body_quat = np.asarray(motion_data.body_quat_w[0, support_indices][:, body_indices], dtype=np.float32)
        body_pos, body_quat = _append_extended_bodies(body_pos, body_quat, self.spec.body_names, self.spec)
        body_lin_vel, body_ang_vel, joint_vel = _compute_motion_velocities(body_pos, body_quat, dof_pos, fps=self.target_fps, sigma=2.0)
        target = slice(1, self.seq_length + 1)
        imu_idx = self.spec.body_names.index(self.spec.imu_body_name)
        state = np.concatenate([dof_pos[target] - self._default_joint_pos, joint_vel[target], _projected_gravity_batch(body_quat[target, imu_idx]), body_ang_vel[target, imu_idx]], axis=-1).astype(np.float32)
        privileged = np.stack([_privileged_state(body_pos[index], body_quat[index], body_lin_vel[index], body_ang_vel[index], self.spec) for index in range(1, self.seq_length + 1)], axis=0)
        return {"state": state, "privileged_state": privileged}

    def compute(self) -> np.ndarray:
        value = self._window()[self.obs_key]
        expected_dim = self.spec.state_dim if self.obs_key == "state" else self.spec.privileged_dim
        if value.shape != (self.seq_length, expected_dim):
            raise ValueError(f"{self.spec.label} {self.obs_key} window shape mismatch: {value.shape}")
        return value.astype(np.float32)


class _HTWindowWeight(Observation):
    spec: _Spec

    def __init__(self, seq_length: int = 8, clamp_to_final: bool = True, motion_t_offset: int = -1, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.seq_length = int(seq_length)
        self.clamp_to_final = bool(clamp_to_final)
        self.motion_t_offset = int(motion_t_offset)

    def compute(self) -> np.ndarray:
        if not self.clamp_to_final:
            return np.ones((self.seq_length, 1), dtype=np.float32)
        required_steps = self.motion_t_offset - 1 + np.arange(self.seq_length + 1, dtype=np.int64)
        available_steps = np.asarray(self.env.motion_future_steps, dtype=np.int64).reshape(-1)
        step_to_index = {int(step): idx for idx, step in enumerate(available_steps)}
        support_indices = np.asarray([step_to_index[int(step)] for step in required_steps], dtype=np.int64)
        motion_data: MotionData | None = self.env.motion_data
        if motion_data is None:
            raise ValueError(f"{self.spec.label} requires motion_data")
        source_step = np.asarray(motion_data.step[0], dtype=np.int64)
        output_step = source_step[support_indices][1:]
        output_required = required_steps[1:]
        weights = np.ones(self.seq_length, dtype=np.float32)
        if self.seq_length > 1:
            weights[1:] = np.where((output_step[1:] == output_step[:-1]) & (output_required[1:] > 0), 0.0, 1.0)
        return weights.reshape(self.seq_length, 1)


class bfm_zero_piplus_lse_state(_HTState, namespace="bfm_zero"):
    spec = PIPLUS_LSE_23DOF_SPEC


class bfm_zero_piplus_lse_last_action(_HTLastAction, namespace="bfm_zero"):
    spec = PIPLUS_LSE_23DOF_SPEC


class bfm_zero_piplus_lse_history_actor(_HTHistoryActor, namespace="bfm_zero"):
    spec = PIPLUS_LSE_23DOF_SPEC


class bfm_zero_piplus_lse_encoder_state_future(_HTBackwardWindow, namespace="bfm_zero"):
    spec = PIPLUS_LSE_23DOF_SPEC
    obs_key = "state"


class bfm_zero_piplus_lse_privileged_state_future(_HTBackwardWindow, namespace="bfm_zero"):
    spec = PIPLUS_LSE_23DOF_SPEC
    obs_key = "privileged_state"


class bfm_zero_piplus_lse_future_window_weight(_HTWindowWeight, namespace="bfm_zero"):
    spec = PIPLUS_LSE_23DOF_SPEC


class bfm_zero_hi25_state(_HTState, namespace="bfm_zero"):
    spec = HI_25DOF_SPEC


class bfm_zero_hi25_last_action(_HTLastAction, namespace="bfm_zero"):
    spec = HI_25DOF_SPEC


class bfm_zero_hi25_history_actor(_HTHistoryActor, namespace="bfm_zero"):
    spec = HI_25DOF_SPEC


class bfm_zero_hi25_encoder_state_future(_HTBackwardWindow, namespace="bfm_zero"):
    spec = HI_25DOF_SPEC
    obs_key = "state"


class bfm_zero_hi25_privileged_state_future(_HTBackwardWindow, namespace="bfm_zero"):
    spec = HI_25DOF_SPEC
    obs_key = "privileged_state"


class bfm_zero_hi25_future_window_weight(_HTWindowWeight, namespace="bfm_zero"):
    spec = HI_25DOF_SPEC
