import numpy as np
import pytest

from scripts.retarget_gem_smpl import _stabilize_standing_base


JOINT_NAMES = [
    "arm_joint",
    "l_hip_joint",
    "head_joint",
    "r_ankle_joint",
]


def test_stabilize_standing_base_locks_support_and_scales_upper_body():
    qpos = np.arange(22, dtype=np.float32).reshape(2, 11)
    default_qpos = np.arange(11, dtype=np.float32) + 100.0

    output, locked_names = _stabilize_standing_base(
        qpos,
        default_qpos=default_qpos,
        joint_names=JOINT_NAMES,
        upper_body_scale=0.5,
    )

    assert locked_names == ["l_hip_joint", "r_ankle_joint"]
    np.testing.assert_array_equal(
        output[:, :7], np.broadcast_to(default_qpos[:7], (qpos.shape[0], 7))
    )
    np.testing.assert_array_equal(output[:, 8], default_qpos[8])
    np.testing.assert_array_equal(output[:, 10], default_qpos[10])
    np.testing.assert_allclose(
        output[:, 7], default_qpos[7] + 0.5 * (qpos[:, 7] - default_qpos[7])
    )
    np.testing.assert_allclose(
        output[:, 9], default_qpos[9] + 0.5 * (qpos[:, 9] - default_qpos[9])
    )
    np.testing.assert_array_equal(qpos, np.arange(22, dtype=np.float32).reshape(2, 11))


@pytest.mark.parametrize("scale", [-0.1, 1.1])
def test_stabilize_standing_base_rejects_scale_outside_unit_interval(scale):
    with pytest.raises(ValueError, match="upper_body_scale"):
        _stabilize_standing_base(
            np.zeros((2, 11), dtype=np.float32),
            default_qpos=np.zeros(11, dtype=np.float32),
            joint_names=JOINT_NAMES,
            upper_body_scale=scale,
        )
