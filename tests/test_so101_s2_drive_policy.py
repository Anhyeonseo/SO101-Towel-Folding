from unittest.mock import Mock
import numpy as np
import pytest
from tools.lib.so101_s2_drive_policy import PositionDrivePolicy


def test_contact_motion_uses_targets_without_reinitializing():
    robot = Mock()
    q = np.zeros(12)
    drive = PositionDrivePolicy(range(12), q, [[0, 1], [0, 1]], .01)
    drive.initialize(robot, q, q)
    for i in range(10):
        q = q.copy(); q[5] += .01
        drive.command(robot, q, q)
    assert robot.write_joint_state_to_sim_index.call_count == 1
    assert robot.set_joint_position_target_index.call_count == 10
    with pytest.raises(RuntimeError):
        drive.initialize(robot, q, q)


@pytest.mark.parametrize('bad', [float('nan'), -.01, 1.1, .03])
def test_invalid_or_too_fast_command_never_reaches_robot(bad):
    robot = Mock(); q = np.zeros(12)
    drive = PositionDrivePolicy(range(12), q, [[0, 1], [0, 1]], .01)
    drive.initialize(robot, q, q)
    q[5] = bad
    with pytest.raises(ValueError):
        drive.command(robot, q, q)
    robot.set_joint_position_target_index.assert_not_called()
