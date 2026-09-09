from math import isclose, radians

from so101_isaac_bridge.mapping import BY_PROJECT_NAME, JOINT_SPECS


def test_round_trip_at_limits_and_zero():
    for spec in JOINT_SPECS:
        for value in (spec.lower, 0.0, spec.upper):
            isaac_value = spec.project_to_isaac_position(value)
            restored = spec.isaac_to_project_position(isaac_value)
            assert isclose(restored, value, abs_tol=1e-12)


def test_arm_axes_are_inverted():
    for spec in JOINT_SPECS[:5]:
        assert spec.project_to_isaac_position(0.1) == -0.1
        assert spec.isaac_to_project_position(-0.1) == 0.1


def test_gripper_q0_is_isaac_minus_ten_degrees():
    spec = BY_PROJECT_NAME["left_gripper_joint"]
    assert isclose(
        spec.project_to_isaac_position(0.0),
        -radians(10.0),
        abs_tol=1e-12,
    )
    assert isclose(
        spec.isaac_to_project_position(-radians(10.0)),
        0.0,
        abs_tol=1e-12,
    )
