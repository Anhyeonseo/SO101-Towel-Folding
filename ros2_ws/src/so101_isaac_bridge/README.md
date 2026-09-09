# SO-101 Isaac bridge

This package adapts the Isaac Sim 6.0.1 JointStates OmniGraph contract to
the validated left-arm MoveIt contract.

- Isaac state input: `/isaac/joint_states`
- Isaac command output: `/isaac/joint_command`
- MoveIt state output: `/joint_states`
- Arm action: `/left_arm_controller/follow_joint_trajectory`
- Gripper action: `/left_gripper_controller/gripper_cmd`

The five arm joints invert sign. The gripper keeps its sign and adds a
10-degree project offset, so MoveIt `q=0` maps to the Isaac `-10 deg`
closed pose.

This node is a simulation backend. It does not open a serial device and
does not communicate with the STM32 bridge.
