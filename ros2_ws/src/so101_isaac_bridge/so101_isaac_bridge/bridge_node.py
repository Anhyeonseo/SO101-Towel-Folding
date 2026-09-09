import math
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .mapping import (
    ARM_JOINTS,
    BY_ISAAC_NAME,
    BY_PROJECT_NAME,
    GRIPPER_JOINT,
    PROJECT_JOINTS,
)


def _duration_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


class So101IsaacBridge(Node):
    def __init__(self):
        super().__init__("so101_isaac_bridge")

        self.declare_parameter("isaac_state_topic", "/isaac/joint_states")
        self.declare_parameter("isaac_command_topic", "/isaac/joint_command")
        self.declare_parameter("project_state_topic", "/joint_states")
        self.declare_parameter("command_rate", 50.0)
        self.declare_parameter("goal_tolerance", 0.03)
        self.declare_parameter("goal_timeout", 2.0)
        self.declare_parameter("gripper_velocity", 1.0)

        self._command_rate = float(self.get_parameter("command_rate").value)
        self._goal_tolerance = float(
            self.get_parameter("goal_tolerance").value
        )
        self._goal_timeout = float(self.get_parameter("goal_timeout").value)
        self._gripper_velocity = float(
            self.get_parameter("gripper_velocity").value
        )

        if self._command_rate <= 0.0:
            raise ValueError("command_rate must be positive")
        if self._goal_tolerance <= 0.0:
            raise ValueError("goal_tolerance must be positive")
        if self._goal_timeout <= 0.0:
            raise ValueError("goal_timeout must be positive")
        if self._gripper_velocity <= 0.0:
            raise ValueError("gripper_velocity must be positive")

        callback_group = ReentrantCallbackGroup()
        self._state_lock = threading.RLock()
        self._actual_position = {}
        self._actual_velocity = {}
        self._actual_effort = {}
        self._arm_active = False
        self._gripper_active = False

        self._isaac_command_pub = self.create_publisher(
            JointState,
            str(self.get_parameter("isaac_command_topic").value),
            10,
        )
        self._project_state_pub = self.create_publisher(
            JointState,
            str(self.get_parameter("project_state_topic").value),
            10,
        )
        self._isaac_state_sub = self.create_subscription(
            JointState,
            str(self.get_parameter("isaac_state_topic").value),
            self._isaac_state_callback,
            10,
            callback_group=callback_group,
        )

        self._arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/left_arm_controller/follow_joint_trajectory",
            execute_callback=self._execute_arm,
            goal_callback=self._arm_goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )
        self._gripper_server = ActionServer(
            self,
            ParallelGripperCommand,
            "/left_gripper_controller/gripper_cmd",
            execute_callback=self._execute_gripper,
            goal_callback=self._gripper_goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )

        self.get_logger().info(
            "Isaac backend ready: /isaac/joint_states -> /joint_states, "
            "MoveIt actions -> /isaac/joint_command"
        )

    def destroy_node(self):
        self._arm_server.destroy()
        self._gripper_server.destroy()
        super().destroy_node()

    def _isaac_state_callback(self, message):
        source_position = self._values_by_name(message.name, message.position)
        if not all(name in source_position for name in BY_ISAAC_NAME):
            self.get_logger().warning(
                "Ignoring incomplete Isaac joint state", throttle_duration_sec=5.0
            )
            return

        source_velocity = self._values_by_name(message.name, message.velocity)
        source_effort = self._values_by_name(message.name, message.effort)

        project_position = {}
        project_velocity = {}
        project_effort = {}
        for isaac_name, spec in BY_ISAAC_NAME.items():
            project_position[spec.project_name] = (
                spec.isaac_to_project_position(source_position[isaac_name])
            )
            project_velocity[spec.project_name] = (
                spec.isaac_to_project_rate(
                    source_velocity.get(isaac_name, 0.0)
                )
            )
            project_effort[spec.project_name] = spec.isaac_to_project_rate(
                source_effort.get(isaac_name, 0.0)
            )

        with self._state_lock:
            self._actual_position = project_position
            self._actual_velocity = project_velocity
            self._actual_effort = project_effort

        mapped = JointState()
        mapped.header.stamp = self.get_clock().now().to_msg()
        mapped.header.frame_id = "workcell_base_link"
        mapped.name = list(PROJECT_JOINTS)
        mapped.position = [project_position[name] for name in PROJECT_JOINTS]
        mapped.velocity = [project_velocity[name] for name in PROJECT_JOINTS]
        mapped.effort = [project_effort[name] for name in PROJECT_JOINTS]
        self._project_state_pub.publish(mapped)

    @staticmethod
    def _values_by_name(names, values):
        return {
            name: float(values[index])
            for index, name in enumerate(names)
            if index < len(values) and math.isfinite(float(values[index]))
        }

    def _state_ready(self):
        with self._state_lock:
            return all(name in self._actual_position for name in PROJECT_JOINTS)

    def _arm_goal_callback(self, goal_request):
        error = self._validate_arm_goal(goal_request)
        if error:
            self.get_logger().error(f"Rejecting arm goal: {error}")
            return GoalResponse.REJECT

        with self._state_lock:
            if self._arm_active:
                self.get_logger().warning("Rejecting concurrent arm goal")
                return GoalResponse.REJECT
            self._arm_active = True

        return GoalResponse.ACCEPT

    def _gripper_goal_callback(self, goal_request):
        error = self._validate_gripper_goal(goal_request)
        if error:
            self.get_logger().error(f"Rejecting gripper goal: {error}")
            return GoalResponse.REJECT

        with self._state_lock:
            if self._gripper_active:
                self.get_logger().warning("Rejecting concurrent gripper goal")
                return GoalResponse.REJECT
            self._gripper_active = True

        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def _validate_arm_goal(self, goal_request):
        if not self._state_ready():
            return "no complete /isaac/joint_states message received"

        trajectory = goal_request.trajectory
        if set(trajectory.joint_names) != set(ARM_JOINTS):
            return "trajectory joint names do not match the five left arm joints"
        if len(trajectory.joint_names) != len(ARM_JOINTS):
            return "trajectory contains duplicate joint names"
        if not trajectory.points:
            return "trajectory has no points"

        last_time = -1.0
        for point in trajectory.points:
            if len(point.positions) != len(trajectory.joint_names):
                return "trajectory point has an invalid position count"
            point_time = _duration_seconds(point.time_from_start)
            if point_time < last_time:
                return "trajectory times are not monotonic"
            last_time = point_time
            for name, value in zip(trajectory.joint_names, point.positions):
                if not math.isfinite(value):
                    return f"{name} position is not finite"
                if not BY_PROJECT_NAME[name].contains(value):
                    return f"{name} position {value} is outside URDF limits"
        return ""

    def _validate_gripper_goal(self, goal_request):
        if not self._state_ready():
            return "no complete /isaac/joint_states message received"

        command = goal_request.command
        if not command.position:
            return "gripper command has no position"
        if command.name and GRIPPER_JOINT not in command.name:
            return f"gripper command does not contain {GRIPPER_JOINT}"

        target = self._gripper_target(command)
        if not math.isfinite(target):
            return "gripper position is not finite"
        if not BY_PROJECT_NAME[GRIPPER_JOINT].contains(target):
            return f"gripper position {target} is outside URDF limits"
        return ""

    @staticmethod
    def _gripper_target(command):
        if command.name:
            index = list(command.name).index(GRIPPER_JOINT)
            return float(command.position[index])
        return float(command.position[0])

    def _execute_arm(self, goal_handle):
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory
        names = list(trajectory.joint_names)

        try:
            with self._state_lock:
                previous = {
                    name: self._actual_position[name] for name in names
                }

            previous_time = 0.0
            for point in trajectory.points:
                target = {
                    name: float(value)
                    for name, value in zip(names, point.positions)
                }
                point_time = _duration_seconds(point.time_from_start)
                segment_duration = max(0.0, point_time - previous_time)
                if not self._run_segment(
                    goal_handle,
                    names,
                    previous,
                    target,
                    segment_duration,
                ):
                    goal_handle.canceled()
                    return result
                previous = target
                previous_time = point_time

            target = {
                name: float(value)
                for name, value in zip(
                    names, trajectory.points[-1].positions
                )
            }
            if not self._wait_for_target(target, self._goal_timeout):
                result.error_code = (
                    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                )
                result.error_string = "Isaac joints did not reach goal tolerance"
                goal_handle.abort()
                return result

            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = "Isaac trajectory reached"
            goal_handle.succeed()
            return result
        finally:
            with self._state_lock:
                self._arm_active = False

    def _run_segment(
        self,
        goal_handle,
        names,
        start,
        target,
        duration,
    ):
        if duration <= 0.0:
            self._publish_project_command(target)
            self._publish_arm_feedback(goal_handle, names, target)
            return not goal_handle.is_cancel_requested

        steps = max(1, int(math.ceil(duration * self._command_rate)))
        period = duration / steps
        for step in range(1, steps + 1):
            if goal_handle.is_cancel_requested:
                return False
            alpha = float(step) / float(steps)
            desired = {
                name: start[name] + alpha * (target[name] - start[name])
                for name in names
            }
            self._publish_project_command(desired)
            self._publish_arm_feedback(goal_handle, names, desired)
            time.sleep(period)
        return True

    def _publish_arm_feedback(self, goal_handle, names, desired):
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(names)
        feedback.desired = JointTrajectoryPoint()
        feedback.actual = JointTrajectoryPoint()
        feedback.error = JointTrajectoryPoint()
        feedback.desired.positions = [desired[name] for name in names]

        with self._state_lock:
            actual = [
                self._actual_position.get(name, desired[name]) for name in names
            ]
        feedback.actual.positions = actual
        feedback.error.positions = [
            desired_value - actual_value
            for desired_value, actual_value in zip(
                feedback.desired.positions,
                feedback.actual.positions,
            )
        ]
        goal_handle.publish_feedback(feedback)

    def _execute_gripper(self, goal_handle):
        result = ParallelGripperCommand.Result()
        target = self._gripper_target(goal_handle.request.command)

        try:
            with self._state_lock:
                start = self._actual_position[GRIPPER_JOINT]

            duration = max(
                0.1,
                abs(target - start) / self._gripper_velocity,
            )
            steps = max(1, int(math.ceil(duration * self._command_rate)))
            period = duration / steps
            for step in range(1, steps + 1):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return result
                alpha = float(step) / float(steps)
                desired = start + alpha * (target - start)
                self._publish_project_command({GRIPPER_JOINT: desired})
                self._publish_gripper_feedback(goal_handle)
                time.sleep(period)

            reached = self._wait_for_target(
                {GRIPPER_JOINT: target}, self._goal_timeout
            )
            result.state = self._gripper_state_message()
            result.stalled = False
            result.reached_goal = reached
            if reached:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        finally:
            with self._state_lock:
                self._gripper_active = False

    def _publish_gripper_feedback(self, goal_handle):
        feedback = ParallelGripperCommand.Feedback()
        feedback.state = self._gripper_state_message()
        goal_handle.publish_feedback(feedback)

    def _gripper_state_message(self):
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = [GRIPPER_JOINT]
        with self._state_lock:
            state.position = [
                self._actual_position.get(GRIPPER_JOINT, float("nan"))
            ]
            state.velocity = [
                self._actual_velocity.get(GRIPPER_JOINT, float("nan"))
            ]
            state.effort = [
                self._actual_effort.get(GRIPPER_JOINT, float("nan"))
            ]
        return state

    def _publish_project_command(self, project_positions):
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        for project_name, value in project_positions.items():
            spec = BY_PROJECT_NAME[project_name]
            command.name.append(spec.isaac_name)
            command.position.append(spec.project_to_isaac_position(value))
        self._isaac_command_pub.publish(command)

    def _wait_for_target(self, target, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._state_lock:
                reached = all(
                    abs(self._actual_position.get(name, math.inf) - value)
                    <= self._goal_tolerance
                    for name, value in target.items()
                )
            if reached:
                return True
            time.sleep(1.0 / self._command_rate)
        return False


def main(args=None):
    rclpy.init(args=args)
    node = So101IsaacBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
