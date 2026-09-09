from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("so101_description"), "urdf", "so101_left.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("so101_bringup"), "config", "ros2_controllers.yaml"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    xacro_file,
                    " arm_slot:=left use_mock_hardware:=true",
                ]
            ),
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_robot_state_publisher",
                default_value="true",
                description=(
                    "Start robot_state_publisher when this launch is used "
                    "standalone"
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                condition=IfCondition(
                    LaunchConfiguration("start_robot_state_publisher")
                ),
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[controllers_file],
                remappings=[
                    (
                        "/controller_manager/robot_description",
                        "/robot_description",
                    )
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "left_arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "left_gripper_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
        ]
    )
