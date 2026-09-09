from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory("manipulation_camera_manager")
    return LaunchDescription(
        [
            Node(
                package="manipulation_camera_manager",
                executable="camera_manager_node",
                name="camera_manager",
                output="screen",
                parameters=[os.path.join(share, "config", "cameras.yaml")],
            )
        ]
    )
