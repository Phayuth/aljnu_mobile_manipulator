from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("aljnu_controllers"),
        "config",
        "backstepping_controller.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="aljnu_controllers",
                executable="backstepping_controller",
                name="backstepping_controller",
                parameters=[config_file],
            )
        ]
    )
