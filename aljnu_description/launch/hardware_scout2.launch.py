import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("aljnu_description"),
        "config",
        "scout2",
        "scout2.yaml",
    )
    scout_base_node = Node(
        package="scout_base",
        executable="scout_base_node",
        name="scout_base_node",
        output="screen",
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription(
        [
            scout_base_node,
        ]
    )
