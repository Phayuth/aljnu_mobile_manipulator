from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('aljnu_controllers'),
        'config',
        'position_controller.yaml'
    )

    return LaunchDescription([
        Node(
            package='aljnu_controllers',
            executable='position_controller',
            name='position_controller',
            parameters=[config_file]
        )
    ])