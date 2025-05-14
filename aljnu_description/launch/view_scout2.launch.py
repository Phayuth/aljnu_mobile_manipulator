import os
import launch
import launch_ros

from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution(
            [FindPackageShare("aljnu_description"), "urdf", "scout_v2", 'scout_v2.xacro']
        ),
    ])

    return launch.LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation clock if true'),

        launch.actions.LogInfo(msg='use_sim_time: '),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('use_sim_time')),

        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                'robot_description':robot_description_content
            }]),
    ])
