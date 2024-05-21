import os
import sys
from glob import glob
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription


def generate_launch_description():
    pkg_dir = get_package_share_directory('manual_controller')
    rviz_config_file = os.path.join(pkg_dir, 'rviz','deck.rviz')
    list = [
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
        ),
        Node(
            package='manual_controller',
            executable='manual_controller',
            namespace='',
            output="screen",
            parameters=[os.path.join(pkg_dir, "config", "steamdeck_manual_controller_param.yaml")],
            respawn=True,
        ),
        Node(
            package='joy',
            executable='joy_node',
            namespace='',
            output="screen",
            respawn=True,
        ),
    ]

    return LaunchDescription(list)