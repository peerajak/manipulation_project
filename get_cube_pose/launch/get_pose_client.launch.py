import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    return LaunchDescription(
    [ Node(
        name="get_cube_pose_v2",
        package="get_cube_pose",
        executable="get_pose_client_v2",
        output="screen"
    )]        
    )