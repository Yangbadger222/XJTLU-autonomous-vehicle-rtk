import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("bringup"),
        "config",
        "fgo_gil.yaml",
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Phase 3/4 FGO-GIL shadow frontend parameters",
            ),
            Node(
                package="fgo_gil_localizer",
                executable="fgo_gil_time_sync_node",
                name="fgo_gil_time_sync",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="fgo_gil_localizer",
                executable="fgo_gil_lidar_frontend_node",
                name="fgo_gil_lidar_frontend",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
