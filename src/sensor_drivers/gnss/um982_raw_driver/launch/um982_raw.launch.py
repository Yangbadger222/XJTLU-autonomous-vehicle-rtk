import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("um982_raw_driver"),
        "config",
        "um982_raw.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Parameters for the dedicated UM982 binary raw port",
            ),
            Node(
                package="um982_raw_driver",
                executable="um982_raw_node",
                name="um982_raw_driver",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
