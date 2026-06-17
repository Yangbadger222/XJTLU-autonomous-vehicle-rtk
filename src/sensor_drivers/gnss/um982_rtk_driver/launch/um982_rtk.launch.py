import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("um982_rtk_driver"),
        "config",
        "um982_rtk.yaml",
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="ROS 2 parameter file for the UM982 RTK driver",
            ),
            Node(
                package="um982_rtk_driver",
                executable="um982_rtk_node",
                name="um982_rtk_driver",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
