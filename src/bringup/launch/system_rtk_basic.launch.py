import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("um982_rtk_driver"),
        "config",
        "um982_rtk.yaml",
    )
    params_file = LaunchConfiguration("params_file")

    rtk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("um982_rtk_driver"), "launch", "um982_rtk.launch.py"]
                )
            ]
        ),
        launch_arguments={"params_file": params_file}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="ROS 2 parameter file for UM982 RTK basic bring-up",
            ),
            rtk_launch,
        ]
    )
