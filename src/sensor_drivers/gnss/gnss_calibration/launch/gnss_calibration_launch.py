import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_rtk_params = os.path.join(
        get_package_share_directory("um982_rtk_driver"),
        "config",
        "um982_rtk.yaml",
    )
    rtk_launch_path = os.path.join(
        get_package_share_directory("um982_rtk_driver"),
        "launch",
        "um982_rtk.launch.py",
    )
    default_calibration_points = os.path.join(
        get_package_share_directory("gnss_calibration"),
        "config",
        "calibration_points.yaml",
    )

    params_file = LaunchConfiguration("params_file")
    calibration_points_file = LaunchConfiguration("calibration_points_file")

    rtk_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtk_launch_path),
        launch_arguments={"params_file": params_file}.items(),
    )

    gnss_calibration = Node(
        package="gnss_calibration",
        executable="gnss_calibration_node",
        name="gnss_calibration",
        output="screen",
        parameters=[{"calibration_points_file": calibration_points_file}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_rtk_params,
                description="ROS2 parameter file used by um982_rtk_driver",
            ),
            DeclareLaunchArgument(
                "calibration_points_file",
                default_value=default_calibration_points,
                description="Calibration point YAML used by gnss_calibration",
            ),
            rtk_driver,
            gnss_calibration,
        ]
    )
