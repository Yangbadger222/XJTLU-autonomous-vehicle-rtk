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
    use_sim_time = LaunchConfiguration("use_sim_time")
    common_parameters = [params_file, {"use_sim_time": use_sim_time}]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Phase 3-6 FGO-GIL shadow estimator parameters",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock for deterministic bag replay",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="false",
                description="Forbidden Phase 7 FGO-GIL TF ownership flag",
            ),
            DeclareLaunchArgument(
                "nav2_use_fgo",
                default_value="false",
                description="Forbidden Phase 7 Nav2 ownership flag",
            ),
            Node(
                package="fgo_gil_localizer",
                executable="fgo_gil_time_sync_node",
                name="fgo_gil_time_sync",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="fgo_gil_localizer",
                executable="fgo_gil_lidar_frontend_node",
                name="fgo_gil_lidar_frontend",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="fgo_gil_localizer",
                executable="fgo_gil_float_fgo_node",
                name="fgo_gil_float_fgo",
                output="screen",
                parameters=common_parameters
                + [
                    {
                        "safety.publish_tf": LaunchConfiguration("publish_tf"),
                        "safety.nav2_use_fgo": LaunchConfiguration("nav2_use_fgo"),
                    }
                ],
            ),
        ]
    )
