import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("um982_rtk_driver"),
        "config",
        "um982_mixed.yaml",
    )

    raw_node = Node(
        package="um982_rtk_driver",
        executable="um982_rtk_node",
        name="um982_rtk_driver",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    session_data_dir = os.environ.get("FYP_LOG_SESSION_DIR", "")
    if session_data_dir:
        session_root = os.path.dirname(session_data_dir)
    else:
        session_root = os.path.expanduser(
            "~/XJTLU-autonomous-vehicle/runtime-data/logs/"
            + datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        )
    bag_dir = os.path.join(session_root, "bag")
    os.makedirs(session_root, exist_ok=True)

    bag_record = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            bag_dir,
            "/gnss/raw/frame",
            "/gnss/raw/observation_epoch",
            "/gnss/raw/ephemeris",
            "/gnss/raw/diagnostics",
            "/fix",
            "/heading",
            "/rtk/status",
            "/rtk/nmea_sentence",
        ],
        output="log",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Parameters for the unified UM982 mixed stream",
            ),
            raw_node,
            bag_record,
        ]
    )
