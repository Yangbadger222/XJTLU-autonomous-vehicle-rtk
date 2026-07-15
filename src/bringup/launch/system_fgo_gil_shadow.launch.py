import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _bag_topics(profile):
    outputs = [
        "/fgo_gil/odom",
        "/fgo_gil/path",
        "/fgo_gil/float_odom_ecef",
        "/fgo_gil/fixed_odom_ecef",
        "/fgo_gil/float_diagnostics",
        "/fgo_gil/factor_diagnostics",
        "/fgo_gil/ambiguity_status",
        "/fgo_gil/timing_status",
        "/fgo_gil/performance",
    ]
    minimal = [
        "/gnss/raw/observation_epoch",
        "/gnss/raw/ephemeris",
        "/livox/imu",
        "/fastlio2/lio_odom",
        "/fgo_gil/lidar_constraints",
    ] + outputs
    if profile == "minimal":
        return minimal
    if profile != "full":
        raise RuntimeError("bag_profile must be 'minimal' or 'full'")
    return [
        "/clock",
        "/gnss/raw/frame",
        "/gnss/raw/observation_epoch",
        "/gnss/raw/ephemeris",
        "/gnss/raw/diagnostics",
        "/gnss/pps/time_reference",
        "/livox/lidar",
        "/livox/imu",
        "/livox/imu_time_reference",
        "/livox/lidar_time_reference",
        "/fastlio2/lio_odom",
        "/fastlio2/body_cloud",
        "/fgo_gil/lidar_constraints",
        "/fgo_gil/lidar_diagnostics",
        "/fgo_gil/time_sync_diagnostics",
        "/tf",
        "/tf_static",
    ] + outputs


def _validate_shadow_safety(context):
    publish_tf = LaunchConfiguration("publish_tf").perform(context)
    nav2_use_fgo = LaunchConfiguration("nav2_use_fgo").perform(context)
    if _as_bool(publish_tf) or _as_bool(nav2_use_fgo):
        raise RuntimeError(
            "FGO-GIL Phase 7 is shadow-only; publish_tf and nav2_use_fgo must be false"
        )
    return []


def _record_bag(context):
    if not _as_bool(LaunchConfiguration("record_bag").perform(context)):
        return []
    profile = LaunchConfiguration("bag_profile").perform(context)
    session_data_dir = os.environ.get("FYP_LOG_SESSION_DIR", "")
    if session_data_dir:
        session_root = os.path.dirname(session_data_dir)
    else:
        session_root = os.path.expanduser(
            "~/XJTLU-autonomous-vehicle/runtime-data/logs/"
            + datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        )
    bag_dir = os.path.join(session_root, "bag", "fgo_gil")
    os.makedirs(os.path.dirname(bag_dir), exist_ok=True)
    return [
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "--storage",
                "sqlite3",
                "--output",
                bag_dir,
            ]
            + _bag_topics(profile),
            output="log",
        )
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    fgo_params = os.path.join(bringup_share, "config", "fgo_gil.yaml")
    um982_share = get_package_share_directory("um982_rtk_driver")
    um982_params = os.path.join(um982_share, "config", "um982_mixed.yaml")
    livox_share = get_package_share_directory("livox_ros_driver2")
    fastlio_share = get_package_share_directory("fastlio2")
    fastlio_config = os.path.join(fastlio_share, "config", "lio.yaml")

    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, "launch_ROS2", "msg_MID360_launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("start_livox")),
    )
    fastlio = Node(
        package="fastlio2",
        namespace="fastlio2",
        executable="lio_node",
        name="lio_node",
        output="screen",
        parameters=[
            {"config_path": fastlio_config},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        condition=IfCondition(LaunchConfiguration("start_fastlio")),
    )
    um982_driver = Node(
        package="um982_rtk_driver",
        executable="um982_rtk_node",
        name="um982_rtk_driver",
        output="screen",
        parameters=[
            LaunchConfiguration("um982_params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        condition=IfCondition(LaunchConfiguration("start_um982_driver")),
    )
    fgo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "system_fgo_gil_float.launch.py")
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("fgo_params_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "publish_tf": LaunchConfiguration("publish_tf"),
            "nav2_use_fgo": LaunchConfiguration("nav2_use_fgo"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("fgo_params_file", default_value=fgo_params),
            DeclareLaunchArgument("um982_params_file", default_value=um982_params),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_livox", default_value="true"),
            DeclareLaunchArgument("start_fastlio", default_value="true"),
            DeclareLaunchArgument("start_um982_driver", default_value="true"),
            DeclareLaunchArgument("record_bag", default_value="true"),
            DeclareLaunchArgument("bag_profile", default_value="full"),
            DeclareLaunchArgument("publish_tf", default_value="false"),
            DeclareLaunchArgument("nav2_use_fgo", default_value="false"),
            OpaqueFunction(function=_validate_shadow_safety),
            livox,
            fastlio,
            um982_driver,
            fgo,
            OpaqueFunction(function=_record_bag),
        ]
    )
