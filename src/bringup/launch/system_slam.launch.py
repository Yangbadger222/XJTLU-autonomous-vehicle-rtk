import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    SLAM 专用 launch 文件：
    1. 同步启动传感器层（Livox）、底盘串口以及点云转激光节点
    2. 延时 5 秒后启动 SLAM Toolbox + Map Saver
    """

    bringup_share = get_package_share_directory("bringup")
    master_params_file = os.path.join(bringup_share, "config", "master_params.yaml")

    use_rtk_arg = DeclareLaunchArgument(
        "use_rtk",
        default_value="false",
        description="Start the UM982 RTK driver during mapping so outdoor fixed samples can be recorded for later geo-registration.",
    )

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("livox_ros_driver2"),
                        "launch_ROS2",
                        "msg_MID360_launch.py",
                    ]
                )
            ]
        )
    )

    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("fastlio2"), "launch", "lio_no_rviz.py"]
                )
            ]
        ),
        launch_arguments={"params_file": master_params_file}.items(),
    )

    rtk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("um982_rtk_driver"), "launch", "um982_rtk.launch.py"]
                )
            ]
        ),
        condition=IfCondition(LaunchConfiguration("use_rtk")),
    )

    serial_node = launch_ros.actions.Node(
        package="serial_twistctl",
        executable="serial_twistctl_node",
        name="serial_twistctl_node",
        output="screen",
        parameters=[master_params_file],
    )

    serial_reader_node = launch_ros.actions.Node(
        package="serial_reader",
        executable="serial_reader_node",
        name="serial_reader_node",
        output="screen",
        parameters=[master_params_file],
    )

    pointcloud_to_laserscan_node = launch_ros.actions.Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[master_params_file],
        remappings=[
            ("cloud_in", "/fastlio2/body_cloud"),
            ("scan", "/scan"),
        ],
    )

    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo_slam.yaml"]
    )
    pgo_node = launch_ros.actions.Node(
        package="pgo",
        executable="pgo_node",
        name="pgo_node",
        output="screen",
        parameters=[
            {"config_path": pgo_config_path.perform(launch.LaunchContext())}
        ],
    )

    slam_rviz_config = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "config", "slam_toolbox_default.rviz"]
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="slam_rviz",
        arguments=["-d", slam_rviz_config],
        output="screen",
    )

    slam_toolbox_dir = get_package_share_directory("slam_toolbox")
    slam_params_file = os.path.join(
        slam_toolbox_dir, "config", "mapper_params_online_async.yaml"
    )
    slam_toolbox_node = launch_ros.actions.Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {
                "base_frame": "base_link",
                "scan_topic": "/scan",
                "transform_timeout": 1.0,
                "tf_buffer_duration": 60.0,
            },
        ],
    )
    map_saver_server = launch_ros.actions.Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[{"save_map_timeout": 5000.0}],
    )
    lifecycle_manager_mapping = launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_mapping",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["map_saver_server"]}],
    )

    delayed_slam = TimerAction(
        period=5.0,
        actions=[slam_toolbox_node, map_saver_server, lifecycle_manager_mapping],
    )

    return LaunchDescription(
        [
            use_rtk_arg,
            livox_launch,
            rtk_launch,
            fastlio_launch,
            serial_node,
            serial_reader_node,
            pointcloud_to_laserscan_node,
            pgo_node,
            rviz_node,
            delayed_slam,
        ]
    )
