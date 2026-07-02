import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    SLAM 专用 launch 文件：
    1. 同步启动传感器层（Livox）、底盘串口以及点云转激光节点
    2. 延时 5 秒后启动 SLAM Toolbox + Map Saver
    """

    bringup_share = get_package_share_directory("bringup")
    master_params_file = os.path.join(bringup_share, "config", "master_params.yaml")

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
        parameters=[slam_params_file],
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
    
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'robot_description.launch.py')
        )
    )

    return LaunchDescription(
        [
            livox_launch,
            fastlio_launch,
            serial_node,
            serial_reader_node,
            pointcloud_to_laserscan_node,
            rviz_node,
            delayed_slam,
            urdf_launch,
        ]
    )
