import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    系统完整启动 launch 文件 (支持建图/导航模式切换)
    """

    bringup_share = get_package_share_directory("bringup")
    master_params_file = os.path.join(bringup_share, "config", "master_params.yaml")

    mapping_mode_arg = DeclareLaunchArgument(
        "mapping_mode",
        default_value="false",
        description="是否启用建图模式 (true=建图, false=导航)",
    )
    mapping_mode = LaunchConfiguration("mapping_mode")

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

    pgo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("pgo"), "launch", "pgo_launch.py"])]
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
        condition=IfCondition(mapping_mode),
    )
    map_saver_server = launch_ros.actions.Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[{"save_map_timeout": 5000.0}],
        condition=IfCondition(mapping_mode),
    )
    lifecycle_manager_mapping = launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_mapping",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["map_saver_server"]}],
        condition=IfCondition(mapping_mode),
    )

    delayed_slam = TimerAction(
        period=5.0,
        actions=[slam_toolbox_node, map_saver_server, lifecycle_manager_mapping],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": os.path.join(bringup_share, "config", "nav2_travel.yaml"),
        }.items(),
        condition=UnlessCondition(mapping_mode),
    )

    delayed_nav2 = TimerAction(period=5.0, actions=[nav2_launch])

    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'robot_description.launch.py')
        )
    )

    return LaunchDescription(
        [
            mapping_mode_arg,
            livox_launch,
            pgo_launch,
            serial_node,
            serial_reader_node,
            pointcloud_to_laserscan_node,
            delayed_slam,
            delayed_nav2,
            urdf_launch,
        ]
    )
