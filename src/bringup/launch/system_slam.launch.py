import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
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
    bag_profile = os.environ.get("FYP_SLAM_BAG_PROFILE", "lean").strip().lower()
    bag_topics = [
        "/livox/imu",
        "/fastlio2/lio_odom",
        "/fastlio2/degeneracy",
        "/odom_CBoar",
        "/scan",
        "/map",
        "/pgo/loop_markers",
        "/tf",
        "/tf_static",
    ]
    if bag_profile in {"debug", "full", "raw"}:
        bag_topics.extend(
            [
                "/livox/lidar",
                "/fastlio2/body_cloud",
                "/fastlio2/body_cloud_localization",
            ]
        )
    bag_session_dir = os.environ.get("FYP_LOG_SESSION_DIR")
    bag_output = (
        os.path.join(bag_session_dir, "slam_bag")
        if bag_session_dir
        else os.path.join("/tmp", f"slam_bag_{os.getpid()}")
    )

    use_rtk_arg = DeclareLaunchArgument(
        "use_rtk",
        default_value="false",
        description="Start the UM982 RTK driver during mapping so outdoor fixed samples can be recorded for later geo-registration.",
    )
    record_bag_arg = DeclareLaunchArgument(
        "record_bag",
        default_value=os.environ.get("FYP_SLAM_RECORD_BAG", "true"),
        description="Record the SLAM lean/debug evidence bag for this session.",
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

    slam_params_file = os.path.join(
        bringup_share, "config", "slam_toolbox_mapping.yaml"
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
    bag_recorder = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", bag_output, *bag_topics],
        output="screen",
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )

    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "robot_description.launch.py")
        )
    )

    return LaunchDescription(
        [
            use_rtk_arg,
            record_bag_arg,
            livox_launch,
            rtk_launch,
            fastlio_launch,
            serial_node,
            serial_reader_node,
            pointcloud_to_laserscan_node,
            pgo_node,
            rviz_node,
            delayed_slam,
            bag_recorder,
            urdf_launch,
        ]
    )
