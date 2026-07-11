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
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """
    Travel mode: prior-map navigation.

    TF ownership:
    - localizer publishes map -> odom after /localizer/relocalize loads a PCD map
    - FAST-LIO2 publishes odom -> base_footprint, with URDF static TF to base_link
    - PGO is optional and must not publish TF in this mode
    """

    bringup_share = get_package_share_directory("bringup")
    default_master_params_file = os.path.join(bringup_share, "config", "master_params.yaml")
    nav2_params_file = os.path.join(bringup_share, "config", "nav2_travel.yaml")
    default_rviz_config = os.path.join(bringup_share, "rviz", "pgo.rviz")
    travel_bt_xml = os.path.join(
        bringup_share,
        "behavior_trees",
        "travel_nav_to_pose_fail_stop.xml",
    )
    travel_through_poses_bt_xml = os.path.join(
        bringup_share,
        "behavior_trees",
        "travel_nav_through_poses_fail_stop.xml",
    )
    localizer_config_path = PathJoinSubstitution(
        [FindPackageShare("localizer"), "config", "localizer.yaml"]
    )
    pgo_no_tf_config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo_slam.yaml"]
    )

    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml",
        default_value="",
        description="Absolute path to the Nav2 2D occupancy-grid map YAML.",
    )
    pcd_map_arg = DeclareLaunchArgument(
        "pcd_map",
        default_value="",
        description="Absolute path to the prior PCD map used by /localizer/relocalize.",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to launch RViz with the travel stack.",
    )
    use_pgo_arg = DeclareLaunchArgument(
        "use_pgo",
        default_value="false",
        description="Whether to launch PGO without TF for map visualization/save-map support.",
    )
    master_params_arg = DeclareLaunchArgument(
        "master_params_file",
        default_value=default_master_params_file,
        description="ROS2 parameter file used by FAST-LIO2, pointcloud conversion, and serial nodes.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="RViz layout used by the prior-map travel stack.",
    )

    rewritten_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            "yaml_filename": LaunchConfiguration("map_yaml"),
            "default_nav_to_pose_bt_xml": travel_bt_xml,
            "default_nav_through_poses_bt_xml": travel_through_poses_bt_xml,
        },
        convert_types=True,
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
        launch_arguments={"params_file": LaunchConfiguration("master_params_file")}.items(),
    )

    pgo_node = launch_ros.actions.Node(
        package="pgo",
        executable="pgo_node",
        name="pgo_node",
        output="screen",
        parameters=[{"config_path": pgo_no_tf_config_path}],
        condition=IfCondition(LaunchConfiguration("use_pgo")),
    )

    localizer_node = launch_ros.actions.Node(
        package="localizer",
        namespace="localizer",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[
            {
                "config_path": localizer_config_path,
                "pcd_map": LaunchConfiguration("pcd_map"),
            }
        ],
    )

    initialpose_relocalize_bridge_node = launch_ros.actions.Node(
        package="bringup",
        executable="initialpose_relocalize_bridge.py",
        name="initialpose_relocalize_bridge",
        output="screen",
        parameters=[
            {
                "pcd_map": LaunchConfiguration("pcd_map"),
            }
        ],
    )

    nav2_cloud_retime_node = launch_ros.actions.Node(
        package="bringup",
        executable="nav2_cloud_retime.py",
        name="nav2_cloud_retime",
        output="screen",
        remappings=[
            ("cloud_in", "/fastlio2/body_cloud_nav2_obstacles"),
            ("cloud_out", "/fastlio2/body_cloud_nav2"),
        ],
    )

    serial_node = launch_ros.actions.Node(
        package="serial_twistctl",
        executable="serial_twistctl_node",
        name="serial_twistctl_node",
        output="screen",
        parameters=[LaunchConfiguration("master_params_file")],
    )

    serial_reader_node = launch_ros.actions.Node(
        package="serial_reader",
        executable="serial_reader_node",
        name="serial_reader_node",
        output="screen",
        parameters=[LaunchConfiguration("master_params_file")],
    )

    pointcloud_to_laserscan_node = launch_ros.actions.Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[LaunchConfiguration("master_params_file")],
        remappings=[
            ("cloud_in", "/fastlio2/body_cloud"),
            ("scan", "/scan"),
        ],
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("nav2_bringup"), "launch", "localization_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "map": LaunchConfiguration("map_yaml"),
            "use_sim_time": "false",
            "params_file": rewritten_nav2_params,
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": rewritten_nav2_params,
        }.items(),
    )
    delayed_nav2 = TimerAction(period=5.0, actions=[localization_launch, navigation_launch])

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="travel_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'robot_description.launch.py')
        )
    )

    return LaunchDescription(
        [
            map_yaml_arg,
            pcd_map_arg,
            use_rviz_arg,
            use_pgo_arg,
            master_params_arg,
            rviz_config_arg,
            livox_launch,
            fastlio_launch,
            pgo_node,
            localizer_node,
            initialpose_relocalize_bridge_node,
            nav2_cloud_retime_node,
            serial_node,
            serial_reader_node,
            pointcloud_to_laserscan_node,
            delayed_nav2,
            urdf_launch,
            rviz_node,
        ]
    )
