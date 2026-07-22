import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """
    Survey Mode launch file.
    Starts LIDAR, PGO, Navigation, Localizer, and Survey state machine.
    """

    bringup_share = get_package_share_directory("bringup")
    default_master_params_file = os.path.join(bringup_share, "config", "master_params.yaml")
    default_nav2_survey_params_file = os.path.join(bringup_share, "config", "nav2_survey.yaml")
    default_survey_params_file = os.path.join(get_package_share_directory("survey_mode"), "config", "survey_mode.yaml")
    
    corridor_bt_xml = os.path.join(
        bringup_share,
        "behavior_trees",
        "navigate_to_pose_w_replanning_3hz_and_recovery.xml",
    )
    default_nav_through_poses_bt_xml = os.path.join(
        get_package_share_directory("nav2_bt_navigator"),
        "behavior_trees",
        "navigate_through_poses_w_replanning_and_recovery.xml",
    )

    rewritten_nav2_params = RewrittenYaml(
        source_file=default_nav2_survey_params_file,
        param_rewrites={
            "default_nav_to_pose_bt_xml": corridor_bt_xml,
            "default_nav_through_poses_bt_xml": default_nav_through_poses_bt_xml,
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

    pgo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("pgo"), "launch", "pgo_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "params_file": default_master_params_file,
            "pgo_config": "",
            "extra_params_file": "",
            "use_rviz": "false",
            "rviz_config": "",
        }.items(),
    )

    serial_node = launch_ros.actions.Node(
        package="serial_twistctl",
        executable="serial_twistctl_node",
        name="serial_twistctl_node",
        output="screen",
        parameters=[default_master_params_file],
    )

    serial_reader_node = launch_ros.actions.Node(
        package="serial_reader",
        executable="serial_reader_node",
        name="serial_reader_node",
        output="screen",
        parameters=[default_master_params_file],
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
            "params_file": rewritten_nav2_params,
        }.items(),
    )

    delayed_nav2 = TimerAction(period=5.0, actions=[nav2_launch])

    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'robot_description.launch.py')
        )
    )

    survey_node = Node(
        package="survey_mode",
        executable="survey_node",
        name="survey_node",
        output="screen",
        parameters=[rewritten_nav2_params, default_survey_params_file],
        on_exit=launch.actions.EmitEvent(event=launch.events.Shutdown())
    )

    localizer_config_path = PathJoinSubstitution(
        [FindPackageShare("localizer"), "config", "localizer.yaml"]
    )
    
    localizer_node = Node(
        package="localizer",
        namespace="localizer",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[
            {
                "config_path": localizer_config_path,
                "pcd_map": "",
                "alignment_file": "",
                "descriptor_index": "",
                "map_id": "",
                "auto_global_localization": False,
                "publish_tf": False,
            }
        ]
    )

    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[{"port": 8765}]
    )

    return LaunchDescription(
        [
            livox_launch,
            pgo_launch,
            serial_node,
            serial_reader_node,
            delayed_nav2,
            urdf_launch,
            survey_node,
            localizer_node,
            foxglove_bridge_node,
        ]
    )
