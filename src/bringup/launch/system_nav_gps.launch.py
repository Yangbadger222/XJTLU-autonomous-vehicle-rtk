import os
import tempfile
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_NAV_GPS_BAG_BASE_TOPICS = [
    "/fix",
    "/heading",
    "/rtk/status",
    "/rtk/nmea_sentence",
    "/fastlio2/lio_odom",
    "/livox/imu",
    "/odom_CBoar",
    "/tf",
    "/tf_static",
    "/gps_system/status",
    "/gps_system/nearest_anchor",
    "/gps_system/nearest_anchor_id",
    "/gps_goal_manager/status",
    "/gps_waypoint_dispatcher/goal_map",
    "/gps_waypoint_dispatcher/path_map",
    "/localization_authority/mode",
    "/localization_authority/status",
    "/localization_authority/diagnostics",
    "/rtk_fgo/odom",
    "/rtk_fgo/path",
    "/rtk_fgo/status",
    "/rtk_fgo/rtk_gate",
    "/rtk_fgo/correction_status",
    "/rtk_fgo/factor_diagnostics",
    "/cmd_vel",
    "/local_costmap/costmap",
    "/global_costmap/costmap",
    "/plan",
]

_NAV_GPS_BAG_DEBUG_TOPICS = [
    "/livox/lidar",
    "/fastlio2/body_cloud",
    "/fastlio2/body_cloud_nav2_obstacles",
]


def _nav_gps_bag_topics(profile):
    normalized = (profile or "lean").strip().lower()
    topics = list(_NAV_GPS_BAG_BASE_TOPICS)
    if normalized in {"debug", "full", "raw"}:
        topics.extend(_NAV_GPS_BAG_DEBUG_TOPICS)
    return topics


def _make_nav_gps_rtk_nav2_params(source_file):
    with open(source_file, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    bt_params = data["bt_navigator"]["ros__parameters"]
    bt_params["bt_loop_duration"] = 50
    bt_params["default_server_timeout"] = 1000

    controller_params = data["controller_server"]["ros__parameters"]
    controller_params["controller_frequency"] = 20.0
    controller_params["failure_tolerance"] = 1.5
    controller_params["progress_checker"]["required_movement_radius"] = 0.10
    controller_params["progress_checker"]["movement_time_allowance"] = 15.0
    controller_params["general_goal_checker"]["stateful"] = False

    follow_path = controller_params["FollowPath"]
    follow_path["batch_size"] = 500
    follow_path["vx_std"] = 0.20
    follow_path["wz_std"] = 0.15
    follow_path["vx_max"] = 0.85
    follow_path["wz_max"] = 0.70
    follow_path["ax_max"] = 0.85
    follow_path["ax_min"] = -1.2
    follow_path["az_max"] = 1.4
    follow_path["temperature"] = 0.45
    follow_path["regenerate_noises"] = True

    smoother_params = data["velocity_smoother"]["ros__parameters"]
    smoother_params["max_velocity"] = [0.85, 0.0, 0.70]
    smoother_params["min_velocity"] = [0.0, 0.0, -0.70]
    smoother_params["max_accel"] = [0.85, 0.0, 1.4]
    smoother_params["max_decel"] = [-1.2, 0.0, -1.8]

    behavior_params = data["behavior_server"]["ros__parameters"]
    behavior_params["behavior_plugins"] = ["wait"]

    rewritten = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="xjtlu_nav_gps_rtk_nav2_",
        suffix=".yaml",
        delete=False,
    )
    yaml.safe_dump(data, rewritten, sort_keys=False)
    rewritten.close()
    return rewritten.name


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    runtime_scene_dir = os.path.expanduser("~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_scene")

    default_scene_params = os.path.join(runtime_scene_dir, "master_params_scene.yaml")
    default_scene_points = os.path.join(runtime_scene_dir, "scene_points.yaml")
    default_route_graph = os.path.join(runtime_scene_dir, "scene_route_graph.geojson")
    default_nav2_params = os.path.join(bringup_share, "config", "nav2_corridor_rtk.yaml")
    pgo_nav_gps_config_file = os.path.join(bringup_share, "config", "pgo_corridor_no_tf.yaml")
    pgo_nav_gps_override_file = os.path.join(bringup_share, "config", "pgo_corridor_no_gps.yaml")
    rtk_fgo_params_file = os.path.join(bringup_share, "config", "rtk_fgo.yaml")
    nav_gps_rtk_nav2_params = _make_nav_gps_rtk_nav2_params(default_nav2_params)
    nav_gps_no_recovery_bt_xml = os.path.join(
        bringup_share,
        "behavior_trees",
        "navigate_to_pose_w_replanning_5hz_no_motion_recovery.xml",
    )
    nav_gps_no_recovery_through_poses_bt_xml = os.path.join(
        bringup_share,
        "behavior_trees",
        "navigate_through_poses_w_replanning_5hz_no_motion_recovery.xml",
    )

    params_file = LaunchConfiguration("params_file")
    rtk_params_file = LaunchConfiguration("rtk_params_file")
    use_rviz = LaunchConfiguration("use_rviz")
    scene_points_file = LaunchConfiguration("scene_points_file")
    route_graph_file = LaunchConfiguration("route_graph_file")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_scene_params,
        description="Runtime scene parameter file generated by build_scene_runtime.py",
    )
    rtk_params_file_arg = DeclareLaunchArgument(
        "rtk_params_file",
        default_value=default_scene_params,
        description="Parameter file used only by um982_rtk_driver",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Whether to launch RViz together with the nav-gps stack",
    )
    scene_points_arg = DeclareLaunchArgument(
        "scene_points_file",
        default_value=default_scene_points,
        description="Compiled scene_points.yaml generated by build_scene_runtime.py",
    )
    route_graph_arg = DeclareLaunchArgument(
        "route_graph_file",
        default_value=default_route_graph,
        description="Compiled scene_route_graph.geojson generated by build_scene_runtime.py",
    )
    enable_fgo_shadow_arg = DeclareLaunchArgument(
        "enable_fgo_shadow",
        default_value=os.environ.get("FYP_NAV_GPS_ENABLE_FGO_SHADOW", "true"),
        description="Start RTK FGO in shadow mode for nav-gps rosbag evidence",
    )

    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "system_explore.launch.py")
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "master_params_file": params_file,
            "pgo_config_file": pgo_nav_gps_config_file,
            "pgo_extra_params_file": pgo_nav_gps_override_file,
            "nav2_params_file": nav_gps_rtk_nav2_params,
            "nav_to_pose_bt_xml": nav_gps_no_recovery_bt_xml,
            "nav_through_poses_bt_xml": nav_gps_no_recovery_through_poses_bt_xml,
        }.items(),
    )

    rtk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("um982_rtk_driver"), "launch", "um982_rtk.launch.py"])]
        ),
        launch_arguments={"params_file": rtk_params_file}.items(),
    )

    anchor_localizer_node = Node(
        package="gnss_calibration",
        executable="gps_anchor_localizer_node",
        name="gps_anchor_localizer",
        output="screen",
        parameters=[params_file, {"scene_points_file": scene_points_file}],
    )

    route_server_node = Node(
        package="nav2_route",
        executable="route_server",
        name="route_server",
        output="screen",
        parameters=[
            {
                "route_frame": "map",
                "global_frame": "map",
                "base_frame": "base_link",
                "graph_filepath": route_graph_file,
                "enable_nn_search": True,
                "path_density": 0.2,
                "smooth_corners": False,
            }
        ],
    )

    route_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="route_lifecycle_manager",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "node_names": ["route_server"],
            }
        ],
    )

    goal_manager_node = Node(
        package="gps_waypoint_dispatcher",
        executable="goal_manager_node",
        name="gps_waypoint_dispatcher",
        output="screen",
        parameters=[
            params_file,
            {
                "scene_points_file": scene_points_file,
                "use_route_pose_start": True,
                "require_nav_ready": False,
            },
        ],
    )

    rtk_authority = Node(
        package="gps_waypoint_dispatcher",
        executable="rtk_map_odom_corrector_node",
        name="rtk_map_odom_corrector",
        output="screen",
        parameters=[
            params_file,
            {
                "fix_topic": "/fix",
                "heading_topic": "/heading",
                "rtk_status_topic": "/rtk/status",
                "alignment_topic": "/gps_scene/enu_to_map",
                "scene_points_file": scene_points_file,
                "use_scene_identity_alignment": True,
            },
        ],
    )

    fgo_shadow = Node(
        package="rtk_fgo_localizer",
        executable="rtk_fgo_node",
        name="rtk_fgo_localizer",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_fgo_shadow")),
        parameters=[
            rtk_fgo_params_file,
            {
                "publish_tf": False,
                "nav2_use_fgo": False,
            },
        ],
    )

    session_data_dir = os.environ.get("FYP_LOG_SESSION_DIR", "")
    if session_data_dir:
        session_root = os.path.dirname(session_data_dir)
    else:
        session_root = os.path.expanduser(
            f"~/XJTLU-autonomous-vehicle/runtime-data/logs/{datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}"
        )
    bag_dir = os.path.join(session_root, "bag")
    os.makedirs(session_root, exist_ok=True)
    bag_profile = os.environ.get("FYP_NAV_GPS_BAG_PROFILE", "lean")

    bag_record = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            bag_dir,
        ] + _nav_gps_bag_topics(bag_profile),
        output="log",
    )

    delayed_rtk_authority = TimerAction(period=3.0, actions=[rtk_authority])
    delayed_anchor_localizer = TimerAction(period=4.0, actions=[anchor_localizer_node])
    delayed_fgo_shadow = TimerAction(period=6.0, actions=[fgo_shadow])
    delayed_goal_manager = TimerAction(period=7.0, actions=[goal_manager_node])

    return LaunchDescription(
        [
            params_file_arg,
            rtk_params_file_arg,
            use_rviz_arg,
            scene_points_arg,
            route_graph_arg,
            enable_fgo_shadow_arg,
            explore_launch,
            rtk_launch,
            route_server_node,
            route_lifecycle_manager,
            LogInfo(msg=f"Nav GPS bag profile: {bag_profile}"),
            bag_record,
            delayed_rtk_authority,
            delayed_anchor_localizer,
            delayed_fgo_shadow,
            delayed_goal_manager,
        ]
    )
