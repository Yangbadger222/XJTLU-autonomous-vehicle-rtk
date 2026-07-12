import os
from pathlib import Path

import yaml

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def _resolve_map_bundle(context):
    bundle_value = LaunchConfiguration("map_bundle").perform(context).strip()
    if not bundle_value:
        return []
    bundle_path = Path(bundle_value).expanduser().resolve()
    manifest_path = bundle_path / "manifest.yaml" if bundle_path.is_dir() else bundle_path
    if not manifest_path.exists():
        raise RuntimeError(f"Indoor map bundle manifest not found: {manifest_path}")
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise RuntimeError("Indoor map bundle manifest must be a mapping")
    if int(manifest.get("schema_version", 0)) != 1:
        raise RuntimeError("Unsupported indoor map bundle schema")
    if not bool(manifest.get("consistency_ok", False)):
        raise RuntimeError("Indoor map bundle did not pass consistency gates")
    if not bool(manifest.get("calibration", {}).get("accepted", False)):
        raise RuntimeError("Indoor map bundle calibration is not accepted")
    map_id = str(manifest.get("map_id", manifest.get("map_name", ""))).strip()
    if not map_id:
        raise RuntimeError("Indoor map bundle is missing map_id")
    root = manifest_path.parent
    artifacts = manifest.get("artifacts", {})
    if not isinstance(artifacts, dict):
        raise RuntimeError("Indoor map bundle artifacts must be a mapping")
    required = {
        "map_yaml": artifacts.get("navigation_map"),
        "pcd_map": artifacts.get("localization_map"),
        "map_alignment_file": artifacts.get("alignment"),
        "descriptor_index": artifacts.get("descriptor_index"),
        "destinations_file": artifacts.get("destinations"),
    }
    for launch_name, relative_path in required.items():
        if not relative_path:
            raise RuntimeError(f"Indoor map bundle is missing artifact: {launch_name}")
        resolved = (root / str(relative_path)).resolve()
        try:
            resolved.relative_to(root)
        except ValueError as exc:
            raise RuntimeError(
                f"Indoor map bundle artifact escapes bundle root: {relative_path}"
            ) from exc
        if not resolved.exists():
            raise RuntimeError(f"Indoor map bundle artifact not found: {resolved}")
        context.launch_configurations[launch_name] = str(resolved)
    context.launch_configurations["map_id"] = map_id
    return [LogInfo(msg=f"Loaded indoor map bundle: {manifest_path}")]


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
    collision_monitor_params = os.path.join(
        bringup_share, "config", "collision_monitor_travel.yaml"
    )
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

    map_bundle_arg = DeclareLaunchArgument(
        "map_bundle",
        default_value="",
        description="Indoor map bundle directory or manifest.yaml. Overrides individual map paths.",
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
    map_alignment_arg = DeclareLaunchArgument("map_alignment_file", default_value="")
    descriptor_index_arg = DeclareLaunchArgument("descriptor_index", default_value="")
    destinations_file_arg = DeclareLaunchArgument("destinations_file", default_value="")
    map_id_arg = DeclareLaunchArgument("map_id", default_value="")
    auto_global_localization_arg = DeclareLaunchArgument(
        "auto_global_localization", default_value="true"
    )
    use_foxglove_arg = DeclareLaunchArgument(
        "use_foxglove",
        default_value=os.environ.get("FYP_USE_FOXGLOVE", "true"),
        description="Start Foxglove Bridge on the vehicle for visualization and control panels.",
    )
    foxglove_port_arg = DeclareLaunchArgument(
        "foxglove_port", default_value="8765"
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
                "alignment_file": LaunchConfiguration("map_alignment_file"),
                "descriptor_index": LaunchConfiguration("descriptor_index"),
                "map_id": LaunchConfiguration("map_id"),
                "auto_global_localization": ParameterValue(
                    LaunchConfiguration("auto_global_localization"), value_type=bool
                ),
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

    localization_cmd_gate_node = launch_ros.actions.Node(
        package="bringup",
        executable="localization_cmd_gate.py",
        name="localization_cmd_gate",
        output="screen",
    )

    collision_monitor_node = launch_ros.actions.Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[collision_monitor_params],
    )
    collision_monitor_lifecycle = launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_collision_monitor",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["collision_monitor"]}],
    )

    indoor_navigation_manager_node = launch_ros.actions.Node(
        package="indoor_navigation_manager",
        executable="indoor_navigation_manager_node",
        name="indoor_navigation_manager",
        output="screen",
        parameters=[
            {
                "destinations_file": LaunchConfiguration("destinations_file"),
                "map_id": LaunchConfiguration("map_id"),
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("destinations_file"), "' != ''"])
        ),
    )

    foxglove_navigation_adapter_node = launch_ros.actions.Node(
        package="indoor_navigation_manager",
        executable="foxglove_navigation_adapter_node",
        name="foxglove_navigation_adapter",
        output="screen",
        parameters=[
            {
                "destinations_file": LaunchConfiguration("destinations_file"),
                "map_id": LaunchConfiguration("map_id"),
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("destinations_file"), "' != ''"])
        ),
    )

    foxglove_bridge_node = launch_ros.actions.Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": ParameterValue(
                    LaunchConfiguration("foxglove_port"), value_type=int
                ),
                "client_topic_whitelist": [
                    "^/initialpose$",
                    "^/foxglove/goal_pose$",
                    "^/move_base_simple/goal$",
                    "^/foxglove/named_destination$",
                ],
                "service_whitelist": [
                    "^/foxglove/cancel_navigation$",
                    "^/localizer/global_relocalize$",
                ],
                "capabilities": [
                    "clientPublish",
                    "services",
                    "connectionGraph",
                    "assets",
                ],
            }
        ],
        condition=IfCondition(LaunchConfiguration("use_foxglove")),
    )

    serial_node = launch_ros.actions.Node(
        package="serial_twistctl",
        executable="serial_twistctl_node",
        name="serial_twistctl_node",
        output="screen",
        parameters=[LaunchConfiguration("master_params_file")],
        remappings=[("/cmd_vel", "/cmd_vel_safe")],
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
            os.path.join(bringup_share, "launch", "robot_description.launch.py")
        )
    )

    return LaunchDescription(
        [
            map_bundle_arg,
            map_yaml_arg,
            pcd_map_arg,
            map_alignment_arg,
            descriptor_index_arg,
            destinations_file_arg,
            map_id_arg,
            auto_global_localization_arg,
            use_foxglove_arg,
            foxglove_port_arg,
            use_rviz_arg,
            use_pgo_arg,
            master_params_arg,
            rviz_config_arg,
            OpaqueFunction(function=_resolve_map_bundle),
            livox_launch,
            fastlio_launch,
            pgo_node,
            localizer_node,
            initialpose_relocalize_bridge_node,
            nav2_cloud_retime_node,
            localization_cmd_gate_node,
            collision_monitor_node,
            collision_monitor_lifecycle,
            indoor_navigation_manager_node,
            foxglove_navigation_adapter_node,
            foxglove_bridge_node,
            serial_node,
            serial_reader_node,
            pointcloud_to_laserscan_node,
            delayed_nav2,
            urdf_launch,
            rviz_node,
        ]
    )
