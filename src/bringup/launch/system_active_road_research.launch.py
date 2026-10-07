"""Single research entry point; old Nav2/SLAM/MPPI task stacks are excluded.

Default mode is replay. ``live`` still leaves actuator authorization to the
existing authority and command guard. The adapter retains the audited original
IMU-origin navigation point; real source health and input qualification govern
motion independently from the local coordinate convention.
"""
import os
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    master = os.path.join(bringup_share, "config", "master_params.yaml")
    super_config = os.path.join(bringup_share, "config", "super_lio_vehicle.yaml")
    safety_config = os.path.join(bringup_share, "config", "research_safety_bridge.yaml")
    cloud_frame_config = os.path.join(
        bringup_share, "config", "super_lio_cloud_frame.yaml")
    active_road_evidence_config = os.path.join(
        bringup_share, "config", "active_road_evidence.yaml")
    mode = DeclareLaunchArgument("execution_mode", default_value="replay",
                                 choices=["replay", "shadow", "live"],
                                 description="replay | shadow | live; replay runs the core without sensor drivers or actuators")
    enable_super = DeclareLaunchArgument("enable_super_lio", default_value="true",
                                         description="Enable pinned Super-LIO; replay consumes bag sensor topics")
    enable_serial = DeclareLaunchArgument("enable_serial", default_value="false",
                                         description="Require explicit true plus execution_mode:=live for the physical serial sink")
    mission_arg = DeclareLaunchArgument("mission_execution_enabled",default_value="false",
        description="Explicit research task permission; does not grant RTK or physical KEY authority")
    console_port = DeclareLaunchArgument("console_port",default_value="8765")
    bag_catalog = DeclareLaunchArgument("bag_catalog_path",default_value="",
        description="Read-only catalog of known original raw bags; replay environment only")
    session_arg=DeclareLaunchArgument("localization_session_id",default_value=uuid.uuid4().hex,
        description="Fresh identity per LIO initialization; evidence submap IDs must start with identity/")
    sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value=PythonExpression(
        ["'", LaunchConfiguration("execution_mode"), "' == 'replay'"]))
    simulated_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    super_lio = Node(package="super_lio", executable="super_lio_node", name="super_lio_node",
                     output="screen",
                     condition=IfCondition(LaunchConfiguration("enable_super_lio")),
                     parameters=[super_config, {"use_sim_time": simulated_time}])
    ego_vehicle = Node(package="ego_planner", executable="motion_plan", name="ego_vehicle_adapter",
                       output="screen",

                       parameters=[os.path.join(bringup_share, "config", "ego_vehicle_adapter.yaml"), {"use_sim_time": simulated_time}])
    tf_integrity = Node(package="ego_planner", executable="research_tf_guard",
        name="research_tf_integrity_guard", output="screen", parameters=[{"use_sim_time": simulated_time,
                                                                         "protect_world_gauge": True}])
    adapter = Node(package="super_lio_vehicle_adapter", executable="super_lio_vehicle_adapter",
                   name="super_lio_vehicle_adapter", output="screen",

                   parameters=[os.path.join(bringup_share, "config", "super_lio_reference.yaml"),
                               {"use_sim_time": simulated_time}])
    cloud_frame = Node(
        package="super_lio_vehicle_adapter", executable="super_lio_cloud_frame_adapter",
        name="super_lio_cloud_frame_adapter", output="screen",

        parameters=[cloud_frame_config, {"use_sim_time": simulated_time}])
    local_grid = Node(package="research_runtime", executable="research_local_obstacle_grid",
                      name="research_local_obstacle_grid", output="screen",

                      parameters=[os.path.join(bringup_share, "config", "research_local_grid.yaml"), {"use_sim_time": simulated_time}])
    active_road_map = Node(package="active_road_mapping", executable="active_road_map",
                           name="active_road_map", output="screen",

                           parameters=[os.path.join(bringup_share, "config", "active_road_mapping.yaml"), {"use_sim_time": simulated_time}])
    active_road_evidence = Node(
        package="active_road_mapping", executable="active_road_evidence",
        name="active_road_evidence", output="screen",

        parameters=[active_road_evidence_config, {"use_sim_time": simulated_time,
                    "localization_session_id":LaunchConfiguration("localization_session_id")}])
    active_observation = Node(package="active_road_mapping", executable="active_observation",
        name="active_observation", output="screen",
        parameters=[os.path.join(bringup_share, "config", "active_observation.yaml"),
                    {"use_sim_time": simulated_time,"execution_mode":LaunchConfiguration("execution_mode"),
                     "mission_execution_enabled":ParameterValue(LaunchConfiguration("mission_execution_enabled"),value_type=bool)}])
    authority = Node(package="gps_waypoint_dispatcher", executable="rtk_map_odom_corrector_node",
                     name="rtk_map_odom_corrector", output="screen",

                     parameters=[master, {"lio_odom_topic": "/lio/odom_vehicle",
                                          "base_frame": "base_footprint"}, {"use_sim_time": simulated_time}])
    cmd_guard = Node(package="gps_waypoint_dispatcher", executable="corridor_cmd_vel_guard_node",
                     name="corridor_cmd_vel_guard", output="screen",

                     # Headerless command/receipt watchdog must keep running
                     # when bag measurement time pauses. Original callbacks,
                     # frequency, thresholds and source remain unchanged.
                     parameters=[master, {"use_sim_time": False}])
    serial = Node(package="serial_twistctl", executable="serial_twistctl_node",
                  name="serial_twistctl_node", output="screen",
                  condition=IfCondition(PythonExpression(["'", LaunchConfiguration("execution_mode"),
                                                           "' == 'live' and '",
                                                           LaunchConfiguration("enable_serial"), "' == 'true'"])),
                  parameters=[master, {"use_sim_time": simulated_time}], remappings=[("/cmd_vel", "/cmd_vel_guarded")])
    research = Node(package="research_runtime", executable="research_safety_bridge",
                    name="research_safety_bridge", output="screen",
                    parameters=[safety_config, {"mode": LaunchConfiguration("execution_mode"),
                                 "actuator_enabled": LaunchConfiguration("enable_serial"),
                                 "health_topic": "/lio/vehicle_health",
                                 "odom_topic": "/lio/odom_vehicle",
                                 "obstacle_grid_topic": "/research/local_obstacle_grid",
                                 "map_version_topic": "/research/map_version"}, {"use_sim_time": simulated_time}])
    console = Node(package="research_runtime",executable="research_operator_console",name="research_operator_console",
        output="screen",parameters=[{"execution_mode":LaunchConfiguration("execution_mode"),
            "actuator_enabled":ParameterValue(LaunchConfiguration("enable_serial"),value_type=bool),
            "mission_execution_enabled":ParameterValue(LaunchConfiguration("mission_execution_enabled"),value_type=bool),
            "http_port":ParameterValue(LaunchConfiguration("console_port"),value_type=int),
            "bag_catalog_path":LaunchConfiguration("bag_catalog_path"),"use_sim_time":simulated_time}])
    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("livox_ros_driver2"),
                                                              "launch_ROS2", "msg_MID360_launch.py"])]),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("execution_mode"), "' != 'replay'"]))
    )
    rtk = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("um982_rtk_driver"),
                                                              "launch", "um982_rtk.launch.py"])]),
        launch_arguments={"params_file": master}.items(),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("execution_mode"), "' != 'replay'"]))
    )
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "robot_description.launch.py")))
    return LaunchDescription([mode, enable_super, enable_serial, mission_arg, console_port, bag_catalog, session_arg, sim_time_arg,
                              LogInfo(msg="Active-road research entry: no Nav2/MPPI/SLAM task stack"),
                              robot_description, livox, rtk,
                              super_lio, adapter, cloud_frame, active_road_map, active_road_evidence, active_observation,
                              local_grid, ego_vehicle, tf_integrity,
                              authority, cmd_guard, serial, research, console])
