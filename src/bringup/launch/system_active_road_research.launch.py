"""Single research entry point; old Nav2/SLAM/MPPI task stacks are excluded.

Default mode is replay. ``live`` still leaves actuator authorization to the
existing authority and command guard, and the adapter's unverified-frame gate
keeps it stopped until measured vehicle extrinsics are supplied.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    master = os.path.join(bringup_share, "config", "master_params.yaml")
    super_config = os.path.join(bringup_share, "config", "super_lio_vehicle.yaml")
    mode = DeclareLaunchArgument("execution_mode", default_value="replay",
                                 description="replay | shadow | live; replay is actuator-free")
    enable_super = DeclareLaunchArgument("enable_super_lio", default_value="true",
                                         description="Enable pinned Super-LIO in shadow/live; replay remains actuator-free")
    enable_serial = DeclareLaunchArgument("enable_serial", default_value="false",
                                         description="Require explicit true plus execution_mode:=live for the physical serial sink")
    super_lio = Node(package="super_lio", executable="super_lio_node", name="super_lio_node",
                     output="screen",
                     condition=IfCondition(PythonExpression(["'", LaunchConfiguration("execution_mode"),
                                                              "' != 'replay' and '",
                                                              LaunchConfiguration("enable_super_lio"), "' == 'true'"])),
                     parameters=[super_config])
    ego_vehicle = Node(package="ego_planner", executable="motion_plan", name="ego_vehicle_adapter",
                       output="screen",
                       condition=IfCondition(PythonExpression(["'", LaunchConfiguration("execution_mode"),
                                                                "' != 'replay'"])),
                       parameters=[os.path.join(bringup_share, "config", "ego_vehicle_adapter.yaml")])
    adapter = Node(package="super_lio_vehicle_adapter", executable="super_lio_vehicle_adapter",
                   name="super_lio_vehicle_adapter", output="screen",
                   condition=IfCondition(PythonExpression(["'", LaunchConfiguration("execution_mode"),
                                                            "' != 'replay'"])),
                   parameters=[{"input_topic": "/lio/odom", "vehicle_odom_topic": "/lio/odom_vehicle",
                                "source_health_topic": "/lio/health", "health_topic": "/lio/vehicle_health",
                                "imu_to_base_extrinsic_verified": False, "require_covariance": True}])
    local_grid = Node(package="research_runtime", executable="research_local_obstacle_grid",
                      name="research_local_obstacle_grid", output="screen",
                      condition=IfCondition(PythonExpression(["'", LaunchConfiguration("execution_mode"),
                                                               "' != 'replay'"])),
                      parameters=[os.path.join(bringup_share, "config", "research_local_grid.yaml")])
    authority = Node(package="gps_waypoint_dispatcher", executable="rtk_map_odom_corrector_node",
                     name="rtk_map_odom_corrector", output="screen",
                     condition=IfCondition(PythonExpression(["'", LaunchConfiguration("execution_mode"),
                                                              "' != 'replay'"])),
                     parameters=[master, {"lio_odom_topic": "/lio/odom_vehicle",
                                          "base_frame": "base_footprint"}])
    cmd_guard = Node(package="gps_waypoint_dispatcher", executable="corridor_cmd_vel_guard_node",
                     name="corridor_cmd_vel_guard", output="screen",
                     condition=IfCondition(PythonExpression(["'", LaunchConfiguration("execution_mode"),
                                                              "' != 'replay'"])),
                     parameters=[master])
    serial = Node(package="serial_twistctl", executable="serial_twistctl_node",
                  name="serial_twistctl_node", output="screen",
                  condition=IfCondition(PythonExpression(["'", LaunchConfiguration("execution_mode"),
                                                           "' == 'live' and '",
                                                           LaunchConfiguration("enable_serial"), "' == 'true'"])),
                  parameters=[master], remappings=[("/cmd_vel", "/cmd_vel_guarded")])
    research = Node(package="research_runtime", executable="research_safety_bridge",
                    name="research_safety_bridge", output="screen",
                    parameters=[{"mode": LaunchConfiguration("execution_mode"),
                                 "actuator_enabled": LaunchConfiguration("enable_serial"),
                                 "health_topic": "/lio/vehicle_health",
                                 "odom_topic": "/lio/odom_vehicle",
                                 "obstacle_grid_topic": "/research/local_obstacle_grid",
                                 "map_version_topic": "/research/map_version"}])
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
    return LaunchDescription([mode, enable_super, enable_serial,
                              LogInfo(msg="Active-road research entry: no Nav2/MPPI/SLAM task stack"),
                              livox, rtk,
                              super_lio, adapter, local_grid, ego_vehicle, authority, cmd_guard, serial, research])
