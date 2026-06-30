import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    runtime_scene_dir = os.path.expanduser("~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_scene")

    default_scene_params = os.path.join(runtime_scene_dir, "master_params_scene.yaml")
    default_scene_points = os.path.join(runtime_scene_dir, "scene_points.yaml")
    default_route_graph = os.path.join(runtime_scene_dir, "scene_route_graph.geojson")
    default_nav2_params = os.path.join(bringup_share, "config", "nav2_gps.yaml")

    params_file = LaunchConfiguration("params_file")
    rtk_params_file = LaunchConfiguration("rtk_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    pgo_config = LaunchConfiguration("pgo_config")
    use_rviz = LaunchConfiguration("use_rviz")
    scene_points_file = LaunchConfiguration("scene_points_file")
    route_graph_file = LaunchConfiguration("route_graph_file")

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

    pgo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("pgo"), "launch", "pgo_launch.py"])]),
        launch_arguments={
            "params_file": params_file,
            "pgo_config": pgo_config,
            "use_rviz": use_rviz,
        }.items(),
    )

    serial_node = launch_ros.actions.Node(
        package="serial_twistctl",
        executable="serial_twistctl_node",
        name="serial_twistctl_node",
        output="screen",
        parameters=[params_file],
    )

    serial_reader_node = launch_ros.actions.Node(
        package="serial_reader",
        executable="serial_reader_node",
        name="serial_reader_node",
        output="screen",
        parameters=[params_file],
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
                "base_frame": "base_footprint",
                "graph_filepath": route_graph_file,
                "enable_nn_search": False,
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
        parameters=[params_file, {"scene_points_file": scene_points_file}],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])]
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": nav2_params_file,
        }.items(),
    )

    delayed_nav2 = TimerAction(period=5.0, actions=[nav2_launch])
    delayed_goal_manager = TimerAction(period=7.0, actions=[goal_manager_node])

    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'robot_description.launch.py')
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_scene_params,
                description="Runtime scene parameter file generated by build_scene_runtime.py",
            ),
            DeclareLaunchArgument(
                "rtk_params_file",
                default_value=default_scene_params,
                description="Parameter file used only by um982_rtk_driver",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params,
                description="Nav2 configuration file for nav-gps mode",
            ),
            DeclareLaunchArgument(
                "pgo_config",
                default_value="",
                description="Optional legacy PGO flat YAML override such as pgo_no_gps.yaml",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Whether to launch RViz together with PGO",
            ),
            DeclareLaunchArgument(
                "scene_points_file",
                default_value=default_scene_points,
                description="Compiled scene_points.yaml generated by build_scene_runtime.py",
            ),
            DeclareLaunchArgument(
                "route_graph_file",
                default_value=default_route_graph,
                description="Compiled scene_route_graph.geojson generated by build_scene_runtime.py",
            ),
            livox_launch,
            rtk_launch,
            anchor_localizer_node,
            pgo_launch,
            serial_node,
            serial_reader_node,
            route_server_node,
            route_lifecycle_manager,
            delayed_nav2,
            delayed_goal_manager,
            urdf_launch,
        ]
    )
