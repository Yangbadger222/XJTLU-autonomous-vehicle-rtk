import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    default_master_params = os.path.join(bringup_share, "config", "master_params.yaml")

    params_file = LaunchConfiguration("params_file")
    rtk_params_file = LaunchConfiguration("rtk_params_file")
    pgo_config = LaunchConfiguration("pgo_config")
    use_rviz = LaunchConfiguration("use_rviz")
    frc_mode = LaunchConfiguration("frc_mode")
    frc_extra_params = LaunchConfiguration("frc_extra_params_file")

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

    gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("gnss_calibration"),
                        "launch",
                        "gnss_calibration_launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={"params_file": rtk_params_file}.items(),
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
            "params_file": os.path.join(bringup_share, "config", "nav2_explore.yaml"),
        }.items(),
    )

    delayed_nav2 = TimerAction(period=5.0, actions=[nav2_launch])

    frc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("frc_bringup"), "launch",
                     "frc_stack.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "mode": frc_mode,
            "master_params_file": params_file,
            "frc_extra_params": frc_extra_params,
        }.items(),
        condition=IfCondition(PythonExpression(["'", frc_mode, "' != 'off'"])),
    )
    delayed_frc = TimerAction(period=8.0, actions=[frc_launch])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_master_params,
                description="Unified ROS2 parameter file for explore-gps mode",
            ),
            DeclareLaunchArgument(
                "rtk_params_file",
                default_value=default_master_params,
                description="Parameter file used only by um982_rtk_driver",
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
                "frc_mode",
                default_value="off",
                description="FRC 双锚风险记忆栈：off | shadow（只发布）| full（注入 costmap）",
            ),
            DeclareLaunchArgument(
                "frc_extra_params_file",
                default_value="",
                description="Optional ROS2 parameter file appended only to the FRC nodes",
            ),
            livox_launch,
            gnss_launch,
            pgo_launch,
            serial_node,
            serial_reader_node,
            delayed_nav2,
            delayed_frc,
        ]
    )
