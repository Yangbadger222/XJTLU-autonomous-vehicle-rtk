import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """
    Explore 模式 launch 文件（导航专用）

    启动顺序：
    1. 同时启动：Livox MID360 激光雷达、PGO(FASTLIO2+SLAM)+RViz、串口控制节点
    2. 启动 Nav2 导航系统（延时 5 秒）
    3. frc_mode != off 时附加 FRC 双锚风险记忆栈（shadow 只发布 / full 注入）
    """

    bringup_share = get_package_share_directory("bringup")
    default_master_params_file = os.path.join(bringup_share, "config", "master_params.yaml")
    default_nav2_params_file = os.path.join(bringup_share, "config", "nav2_explore.yaml")
    default_rviz_config = os.path.join(bringup_share, "rviz", "pgo.rviz")
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
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to launch RViz together with the Explore stack",
    )
    master_params_arg = DeclareLaunchArgument(
        "master_params_file",
        default_value=default_master_params_file,
        description="ROS2 parameter file used by FAST-LIO2, PGO, and serial nodes",
    )
    pgo_extra_params_arg = DeclareLaunchArgument(
        "pgo_extra_params_file",
        default_value="",
        description="Optional ROS2 parameter file appended only to the PGO node",
    )
    pgo_config_arg = DeclareLaunchArgument(
        "pgo_config_file",
        default_value="",
        description="Optional legacy flat PGO config file passed as config_path",
    )
    nav2_params_arg = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=default_nav2_params_file,
        description="Nav2 parameter file used by the Explore/Corridor stack",
    )
    nav_to_pose_bt_xml_arg = DeclareLaunchArgument(
        "nav_to_pose_bt_xml",
        default_value=corridor_bt_xml,
        description="NavigateToPose behavior tree XML injected into bt_navigator",
    )
    nav_through_poses_bt_xml_arg = DeclareLaunchArgument(
        "nav_through_poses_bt_xml",
        default_value=default_nav_through_poses_bt_xml,
        description="NavigateThroughPoses behavior tree XML injected into bt_navigator",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="RViz layout used by the Explore/Corridor stack",
    )
    frc_mode_arg = DeclareLaunchArgument(
        "frc_mode",
        default_value="off",
        description="FRC 双锚风险记忆栈：off | shadow（只发布）| full（注入 costmap）",
    )
    frc_extra_params_arg = DeclareLaunchArgument(
        "frc_extra_params_file",
        default_value="",
        description="Optional ROS2 parameter file appended only to the FRC nodes",
    )
    guarded_cmd_vel_arg = DeclareLaunchArgument(
        "guarded_cmd_vel",
        default_value="false",
        description="Route Nav2 output through the corridor command guard",
    )
    rewritten_nav2_params = RewrittenYaml(
        source_file=LaunchConfiguration("nav2_params_file"),
        param_rewrites={
            "default_nav_to_pose_bt_xml": LaunchConfiguration("nav_to_pose_bt_xml"),
            "default_nav_through_poses_bt_xml": LaunchConfiguration("nav_through_poses_bt_xml"),
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
            "params_file": LaunchConfiguration("master_params_file"),
            "pgo_config": LaunchConfiguration("pgo_config_file"),
            "extra_params_file": LaunchConfiguration("pgo_extra_params_file"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "rviz_config": LaunchConfiguration("rviz_config"),
        }.items(),
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

    guarded_nav2 = GroupAction(
        [
            SetRemap(
                src="/cmd_vel_nav",
                dst="/cmd_vel_controller",
                condition=IfCondition(LaunchConfiguration("guarded_cmd_vel")),
            ),
            SetRemap(
                src="/cmd_vel",
                dst="/cmd_vel_nav",
                condition=IfCondition(LaunchConfiguration("guarded_cmd_vel")),
            ),
            nav2_launch,
        ]
    )
    delayed_nav2 = TimerAction(period=5.0, actions=[guarded_nav2])

    # P5（FRC）：frc_mode != off 时附加 FRC 栈（延时 8 秒，等 LIO/PGO 起稳）
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
            "mode": LaunchConfiguration("frc_mode"),
            "master_params_file": LaunchConfiguration("master_params_file"),
            "frc_extra_params": LaunchConfiguration("frc_extra_params_file"),
        }.items(),
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("frc_mode"), "' != 'off'"]
            )
        ),
    )
    delayed_frc = TimerAction(period=8.0, actions=[frc_launch])

    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'robot_description.launch.py')
        )
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            master_params_arg,
            pgo_extra_params_arg,
            pgo_config_arg,
            nav2_params_arg,
            nav_to_pose_bt_xml_arg,
            nav_through_poses_bt_xml_arg,
            rviz_config_arg,
            frc_mode_arg,
            frc_extra_params_arg,
            guarded_cmd_vel_arg,
            livox_launch,
            pgo_launch,
            serial_node,
            serial_reader_node,
            delayed_nav2,
            delayed_frc,
            urdf_launch,
        ]
    )
