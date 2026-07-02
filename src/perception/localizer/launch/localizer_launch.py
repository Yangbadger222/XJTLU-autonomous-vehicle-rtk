import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("localizer"), "rviz", "localizer.rviz"]
    )
    localizer_config_path = PathJoinSubstitution(
        [FindPackageShare("localizer"), "config", "localizer.yaml"]
    )
    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )

    config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value=localizer_config_path,
        description="YAML configuration file for localizer_node.",
    )
    pcd_map_arg = DeclareLaunchArgument(
        "pcd_map",
        default_value="",
        description="Optional prior PCD map loaded at startup.",
    )
    use_lio_arg = DeclareLaunchArgument(
        "use_lio",
        default_value="true",
        description="Whether this standalone launch should start FAST-LIO2.",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether this standalone launch should start RViz.",
    )

    return launch.LaunchDescription(
        [
            config_path_arg,
            pcd_map_arg,
            use_lio_arg,
            use_rviz_arg,
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[{"config_path": lio_config_path}],
                condition=IfCondition(LaunchConfiguration("use_lio")),
            ),
            launch_ros.actions.Node(
                package="localizer",
                namespace="localizer",
                executable="localizer_node",
                name="localizer_node",
                output="screen",
                parameters=[
                    {
                        "config_path": LaunchConfiguration("config_path"),
                        "pcd_map": LaunchConfiguration("pcd_map"),
                    }
                ],
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="localizer",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
