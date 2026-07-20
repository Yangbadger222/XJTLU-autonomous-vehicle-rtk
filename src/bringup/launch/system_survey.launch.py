import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Survey Mode launch file.
    It includes the Explore stack and starts the Survey state machine node.
    """
    bringup_share = get_package_share_directory("bringup")
    default_survey_params_file = os.path.join(bringup_share, "config", "survey_mode.yaml")
    
    survey_params_arg = DeclareLaunchArgument(
        "survey_params_file",
        default_value=default_survey_params_file,
        description="ROS2 parameter file for survey mode node",
    )

    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("bringup"), "launch", "system_explore.launch.py"]
                )
            ]
        )
    )

    survey_node = Node(
        package="survey_mode",
        executable="survey_node",
        name="survey_node",
        output="screen",
        parameters=[LaunchConfiguration("survey_params_file")],
    )

    return LaunchDescription(
        [
            survey_params_arg,
            explore_launch,
            survey_node,
        ]
    )
