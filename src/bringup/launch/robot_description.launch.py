import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    xacro_path = os.path.join(
        bringup_share,
        'urdf',
        'rosbot',
        'rosbot.urdf.xacro'
    )

    # Use Command to evaluate the xacro file into a standard URDF string
    robot_desc = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    # Pass the compiled URDF to the Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher
    ])