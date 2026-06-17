import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, Shutdown, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')
    master_params_file = os.path.join(bringup_share, 'config', 'master_params.yaml')
    pgo_corridor_override_file = os.path.join(
        bringup_share, 'config', 'pgo_corridor_no_gps.yaml'
    )

    route_file_arg = DeclareLaunchArgument(
        'route_file',
        default_value=os.path.expanduser('~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_route.yaml'),
        description='Runtime YAML for the GPS route corridor',
    )
    rtk_params_file_arg = DeclareLaunchArgument(
        'rtk_params_file',
        default_value=master_params_file,
        description='Parameter file used only by um982_rtk_driver',
    )
    startup_wait_timeout_arg = DeclareLaunchArgument(
        'startup_wait_timeout_s',
        default_value='90.0',
        description='Maximum wait time for stable /fix, TF, and Nav2 readiness',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz together with the corridor stack',
    )

    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'system_explore.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'master_params_file': master_params_file,
            'pgo_extra_params_file': pgo_corridor_override_file,
        }.items(),
    )

    rtk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare('um982_rtk_driver'), 'launch', 'um982_rtk.launch.py']
                )
            ]
        ),
        launch_arguments={'params_file': LaunchConfiguration('rtk_params_file')}.items(),
    )

    global_aligner = Node(
        package='gps_waypoint_dispatcher',
        executable='gps_global_aligner_node',
        name='gps_global_aligner',
        output='screen',
        on_exit=Shutdown(reason='gps_global_aligner exited'),
        parameters=[
            master_params_file,
            {
                'route_file': LaunchConfiguration('route_file'),
                'startup_wait_timeout_s': LaunchConfiguration('startup_wait_timeout_s'),
                'route_frame': 'map',
                'base_frame': 'base_link',
                'fix_topic': '/fix',
                'alignment_topic': '/gps_corridor/enu_to_map',
                'status_topic': '/gps_corridor/alignment_status',
                'debug_topic': '/gps_corridor/alignment_debug',
            }
        ],
    )

    corridor_runner = Node(
        package='gps_waypoint_dispatcher',
        executable='gps_route_runner_node',
        name='gps_route_runner',
        output='screen',
        on_exit=Shutdown(reason='gps_route_runner exited'),
        parameters=[
            master_params_file,
            {
                'route_file': LaunchConfiguration('route_file'),
                'startup_wait_timeout_s': LaunchConfiguration('startup_wait_timeout_s'),
                'route_frame': 'map',
                'base_frame': 'base_link',
                'fix_topic': '/fix',
                'alignment_topic': '/gps_corridor/enu_to_map',
            }
        ],
    )

    session_data_dir = os.environ.get('FYP_LOG_SESSION_DIR', '')
    if session_data_dir:
        session_root = os.path.dirname(session_data_dir)
    else:
        session_root = os.path.expanduser(
            f'~/XJTLU-autonomous-vehicle/runtime-data/logs/{datetime.now().strftime("%Y-%m-%d-%H-%M-%S")}'
        )
    bag_dir = os.path.join(session_root, 'bag')
    os.makedirs(session_root, exist_ok=True)

    bag_record = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'record',
            '--output',
            bag_dir,
            '/fix',
            '/heading',
            '/rtk/status',
            '/rtk/nmea_sentence',
            '/fastlio2/lio_odom',
            '/tf',
            '/tf_static',
            '/gps_corridor/status',
            '/gps_corridor/alignment_status',
            '/gps_corridor/alignment_debug',
            '/gps_corridor/calibration_request',
            '/gps_corridor/calibration_status',
            '/gps_corridor/enu_to_map',
            '/gps_corridor/pgo_enu_to_map',
            '/gps_corridor/goal_map',
            '/gps_corridor/path_map',
            '/cmd_vel',
            '/local_costmap/costmap',
            '/global_costmap/costmap',
            '/plan',
        ],
        output='log',
    )

    delayed_aligner = TimerAction(period=2.0, actions=[global_aligner])
    delayed_runner = TimerAction(period=8.0, actions=[corridor_runner])

    return LaunchDescription([
        route_file_arg,
        rtk_params_file_arg,
        startup_wait_timeout_arg,
        use_rviz_arg,
        explore_launch,
        rtk_launch,
        bag_record,
        delayed_aligner,
        delayed_runner,
    ])
