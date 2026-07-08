import os
import tempfile
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    Shutdown,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_CORRIDOR_BAG_BASE_TOPICS = [
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
    '/localization_authority/mode',
    '/localization_authority/status',
    '/localization_authority/diagnostics',
    '/gps_corridor/goal_map',
    '/gps_corridor/path_map',
    '/cmd_vel',
    '/local_costmap/costmap',
    '/global_costmap/costmap',
    '/plan',
]

_CORRIDOR_BAG_DEBUG_TOPICS = [
    '/livox/lidar',
    '/livox/imu',
    '/fastlio2/body_cloud',
]


def _corridor_bag_topics(profile):
    normalized = (profile or 'lean').strip().lower()
    topics = list(_CORRIDOR_BAG_BASE_TOPICS)
    if normalized in {'debug', 'full', 'raw'}:
        topics.extend(_CORRIDOR_BAG_DEBUG_TOPICS)
    return topics


def _make_corridor_nav2_params(source_file):
    with open(source_file, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)

    bt_params = data['bt_navigator']['ros__parameters']
    bt_params['bt_loop_duration'] = 50
    bt_params['default_server_timeout'] = 1000

    controller_params = data['controller_server']['ros__parameters']
    controller_params['controller_frequency'] = 20.0
    controller_params['general_goal_checker']['stateful'] = False

    follow_path = controller_params['FollowPath']
    follow_path['batch_size'] = 500
    follow_path['vx_std'] = 0.18
    follow_path['wz_std'] = 0.10
    follow_path['vx_max'] = 0.65
    follow_path['wz_max'] = 0.50
    follow_path['ax_max'] = 0.70
    follow_path['ax_min'] = -1.2
    follow_path['az_max'] = 1.0

    smoother_params = data['velocity_smoother']['ros__parameters']
    smoother_params['max_velocity'] = [0.65, 0.0, 0.50]
    smoother_params['min_velocity'] = [0.0, 0.0, -0.50]
    smoother_params['max_accel'] = [0.70, 0.0, 0.9]
    smoother_params['max_decel'] = [-1.2, 0.0, -1.0]

    behavior_params = data['behavior_server']['ros__parameters']
    behavior_params['behavior_plugins'] = ['wait']

    rewritten = tempfile.NamedTemporaryFile(
        mode='w',
        prefix='xjtlu_corridor_nav2_',
        suffix='.yaml',
        delete=False,
    )
    yaml.safe_dump(data, rewritten, sort_keys=False)
    rewritten.close()
    return rewritten.name


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')
    master_params_file = os.path.join(bringup_share, 'config', 'master_params.yaml')
    pgo_corridor_config_file = os.path.join(
        bringup_share, 'config', 'pgo_corridor_no_tf.yaml'
    )
    pgo_corridor_override_file = os.path.join(
        bringup_share, 'config', 'pgo_corridor_no_gps.yaml'
    )
    nav2_corridor_params_file = os.path.join(bringup_share, 'config', 'nav2_corridor_rtk.yaml')
    corridor_nav2_params = _make_corridor_nav2_params(nav2_corridor_params_file)
    corridor_no_recovery_bt_xml = os.path.join(
        bringup_share,
        'behavior_trees',
        'navigate_to_pose_w_replanning_5hz_no_motion_recovery.xml',
    )
    corridor_no_recovery_through_poses_bt_xml = os.path.join(
        bringup_share,
        'behavior_trees',
        'navigate_through_poses_w_replanning_5hz_no_motion_recovery.xml',
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
            'pgo_config_file': pgo_corridor_config_file,
            'pgo_extra_params_file': pgo_corridor_override_file,
            'nav2_params_file': corridor_nav2_params,
            'nav_to_pose_bt_xml': corridor_no_recovery_bt_xml,
            'nav_through_poses_bt_xml': corridor_no_recovery_through_poses_bt_xml,
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
                'terminal_stop_hold_s': 1.2,
                'terminal_stop_publish_hz': 20.0,
            }
        ],
    )

    rtk_authority = Node(
        package='gps_waypoint_dispatcher',
        executable='rtk_map_odom_corrector_node',
        name='rtk_map_odom_corrector',
        output='screen',
        parameters=[
            master_params_file,
            {
                'fix_topic': '/fix',
                'heading_topic': '/heading',
                'rtk_status_topic': '/rtk/status',
                'alignment_topic': '/gps_corridor/enu_to_map',
            },
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
    bag_profile = os.environ.get('FYP_CORRIDOR_BAG_PROFILE', 'lean')

    bag_record = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'record',
            '--output',
            bag_dir,
        ] + _corridor_bag_topics(bag_profile),
        output='log',
    )

    delayed_aligner = TimerAction(period=2.0, actions=[global_aligner])
    delayed_rtk_authority = TimerAction(period=3.0, actions=[rtk_authority])
    delayed_runner = TimerAction(period=8.0, actions=[corridor_runner])

    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'robot_description.launch.py')
        )
    )

    return LaunchDescription([
        route_file_arg,
        rtk_params_file_arg,
        startup_wait_timeout_arg,
        use_rviz_arg,
        explore_launch,
        rtk_launch,
        LogInfo(msg=f'Corridor bag profile: {bag_profile}'),
        bag_record,
        delayed_aligner,
        delayed_rtk_authority,
        delayed_runner,
        urdf_launch,
    ])
