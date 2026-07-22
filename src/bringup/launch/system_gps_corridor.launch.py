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
from launch.conditions import IfCondition
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
    '/livox/imu',
    '/odom_CBoar',
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
    '/localization_authority/motion_allowed',
    '/localization_authority/max_linear_speed_mps',
    '/gps_corridor/stop_override',
    '/rtk_fgo/odom',
    '/rtk_fgo/path',
    '/rtk_fgo/status',
    '/rtk_fgo/rtk_gate',
    '/rtk_fgo/correction_status',
    '/rtk_fgo/factor_diagnostics',
    '/gps_corridor/goal_map',
    '/gps_corridor/path_map',
    '/cmd_vel',
    '/cmd_vel_nav',
    '/cmd_vel_guarded',
    '/local_costmap/costmap',
    '/global_costmap/costmap',
    '/plan',
]

_CORRIDOR_BAG_DEBUG_TOPICS = [
    '/livox/lidar',
    '/fastlio2/body_cloud',
    '/fastlio2/body_cloud_nav2_obstacles',
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
    controller_params['failure_tolerance'] = 1.5
    controller_params['progress_checker']['required_movement_radius'] = 0.10
    controller_params['progress_checker']['movement_time_allowance'] = 15.0
    controller_params['general_goal_checker']['stateful'] = False

    follow_path = controller_params['FollowPath']
    follow_path['batch_size'] = 500
    follow_path['vx_std'] = 0.20
    follow_path['wz_std'] = 0.15
    follow_path['vx_max'] = 0.85
    follow_path['wz_max'] = 0.70
    follow_path['ax_max'] = 0.85
    follow_path['ax_min'] = -1.2
    follow_path['az_max'] = 1.4
    follow_path['temperature'] = 0.45
    follow_path['regenerate_noises'] = True
    follow_path['open_loop'] = False

    smoother_params = data['velocity_smoother']['ros__parameters']
    smoother_params['max_velocity'] = [0.85, 0.0, 0.70]
    smoother_params['min_velocity'] = [0.0, 0.0, -0.70]
    smoother_params['max_accel'] = [0.85, 0.0, 1.4]
    smoother_params['max_decel'] = [-1.2, 0.0, -1.8]

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
    rtk_fgo_params_file = os.path.join(bringup_share, 'config', 'rtk_fgo.yaml')
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
    enable_fgo_shadow_arg = DeclareLaunchArgument(
        'enable_fgo_shadow',
        default_value=os.environ.get('FYP_CORRIDOR_ENABLE_FGO_SHADOW', 'false'),
        description='Optionally start RTK FGO in shadow mode for corridor rosbag evidence',
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
            'guarded_cmd_vel': 'true',
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
                'lio_odom_topic': '/fastlio2/lio_odom',
                'motion_allowed_topic': '/localization_authority/motion_allowed',
                'authority_status_topic': '/localization_authority/status',
                'stop_override_topic': '/gps_corridor/stop_override',
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
        on_exit=Shutdown(reason='rtk_map_odom_corrector exited'),
        parameters=[
            master_params_file,
            {
                'fix_topic': '/fix',
                'heading_topic': '/heading',
                'nmea_topic': '/rtk/nmea_sentence',
                'lio_odom_topic': '/fastlio2/lio_odom',
                'base_frame': 'base_footprint',
                'alignment_topic': '/gps_corridor/enu_to_map',
            },
        ],
    )

    corridor_cmd_guard = Node(
        package='gps_waypoint_dispatcher',
        executable='corridor_cmd_vel_guard_node',
        name='corridor_cmd_vel_guard',
        output='screen',
        on_exit=Shutdown(reason='corridor_cmd_vel_guard exited'),
        parameters=[master_params_file],
    )

    fgo_shadow = Node(
        package='rtk_fgo_localizer',
        executable='rtk_fgo_node',
        name='rtk_fgo_localizer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_fgo_shadow')),
        parameters=[
            rtk_fgo_params_file,
            {
                'publish_tf': False,
                'nav2_use_fgo': False,
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
    delayed_fgo_shadow = TimerAction(period=6.0, actions=[fgo_shadow])
    delayed_runner = TimerAction(period=8.0, actions=[corridor_runner])

    return LaunchDescription([
        route_file_arg,
        rtk_params_file_arg,
        startup_wait_timeout_arg,
        use_rviz_arg,
        enable_fgo_shadow_arg,
        explore_launch,
        rtk_launch,
        LogInfo(msg=f'Corridor bag profile: {bag_profile}'),
        bag_record,
        delayed_aligner,
        delayed_rtk_authority,
        corridor_cmd_guard,
        delayed_fgo_shadow,
        delayed_runner,
    ])
