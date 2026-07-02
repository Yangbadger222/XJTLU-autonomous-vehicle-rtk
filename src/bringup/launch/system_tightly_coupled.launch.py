import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    master_params_file = os.path.join(bringup_share, "config", "master_params.yaml")
    rtk_fgo_params_file = os.path.join(bringup_share, "config", "rtk_fgo.yaml")

    rtk_params_file_arg = DeclareLaunchArgument(
        "rtk_params_file",
        default_value=master_params_file,
        description="Parameter file used only by um982_rtk_driver",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Whether to launch RViz together with the tightly coupled shadow stack",
    )
    publish_fgo_tf_arg = DeclareLaunchArgument(
        "publish_fgo_tf",
        default_value="false",
        description="Explicitly enable experimental RTK FGO TF publication",
    )
    nav2_use_fgo_arg = DeclareLaunchArgument(
        "nav2_use_fgo",
        default_value="false",
        description="Explicitly mark Nav2 as using the experimental RTK FGO output",
    )

    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "system_explore.launch.py")
        ),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
            "master_params_file": master_params_file,
        }.items(),
    )

    rtk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("um982_rtk_driver"), "launch", "um982_rtk.launch.py"]
                )
            ]
        ),
        launch_arguments={"params_file": LaunchConfiguration("rtk_params_file")}.items(),
    )

    rtk_fgo_node = Node(
        package="rtk_fgo_localizer",
        executable="rtk_fgo_node",
        name="rtk_fgo_localizer",
        output="screen",
        parameters=[
            rtk_fgo_params_file,
            {
                "publish_tf": LaunchConfiguration("publish_fgo_tf"),
                "nav2_use_fgo": LaunchConfiguration("nav2_use_fgo"),
            },
        ],
    )

    session_data_dir = os.environ.get("FYP_LOG_SESSION_DIR", "")
    if session_data_dir:
        session_root = os.path.dirname(session_data_dir)
    else:
        session_root = os.path.expanduser(
            f"~/XJTLU-autonomous-vehicle/runtime-data/logs/{datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}"
        )
    bag_dir = os.path.join(session_root, "bag")
    os.makedirs(session_root, exist_ok=True)

    bag_record = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            bag_dir,
            "/fix",
            "/heading",
            "/rtk/status",
            "/rtk/nmea_sentence",
            "/livox/lidar",
            "/livox/imu",
            "/fastlio2/lio_odom",
            "/fastlio2/body_cloud",
            "/odom_CBoar",
            "/pgo/optimized_odom",
            "/pgo/loop_markers",
            "/rtk_fgo/odom",
            "/rtk_fgo/path",
            "/rtk_fgo/status",
            "/rtk_fgo/rtk_gate",
            "/rtk_fgo/correction_status",
            "/rtk_fgo/factor_diagnostics",
            "/tf",
            "/tf_static",
            "/cmd_vel",
            "/plan",
        ],
        output="log",
    )

    delayed_rtk_fgo = TimerAction(period=6.0, actions=[rtk_fgo_node])

    return LaunchDescription(
        [
            rtk_params_file_arg,
            use_rviz_arg,
            publish_fgo_tf_arg,
            nav2_use_fgo_arg,
            explore_launch,
            rtk_launch,
            bag_record,
            delayed_rtk_fgo,
        ]
    )
