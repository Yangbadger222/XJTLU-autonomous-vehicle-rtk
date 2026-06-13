"""FRC 栈 launch：mode:=off|shadow|full。

- off：不启动任何 FRC 节点（由上层 system_explore.launch.py 的条件 Include 保证）；
- shadow：四个在线节点全启，risk_grid 正常发布，frc_layer 保持 enabled=false 旁路；
- full：与 shadow 完全相同的节点集，延时后调用 /frc/enable 打开 costmap 注入。
  shadow 与 full 的唯一区别就是 frc_layer 的 enabled——两种模式录的 bag 完全同构。

trial_runner 不在此启动（实验时按需手动运行）。
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    del args, kwargs

    params = []
    master_params = LaunchConfiguration("master_params_file").perform(context).strip()
    extra_params = LaunchConfiguration("frc_extra_params").perform(context).strip()
    if master_params:
        params.append(master_params)
    if extra_params:
        params.append(extra_params)

    def frc_node(package, executable, name):
        return Node(
            package=package,
            executable=executable,
            name=name,
            output="screen",
            parameters=params,
        )

    nodes = [
        frc_node("frc_nodes_cpp", "frc_health_aggregator",
                 "frc_health_aggregator"),
        frc_node("frc_nodes_cpp", "frc_event_marker", "frc_event_marker"),
        frc_node("frc_nodes", "frc_risk_pipeline", "frc_risk_pipeline"),
        frc_node("frc_nodes", "frc_memory_manager", "frc_memory_manager"),
    ]

    # full 模式：等 Nav2 costmap 与插件就绪后调用 /frc/enable。
    # 注入与否的唯一开关就是这个服务；失败不致命（保持 shadow 行为）。
    enable_call = TimerAction(
        period=20.0,
        actions=[ExecuteProcess(
            cmd=["ros2", "service", "call", "/frc/enable",
                 "std_srvs/srv/Trigger", "{}"],
            output="screen")],
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration("mode"), "' == 'full'"])),
    )

    return nodes + [enable_call]


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        "mode", default_value="shadow",
        description="FRC 运行模式：shadow（只发布不注入）| full（注入 costmap）")
    params_arg = DeclareLaunchArgument(
        "master_params_file", default_value="",
        description="master_params.yaml 路径（整文件传入各 FRC 节点）")
    extra_params_arg = DeclareLaunchArgument(
        "frc_extra_params", default_value="",
        description="实验期 overrides 参数文件，叠加在 master_params 之后")

    return LaunchDescription(
        [mode_arg, params_arg, extra_params_arg, OpaqueFunction(function=_launch_setup)])
