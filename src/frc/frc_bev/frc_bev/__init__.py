"""FRC BEV 特征构建共用库。

train/runtime 唯一真源：在线 risk_pipeline_node 与离线 bag_to_bev_dataset.py
import 同一份代码。核心模块零 ROS import，ROS 消息转换收口在 ros_adapter.py。
"""

from frc_bev.config import BevConfig, DEFAULT_CONFIG
from frc_bev.builder import BevBuilder, BevFrame

__all__ = ["BevConfig", "DEFAULT_CONFIG", "BevBuilder", "BevFrame"]
