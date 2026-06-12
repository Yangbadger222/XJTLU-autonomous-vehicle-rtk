"""BEV 配置：全系统唯一的 BEV 参数真源。

fingerprint() 写入数据集 manifest 与模型 checkpoint，risk_pipeline_node 启动时
三方比对（代码/数据/模型），不一致拒绝加载——防 train/runtime 漂移。
"""

import hashlib
import json
from dataclasses import asdict, dataclass, field


@dataclass(frozen=True)
class BevConfig:
    """v0 冻结值。z 窗口必须匹配 FAST-LIO2 publish_cloud_min_z/max_z 裁剪（F5）。"""

    size_m: float = 12.0
    resolution: float = 0.1            # -> 120x120
    agg_window_s: float = 1.5          # 点云时间聚合窗口
    z_min: float = -0.35               # 略宽于上游裁剪 [-0.33, 0.30]，避免边界丢点
    z_max: float = 0.32
    channels: tuple = (
        "max_z", "min_z", "mean_z", "var_z", "density", "unknown", "costmap",
    )
    health_keys: tuple = (
        "lio_min_eig", "lio_degenerate", "pgo_correcting", "rtk_status", "v", "w",
    )
    # 上游 FAST-LIO2 发布前的高度裁剪窗口。本身不参与 BEV 计算，
    # 但纳入 fingerprint：上游一改指纹即变，强制重新生成数据集。
    upstream_crop: tuple = (-0.33, 0.30)
    # patch 检索参数（特征锚）
    patch_size_m: float = 4.0
    patch_stride_m: float = 2.0

    @property
    def grid_size(self) -> int:
        return int(round(self.size_m / self.resolution))

    @property
    def num_channels(self) -> int:
        return len(self.channels)

    def fingerprint(self) -> str:
        payload = json.dumps(asdict(self), sort_keys=True, default=list)
        return hashlib.sha1(payload.encode()).hexdigest()[:12]


DEFAULT_CONFIG = BevConfig()
