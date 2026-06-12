"""会话与运行时路径工具（与 launch_with_logs.sh / lio_node 的约定保持一致）。"""

import os
import time
from pathlib import Path


def runtime_root() -> Path:
    root = os.environ.get("FYP_RUNTIME_ROOT", "")
    if root:
        return Path(root)
    return Path.home() / "XJTLU-autonomous-vehicle" / "runtime-data"


def frc_dir(*sub: str) -> Path:
    """runtime-data/frc/<sub...>，确保存在。"""
    p = runtime_root() / "frc"
    for s in sub:
        p = p / s
    p.mkdir(parents=True, exist_ok=True)
    return p


def session_id() -> str:
    """session_id = launch_with_logs.sh 的会话时间戳。

    FYP_LOG_SESSION_DIR 形如 .../runtime-data/logs/<timestamp>/data；
    脚本外手动运行时退化为 adhoc-<启动时刻>。
    """
    session_dir = os.environ.get("FYP_LOG_SESSION_DIR", "")
    if session_dir:
        p = Path(session_dir)
        if p.name == "data":
            p = p.parent
        if p.name:
            return p.name
    return "adhoc-" + time.strftime("%Y-%m-%d-%H-%M-%S")
