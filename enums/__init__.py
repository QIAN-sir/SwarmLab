"""SwarmLab_py.enums —— 枚举集中模块。

对应 MATLAB 的 ``enumerators/`` 目录（每个枚举一个文件）。
本包统一导出全部 5 个枚举，便于 ``from SwarmLab_py.enums import DroneState`` 使用。

包含:
    - DroneType        (enumerators/DroneType.m)
    - DroneState       (enumerators/DroneState.m)      ★二次开发新增★
    - PathType         (enumerators/PathType.m)
    - SwarmAlgorithm   (enumerators/SwarmAlgorithm.m)
    - SwarmViewerType  (enumerators/SwarmViewerType.m)
"""

from .drone_type import DroneType
from .drone_state import DroneState
from .path_type import PathType
from .swarm_algorithm import SwarmAlgorithm
from .swarm_viewer_type import SwarmViewerType

__all__ = [
    "DroneType",
    "DroneState",
    "PathType",
    "SwarmAlgorithm",
    "SwarmViewerType",
]
