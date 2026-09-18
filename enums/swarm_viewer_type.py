"""SwarmViewerType —— 群体可视化模式枚举。

对应 MATLAB 源文件: enumerators/SwarmViewerType.m

MATLAB 原始注释（保留）:
    % This class (enumerator) is meant to manage the Swarm viewer types:
    % drone, agent, agent_with_energy

迁移说明:
    - 采用 ``(str, Enum)`` 混入，保持 MATLAB 中 ``self.viewer_type == "drone"``
      这类字符串比较行为（见 SwarmViewer.update）。
    - 群体示例中：point_mass → "agent"，quadcopter → "drone"。
    - 可视化在 Phase 12 迁移，此处仅提供枚举定义。
"""

from enum import Enum


class SwarmViewerType(str, Enum):
    """管理群体可视化类型：drone（刚体）、agent（质点）、agent_with_energy（带能量）。"""

    DRONE = "drone"
    AGENT = "agent"
    AGENT_WITH_ENERGY = "agent_with_energy"
