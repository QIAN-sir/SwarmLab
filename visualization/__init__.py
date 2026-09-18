"""SwarmLab_py.visualization —— 可视化（对应 MATLAB graphics/）。

Stage 2:
    swarm_viewer.SwarmViewer      群体在线 3D（故障状态 ○/□/△ 着色）
    map_drawing                   建筑/圆柱障碍绘制
    offline_plots                 离线轨迹图 + 性能指标图
待后续：SwarmViewerOffline（回放）、DroneViewer（单机，Phase 8+）、视频录制。
"""

from .swarm_viewer import SwarmViewer
from .map_drawing import draw_buildings, draw_cylinders, set_map_axes
from .offline_plots import plot_trajectories, plot_performance

__all__ = [
    "SwarmViewer",
    "draw_buildings", "draw_cylinders", "set_map_axes",
    "plot_trajectories", "plot_performance",
]
