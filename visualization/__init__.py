"""SwarmLab_py.visualization —— 可视化（对应 MATLAB graphics/）。

Stage 2:
    swarm_viewer.SwarmViewer      群体在线 3D（故障状态 ○/□/△ 着色）
    map_drawing                   建筑/圆柱障碍绘制
    offline_plots                 离线轨迹图 + 性能指标图
Stage 8 (Phase 12 真实动力学可视化):
    drone_body                    机体几何 define_quad_body/define_wing_body + rotate/translate
    drone_viewer.DroneViewer      单机在线 3D（quad/wing 机体姿态渲染）
    drone_state_plot.DroneStatePlotter  单机状态时序曲线（@Drone/plot_state.m，7 组 real/est/cmd）
Stage 9 (路径可视化 + 视频录制，供汇报/回顾):
    path_drawing                  draw_waypoints / draw_path / points_along_dubins_path
    video_writer.VideoWriterWithRate    figure 按帧率录 mp4（FFMpegWriter）
待后续：SwarmViewerOffline（回放，用户明确不需要）。
"""

from .swarm_viewer import SwarmViewer
from .map_drawing import draw_buildings, draw_cylinders, set_map_axes
from .offline_plots import plot_trajectories, plot_performance
from .drone_body import (define_body, define_quad_body, define_wing_body,
                         rotate, translate)
from .drone_viewer import DroneViewer
from .drone_state_plot import DroneStatePlotter
from .path_drawing import draw_waypoints, draw_path, points_along_dubins_path
from .video_writer import VideoWriterWithRate

__all__ = [
    "SwarmViewer",
    "draw_buildings", "draw_cylinders", "set_map_axes",
    "plot_trajectories", "plot_performance",
    "define_body", "define_quad_body", "define_wing_body", "rotate", "translate",
    "DroneViewer", "DroneStatePlotter",
    "draw_waypoints", "draw_path", "points_along_dubins_path",
    "VideoWriterWithRate",
]
