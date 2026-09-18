"""SwarmViewer —— 群体在线 3D 可视化（matplotlib）。

对应 MATLAB: graphics/graphics_swarm/SwarmViewer.m
- NED→ENU 绘图：x=east(pe), y=north(pn), z=up(-pd)
- 故障状态用不同 marker：Normal='o'、Stalled='s'、Drifting='^'（对应 MATLAB 二次开发）
- 环境（建筑/圆柱）只在首帧绘制一次；此后每帧仅更新无人机散点（对应 plot_initialized 逻辑）
- output_rate 控制刷新频率（对应 MATLAB 的 time_of_last_frame 节流）

headless 友好：不调用 plt.show()；可用 save_frame() 存图（Agg 后端），Stage 3 GUI 复用本类。
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from ..enums import DroneState
from .map_drawing import draw_buildings, draw_cylinders, set_map_axes

# 故障状态 → marker（对应 SwarmViewer.m 的 o/s/^）
_STATE_MARKER = {
    "Normal": "o",
    "Stalled": "s",
    "Drifting": "^",
}


class SwarmViewer:
    """群体 3D 在线可视化。"""

    def __init__(self, output_rate=0.5, center_view_on_swarm=False,
                 viewer_type="agent", figsize=(8, 8), marker_size=60):
        self.output_rate = output_rate
        self.center_view_on_swarm = center_view_on_swarm
        self.viewer_type = viewer_type       # 'agent' / 'drone'（drone 刚体模型待 Phase 8+）
        self.figsize = figsize
        self.marker_size = marker_size
        self.fig = None
        self.ax = None
        self.plot_initialized = False
        self.time_of_last_frame = 0.0
        self._scatters = []

    # ------------------------------------------------------------------
    def update(self, time, swarm, map_params):
        """刷新可视化（节流到 output_rate）。对应 SwarmViewer.update。"""
        if self.plot_initialized and (time - self.time_of_last_frame) < self.output_rate - 1e-6:
            return
        self.time_of_last_frame = time
        if not self.plot_initialized:
            self._init_plot(swarm, map_params)
            self.plot_initialized = True
        self._update_agents(swarm)
        self.ax.set_title(f"Simulation time: {time:.1f} seconds")
        self.fig.canvas.draw_idle()

    def _init_plot(self, swarm, map_params):
        self.fig = plt.figure(figsize=self.figsize)
        self.ax = self.fig.add_subplot(111, projection="3d")
        # 环境
        if getattr(map_params, "ACTIVE_ENVIRONMENT", False):
            if map_params.building_shape == "rectangle":
                draw_buildings(self.ax, map_params)
            elif map_params.building_shape == "cylinder":
                draw_cylinders(self.ax, map_params)
            set_map_axes(self.ax, map_params)
        else:
            self._auto_axes(swarm)
        self.ax.set_xlabel("Y position [m]")     # 对应 MATLAB：x 轴显示 east
        self.ax.set_ylabel("X position [m]")     # y 轴显示 north
        self.ax.set_zlabel("Z position [m]")
        self.ax.grid(True)

    def _auto_axes(self, swarm, margin=40.0):
        pos = swarm.get_pos_ned()
        mn = pos.min(axis=1)
        mx = pos.max(axis=1)
        edge = max(mx - mn)
        center = (mn + mx) / 2
        self.ax.set_xlim(center[1] - edge / 2 - margin, center[1] + edge / 2 + margin)  # east
        self.ax.set_ylim(center[0] - edge / 2 - margin, center[0] + edge / 2 + margin)  # north
        self.ax.set_zlim(0, max(edge, 2 * margin))

    def _update_agents(self, swarm):
        for sc in self._scatters:
            sc.remove()
        self._scatters = []

        pos = swarm.get_pos_ned()          # (3,N) = [pn; pe; pd]
        x = pos[1, :]                      # east
        y = pos[0, :]                      # north
        z = -pos[2, :]                     # up
        colors = swarm.get_colors()        # (3,N)
        states = [d.drone_state for d in swarm.drones]

        for state, marker in _STATE_MARKER.items():
            idx = [i for i, s in enumerate(states) if s == state]
            if not idx:
                continue
            sc = self.ax.scatter(x[idx], y[idx], z[idx],
                                 c=colors[:, idx].T, marker=marker,
                                 s=self.marker_size, depthshade=False)
            self._scatters.append(sc)

    # ------------------------------------------------------------------
    def save_frame(self, path, dpi=100):
        """保存当前帧到图片文件（headless 验证/离线出图）。"""
        self.fig.savefig(path, dpi=dpi, bbox_inches="tight")

    def close(self):
        if self.fig is not None:
            plt.close(self.fig)
            self.fig = None
            self.plot_initialized = False
