# -*- coding: utf-8 -*-
"""DroneViewer —— 单机在线 3D 可视化（matplotlib），渲染 quad/wing 机体姿态。

对应 MATLAB: graphics/graphics_drone/DroneViewer.m
- draw_body：vertices = rotate(phi,theta,psi) → translate(pn,pe,pd) → NED→ENU → Poly3DCollection
- draw_point：point_mass → scatter
- NED→ENU：plot x=east(pe), y=north(pn), z=up(-pd)（与 SwarmViewer/map_drawing 一致）
- output_rate 节流；center_view_on_drone 时视图跟随无人机；view(32,47)→view_init(elev=47,azim=32)
- 未迁移：draw_waypoints/draw_path（graphics_map 路径可视化，按需）

headless 友好：不调用 plt.show()；save_frame() 存图（Agg 后端）。
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from .drone_body import define_body, rotate, translate
from .map_drawing import draw_buildings, draw_cylinders, set_map_axes

# NED→ENU 旋转（DroneViewer.m 的 R=[0,1,0; 1,0,0; 0,0,-1]）
_R_NED2ENU = np.array([[0.0, 1.0, 0.0],
                       [1.0, 0.0, 0.0],
                       [0.0, 0.0, -1.0]])


class DroneViewer:
    """单机 3D 在线可视化（机体刚体渲染）。"""

    def __init__(self, drone, output_rate=0.5, center_view_on_drone=True,
                 figsize=(8, 8), draw_map=True):
        self.output_rate = output_rate
        self.center_view_on_drone = center_view_on_drone
        self.figsize = figsize
        self.draw_map = draw_map
        self.time_of_last_frame = 0.0
        self.plot_initialized = False
        self.fig = None
        self.ax = None
        self.body_coll = None
        self.point_handle = None
        # 机体几何（对应 DroneViewer 构造里的 define_body）
        self.vertices, self.faces, self.face_colors = define_body(drone.drone_type)

    # ------------------------------------------------------------------
    def update(self, time, drone, map_params=None):
        """刷新一帧（节流到 output_rate）。对应 DroneViewer.update。"""
        if self.plot_initialized and (time - self.time_of_last_frame) < self.output_rate - 1e-6:
            return
        self.time_of_last_frame = time
        if not self.plot_initialized:
            self._init_plot(map_params)
            self.plot_initialized = True
        if self.faces.shape[0] > 0:            # quad / wing：渲染机体
            self._draw_body(drone)
        else:                                  # point_mass：散点
            self._draw_point(drone)
        self.ax.set_title(f"Simulation time: {time:.2f} seconds")
        self.fig.canvas.draw_idle()

    def _init_plot(self, map_params):
        self.fig = plt.figure(figsize=self.figsize)
        self.ax = self.fig.add_subplot(111, projection="3d")
        if self.draw_map and map_params is not None and getattr(map_params, "ACTIVE_ENVIRONMENT", False):
            if map_params.building_shape == "rectangle":
                draw_buildings(self.ax, map_params)
            elif map_params.building_shape == "cylinder":
                draw_cylinders(self.ax, map_params)
            set_map_axes(self.ax, map_params)
        self.ax.set_xlabel("y position [m]")   # east
        self.ax.set_ylabel("x position [m]")   # north
        self.ax.set_zlabel("z position [m]")   # up
        self.ax.view_init(elev=47, azim=32)    # MATLAB view(32, 47)
        self.ax.grid(True)

    def _draw_body(self, drone):
        pn, pe, pd = drone.pos_ned
        phi, theta, psi = drone.attitude
        v = rotate(self.vertices, phi, theta, psi)      # (3,N) 机体→惯常(NED)
        v = translate(v, pn, pe, pd)                    # 平移到 NED 位置
        v = _R_NED2ENU @ v                              # NED→ENU（plot 坐标）
        polys = [v[:, self.faces[i]].T for i in range(self.faces.shape[0])]
        if self.body_coll is not None:
            self.body_coll.remove()
        self.body_coll = Poly3DCollection(polys, facecolors=self.face_colors,
                                          edgecolors="k", linewidths=0.5)
        self.ax.add_collection3d(self.body_coll)
        if self.center_view_on_drone:
            self._center(pe, pn, -pd)

    def _draw_point(self, drone):
        pn, pe, pd = drone.pos_ned
        if self.point_handle is None:
            self.point_handle = self.ax.scatter([pe], [pn], [-pd], c=[drone.color],
                                                s=60, depthshade=False)
        else:
            self.point_handle._offsets3d = ([pe], [pn], [-pd])
        if self.center_view_on_drone:
            self._center(pe, pn, -pd)

    def _center(self, x, y, z, margin=10.0):
        self.ax.set_xlim(x - margin, x + margin)
        self.ax.set_ylim(y - margin, y + margin)
        self.ax.set_zlim(z - margin, z + margin)

    # ------------------------------------------------------------------
    def save_frame(self, path, dpi=100):
        """保存当前帧（headless 验证/离线出图）。"""
        self.fig.savefig(path, dpi=dpi, bbox_inches="tight")

    def close(self):
        if self.fig is not None:
            plt.close(self.fig)
            self.fig = None
            self.plot_initialized = False
