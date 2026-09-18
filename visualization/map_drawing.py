"""map_drawing —— 障碍物（建筑/圆柱）3D 绘制（matplotlib）。

对应 MATLAB: graphics/graphics_map/draw_buildings.m, draw_cylinders.m
坐标约定（与 SwarmViewer 一致）：plot x=east, y=north, z=up（NED→ENU）。
建筑默认长方体（rectangle），从 z=0 到 z=height；圆柱用 NB_EDGES 棱柱近似。

注：matplotlib 与 MATLAB 的 patch/surf 渲染不完全一致，此处追求“视觉等价”（ obstacle
位置/尺寸/颜色一致），非像素级复刻；数值验证不依赖可视化（计划 §17）。
"""

import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def _building_boxes(map_params):
    """生成各建筑的长方体面片（plot 坐标 x=east,y=north,z=up）。"""
    bn = np.asarray(map_params.buildings_north).ravel()
    be = np.asarray(map_params.buildings_east).ravel()
    bh = np.asarray(map_params.buildings_heights).ravel()
    w = map_params.building_width
    dx = dy = w / 2.0
    polys = []
    for n, e, h in zip(bn, be, bh):
        c = [
            (e - dx, n - dy, 0.0), (e + dx, n - dy, 0.0),
            (e + dx, n + dy, 0.0), (e - dx, n + dy, 0.0),
            (e - dx, n - dy, h), (e + dx, n - dy, h),
            (e + dx, n + dy, h), (e - dx, n + dy, h),
        ]
        polys += [
            [c[0], c[1], c[2], c[3]],   # bottom
            [c[4], c[5], c[6], c[7]],   # top
            [c[0], c[1], c[5], c[4]],
            [c[1], c[2], c[6], c[5]],
            [c[2], c[3], c[7], c[6]],
            [c[3], c[0], c[4], c[7]],
        ]
    return polys


def draw_buildings(ax, map_params, face_color=(0.7, 0.7, 0.7), alpha=0.5):
    """绘制长方体建筑（对应 draw_buildings.m）。"""
    polys = _building_boxes(map_params)
    if not polys:
        return
    pc = Poly3DCollection(polys, facecolors=face_color,
                          edgecolors=(0.4, 0.4, 0.4, 0.6), linewidths=0.5, alpha=alpha)
    ax.add_collection3d(pc)


def draw_cylinders(ax, map_params, nb_edges=8, face_color=(0.8, 0.8, 0.8), alpha=0.5):
    """绘制圆柱障碍（对应 draw_cylinders.m，NB_EDGES=8 棱柱近似）。"""
    bn = np.asarray(map_params.buildings_north).ravel()
    be = np.asarray(map_params.buildings_east).ravel()
    r = map_params.building_width / 2.0
    h = map_params.max_height
    theta = np.linspace(0, 2 * np.pi, nb_edges + 1)
    polys = []
    for n, e in zip(bn, be):
        ring_b = [(e + r * np.cos(t), n + r * np.sin(t), 0.0) for t in theta]
        ring_t = [(e + r * np.cos(t), n + r * np.sin(t), h) for t in theta]
        for j in range(nb_edges):
            polys.append([ring_b[j], ring_b[j + 1], ring_t[j + 1], ring_t[j]])
        polys.append(ring_t)   # top
    if not polys:
        return
    pc = Poly3DCollection(polys, facecolors=face_color,
                          edgecolors=(0.5, 0.5, 0.5, 0.4), linewidths=0.3, alpha=alpha)
    ax.add_collection3d(pc)


def set_map_axes(ax, map_params):
    """按地图范围设置坐标轴（对应 draw_buildings/draw_cylinders 的 axis 设置）。"""
    w = map_params.width
    margin = w / 5
    ax.set_xlim(-margin, w + margin)
    ax.set_ylim(-margin, w + margin)
    ax.set_zlim(0, 1.2 * map_params.max_height)
