# -*- coding: utf-8 -*-
"""obstacle_distance —— UAV 与建筑物（有限高圆柱）的三维几何距离（Stage 14 评价体系重构）。

物理意义：UAV 中心点 ↔ 建筑物**三维表面**的最小距离（点–有限高圆柱），而非 2D 水平距离、
也非到建筑物中心点距离。与避障口径一致（建筑物以圆柱近似：cylinders=[x;y;r] + cyl_heights）。

    d_horiz = max( horizontal_dist(uav_xy, cyl_xy) - r , 0 )     # 到侧面（footprint 内为 0）
    dz      = h_uav - h_top   (uav 在顶部以上)
              0               (uav 在柱身高度范围内)
              -h_uav          (uav 在地下，正常不出现)
    dist3d  = sqrt(d_horiz^2 + dz^2)

高空飞越（h_uav > h_top）时 dist3d = 垂直距离（大）→ 不再误判碰撞/近距离；
低空 footprint 内时 dist3d = 到侧面水平距离 → 与原低空语义一致。
"""
import math

import numpy as np


def uav_cyl_dist3d(pos_ned, cyl_xy, r, h_top):
    """单 UAV(3,) 与单有限高圆柱的三维表面距离 [m]。"""
    h_uav = -float(pos_ned[2])
    d_horiz = max(math.hypot(float(pos_ned[0]) - float(cyl_xy[0]),
                             float(pos_ned[1]) - float(cyl_xy[1])) - float(r), 0.0)
    if h_uav > h_top:
        dz = h_uav - h_top
    elif h_uav < 0.0:
        dz = -h_uav
    else:
        dz = 0.0
    return math.sqrt(d_horiz ** 2 + dz ** 2)


def min_building_dist3d(pos_ned, p_swarm):
    """单 UAV 到所有建筑物的最小三维表面距离 [m]；无建筑物返回 nan。"""
    n = getattr(p_swarm, "n_cyl", 0)
    heights = getattr(p_swarm, "cyl_heights", None)
    if n == 0 or heights is None or np.asarray(heights).size == 0:
        return float("nan")
    cyl = np.asarray(p_swarm.cylinders, dtype=float)
    heights = np.asarray(heights, dtype=float).ravel()
    best = float("inf")
    for i in range(min(n, heights.size)):
        d = uav_cyl_dist3d(pos_ned, cyl[0:2, i], cyl[2, i], heights[i])
        if d < best:
            best = d
    return best


def min_building_dist3d_all(pos_k, p_swarm):
    """(3,N) 位置 → (N,) 每机到建筑物最小三维表面距离；无建筑物返回全 nan。"""
    pos_k = np.asarray(pos_k, dtype=float)
    n = pos_k.shape[1]
    return np.array([min_building_dist3d(pos_k[:, j], p_swarm) for j in range(n)])
