# -*- coding: utf-8 -*-
"""building height constraint —— 建筑物三维高度感知与高空向下约束（Stage 13）。

解决：无人机已高于建筑物顶部时仍按低空逻辑绕行（圆柱避障仅 2D 水平、无 z 判断）。

提供（逐建筑物、基于实际几何，不用集群统一高度）:
    building_top_under_uav(pos_ned, p_swarm, margin)
        该 UAV 水平 footprint 内（水平距 cyl 中心 < r_obs）建筑物的最大顶部高度 [m]；
        逐建筑物判断（不同建筑不同高度）；footprint 内无建筑 → 0（地面）。
    is_above_buildings(pos_ned, p_swarm, margin)
        该 UAV 是否明显高于其 footprint 内建筑物顶部（高空状态）。
    limit_downward_vel_z(pos_ned, vel_z, p_swarm, margin, horizon)
        高空 UAV "禁止向下进入建筑物高度区域" = **约束可行方向空间**而非指定固定方向：
        仅 clip vel_z(NED 向下正) 的向下量 <= (h_uav - h_top - margin)/horizon，
        水平/向上分量完全不受影响、仍由原算法自行计算；低空 UAV 不约束（原有逻辑基准）。

数据复用：p_swarm.cylinders (3,n_cyl)=[bn;be;r] 与 p_swarm.cyl_heights (n_cyl,) 逐圆柱对应
（build_swarm_params 由 map.buildings_* 生成，索引一致），不重新设计建筑物模型。
"""
import numpy as np

BUILDING_CLEARANCE = 2.0   # 顶部安全余量 [m]（"明显高于"的判定余量）
DOWN_HORIZON = 5.0         # 向下约束的时间 horizon [s]（允许缓慢下降但不进入建筑高度）


def building_top_under_uav(pos_ned, p_swarm, margin=BUILDING_CLEARANCE):
    """该 UAV 水平 footprint 内建筑物最大顶部高度 [m]（逐建筑物；无则 0=地面）。"""
    h_top = 0.0
    n = getattr(p_swarm, "n_cyl", 0)
    heights = getattr(p_swarm, "cyl_heights", None)
    if n == 0 or heights is None:
        return h_top
    heights = np.asarray(heights, dtype=float).ravel()
    if heights.size == 0:
        return h_top
    cyl = np.asarray(p_swarm.cylinders, dtype=float)
    px, py = float(pos_ned[0]), float(pos_ned[1])
    for i in range(min(n, heights.size)):
        r_obs = cyl[2, i]
        if np.hypot(px - cyl[0, i], py - cyl[1, i]) < r_obs:   # 水平 footprint 内
            if heights[i] > h_top:
                h_top = float(heights[i])
    return h_top


def is_above_buildings(pos_ned, p_swarm, margin=BUILDING_CLEARANCE):
    """该 UAV 是否明显高于其 footprint 内建筑物顶部（高空状态）。"""
    h_uav = -float(pos_ned[2])
    return h_uav > building_top_under_uav(pos_ned, p_swarm, margin) + margin


def limit_downward_vel_z(pos_ned, vel_z, p_swarm,
                         margin=BUILDING_CLEARANCE, horizon=DOWN_HORIZON):
    """高空 UAV 禁止向下进入建筑物高度区域（约束可行方向空间，非指定方向）。

    NED z 向下正，向下=vel_z 正。仅限制向下量：vel_z <= max(0,(h_uav-h_top-margin)/horizon)。
    h_uav 接近 h_top 时 allow→0（禁向下）；远高于时 allow 大（允许缓慢下降但不进入）。
    低空 UAV（h_uav <= h_top+margin）原样返回（原有低空避障逻辑基准，不约束）。
    """
    h_uav = -float(pos_ned[2])
    if not getattr(p_swarm, "height_aware_cyl", True):
        return vel_z                                    # 开关关闭：还原 MATLAB 原行为
    h_top = building_top_under_uav(pos_ned, p_swarm, margin)
    if h_uav <= h_top + margin:
        return vel_z                                    # 低空：原有逻辑，不约束
    v_down_allow = max(0.0, (h_uav - h_top - margin) / horizon)
    return min(float(vel_z), v_down_allow)              # 只 clip 向下分量
