# -*- coding: utf-8 -*-
"""compute_swarm_performance —— 群体性能指标（Stage 14 评价体系重构）。

对应 MATLAB: math_tools/compute_swarm_performance.m（仅参考，非正确性标准）。
输入 pos_history/vel_history (T, 3N)，输出各指标**时间序列**（原始量），统计量另由
compute_performance_stats 给出（时间序列→统计），便于画 Safety/Connectivity/Consensus-Time 曲线。

指标口径（与当前三维仿真模型/失效模型一致）:
    safety     UAV-UAV 安全性：全体 UAV（含失效，失效仍是物理实体）3D 两两距离 < 2*r_coll 记碰撞；
               safety[k] = 1 - 碰撞对数/总对数。
    order      一致性（速度/航向协同）：**健康 UAV 子集**（失效/零速排除，避免除零与失真）
               邻居速度方向余弦均值。反映群体运动方向协同，非位置接近。
    union      连通性（图论）：**健康 UAV 子集**通信图(3D 距离<r)连通分量 → (N_h-n_comp)/(N_h-1)。
               真·网络连通（非局部邻居数）。
    alg_conn   代数连通度：健康子集拉普拉斯 Fiedler / N_h。
    safety_obs UAV-建筑物安全性：每步有无 UAV 与建筑物**三维表面**距离 < r_coll（碰撞）；
               safety_obs[k] = 1 - 碰撞机数/N。高空飞越(垂直距离大)不再误判。
    min_d_obs  每机到建筑物**三维表面**(点-有限高圆柱)最小距离 (T,N)；无建筑=nan。

健康子集判定：优先 states_history[k][i]=="Normal"（若提供）；否则速度范数>VEL_EPS（失效
Stalled 速度=0、Drifting 衰减趋 0 → 被排除）。

⚠️ 对原项目的修正（非简单迁移）：原 MATLAB 的建筑物距离/碰撞用 2D 水平距离（高空误判）、
order/连通含失效 UAV（除零/口径失真）、min_d_obs 无垂直分量。此处按当前三维模型修正。
球形障碍距离亦修正为 3D（原 B-15 的 2D/3D 维度不匹配在评价层消除）。
"""

import os

import numpy as np
from scipy.spatial.distance import pdist, squareform, cdist

from .compute_neighborhood import compute_neighborhood
from .network_components import network_components
from .obstacle_distance import min_building_dist3d_all

VEL_EPS = 1e-3          # 零速判定阈值（失效/悬停排除于一致性/连通性健康子集）


def _compute_alg_connectivity(A) -> float:
    """代数连通度：拉普拉斯矩阵第二小特征值（对应 MATLAB compute_alg_connectivity）。"""
    A = np.asarray(A, dtype=float)
    deg = A.sum(axis=1)              # 行和 = 度
    L = np.diag(deg) - A             # 拉普拉斯
    eig = np.linalg.eigvalsh(L)      # 对称阵 → 实特征值，升序
    return float(eig[1])             # 第二小（Fiedler）


def _healthy_mask(k, vel_k, states_history):
    """第 k 步健康 UAV 掩码 (N,)。优先用 states_history，否则速度范数>eps。"""
    n = vel_k.shape[1]
    if states_history is not None:
        row = states_history[k]
        return np.array([str(s) == "Normal" for s in row])
    return np.linalg.norm(vel_k, axis=0) > VEL_EPS


def compute_swarm_performance(pos_history, vel_history, p_swarm, dirname=None,
                              states_history=None):
    """逐时刻计算群体性能指标。返回 6 元组时间序列（兼容旧接口）:
    (safety, order, union, alg_conn, safety_obs, min_d_obs)。"""
    pos_history = np.asarray(pos_history, dtype=float)
    vel_history = np.asarray(vel_history, dtype=float)
    t_steps, nx = pos_history.shape
    nb_agents = nx // 3
    r_coll = float(p_swarm.r_coll)

    safety = np.ones(t_steps)
    order = np.full(t_steps, np.nan)
    union = np.full(t_steps, np.nan)
    alg_conn = np.full(t_steps, np.nan)
    safety_obs = np.ones(t_steps)
    min_d_obs = np.full((t_steps, nb_agents), np.nan)

    has_cyl = getattr(p_swarm, "n_cyl", 0) > 0
    has_sph = getattr(p_swarm, "is_active_spheres", False) and getattr(p_swarm, "n_spheres", 0) > 0

    for k in range(t_steps):
        pos_k = pos_history[k].reshape(nb_agents, 3).T        # (3,N) 列=无人机
        vel_k = vel_history[k].reshape(nb_agents, 3).T        # (3,N)
        healthy = _healthy_mask(k, vel_k, states_history)

        # ================= UAV-UAV 安全性（全体含失效，3D）=================
        dist_vect_k = pdist(pos_k.T)                          # 3D 压缩两两距离
        if dist_vect_k.size > 0:
            nb_ag_coll = np.sum(dist_vect_k < 2 * r_coll)
            safety[k] = 1 - nb_ag_coll / dist_vect_k.size
        else:
            safety[k] = 1.0

        # ================= 一致性 order（健康子集，速度方向余弦）=================
        distance_matrix_k = squareform(dist_vect_k)
        M_k = compute_neighborhood(distance_matrix_k, p_swarm.r, p_swarm.max_neig)
        hidx = np.flatnonzero(healthy)
        if hidx.size > 0:
            acc = 0.0
            cnt = 0
            for a in hidx:
                neig = (M_k[:, a] == 1) & healthy            # 健康邻居
                nn = int(neig.sum())
                if nn != 0:
                    va = vel_k[:, a]
                    vn = vel_k[:, neig]
                    sa = np.linalg.norm(va)
                    sn = np.linalg.norm(vn, axis=0)
                    with np.errstate(divide="ignore", invalid="ignore"):
                        cos = (va @ vn) / (sa * sn)
                    acc += np.nansum(cos) / nn
                else:
                    acc += 1.0                                # 无邻居视为自一致
                cnt += 1
            order[k] = acc / cnt if cnt else np.nan

        # ================= 连通性 union / alg_conn（健康子集通信图）=================
        if hidx.size > 1:
            A = M_k[np.ix_(hidx, hidx)]
            A = (A + A.T) > 0
            n_comp, _, _ = network_components(A)
            nh = hidx.size
            union[k] = (nh - n_comp) / (nh - 1)
            alg_conn[k] = _compute_alg_connectivity(A.astype(float)) / nh
        elif hidx.size == 1:
            union[k] = 1.0
            alg_conn[k] = 1.0

        # ================= UAV-建筑物 安全性 + 最小距离（三维表面）=================
        if has_cyl or has_sph:
            coll_count = 0
            # 圆柱/建筑物：三维表面距离
            if has_cyl:
                d3 = min_building_dist3d_all(pos_k, p_swarm)   # (N,)
                min_d_obs[k, :] = d3
                coll_count += int(np.nansum(d3 < r_coll))
            # 球形：3D 表面距离（修正原 B-15 的 2D/3D 维度不匹配）
            if has_sph:
                c_spheres = p_swarm.spheres[0:3, :]
                r_spheres = p_swarm.spheres[3, :]
                D_sph = cdist(pos_k.T, c_spheres.T) - r_spheres[None, :]   # (N, n_sph) 3D
                d_sph_min = D_sph.min(axis=1)
                if not has_cyl:
                    min_d_obs[k, :] = d_sph_min
                else:
                    min_d_obs[k, :] = np.nanmin(np.vstack([min_d_obs[k, :], d_sph_min]), axis=0)
                coll_count += int(np.sum(d_sph_min < r_coll))
            safety_obs[k] = 1.0 - coll_count / nb_agents
        else:
            safety_obs[k] = 1.0                                # 无障碍 → 完全安全

    # ================= 保存（时间序列 npz，供复画/统计）=================
    if dirname:
        os.makedirs(dirname, exist_ok=True)
        np.savez(os.path.join(dirname, "performance.npz"),
                 safety=safety, order=order, union=union,
                 alg_conn=alg_conn, safety_obs=safety_obs, min_d_obs=min_d_obs)

    return safety, order, union, alg_conn, safety_obs, min_d_obs


def compute_performance_stats(safety, order, union, alg_conn, safety_obs, min_d_obs):
    """时间序列 → 统计量（原始量与统计分离，便于验证与汇报）。"""
    safety = np.asarray(safety, float)
    order = np.asarray(order, float)
    union = np.asarray(union, float)
    alg_conn = np.asarray(alg_conn, float)
    safety_obs = np.asarray(safety_obs, float)
    min_d_obs = np.asarray(min_d_obs, float)
    with np.errstate(all="ignore"):
        return {
            "uav_collision_steps": int(np.sum(safety < 1.0)),          # 有机-机碰撞的时间步数
            "building_collision_steps": int(np.sum(safety_obs < 1.0)),  # 有机-障碰撞的时间步数
            "building_danger_ratio": float(np.mean(safety_obs < 1.0)),  # 危险时间占比
            "min_building_dist": float(np.nanmin(min_d_obs)) if np.any(np.isfinite(min_d_obs)) else float("nan"),
            "mean_building_dist": float(np.nanmean(min_d_obs)) if np.any(np.isfinite(min_d_obs)) else float("nan"),
            "full_connectivity_ratio": float(np.nanmean(union > 0.999)),  # 全连通时间占比
            "mean_order": float(np.nanmean(order)),                    # 平均一致性
            "mean_safety": float(np.mean(safety)),
            "mean_safety_obs": float(np.mean(safety_obs)),
            "mean_alg_conn": float(np.nanmean(alg_conn)),
        }
