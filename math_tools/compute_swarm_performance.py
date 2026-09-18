"""compute_swarm_performance —— 群体性能指标（6 项）。

对应 MATLAB: math_tools/compute_swarm_performance.m
输入 pos_history/vel_history (T, 3N)，输出各 (T,) 指标（min_d_obs 为 (T,N)）：
    safety     安全性（1 - 机间碰撞比例）
    order      有序性（邻居速度方向余弦一致性）
    union      连通性 (N - 连通分量数)/(N-1)
    alg_conn   代数连通度（拉普拉斯第二小特征值 Fiedler / N）
    safety_obs 避障安全性（1 - 障碍碰撞比例）
    min_d_obs  各机到障碍物最小距离

MATLAB→Python 映射：pdist/squareform→scipy.spatial.distance；pdist2→cdist；eig→numpy.linalg.eigvalsh。

⚠️ B-15（忠实保留）：MATLAB 球形障碍的 min_d_obs 行
    `min(pdist2(pos_k(1:2,:)', c_spheres') - ...)`
   用 2D 的 pos_k(1:2,:) 对 3D 的 c_spheres'（维度不匹配）→ 若启用球形障碍会报错。
   因默认 is_active_spheres=False，此路径为死代码；此处照搬（启用球形时同样报错）。
"""

import os

import numpy as np
from scipy.spatial.distance import pdist, squareform, cdist

from .compute_neighborhood import compute_neighborhood
from .network_components import network_components


def _compute_alg_connectivity(A) -> float:
    """代数连通度：拉普拉斯矩阵第二小特征值（对应 MATLAB compute_alg_connectivity）。"""
    A = np.asarray(A, dtype=float)
    deg = A.sum(axis=1)              # sum(A,2) 行和 = 度
    L = np.diag(deg) - A             # 拉普拉斯
    eig = np.linalg.eigvalsh(L)      # 对称阵 → 实特征值，升序
    return float(eig[1])             # 第二小（Fiedler）；MATLAB eigenvalues(2)


def compute_swarm_performance(pos_history, vel_history, p_swarm, dirname=None):
    """逐时刻计算群体性能指标。返回 (safety, order, union, alg_conn, safety_obs, min_d_obs)。"""
    pos_history = np.asarray(pos_history, dtype=float)
    vel_history = np.asarray(vel_history, dtype=float)
    t_steps, nx = pos_history.shape
    nb_agents = nx // 3

    safety = np.ones(t_steps)
    order = np.zeros(t_steps)
    union = np.zeros(t_steps)
    alg_conn = np.zeros(t_steps)
    safety_obs = np.zeros(t_steps)
    min_d_obs = np.zeros((t_steps, nb_agents))
    nb_obs_coll = np.zeros(t_steps)

    for k in range(t_steps):
        # ================= 安全性（机间碰撞）=================
        pos_k = pos_history[k].reshape(nb_agents, 3).T        # (3,N)，列=无人机
        dist_vect_k = pdist(pos_k.T)                          # 压缩两两距离
        if dist_vect_k.size > 0:
            nb_ag_coll = np.sum(dist_vect_k < 2 * p_swarm.r_coll)
            safety[k] = 1 - nb_ag_coll / dist_vect_k.size
        else:
            safety[k] = 1.0

        # ================= 有序性（速度一致性）=================
        distance_matrix_k = squareform(dist_vect_k)
        M_k = compute_neighborhood(distance_matrix_k, p_swarm.r, p_swarm.max_neig)
        vel_k = vel_history[k].reshape(nb_agents, 3).T        # (3,N)
        for agent in range(nb_agents):
            neig = M_k[:, agent] == 1                         # agent 的邻居掩码（对 i）
            nn = int(neig.sum())
            if nn != 0:
                scalar_prod = vel_k[:, agent] @ vel_k[:, neig]          # (nn,)
                speed_agent = np.linalg.norm(vel_k[:, agent])
                speed_neig = np.sqrt((vel_k[:, neig] ** 2).sum(axis=0))  # (nn,)
                with np.errstate(divide="ignore", invalid="ignore"):
                    order[k] += np.sum(scalar_prod / (speed_agent * speed_neig)) / nn
            else:
                order[k] += 1
        order[k] /= nb_agents

        # ================= 连通性 =================
        A = (M_k + M_k.T) > 0
        n_comp, _, _ = network_components(A)
        union[k] = (nb_agents - n_comp) / (nb_agents - 1)
        alg_conn[k] = _compute_alg_connectivity(A) / nb_agents

        # ================= 避障安全性 =================
        nb_possible_coll = 0
        if p_swarm.is_active_spheres or p_swarm.is_active_cyl or p_swarm.is_active_arena:
            # ---- 球形障碍物 ----
            if p_swarm.is_active_spheres and p_swarm.n_spheres > 0:
                c_spheres = p_swarm.spheres[0:3, :]           # (3, n_sph)
                r_spheres = p_swarm.spheres[3, :]             # (n_sph,)
                D_spheres = cdist(pos_k.T, c_spheres.T)       # (N, n_sph) 3D
                # ⚠️ B-15：MATLAB 此行用 2D pos 对 3D 球心 → 维度不匹配（照搬，启用球形时报错）
                min_d_obs[k, :] = (cdist(pos_k[0:2, :].T, c_spheres.T)
                                   - np.tile(r_spheres, (nb_agents, 1))).min(axis=1)
                nb_obs_coll[k] += np.sum(D_spheres < np.tile(r_spheres, (nb_agents, 1)))
                nb_possible_coll += nb_agents * r_spheres.size

            # ---- 圆柱障碍物 ----
            if p_swarm.is_active_cyl and p_swarm.n_cyl > 0:
                c_cyl = p_swarm.cylinders[0:2, :]             # (2, n_cyl)
                r_cyl = p_swarm.cylinders[2, :]               # (n_cyl,)
                D_cyl = cdist(pos_k[0:2, :].T, c_cyl.T)       # (N, n_cyl) 平面距离
                min_d_obs[k, :] = (D_cyl - r_cyl[None, :]).min(axis=1)
                nb_obs_coll[k] += np.sum(D_cyl < r_cyl[None, :])
                nb_possible_coll += nb_agents * r_cyl.size

            # ---- 边界（arena）----
            if p_swarm.is_active_arena:
                xa = p_swarm.x_arena
                x_coll = np.sum((pos_k[0, :] < xa[0, 0]) | (pos_k[0, :] > xa[0, 1]))
                y_coll = np.sum((pos_k[1, :] < xa[1, 0]) | (pos_k[1, :] > xa[1, 1]))
                z_coll = np.sum((pos_k[2, :] < xa[2, 0]) | (pos_k[2, :] > xa[2, 1]))
                nb_obs_coll[k] += x_coll + y_coll + z_coll
                nb_possible_coll += nb_agents

            if nb_possible_coll > 0:
                safety_obs[k] = 1 - nb_obs_coll[k] / nb_possible_coll

    # ================= 保存（对应 MATLAB save(path)）=================
    if dirname:
        os.makedirs(dirname, exist_ok=True)
        np.savez(os.path.join(dirname, "performance.npz"),
                 safety=safety, order=order, union=union,
                 alg_conn=alg_conn, safety_obs=safety_obs, min_d_obs=min_d_obs)

    return safety, order, union, alg_conn, safety_obs, min_d_obs
