"""compute_neighborhood —— 由距离矩阵构造邻接矩阵。

对应 MATLAB: math_tools/compute_neighborhood.m
    M(i,j)：列 j 包含 agent j 的邻居 i（有限视野下可能非对称；对角为 0）。

⚠️ B-16（忠实保留，计划 §2.3）：MATLAB 拓扑裁剪循环中
    `M(I(max_neig+1:end,agent), agent) = 0`
   用的是循环变量 `agent`（1..|to_cut|）作为 M 的列索引，而非 `to_cut(agent)`。
   当 to_cut 不是 [1,2,...] 连续序列时会裁错列。仅在某列邻居数 > max_neig
   （即 N > max_neig+1，默认 max_neig=10 → N>11）时才触发；本项目常用 N≤10，不触发。
   此处按 MATLAB 原样迁移，不修正。
"""

import numpy as np


def compute_neighborhood(distance_matrix: np.ndarray, r_comm: float, max_neig: int) -> np.ndarray:
    """返回邻接矩阵 M (N,N)：M[i,j]=1 表示 i 是 j 的邻居。"""
    dm = np.array(distance_matrix, dtype=float, copy=True)
    N = dm.shape[1]
    M = np.ones((N, N)) - np.eye(N, N)
    dm[np.eye(N, dtype=bool)] = np.inf            # 对角置 Inf（自身非邻居）

    # 通信半径约束
    M[dm > r_comm] = 0
    nn = M.sum(axis=0)                            # 各列之和 = 每个 agent 的邻居数

    # 拓扑距离约束（最多 max_neig 个最近邻）
    to_cut = np.flatnonzero(nn > max_neig)
    if to_cut.size > 0:
        I = np.argsort(dm[:, to_cut], axis=0)     # N × |to_cut|，0-based 升序索引
        for agent in range(len(to_cut)):
            rows = I[max_neig:, agent]            # MATLAB I(max_neig+1:end, agent)
            M[rows, agent] = 0                    # ⚠️ B-16：用 agent（非 to_cut[agent]），照搬
    return M
