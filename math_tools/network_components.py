"""network_components —— 无向图的连通分量（第三方算法，Daniel Larremore, 2014）。

对应 MATLAB: math_tools/network_components.m
    [nComponents, sizes, members] = network_components(A)
输入 A 为邻接矩阵（有向会先对称化；权重不影响分量）。
本项目 compute_swarm_performance 仅用到 nComponents。
"""

import numpy as np


def network_components(A):
    """返回 (nComponents, sizes, members)。

    nComponents : 连通分量数量
    sizes       : 各分量大小（降序）
    members     : list[list[int]]，各分量成员（按分量大小降序）
    """
    A = np.array(A, dtype=float, copy=True)
    N = A.shape[0]
    A[np.arange(N), np.arange(N)] = 0.0           # 去对角（MATLAB A(1:N+1:end)=0）
    A = A + A.T                                    # 对称化
    is_discovered = np.zeros(N, dtype=int)
    members = []
    for n in range(N):
        if not is_discovered[n]:
            members.append([n])
            is_discovered[n] = 1
            ptr = 0
            while ptr < len(members[-1]):
                nbrs = np.flatnonzero(A[:, members[-1][ptr]])
                new_nbrs = nbrs[is_discovered[nbrs] == 0]
                is_discovered[new_nbrs] = 1
                members[-1].extend(new_nbrs.tolist())
                ptr += 1
    n_components = len(members)
    sizes = [len(m) for m in members]
    order = np.argsort(sizes)[::-1]                # 降序
    sizes = [sizes[i] for i in order]
    members = [members[i] for i in order]
    return n_components, sizes, members
