"""pos2dist —— 由位置矩阵计算两两距离矩阵。

对应 MATLAB: math_tools/pos2dist.m
    distances = pdist(positions');  dist_matrix = squareform(distances);
"""

import numpy as np
from scipy.spatial.distance import pdist, squareform


def pos2dist(positions: np.ndarray) -> np.ndarray:
    """输入 positions (3, nb_agents)，返回对称两两距离矩阵 (nb_agents, nb_agents)。"""
    positions = np.asarray(positions, dtype=float)
    return squareform(pdist(positions.T))
