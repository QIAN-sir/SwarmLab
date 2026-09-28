"""SwarmLab_py.math_tools —— 数学/几何/图论工具（对应 MATLAB math_tools/）。

已迁移:
    Rb2i                      机体→NED 旋转矩阵（Phase 4）
    pos2dist                  位置→两两距离矩阵
    compute_neighborhood      距离矩阵→邻接矩阵
    network_components        连通分量（第三方 Larremore 算法）
    compute_swarm_performance 群体性能 6 指标时间序列 + compute_performance_stats 统计量（Stage 14 重构）
    obstacle_distance       UAV-建筑物三维表面距离（点-有限高圆柱）（Stage 14）
其余（rotate/rotate_b2i/translate/x2v_ned/x2vi）在相应 Phase（可视化/物理）按需迁移。
"""

from .rb2i import Rb2i
from .pos2dist import pos2dist
from .compute_neighborhood import compute_neighborhood
from .network_components import network_components
from .compute_swarm_performance import compute_swarm_performance, compute_performance_stats
from .obstacle_distance import uav_cyl_dist3d, min_building_dist3d, min_building_dist3d_all

__all__ = [
    "Rb2i",
    "pos2dist",
    "compute_neighborhood",
    "network_components",
    "compute_swarm_performance",
    "compute_performance_stats",
    "uav_cyl_dist3d",
    "min_building_dist3d",
    "min_building_dist3d_all",
]
