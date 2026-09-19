# -*- coding: utf-8 -*-
"""SwarmLab_py.guidance —— 制导（路径规划/管理/跟踪，对应 MATLAB guidance/，Phase 11 / Stage 5）。

已迁移（quad / point_mass 主线）:
    path_planner.plan_RRT          直线 RRT（generate_random_node/collision/points_along_path/
                                   down_at_NE/extend_tree/find_minimum_path/smooth_path）
    path_manager.path_manager_quad 航点路径管理（fillet 圆角过渡，persistent→状态字典）
    path_follower.path_follower_quad 路径跟踪→速度指令
未迁移: plan_RRT_dubins / plan_cover_RRT_dubins（MATLAB 缺失，B-10）、path_manager_wing /
        path_follower_wing（固定翼，按需）、plot_map（可视化，Phase 12）。
"""
from .path_planner import plan_RRT
from .path_manager import path_manager_quad, new_pm_state
from .path_follower import path_follower_quad

__all__ = ["plan_RRT", "path_manager_quad", "new_pm_state", "path_follower_quad"]
