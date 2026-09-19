# -*- coding: utf-8 -*-
"""SwarmLab_py.guidance —— 制导（路径规划/管理/跟踪，对应 MATLAB guidance/，Phase 11 / Stage 5）。

已迁移（quad / point_mass + fixed_wing 主线）:
    path_planner.plan_RRT          直线 RRT（generate_random_node/collision/points_along_path/
                                   down_at_NE/extend_tree/find_minimum_path/smooth_path）
    path_manager.path_manager_quad 四旋翼航点路径管理（fillet 圆角，persistent→状态字典）
    path_manager_wing.path_manager_wing 固定翼路径管理（fillet + dubins 状态机）
    dubins.compute_dubins_param    Dubins 路径参数（RSR/RSL/LSR/LSL 取最短）
    path_follower.path_follower_quad / path_follower_wing 路径跟踪→速度/空速指令
未迁移: plan_RRT_dubins / plan_cover_RRT_dubins（MATLAB 缺失，B-10）、plot_map（可视化，Phase 12）。
"""
from .path_planner import plan_RRT
from .path_manager import path_manager_quad, new_pm_state
from .path_manager_wing import path_manager_wing, new_pm_state_wing
from .path_follower import path_follower_quad, path_follower_wing
from .dubins import compute_dubins_param

__all__ = ["plan_RRT", "path_manager_quad", "new_pm_state",
           "path_manager_wing", "new_pm_state_wing", "compute_dubins_param",
           "path_follower_quad", "path_follower_wing"]
