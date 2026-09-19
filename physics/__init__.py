# -*- coding: utf-8 -*-
"""SwarmLab_py.physics —— 真实动力学（对应 MATLAB physics/，Phase 8 / Stage 4）。

已迁移:
    forces_moments_quad   四旋翼机体系力/力矩（forces_moments_quad.m）
    forces_moments_wing   固定翼机体系力/力矩（forces_moments_wing.m）
    kinematics_ode_fun    刚体 12 维状态导数（kinematics_ode_fun.m，供 solve_ivp/ode23）

未迁移（按需）: get_wind（湍流风场，persistent wind_struct）、sensors、gps（Phase 10 估计链路）。
"""
from .forces_moments import forces_moments_quad, forces_moments_wing
from .kinematics import kinematics_ode_fun

__all__ = ["forces_moments_quad", "forces_moments_wing", "kinematics_ode_fun"]
