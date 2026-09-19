# -*- coding: utf-8 -*-
"""SwarmLab_py.physics —— 真实动力学（对应 MATLAB physics/，Phase 8 / Stage 4）。

已迁移:
    forces_moments_quad   四旋翼机体系力/力矩（forces_moments_quad.m）
    forces_moments_wing   固定翼机体系力/力矩（forces_moments_wing.m）
    kinematics_ode_fun    刚体 12 维状态导数（kinematics_ode_fun.m，供 solve_ivp/ode23）
    sensors               IMU+气压计测量仿真（sensors.m，Phase 10；含 B-21 修复）
    gps                   GPS 测量仿真（gps.m，Phase 10；persistent Markov→状态字典）

未迁移（按需）: get_wind（湍流风场，persistent wind_struct）。
"""
from .forces_moments import forces_moments_quad, forces_moments_wing
from .kinematics import kinematics_ode_fun
from .sensors import sensors
from .gps import gps, new_gps_state

__all__ = ["forces_moments_quad", "forces_moments_wing", "kinematics_ode_fun",
           "sensors", "gps", "new_gps_state"]
