"""PhysicsParams —— 物理常量参数。

对应 MATLAB: parameters/param_physics.m
    p_physics.gravity = 9.81;
    p_physics.rho     = 1.2682;

被依赖方: param_sensors(sd_accel)、param_drone(R_min)、forces_moments_*、
          kinematics_ode_fun、compute_tf_model 等均引用 gravity / rho。
"""

from dataclasses import dataclass


@dataclass
class PhysicsParams:
    """物理常量（重力加速度、空气密度）。"""

    gravity: float = 9.81     # [m/s^2]
    rho: float = 1.2682       # [kg/m^3]
