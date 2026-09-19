# -*- coding: utf-8 -*-
"""sensors —— IMU + 气压计测量仿真（对应 MATLAB physics/sensors.m）。

输出 y(8) = [gyro_x; gyro_y; gyro_z; accel_x; accel_y; accel_z; static_pres; diff_pres]。
x 为 12 维状态 [pos_ned(3); vel_xyz(3); attitude(3); rates(3)]（0-based）。
MATLAB normrnd(0, sd) → numpy.random.normal(0, sd)。

⚠️ B-21（修复，计划 §2.3 用户授权）：MATLAB/上游原版 y_accel_z 误用 ``fx/mass``
   （应为 ``fz/mass``，与 x/y 轴一致）。根因=复制粘贴笔误；意图=z 轴加速度计测 z 向
   比力。此处按意图改用 fz，使三轴加速度计一致、phi/theta EKF 量测正确。
"""
import math

import numpy as np

from ..math_tools import Rb2i  # noqa: F401  (保留以对齐 MATLAB 依赖；本函数未直接用)


def sensors(x, forces, airdata, p_drone, p_physics):
    """仿真陀螺/加速度计/气压计测量（sensors.m）。

    Inputs:
        x        : 12 维真实状态
        forces   : 机体系力 [fx; fy; fz]
        airdata  : [Va; alpha; beta; wn; we; wd]
        p_drone  : 含 sd_gyro/sd_accel/sd_*_pres/bias_gyro_*/pres0/mass
        p_physics: 含 gravity/rho
    Output:
        y(8)
    """
    x = np.asarray(x, dtype=float).ravel()
    forces = np.asarray(forces, dtype=float).ravel()
    airdata = np.asarray(airdata, dtype=float).ravel()

    # State: pn, pe, pd, phi, theta, psi, p, q, r  (MATLAB 1-based → 0-based)
    pd = x[2]
    phi = x[6]
    theta = x[7]
    p = x[9]
    q = x[10]
    r = x[11]

    # Forces: fx, fy, fz
    fx = forces[0]
    fy = forces[1]
    fz = forces[2]          # B-21 修复：z 轴加速度计需要 fz

    # Air data: Va
    Va = airdata[0]

    g = p_physics.gravity

    # Simulate rate gyros (units are rad/sec)
    y_gyro_x = p + np.random.normal(0, p_drone.sd_gyro) + p_drone.bias_gyro_x
    y_gyro_y = q + np.random.normal(0, p_drone.sd_gyro) + p_drone.bias_gyro_y
    y_gyro_z = r + np.random.normal(0, p_drone.sd_gyro) + p_drone.bias_gyro_z

    # Simulate accelerometers (units of m/s^2)
    y_accel_x = fx / p_drone.mass + g * math.sin(theta) + np.random.normal(0, p_drone.sd_accel)
    y_accel_y = fy / p_drone.mass - g * math.cos(theta) * math.sin(phi) + np.random.normal(0, p_drone.sd_accel)
    # B-21 修复：原为 fx/p_drone.mass，按意图改为 fz/p_drone.mass
    y_accel_z = fz / p_drone.mass - g * math.cos(theta) * math.cos(phi) + np.random.normal(0, p_drone.sd_accel)

    # Simulate pressure sensors
    y_static_pres = p_drone.pres0 + p_physics.rho * g * pd + np.random.normal(0, p_drone.sd_static_pres)
    y_diff_pres = p_physics.rho * Va ** 2 / 2 + np.random.normal(0, p_drone.sd_diff_pres)

    return np.array([y_gyro_x, y_gyro_y, y_gyro_z,
                     y_accel_x, y_accel_y, y_accel_z,
                     y_static_pres, y_diff_pres])
