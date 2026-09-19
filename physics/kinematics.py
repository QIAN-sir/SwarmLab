# -*- coding: utf-8 -*-
"""kinematics_ode_fun —— 刚体 12 维状态导数（对应 MATLAB physics/kinematics_ode_fun.m）。

供 ode23 / scipy solve_ivp 使用：orientation 在积分区间内冻结为 xx（步初状态），
与 MATLAB `ode23(@(t,x) kinematics_ode_fun(time,x,x_old,uu,p_drone))` 一致。
"""
import math

import numpy as np

from ..math_tools import Rb2i


def kinematics_ode_fun(t, x, xx, uu, P):
    """返回 dxdt(12) = [pos_dot(3); v_dot(3); euler_dot(3); rates_dot(3)]。"""
    x = np.asarray(x, dtype=float)
    xx = np.asarray(xx, dtype=float)
    uu = np.asarray(uu, dtype=float)

    # Speed and position
    vx, vy, vz = x[3], x[4], x[5]
    p, q, r = x[9], x[10], x[11]

    # Forces and moments
    fx, fy, fz = uu[0], uu[1], uu[2]
    l, m, n = uu[3], uu[4], uu[5]

    # Orientation (frozen at step-start state xx)
    phi0, theta0, psi0 = xx[6], xx[7], xx[8]

    # Trigonometry
    cr = math.cos(phi0)
    cp = math.cos(theta0)
    sr = math.sin(phi0)
    tp = math.tan(theta0)

    # Rotation matrix inertial->body
    Rbi = Rb2i(phi0, theta0, psi0)

    # Velocity in inertial frame
    pos_dot = Rbi @ np.array([vx, vy, vz])
    pndot, pedot, pddot = pos_dot[0], pos_dot[1], pos_dot[2]

    # Acceleration in body frame
    vxdot = r * vy - q * vz + fx / P.mass
    vydot = p * vz - r * vx + fy / P.mass
    vzdot = q * vx - p * vy + fz / P.mass

    # Rotation-rate matrix
    Si_b = np.array([
        [1, sr * tp, cr * tp],
        [0, cr, -sr],
        [0, sr / cp, cr / cp]])
    pqr = np.array([p, q, r])
    phidot, thetadot, psidot = Si_b @ pqr

    # Rotation acceleration in body frame
    pdot = P.gamma1 * p * q - P.gamma2 * q * r + P.gamma3 * l + P.gamma4 * n
    qdot = P.gamma5 * p * r - P.gamma6 * (p ** 2 - r ** 2) + m / P.Jy
    rdot = P.gamma7 * p * q - P.gamma1 * q * r + P.gamma4 * l + P.gamma8 * n

    return np.array([pndot, pedot, pddot, vxdot, vydot, vzdot,
                     phidot, thetadot, psidot, pdot, qdot, rdot])
