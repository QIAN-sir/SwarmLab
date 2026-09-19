# -*- coding: utf-8 -*-
"""forces_moments —— 机体系力/力矩（对应 MATLAB physics/forces_moments_quad.m / _wing.m）。

输出 out(12) = [F(3); M(3); Va; alpha; beta; wn; we; wd]，与 MATLAB 一致。
坐标/索引：x 为 12 维状态 [pos_ned(3); vel_xyz(3); attitude(3); rates(3)]（0-based）。
"""
import math

import numpy as np

from ..math_tools import Rb2i


def forces_moments_quad(x, delta, wind, p_drone, p_sim, p_physics):
    """四旋翼机体系力/力矩（forces_moments_quad.m）。

    p_sim 仅在被注释的螺旋桨角加速度项中使用（MATLAB 亦注释），保留签名以对齐调用。
    """
    x = np.asarray(x, dtype=float)
    delta = np.asarray(delta, dtype=float)
    wind = np.asarray(wind, dtype=float)

    # UAV velocity wrt inertial frame in body frame
    v_xyz = x[3:6]
    # Euler angles
    phi, theta, psi = x[6], x[7], x[8]
    # Rotation rates
    p, q, r = x[9], x[10], x[11]
    # Actuators quadcopter
    d1, d2, d3, d4 = delta[0], delta[1], delta[2], delta[3]

    Rbi = Rb2i(phi, theta, psi)

    # ---- Wind computation ----
    ws_ned = wind[0:3]      # steady wind in NED frame
    wg_xyz = wind[3:6]      # gusts along body xyz axis
    wg_ned = Rbi @ wg_xyz   # gust in NED = Rbi * gust in body
    w_ned = ws_ned + wg_ned
    wn, we, wd = w_ned[0], w_ned[1], w_ned[2]
    Rib = Rbi.T
    ws_xyz = Rib @ ws_ned   # steady wind in body frame
    w_xyz = ws_xyz + wg_xyz
    va_xyz = v_xyz - w_xyz  # air data wrt inertial, in body frame
    vax, vay, vaz = va_xyz[0], va_xyz[1], va_xyz[2]
    Va = float(np.linalg.norm(va_xyz))
    alpha = math.atan2(vaz, vax) if vax != 0 else 0.0
    beta = math.asin(vay / Va) if Va != 0 else 0.0

    # ---- Forces on quadcopter ----
    weight = p_drone.mass * p_physics.gravity * np.array([
        -math.sin(theta),
        math.cos(theta) * math.sin(phi),
        math.cos(theta) * math.cos(phi)])
    f_thrust = np.array([0.0, 0.0,
                         -p_drone.C_prop * p_drone.k_omega ** 2 * (d1**2 + d2**2 + d3**2 + d4**2)])
    # Non-linear drag（MATLAB 中线性 drag 被注释）
    f_drag = -0.5 * p_physics.rho * Va * np.asarray(p_drone.drag_area, dtype=float) * va_xyz
    f_tot = weight + f_thrust + f_drag

    # ---- Torques on quadcopter ----
    # a_prop = k_omega*(delta-delta_prev)/dt 在 MATLAB 中被置零
    a_prop = np.zeros(4)
    t_prop = p_drone.CD * (p_drone.k_omega * delta) ** 2 + p_drone.J_prop * a_prop
    torque_thrust = np.array([
        p_drone.l_arm * p_drone.C_prop * p_drone.k_omega ** 2 * (-d2**2 + d4**2),
        p_drone.l_arm * p_drone.C_prop * p_drone.k_omega ** 2 * (+d1**2 - d3**2),
        -t_prop[0] + t_prop[1] - t_prop[2] + t_prop[3]])
    torque_aerodyn = np.zeros(3)
    torque_tot = torque_thrust + torque_aerodyn

    return np.concatenate([f_tot, torque_tot, [Va, alpha, beta, wn, we, wd]])


def forces_moments_wing(x, delta, wind, p_drone, p_physics):
    """固定翼机体系力/力矩（forces_moments_wing.m）。"""
    x = np.asarray(x, dtype=float)
    delta = np.asarray(delta, dtype=float)
    wind = np.asarray(wind, dtype=float)

    v_xyz = x[3:6]
    phi, theta, psi = x[6], x[7], x[8]
    p, q, r = x[9], x[10], x[11]
    # Actuators fixed-wing: elevator, aileron, rudder, thrust
    de, da, dr, dt = delta[0], delta[1], delta[2], delta[3]

    Rbi = Rb2i(phi, theta, psi)

    # ---- Wind computation ----
    ws_ned = wind[0:3]
    wg_xyz = wind[3:6]
    wg_ned = Rbi @ wg_xyz
    w_ned = ws_ned + wg_ned
    wn, we, wd = w_ned[0], w_ned[1], w_ned[2]
    Rib = Rbi.T
    ws_xyz = Rib @ ws_ned
    w_xyz = ws_xyz + wg_xyz
    va_xyz = v_xyz - w_xyz
    vax, vay, vaz = va_xyz[0], va_xyz[1], va_xyz[2]
    Va = float(np.linalg.norm(va_xyz))
    alpha = math.atan2(vaz, vax) if vax != 0 else 0.0
    beta = math.asin(vay / Va) if Va != 0 else 0.0
    ca = math.cos(alpha)
    sa = math.sin(alpha)

    # ---- Forces on fixed wing ----
    CL = p_drone.CL0 + p_drone.CL_alpha * alpha
    CD = p_drone.CD0 + p_drone.CD_alpha * alpha
    Cx = -CD * ca + CL * sa
    Cx_q = -p_drone.CD_q * ca + p_drone.CL_q * sa
    Cx_de = -p_drone.CD_de * ca + p_drone.CL_de * sa
    Cz = -CD * sa - CL * ca
    Cz_q = -p_drone.CD_q * sa - p_drone.CL_q * ca
    Cz_de = -p_drone.CD_de * sa - p_drone.CL_de * ca

    weight = p_drone.mass * p_physics.gravity * np.array([
        -math.sin(theta),
        math.cos(theta) * math.sin(phi),
        math.cos(theta) * math.cos(phi)])

    p_dyn = 0.5 * p_physics.rho * Va ** 2
    if Va != 0:
        kc = 0.5 * p_drone.c / Va
        kb = 0.5 * p_drone.b / Va
    else:
        kc = 0.0
        kb = 0.0

    f_drag = p_dyn * p_drone.S_wing * np.array([
        Cx + Cx_q * kc * q + Cx_de * de,
        p_drone.CY0 + p_drone.CY_beta * beta + p_drone.CY_p * kb * p
        + p_drone.CY_r * kb * r + p_drone.CY_da * da + p_drone.CY_dr * dr,
        Cz + Cz_q * kc * q + Cz_de * de])

    f_thrust = 0.5 * p_physics.rho * p_drone.S_prop * p_drone.C_prop * np.array([
        (p_drone.k_motor * dt) ** 2 - Va ** 2, 0.0, 0.0])

    f_tot = weight + f_drag + f_thrust

    # ---- Torques ----
    torque_aerodyn = p_dyn * p_drone.S_wing * np.array([
        p_drone.b * (p_drone.Cl0 + p_drone.Cl_beta * beta + p_drone.Cl_p * kb * p
                     + p_drone.Cl_r * kb * r + p_drone.Cl_da * da + p_drone.Cl_dr * dr),
        p_drone.c * (p_drone.Cm0 + p_drone.Cm_alpha * alpha + p_drone.Cm_q * kc * q
                     + p_drone.Cm_de * de),
        p_drone.b * (p_drone.Cn0 + p_drone.Cn_beta * beta + p_drone.Cn_p * kb * p
                     + p_drone.Cn_r * kb * r + p_drone.Cn_da * da + p_drone.Cn_dr * dr)])
    torque_thrust = np.array([-p_drone.k_TP * (p_drone.k_omega * dt) ** 2, 0.0, 0.0])
    torque_tot = torque_aerodyn + torque_thrust

    return np.concatenate([f_tot, torque_tot, [Va, alpha, beta, wn, we, wd]])
