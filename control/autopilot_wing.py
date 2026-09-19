# -*- coding: utf-8 -*-
"""autopilot_wing —— 固定翼自动驾驶仪（对应 MATLAB control/control_drone/autopilot_wing.m）。

autopilot_version 硬编码 = 2 → autopilot_uavbook（uavbook 标准自动驾驶仪）。
输出 y = [delta(4); x_command(19)]，delta = [de; da; dr; dt]。
z_hat 索引(0-based): h=2, Va=6, phi=9, theta=10, chi=12, p=13, q=14, r=15。
command: Va_c=command[0], h_c=command[1], chi_c=command[2]。

⚠️ B-18（记录，不修复，计划 §2.3）：原版 fixed_wing 仿真在最初几秒无人机急速下坠并
   消失于视图（原始团队 fixed_wing 代码不完善/严重 bug）。本迁移**忠实复现**该行为。
"""
import math

import numpy as np

from .pid import pid_loop, reset_pid_param, new_pid_param, saturate_range

_INF = float("inf")
_NAN = float("nan")
_D2R = math.pi / 180.0


def _P(drone, name):
    P = getattr(drone, name)
    if P is None:
        P = new_pid_param()
        setattr(drone, name, P)
    return P


# ======================================================================
# Lateral
# ======================================================================
def phi_da_hold(phi_c, phi, p, p_drone, p_sim, drone):
    P = _P(drone, "P_phi_da")     # MATLAB: isempty(P) 首调用重置（dict 新建即零）
    out, P = pid_loop(phi_c, phi, _INF, p_drone.kp_phi, 0, p_drone.kd_phi,
                      p_drone.da_max, p_sim.dt, p_drone.tau_phi, P)
    drone.P_phi_da = P
    return out


def chi_phi_hold(chi_c, chi, r, reinit, p_drone, p_sim, drone):
    P = _P(drone, "P_chi_phi")
    if reinit:
        reset_pid_param(P)
    out, P = pid_loop(chi_c, chi, r, p_drone.kp_chi, p_drone.ki_chi, 0,
                      p_drone.phi_max, p_sim.dt, 0, P)
    drone.P_chi_phi = P
    return out


# ======================================================================
# Longitudinal
# ======================================================================
def theta_de_hold(theta_c, theta, q, p_drone, p_sim, drone):
    P = _P(drone, "P_theta_de")
    out, P = pid_loop(theta_c, theta, _NAN, p_drone.kp_theta, 0, p_drone.kd_theta,
                      p_drone.de_max, p_sim.dt, p_drone.tau_theta, P)
    drone.P_theta_de = P
    return out


def h_theta_hold(h_c, h, reinit, p_drone, p_sim, drone):
    P = _P(drone, "P_h_theta")
    if reinit:
        reset_pid_param(P)
    out, P = pid_loop(h_c, h, _NAN, p_drone.kp_h, p_drone.ki_h, 0,
                      p_drone.theta_max, p_sim.dt, 0, P)
    drone.P_h_theta = P
    return out


def va_dt_hold(Va_c, Va, reinit, p_drone, p_sim, drone):
    P = _P(drone, "P_va_dt")
    if reinit:
        reset_pid_param(P)
    out, P = pid_loop(Va_c, Va, _NAN, p_drone.kp_va_dt, p_drone.ki_va_dt, 0,
                      p_drone.dt_max, p_sim.dt, 0, P)
    drone.P_va_dt = P
    return out


def va_theta_hold(Va_c, Va, reinit, p_drone, p_sim, drone):
    P = _P(drone, "P_va_theta")
    if reinit:
        reset_pid_param(P)
    out, P = pid_loop(Va_c, Va, _NAN, p_drone.kp_va_theta, p_drone.ki_va_theta, 0,
                      p_drone.theta_max, p_sim.dt, 0, P)
    drone.P_va_theta = P
    return out


# ======================================================================
# Autopilot versions
# ======================================================================
def _xc_wing(h_c, Va_c, phi_c, theta_c, chi_c):
    return np.array([0, 0, h_c, 0, 0, 0, 0, 0, 0, Va_c, 0, 0,
                     phi_c, theta_c, 0, chi_c, 0, 0, 0], dtype=float)


def autopilot_uavbook(Va_c, h_c, chi_c, Va, h, chi, phi, theta, p, q, r,
                      t, control_mode, drone):
    p_drone = drone.p_drone
    p_sim = drone.p_sim

    # ---- Lateral ----
    drone.lat_state_prev = drone.lat_state
    if control_mode == 32 and drone.lat_state_prev == 1 and drone.lat_counter_last_change > 1:
        drone.lat_state = 2
        drone.lat_counter_last_change = 0
    elif control_mode == 32 and drone.lat_state_prev == 2 and drone.lat_counter_last_change > 1:
        drone.lat_state = 1
        drone.lat_counter_last_change = 0
    elif t == 0:
        drone.lat_state = 1

    if t == 0 or drone.lat_state != drone.lat_state_prev:
        drone.lat_init_integrator = 1
        drone.lat_counter_last_change = 0
    else:
        drone.lat_init_integrator = 0

    if drone.lat_state == 1:
        phi_c = chi_phi_hold(chi_c, chi, r, drone.lat_init_integrator, p_drone, p_sim, drone)
        da = phi_da_hold(phi_c, phi, p, p_drone, p_sim, drone)
    else:
        phi_c = min(max(chi_c, -30 * _D2R), 30 * _D2R)
        da = phi_da_hold(phi_c, phi, p, p_drone, p_sim, drone)
    dr = 0.0
    drone.lat_counter_last_change = drone.lat_counter_last_change + p_sim.dt

    # ---- Longitudinal (altitude state machine) ----
    drone.altitude_state_prev = drone.altitude_state
    if h <= p_drone.altitude_take_off_zone:
        drone.altitude_state = 1
    elif h <= h_c - p_drone.h_theta_hold_zone:
        drone.altitude_state = 2
    elif h >= h_c + p_drone.h_theta_hold_zone:
        drone.altitude_state = 3
    else:
        drone.altitude_state = 4

    if t == 0 or drone.altitude_state != drone.altitude_state_prev:
        drone.initialize_integrator = 1
    else:
        drone.initialize_integrator = 0

    if drone.altitude_state == 1:      # take-off zone
        dt = 1.0
        theta_c = 30 * _D2R
        de = theta_de_hold(theta_c, theta, q, p_drone, p_sim, drone)
    elif drone.altitude_state == 2:    # climb
        dt = 1.0
        theta_c = va_theta_hold(Va_c, Va, drone.initialize_integrator, p_drone, p_sim, drone)
        de = theta_de_hold(theta_c, theta, q, p_drone, p_sim, drone)
    elif drone.altitude_state == 3:    # descend
        dt = 0.0
        theta_c = va_theta_hold(Va_c, Va, drone.initialize_integrator, p_drone, p_sim, drone)
        de = theta_de_hold(theta_c, theta, q, p_drone, p_sim, drone)
    else:                              # altitude hold
        theta_c = h_theta_hold(h_c, h, drone.initialize_integrator, p_drone, p_sim, drone)
        de = theta_de_hold(theta_c, theta, q, p_drone, p_sim, drone)
        dt = va_dt_hold(Va_c, Va, drone.initialize_integrator, p_drone, p_sim, drone)

    dt = saturate_range(dt, p_drone.dt_max, 0)
    delta = np.array([de, da, dr, dt])
    return delta, _xc_wing(h_c, Va_c, phi_c, theta_c, chi_c)


def autopilot_tuning(Va_c, h_c, chi_c, Va, h, chi, phi, theta, p, q, r, t, drone):
    """mode=5（tune pitch-to-altitude loop）。仅 tuning 用，默认不启用。"""
    p_drone = drone.p_drone
    p_sim = drone.p_sim
    mode = 5
    if mode == 5:
        if t == 0:
            phi_c = chi_phi_hold(chi_c, chi, r, 1, p_drone, p_sim, drone)
            theta_c = h_theta_hold(h_c, h, 1, p_drone, p_sim, drone)
            dt = va_dt_hold(Va_c, Va, 1, p_drone, p_sim, drone)
        else:
            phi_c = chi_phi_hold(chi_c, chi, r, 0, p_drone, p_sim, drone)
            theta_c = h_theta_hold(h_c, h, 0, p_drone, p_sim, drone)
            dt = va_dt_hold(Va_c, Va, 0, p_drone, p_sim, drone)
        da = phi_da_hold(phi_c, phi, p, p_drone, p_sim, drone)
        de = theta_de_hold(theta_c, theta, q, p_drone, p_sim, drone)
        dr = 0.0
    else:
        da = de = dr = dt = 0.0
        phi_c = theta_c = 0.0
    dt = saturate_range(dt, p_drone.dt_max, 0)
    return np.array([de, da, dr, dt]), _xc_wing(h_c, Va_c, phi_c, theta_c, chi_c)


def autopilot_TECS(Va_c, h_c, chi_c, Va, h, chi, phi, theta, p, q, r, t, drone):
    """TECS 纵向占位（MATLAB 中 de=dt=0 未实现）。"""
    p_drone = drone.p_drone
    p_sim = drone.p_sim
    if t == 0:
        dr = 0.0
        phi_c = chi_phi_hold(chi_c, chi, r, 1, p_drone, p_sim, drone)
    else:
        phi_c = chi_phi_hold(chi_c, chi, r, 0, p_drone, p_sim, drone)
        dr = 0.0
    da = phi_da_hold(phi_c, phi, p, p_drone, p_sim, drone)
    de = 0.0
    dt = 0.0
    theta_c = 0.0
    return np.array([de, da, dr, dt]), _xc_wing(h_c, Va_c, phi_c, theta_c, chi_c)


# ======================================================================
# 主入口
# ======================================================================
def autopilot_wing(drone, control_mode, t):
    z = drone.z_hat
    h, Va = z[2], z[6]
    phi, theta, chi = z[9], z[10], z[12]
    p, q, r = z[13], z[14], z[15]
    Va_c, h_c, chi_c = drone.command[0], drone.command[1], drone.command[2]

    autopilot_version = 2     # MATLAB 硬编码
    if autopilot_version == 1:
        delta, xc = autopilot_tuning(Va_c, h_c, chi_c, Va, h, chi, phi, theta, p, q, r, t, drone)
    elif autopilot_version == 2:
        delta, xc = autopilot_uavbook(Va_c, h_c, chi_c, Va, h, chi, phi, theta, p, q, r,
                                      t, control_mode, drone)
    else:
        delta, xc = autopilot_TECS(Va_c, h_c, chi_c, Va, h, chi, phi, theta, p, q, r, t, drone)
    return np.concatenate([delta, xc])
