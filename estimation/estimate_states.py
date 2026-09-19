# -*- coding: utf-8 -*-
"""estimate_states —— 状态估计器（对应 MATLAB estimation/estimate_states.m）。

由 IMU/气压/GPS 测量 uu(13) 估计无人机状态，输出 z_hat(22)。结构（忠实原版）:
    - 低通滤波：p/q/r（陀螺）、h（静压）、Va（动压）、pn/pe（GPS）、chi/Vg（GPS）
    - 2 状态 EKF：仅估计 phi/theta（滚转/俯仰），量测=三轴加速度
    - 其余（alpha/beta/psi/vx/vy/vz/wn/we/b*）不估计，置 0

⚠️ 本估计器为**固定翼风格**：估计 {pn,pe,h,Va,phi,theta,chi,p,q,r,Vg}，
   **不估计机体速度 vx/vy/vz 与偏航 psi**（原版即置 0；传感器套件无磁力计，yaw 不可观测）。
   故适用于 fixed_wing 高保真；quad 仅姿态/高度可用，速度/偏航无估计（原版设计范围）。

MATLAB persistent x/kf_P/kf1 → 状态字典 st（计划 §10），由 Drone 持有（self.est_state）。

修复的潜在 bug（计划 §2.3 用户授权：定位根因+理解意图+修复+记录）:
    B-11 —— z_hat 布局与 true_states/自驾仪索引不一致（原版把 chi 放 11、p/q/r 放 12-14、
            psi 放 18）。意图=与 true_states 同一规范布局。修复：按规范布局输出（见下）。
    B-22 —— EKF 更新门限原版 ``norm(z-gravity)<0.5``（把标量 g 从每个加速度分量减），
            几乎永不触发→phi/theta 只预测不更新。意图=加速度模≈g 的低动态判据。
            修复：``abs(norm(z)-gravity)<0.5``。
    B-20 —— lp_filter 退化为近似直通（见 lp_filter.py）。修复：各通道带记忆真滤波。
"""
import math

import numpy as np

from .lp_filter import lp_step


def new_est_state():
    """估计器持久状态（对应 MATLAB persistent x/kf_P/kf1 + B-20 的 LP 通道记忆）。"""
    return {
        "x": np.zeros(2),          # EKF 状态 [phi_hat; theta_hat]
        "kf_P": np.zeros((2, 2)),  # EKF 协方差
        "lp": {},                  # 各低通通道上一步输出（B-20）
        "kf1_x_prev": None,        # MATLAB kf1（t=0 初始化后未再使用，忠实保留占位）
        "kf1_P_prev": None,
    }


def estimate_states(uu, time, dt, p_drone, p_physics, st):
    """状态估计（estimate_states.m）。uu(13)=[gyro(3);accel(3);pres(2);gps(5)]；返回 z_hat(22)。"""
    uu = np.asarray(uu, dtype=float).ravel()

    # ---- Rename inputs (MATLAB 1-based → 0-based) ----
    y_gyro_x = uu[0]
    y_gyro_y = uu[1]
    y_gyro_z = uu[2]
    y_accel_x = uu[3]
    y_accel_y = uu[4]
    y_accel_z = uu[5]
    y_static_pres = uu[6]
    y_diff_pres = uu[7]
    y_gps_n = uu[8]
    y_gps_e = uu[9]
    y_gps_h = uu[10]          # 读取但未使用（h 由静压估计，与 MATLAB 一致）
    y_gps_Vg = uu[11]
    y_gps_course = uu[12]

    g = p_physics.gravity
    rho = p_physics.rho

    # ---- Initialize at t=0（对应 MATLAB if time==0）----
    if time == 0:
        st["x"] = np.zeros(2)
        st["kf_P"] = np.zeros((2, 2))
        st["lp"] = {}
        # kf1 为 MATLAB 中初始化后未再使用的遗留字段（忠实保留占位）
        st["kf1_x_prev"] = np.array([getattr(p_drone, "phi_T", 0.0),
                                     getattr(p_drone, "theta_T", 0.0)])
        st["kf1_P_prev"] = np.zeros((2, 2))

    # ---- 低通滤波（B-20 修复：各通道带记忆）----
    phat = lp_step(st["lp"], "p", y_gyro_x, p_drone.a_gyro, dt)
    qhat = lp_step(st["lp"], "q", y_gyro_y, p_drone.a_gyro, dt)
    rhat = lp_step(st["lp"], "r", y_gyro_z, p_drone.a_gyro, dt)

    h_pres = lp_step(st["lp"], "h", y_static_pres, p_drone.a_static_pres, dt)
    hhat = (p_drone.pres0 - h_pres) / (rho * g)

    diff_pres = lp_step(st["lp"], "Va", y_diff_pres, p_drone.a_diff_pres, dt)
    with np.errstate(invalid="ignore"):
        Vahat = float(np.sqrt(2.0 / rho * diff_pres))   # 低空速下 diff_pres 可能为负 → nan（同 MATLAB）

    pnhat = lp_step(st["lp"], "pn", y_gps_n, p_drone.a_gps_pos, dt)
    pehat = lp_step(st["lp"], "pe", y_gps_e, p_drone.a_gps_pos, dt)
    chihat = lp_step(st["lp"], "chi", y_gps_course, p_drone.a_gps_speed, dt)
    Vghat = lp_step(st["lp"], "Vg", y_gps_Vg, p_drone.a_gps_speed, dt)

    # ---- Not estimated（原版置 0）----
    alphahat = 0.0
    betahat = 0.0
    psihat = 0.0
    wnhat = 0.0
    wehat = 0.0
    bxhat = byhat = bzhat = 0.0

    # ---- 2 状态 EKF（phi/theta），量测=三轴加速度 ----
    x_prev = st["x"].copy()
    phi_hat = x_prev[0]
    theta_hat = x_prev[1]
    kf_P_prev = st["kf_P"].copy()

    z = np.array([y_accel_x, y_accel_y, y_accel_z])

    cp = math.cos(phi_hat)
    sp = math.sin(phi_hat)
    ct = math.cos(theta_hat)
    stt = math.sin(theta_hat)
    tt = math.tan(theta_hat)

    p = y_gyro_x
    q = y_gyro_y
    r = y_gyro_z
    Va = Vahat

    f = np.array([p + q * sp * tt + r * cp * tt,
                  q * cp - r * sp])

    F = np.array([[q * cp * tt - r * sp * tt, (q * sp - r * cp) / (ct ** 2)],
                  [-q * sp - r * cp, 0.0]])

    h = np.array([q * Va * stt + g * stt,
                  r * Va * ct - p * Va * stt - g * ct * sp,
                  -q * Va * ct - g * ct * cp])

    H = np.array([[0.0, q * Va * ct + g * ct],
                  [-g * cp * ct, -r * Va * stt - p * Va * ct + g * sp * stt],
                  [g * sp * ct, (q * Va + g * cp) * stt]])

    # ---- Prediction (a priori) ----
    x = x_prev + dt * f
    kf_P = kf_P_prev + (F @ kf_P_prev + kf_P_prev @ F.T + p_drone.Q)

    # ---- Update (a posteriori) ----
    # B-22 修复：原 MATLAB `if norm(z - gravity) < 0.5`（几乎永不触发）→
    #            意图为"加速度模≈g 的低动态"判据 `abs(norm(z) - g) < 0.5`
    if abs(float(np.linalg.norm(z)) - g) < 0.5:
        z_tilde = z - H @ x                       # innovation
        S = p_drone.R + H @ kf_P @ H.T            # innovation covariance
        K = kf_P @ H.T @ np.linalg.inv(S)         # optimal Kalman gain
        x = x + K @ z_tilde                        # a posteriori state
        kf_P = (np.eye(2) - K @ H) @ kf_P          # a posteriori covariance

    st["x"] = x
    st["kf_P"] = kf_P

    phihat = x[0]
    thetahat = x[1]

    # ---- Output z_hat(22)：B-11 修复 —— 采用与 true_states 一致的规范布局 ----
    # idx:  0   1   2   3   4   5   6    7       8       9      10       11(psi) 12(chi) 13  14  15  16   17   18   19  20  21
    z_hat = np.array([
        pnhat, pehat, hhat,
        0.0, 0.0, 0.0,            # vx,vy,vz：本估计器不估计（原版即 0）
        Vahat, alphahat, betahat,
        phihat, thetahat,
        psihat,                   # 11 psi（B-11：规范位；yaw 不可观测→0）
        chihat,                   # 12 chi（B-11：原误置 p）
        phat, qhat, rhat,         # 13,14,15 p,q,r（B-11：原误置 q,r,Vg）
        Vghat, wnhat, wehat,      # 16,17,18 Vg,wn,we（B-11：原误置 Vg,wn,we→psi 错位）
        bxhat, byhat, bzhat,      # 19,20,21
    ])
    return z_hat
