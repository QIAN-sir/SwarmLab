"""true_states —— “真实状态”伪估计器（直接由真实状态构造 z_hat）。

对应 MATLAB: control/control_drone/true_states.m
被 Drone.set_pos / Drone.estimate_states 调用（use_estimation=False 时的默认路径）。

输入 x = [state(12); airdata(6); time(1)]（共 19，1-based → 0-based 见下）:
    state   = [pos_ned(3); vel_xyz(3); attitude(3); rates(3)]
    airdata = [Va; alpha; beta; wn; we; wd]
输出 z_hat（22）:
    [pn; pe; h; vx; vy; vz; Va; alpha; beta; phi; theta; psi;
     chihat; p; q; r; Vghat; wnhat; wehat; bxhat; byhat; bzhat]

⚠️ 注意（承 PROJECT_ARCHITECTURE.md §14 B-11）：本 z_hat 布局与 estimate_states.m 的
   布局在第 4-6、12-13、19 位含义不同；迁移时各自忠实保留，切勿混用。
"""

import math

import numpy as np


def true_states(x):
    """由真实状态 + 空气数据构造 22 维估计状态 z_hat。

    对应 MATLAB true_states.m。x 形如 [state(12); airdata(6); time]。
    """
    x = np.asarray(x, dtype=float).ravel()

    # ---- 状态 (MATLAB x(1..12) → x[0..11]) ----
    pn = x[0]        # inertial North position
    pe = x[1]        # inertial East position
    h = -x[2]        # altitude = -pd
    vx = x[3]
    vy = x[4]
    vz = x[5]
    phi = x[6]       # roll
    theta = x[7]     # pitch
    psi = x[8]       # yaw
    p = x[9]         # body roll rate
    q = x[10]
    r = x[11]

    # ---- 空气数据 (MATLAB x(13..18) → x[12..17]) ----
    Va = x[12]       # airspeed
    alpha = x[13]    # angle of attack
    beta = x[14]     # sideslip
    wn = x[15]       # wind North
    we = x[16]       # wind East
    # wd = x[17]     # wind down（MATLAB 中注释未用）
    # time = x[18]   # 未使用

    # ---- 由真实状态推算的量 ----
    chihat = math.atan2(Va * math.sin(psi) + we, Va * math.cos(psi) + wn)
    Vghat = math.sqrt((Va * math.cos(psi) + wn) ** 2 + (Va * math.sin(psi) + we) ** 2)
    wnhat = wn
    wehat = we
    bxhat = 0.0
    byhat = 0.0
    bzhat = 0.0

    z_hat = np.array([
        pn, pe, h, vx, vy, vz,
        Va, alpha, beta,
        phi, theta, psi,
        chihat, p, q, r,
        Vghat, wnhat, wehat,
        bxhat, byhat, bzhat,
    ])
    return z_hat
