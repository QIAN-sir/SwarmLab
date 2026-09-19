# -*- coding: utf-8 -*-
"""path_follower_quad / path_follower_wing（对应 MATLAB guidance/path_following/）。

path_follower_quad：仅直线路径；输出 command = [psi_c; vn_c; ve_c; vd_c]（速度自驾仪指令）。
path_follower_wing：直线(flag=1)/orbit(flag=2)；输出 command = [Va_c; h_c; chi_c; phi_ff]。

⚠️ B-11 统一：state=z_hat[0:16] 用 canonical 布局（chi@12），故 chi 读 path[NN+12]
   （原 MATLAB path_follower_* 按 Beard-McLain 布局读 chi@state[8]，与 true_states 不符）。
"""
import math

import numpy as np


def path_follower_quad(path, p_drone):
    path = np.asarray(path, dtype=float)
    NN = 0
    Va_d = path[1 + NN]
    r_path = path[2 + NN:5 + NN]
    q_path = path[5 + NN:8 + NN]
    r_path_next = path[8 + NN:11 + NN]
    NN = NN + 16
    pn, pe, h = path[NN], path[NN + 1], path[NN + 2]
    chi = path[NN + 12]   # B-11 统一：canonical z_hat chi@12（原 MATLAB 误读 @8）

    qn, qe = q_path[0], q_path[1]
    chi_q = np.arctan2(qe, qn)
    while (chi_q - chi) < -np.pi:
        chi_q += 2 * np.pi
    while (chi_q - chi) > np.pi:
        chi_q -= 2 * np.pi

    Rip = np.array([[np.cos(chi_q), np.sin(chi_q), 0],
                    [-np.sin(chi_q), np.cos(chi_q), 0],
                    [0, 0, 1]])
    ep = Rip @ (np.array([pn, pe, -h]) - r_path)
    epy = ep[1]
    chi_c = chi_q - p_drone.chi_inf * 2 / np.pi * np.arctan(p_drone.k_path * epy)
    h_c = -r_path_next[2]

    vn_c = Va_d * np.cos(chi_c)
    ve_c = Va_d * np.sin(chi_c)
    vd_c = (h - h_c)
    psi_c = chi_c     # MATLAB 注释：不准确（参考 Beard & McLain p21），忠实保留
    return np.array([psi_c, vn_c, ve_c, vd_c])


def path_follower_wing(path, p_drone):
    """固定翼路径跟踪（path_follower_wing.m）：直线(flag=1)/orbit(flag=2)。

    path 布局(wing,30)：[flag, Va_d, r(3), q(3), c(3), rho, lambda, state(16), flag_new]。
    输出 command = [Va_c; h_c; chi_c; phi_ff]（供 autopilot_wing：Va_c=cmd[0],h_c=cmd[1],chi_c=cmd[2]）。
    ⚠️ B-11 统一：state=z_hat[0:16] canonical 布局，chi=state[12]=path[13+12]=path[25]。
    """
    path = np.asarray(path, dtype=float)
    flag = path[0]
    Va_d = path[1]
    r_path = path[2:5]
    q_path = path[5:8]
    c_orbit = path[8:11]
    rho_orbit = path[11]
    lam_orbit = path[12]
    NN = 13                        # state 起点（path[13:29]=z_hat[0:16]）
    pn = path[NN]
    pe = path[NN + 1]
    h = path[NN + 2]
    chi = path[NN + 12]            # B-11：canonical chi@12

    if flag == 1:                  # 跟随直线（r_path, q_path）
        qn, qe = q_path[0], q_path[1]
        chi_q = math.atan2(qe, qn)
        while (chi_q - chi) < -math.pi:
            chi_q += 2 * math.pi
        while (chi_q - chi) > math.pi:
            chi_q -= 2 * math.pi
        Rip = np.array([[math.cos(chi_q), math.sin(chi_q), 0.0],
                        [-math.sin(chi_q), math.cos(chi_q), 0.0],
                        [0.0, 0.0, 1.0]])
        ep = Rip @ (np.array([pn, pe, -h]) - r_path)
        epy = ep[1]
        chi_c = chi_q - p_drone.chi_inf * 2 / math.pi * math.atan(p_drone.k_path * epy)
        h_c = -r_path[2]
        phi_ff = 0.0
    else:                          # flag == 2：跟随 orbit（c_orbit, rho_orbit, lam_orbit）
        cn, ce = c_orbit[0], c_orbit[1]
        d = math.sqrt((pn - cn) ** 2 + (pe - ce) ** 2)
        phi = math.atan2(pe - ce, pn - cn)   # 忠实：覆盖 state 的 phi
        while (phi - chi) < -math.pi:
            phi += 2 * math.pi
        while (phi - chi) > math.pi:
            phi -= 2 * math.pi
        chi_c = phi + lam_orbit * (math.pi / 2 + math.atan(p_drone.k_orbit * (d - rho_orbit) / rho_orbit))
        h_c = -c_orbit[2]
        phi_ff = 0.0

    Va_c = Va_d                    # 指令空速 = 期望空速
    return np.array([Va_c, h_c, chi_c, phi_ff])
