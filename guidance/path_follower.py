# -*- coding: utf-8 -*-
"""path_follower_quad（对应 MATLAB guidance/path_following/path_follower_quad.m）。

仅直线路径；输出 command = [psi_c; vn_c; ve_c; vd_c]（速度自动驾驶仪指令）。
"""
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
    chi = path[NN + 8]

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
