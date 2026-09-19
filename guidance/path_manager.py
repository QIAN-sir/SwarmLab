# -*- coding: utf-8 -*-
"""path_manager_quad / fillet（对应 MATLAB guidance/path_manager/path_manager_quad.m）。

persistent 状态（start_of_simulation / wpt_list_prev / idx / flag_transition /
flag_need_new_wpts）以可变字典 st 承载（计划 §10：persistent → 对象/状态实例），
由 Drone 持有（self.pm_state）。idx 存 0-based（MATLAB idx=2 → 0-based 1）。
"""
import numpy as np


def new_pm_state():
    return {"start_of_simulation": 0, "wpt_list_prev": None, "idx": 1,
            "flag_transition": 1, "flag_need_new_wpts": 0}


def path_manager_quad(in_, p_drone, p_sim, st):
    in_ = np.asarray(in_, dtype=float)
    time = in_[-1]
    if time == 0 or time == p_sim.dt:
        st["start_of_simulation"] = 1
    NN = 0
    nb_wpts = in_[0]
    if nb_wpts == 0:
        flag = 1.0
        Va_d = p_drone.Va0
        NN = 1 + 5 * p_drone.size_wpt_array
        pn, pe, h = in_[NN], in_[NN + 1], in_[NN + 2]
        chi = in_[NN + 12]   # B-11 统一：canonical z_hat chi@12（原 MATLAB 按 B-M 布局误读 @8）
        r = np.array([pn, pe, -h])
        q = np.array([np.cos(chi), np.sin(chi), 0.0])
        c = np.zeros(3)
        rho = p_drone.radius
        lam = 0.0
        state = in_[NN:NN + 16]
        flag_new = 1.0
        out = np.concatenate([[flag, Va_d], r, q, r, c, [rho, lam], state, [flag_new]])
    else:
        out = path_manager_fillet_quad(in_, p_drone, st)
        st["start_of_simulation"] = 0
    return out


def path_manager_fillet_quad(in_, p_drone, st):
    in_ = np.asarray(in_, dtype=float)
    size = p_drone.size_wpt_array
    NN = 0
    wpt_list = in_[1:5 * size + 1].reshape(5, size, order="F")   # 列 = 航点
    NN = 1 + 5 * size
    pn, pe, h = in_[NN], in_[NN + 1], in_[NN + 2]
    state = in_[NN:NN + 16]
    NN += 16
    p = np.array([pn, pe, -h])
    R = p_drone.R_min * 1.3

    if st["start_of_simulation"] or st["wpt_list_prev"] is None:
        st["wpt_list_prev"] = np.zeros((5, size))
        st["flag_need_new_wpts"] = 0
    if np.min(wpt_list == st["wpt_list_prev"]) == 0:
        st["idx"] = 1
        st["wpt_list_prev"] = wpt_list.copy()
        st["flag_transition"] = 1
        st["flag_need_new_wpts"] = 0

    idx = st["idx"]
    wpt_prev = wpt_list[:, idx - 1]
    wpt = wpt_list[:, idx]
    wpt_next = wpt_list[:, idx + 1]
    r_prev = wpt_prev[0:3]
    r = wpt[0:3]
    r_next = wpt_next[0:3]
    q_prev = (r - r_prev) / np.linalg.norm(r - r_prev)
    q = (r_next - r) / np.linalg.norm(r_next - r)
    rho_p = np.acos(-q_prev @ q)
    Va_d = wpt[4]

    ft = st["flag_transition"]
    if ft == 1:
        flag = 1.0
        r_next_out = r.copy()
        r_out = r_prev.copy()
        q_out = q_prev.copy()
        z = wpt[0:3] - (R / np.tan(rho_p / 2)) * q_prev
        c = r_out.copy()
        rho = R
        lam = 1.0
        if (p - z) @ q_prev >= 0:
            flag = 2.0
            st["flag_transition"] = 2
    else:
        flag = 2.0
        r_out = r.copy()
        q_out = q.copy()
        r_next_out = r_next.copy()
        c = r - (R / np.sin(rho_p / 2)) * (q_prev - q) / np.linalg.norm(q_prev - q)
        rho = R
        lam = float(np.sign(q_prev[0] * q[1] - q_prev[1] * q[0]))
        z = r + (R / np.tan(rho_p / 2)) * q
        if (p - z) @ q >= 0:
            if idx < size - 2:
                st["idx"] = idx + 1
            flag = 1.0
            st["flag_transition"] = 1

    return np.concatenate([[flag, Va_d], r_out, q_out, r_next_out, c,
                           [rho, lam], state, [st["flag_need_new_wpts"]]])
