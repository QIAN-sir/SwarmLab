# -*- coding: utf-8 -*-
"""path_manager_wing / fillet / dubins（对应 MATLAB guidance/path_manager/path_manager_wing.m
+ path_manager_fillet.m + path_manager_dubins.m）。

固定翼路径管理：nb_wpts==0 → 直线起步；否则按首航点 chi_d 选 fillet(|chi_d|>=2π) 或 dubins。
输出 out(30) = [flag; Va_d; r(3); q(3); c(3); rho; lambda; state(16); flag_need_new_wpts]
（⚠️ 与 quad 版 path_manager_quad 的 out(33) 不同：wing 无 r_next）。

persistent（start_of_simulation / fillet:wpt_list_prev,idx,flag_transition,flag_need_new_wpts /
dubins:wpt_list_prev,idx_a,idx_b,flag_transition,dubinspath,flag_need_new_wpts,
flag_first_time_in_state）→ 状态字典 st（计划 §10），由 Drone.pm_state 持有。
idx/idx_a/idx_b 存 0-based（MATLAB idx=2→1，idx_a=1→0，idx_b=2→1）。
"""
import math

import numpy as np

from .dubins import compute_dubins_param


def new_pm_state_wing():
    """固定翼路径管理持久状态（对应三个函数的 persistent）。"""
    return {
        "start_of_simulation": 0,
        "fillet": {"wpt_list_prev": None, "idx": 1, "flag_transition": 1,
                   "flag_need_new_wpts": 0},
        "dubins": {"wpt_list_prev": None, "idx_a": 0, "idx_b": 1, "flag_transition": 0,
                   "dubinspath": None, "flag_need_new_wpts": 0, "flag_first_time_in_state": 1},
    }


def _out(flag, Va_d, r, q, c, rho, lam, state, flag_new):
    return np.concatenate([[flag, Va_d], np.asarray(r, float).ravel(),
                           np.asarray(q, float).ravel(), np.asarray(c, float).ravel(),
                           [rho, lam], np.asarray(state, float).ravel(), [flag_new]])


def path_manager_wing(in_, p_drone, p_sim, st):
    """固定翼路径管理主入口（path_manager_wing.m）。"""
    in_ = np.asarray(in_, dtype=float)
    t = in_[-1]
    if t == 0 or t == p_sim.dt:
        st["start_of_simulation"] = 1
    nb_wpts = in_[0]
    if nb_wpts == 0:
        flag = 1.0
        Va_d = p_drone.Va0
        NN = 1 + 5 * p_drone.size_wpt_array
        pn, pe, h = in_[NN], in_[NN + 1], in_[NN + 2]
        chi = in_[NN + 12]   # B-11 统一：canonical z_hat chi@12（原 MATLAB 按 B-M 布局误读 @8）
        r = np.array([pn, pe, -h])
        q = np.array([math.cos(chi), math.sin(chi), 0.0])
        c = np.zeros(3)
        rho = p_drone.radius
        lam = 0.0
        state = in_[NN:NN + 16]
        out = _out(flag, Va_d, r, q, c, rho, lam, state, 1.0)
    else:
        size = p_drone.size_wpt_array
        wpt_list = in_[1:5 * size + 1].reshape(5, size, order="F")
        if abs(wpt_list[3, 0]) >= 2 * math.pi:      # wpt_list(4,1)=chi_d of wpt1
            out = path_manager_fillet_wing(in_, p_drone, st)
        else:
            out = path_manager_dubins(in_, p_drone, st)
        st["start_of_simulation"] = 0
    return out


def path_manager_fillet_wing(in_, p_drone, st):
    """直线航点 + 圆角过渡（path_manager_fillet.m）。out(30)，无 r_next。"""
    in_ = np.asarray(in_, dtype=float)
    size = p_drone.size_wpt_array
    wpt_list = in_[1:5 * size + 1].reshape(5, size, order="F")
    NN = 1 + 5 * size
    pn, pe, h = in_[NN], in_[NN + 1], in_[NN + 2]
    state = in_[NN:NN + 16]
    p = np.array([pn, pe, -h])
    R = p_drone.R_min * 1.3

    f = st["fillet"]
    if st["start_of_simulation"] or f["wpt_list_prev"] is None:
        f["wpt_list_prev"] = np.zeros((5, size))
        f["flag_need_new_wpts"] = 0
    if np.min(wpt_list == f["wpt_list_prev"]) == 0:
        f["idx"] = 1                                  # MATLAB idx=2 (1-based)
        f["wpt_list_prev"] = wpt_list.copy()
        f["flag_transition"] = 1
        f["flag_need_new_wpts"] = 0

    idx = f["idx"]
    wpt_prev = wpt_list[:, idx - 1]
    wpt = wpt_list[:, idx]
    wpt_next = wpt_list[:, idx + 1]
    r_prev = wpt_prev[0:3]
    r = wpt[0:3]
    r_next = wpt_next[0:3]
    q_prev = (r - r_prev) / np.linalg.norm(r - r_prev)
    q = (r_next - r) / np.linalg.norm(r_next - r)
    rho_p = math.acos(-q_prev @ q)
    Va_d = wpt[4]

    if f["flag_transition"] == 1:                     # 直线段 wpt_prev→wpt
        flag = 1.0
        r_out = r_prev.copy()
        q_out = q_prev.copy()
        z = wpt[0:3] - (R / math.tan(rho_p / 2)) * q_prev
        c = r_out.copy()
        rho = R
        lam = 1.0
        if (p - z) @ q_prev >= 0:
            flag = 2.0
            f["flag_transition"] = 2
    else:                                             # 圆角过渡
        flag = 2.0
        c = r - (R / math.sin(rho_p / 2)) * (q_prev - q) / np.linalg.norm(q_prev - q)
        rho = R
        lam = float(np.sign(q_prev[0] * q[1] - q_prev[1] * q[0]))
        z = r + (R / math.tan(rho_p / 2)) * q
        r_out = r.copy()
        q_out = q.copy()
        if (p - z) @ q >= 0:
            if idx < size - 2:                        # MATLAB idx < size_wpt_array-1 (1-based)
                f["idx"] = idx + 1
            flag = 1.0
            f["flag_transition"] = 1

    return _out(flag, Va_d, r_out, q_out, c, rho, lam, state, f["flag_need_new_wpts"])


def path_manager_dubins(in_, p_drone, st):
    """Dubins 路径状态机（path_manager_dubins.m，6 状态）。"""
    in_ = np.asarray(in_, dtype=float)
    size = p_drone.size_waypoint_array
    nb_wpts = int(in_[0])
    wpt_list = in_[1:5 * size + 1].reshape(5, size, order="F")
    NN = 1 + 5 * size
    pn, pe, h = in_[NN], in_[NN + 1], in_[NN + 2]
    state = in_[NN:NN + 16]
    p = np.array([pn, pe, -h])
    R = p_drone.radius

    d = st["dubins"]
    if st["start_of_simulation"]:
        d["wpt_list_prev"] = np.zeros((5, p_drone.size_wpt_array))
        d["flag_need_new_wpts"] = 0
        d["flag_transition"] = 0
        d["flag_first_time_in_state"] = 1

    if np.min(wpt_list == d["wpt_list_prev"]) == 0:
        d["wpt_list_prev"] = wpt_list.copy()
        if st["start_of_simulation"]:
            d["flag_transition"] = 0
        d["idx_a"] = 0                                # MATLAB idx_a=1 (1-based)
        d["idx_b"] = 1                                # MATLAB idx_b=2
        start_node = np.concatenate([wpt_list[0:4, d["idx_a"]], [0.0, 0.0]])
        end_node = np.concatenate([wpt_list[0:4, d["idx_b"]], [0.0, 0.0]])
        d["dubinspath"] = compute_dubins_param(start_node, end_node, R)
        d["flag_need_new_wpts"] = 0
        d["flag_first_time_in_state"] = 1

    dp = d["dubinspath"]
    Va_d = p_drone.Va0
    ft = d["flag_transition"]

    if ft == 0:                                       # 仿真开始
        flag = 1.0
        r, q, c, rho, lam = dp["ps"], dp["q1"], dp["cs"], dp["R"], dp["lams"]
        if d["flag_first_time_in_state"]:
            d["flag_first_time_in_state"] = 0
        d["flag_transition"] = 1
    elif ft == 1:                                     # 沿首个 orbit 直到 intersect H1
        flag = 2.0
        r, q, c, rho, lam = dp["w1"], dp["q1"], dp["cs"], dp["R"], dp["lams"]
        if ((p - dp["w1"]) @ dp["q1"] >= 0) and d["flag_first_time_in_state"] == 1:
            d["flag_transition"] = 2
            d["flag_first_time_in_state"] = 1
        elif (p - dp["w1"]) @ dp["q1"] >= 0:
            d["flag_transition"] = 3
            d["flag_first_time_in_state"] = 1
        else:
            d["flag_first_time_in_state"] = 0
    elif ft == 2:                                     # 沿首个 orbit 直到 H1 右侧
        flag = 2.0
        r, q, c, rho, lam = dp["w1"], dp["q1"], dp["cs"], dp["R"], dp["lams"]
        if (p - dp["w1"]) @ dp["q1"] < 0:
            d["flag_transition"] = 1
            d["flag_first_time_in_state"] = 1
        else:
            d["flag_first_time_in_state"] = 0
    elif ft == 3:                                     # 沿直线直到 intersect H2
        flag = 1.0
        r, q, c, rho, lam = dp["w1"], dp["q1"], dp["cs"], dp["R"], dp["lams"]
        d["flag_first_time_in_state"] = 0
        if (p - dp["w2"]) @ dp["q1"] >= 0:
            d["flag_transition"] = 4
            d["flag_first_time_in_state"] = 1
    elif ft == 4:                                     # 沿第二个 orbit 直到 intersect H3
        flag = 2.0
        r, q, c, rho, lam = dp["w1"], dp["q1"], dp["ce"], dp["R"], dp["lame"]
        d["flag_first_time_in_state"] = 0             # 忠实：此行使下面 ==1 分支恒不触发
        if ((p - dp["w3"]) @ dp["q3"] >= 0) and d["flag_first_time_in_state"] == 1:
            d["flag_transition"] = 5
            d["flag_first_time_in_state"] = 1
        elif (p - dp["w3"]) @ dp["q3"] >= 0:
            if d["idx_a"] == nb_wpts - 2:             # MATLAB idx_a==nb_wpts-1 (1-based)
                d["flag_need_new_wpts"] = 1
                d["idx_b"] = d["idx_a"] + 1
            else:
                d["idx_a"] = d["idx_a"] + 1
                d["idx_b"] = d["idx_a"] + 1
                d["flag_transition"] = 1
                d["flag_first_time_in_state"] = 1
            start_node = np.concatenate([wpt_list[0:4, d["idx_a"]], [0.0, 0.0]])
            end_node = np.concatenate([wpt_list[0:4, d["idx_b"]], [0.0, 0.0]])
            d["dubinspath"] = compute_dubins_param(start_node, end_node, R)
            dp = d["dubinspath"]
        else:
            d["flag_first_time_in_state"] = 0
    else:                                             # ft == 5：沿 orbit 直到 H3 右侧
        flag = 2.0
        r, q, c, rho, lam = dp["w1"], dp["q1"], dp["cs"], dp["R"], dp["lams"]
        d["flag_first_time_in_state"] = 0
        if (p - dp["w1"]) @ dp["q1"] < 0:
            d["flag_transition"] = 4
            d["flag_first_time_in_state"] = 1
        else:
            d["flag_first_time_in_state"] = 0

    return _out(flag, Va_d, r, q, c, rho, lam, state, d["flag_need_new_wpts"])
