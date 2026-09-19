# -*- coding: utf-8 -*-
"""Stage 7 —— 固定翼路径（path_manager_wing / path_follower_wing / dubins）对照测试。

验证:
    - compute_dubins_param 几何（结构/直线长度/过近返回 None）
    - path_manager_wing 三分支（nb_wpts==0 直线 / fillet / dubins），out 长 30
    - path_follower_wing 直线(flag=1)/orbit(flag=2) → command=[Va_c,h_c,chi_c,phi_ff]
    - B-11 统一：chi 从 canonical z_hat[12] 读取（非 B-M 布局 @8）
    - Drone fixed_wing 闭环（B-18 trim + B-11）：不下坠、状态有限
"""
import math
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import numpy as np

from SwarmLab_py.core import Drone
from SwarmLab_py.enums import DroneType, DroneState
from SwarmLab_py.parameters import build_params
from SwarmLab_py.guidance import (compute_dubins_param, path_manager_wing,
                                  new_pm_state_wing, path_follower_wing)

PASS = []


def _check(name, cond):
    PASS.append((name, bool(cond)))
    print(("[PASS] " if cond else "[FAIL] ") + name)


def _wing():
    b = build_params("fixed_wing")
    d = Drone(DroneType.FIXED_WING, DroneState.NORMAL, b.p_drone, b.p_battery,
              b.p_sim, b.p_physics, b.map)
    return b, b.p_drone, d


def test_compute_dubins_param():
    _, pd, _ = _wing()
    R = pd.radius
    sn = np.array([0, 0, -50, 0, 0, 0.0])       # 起点朝北(chi=0)
    en = np.array([300, 0, -50, 0, 0, 0.0])     # 终点正北 300m，同航向
    dp = compute_dubins_param(sn, en, R)
    _check("dubins: 返回 15 字段结构", isinstance(dp, dict) and len(dp) == 15)
    _check("dubins: R 回显", abs(dp["R"] - R) < 1e-9)
    _check("dubins: 同航向直线 L≈300", abs(dp["L"] - 300.0) < 1.0)
    _check("dubins: q1 单位向量指向北", np.allclose(dp["q1"], [1, 0, 0], atol=1e-6))


def test_dubins_too_close_returns_none():
    _, pd, _ = _wing()
    sn = np.array([0, 0, -50, 0, 0, 0.0])
    en = np.array([1, 0, -50, 0, 0, 0.0])       # 距离 1 << 2R
    _check("dubins: 距离<2R → None（MATLAB []）", compute_dubins_param(sn, en, pd.radius) is None)


def test_path_manager_wing_nb0():
    _, pd, d = _wing()
    d.set_pos(np.array([0.0, 0.0, -50.0]))
    d.z_hat = np.zeros(22)
    d.z_hat[2] = 50.0                            # h
    d.manage_path(0.0)                           # nb_waypoints=None → nb_wpts==0 直线起步
    _check("manager_wing nb0: path 长 30", len(d.path) == 30)
    _check("manager_wing nb0: flag=1(直线)", d.path[0] == 1.0)
    _check("manager_wing nb0: Va_d=Va0", abs(d.path[1] - pd.Va0) < 1e-9)


def test_B11_chi_from_canonical_slot():
    """B-11：chi 必须取自 canonical z_hat[12]（若误读 B-M 布局 @8 会得到错误 q）。"""
    _, pd, d = _wing()
    d.set_pos(np.array([0.0, 0.0, -50.0]))
    d.z_hat = np.zeros(22)
    d.z_hat[2] = 50.0
    d.z_hat[8] = 0.0                             # beta@8（B-M 布局会误当 chi）
    d.z_hat[12] = math.pi / 2                    # chi@12（canonical）→ 朝东
    d.manage_path(0.0)
    q = d.path[5:8]                              # nb0 分支 q=[cos(chi),sin(chi),0]
    _check("B-11: q=[cos(pi/2),sin(pi/2),0]=[0,1,0]（chi 取自@12）",
           np.allclose(q, [0.0, 1.0, 0.0], atol=1e-9))


def test_plan_manage_fillet():
    _, pd, d = _wing()
    d.set_pos(np.array([0.0, 0.0, -50.0]))
    d.plan_path("fillet", 0.0)                   # chi_d=-9999 → |chi_d|>=2π → fillet
    _check("fillet: nb_waypoints=4", d.nb_waypoints == 4)
    d.manage_path(0.0)
    _check("fillet: path 长 30", len(d.path) == 30)
    _check("fillet: flag∈{1,2}", d.path[0] in (1.0, 2.0))


def test_plan_manage_dubins():
    _, pd, d = _wing()
    d.set_pos(np.array([0.0, 0.0, -200.0]))
    d.plan_path("dubins", 0.0)                   # chi_d=45°<2π → dubins
    _check("dubins: nb_waypoints=5", d.nb_waypoints == 5)
    d.manage_path(0.0)
    _check("dubins: path 长 30", len(d.path) == 30)
    _check("dubins: flag=1(初始直线段)", d.path[0] == 1.0)


def test_path_follower_wing_straight():
    _, pd, _ = _wing()
    path = np.zeros(30)
    path[0] = 1.0                                # flag=1 直线
    path[1] = 30.0                               # Va_d
    path[2:5] = [0.0, 0.0, -50.0]                # r_path（h=50）
    path[5:8] = [1.0, 0.0, 0.0]                  # q_path 朝北
    path[13] = 0.0; path[14] = 0.0; path[15] = 50.0   # pn,pe,h（state）
    path[13 + 12] = 0.0                          # chi@canonical
    cmd = path_follower_wing(path, pd)
    _check("follower_wing 直线: command 长 4", cmd.shape == (4,))
    _check("follower_wing 直线: Va_c=Va_d=30", abs(cmd[0] - 30.0) < 1e-9)
    _check("follower_wing 直线: h_c=-r_path[2]=50", abs(cmd[1] - 50.0) < 1e-9)


def test_path_follower_wing_orbit():
    _, pd, _ = _wing()
    path = np.zeros(30)
    path[0] = 2.0                                # flag=2 orbit
    path[1] = 35.0                               # Va_d
    path[8:11] = [100.0, 0.0, -50.0]             # c_orbit（圆心, cd=-50→h_c=50）
    path[11] = 50.0                              # rho_orbit
    path[12] = 1.0                               # lambda（CW）
    path[13] = 150.0; path[14] = 0.0; path[15] = 50.0   # pn,pe,h
    cmd = path_follower_wing(path, pd)
    _check("follower_wing orbit: command 长 4", cmd.shape == (4,))
    _check("follower_wing orbit: Va_c=35", abs(cmd[0] - 35.0) < 1e-9)
    _check("follower_wing orbit: h_c=-c_orbit[2]=50", abs(cmd[1] - 50.0) < 1e-9)
    _check("follower_wing orbit: chi_c 有限", math.isfinite(cmd[2]))


def test_wing_fillet_closed_loop_no_dive():
    """B-18+B-11 综合：trim 初始化的 wing 沿 fillet 路径闭环飞，不下坠、状态有限。"""
    b, pd, d = _wing()
    d.set_pos(np.array([0.0, 0.0, -50.0]))
    d.plan_path("fillet", 0.0)
    hs = []
    for i in range(200):
        t = round(i * b.p_sim.dt, 4)
        d.manage_path(t)
        d.follow_path()
        d.update_state(np.zeros(6), t, 1.0)
        hs.append(-d.pos_ned[2])
    _check("wing fillet 闭环: 状态有限", np.all(np.isfinite(d.pos_ned)) and np.all(np.isfinite(d.attitude)))
    _check("wing fillet 闭环: 不自由落体(2s 内 h>30)", min(hs) > 30.0)
    _check("wing fillet 闭环: 空速维持在飞行区间(Va>10)", d.z_hat[6] > 10.0)


def test_wing_dubins_closed_loop():
    b, pd, d = _wing()
    d.set_pos(np.array([0.0, 0.0, -200.0]))
    d.plan_path("dubins", 0.0)
    for i in range(200):
        t = round(i * b.p_sim.dt, 4)
        d.manage_path(t)
        d.follow_path()
        d.update_state(np.zeros(6), t, 1.0)
    _check("wing dubins 闭环: 状态有限", np.all(np.isfinite(d.pos_ned)) and np.all(np.isfinite(d.attitude)))
    _check("wing dubins 闭环: path 长 30", len(d.path) == 30)


if __name__ == "__main__":
    np.random.seed(0)
    test_compute_dubins_param()
    test_dubins_too_close_returns_none()
    test_path_manager_wing_nb0()
    test_B11_chi_from_canonical_slot()
    test_plan_manage_fillet()
    test_plan_manage_dubins()
    test_path_follower_wing_straight()
    test_path_follower_wing_orbit()
    test_wing_fillet_closed_loop_no_dive()
    test_wing_dubins_closed_loop()
    n = len(PASS); ok = sum(1 for _, c in PASS if c)
    print("-" * 50)
    print(f"Total: {n}  Passed: {ok}  Failed: {n - ok}")
    sys.exit(0 if ok == n else 1)
