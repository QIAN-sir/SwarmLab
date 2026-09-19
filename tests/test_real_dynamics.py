# -*- coding: utf-8 -*-
"""Phase 9 / Stage 4 —— 真实动力学接通测试（quadcopter 闭环）。

验证 Drone.update_state 真实动力学分支：autopilot_quad(velocity) + forces_moments_quad
+ kinematics(RK23) 闭环能悬停保持高度、能响应北向速度指令；fixed_wing 暂抛 NotImplementedError。
（精确 hover/trim 数值对照属 §17/缺口B，后续补。）
"""
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import numpy as np

from SwarmLab_py.core import Drone
from SwarmLab_py.enums import DroneState, DroneType
from SwarmLab_py.parameters import build_params

PASS = []


def _check(name, cond):
    PASS.append((name, bool(cond)))
    print(("[PASS] " if cond else "[FAIL] ") + name)


def _quad_drone():
    b = build_params("quadcopter")
    d = Drone(DroneType.QUADCOPTER, DroneState.NORMAL, b.p_drone, b.p_battery,
              b.p_sim, b.p_physics, b.map)
    d.set_pos(np.array([0.0, 0.0, -20.0]))     # h = 20 m
    return d, b


def test_quad_hover_hold():
    d, b = _quad_drone()
    d.command = np.array([0.0, 0.0, 0.0, 0.0])    # v2: psi_c,vn_c,ve_c,vd_c=0 → 悬停
    wind = np.zeros(6)
    for i in range(500):
        d.update_state(wind, i * b.p_sim.dt, 1.0)
    h = -d.pos_ned[2]
    _check("quad hover: 高度保持(5~30m)", 5.0 < h < 30.0 and np.isfinite(h))
    _check("quad hover: 姿态有界", np.all(np.isfinite(d.attitude))
           and abs(d.attitude[0]) < 0.5 and abs(d.attitude[1]) < 0.5)
    _check("quad hover: delta 在[0,dt_max]", np.all(d.delta >= 0)
           and np.all(d.delta <= d.p_drone.dt_max + 1e-9))


def test_quad_north_response():
    d, b = _quad_drone()
    d.command = np.array([0.0, 2.0, 0.0, 0.0])    # vn_c=2 → 向北
    wind = np.zeros(6)
    for i in range(300):
        d.update_state(wind, i * b.p_sim.dt, 1.0)
    _check("quad: vn_c=2 → 向北移动(pn>0.5)", d.pos_ned[0] > 0.5)


def test_fixed_wing_trim_flight():
    # B-18 修复：固定翼构造时已按 trim 初始化 vel/attitude/delta；给配平空速命令应平飞不下坠。
    b = build_params("fixed_wing")
    pd = b.p_drone
    d = Drone(DroneType.FIXED_WING, DroneState.NORMAL, pd, b.p_battery,
              b.p_sim, b.p_physics, b.map)
    d.set_pos(np.array([0.0, 0.0, -50.0]))          # h=50（构造函数已置 trim vel/attitude/delta）
    d.command = np.array([pd.Va_T, 50.0, 0.0, 0.0])  # Va_c=Va_T, h_c=50, chi_c=0
    hs = []
    for i in range(300):
        d.update_state(np.zeros(6), i * b.p_sim.dt, 1.0)
        hs.append(-d.pos_ned[2])
    _check("fixed_wing: 闭环状态有限", np.all(np.isfinite(d.pos_ned))
           and np.all(np.isfinite(d.attitude)))
    _check("fixed_wing B-18: 3s 不下坠（h∈[45,55]）", 45.0 < hs[-1] < 55.0)
    _check("fixed_wing B-18: 最大下坠 <5m", (50.0 - min(hs)) < 5.0)
    _check("fixed_wing B-18: 空速收敛到 Va_T", abs(d.z_hat[6] - pd.Va_T) < 1.0)
    _check("fixed_wing B-18: 油门收敛到 trim dt_T", abs(d.delta[3] - pd.dt_T) < 0.05)
    print(f"    [info] fixed_wing 3s: h={hs[-1]:.2f}m Va={d.z_hat[6]:.2f} dt={d.delta[3]:.4f}(trim {pd.dt_T:.4f})")


if __name__ == "__main__":
    test_quad_hover_hold()
    test_quad_north_response()
    test_fixed_wing_trim_flight()
    n = len(PASS); ok = sum(1 for _, c in PASS if c)
    print("-" * 50)
    print(f"Total: {n}  Passed: {ok}  Failed: {n - ok}")
    sys.exit(0 if ok == n else 1)
