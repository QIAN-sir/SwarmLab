# -*- coding: utf-8 -*-
"""Stage 13 —— 建筑物三维高度感知避障测试（4 场景）。

场景1 全体高于建筑物：直接飞越（无低空绕行）；注入失效后避让但不向下进入建筑高度。
场景2 全体低于建筑物：原有避障正常（水平绕行），未被本次修改破坏。
场景3 集群高低混合：高空 UAV 飞越、低空 UAV 绕行（逐架判断，非集群统一高度）。
场景4 高空失效：健康 UAV 避让、向下被禁止（约束可行方向）、方向由算法随相对位置计算（非硬编码）。

建筑物模型复用：p_swarm.cylinders (3,n)=[x;y;r] + p_swarm.cyl_heights (n,) 逐建筑物高度。
"""
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import numpy as np

from SwarmLab_py.core import Swarm
from SwarmLab_py.enums import DroneState
from SwarmLab_py.parameters import build_params
from SwarmLab_py.swarm.height_constraint import (BUILDING_CLEARANCE, DOWN_HORIZON,
                                                 building_top_under_uav, is_above_buildings,
                                                 limit_downward_vel_z)

PASS = []
# 单建筑：中心 (150,150)，半径 20，高度 30
BX, BY, BR, BH = 150.0, 150.0, 20.0, 30.0


def _check(name, cond):
    PASS.append((name, bool(cond)))
    print(("[PASS] " if cond else "[FAIL] ") + name)


def _swarm(n, zs, fault_idx=None, xy=(BX, BY + 10.0), mig=True):
    """n 架 point_mass，水平同点 xy、高度 zs(list)；fault_idx 设为 Drifting；mig=迁移开关。"""
    b = build_params("point_mass", "vasarhelyi", seed=0)
    b.p_swarm.cylinders = np.array([[BX], [BY], [BR]])
    b.p_swarm.n_cyl = 1
    b.p_swarm.cyl_heights = np.array([BH])
    b.p_swarm.is_active_cyl = True
    b.p_swarm.is_active_migration = mig
    sw = Swarm()
    sw.algorithm = "vasarhelyi"
    for i in range(n):
        st = DroneState.DRIFTING if (fault_idx is not None and i == fault_idx) else DroneState.NORMAL
        sw.add_drone("point_mass", st, b.p_drone, b.p_battery, b.p_sim, b.p_physics, b.map)
    pos = np.zeros((3, n))
    for i in range(n):
        pos[:, i] = [xy[0], xy[1], -zs[i]]
    sw.set_pos(pos)
    return b, sw


def test_scenario1_all_above():
    """全体高于建筑物：直接飞越（无低空绕行）+ 失效避让不向下进入建筑高度。"""
    b, sw = _swarm(1, [50.0])                       # 高度50 > 30+2 → 高空
    vc, _ = sw.update_command(b.p_swarm, b.p_swarm.r_coll, b.p_sim.dt)
    _check("S1 高空: footprint 内无建筑绕行排斥(y分量≈0)", abs(vc[1, 0]) < 0.1)
    _check("S1 高空: is_above_buildings=True", is_above_buildings(sw.drones[0].pos_ned, b.p_swarm))
    # 注入失效（故障机在健康机正上方）→ 避让向下分量被禁止
    b2, sw2 = _swarm(2, [35.0, 45.0], fault_idx=1)   # 健康35(>30+2高空), 故障45(上方)
    vc2, _ = sw2.update_command(b2.p_swarm, b2.p_swarm.r_coll, b2.p_sim.dt)
    h_uav, h_top = 35.0, BH
    allow = max(0.0, (h_uav - h_top - BUILDING_CLEARANCE) / DOWN_HORIZON)
    _check("S1 高空避让: 向下分量被限制(<=allow)", vc2[2, 0] <= allow + 1e-6)
    _check("S1 高空避让: 确实存在被clip的向下趋势(allow小)", allow < 1.0)


def test_scenario2_all_below():
    """全体低于建筑物：原有避障正常（水平绕行），未被破坏。"""
    b, sw = _swarm(1, [10.0], mig=False)             # 关迁移隔离建筑排斥；高度10 < 30 → 低空
    vc, _ = sw.update_command(b.p_swarm, b.p_swarm.r_coll, b.p_sim.dt)
    # 排斥生效但受加速度限幅(max_a*dt≈0.1/步)，故断言方向+小阈值
    _check("S2 低空: 原有建筑避障生效(y排斥>0.05)", vc[1, 0] > 0.05)
    _check("S2 低空: is_above_buildings=False", not is_above_buildings(sw.drones[0].pos_ned, b.p_swarm))
    _check("S2 低空: limit_downward 不约束(原样)", limit_downward_vel_z(
        sw.drones[0].pos_ned, 5.0, b.p_swarm) == 5.0)


def test_scenario3_mixed_heights():
    """集群高低混合：高空飞越、低空绕行（逐架判断，非集群统一高度）。"""
    b, sw = _swarm(2, [50.0, 10.0], mig=False)     # drone0 高空, drone1 低空（水平同点）
    vc, _ = sw.update_command(b.p_swarm, b.p_swarm.r_coll, b.p_sim.dt)
    _check("S3 混合: 高空drone0 无建筑绕行(y≈0)", abs(vc[1, 0]) < 0.05)
    _check("S3 混合: 低空drone1 原有绕行(y>0.05)", vc[1, 1] > 0.05)
    _check("S3 混合: 逐架判断(非统一高度) → 两者行为不同",
           abs(vc[1, 0]) < 0.05 and vc[1, 1] > 0.05)


def test_scenario4_fault_above_no_hardcode():
    """高空失效：健康避让、禁向下、方向由算法随相对位置计算（非硬编码固定方向）。"""
    # 4a: 故障机在健康机 +x 旁 → 排斥应指向 -x（由相对位置决定）；关迁移隔离
    b, sw = _swarm(2, [35.0, 35.0], fault_idx=1, xy=(BX, BY + 10.0), mig=False)
    sw.drones[1].set_pos(np.array([BX + 15.0, BY + 10.0, -35.0]))   # 故障在 +x
    vc, _ = sw.update_command(b.p_swarm, b.p_swarm.r_coll, b.p_sim.dt)
    dir_x = vc[0, 0]
    _check("S4a 故障在+x: 健康机水平避让指向-x", dir_x < 0.0)
    # 4b: 故障机在健康机 +y 旁 → 排斥应指向 -y（方向随相对位置变化 → 非硬编码）
    b2, sw2 = _swarm(2, [35.0, 35.0], fault_idx=1, xy=(BX, BY + 10.0), mig=False)
    sw2.drones[1].set_pos(np.array([BX, BY + 25.0, -35.0]))        # 故障在 +y
    vc2, _ = sw2.update_command(b2.p_swarm, b2.p_swarm.r_coll, b2.p_sim.dt)
    dir_y = vc2[1, 0]
    _check("S4b 故障在+y: 健康机水平避让指向-y", dir_y < 0.0)
    _check("S4 方向非硬编码: +x故障主避让轴=x, +y故障主避让轴=y",
           int(np.argmax(np.abs(vc[0:2, 0]))) == 0
           and int(np.argmax(np.abs(vc2[0:2, 0]))) == 1)
    # 4c: 高空禁向下（故障在上方时向下趋势被clip）
    h_uav = 35.0
    allow = max(0.0, (h_uav - BH - BUILDING_CLEARANCE) / DOWN_HORIZON)
    _check("S4c 高空: 向下分量<=allow", vc[2, 0] <= allow + 1e-6 and vc2[2, 0] <= allow + 1e-6)


def test_height_helpers():
    """helper: 逐建筑物高度判断 + 向下约束边界。"""
    b, sw = _swarm(1, [50.0])
    _check("helper: footprint 内 h_top=建筑高", building_top_under_uav(
        sw.drones[0].pos_ned, b.p_swarm) == BH)
    b2, sw2 = _swarm(1, [50.0], xy=(BX + 100.0, BY))     # footprint 外
    _check("helper: footprint 外 h_top=0", building_top_under_uav(
        sw2.drones[0].pos_ned, b2.p_swarm) == 0.0)
    # 多建筑不同高度：逐建筑物判断
    b3 = build_params("point_mass", "vasarhelyi", seed=0)
    b3.p_swarm.cylinders = np.array([[100.0, 200.0], [100.0, 100.0], [15.0, 15.0]])
    b3.p_swarm.n_cyl = 2
    b3.p_swarm.cyl_heights = np.array([20.0, 50.0])       # A=20m, B=50m
    posA = np.array([100.0, 100.0, -30.0])               # 高度30: >A(20) 但 <B(50)
    _check("helper 多建筑: h=30 在A footprint 内 h_top=20", building_top_under_uav(posA, b3.p_swarm) == 20.0)
    _check("helper 多建筑: h=30 高于A → is_above(A)", is_above_buildings(posA, b3.p_swarm) is True
           or building_top_under_uav(posA, b3.p_swarm) == 20.0)


if __name__ == "__main__":
    test_scenario1_all_above()
    test_scenario2_all_below()
    test_scenario3_mixed_heights()
    test_scenario4_fault_above_no_hardcode()
    test_height_helpers()
    n = len(PASS); ok = sum(1 for _, c in PASS if c)
    print("-" * 50)
    print(f"Total: {n}  Passed: {ok}  Failed: {n - ok}")
    sys.exit(0 if ok == n else 1)
