# -*- coding: utf-8 -*-
"""Phase 11 / Stage 5 —— guidance 对照测试（RRT / path_manager / path_follower / Drone 链路）。"""
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import numpy as np

from SwarmLab_py.core import Drone
from SwarmLab_py.enums import DroneState, DroneType
from SwarmLab_py.guidance import (plan_RRT, path_manager_quad, new_pm_state,
                                  path_follower_quad)
from SwarmLab_py.guidance.path_planner import collision
from SwarmLab_py.parameters import build_params

PASS = []


def _check(name, cond):
    PASS.append((name, bool(cond)))
    print(("[PASS] " if cond else "[FAIL] ") + name)


def _map():
    return build_params("point_mass", "olfati_saber").map


def test_plan_rrt():
    mp = _map()
    w = mp.width
    wpt_start = [0, 0, -10, 0, 10]
    wpt_end = [w, w, -10, 0, 10]
    np.random.seed(0)
    path = plan_RRT(wpt_start, wpt_end, mp)
    _check("rrt: path >=2 行 且 7 列", path.ndim == 2 and path.shape[0] >= 2 and path.shape[1] == 7)
    _check("rrt: 起点≈start 终点≈end",
           np.allclose(path[0, 0:2], [0, 0], atol=1e-6)
           and np.allclose(path[-1, 0:2], [w, w], atol=1e-6))
    ok = all(collision(path[i], path[i + 1], mp) == 0 for i in range(path.shape[0] - 1))
    _check("rrt: 平滑路径各段无碰撞", ok)


def test_path_manager_zero_wpts():
    b = build_params("quadcopter")
    st = new_pm_state()
    size = b.p_drone.size_wpt_array
    z = np.zeros(22); z[0:3] = [10, 20, -30]
    in_ = np.concatenate([[0], np.zeros(5 * size), z, [0.0]])
    out = path_manager_quad(in_, b.p_drone, b.p_sim, st)
    _check("pm(nb=0): flag=1 且 flag_new=1", out[0] == 1 and out[-1] == 1)
    _check("pm(nb=0): r=当前pos", np.allclose(out[2:5], [10, 20, 30], atol=1e-9))


def test_path_manager_fillet():
    b = build_params("quadcopter")
    st = new_pm_state()
    size = b.p_drone.size_wpt_array
    wpts = np.array([[0, 0, -10, -9999, 10], [100, 0, -10, -9999, 10],
                     [100, 100, -10, -9999, 10], [100, 200, -10, -9999, 10]], float)
    wpt_list = np.vstack([wpts, np.tile([-9999.0] * 5, (size - 4, 1))])
    z = np.zeros(22); z[0:3] = [10, 10, -10]
    in_ = np.concatenate([[4], wpt_list.flatten(), z, [0.0]])
    out = path_manager_quad(in_, b.p_drone, b.p_sim, st)
    _check("pm(fillet): out 长=33", len(out) == 33)
    _check("pm(fillet): flag in {1,2}", out[0] in (1, 2))
    # 再走一步（t>0）状态机延续
    in2 = np.concatenate([[4], wpt_list.flatten(), z, [b.p_sim.dt]])
    out2 = path_manager_quad(in2, b.p_drone, b.p_sim, st)
    _check("pm(fillet t>0): 仍返回 33 维", len(out2) == 33)


def test_path_follower():
    b = build_params("quadcopter")
    path = np.zeros(33)
    path[1] = 10.0                       # Va_d
    path[2:5] = [0, 0, 10]             # r
    path[5:8] = [1, 0, 0]              # q (north)
    path[8:11] = [100, 0, 10]          # r_next
    path[16:19] = [10, 5, -10]         # state pn,pe,h
    cmd = path_follower_quad(path, b.p_drone)
    _check("follower: command 长4 且有限", len(cmd) == 4 and np.all(np.isfinite(cmd)))
    _check("follower: vn>0 (向北路径)", cmd[1] > 0)


def test_drone_path_chain():
    b = build_params("quadcopter")
    d = Drone(DroneType.QUADCOPTER, DroneState.NORMAL, b.p_drone, b.p_battery,
              b.p_sim, b.p_physics, b.map)
    d.set_pos(np.array([10.0, 10.0, -30.0]))
    d.plan_path("fillet", 0.0)
    _check("drone.plan_path(fillet): nb_waypoints=4", d.nb_waypoints == 4)
    d.manage_path(0.0)
    _check("drone.manage_path: path 长33", len(d.path) == 33)
    d.follow_path()
    _check("drone.follow_path: command 长4 有限", len(d.command) == 4 and np.all(np.isfinite(d.command)))
    # straight_line_rrt 规划（有障碍地图）
    np.random.seed(1)
    d.plan_path("straight_line_rrt", 0.0)
    _check("drone.plan_path(rrt): nb_waypoints>=2", (d.nb_waypoints or 0) >= 2)


if __name__ == "__main__":
    test_plan_rrt()
    test_path_manager_zero_wpts()
    test_path_manager_fillet()
    test_path_follower()
    test_drone_path_chain()
    n = len(PASS); ok = sum(1 for _, c in PASS if c)
    print("-" * 50)
    print(f"Total: {n}  Passed: {ok}  Failed: {n - ok}")
    sys.exit(0 if ok == n else 1)
