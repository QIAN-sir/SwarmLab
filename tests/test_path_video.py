# -*- coding: utf-8 -*-
"""Stage 9 —— 路径可视化 + 视频录制测试（headless Agg + ffmpeg mp4）。

验证:
    - draw_waypoints：fillet(chi_d=-9999) 直连 / dubins 密集点
    - draw_path：flag=1 直线 / flag=2 orbit（wing30 与 quad33 布局，B-24 修复）
    - points_along_dubins_path 密集点连续
    - DroneViewer 路径层（wpt_handle/path_handle 非 None）
    - VideoWriterWithRate 录 mp4 成功
"""
import os
import sys
import tempfile

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

from SwarmLab_py.core import Drone
from SwarmLab_py.enums import DroneType, DroneState
from SwarmLab_py.parameters import build_params
from SwarmLab_py.guidance import compute_dubins_param
from SwarmLab_py.visualization import (draw_waypoints, draw_path, points_along_dubins_path,
                                       DroneViewer, VideoWriterWithRate)

PASS = []
_TMP = tempfile.mkdtemp(prefix="swarmlab_pathvid_")


def _check(name, cond):
    PASS.append((name, bool(cond)))
    print(("[PASS] " if cond else "[FAIL] ") + name)


def _ax3d():
    fig = plt.figure(figsize=(6, 6))
    return fig, fig.add_subplot(111, projection="3d")


def test_draw_waypoints_fillet():
    fig, ax = _ax3d()
    wpt = np.array([[0, 0, -50, -9999, 35], [300, 0, -50, -9999, 35],
                    [300, 300, -50, -9999, 35]], float)
    ln = draw_waypoints(ax, wpt, 50.0)
    xd, yd = ln.get_data()
    _check("draw_waypoints fillet: 点数=航点数(3)", len(xd) == 3)
    _check("draw_waypoints fillet: x=east", np.allclose(xd, wpt[:, 1]))
    plt.close(fig)


def test_draw_waypoints_dubins():
    fig, ax = _ax3d()
    wpt = np.array([[0, 0, -50, 0.785, 35], [300, 0, -50, 0.785, 35]], float)
    ln = draw_waypoints(ax, wpt, 50.0)
    xd, yd = ln.get_data()
    _check("draw_waypoints dubins: 密集点>航点数", len(xd) > 20)
    _check("draw_waypoints dubins: 起点≈(0,0)", abs(xd[0]) < 1e-6 and abs(yd[0]) < 1e-6)
    plt.close(fig)


def test_points_along_dubins_path():
    dp = compute_dubins_param(np.array([0, 0, -50, 0, 0, 0.0]),
                              np.array([300, 0, -50, 0, 0, 0.0]), 50.0)
    X, Y, Z = points_along_dubins_path(dp, 0.1)
    _check("points_along_dubins: 点数>0", len(X) > 0)
    _check("points_along_dubins: Z 恒定(-50)", np.allclose(Z, -50.0))
    _check("points_along_dubins: 起点≈ps", abs(X[0] - dp["ps"][0]) < 1e-6 and abs(Y[0] - dp["ps"][1]) < 1e-6)


def test_draw_path_straight():
    fig, ax = _ax3d()
    path = np.zeros(30)
    path[0] = 1.0
    path[2:5] = [0, 0, -50]          # r
    path[5:8] = [1, 0, 0]            # q 朝北
    ln = draw_path(ax, path, 100.0)
    xd, yd = ln.get_data()
    _check("draw_path 直线: 2 端点", len(xd) == 2)
    _check("draw_path 直线: 沿 north(y) 延伸 100", abs(yd[1] - yd[0] - 100.0) < 1e-9)
    plt.close(fig)


def test_draw_path_orbit_wing_layout():
    fig, ax = _ax3d()
    path = np.zeros(30)              # wing 布局：c@8:11, rho@11
    path[0] = 2.0
    path[8:11] = [100, 0, -50]
    path[11] = 50.0
    ln = draw_path(ax, path, 100.0)
    xd, yd = ln.get_data()
    _check("draw_path orbit(wing30): 101 点", len(xd) == 101)
    _check("draw_path orbit(wing30): 圆心 north=100", abs((yd.max() + yd.min()) / 2 - 100.0) < 1e-6)
    _check("draw_path orbit(wing30): 半径=50", abs(yd.max() - yd.min() - 100.0) < 1e-6)
    plt.close(fig)


def test_draw_path_orbit_quad_layout_B24():
    """B-24：quad(33) 布局 c@11:14/rho@14 应被正确读取（原 draw_path.m 按 wing 索引会错位）。"""
    fig, ax = _ax3d()
    path = np.zeros(33)              # quad 布局：r_next@8:11, c@11:14, rho@14
    path[0] = 2.0
    path[8:11] = [999, 999, 999]     # r_next（干扰项，不应被当 c）
    path[11:14] = [200, 0, -50]      # 真 c
    path[14] = 50.0                  # 真 rho
    ln = draw_path(ax, path, 100.0)
    xd, yd = ln.get_data()
    _check("draw_path orbit(quad33,B-24): 圆心 north=200(取 c@11:14)", abs((yd.max() + yd.min()) / 2 - 200.0) < 1e-6)
    _check("draw_path orbit(quad33,B-24): 半径=50", abs(yd.max() - yd.min() - 100.0) < 1e-6)
    plt.close(fig)


def test_drone_viewer_path_layer():
    b = build_params("fixed_wing")
    d = Drone(DroneType.FIXED_WING, DroneState.NORMAL, b.p_drone, b.p_battery,
              b.p_sim, b.p_physics, b.map)
    d.set_pos(np.array([0.0, 0.0, -200.0]))
    d.plan_path("dubins", 0.0)
    d.manage_path(0.0)
    vw = DroneViewer(d, output_rate=0.0, center_view_on_drone=False, draw_map=False)
    vw.update(0.0, d, b.map)
    _check("DroneViewer 路径层: wpt_handle 已建", vw.wpt_handle is not None)
    _check("DroneViewer 路径层: path_handle 已建", vw.path_handle is not None)
    vw.close()


def test_video_writer_mp4():
    b = build_params("fixed_wing")
    d = Drone(DroneType.FIXED_WING, DroneState.NORMAL, b.p_drone, b.p_battery,
              b.p_sim, b.p_physics, b.map)
    d.set_pos(np.array([0.0, 0.0, -50.0]))
    d.command = np.array([b.p_drone.Va_T, 50.0, 0.0, 0.0])
    vw = DroneViewer(d, output_rate=0.1, center_view_on_drone=True, draw_map=False)
    vpath = os.path.join(_TMP, "test_video.mp4")
    wr = VideoWriterWithRate(vpath, output_rate=0.1, fps=10)
    for i in range(30):
        t = round(i * 0.05, 4)
        d.update_state(np.zeros(6), t, 1.0)
        vw.update(t, d, b.map)
        wr.update(t, vw.fig)
    wr.close()
    vw.close()
    _check("VideoWriterWithRate: 抓到帧", wr.nframes > 0)
    _check("VideoWriterWithRate: mp4 生成且非空", os.path.exists(vpath) and os.path.getsize(vpath) > 0)


if __name__ == "__main__":
    np.random.seed(0)
    test_draw_waypoints_fillet()
    test_draw_waypoints_dubins()
    test_points_along_dubins_path()
    test_draw_path_straight()
    test_draw_path_orbit_wing_layout()
    test_draw_path_orbit_quad_layout_B24()
    test_drone_viewer_path_layer()
    test_video_writer_mp4()
    n = len(PASS); ok = sum(1 for _, c in PASS if c)
    print("-" * 50)
    print(f"Total: {n}  Passed: {ok}  Failed: {n - ok}")
    sys.exit(0 if ok == n else 1)
