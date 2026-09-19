# -*- coding: utf-8 -*-
"""Stage 8 / Phase 12 —— 真实动力学可视化测试（headless Agg）。

验证:
    - define_quad_body/define_wing_body 几何（顶点/面/配色形状，0-based 面索引）
    - rotate/translate 刚体变换
    - DroneViewer：quad/wing 机体渲染 + point_mass 散点，存图成功
    - DroneStatePlotter：quad/wing 时序曲线存图；point_mass 跳过（不建图）
"""
import math
import os
import sys
import tempfile

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import matplotlib
matplotlib.use("Agg")
import numpy as np

from SwarmLab_py.core import Drone
from SwarmLab_py.enums import DroneType, DroneState
from SwarmLab_py.parameters import build_params
from SwarmLab_py.visualization import (DroneViewer, DroneStatePlotter, define_body,
                                       define_quad_body, define_wing_body, rotate, translate)

PASS = []
_TMP = tempfile.mkdtemp(prefix="swarmlab_viz_")


def _check(name, cond):
    PASS.append((name, bool(cond)))
    print(("[PASS] " if cond else "[FAIL] ") + name)


def _drone(dtype):
    b = build_params(dtype)
    dt = {"quadcopter": DroneType.QUADCOPTER, "fixed_wing": DroneType.FIXED_WING,
          "point_mass": DroneType.POINT_MASS}[dtype]
    d = Drone(dt, DroneState.NORMAL, b.p_drone, b.p_battery, b.p_sim, b.p_physics, b.map)
    d.set_pos(np.array([0.0, 0.0, -50.0]))
    return b, d


def test_define_quad_body():
    v, f, c = define_quad_body()
    _check("quad body: vertices (3,21)", v.shape == (3, 21))
    _check("quad body: faces (9,5) 且 0-based 合法", f.shape == (9, 5) and f.min() >= 0 and f.max() < 21)
    _check("quad body: face_colors (9,3)", c.shape == (9, 3))


def test_define_wing_body():
    v, f, c = define_wing_body()
    _check("wing body: vertices (3,10)", v.shape == (3, 10))
    _check("wing body: faces (3,4) 且 0-based 合法", f.shape == (3, 4) and f.min() >= 0 and f.max() < 10)
    _check("wing body: face_colors (3,3)", c.shape == (3, 3))


def test_define_body_dispatch():
    _check("define_body point_mass → 空 faces", define_body("point_mass")[1].shape[0] == 0)
    _check("define_body quadcopter → (9,5)", define_body("quadcopter")[1].shape == (9, 5))
    _check("define_body fixed_wing → (3,4)", define_body("fixed_wing")[1].shape == (3, 4))


def test_rotate_translate():
    pts = np.array([[1.0], [0.0], [0.0]])
    _check("rotate: 零角不变", np.allclose(rotate(pts, 0, 0, 0), pts, atol=1e-12))
    r90 = rotate(pts, 0, 0, math.pi / 2)
    _check("rotate: 保模长(刚体旋转)", abs(np.linalg.norm(r90) - 1.0) < 1e-9)
    _check("rotate: psi=90° body-x → inertial +y(东)", np.allclose(r90.ravel(), [0, 1, 0], atol=1e-9))
    t = translate(pts, 10.0, 20.0, -30.0)
    _check("translate: 每列加 [pn,pe,pd]", np.allclose(t, [[11.0], [20.0], [-30.0]], atol=1e-12))


def test_drone_viewer_renders():
    for dtype in ("quadcopter", "fixed_wing"):
        b, d = _drone(dtype)
        vw = DroneViewer(d, output_rate=0.0, center_view_on_drone=True)
        for i in range(3):
            d.update_state(np.zeros(6), round(i * 0.01, 4), 1.0)
            vw.update(i * 0.01, d, b.map)
        path = os.path.join(_TMP, f"viewer_{dtype}.png")
        vw.save_frame(path)
        _check(f"DroneViewer {dtype}: 存图成功", os.path.exists(path) and os.path.getsize(path) > 0)
        _check(f"DroneViewer {dtype}: 机体面片已绘制", vw.body_coll is not None)
        vw.close()


def test_drone_viewer_point_mass():
    b, d = _drone("point_mass")
    vw = DroneViewer(d, output_rate=0.0, center_view_on_drone=True)
    vw.update(0.0, d, b.map)
    path = os.path.join(_TMP, "viewer_point_mass.png")
    vw.save_frame(path)
    _check("DroneViewer point_mass: 散点分支存图", os.path.exists(path) and vw.body_coll is None)
    vw.close()


def test_state_plotter_renders():
    for dtype in ("quadcopter", "fixed_wing"):
        b, d = _drone(dtype)
        if dtype == "fixed_wing":
            d.command = np.array([d.p_drone.Va_T, 50.0, 0.0, 0.0])
        pl = DroneStatePlotter(dtype, output_rate=0.0)
        for i in range(5):
            d.update_state(np.zeros(6), round(i * 0.01, 4), 1.0)
            pl.update(i * 0.01, d)
        path = os.path.join(_TMP, f"state_{dtype}.png")
        pl.save(path)
        _check(f"StatePlotter {dtype}: 存图成功", os.path.exists(path) and os.path.getsize(path) > 0)
        _check(f"StatePlotter {dtype}: 已累积 5 帧", len(pl._t) == 5)
        pl.close()


def test_state_plotter_point_mass_skips():
    b, d = _drone("point_mass")
    pl = DroneStatePlotter("point_mass", output_rate=0.0)
    pl.update(0.0, d)
    _check("StatePlotter point_mass: 跳过(不建图)", pl.fig is None and not pl._built)
    pl.close()


if __name__ == "__main__":
    np.random.seed(0)
    test_define_quad_body()
    test_define_wing_body()
    test_define_body_dispatch()
    test_rotate_translate()
    test_drone_viewer_renders()
    test_drone_viewer_point_mass()
    test_state_plotter_renders()
    test_state_plotter_point_mass_skips()
    n = len(PASS); ok = sum(1 for _, c in PASS if c)
    print("-" * 50)
    print(f"Total: {n}  Passed: {ok}  Failed: {n - ok}")
    sys.exit(0 if ok == n else 1)
