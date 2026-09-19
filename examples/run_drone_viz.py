# -*- coding: utf-8 -*-
"""run_drone_viz —— 单机真实动力学可视化 demo（Stage 8 / Phase 12）。

对应 MATLAB examples_drone/example_controller.m 的可视化部分：
    quadcopter 悬停 + fixed_wing 配平平飞（B-18 修复后），用
    DroneViewer 出机体 3D 图、DroneStatePlotter 出状态时序曲线（real/est/cmd）。
输出到 SwarmLab_py/examples/output/。headless（Agg）。
"""
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import matplotlib
matplotlib.use("Agg")
import numpy as np

from SwarmLab_py.core import Drone
from SwarmLab_py.enums import DroneType, DroneState
from SwarmLab_py.parameters import build_params
from SwarmLab_py.visualization import DroneViewer, DroneStatePlotter

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
_DTYPE = {"quadcopter": DroneType.QUADCOPTER, "fixed_wing": DroneType.FIXED_WING}


def run(dtype, nsteps, command):
    os.makedirs(OUT, exist_ok=True)
    b = build_params(dtype)
    d = Drone(_DTYPE[dtype], DroneState.NORMAL, b.p_drone, b.p_battery,
              b.p_sim, b.p_physics, b.map)
    d.set_pos(np.array([0.0, 0.0, -50.0]))       # h=50（wing 构造时已按 trim 初始化）
    d.command = np.asarray(command, dtype=float)

    viewer = DroneViewer(d, output_rate=0.5, center_view_on_drone=True)
    plotter = DroneStatePlotter(dtype, output_rate=0.1)
    for i in range(nsteps):
        t = round(i * b.p_sim.dt, 4)
        d.update_state(np.zeros(6), t, 1.0)
        viewer.update(t, d, b.map)
        plotter.update(t, d)
    vpath = os.path.join(OUT, f"drone_viewer_{dtype}.png")
    spath = os.path.join(OUT, f"drone_state_{dtype}.png")
    viewer.save_frame(vpath)
    plotter.save(spath)
    viewer.close(); plotter.close()
    print(f"[{dtype}] {nsteps*b.p_sim.dt:.1f}s: h={-d.pos_ned[2]:.1f}m "
          f"Va={d.z_hat[6]:.2f} -> {vpath} , {spath}")


if __name__ == "__main__":
    np.random.seed(0)
    # quad 悬停：psi_c=vn_c=ve_c=vd_c=0
    run("quadcopter", 300, [0.0, 0.0, 0.0, 0.0])
    # fixed_wing 配平平飞：Va_c=Va_T, h_c=50, chi_c=0, phi_ff=0
    b = build_params("fixed_wing")
    run("fixed_wing", 300, [b.p_drone.Va_T, 50.0, 0.0, 0.0])
    print("done. 输出目录:", OUT)
