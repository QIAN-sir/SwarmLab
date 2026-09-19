# -*- coding: utf-8 -*-
"""run_drone_video —— 固定翼沿 Dubins 路径飞行：路径可视化 + 录 mp4（Stage 9，供汇报/回顾）。

对应 MATLAB example_path_manager / example_controller 的可视化+录制部分：
    fixed_wing（B-18 修复后 trim 平飞）plan_path('dubins') → manage/follow 闭环，
    DroneViewer 全局视图叠加航点连线(蓝)+当前 path 段(红)，VideoWriterWithRate 录 mp4。
输出到 SwarmLab_py/examples/output/：drone_path_flight.mp4 + .png。headless（Agg+ffmpeg）。
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
from SwarmLab_py.visualization import DroneViewer, VideoWriterWithRate

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")


def main(nsteps=1000):
    os.makedirs(OUT, exist_ok=True)
    b = build_params("fixed_wing")
    d = Drone(DroneType.FIXED_WING, DroneState.NORMAL, b.p_drone, b.p_battery,
              b.p_sim, b.p_physics, b.map)
    d.set_pos(np.array([0.0, 0.0, -70.0]))        # 起点高度匹配 dubins 航点(-7*10)
    d.plan_path("dubins", 0.0)

    viewer = DroneViewer(d, output_rate=0.1, center_view_on_drone=False, draw_map=True)
    vpath = os.path.join(OUT, "drone_path_flight.mp4")
    writer = VideoWriterWithRate(vpath, output_rate=0.1, fps=10)

    for i in range(nsteps):
        t = round(i * b.p_sim.dt, 4)
        d.manage_path(t)
        d.follow_path()
        d.update_state(np.zeros(6), t, 1.0)
        viewer.update(t, d, b.map)
        writer.update(t, viewer.fig)

    writer.close()
    ppath = os.path.join(OUT, "drone_path_flight.png")
    viewer.save_frame(ppath)
    viewer.close()
    print(f"fixed_wing dubins {nsteps * b.p_sim.dt:.1f}s: pos=({d.pos_ned[0]:.0f},"
          f"{d.pos_ned[1]:.0f}) h={-d.pos_ned[2]:.1f} Va={d.z_hat[6]:.1f}")
    print("video:", vpath, f"({writer.nframes} frames)")
    print("image:", ppath)


if __name__ == "__main__":
    np.random.seed(0)
    main()
