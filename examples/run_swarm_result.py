# -*- coding: utf-8 -*-
"""run_swarm_result —— 群体仿真 → SwarmLab_py/result/<alg>_<ts>/ 存视频 + debug 参数图。

对应 MATLAB example_*.m 的结果保存部分（results/results_swarm/<alg>_<timestamp>/）：
    群体故障仿真 → 录 swarm_simulation.mp4 + 故障三态关键帧 + 轨迹图 + 6 项性能图 +
    碰撞/障碍距离图 + performance.npz，全部集中到本次结果目录，便于后续挑选/对比。

运行:
    python SwarmLab_py/examples/run_swarm_result.py [olfati_saber|vasarhelyi] [tag]
输出:
    SwarmLab_py/result/<algorithm>_<yyyy_mm_dd_HH_MM_SS>[_tag]/
"""
import matplotlib
matplotlib.use("Agg")            # headless，先于 pyplot

import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import numpy as np                                   # noqa: E402
import matplotlib.pyplot as plt                      # noqa: E402

from SwarmLab_py.simulation import Simulation, FaultEvent, new_result_dir   # noqa: E402
from SwarmLab_py.visualization import (              # noqa: E402
    SwarmViewer, VideoWriterWithRate, plot_trajectories, plot_performance,
)
from SwarmLab_py.math_tools import compute_swarm_performance                # noqa: E402


def main():
    algorithm = sys.argv[1] if len(sys.argv) > 1 else "olfati_saber"
    tag = sys.argv[2] if len(sys.argv) > 2 else None
    rdir = new_result_dir(algorithm, tag=tag)

    sim = Simulation(
        drone_type="point_mass", algorithm=algorithm, nb_agents=10,
        active_environment=True, end_time=60.0, seed=0,
        fault_schedule=[
            FaultEvent(15.0, "Stalled", 0.3),
            FaultEvent(30.0, "Drifting", 0.3),
            FaultEvent(50.0, "Normal"),
        ],
    )
    print(f"=== 群体仿真 {algorithm}（10 机/60s）→ {rdir} ===")

    viewer = SwarmViewer(output_rate=0.5)
    writer = VideoWriterWithRate(os.path.join(rdir, "swarm_simulation.mp4"),
                                 output_rate=0.1, fps=10)
    snapshots = {10.0: "1_normal", 20.0: "2_stalled", 40.0: "3_drifting"}
    saved = set()

    def on_step(s):
        viewer.update(s.time, s.swarm, s.params.map)
        writer.update(s.time, viewer.fig)
        for t, name in snapshots.items():
            if name not in saved and abs(s.time - t) < 0.5:
                viewer.save_frame(os.path.join(rdir, f"frame_{name}.png"))
                saved.add(name)

    sim.run(on_step=on_step)
    writer.close()
    viewer.close()

    # 轨迹图
    pos = sim.get_pos_history()[1:]
    vel = sim.get_vel_history()
    colors = sim.swarm.get_colors()
    fig = plot_trajectories(pos, sim.p_swarm.nb_agents, colors, sim.params.map,
                            save_path=os.path.join(rdir, "trajectories.png"))
    plt.close(fig)

    # 性能指标（npz + 6 图 + 碰撞 + 障碍距离）
    th = np.array(sim.time_history)
    n = min(len(th), pos.shape[0], vel.shape[0])
    metrics = compute_swarm_performance(pos[:n], vel[:n], sim.p_swarm, dirname=rdir)
    figs = plot_performance(th[:n], *metrics,
                            collisions_history=sim.swarm.collisions_history[:n],
                            dirname=rdir)
    for f in figs.values():
        plt.close(f)

    print(f"累计碰撞 [机-机, 机-障, min_dist累计] = {np.round(sim.collisions_sum, 2)}")
    print("结果目录:", rdir)
    print("生成文件:", sorted(os.listdir(rdir)))


if __name__ == "__main__":
    main()
