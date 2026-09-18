"""可视化示例：运行故障群仿真并输出图像（关键帧 + 轨迹 + 性能曲线）。

对应 MATLAB example_*.m 的 DEBUG/VIDEO 出图部分（headless，Agg 后端，无需显示器）。
演示"故障注入 + 群体避让"主线的可视化：t=15s 注入 30% Stalled，t=30s 注入 30% Drifting，
t=50s 恢复 Normal。

运行:
    python SwarmLab_py/examples/run_swarm_viz.py [olfati_saber|vasarhelyi]
输出:
    SwarmLab_py/examples/output/*.png
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

from SwarmLab_py.simulation import Simulation, FaultEvent          # noqa: E402
from SwarmLab_py.visualization import (              # noqa: E402
    SwarmViewer, plot_trajectories, plot_performance,
)
from SwarmLab_py.math_tools import compute_swarm_performance       # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")


def main():
    algorithm = sys.argv[1] if len(sys.argv) > 1 else "olfati_saber"
    os.makedirs(OUT, exist_ok=True)

    sim = Simulation(
        drone_type="point_mass", algorithm=algorithm, nb_agents=10,
        active_environment=True, end_time=60.0, seed=0,
        fault_schedule=[
            FaultEvent(15.0, "Stalled", 0.3),
            FaultEvent(30.0, "Drifting", 0.3),
            FaultEvent(50.0, "Normal"),
        ],
    )
    print(f"=== 可视化仿真：{algorithm}，10 机 / 60s / n_cyl={sim.p_swarm.n_cyl} ===")

    viewer = SwarmViewer(output_rate=1.0)
    snapshots = {10.0: "1_normal", 20.0: "2_stalled", 40.0: "3_drifting"}
    saved = set()

    def on_step(s):
        viewer.update(s.time, s.swarm, s.params.map)
        for t, name in snapshots.items():
            if name not in saved and abs(s.time - t) < 0.5:
                viewer.save_frame(os.path.join(OUT, f"frame_{name}.png"))
                saved.add(name)
                print(f"  保存帧 t≈{s.time:.1f}s → frame_{name}.png")

    sim.run(on_step=on_step)
    viewer.close()

    # 轨迹图
    pos = sim.get_pos_history()[1:]        # 去 set_pos 首帧，与 time_history 对齐
    vel = sim.get_vel_history()
    colors = sim.swarm.get_colors()
    fig = plot_trajectories(pos, sim.p_swarm.nb_agents, colors, sim.params.map,
                            save_path=os.path.join(OUT, "trajectories.png"))
    plt.close(fig)

    # 性能曲线
    th = np.array(sim.time_history)
    n = min(len(th), pos.shape[0], vel.shape[0])
    metrics = compute_swarm_performance(pos[:n], vel[:n], sim.p_swarm)
    figs = plot_performance(th[:n], *metrics,
                            collisions_history=sim.swarm.collisions_history[:n], dirname=OUT)
    for f in figs.values():
        plt.close(f)

    print(f"\n累计碰撞 [机-机, 机-障, min_dist累计] = {np.round(sim.collisions_sum, 2)}")
    print(f"输出目录: {OUT}")
    print("生成文件:", sorted(os.listdir(OUT)))


if __name__ == "__main__":
    main()
