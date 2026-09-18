"""Stage 2B —— 可视化测试（headless，Agg 后端）。

验证 SwarmViewer 在线渲染（含故障状态 marker）、离线轨迹图、性能指标图能正确
生成并保存为图片文件（不依赖显示器）。

运行方式（无需 pytest）:
    python SwarmLab_py/tests/test_visualization.py
"""

import os
import sys
import tempfile

import matplotlib
matplotlib.use("Agg")            # headless，必须在导入 pyplot / visualization 之前

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


def _png_ok(path):
    return os.path.exists(path) and os.path.getsize(path) > 1000


def test_swarm_viewer_renders_and_saves():
    sim = Simulation(algorithm="olfati_saber", nb_agents=5, end_time=2.0, seed=0,
                     active_environment=True,
                     fault_schedule=[FaultEvent(1.0, "Drifting", 0.4)])
    viewer = SwarmViewer(output_rate=0.5)
    sim.run(on_step=lambda s: viewer.update(s.time, s.swarm, s.params.map))
    assert viewer.plot_initialized is True
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "frame.png")
        viewer.save_frame(p)
        assert _png_ok(p)
    viewer.close()


def test_viewer_fault_markers_present():
    # 注入 Drifting(40%)：应同时存在 Normal('o') 与 Drifting('^') 两类散点
    sim = Simulation(algorithm="vasarhelyi", nb_agents=5, end_time=1.5, seed=0,
                     active_environment=True,
                     fault_schedule=[FaultEvent(0.5, "Drifting", 0.4)])
    viewer = SwarmViewer(output_rate=0.5)
    sim.run(on_step=lambda s: viewer.update(s.time, s.swarm, s.params.map))
    markers = {sc.get_facecolors().shape[0] for sc in viewer._scatters}
    assert len(viewer._scatters) >= 1                 # 至少绘制了一组散点
    n_drifting = sum(1 for d in sim.swarm.drones if d.drone_state == "Drifting")
    assert n_drifting >= 1                            # 确有故障机
    viewer.close()


def test_plot_trajectories_saves():
    sim = Simulation(algorithm="olfati_saber", nb_agents=4, end_time=1.0, seed=0,
                     active_environment=True)
    sim.run()
    pos = sim.get_pos_history()[1:]                   # 去 set_pos 首帧，与 time_history 对齐
    colors = sim.swarm.get_colors()
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "traj.png")
        fig = plot_trajectories(pos, 4, colors, sim.params.map, save_path=p)
        assert _png_ok(p)
        plt.close(fig)


def test_plot_performance_saves():
    sim = Simulation(algorithm="olfati_saber", nb_agents=4, end_time=2.0, seed=0,
                     active_environment=True,
                     fault_schedule=[FaultEvent(1.0, "Drifting", 0.3)])
    sim.run()
    pos = sim.get_pos_history()[1:]
    vel = sim.get_vel_history()
    th = np.array(sim.time_history)
    n = min(len(th), pos.shape[0], vel.shape[0])
    safety, order, union, alg_conn, safety_obs, min_d_obs = \
        compute_swarm_performance(pos[:n], vel[:n], sim.p_swarm)
    assert safety.shape == (n,)
    with tempfile.TemporaryDirectory() as d:
        figs = plot_performance(th[:n], safety, order, union, alg_conn, safety_obs, min_d_obs,
                                collisions_history=sim.swarm.collisions_history[:n], dirname=d)
        assert _png_ok(os.path.join(d, "performance.png"))
        assert _png_ok(os.path.join(d, "collision_history.png"))
        assert _png_ok(os.path.join(d, "obstacle_distance.png"))
        for f in figs.values():
            plt.close(f)


def _run_all():
    tests = [
        test_swarm_viewer_renders_and_saves, test_viewer_fault_markers_present,
        test_plot_trajectories_saves, test_plot_performance_saves,
    ]
    passed = failed = 0
    for t in tests:
        try:
            t()
            print(f"[PASS] {t.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"[FAIL] {t.__name__}: {e}")
            failed += 1
        except Exception as e:  # noqa: BLE001
            print(f"[ERROR] {t.__name__}: {type(e).__name__}: {e}")
            failed += 1
    print("-" * 50)
    print(f"Total: {passed + failed}  Passed: {passed}  Failed: {failed}")
    return failed == 0


if __name__ == "__main__":
    sys.exit(0 if _run_all() else 1)
