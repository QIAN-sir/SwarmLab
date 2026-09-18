"""headless 示例：point_mass 故障群仿真（对应 MATLAB example_olfati_saber.m / example_vasarhelyi.m）。

脱离 GUI 与可视化，纯命令行运行，演示"故障注入 + 群体避让"主线：
    - 10 架 point_mass，Olfati-Saber 算法，启用迁移
    - t=20s 注入 30% Stalled（动力锁死）
    - t=40s 注入 30% Drifting（失控漂移，fad 随时间衰减）
    - t=70s 恢复 Normal

运行:
    python SwarmLab_py/examples/run_swarm_fault.py [olfati_saber|vasarhelyi]
"""

import os
import sys

import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from SwarmLab_py.simulation import Simulation, FaultEvent   # noqa: E402


def main():
    algorithm = sys.argv[1] if len(sys.argv) > 1 else "olfati_saber"

    sim = Simulation(
        drone_type="point_mass",
        algorithm=algorithm,
        nb_agents=10,
        active_environment=True,     # 圆柱障碍（由 map 建筑生成）
        end_time=80.0,
        seed=0,                      # 固定种子 → Pos0 可复现
        fault_schedule=[
            FaultEvent(time=20.0, state="Stalled", loss_radio=0.3),
            FaultEvent(time=40.0, state="Drifting", loss_radio=0.3),
            FaultEvent(time=70.0, state="Normal"),
        ],
    )

    print(f"=== SwarmLab_py headless 仿真：{algorithm} ===")
    print(f"nb_agents={sim.p_swarm.nb_agents}  dt={sim.dt}  end_time={sim.end_time}")
    print(f"n_cyl={sim.p_swarm.n_cyl}  migration={sim.p_swarm.is_active_migration}  "
          f"loss_radio={sim.loss_radio}")
    print("故障时间表:", [(e.time, e.state) for e in sim.fault_schedule])

    sim.run()

    # ---- 结果 ----
    pos = sim.get_pos_history()      # (T, 3N)
    vel = sim.get_vel_history()
    n_steps = len(sim.time_history)
    print(f"\n步数={n_steps}  历史形状 pos={pos.shape} vel={vel.shape}")
    print(f"累计碰撞 [机-机, 机-障, min_dist累计]={np.round(sim.collisions_sum, 3)}")

    # 各阶段 fad 抽样
    def fad_at(t):
        i = min(range(n_steps), key=lambda j: abs(sim.time_history[j] - t))
        return sim.time_history[i], sim.fad_history[i], sim.state_history[i]

    for t in (10.0, 30.0, 45.0, 60.0, 75.0):
        tt, fad, st = fad_at(t)
        print(f"  t≈{tt:6.2f}s  state={st:9s}  fad={fad:.4f}")

    # 故障期健康机数量
    n_normal = sum(1 for d in sim.swarm.drones if d.drone_state == "Normal")
    print(f"\n结束时健康机数={n_normal}/{sim.p_swarm.nb_agents}（t=70s 已恢复 Normal）")
    print("仿真结束。")


if __name__ == "__main__":
    main()
