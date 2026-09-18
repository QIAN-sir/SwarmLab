"""Stage 1 —— 仿真主循环 runner 端到端测试。

验证 Simulation 引擎忠实复刻 example_*.m 主循环：故障注入(set_dronestate)、
fad=1/(1+0.2*(t-t_fault)) 衰减、Normal/Stalled/Drifting 运动学、历史/碰撞记录、
step 逐步模式、可复现性、两算法均可跑通。

运行方式（无需 pytest）:
    python SwarmLab_py/tests/test_simulation.py
"""

import math
import os
import sys

import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from SwarmLab_py.simulation import Simulation, FaultEvent   # noqa: E402


def _fad_at(sim, t):
    th = np.array(sim.time_history)
    i = int(np.argmin(np.abs(th - t)))
    return sim.fad_history[i], sim.state_history[i], sim.time_history[i]


def _sim(**kw):
    kw.setdefault("drone_type", "point_mass")
    kw.setdefault("algorithm", "olfati_saber")
    kw.setdefault("nb_agents", 3)
    kw.setdefault("active_environment", False)
    kw.setdefault("seed", 0)
    kw.setdefault("end_time", 3.0)
    return Simulation(**kw)


def test_normal_migration_moves():
    sim = _sim()                       # 无故障、migration 默认开
    p0 = sim.swarm.get_pos_ned().copy()
    sim.run()
    p1 = sim.swarm.get_pos_ned()
    # 迁移方向 u_ref=[1,0,0] → 群体应整体向 +x 移动
    assert np.all(p1[0, :] > p0[0, :])
    assert all(f == 1.0 for f in sim.fad_history)          # 全程 Normal → fad=1
    assert sim.done is True
    assert len(sim.time_history) == 301                    # t=0..3.0, dt=0.01


def test_stalled_freezes():
    sim = _sim(fault_schedule=[FaultEvent(1.0, "Stalled", 1.0)])   # 全部 Stalled
    sim.run()
    for d in sim.swarm.drones:
        assert np.allclose(d.vel_xyz, np.zeros(3))         # 速度归零
        assert d.drone_state == "Stalled"
    # 位置冻结：历史最后两帧相同
    pos = sim.get_pos_history()
    assert np.allclose(pos[-1, :], pos[-2, :])


def test_drifting_fad_formula():
    sim = _sim(fault_schedule=[FaultEvent(1.0, "Drifting", 1.0)])
    sim.run()
    # 注入前 Normal → fad=1
    f0, s0, _ = _fad_at(sim, 0.5)
    assert s0 == "Normal" and math.isclose(f0, 1.0)
    # t=2.0: 1/(1+0.2*(2-1))=1/1.2
    f2, s2, _ = _fad_at(sim, 2.0)
    assert s2 == "Drifting" and math.isclose(f2, 1 / 1.2, rel_tol=1e-9)
    # t=3.0: 1/(1+0.2*(3-1))=1/1.4
    f3, s3, _ = _fad_at(sim, 3.0)
    assert math.isclose(f3, 1 / 1.4, rel_tol=1e-9)


def test_drifting_velocity_decay():
    sim = _sim(fault_schedule=[FaultEvent(1.0, "Drifting", 1.0)])
    sim.run()
    # Drifting: vel_xyz = fad * drift_vel（drift_vel 首次进入时记录，之后固定）
    for d in sim.swarm.drones:
        assert d.drone_state == "Drifting"
        assert np.linalg.norm(d.drift_vel) > 0             # 故障前在运动
        assert np.allclose(d.vel_xyz, sim.swarm.fad * d.drift_vel)
    # fad 递减 → 速度较故障初期衰减
    assert sim.fad_history[-1] < sim.fad_history[100]      # t=3.0 < t=1.0


def test_fad_zero_when_small():
    # 长时间 Drifting：t-t_fault>45 → fad<0.1 → 归零
    sim = Simulation(algorithm="olfati_saber", nb_agents=2, active_environment=False,
                     seed=0, end_time=60.0,
                     fault_schedule=[FaultEvent(1.0, "Drifting", 1.0)])
    sim.run()
    f, s, t = _fad_at(sim, 59.0)
    assert s == "Drifting"
    assert f == 0.0                                        # 1/(1+0.2*58)=0.079<0.1 → 0
    for d in sim.swarm.drones:
        assert np.allclose(d.vel_xyz, np.zeros(3))         # fad=0 → 停止


def test_state_history():
    sim = _sim(end_time=3.0, fault_schedule=[
        FaultEvent(1.0, "Stalled", 1.0),
        FaultEvent(2.0, "Normal"),
    ])
    sim.run()
    assert _fad_at(sim, 0.5)[1] == "Normal"
    assert _fad_at(sim, 1.5)[1] == "Stalled"
    assert _fad_at(sim, 2.5)[1] == "Normal"


def test_step_mode():
    sim = _sim(end_time=0.05)            # 很短
    assert sim.time == 0.0
    sim.step()
    assert math.isclose(sim.time, 0.01)  # 推进一个 dt
    assert len(sim.time_history) == 1
    # 逐步跑到结束
    while sim.step():
        pass
    assert sim.done is True


def test_reproducibility():
    a = _sim(); a.run()
    b = _sim(); b.run()
    assert np.allclose(a.swarm.get_pos_ned(), b.swarm.get_pos_ned())   # 同 seed → 同轨迹
    assert np.allclose(a.get_pos_history(), b.get_pos_history())


def test_both_algorithms_run():
    for algo in ("olfati_saber", "vasarhelyi"):
        sim = _sim(algorithm=algo, end_time=1.0,
                   fault_schedule=[FaultEvent(0.5, "Drifting", 1.0)])
        sim.run()
        assert sim.done and sim.collisions_sum.shape == (3,)
        assert sim.get_pos_history().shape[1] == 3 * sim.p_swarm.nb_agents


def test_collisions_and_history_shapes():
    sim = _sim(nb_agents=4, end_time=1.0)
    sim.run()
    pos = sim.get_pos_history()
    vel = sim.get_vel_history()
    assert pos.shape[1] == 12 and vel.shape[1] == 12       # 3N
    assert pos.shape[0] == vel.shape[0] + 1                # set_pos 多一帧
    assert sim.collisions_sum.shape == (3,)


def _run_all():
    tests = [
        test_normal_migration_moves, test_stalled_freezes, test_drifting_fad_formula,
        test_drifting_velocity_decay, test_fad_zero_when_small, test_state_history,
        test_step_mode, test_reproducibility, test_both_algorithms_run,
        test_collisions_and_history_shapes,
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
