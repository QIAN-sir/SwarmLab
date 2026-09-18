"""Phase 6 —— Vasarhelyi 算法对照测试。

覆盖：get_v_max 制动曲线、邻居排斥/吸引、故障邻居增强排斥、故障机自身指令、
迁移项、碰撞统计、速度/加速度限幅，以及经 Swarm.update_command 的端到端接线。
期望值按 MATLAB compute_vel_vasarhelyi.m 公式独立手算（p_rep=0.03, r0_rep=d_ref=25）。

运行方式（无需 pytest）:
    python SwarmLab_py/tests/test_vasarhelyi.py
"""

import math
import os
import sys

import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from SwarmLab_py.core import Swarm                         # noqa: E402
from SwarmLab_py.enums import DroneState                   # noqa: E402
from SwarmLab_py.parameters import build_params            # noqa: E402
from SwarmLab_py.swarm.vasarhelyi import get_v_max         # noqa: E402


def _make_swarm(positions, states=None, migration=True, max_v=None, max_a=None):
    """构建 vasarhelyi 群体；positions (3,N)；states 为各机 DroneState（默认全 Normal）。"""
    positions = np.asarray(positions, dtype=float)
    n = positions.shape[1]
    b = build_params("point_mass", "vasarhelyi", nb_agents=n, seed=0,
                     active_environment=False)
    b.p_swarm.is_active_migration = migration
    b.p_swarm.max_v = max_v
    b.p_swarm.max_a = max_a
    s = Swarm()
    s.algorithm = "vasarhelyi"
    for _ in range(n):
        s.add_drone("point_mass", "Normal", b.p_drone, b.p_battery, b.p_sim, b.p_physics, b.map)
    s.set_pos(positions)
    if states is not None:
        for i, st in enumerate(states):
            s.swarm_state[i] = st
            s.drones[i].drone_state = st
    return s, b


def test_get_v_max():
    # r<0 → v_fricmax=0，再抬到 v_fric
    assert math.isclose(get_v_max(0.63, -1.0, 4.16, 3.2), 0.63)
    assert math.isclose(get_v_max(0.0, -1.0, 4.16, 3.2), 0.0)
    # 0 < r*p < a/p → v_fricmax = r*p（=0.32），但 < v_fric(0.63) → 抬到 0.63
    assert math.isclose(get_v_max(0.63, 0.1, 4.16, 3.2), 0.63)
    # r*p=3.2 不 < a/p=1.3 → else 分支 sqrt(2*a*r - a^2/p^2)=sqrt(8.32-1.69)
    exp = math.sqrt(2 * 4.16 * 1.0 - 4.16 ** 2 / 3.2 ** 2)
    assert math.isclose(get_v_max(0.63, 1.0, 4.16, 3.2), exp, rel_tol=1e-9)


def test_migration_term():
    # 两机相距 200 > r(150) → 无邻居交互；migration 开 → vel_command = v_ref*u_ref = [6,0,0]
    s, b = _make_swarm([[0.0, 200.0], [0.0, 0.0], [0.0, 0.0]], migration=True)
    vel_cmd, coll = s.update_command(b.p_swarm, b.p_swarm.r_coll, b.p_sim.dt)
    assert vel_cmd.shape == (3, 2)
    assert np.allclose(vel_cmd[:, 0], [6.0, 0.0, 0.0])
    assert np.allclose(vel_cmd[:, 1], [6.0, 0.0, 0.0])


def test_normal_repulsion():
    # 两 Normal 机相距 10 < r0_rep(25) → 排斥；migration 关、vel=0 → 无自推进
    # vel_rep = p_rep*(r0_rep-dist)*(-单位向量) = 0.03*(25-10)*[-1,0,0] = [-0.45,0,0]
    s, b = _make_swarm([[0.0, 10.0], [0.0, 0.0], [0.0, 0.0]], migration=False)
    vel_cmd, _ = s.update_command(b.p_swarm, b.p_swarm.r_coll, b.p_sim.dt)
    assert np.allclose(vel_cmd[:, 0], [-0.45, 0.0, 0.0])   # agent0 被推离 agent1(-x)
    assert np.allclose(vel_cmd[:, 1], [0.45, 0.0, 0.0])    # agent1 对称(+x)


def test_fault_neighbor_enhanced_repulsion():
    # 同样相距 10，比较邻居为 Normal vs Stalled 时 agent0 受到的排斥
    pos = [[0.0, 10.0], [0.0, 0.0], [0.0, 0.0]]
    s_n, b_n = _make_swarm(pos, states=[DroneState.NORMAL, DroneState.NORMAL], migration=False)
    v_n, _ = s_n.update_command(b_n.p_swarm, b_n.p_swarm.r_coll, b_n.p_sim.dt)
    s_f, b_f = _make_swarm(pos, states=[DroneState.NORMAL, DroneState.STALLED], migration=False)
    v_f, _ = s_f.update_command(b_f.p_swarm, b_f.p_swarm.r_coll, b_f.p_sim.dt)
    # 故障邻居：p_fault=3*p_rep, r_safe=3*r0_rep=75 → 0.09*(75-10)=[-5.85,0,0]
    assert np.allclose(v_f[:, 0], [-5.85, 0.0, 0.0])
    assert abs(v_f[0, 0]) > abs(v_n[0, 0])                 # 故障邻居排斥显著更强


def test_faulted_drone_command():
    # migration 关、vel=0：Stalled 机自身指令 = [0,0,0]
    s, b = _make_swarm([[0.0, 10.0], [0.0, 0.0], [0.0, 0.0]],
                       states=[DroneState.NORMAL, DroneState.STALLED], migration=False)
    vel_cmd, _ = s.update_command(b.p_swarm, b.p_swarm.r_coll, b.p_sim.dt)
    assert np.allclose(vel_cmd[:, 1], [0.0, 0.0, 0.0])
    # migration 开：故障机指令 = [0,0,0] + v_ref*u_ref（忠实复刻 MATLAB 顺序）
    s2, b2 = _make_swarm([[0.0, 200.0], [0.0, 0.0], [0.0, 0.0]],
                         states=[DroneState.NORMAL, DroneState.STALLED], migration=True)
    vel_cmd2, _ = s2.update_command(b2.p_swarm, b2.p_swarm.r_coll, b2.p_sim.dt)
    assert np.allclose(vel_cmd2[:, 1], [6.0, 0.0, 0.0])


def test_collision_count():
    # 两机相距 0.5 < 2*r_agent(=1) → 记 1 次碰撞（互相各计一次后 /2）
    s, b = _make_swarm([[0.0, 0.5], [0.0, 0.0], [0.0, 0.0]], migration=False)
    _, coll = s.update_command(b.p_swarm, b.p_swarm.r_coll, b.p_sim.dt)
    assert math.isclose(coll[0], 1.0)                       # nb_agent_collisions
    assert coll[2] == 20.0                                  # min_dist_obs 初值（无障碍）


def test_velocity_bounding():
    # migration 给出 [6,0,0]，max_v=3 → 限幅到模长 3
    s, b = _make_swarm([[0.0, 200.0], [0.0, 0.0], [0.0, 0.0]], migration=True,
                       max_v=3.0, max_a=None)
    vel_cmd, _ = s.update_command(b.p_swarm, b.p_swarm.r_coll, b.p_sim.dt)
    assert np.allclose(vel_cmd[:, 0], [3.0, 0.0, 0.0])
    assert np.all(np.sqrt(np.sum(vel_cmd ** 2, axis=0)) <= 3.0 + 1e-9)


def test_accel_bounding():
    # migration [6,0,0]，max_a=10, dt=0.01：accel=(6-0)/0.01=600>10 → 限幅
    # vel_command = vel + dt*max_a*accel_cmd/|accel| = 0 + 0.01*10*[1,0,0] = [0.1,0,0]
    s, b = _make_swarm([[0.0, 200.0], [0.0, 0.0], [0.0, 0.0]], migration=True,
                       max_v=None, max_a=10.0)
    vel_cmd, _ = s.update_command(b.p_swarm, b.p_swarm.r_coll, b.p_sim.dt)
    assert np.allclose(vel_cmd[:, 0], [0.1, 0.0, 0.0])


def test_update_command_writes_and_history():
    s, b = _make_swarm([[0.0, 200.0], [0.0, 0.0], [0.0, 0.0]], migration=True)
    vel_cmd, coll = s.update_command(b.p_swarm, b.p_swarm.r_coll, b.p_sim.dt)
    # set_vel_commands：command(1)=0, command(2:4)=vel
    assert s.drones[0].command[0] == 0.0
    assert np.allclose(s.drones[0].command[1:4], vel_cmd[:, 0])
    # collisions_history 累加
    assert s.collisions_history.shape == (1, 3)
    s.update_command(b.p_swarm, b.p_swarm.r_coll, b.p_sim.dt)
    assert s.collisions_history.shape == (2, 3)


def _run_all():
    tests = [
        test_get_v_max, test_migration_term, test_normal_repulsion,
        test_fault_neighbor_enhanced_repulsion, test_faulted_drone_command,
        test_collision_count, test_velocity_bounding, test_accel_bounding,
        test_update_command_writes_and_history,
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
