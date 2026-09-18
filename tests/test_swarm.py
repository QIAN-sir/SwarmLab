"""Phase 4 —— Swarm 类对照测试（重点：故障注入 + 状态传播 + 健康质心）。

验证 Swarm 的构造、add_drone、set_dronestate（按 loss_radio 随机注入）、
update_state(fad) 端到端驱动 point_mass、get_barycenter（仅健康机）、各 getter，
以及对 B-1/B-2/B-5 等已知 MATLAB bug 的“忠实行为”锁定。

运行方式（无需 pytest）:
    python SwarmLab_py/tests/test_swarm.py
"""

import math
import os
import sys

import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from SwarmLab_py.core import Swarm, Drone                 # noqa: E402
from SwarmLab_py.enums import DroneState                  # noqa: E402
from SwarmLab_py.parameters import build_params           # noqa: E402


def _make_swarm(n=5, algorithm=None):
    b = build_params("point_mass", None, active_environment=True)
    s = Swarm()
    if algorithm is not None:
        s.algorithm = algorithm
    for _ in range(n):
        s.add_drone("point_mass", "Normal", b.p_drone, b.p_battery, b.p_sim, b.p_physics, b.map)
    return s, b


def test_construction():
    s = Swarm()
    assert s.nb_agents == 0 and s.drones == [] and s.swarm_state == []
    assert s.algorithm is None and s.equivalent_drone is None
    assert s.normal_drone == 0 and s.fad == 0.0 and s.t_fault == 0.0
    assert s.collisions_history.shape == (0, 3)


def test_add_drone():
    s, _ = _make_swarm(5)
    assert s.nb_agents == 5 and len(s.drones) == 5
    assert len(s.swarm_state) == 5
    assert all(st == DroneState.NORMAL for st in s.swarm_state)   # 初始全 Normal
    assert all(isinstance(d, Drone) for d in s.drones)


def test_set_pos_get_pos_ned():
    s, _ = _make_swarm(3)
    pos = np.array([[0.0, 1.0, 2.0],
                    [0.0, 0.0, 0.0],
                    [-10.0, -20.0, -30.0]])   # 3×3
    s.set_pos(pos)
    assert np.allclose(s.get_pos_ned(), pos)
    # set_pos 也写入各 drone 的 pos_ned_history
    assert s.drones[0].pos_ned_history.shape == (1, 3)


def test_get_vel_ned_zero_attitude():
    s, _ = _make_swarm(2)
    # point_mass 姿态为 0 时，Rb2i(0,0,0)=I → vel_ned == vel_xyz
    s.drones[0].vel_xyz = np.array([1.0, 2.0, 3.0])
    s.drones[1].vel_xyz = np.array([0.0, 0.0, 0.0])
    vn = s.get_vel_ned()
    assert vn.shape == (3, 2)
    assert np.allclose(vn[:, 0], [1.0, 2.0, 3.0])


def test_set_dronestate_stalled_count():
    s, _ = _make_swarm(10)
    s.set_dronestate("Stalled", 1, 0.2)          # ceil(0.2*10)=2 架 Stalled
    n_stalled = sum(1 for st in s.swarm_state if st == DroneState.STALLED)
    assert n_stalled == 2
    # 同步到各 Drone
    n_drone_stalled = sum(1 for d in s.drones if d.drone_state == "Stalled")
    assert n_drone_stalled == 2


def test_set_dronestate_drifting_count():
    s, _ = _make_swarm(10)
    s.set_dronestate("Drifting", 2, 0.3)         # ceil(0.3*10)=3 架 Drifting
    n_drifting = sum(1 for st in s.swarm_state if st == DroneState.DRIFTING)
    assert n_drifting == 3
    assert sum(1 for d in s.drones if d.drone_state == "Drifting") == 3


def test_set_dronestate_normal_resets():
    s, _ = _make_swarm(8)
    s.set_dronestate("Stalled", 1, 0.5)          # 先注入 4 架
    s.set_dronestate("Normal", 1, 0.5)           # 再全部恢复
    assert all(st == DroneState.NORMAL for st in s.swarm_state)
    assert all(d.drone_state == "Normal" for d in s.drones)


def test_update_state_propagation():
    s, _ = _make_swarm(3)
    s.set_pos(np.zeros((3, 3)))
    # 所有无人机速度指令 [1,0,0]
    s.set_vel_commands(np.tile(np.array([[1.0], [0.0], [0.0]]), (1, 3)))
    dt = s.drones[0].p_sim.dt
    s.update_state(np.zeros(6), 0.0, 1.0)        # Normal → pos += [1,0,0]*dt
    for i in range(3):
        assert np.allclose(s.drones[i].pos_ned, [1.0 * dt, 0.0, 0.0])
        assert s.drones[i].command[0] == 0.0     # set_vel_commands 置 command(1)=0


def test_update_state_respects_fault():
    s, _ = _make_swarm(2)
    s.set_pos(np.zeros((3, 2)))
    s.set_vel_commands(np.tile(np.array([[2.0], [0.0], [0.0]]), (1, 2)))
    # 手动把 drone1 设为 Stalled（并同步 swarm_state）
    s.swarm_state[1] = DroneState.STALLED
    s.drones[1].drone_state = DroneState.STALLED
    s.update_state(np.zeros(6), 0.0, 1.0)
    assert np.allclose(s.drones[0].pos_ned, [2.0 * 0.01, 0, 0])   # Normal 前进
    assert np.allclose(s.drones[1].pos_ned, [0, 0, 0])             # Stalled 不动


def test_get_barycenter_healthy_only():
    s, _ = _make_swarm(2)
    s.set_pos(np.array([[0.0, 10.0], [0.0, 0.0], [0.0, 0.0]]))   # d0@[0,0,0], d1@[10,0,0]
    s.swarm_state = [DroneState.NORMAL, DroneState.STALLED]       # 仅 d0 健康
    eq = s.get_barycenter()
    assert np.allclose(eq.pos_ned, [0.0, 0.0, 0.0])              # 只平均健康机 d0
    assert s.normal_drone == 1


def test_get_barycenter_normal_drone_accumulates_B5():
    """B-5：normal_drone 不清零，多次调用累加（忠实保留 MATLAB 行为）。"""
    s, _ = _make_swarm(2)
    s.set_pos(np.array([[0.0, 2.0], [0.0, 0.0], [0.0, 0.0]]))
    s.get_barycenter()
    assert s.normal_drone == 2          # 第 1 次：2 架健康 → 0+2
    eq2 = s.get_barycenter()
    assert s.normal_drone == 4          # 第 2 次：再 +2（未清零）→ 质心被错误地除以 4
    assert np.allclose(eq2.pos_ned, [(0.0 + 2.0) / 4, 0, 0])


def test_getters_shapes():
    s, _ = _make_swarm(4)
    s.set_pos(np.zeros((3, 4)))
    s.set_vel_commands(np.zeros((3, 4)))
    s.update_state(np.zeros(6), 0.0, 1.0)
    assert s.get_colors().shape == (3, 4)
    assert s.get_Qt().shape == (4,) and s.get_Q().shape == (4,)
    assert s.get_path_len().shape == (4,)
    assert s.get_pos_ned_history().shape[1] == 12    # 3N
    assert s.get_vel_xyz_history().shape[1] == 12


def test_update_command_dispatch():
    # Phase 6/7 后：update_command 按 algorithm 分派到对应算法，产出 (3,N) 指令 + collisions(3,)
    for algo in ("olfati_saber", "vasarhelyi"):
        b = build_params("point_mass", algo, nb_agents=3, seed=0, active_environment=False)
        s = Swarm()
        s.algorithm = algo
        for _ in range(3):
            s.add_drone("point_mass", "Normal", b.p_drone, b.p_battery, b.p_sim, b.p_physics, b.map)
        s.set_pos(np.array([[0.0, 30.0, 60.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]))
        vel_cmd, coll = s.update_command(b.p_swarm, b.p_swarm.r_coll, b.p_sim.dt)
        assert vel_cmd.shape == (3, 3)
        assert coll.shape == (3,)


def test_bug_B2_set_vel():
    """B-2：set_vel 使用未定义的 self.nb_drones → AttributeError（忠实保留）。"""
    s, _ = _make_swarm(2)
    try:
        s.set_vel(np.zeros((3, 2)))
        raised = False
    except AttributeError:
        raised = True
    assert raised


def test_bug_B1_add_n_drones():
    """B-1：add_n_drones 以 1 参调用 add_drone（需 7 参）→ TypeError（忠实保留）。"""
    s, _ = _make_swarm(1)
    try:
        s.add_n_drones(s.drones[0], 3)
        raised = False
    except TypeError:
        raised = True
    assert raised


def _run_all():
    tests = [
        test_construction, test_add_drone, test_set_pos_get_pos_ned,
        test_get_vel_ned_zero_attitude, test_set_dronestate_stalled_count,
        test_set_dronestate_drifting_count, test_set_dronestate_normal_resets,
        test_update_state_propagation, test_update_state_respects_fault,
        test_get_barycenter_healthy_only, test_get_barycenter_normal_drone_accumulates_B5,
        test_getters_shapes, test_update_command_dispatch,
        test_bug_B2_set_vel, test_bug_B1_add_n_drones,
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
