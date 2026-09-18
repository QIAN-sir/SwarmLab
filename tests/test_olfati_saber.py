"""Phase 7 —— Olfati-Saber 算法对照测试。

覆盖：rho_f/rho_der_f 分段函数、迁移加速度积分、B-8 限幅死代码、正常邻居位置匹配、
故障邻居增强排斥 + 侧向滑移、方案2 is_stuck 主动逃逸、坐标翻转、碰撞与端到端接线。
期望值按 MATLAB compute_vel_olfati_saber.m 公式独立手算。

运行方式（无需 pytest）:
    python SwarmLab_py/tests/test_olfati_saber.py
"""

import math
import os
import sys

import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from SwarmLab_py.core import Swarm                          # noqa: E402
from SwarmLab_py.enums import DroneState                    # noqa: E402
from SwarmLab_py.parameters import build_params             # noqa: E402
from SwarmLab_py.swarm.olfati_saber import rho_f, rho_der_f  # noqa: E402


def _make_swarm(positions, states=None, migration=True, max_v=7.0, max_a=10.0, vels=None):
    positions = np.asarray(positions, dtype=float)
    n = positions.shape[1]
    b = build_params("point_mass", "olfati_saber", nb_agents=n, seed=0,
                     active_environment=False)
    b.p_swarm.is_active_migration = migration
    b.p_swarm.max_v = max_v
    b.p_swarm.max_a = max_a
    s = Swarm()
    s.algorithm = "olfati_saber"
    for _ in range(n):
        s.add_drone("point_mass", "Normal", b.p_drone, b.p_battery, b.p_sim, b.p_physics, b.map)
    s.set_pos(positions)
    if states is not None:
        for i, st in enumerate(states):
            s.swarm_state[i] = st
            s.drones[i].drone_state = st
    if vels is not None:
        for i, v in enumerate(vels):
            s.drones[i].vel_xyz = np.asarray(v, dtype=float)
    return s, b


def test_rho_f():
    delta, r, k = 0.2, 150.0, 2.0
    assert rho_f(10, delta, r, k) == 1.0                 # z < delta*r(=30) → 1
    assert rho_f(200, delta, r, k) == 0.0                # z >= r → 0
    # z=90: (1/2^k)*(1+cos(pi*(z/r-delta)/(1-delta)))^k = 0.25*(1+cos(pi*0.5))^2 = 0.25
    assert math.isclose(rho_f(90, delta, r, k), 0.25, rel_tol=1e-9)


def test_rho_der_f():
    delta, r, k = 0.2, 150.0, 2.0
    assert rho_der_f(10, delta, r, k) == 0.0             # z < delta*r → 0
    assert rho_der_f(200, delta, r, k) == 0.0            # z >= r → 0
    # z=90: arg=pi*(0.6-0.2)/0.8=pi*0.5; -pi/0.8*2/4*(1+cos(arg))^1*sin(arg) = -pi/0.8*0.5*1*1
    exp = -math.pi / 0.8 * 2 / 4 * (1 + math.cos(math.pi * 0.5)) ** 1 * math.sin(math.pi * 0.5)
    assert math.isclose(rho_der_f(90, delta, r, k), exp, rel_tol=1e-9)


def test_migration_acceleration():
    # 两机相距 200 > r(150) → 无邻居；migration 开：acc_vm=c_vm*(u_ref*v_ref-vel)=[18,0,0]
    # vel_command = vel + acc*dt = [18,0,0]*0.01 = [0.18,0,0]（B-8：不限幅）
    s, b = _make_swarm([[0.0, 200.0], [0.0, 0.0], [0.0, 0.0]], migration=True)
    vel_cmd, coll = s.update_command(b.p_swarm, b.p_swarm.r_coll, b.p_sim.dt)
    assert vel_cmd.shape == (3, 2)
    assert np.allclose(vel_cmd[:, 0], [3.0 * 6.0 * 0.01, 0.0, 0.0])   # [0.18,0,0]
    assert np.allclose(vel_cmd[2, :], 0.0)                            # z 分量保持 0（坐标翻转无误）


def test_no_bounding_B8():
    # B-8：max_v 即便设得很小(0.01)，因 `if max_v is None` 恒假 → 不限幅（死代码，忠实保留）
    s, b = _make_swarm([[0.0, 200.0], [0.0, 0.0], [0.0, 0.0]], migration=True,
                       max_v=0.01, max_a=0.01)
    vel_cmd, _ = s.update_command(b.p_swarm, b.p_swarm.r_coll, b.p_sim.dt)
    assert math.isclose(vel_cmd[0, 0], 0.18, rel_tol=1e-9)   # 未被限幅到 0.01
    assert np.linalg.norm(vel_cmd[:, 0]) > 0.01              # 明确超过 max_v（限幅未生效）


def test_normal_position_matching():
    # migration 关，两 Normal 机相距 10(<delta*r=30 → rho=1, rho_der=0 → phi=psi_der)
    # agent0 被排斥（agent1 在 +x）：vel_command_x = psi_der(10)*2*dt < 0
    s, b = _make_swarm([[0.0, 10.0], [0.0, 0.0], [0.0, 0.0]], migration=False)
    p = b.p_swarm
    vel_cmd, _ = s.update_command(p, p.r_coll, b.p_sim.dt)
    # 独立重算 psi_der(10)
    zz = 10.0 - p.d_ref + p.c
    psi_der10 = (p.a + p.b) / 2 * zz / math.sqrt(1 + zz ** 2) + (p.a - p.b) / 2
    assert math.isclose(vel_cmd[0, 0], psi_der10 * 2 * b.p_sim.dt, rel_tol=1e-9)
    assert vel_cmd[0, 0] < 0                                # 排斥方向（-x）
    assert math.isclose(vel_cmd[0, 1], -psi_der10 * 2 * b.p_sim.dt, rel_tol=1e-9)  # agent1 对称


def test_fault_neighbor_stronger_and_lateral():
    # migration 关，隔离方案1：邻居 Normal vs Stalled，比较 agent0 的排斥
    pos = [[0.0, 10.0], [0.0, 0.0], [0.0, 0.0]]
    s_n, b_n = _make_swarm(pos, states=[DroneState.NORMAL, DroneState.NORMAL], migration=False)
    v_n, _ = s_n.update_command(b_n.p_swarm, b_n.p_swarm.r_coll, b_n.p_sim.dt)
    s_f, b_f = _make_swarm(pos, states=[DroneState.NORMAL, DroneState.STALLED], migration=False)
    v_f, _ = s_f.update_command(b_f.p_swarm, b_f.p_swarm.r_coll, b_f.p_sim.dt)
    # 故障邻居：k_fault=0.5, r_fault_safe=3*d_ref=75 → repulsion_mag=0.5*(75-10)=32.5（远强于正常）
    assert np.linalg.norm(v_f[:, 0]) > np.linalg.norm(v_n[:, 0])
    assert v_f[0, 0] < 0                                     # 仍为排斥(-x)
    assert abs(v_f[1, 0]) > 1e-6                             # 侧向滑移：出现 +y 分量（打破左右对称）


def test_is_stuck_escape():
    # migration 开、故障邻居近(10<2*d_ref=50)：
    #   vel=0 → vel_norm<0.3 → is_stuck → 方案2 主动逃逸（后退，克服迁移）→ vel_command_x<0
    pos = [[0.0, 10.0], [0.0, 0.0], [0.0, 0.0]]
    st = [DroneState.NORMAL, DroneState.STALLED]
    s_stuck, b1 = _make_swarm(pos, states=st, migration=True, vels=[[0.0, 0, 0], [0, 0, 0]])
    v_stuck, _ = s_stuck.update_command(b1.p_swarm, b1.p_swarm.r_coll, b1.p_sim.dt)
    assert v_stuck[0, 0] < 0                                 # 逃逸后退（-x，与迁移 +x 相反）

    # vel=[1,0,0] → vel_norm=1>=0.3 → 不 stuck → 无方案2 → 迁移主导 → vel_command_x>0
    s_free, b2 = _make_swarm(pos, states=st, migration=True, vels=[[1.0, 0, 0], [0, 0, 0]])
    v_free, _ = s_free.update_command(b2.p_swarm, b2.p_swarm.r_coll, b2.p_sim.dt)
    assert v_free[0, 0] > 0
    # 逃逸显著改变指令
    assert abs(v_stuck[0, 0] - v_free[0, 0]) > 0.1


def test_collisions_and_end_to_end():
    # 两机相距 0.5 < 2*r_agent(=1) → 1 次碰撞
    s, b = _make_swarm([[0.0, 0.5], [0.0, 0.0], [0.0, 0.0]], migration=False)
    vel_cmd, coll = s.update_command(b.p_swarm, b.p_swarm.r_coll, b.p_sim.dt)
    assert math.isclose(coll[0], 1.0)
    assert coll[2] == 20.0                                   # 无障碍 → min_dist_obs 初值
    # 端到端写入 command + collisions_history
    assert np.allclose(s.drones[0].command[1:4], vel_cmd[:, 0])
    assert s.collisions_history.shape == (1, 3)


def _run_all():
    tests = [
        test_rho_f, test_rho_der_f, test_migration_acceleration, test_no_bounding_B8,
        test_normal_position_matching, test_fault_neighbor_stronger_and_lateral,
        test_is_stuck_escape, test_collisions_and_end_to_end,
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
