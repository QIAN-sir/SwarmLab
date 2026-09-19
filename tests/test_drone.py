"""Phase 3 —— Drone 类对照测试（重点：point_mass 故障状态机）。

验证 Drone 的构造、状态/历史访问器，以及 update_state 的 point_mass 三态
（Normal / Stalled / Drifting + fad + drift_vel）与 MATLAB @Drone/Drone.m 一致。
期望值均按 MATLAB 逻辑独立手算（dt=0.01）。

运行方式（无需 pytest）:
    python SwarmLab_py/tests/test_drone.py
"""

import math
import os
import sys

import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from SwarmLab_py.core import Drone                       # noqa: E402
from SwarmLab_py.parameters import build_params           # noqa: E402


def _make_point_mass(state="Normal"):
    b = build_params("point_mass", None, active_environment=True)
    return Drone("point_mass", state, b.p_drone, b.p_battery, b.p_sim, b.p_physics, b.map)


def test_construction():
    d = _make_point_mass("Normal")
    assert d.autopilot_version == 2                 # point_mass → 2
    assert np.allclose(d.pos_ned, np.zeros(3))
    assert np.allclose(d.drift_vel, np.zeros(3))
    assert d.command.shape == (4,) and d.z_hat.shape == (22,)
    assert d.color.shape == (3,)                    # jet(16) 随机一行
    assert d.pos_ned_history.shape == (0, 3)
    assert d.use_estimation is False and d.drift_initialized is False
    assert d.Q == d.p_battery.Q0 and d.V == d.p_battery.V0


def test_set_pos_vel_get_state():
    d = _make_point_mass()
    d.set_pos(np.array([10.0, 20.0, -30.0]))
    assert d.pos_ned_history.shape == (1, 3)
    # z_hat = true_states([state; airdata; NaN]) → z_hat[0:3] = [pn, pe, -pd]
    assert np.allclose(d.z_hat[0:3], [10.0, 20.0, 30.0])
    d.set_vel(np.array([1.0, 2.0, 3.0]))
    assert d.vel_xyz_history.shape == (1, 3)
    st = d.get_state()
    assert st.shape == (12,)
    assert np.allclose(st[0:3], [10.0, 20.0, -30.0])
    assert np.allclose(st[3:6], [1.0, 2.0, 3.0])


def test_normal_update():
    d = _make_point_mass("Normal")
    d.set_pos(np.zeros(3))
    d.command = np.array([0.5, 2.0, 0.0, 0.0])   # psi=0.5, vel=[2,0,0]
    dt = d.p_sim.dt                               # 0.01
    d.update_state(np.zeros(6), 0.0, 1.0)
    assert np.allclose(d.vel_xyz, [2.0, 0.0, 0.0])
    assert np.allclose(d.pos_ned, [2.0 * dt, 0.0, 0.0])
    assert math.isclose(d.attitude[2], 0.5)       # psi = command(1)
    d.update_state(np.zeros(6), dt, 1.0)
    assert np.allclose(d.pos_ned, [2.0 * dt * 2, 0.0, 0.0])
    assert d.vel_xyz_history.shape[0] == 2        # 两次 update 各追加一行


def test_stalled_update():
    d = _make_point_mass("Normal")
    d.set_pos(np.array([5.0, 0.0, 0.0]))
    d.command = np.array([0.0, 2.0, 0.0, 0.0])
    d.drone_state = "Stalled"                     # 动力锁死
    pos_before = d.pos_ned.copy()
    d.update_state(np.zeros(6), 0.0, 1.0)
    assert np.allclose(d.vel_xyz, np.zeros(3))    # 速度归零
    assert np.allclose(d.pos_ned, pos_before)     # 位置不变


def test_drifting_update_and_decay():
    d = _make_point_mass("Normal")
    d.set_pos(np.zeros(3))
    d.command = np.array([0.5, 2.0, 0.0, 0.0])    # vel=[2,0,0]
    dt = d.p_sim.dt
    # 先跑两步 Normal，使 vel_xyz_history 记录到故障前速度 [2,0,0]
    d.update_state(np.zeros(6), 0.0, 1.0)         # pos=[0.02,0,0]
    d.update_state(np.zeros(6), dt, 1.0)          # pos=[0.04,0,0]
    assert np.allclose(d.vel_xyz_history[-1], [2.0, 0.0, 0.0])

    # 切入 Drifting：首次记录 drift_vel = 故障前速度，vel = fad*drift_vel
    d.drone_state = "Drifting"
    d.update_state(np.zeros(6), 2 * dt, 0.5)      # fad=0.5
    assert d.drift_initialized is True
    assert np.allclose(d.drift_vel, [2.0, 0.0, 0.0])
    assert np.allclose(d.vel_xyz, [1.0, 0.0, 0.0])            # 0.5*[2,0,0]
    assert np.allclose(d.pos_ned, [0.04 + 1.0 * dt, 0.0, 0.0])  # [0.05,0,0]

    # 再一步：drift_vel 保持（不重复记录），vel 随 fad 继续衰减
    d.update_state(np.zeros(6), 3 * dt, 0.25)     # fad=0.25
    assert np.allclose(d.drift_vel, [2.0, 0.0, 0.0])
    assert np.allclose(d.vel_xyz, [0.5, 0.0, 0.0])            # 0.25*[2,0,0]
    assert np.allclose(d.pos_ned, [0.05 + 0.5 * dt, 0.0, 0.0])  # [0.055,0,0]

    # fad→0 时应趋停
    d.update_state(np.zeros(6), 4 * dt, 0.0)
    assert np.allclose(d.vel_xyz, np.zeros(3))


def test_drift_initialized_reset_on_normal():
    d = _make_point_mass("Normal")
    d.set_pos(np.zeros(3))
    d.command = np.array([0.0, 1.0, 0.0, 0.0])
    d.update_state(np.zeros(6), 0.0, 1.0)
    d.drone_state = "Drifting"
    d.update_state(np.zeros(6), 0.01, 0.5)
    assert d.drift_initialized is True
    d.drone_state = "Normal"
    d.update_state(np.zeros(6), 0.02, 1.0)        # 非 Drifting → 复位 drift_initialized
    assert d.drift_initialized is False


def test_real_dynamics_quad_active():
    # Phase 8/9 后：quadcopter 真实动力学已接通（不再抛 NotImplementedError）
    b = build_params("quadcopter", None)
    d = Drone("quadcopter", "Normal", b.p_drone, b.p_battery, b.p_sim, b.p_physics, b.map)
    d.set_pos(np.array([0.0, 0.0, -20.0]))
    d.command = np.zeros(4)
    d.update_state(np.zeros(6), 0.0, 1.0)   # 应正常运行
    assert np.all(np.isfinite(d.pos_ned))
    # fixed_wing 自动驾驶仪待迁移 → 仍抛 NotImplementedError
    bw = build_params("fixed_wing", None)
    dw = Drone("fixed_wing", "Normal", bw.p_drone, bw.p_battery, bw.p_sim, bw.p_physics, bw.map)
    dw.set_pos(np.array([0.0, 0.0, -50.0]))
    dw.command = np.zeros(4)
    try:
        dw.update_state(np.zeros(6), 0.0, 1.0)
        raised = False
    except NotImplementedError:
        raised = True
    assert raised, "fixed_wing 自动驾驶仪待迁移，应抛 NotImplementedError"


def _run_all():
    tests = [
        test_construction, test_set_pos_vel_get_state, test_normal_update,
        test_stalled_update, test_drifting_update_and_decay,
        test_drift_initialized_reset_on_normal, test_real_dynamics_quad_active,
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
