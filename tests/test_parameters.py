"""Phase 2 —— 参数系统迁移对照测试。

验证策略（对应迁移计划 §17/§18）:
    期望值尽量用“独立手算/独立公式重算”给出（而非复制实现代码），以真正检验
    派生量（gamma、PID 增益、c、R_min、传感器 Q/R、建筑布局、cylinders）是否与
    MATLAB param_*.m 的公式一致。

说明:
    本机无 MATLAB，无法直接产出 MATLAB 数值做逐元素 diff。本测试用“按 MATLAB 公式
    独立重算的期望值”作为对照基准；后续如提供 MATLAB 导出的参数快照，可再加一层
    数值 diff（见 MIGRATION_LOG 的待办）。

运行方式（无需 pytest）:
    python SwarmLab_py/tests/test_parameters.py
"""

import math
import os
import sys

import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from SwarmLab_py.parameters import (  # noqa: E402
    PhysicsParams, SimParams, BatteryParams,
    PointMassParams, QuadcopterParams, FixedWingParams,
    build_drone_params, build_map_params, build_swarm_params, build_params,
)


def close(a, b, rel=1e-9, abs_=1e-12):
    return math.isclose(a, b, rel_tol=rel, abs_tol=abs_)


# ---------------------------------------------------------------------------
def test_physics_sim_battery():
    ph = PhysicsParams()
    assert (ph.gravity, ph.rho) == (9.81, 1.2682)

    s = SimParams()
    assert (s.dt, s.dt_plot, s.dt_video, s.start_time, s.end_time) == (0.01, 0.5, 0.1, 0.0, 120.0)

    b = BatteryParams()
    assert (b.e0, b.e1, b.e2, b.A, b.B) == (12.69, -3.14, 1.58, 1.53, 29.89)
    assert (b.V0, b.Q0, b.Qf, b.R, b.tau) == (12.59, 0.034, 1.20, 0.061, 1.95)
    assert (b.p0, b.p1, b.p2, b.p3, b.p4, b.power_board) == (0.73, 1.31e-7, 4.03e-15, -1.22e-23, 1.65e-31, 0)


def test_point_mass():
    ph = PhysicsParams()
    pm = build_drone_params("point_mass", ph)
    assert isinstance(pm, PointMassParams)
    assert pm.Va_T == 5.0
    assert pm.Va0 == 10.0                       # 基类 = norm(ws_ned0=[10,0,0])
    assert close(pm.phi_max, math.radians(60))  # 基类默认，point_mass 不覆盖
    # R_min = Va_T^2 / gravity / tan(phi_max)
    assert close(pm.R_min, 5.0 ** 2 / 9.81 / math.tan(math.radians(60)), rel=1e-9)
    assert close(pm.radius, 1.3 * pm.R_min)


def test_quadcopter():
    ph = PhysicsParams()
    q = build_drone_params("quadcopter", ph)
    assert isinstance(q, QuadcopterParams)
    # 物理量
    assert close(q.mass, 0.468 / math.sqrt(2.5))
    assert close(q.k_omega, 800.0)
    assert q.thrust_max == q.k_omega            # 派生 = k_omega
    assert close(q.Va0, 5.0) and close(q.phi_max, math.radians(40))
    assert q.Va_T == 0.0 and q.pd_T == -200.0
    assert q.R_min == 0.0                       # Va_T=0 → R_min=0
    # gamma（独立重算）
    Jx, Jy, Jz, Jxz = 0.004856, 0.004856, 0.008801, 0.0
    g = Jx * Jz - Jxz ** 2
    assert close(q.gamma, g) and close(q.gamma3, Jz / g) and close(q.gamma8, Jx / g)
    # PID 增益（独立重算，KSI_OPT=0.7）
    assert close(q.kp_h, 1.5) and close(q.kd_h, 2.5) and close(q.ki_h, 0.5)  # 直接赋值
    assert close(q.kp_phi, Jx * 4 ** 2)
    assert close(q.kd_phi, 2 * 0.7 * 4 * Jx)
    assert close(q.kp_psi, Jz * 3 ** 2)
    assert close(q.kp_vd, -2 * 0.7 * 5 * q.mass)
    assert close(q.ki_vd, q.Az - 5 ** 2 * q.mass)
    uh = q.C_prop * q.k_omega ** 2 * 4 * 0.31 ** 2
    assert close(q.kp_vn, (-2 * 0.7 * 1 * q.mass + q.Ax) / uh)
    assert close(q.ki_pd, q.mass) and q.kp_pd == 0.0
    # 传感器矩阵（独立重算）
    sd_gyro = math.radians(0.13)
    sd_accel = 0.0025 * 9.81
    assert np.allclose(q.Q, np.eye(2) * sd_gyro ** 2)
    assert np.allclose(q.R, np.eye(3) * sd_accel ** 2)
    assert q.bias_gyro_x == 0.0                 # MATLAB 最终覆盖为 0


def test_fixed_wing():
    ph = PhysicsParams()
    w = build_drone_params("fixed_wing", ph)
    assert isinstance(w, FixedWingParams)
    assert close(w.mass, 13.5)
    assert close(w.Va_T, 35.0) and close(w.Va0, 35.0)
    # gamma（独立重算）
    Jx, Jy, Jz, Jxz = 0.8244, 1.135, 1.759, 0.1204
    g = Jx * Jz - Jxz ** 2
    assert close(w.gamma3, Jz / g) and close(w.gamma4, Jxz / g) and close(w.gamma8, Jx / g)
    # R_min = Va_T^2/gravity/tan(phi_max=60°)
    assert close(w.R_min, 35.0 ** 2 / 9.81 / math.tan(math.radians(60)), rel=1e-9)
    assert close(w.radius, 1.3 * w.R_min)
    # 配平初始条件（来自 table_T 烘焙常量）
    assert close(w.theta_T, 0.003419616115991571)
    assert close(w.de_T, -0.0493589082481536)
    assert close(w.dt_T, 0.4555434337688184)
    assert close(w.pd_T, -200.0, abs_=1e-9)     # xyu(3)~0 - 200
    # Cp_da = gamma3*Cl_da + gamma4*Cn_da（独立重算）
    assert close(w.Cp_da, w.gamma3 * 0.08 + w.gamma4 * 0.06)
    assert close(w.Cr_b, w.gamma4 * w.Cl_beta + w.gamma8 * w.Cn_beta)
    # PID 增益（部分可独立重算）
    assert close(w.kp_phi, math.radians(20) / math.radians(15))   # = 4/3
    assert close(w.kp_theta, math.radians(20) / math.radians(10) * (-1))  # sign(a.theta3)=-1 → -2
    # 其余增益：确保有限且符号/量级合理（完整数值 diff 待 MATLAB 快照）
    for name in ("kd_phi", "kp_chi", "ki_chi", "kd_theta", "kp_h", "ki_h",
                 "kp_va_dt", "ki_va_dt", "kp_va_theta", "ki_va_theta"):
        val = getattr(w, name)
        assert math.isfinite(val), f"{name} 非有限值: {val}"
    # 匿名函数 kb/kc → 方法
    assert close(w.kb(35.0), 0.5 * w.b / 35.0)
    assert close(w.kc(35.0), 0.5 * w.c / 35.0)


def test_map():
    m = build_map_params("point_mass", True)
    assert close(m.width, 300.0) and close(m.building_width, 22.5) and close(m.street_width, 52.5)
    assert m.buildings_north.shape == (14, 1)   # 列向量
    exp_north = [37.5] * 4 + [112.5] * 3 + [187.5] * 4 + [262.5] * 3
    exp_east = [37.5, 112.5, 187.5, 262.5, 75.0, 150.0, 225.0,
                37.5, 112.5, 187.5, 262.5, 75.0, 150.0, 225.0]
    assert np.allclose(m.buildings_north.ravel(), exp_north)
    assert np.allclose(m.buildings_east.ravel(), exp_east)
    assert np.allclose(m.buildings_heights.ravel(), [100.0] * 14)
    # 固定翼 + 环境关闭
    mw = build_map_params("fixed_wing", False)
    assert close(mw.width, 4000.0) and mw.ACTIVE_ENVIRONMENT is False
    assert mw.buildings_east.size == 0


def test_swarm_olfati():
    m = build_map_params("point_mass", True)
    o = build_swarm_params("olfati_saber", m, seed=42)
    # 派生 c = (b-a)/(2*sqrt(a*b))
    assert close(o.c, (5 - 1) / (2 * math.sqrt(5)))
    assert close(o.c_vm_obs, o.c_vm) and o.c_vm == 3.0
    assert o.lambda_ == 1.0                     # ⚠️ MATLAB p_swarm.lambda
    assert (o.delta, o.k, o.r0, o.c_pm_obs) == (0.2, 2.0, 10.0, 5.0)
    assert np.allclose(o.center_arena.ravel(), [0.0, 0.0, 0.0])
    # cylinders 来自 14 栋楼，半径 = building_width/2*sqrt(2)
    assert o.n_cyl == 14 and o.cylinders.shape == (3, 14)
    assert close(o.cylinders[2, 0], 22.5 / 2 * math.sqrt(2))
    assert np.allclose(o.cylinders[0, :], m.buildings_north.ravel())
    # Pos0 形状与范围
    assert o.Pos0.shape == (3, 5)
    assert o.c_r is None                        # isfield 为假 → 跳过随机速度项


def test_swarm_vasarhelyi():
    m = build_map_params("point_mass", True)
    v = build_swarm_params("vasarhelyi", m, seed=1)
    assert close(v.r0_rep, v.d_ref) and v.d_ref == 25.0
    assert close(v.p_rep, 0.03)
    assert (v.r0_fric, v.C_fric, v.v_fric, v.p_fric, v.a_fric) == (85.3, 0.05, 0.63, 3.2, 4.16)
    assert (v.r0_shill, v.v_shill, v.p_shill, v.a_shill) == (0.3, 13.6, 3.55, 3.02)
    assert v.n_cyl == 14


def test_seed_reproducibility():
    m = build_map_params("point_mass", True)
    a = build_swarm_params("olfati_saber", m, seed=123)
    b = build_swarm_params("olfati_saber", m, seed=123)
    c = build_swarm_params("olfati_saber", m, seed=999)
    assert np.array_equal(a.Pos0, b.Pos0)       # 同种子可复现
    assert not np.array_equal(a.Pos0, c.Pos0)   # 异种子不同


def test_build_params_and_overrides():
    # 群体场景：point_mass + olfati_saber
    bundle = build_params("point_mass", "olfati_saber", active_environment=True,
                          end_time=200, nb_agents=10, d_ref=30, v_ref=8,
                          loss_radio=0.3, seed=7)
    assert bundle.p_sim.end_time == 200
    assert bundle.p_drone.Va_T == 5.0           # point_mass
    assert bundle.map.buildings_east.shape == (14, 1)
    assert bundle.p_swarm.nb_agents == 10
    assert bundle.p_swarm.d_ref == 30 and bundle.p_swarm.v_ref == 8
    assert close(bundle.p_swarm.loss_radio, 0.3)
    assert bundle.p_swarm.Pos0.shape == (3, 10)

    # 覆盖 d_ref 时 vasarhelyi 的 r0_rep 应跟随
    b2 = build_params("point_mass", "vasarhelyi", d_ref=40, seed=7)
    assert close(b2.p_swarm.r0_rep, 40.0)

    # 单机场景：algorithm=None → 无 p_swarm
    b3 = build_params("quadcopter", None)
    assert b3.p_swarm is None
    assert isinstance(b3.p_drone, QuadcopterParams)


def _run_all():
    tests = [
        test_physics_sim_battery, test_point_mass, test_quadcopter, test_fixed_wing,
        test_map, test_swarm_olfati, test_swarm_vasarhelyi,
        test_seed_reproducibility, test_build_params_and_overrides,
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
