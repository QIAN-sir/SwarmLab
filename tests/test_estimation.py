# -*- coding: utf-8 -*-
"""Phase 10 / Stage 6 —— estimation 对照测试。

无 MATLAB use_estimation 参考数据，故验证策略：
    (1) 零噪声下手算期望值（sensors/gps/estimate_states 布局与数值）；
    (2) 修复行为断言（B-11 布局对齐、B-20 滤波记忆、B-21 accel_z=fz、B-22 EKF 门限）；
    (3) Drone use_estimation=True 接线冒烟（z_hat 合理）。
"""
import math
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import numpy as np

from SwarmLab_py.core import Drone
from SwarmLab_py.enums import DroneType, DroneState
from SwarmLab_py.parameters import build_params
from SwarmLab_py.physics import sensors, gps, new_gps_state
from SwarmLab_py.estimation import lp_filter, lp_step, estimate_states, new_est_state
from SwarmLab_py.control.true_states import true_states

PASS = []


def _check(name, cond):
    PASS.append((name, bool(cond)))
    print(("[PASS] " if cond else "[FAIL] ") + name)


def _wing_zero_noise():
    """fixed_wing 参数 + 关闭全部传感器噪声（确定性）。"""
    b = build_params("fixed_wing")
    pd = b.p_drone
    pd.sd_gyro = 0.0
    pd.sd_accel = 0.0
    pd.sd_static_pres = 0.0
    pd.sd_diff_pres = 0.0
    pd.sd_gps_pn = pd.sd_gps_pe = pd.sd_gps_ph = 0.0
    pd.sd_gps_vn = pd.sd_gps_ve = pd.sd_gps_vh = 0.0
    return b, pd, b.p_physics


def test_sensors_shape_and_B21():
    b, pd, pph = _wing_zero_noise()
    x = np.array([100, 200, -50, 15, 0, 0, 0, 0, 0, 0.1, 0.2, 0.3], float)
    forces = np.array([0.0, 0.0, 0.0])
    airdata = np.array([15.0, 0, 0, 0, 0, 0])
    y = sensors(x, forces, airdata, pd, pph)
    _check("sensors: shape (8,)", y.shape == (8,))
    _check("sensors: gyro = p,q,r (零噪声/零偏置)", np.allclose(y[0:3], [0.1, 0.2, 0.3], atol=1e-12))
    g = pph.gravity
    _check("sensors: accel_z = fz/m - g·cos·cos (B-21, phi=theta=0,fz=0 → -g)",
           abs(y[5] - (-g)) < 1e-9)
    # B-21: 改 fz 应改变 accel_z（原误用 fx 则不变）
    y2 = sensors(x, np.array([0.0, 0.0, pd.mass * 3.0]), airdata, pd, pph)
    _check("sensors B-21: accel_z 随 fz 变化 (+3 → -g+3)", abs(y2[5] - (3.0 - g)) < 1e-9)
    _check("sensors B-21: accel_x 不受 fz 影响", abs(y2[3] - y[3]) < 1e-12)


def test_gps_passthrough_and_state():
    b, pd, pph = _wing_zero_noise()
    x = np.array([100, 200, -50, 15, 0, 0, 0, 0, 0, 0, 0, 0], float)
    st = new_gps_state()
    y = gps(x, 0.01, pd, st)
    _check("gps: shape (5,)", y.shape == (5,))
    # 零噪声 → mp_*=0 → [pn, pe, -pd, Vg, course]；level psi=0, vx=15 → Vg=15, course=0
    _check("gps: 零噪声直通 [pn,pe,-pd,Vg,course]",
           np.allclose(y, [100, 200, 50, 15, 0], atol=1e-9))
    _check("gps: Markov 状态保持 0（零激励）", st["mp_n"] == 0.0 and st["mp_h"] == 0.0)


def test_lp_filter_formula_and_memory_B20():
    # 公式：lp_filter(y_prev,u,a,dt)=exp(-a dt)y_prev+(1-exp(-a dt))u
    a, dt = 0.1, 0.01
    _check("lp_filter: (1,0) → exp(-a·dt)", abs(lp_filter(1.0, 0.0, a, dt) - math.exp(-a * dt)) < 1e-12)
    # B-20：带记忆滤波，阶跃输入 u:0→10 应逐步逼近 10（有滞后），而非一步到位/退化
    lpst = {}
    lp_step(lpst, "c", 0.0, 5.0, dt)          # 初始化到 0
    vals = [lp_step(lpst, "c", 10.0, 5.0, dt) for _ in range(200)]
    _check("lp_step B-20: 阶跃响应有滞后（首步 <10）", vals[0] < 10.0 - 1e-6)
    _check("lp_step B-20: 单调收敛到 u=10", abs(vals[-1] - 10.0) < 1e-3 and vals[-1] > vals[0])


def test_estimate_states_layout_B11():
    b, pd, pph = _wing_zero_noise()
    x = np.array([100, 200, -50, 15, 0, 0, 0, 0, 0, 0.1, 0.2, 0.3], float)
    forces = np.zeros(3)
    airdata = np.array([15.0, 0, 0, 0, 0, 0])
    y_imu = sensors(x, forces, airdata, pd, pph)
    y_gps = gps(x, 0.01, pd, new_gps_state())
    uu = np.concatenate([y_imu, y_gps])
    st = new_est_state()
    z = estimate_states(uu, 0.0, 0.01, pd, pph, st)
    _check("estimate_states: z_hat shape (22,)", z.shape == (22,))
    # B-11 规范布局（与 true_states 一致）：pn0 pe1 h2 Va6 phi9 theta10 psi11 chi12 p13 q14 r15 Vg16
    _check("B-11: idx0/1 = pn/pe", abs(z[0] - 100) < 1e-6 and abs(z[1] - 200) < 1e-6)
    _check("B-11: idx2 = h = 50（静压反解）", abs(z[2] - 50.0) < 1e-6)
    _check("B-11: idx6 = Va = 15（动压反解）", abs(z[6] - 15.0) < 1e-6)
    _check("B-11: idx11 = psi = 0（不可观测）", abs(z[11]) < 1e-12)
    _check("B-11: idx12 = chi = course = 0", abs(z[12]) < 1e-6)
    _check("B-11: idx13/14/15 = p/q/r（陀螺，首步直通）",
           np.allclose(z[13:16], [0.1, 0.2, 0.3], atol=1e-9))


def test_estimate_states_matches_true_states_slots():
    """B-11 核心：estimate_states 与 true_states 在共同可估计字段上索引槽位一致。"""
    b, pd, pph = _wing_zero_noise()
    x = np.array([100, 200, -50, 15, 0, 0, 0, 0, 0, 0, 0, 0], float)
    airdata = np.array([15.0, 0, 0, 0, 0, 0])
    zt = true_states(np.concatenate([x, airdata, [0.0]]))
    y_imu = sensors(x, np.zeros(3), airdata, pd, pph)
    y_gps = gps(x, 0.01, pd, new_gps_state())
    ze = estimate_states(np.concatenate([y_imu, y_gps]), 0.0, 0.01, pd, pph, new_est_state())
    for idx, label in [(0, "pn"), (1, "pe"), (2, "h"), (6, "Va"), (13, "p"), (14, "q"), (15, "r")]:
        _check(f"B-11 槽位一致 idx{idx}={label}", abs(zt[idx] - ze[idx]) < 1e-6)


def test_ekf_gate_B22():
    """B-22：|‖accel‖-g|<0.5 时 EKF 更新触发（协方差收缩）；否则只预测（协方差=Q）。"""
    b, pd, pph = _wing_zero_noise()
    g = pph.gravity

    def run_once(accel):
        uu = np.zeros(13)
        uu[3:6] = accel          # 三轴加速度
        uu[6] = pd.pres0 + pph.rho * g * (-50.0)   # 静压 → h=50
        uu[7] = pph.rho * 15.0 ** 2 / 2            # 动压 → Va=15
        st = new_est_state()
        estimate_states(uu, 0.0, 0.01, pd, pph, st)
        return float(np.trace(st["kf_P"]))

    tr_fire = run_once(np.array([0.0, 0.0, -g]))   # ‖z‖=g → 触发更新
    tr_skip = run_once(np.array([0.0, 0.0, 0.0]))  # ‖z‖=0 → 跳过更新
    _check("B-22: 触发更新时 trace(kf_P) < 跳过时（协方差收缩）", tr_fire < tr_skip - 1e-12)
    _check("B-22: 跳过更新时 kf_P = Q（仅预测）", abs(tr_skip - float(np.trace(pd.Q))) < 1e-12)


def test_drone_wiring_estimation():
    """Drone use_estimation=True 接线冒烟：手动跑 传感→估计，检查 z_hat 合理。"""
    b, pd, pph = _wing_zero_noise()
    d = Drone(DroneType.FIXED_WING, DroneState.NORMAL, pd, b.p_battery,
              b.p_sim, pph, b.map)
    d.use_estimation = True
    d.set_state(np.array([100, 200, -50.0]), np.array([15.0, 0, 0]),
                np.zeros(3), np.zeros(3))
    d.airdata = np.array([15.0, 0, 0, 0, 0, 0])
    d.forces = np.zeros(3)
    d.update_sensor_measurements()
    _check("Drone 接线: y shape (13,)", d.y is not None and d.y.shape == (13,))
    d.estimate_states(0.0)
    _check("Drone 接线: z_hat h≈50", abs(d.z_hat[2] - 50.0) < 1e-6)
    _check("Drone 接线: z_hat Va≈15", abs(d.z_hat[6] - 15.0) < 1e-6)
    _check("Drone 接线: z_hat 全部有限", np.all(np.isfinite(d.z_hat)))


def test_estimator_converges_level_flight():
    """零噪声水平飞行多步：phi/theta 估计保持有界（EKF 稳定，不发散）。"""
    b, pd, pph = _wing_zero_noise()
    g = pph.gravity
    st = new_est_state()
    ok = True
    for k in range(500):
        uu = np.zeros(13)
        uu[3:6] = [0.0, 0.0, -g]                 # 水平：加速度≈-g（z 轴）
        uu[6] = pd.pres0 + pph.rho * g * (-50.0)
        uu[7] = pph.rho * 15.0 ** 2 / 2
        uu[8:11] = [100.0, 0.0, 50.0]            # gps pn,pe,ph
        uu[11] = 15.0                            # Vg
        z = estimate_states(uu, k * 0.01, 0.01, pd, pph, st)
        if not np.isfinite(z[9]) or not np.isfinite(z[10]) or abs(z[9]) > 1.0 or abs(z[10]) > 1.0:
            ok = False
            break
    _check("EKF 稳定: 水平飞行 500 步 phi/theta 估计有界(|·|<1rad)", ok)


if __name__ == "__main__":
    np.random.seed(0)
    test_sensors_shape_and_B21()
    test_gps_passthrough_and_state()
    test_lp_filter_formula_and_memory_B20()
    test_estimate_states_layout_B11()
    test_estimate_states_matches_true_states_slots()
    test_ekf_gate_B22()
    test_drone_wiring_estimation()
    test_estimator_converges_level_flight()
    n = len(PASS); ok = sum(1 for _, c in PASS if c)
    print("-" * 50)
    print(f"Total: {n}  Passed: {ok}  Failed: {n - ok}")
    sys.exit(0 if ok == n else 1)
