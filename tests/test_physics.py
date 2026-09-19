# -*- coding: utf-8 -*-
"""Phase 8 / Stage 4 —— physics 对照测试（解析/手算期望值）。

验证 forces_moments_quad / forces_moments_wing / kinematics_ode_fun 与
MATLAB 物理方程的解析性质一致（悬停平衡、力矩符号、升力、刚体导数）。
"""
import math
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import numpy as np

from SwarmLab_py.math_tools import Rb2i
from SwarmLab_py.parameters import build_params
from SwarmLab_py.physics import (forces_moments_quad, forces_moments_wing,
                                 kinematics_ode_fun)

PASS = []


def _check(name, cond):
    PASS.append((name, bool(cond)))
    print(("[PASS] " if cond else "[FAIL] ") + name)


def _quad():
    b = build_params("quadcopter")
    return b.p_drone, b.p_sim, b.p_physics


def _wing():
    b = build_params("fixed_wing")
    return b.p_drone, b.p_physics


def test_quad_hover_equilibrium():
    pd, psim, pph = _quad()
    d = math.sqrt(pd.mass * pph.gravity / (4 * pd.C_prop * pd.k_omega ** 2))
    out = forces_moments_quad(np.zeros(12), np.full(4, d), np.zeros(6), pd, psim, pph)
    _check("quad hover: F≈0", np.allclose(out[0:3], 0, atol=1e-8))
    _check("quad hover: M≈0", np.allclose(out[3:6], 0, atol=1e-8))
    _check("quad hover: Va=0,alpha=0", out[6] == 0 and out[7] == 0)


def test_quad_thrust_torque_signs():
    pd, psim, pph = _quad()
    d = math.sqrt(pd.mass * pph.gravity / (4 * pd.C_prop * pd.k_omega ** 2))
    delta = np.full(4, d); delta[0] *= 1.2     # 加大 d1
    out = forces_moments_quad(np.zeros(12), delta, np.zeros(6), pd, psim, pph)
    _check("quad: +d1 → M_y>0", out[4] > 0)      # torque_y ∝ +d1^2-d3^2
    _check("quad: +d1 → M_x≈0 (x 仅含 d2,d4)", abs(out[3]) < 1e-8)
    delta2 = np.full(4, d); delta2[1] *= 1.2
    out2 = forces_moments_quad(np.zeros(12), delta2, np.zeros(6), pd, psim, pph)
    _check("quad: +d2 → M_x<0", out2[3] < 0)     # torque_x ∝ -d2^2+d4^2


def test_wing_lift_drag():
    pd, pph = _wing()
    Va = 20.0
    x = np.zeros(12); x[3] = Va                  # body +x 速度, level
    out = forces_moments_wing(x, np.zeros(4), np.zeros(6), pd, pph)
    p_dyn = 0.5 * pph.rho * Va ** 2
    CL0, CD0 = pd.CL0, pd.CD0
    # alpha=0 → Cx=-CD0, Cz=-CL0; f_tot[2] = m*g + p_dyn*S*Cz (thrust z=0)
    expect_fz = pd.mass * pph.gravity + p_dyn * pd.S_wing * (-CL0)
    _check("wing: f_z = mg - q*S*CL0", abs(out[2] - expect_fz) < 1e-8)
    _check("wing: alpha=0", abs(out[7]) < 1e-12)
    _check("wing: Va echoed", abs(out[6] - Va) < 1e-12)


def test_kinematics_zero():
    pd, _, _ = _quad()
    dxdt = kinematics_ode_fun(0, np.zeros(12), np.zeros(12), np.zeros(6), pd)
    _check("kinematics: all-zero state → dxdt=0", np.allclose(dxdt, 0, atol=1e-12))


def test_kinematics_accel_and_rotation():
    pd, _, _ = _quad()
    x = np.zeros(12); x[3] = 1.0                 # body +x 速度
    uu = np.zeros(6); uu[0] = pd.mass * 2.0      # fx = m*2 → vxdot=2
    dxdt = kinematics_ode_fun(0, x, x, uu, pd)
    _check("kinematics: vxdot = fx/m", abs(dxdt[3] - 2.0) < 1e-12)
    _check("kinematics: pos_dot = Rbi@v (level→[1,0,0])",
           np.allclose(dxdt[0:3], [1, 0, 0], atol=1e-12))
    # psi=90°: body +x → NED east [0,1,0]
    x2 = np.zeros(12); x2[3] = 1.0; x2[8] = math.radians(90)
    dx2 = kinematics_ode_fun(0, x2, x2, np.zeros(6), pd)
    expect = Rb2i(0, 0, math.radians(90)) @ np.array([1.0, 0, 0])
    _check("kinematics: psi=90 pos_dot = Rb2i@v", np.allclose(dx2[0:3], expect, atol=1e-12))


if __name__ == "__main__":
    test_quad_hover_equilibrium()
    test_quad_thrust_torque_signs()
    test_wing_lift_drag()
    test_kinematics_zero()
    test_kinematics_accel_and_rotation()
    n = len(PASS); ok = sum(1 for _, c in PASS if c)
    print("-" * 50)
    print(f"Total: {n}  Passed: {ok}  Failed: {n - ok}")
    sys.exit(0 if ok == n else 1)
