# -*- coding: utf-8 -*-
"""§17/§18 —— MATLAB/Python 数值对照（point_mass 无故障主线）。

加载 matlab_ref/matlab_ref_{olfati_saber,vasarhelyi}.mat（由 MATLAB export_ref_swarm.m 导出），
用同一 Pos0 在 Python 跑无故障仿真，逐帧比较 pos/vel，报告 max/mean 绝对误差（§18）。
.mat 不存在时 SKIP（不阻塞回归）。
"""
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import numpy as np
import scipy.io as sio

from SwarmLab_py.simulation import Simulation

REF_DIR = os.path.join(_REPO_ROOT, "matlab_ref")
TOL = 1e-6

PASS = []


def _check(name, cond):
    PASS.append((name, bool(cond)))
    print(("[PASS] " if cond else "[FAIL] ") + name)


def _compare(alg):
    fname = {"olfati_saber": "matlab_ref_olfati.mat",
             "vasarhelyi": "matlab_ref_vasarhelyi.mat"}[alg]
    path = os.path.join(REF_DIR, fname)
    if not os.path.exists(path):
        print(f"[SKIP] 缺少 {path}")
        return None
    m = sio.loadmat(path)
    Pos_hist = np.asarray(m["Pos_hist"], dtype=float)
    Vel_hist = np.asarray(m["Vel_hist"], dtype=float)
    Pos0 = np.asarray(m["Pos0"], dtype=float)
    dt = float(np.asarray(m["dt"]).ravel()[0])
    T = int(np.asarray(m["T"]).ravel()[0])
    N = int(np.asarray(m["N"]).ravel()[0])

    sim = Simulation(drone_type="point_mass", algorithm=alg, nb_agents=N,
                     active_environment=True, end_time=T * dt)
    # Stage 13 高度感知为有意偏离 MATLAB 原行为；对照测试关闭它以还原原 2D 避障行为验证迁移忠实
    sim.p_swarm.height_aware_cyl = False
    # 用 MATLAB 的 Pos0 覆盖；重置历史使 row0 = Pos0（消除构造时自带 set_pos 的额外行）
    for d in sim.swarm.drones:
        d.pos_ned_history = np.zeros((0, 3))
        d.vel_xyz_history = np.zeros((0, 3))
    sim.p_swarm.Pos0 = Pos0
    sim.swarm.set_pos(Pos0)
    # 补初始 vel 行（不用 Swarm.set_vel，其为忠实保留的 B-2 bug 会抛 AttributeError）
    for d in sim.swarm.drones:
        d.vel_xyz = np.zeros(3)
        d._append_vel_history()
    for _ in range(T):
        sim.step()

    Pos_py = sim.get_pos_history()
    Vel_py = sim.get_vel_history()
    if Pos_py.shape != Pos_hist.shape:
        print(f"  shape mismatch: py{Pos_py.shape} vs mat{Pos_hist.shape}")
        return None
    ep = np.abs(Pos_py - Pos_hist)
    ev = np.abs(Vel_py - Vel_hist)
    return dict(alg=alg, pos_max=float(ep.max()), pos_mean=float(ep.mean()),
                vel_max=float(ev.max()), vel_mean=float(ev.mean()))


def test_matlab_ref_olfati():
    r = _compare("olfati_saber")
    if r is None:
        return
    print(f"  olfati: pos max={r['pos_max']:.3e} mean={r['pos_mean']:.3e} | "
          f"vel max={r['vel_max']:.3e} mean={r['vel_mean']:.3e}")
    _check("olfati pos max abs err < TOL", r["pos_max"] < TOL)


def test_matlab_ref_vasarhelyi():
    r = _compare("vasarhelyi")
    if r is None:
        return
    print(f"  vasarhelyi: pos max={r['pos_max']:.3e} mean={r['pos_mean']:.3e} | "
          f"vel max={r['vel_max']:.3e} mean={r['vel_mean']:.3e}")
    _check("vasarhelyi pos max abs err < TOL", r["pos_max"] < TOL)


if __name__ == "__main__":
    test_matlab_ref_olfati()
    test_matlab_ref_vasarhelyi()
    n = len(PASS); ok = sum(1 for _, c in PASS if c)
    print("-" * 50)
    print(f"Total: {n}  Passed: {ok}  Failed: {n - ok}")
    sys.exit(0 if ok == n else 1)
