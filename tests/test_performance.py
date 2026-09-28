"""Stage 2A —— 群体性能指标对照测试。

用可手算的小场景验证 pos2dist / compute_neighborhood / network_components /
compute_swarm_performance（safety/order/union/alg_conn/safety_obs/min_d_obs）。

运行方式（无需 pytest）:
    python SwarmLab_py/tests/test_performance.py
"""

import math
import os
import sys

import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from SwarmLab_py.math_tools import (                       # noqa: E402
    pos2dist, compute_neighborhood, network_components, compute_swarm_performance,
)
from SwarmLab_py.parameters import build_params             # noqa: E402


def test_pos2dist():
    positions = np.array([[0.0, 3.0], [0.0, 4.0], [0.0, 0.0]])   # (3,2)：d0原点, d1(3,4,0)
    D = pos2dist(positions)
    assert D.shape == (2, 2)
    assert math.isclose(D[0, 1], 5.0) and math.isclose(D[1, 0], 5.0)
    assert D[0, 0] == 0.0


def test_compute_neighborhood():
    dm = np.array([[0.0, 5.0, 200.0],
                   [5.0, 0.0, 200.0],
                   [200.0, 200.0, 0.0]])
    M = compute_neighborhood(dm, r_comm=150.0, max_neig=10)
    assert M[0, 1] == 1 and M[1, 0] == 1        # 0-1 距离 5<150 → 邻居
    assert M[0, 2] == 0 and M[2, 0] == 0        # 2 距离 200>150 → 孤立
    assert M[0, 0] == 0                          # 对角为 0


def test_network_components():
    A = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 0]])   # 0-1 连通，2 孤立
    ncomp, sizes, members = network_components(A)
    assert ncomp == 2 and sorted(sizes) == [1, 2]
    A2 = np.ones((3, 3)) - np.eye(3)                   # 完全图
    assert network_components(A2)[0] == 1


def _perf(pos_history, vel_history, env=False):
    p = build_params("point_mass", "olfati_saber", active_environment=env, seed=0).p_swarm
    return compute_swarm_performance(np.array(pos_history), np.array(vel_history), p), p


def test_safety_and_order_connected():
    # N=2 相距 10(>2*r_coll=1) 无碰撞；同向速度 → order=1；连通 → union=1, alg_conn=1
    (safety, order, union, alg_conn, safety_obs, min_d_obs), p = _perf(
        [[0, 0, 0, 10, 0, 0]], [[1, 0, 0, 1, 0, 0]])
    assert math.isclose(safety[0], 1.0)
    assert math.isclose(order[0], 1.0)
    assert math.isclose(union[0], 1.0)
    assert math.isclose(alg_conn[0], 1.0)          # K2 Fiedler=2, /N=2 → 1


def test_safety_collision():
    # 相距 0.4 < 2*r_coll(=1) → 1 对碰撞 / 1 对 → safety=0
    (safety, *_), p = _perf([[0, 0, 0, 0.4, 0, 0]], [[1, 0, 0, 1, 0, 0]])
    assert math.isclose(safety[0], 0.0)


def test_order_opposite():
    # 反向速度 → 余弦=-1 → order=-1
    (safety, order, *_), p = _perf([[0, 0, 0, 10, 0, 0]], [[1, 0, 0, -1, 0, 0]])
    assert math.isclose(order[0], -1.0)


def test_union_disconnected():
    # N=3，drone2 远在 500(>r=150) → 2 连通分量 → union=(3-2)/2=0.5, alg_conn=0
    (safety, order, union, alg_conn, *_), p = _perf(
        [[0, 0, 0, 10, 0, 0, 500, 0, 0]], [[1, 0, 0, 1, 0, 0, 1, 0, 0]])
    assert math.isclose(union[0], 0.5)
    assert math.isclose(alg_conn[0], 0.0, abs_tol=1e-9)


def test_safety_obs_with_cylinders():
    # 启用环境（14 圆柱），无人机置于远离建筑处 → 无碰撞 → safety_obs=1，min_d_obs>0
    (safety, order, union, alg_conn, safety_obs, min_d_obs), p = _perf(
        [[-1000, 0, 0, -1000, 50, 0, -1000, 100, 0]],
        [[1, 0, 0, 1, 0, 0, 1, 0, 0]], env=True)
    assert p.n_cyl == 14
    assert math.isclose(safety_obs[0], 1.0)
    assert np.all(min_d_obs[0] > 0)


def test_shapes_multi_step():
    # T=3, N=3 → 输出 (3,)，min_d_obs (3,3)
    pos = np.tile(np.array([[0, 0, 0, 10, 0, 0, 20, 0, 0]], dtype=float), (3, 1))
    vel = np.tile(np.array([[1, 0, 0, 1, 0, 0, 1, 0, 0]], dtype=float), (3, 1))
    (safety, order, union, alg_conn, safety_obs, min_d_obs), p = _perf(pos, vel)
    assert safety.shape == (3,) and order.shape == (3,)
    assert union.shape == (3,) and alg_conn.shape == (3,)
    assert min_d_obs.shape == (3, 3)


def test_obstacle_dist3d_geometry():
    """点-有限高圆柱三维表面距离手工核对。"""
    from SwarmLab_py.math_tools import uav_cyl_dist3d
    assert math.isclose(uav_cyl_dist3d(np.array([0, 0, -50.]), [0, 0], 10, 30), 20.0)   # 正上方 dz=20
    assert math.isclose(uav_cyl_dist3d(np.array([0, 0, -10.]), [0, 0], 10, 30), 0.0)    # 柱身内=0
    assert math.isclose(uav_cyl_dist3d(np.array([25, 0, -10.]), [0, 0], 10, 30), 15.0)  # 低空侧面
    assert math.isclose(uav_cyl_dist3d(np.array([25, 0, -50.]), [0, 0], 10, 30), 25.0)  # 高空斜向


def test_high_altitude_no_false_collision():
    """高空飞越不误判碰撞/近距离；低空 footprint 内判碰撞。"""
    p = build_params("point_mass", "olfati_saber", active_environment=True, seed=0).p_swarm
    p.cylinders = np.array([[0.], [0.], [10.]]); p.n_cyl = 1; p.cyl_heights = np.array([30.])
    (_, _, _, _, sobs, mdo) = compute_swarm_performance(
        np.array([[0, 0, -50.]]), np.array([[1, 0, 0.]]), p)
    assert math.isclose(sobs[0], 1.0), f"高空误判碰撞 sobs={sobs[0]}"
    assert math.isclose(mdo[0, 0], 20.0), f"高空 min_d_obs 应=20, got {mdo[0, 0]}"
    (_, _, _, _, so2, md2) = compute_swarm_performance(
        np.array([[0, 0, -10.]]), np.array([[1, 0, 0.]]), p)
    assert so2[0] < 1.0 and math.isclose(md2[0, 0], 0.0)


def test_fault_zero_vel_no_nan():
    """失效零速 UAV 不导致 order/union 除零 nan（健康子集）。"""
    (safety, order, union, ac, sobs, mdo) = compute_swarm_performance(
        np.array([[0, 0, 0, 10, 0, 0]]), np.array([[0, 0, 0, 1, 0, 0]]),
        build_params("point_mass", "olfati_saber", seed=0).p_swarm)
    assert not np.isnan(order[0]), "失效零速导致 order nan"
    assert not np.isnan(union[0]), "失效零速导致 union nan"


def test_stats():
    from SwarmLab_py.math_tools import compute_performance_stats
    (safety, order, union, ac, sobs, mdo) = compute_swarm_performance(
        np.array([[0, 0, 0, 10, 0, 0]]), np.array([[1, 0, 0, 1, 0, 0]]),
        build_params("point_mass", "olfati_saber", seed=0).p_swarm)
    st = compute_performance_stats(safety, order, union, ac, sobs, mdo)
    assert "uav_collision_steps" in st and "min_building_dist" in st
    assert st["uav_collision_steps"] == 0


def _run_all():
    tests = [
        test_pos2dist, test_compute_neighborhood, test_network_components,
        test_safety_and_order_connected, test_safety_collision, test_order_opposite,
        test_union_disconnected, test_safety_obs_with_cylinders, test_shapes_multi_step,
        test_obstacle_dist3d_geometry, test_high_altitude_no_false_collision,
        test_fault_zero_vel_no_nan, test_stats,
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
