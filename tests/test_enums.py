"""Phase 1 —— 枚举迁移对照测试。

验证目标（对应迁移计划 §17 “MATLAB/Python 对照” 与 §25 “数值/行为一致”）：
    1. 每个枚举的成员“值”与 MATLAB enumerators/*.m 中定义的成员名严格一致
       （含大小写与顺序）。
    2. ``(str, Enum)`` 混入带来的“枚举 == 字符串”比较行为与 MATLAB 一致，
       即 MATLAB 中 ``drone_type == "point_mass"``、``drone_state == "Normal"``
       在 Python 中同样成立。
    3. 成员数量一致（防止漏迁/多迁）。

运行方式（无需 pytest）：
    python SwarmLab_py/tests/test_enums.py
或（若已安装 pytest）：
    pytest SwarmLab_py/tests/test_enums.py
"""

import os
import sys

# 将 SwarmLab_py 的父目录（仓库根 c:/SwarmLab）加入 sys.path，
# 使得无论当前工作目录为何，都能 import SwarmLab_py.*
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from SwarmLab_py.enums import (  # noqa: E402
    DroneType,
    DroneState,
    PathType,
    SwarmAlgorithm,
    SwarmViewerType,
)

# ---------------------------------------------------------------------------
# MATLAB enumerators/*.m 中定义的成员名（权威参照，顺序即 MATLAB 声明顺序）
# ---------------------------------------------------------------------------
MATLAB_MEMBERS = {
    "DroneType": ["quadcopter", "fixed_wing", "point_mass"],
    "DroneState": ["Normal", "Stalled", "Drifting"],
    "PathType": ["fillet", "dubins", "straight_rrt"],
    "SwarmAlgorithm": ["vasarhelyi", "olfati_saber"],
    "SwarmViewerType": ["drone", "agent", "agent_with_energy"],
}

ENUMS = {
    "DroneType": DroneType,
    "DroneState": DroneState,
    "PathType": PathType,
    "SwarmAlgorithm": SwarmAlgorithm,
    "SwarmViewerType": SwarmViewerType,
}


def test_values_match_matlab():
    """成员值与顺序须与 MATLAB 定义完全一致。"""
    for name, enum_cls in ENUMS.items():
        expected = MATLAB_MEMBERS[name]
        actual = [m.value for m in enum_cls]
        assert actual == expected, f"{name} 值不一致: MATLAB={expected} Python={actual}"


def test_member_count():
    """成员数量一致。"""
    for name, enum_cls in ENUMS.items():
        assert len(list(enum_cls)) == len(MATLAB_MEMBERS[name]), f"{name} 成员数量不一致"


def test_string_comparison_behavior():
    """关键：枚举与字符串直接比较须成立（复刻 MATLAB 行为）。"""
    for name, enum_cls in ENUMS.items():
        for matlab_name in MATLAB_MEMBERS[name]:
            member = enum_cls(matlab_name)          # 由值取成员
            assert member == matlab_name, (
                f"{name}({matlab_name}) == '{matlab_name}' 应为 True（MATLAB 行为）"
            )
            assert member.value == matlab_name


def test_specific_matlab_comparisons():
    """抽查 MATLAB 源码中真实出现的比较表达式。"""
    # Drone.update_state: self.drone_type == "point_mass"
    assert DroneType.POINT_MASS == "point_mass"
    assert not (DroneType.QUADCOPTER == "point_mass")
    # Drone.update_state: self.drone_state == "Normal" / "Stalled" / "Drifting"
    assert DroneState.NORMAL == "Normal"
    assert DroneState.STALLED == "Stalled"
    assert DroneState.DRIFTING == "Drifting"
    # Swarm.update_command: self.algorithm == "olfati_saber"
    assert SwarmAlgorithm.OLFATI_SABER == "olfati_saber"
    assert SwarmAlgorithm.VASARHELYI == "vasarhelyi"
    # SwarmViewer.update: self.viewer_type == "drone" / "agent"
    assert SwarmViewerType.AGENT == "agent"
    # PathType
    assert PathType.STRAIGHT_RRT == "straight_rrt"


def _run_all():
    """无 pytest 时的简易运行器。"""
    tests = [
        test_values_match_matlab,
        test_member_count,
        test_string_comparison_behavior,
        test_specific_matlab_comparisons,
    ]
    passed = 0
    failed = 0
    for t in tests:
        try:
            t()
            print(f"[PASS] {t.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"[FAIL] {t.__name__}: {e}")
            failed += 1
    print("-" * 50)
    print(f"Total: {passed + failed}  Passed: {passed}  Failed: {failed}")
    return failed == 0


if __name__ == "__main__":
    ok = _run_all()
    sys.exit(0 if ok else 1)
