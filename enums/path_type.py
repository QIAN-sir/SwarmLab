"""PathType —— 路径类型枚举。

对应 MATLAB 源文件: enumerators/PathType.m

MATLAB 原始注释（保留）:
    % This class (enumerator) is meant to manage the  path types:
    % fillet, dubins, straight_rrt

迁移说明:
    - 采用 ``(str, Enum)`` 混入，保持与字符串比较的行为。
    - ⚠️ 已知不一致（记录，不修改；见 PROJECT_ARCHITECTURE.md §14 B-10）：
      MATLAB 的 @Drone/plan_path.m 中 switch 分支实际使用的是
      "fillet" / "dubins" / "city_dubins" / "straight_line_rrt" / 5，
      与本枚举 {fillet, dubins, straight_rrt} 并不完全对应。
      迁移 PathType 时保持与本枚举（原版定义）一致，plan_path 的差异在
      Phase 11 单独处理并记录。
"""

from enum import Enum


class PathType(str, Enum):
    """管理路径类型：fillet（圆角过渡）、dubins（杜宾斯曲线）、straight_rrt（直线 RRT）。"""

    FILLET = "fillet"
    DUBINS = "dubins"
    STRAIGHT_RRT = "straight_rrt"
