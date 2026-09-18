"""SwarmLab_py —— SwarmLab (MATLAB) → Python 迁移项目。

本项目在保持原有软件架构、类调用关系、算法数学逻辑与仿真行为基本一致的
前提下，将基于 EPFL SwarmLab 二次开发（含无人机群成员失效/故障机制）的
MATLAB 仿真项目逐步迁移为结构清晰、可维护、可验证的 Python 项目。

参考文档:
    - 迁移计划: c:/SwarmLab/SwarmLab_MATLAB_to_Python_Migration_Plan.md
    - 架构分析(Phase 0): c:/SwarmLab/PROJECT_ARCHITECTURE.md
    - 迁移日志: SwarmLab_py/MIGRATION_LOG.md

目录组织（按功能分类，参考 MATLAB 开发文件存放规则）:
    enums/       枚举与常量          (MATLAB enumerators/)
    tests/       测试
    ...          后续 Phase 逐步补充 (core/ parameters/ swarm/ physics/ ...)
"""

__version__ = "0.1.0"
