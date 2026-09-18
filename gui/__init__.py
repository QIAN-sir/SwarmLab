# -*- coding: utf-8 -*-
"""SwarmLab_py.gui —— PySide6 桌面 GUI（Stage 3 重做；对应 MATLAB GUI_swarm.mlapp，计划 Phase 13）。

按原版面板复刻（类型选择 / 集群参数设置 / 开始仿真开关），支持**运行中实时控制**
（方向罗盘、间距、速度、失控比例、无人机状态即时生效；数量/算法/地图/时长为开局设定）。
3D 视图内嵌 matplotlib QtAgg，复用 visualization.map_drawing 与 SwarmViewer 的 marker/坐标约定。
（早期 Streamlit 版 gui/app.py 已按用户要求舍弃，可从 git 3a229b2 找回。）

导入约定（见 MIGRATION_LOG.md 附录 A.0）：入口把 _REPO_ROOT(=c:\\SwarmLab) 加入 sys.path，
统一用 ``from SwarmLab_py.xxx import ...`` 绝对导入（源码内部才用相对导入）。

启动：
    C:\\Python\\python.exe -m SwarmLab_py.gui.desktop_app
依赖：pip install PySide6
"""
