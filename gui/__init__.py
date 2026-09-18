# -*- coding: utf-8 -*-
"""SwarmLab_py.gui —— Streamlit + Plotly 前端（Stage 3；对应 MATLAB GUI_swarm.mlapp，计划 Phase 13）。

导入约定（见 MIGRATION_LOG.md 附录 A.0）：入口把 _REPO_ROOT(=c:\\SwarmLab) 加入 sys.path，
统一用 ``from SwarmLab_py.xxx import ...`` 绝对导入（源码内部才用相对导入）。

启动：
    C:\\Python\\python.exe -m streamlit run SwarmLab_py/gui/app.py
"""
