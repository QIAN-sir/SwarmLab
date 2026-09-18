# -*- coding: utf-8 -*-
"""Stage 3 — GUI 测试（Streamlit AppTest headless）。

无头运行 gui/app.py：验证初始渲染无异常、点击『运行仿真』后 session_state 生成 sim/pos、
概览指标渲染。导入约定见 MIGRATION_LOG 附录 A.0（_REPO_ROOT=c:\\SwarmLab）。
"""
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from streamlit.testing.v1 import AppTest

APP = os.path.join(_REPO_ROOT, "SwarmLab_py", "gui", "app.py")


def test_gui_runs_and_simulates():
    at = AppTest.from_file(APP, default_timeout=180)
    at.run()
    assert not at.exception, f"初始渲染异常: {at.exception}"

    # best-effort：缩短仿真以加速（按侧边栏控件出现顺序索引）
    try:
        at.sidebar.slider[0].set_value(5)           # nb_agents
        at.sidebar.number_input[2].set_value(6.0)   # end_time
    except Exception as e:                          # noqa: BLE001
        print("  (控件预设跳过:", repr(e), ")")

    at.sidebar.button[0].click()                    # 『运行仿真』
    at.run()
    assert not at.exception, f"运行后异常: {at.exception}"
    assert "sim" in at.session_state
    assert at.session_state["n"] > 0
    assert len(at.metric) >= 6                       # 概览 6 个指标
    print(f"GUI AppTest OK: n={at.session_state['n']}, 概览指标={len(at.metric)}, 无异常")


if __name__ == "__main__":
    test_gui_runs_and_simulates()
    print("test_gui: 1/1 PASS")
