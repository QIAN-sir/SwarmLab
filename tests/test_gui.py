# -*- coding: utf-8 -*-
"""Stage 3 —— 桌面 GUI（PySide6）测试（offscreen headless）。

未安装 PySide6 时自动 SKIP（不阻塞回归）；安装后（pip install PySide6）以 offscreen
平台构建 MainWindow，并手动步进 SimController 验证：建群 / 实时参数 / 故障注入 / 切回恢复。
导入约定见 MIGRATION_LOG 附录 A.0（_REPO_ROOT=c:\\SwarmLab）。
"""
import os
import shutil
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

try:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication          # noqa: E402
    HAVE_QT = True
except Exception:                                        # noqa: BLE001
    HAVE_QT = False


def _states(ctrl):
    return [getattr(d.drone_state, "value", d.drone_state) for d in ctrl.swarm.drones]


def test_gui_window_and_live_sim():
    if not HAVE_QT:
        print("[SKIP] 未安装 PySide6，跳过桌面 GUI 测试（pip install PySide6 后复跑）")
        return
    from SwarmLab_py.gui.desktop_app import MainWindow
    import SwarmLab_py.gui.desktop_app as _da
    _da.HAVE_PYVISTA = False   # 无头测试走稳定的 matplotlib 回退路径（pyvista 需 GPU/显示）
    app = QApplication.instance() or QApplication([])
    win = MainWindow()

    # 开局参数 → 拨动『开始仿真』→ 建控制器
    win.nb_spin.setValue(4)
    win.end_spin.setValue(2)
    win._on_start_toggled(True)
    assert win.controller is not None
    ctrl = win.controller
    rd = win._result_dir          # 自动记录的结果目录（测试后清理）

    # 高度保持（方案1）：h_target 设定后集群中心高度收敛到目标；h_target=0 → 贴地(无人车)
    ctrl.end_time = 100.0          # 高度收敛需较长时间，放宽 end（默认 end_spin=2 会提前 done）
    h0 = -float(ctrl.swarm.get_pos_ned()[2].mean())
    ctrl.h_target = h0 + 20.0
    for _ in range(1000):
        ctrl.apply_live(180, 10, 5)
        ctrl.step()
    h1 = -float(ctrl.swarm.get_pos_ned()[2].mean())
    assert abs(h1 - (h0 + 20.0)) < 3.0, f"高度保持未收敛 (h={h1:.1f}, target={h0 + 20:.1f})"
    ctrl.h_target = 0.0
    for _ in range(3000):
        ctrl.apply_live(180, 10, 5)
        ctrl.step()
    h2 = -float(ctrl.swarm.get_pos_ned()[2].mean())
    assert h2 < 3.0, f"h_target=0 未贴地/无人车 (h={h2:.1f})"
    ctrl.h_target = h0

    # 实时故障注入：Stalled, loss=0.3, nb=4 → ceil(1.2)=2 架
    ctrl.apply_live(180, 10, 5)
    ctrl.set_state("Stalled", 0.3)
    n_stalled = sum(1 for s in _states(ctrl) if s == "Stalled")
    assert n_stalled > 0, "set_dronestate 未注入 Stalled"

    # 手动步进（实时控制路径）
    for _ in range(30):
        ctrl.apply_live(180, 10, 5)
        ctrl.step()
    assert ctrl.time > 0
    assert len(ctrl.pos_history) > 0
    win._update_view()      # 3D 视图（含尾迹）offscreen 渲染
    win._update_debug()     # 调试视图渲染

    # 切回正常 → 全体恢复
    ctrl.set_state("Normal", 0.3)
    assert all(s == "Normal" for s in _states(ctrl)), "切回 Normal 未全体恢复"

    win._on_start_toggled(False)
    win.close()
    if rd:
        shutil.rmtree(rd, ignore_errors=True)   # 清理测试产物，保持 result/ 干净
    print(f"GUI(PySide6) OK: steps={len(ctrl.pos_history)}, stalled_injected={n_stalled}")


if __name__ == "__main__":
    test_gui_window_and_live_sim()
    print("test_gui: 1/1 PASS")
