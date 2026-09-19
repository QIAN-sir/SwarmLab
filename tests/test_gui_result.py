# -*- coding: utf-8 -*-
"""Stage 11 —— GUI 自动结果记录测试（offscreen headless）。

验证:
    - FrameVideoWriter：numpy 帧序列 → mp4（ffmpeg rawvideo 管道）
    - GUI 每次仿真自动记录：开始→建 result/<alg>_<ts>/ + 启动抓帧；停止→关视频 +
      存参数图（trajectories/performance/...）。视频录的是 GUI 3D 视图画面（含视角操作）。
未安装 PySide6 时 GUI 部分 SKIP；FrameVideoWriter 部分仍测。
"""
import os
import shutil
import sys
import tempfile

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import numpy as np                                    # noqa: E402

try:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication        # noqa: E402
    HAVE_QT = True
except Exception:                                      # noqa: BLE001
    HAVE_QT = False

PASS = []


def _check(name, cond):
    PASS.append((name, bool(cond)))
    print(("[PASS] " if cond else "[FAIL] ") + name)


def test_frame_video_writer():
    from SwarmLab_py.visualization import FrameVideoWriter
    d = tempfile.mkdtemp(prefix="swarmlab_fv_")
    p = os.path.join(d, "t.mp4")
    w = FrameVideoWriter(p, fps=10)
    rng = np.random.default_rng(0)
    for _ in range(20):
        w.add_frame((rng.random((120, 160, 3)) * 255).astype(np.uint8))
    w.close()
    _check("FrameVideoWriter: 20 帧计数", w.nframes == 20)
    _check("FrameVideoWriter: mp4 生成且非空", os.path.exists(p) and os.path.getsize(p) > 0)
    shutil.rmtree(d, ignore_errors=True)


def test_gui_auto_result():
    if not HAVE_QT:
        print("[SKIP] 未安装 PySide6，跳过 GUI 自动记录测试")
        return
    from SwarmLab_py.gui.desktop_app import MainWindow
    import SwarmLab_py.gui.desktop_app as _da
    _da.HAVE_PYVISTA = False        # offscreen 走 matplotlib 回退（稳定）
    app = QApplication.instance() or QApplication([])
    win = MainWindow()
    win.nb_spin.setValue(3)
    win.end_spin.setValue(2)

    win._on_start_toggled(True)                 # 开始 → 建 result 目录 + 启动录制
    rd = win._result_dir
    _check("GUI: 开始仿真自动建 result 目录", rd is not None and os.path.isdir(rd))
    _check("GUI: 视频 writer 已启动", win._video is not None)

    # 手动步进 + 抓帧（模拟仿真运行 + 视角画面采集）
    for _ in range(20):
        win.controller.step()
        win._update_view()
        win._capture_frame()
    _check("GUI: 抓到帧(>0)", win._video is not None and win._video.nframes > 0)

    win._on_start_toggled(False)                # 停止 → finalize：关视频 + 存参数图
    files = os.listdir(rd) if rd and os.path.isdir(rd) else []
    _check("GUI: 视频 mp4 已存且非空",
           "swarm_simulation.mp4" in files
           and os.path.getsize(os.path.join(rd, "swarm_simulation.mp4")) > 0)
    _check("GUI: 轨迹图已存", "trajectories.png" in files)
    _check("GUI: 性能图已存", "performance.png" in files)
    _check("GUI: performance.npz 已存", "performance.npz" in files)
    print(f"    [info] result 目录文件: {sorted(files)}")

    shutil.rmtree(rd, ignore_errors=True)       # 清理测试产物，保持 result/ 干净
    win.close()


if __name__ == "__main__":
    test_frame_video_writer()
    test_gui_auto_result()
    n = len(PASS); ok = sum(1 for _, c in PASS if c)
    print("-" * 50)
    print(f"Total: {n}  Passed: {ok}  Failed: {n - ok}")
    sys.exit(0 if ok == n else 1)
