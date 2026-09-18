# -*- coding: utf-8 -*-
"""SwarmLab 迁移版桌面 GUI —— PySide6（Stage 3 重做；对应 MATLAB examples/GUI_swarm.mlapp）。

按用户提供的原版面板截图复刻布局与控件，并支持**运行中实时控制**：
  - 左列「类型选择」：无人机类型(质点型) / 群体分布式优化算法(Vasarhelyi|Olfati-Saber) /
    无人机状态(正常|动力锁死|失控漂移) / 失控比例滑杆(0-60) / 地图设置(显示地图障碍) / 调试设置(绘制调试视图)
  - 右列「集群参数设置」：无人机数量 / 间距滑杆[m](5-15) / 速度滑杆[m/s](2-8) / 仿真时长 / 前进方向罗盘(180=N)
  - 底部「开始仿真」拨动开关（关闭/开启）
实时性：方向罗盘、间距、速度、失控比例、无人机状态在运行中即时生效（每步读取）；
仅 无人机数量 / 避障算法 / 障碍物(地图) / 仿真时长 为开局设定（运行中禁用）。

仿真引擎：QTimer 步进，逐步复刻 runner 的主循环顺序
（update_command → 计算 fad → update_state），但参数来自实时控件（不预设故障/恢复时间）。
3D 视图：内嵌 matplotlib QtAgg 画布，复用 visualization.map_drawing 的建筑/坐标轴绘制，
故障 marker ○/□/△ 与 SwarmViewer 一致。

导入约定（MIGRATION_LOG 附录 A.0）：_REPO_ROOT=c:\\SwarmLab 入 sys.path，用 from SwarmLab_py.xxx。

启动：
    C:\\Python\\python.exe -m SwarmLab_py.gui.desktop_app
依赖：pip install PySide6
"""
import math
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import numpy as np                                                     # noqa: E402
import matplotlib                                                      # noqa: E402

# 中文字体：修复 3D 标题/标签汉字显示为方框（默认 DejaVu 无 CJK 字形）
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DengXian", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

from PySide6.QtCore import Qt, QTimer, Signal                          # noqa: E402
from PySide6.QtGui import QPainter, QPen, QColor, QFont                # noqa: E402
from PySide6.QtWidgets import (                                        # noqa: E402
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QRadioButton, QSlider, QCheckBox, QSpinBox, QLabel, QSizePolicy,
    QPushButton)

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg       # noqa: E402
from matplotlib.figure import Figure                                   # noqa: E402

from SwarmLab_py.core import Swarm                                     # noqa: E402
from SwarmLab_py.enums import DroneState                               # noqa: E402
from SwarmLab_py.parameters import build_params                        # noqa: E402
from SwarmLab_py.visualization.map_drawing import (                    # noqa: E402
    draw_buildings, draw_cylinders, set_map_axes)

HAVE_PYVISTA = False
try:
    import pyvista as pv                                               # noqa: E402
    from pyvistaqt import QtInteractor                                 # noqa: E402
    HAVE_PYVISTA = True
except Exception:                                                      # noqa: BLE001
    HAVE_PYVISTA = False

# 故障状态 -> matplotlib marker（与 SwarmViewer 的 ○/□/△ 一致）
_STATE_MARKER = {"Normal": "o", "Stalled": "s", "Drifting": "^"}
_STATE_COLOR = {"Normal": "#1f77b4", "Stalled": "#ff7f0e", "Drifting": "#d62728"}


# ======================================================================
# 实时仿真控制器（逐步复刻 runner 主循环，参数来自实时控件）
# ======================================================================
# ======================================================================
# 无人机图标尺寸（世界单位，三种类型统一大小）
# ★ 后期手动调整：只需改 DRONE_MARKER_SIZE 这一个数。
#   球体半径 = S；正方体边长 = 2*S；四棱锥底边 = 2*S、高 = 2*S。
#   地图宽约 300 m，取值越大越夸张，建议 0.5 ~ 3。
DRONE_MARKER_SIZE = 1.5
# ======================================================================


def _marker_geoms(s):
    """三种状态的世界单位 glyph 几何（统一尺寸 s）：球体 / 正方体 / 四棱锥。"""
    sphere = pv.Sphere(radius=s)
    cube = pv.Cube(x_length=2 * s, y_length=2 * s, z_length=2 * s)
    pts = [(-s, -s, -s), (s, -s, -s), (s, s, -s), (-s, s, -s), (0, 0, s)]
    faces = [4, 0, 1, 2, 3,
             3, 0, 1, 4,
             3, 1, 2, 4,
             3, 2, 3, 4,
             3, 3, 0, 4]
    pyr = pv.PolyData(pts, faces=faces)
    return sphere, cube, pyr


class SimController:
    """point_mass 故障群实时仿真。开局参数建群；运行中参数每步读取。"""

    def __init__(self, drone_type, algorithm, nb_agents, active_environment, end_time):
        self.params = build_params(drone_type, algorithm,
                                   active_environment=active_environment,
                                   nb_agents=nb_agents, end_time=end_time)
        self.algorithm = algorithm
        self.swarm = Swarm()
        self.swarm.algorithm = algorithm
        for _ in range(self.params.p_swarm.nb_agents):
            self.swarm.add_drone(drone_type, DroneState.NORMAL, self.params.p_drone,
                                 self.params.p_battery, self.params.p_sim,
                                 self.params.p_physics, self.params.map)
        self.swarm.set_pos(self.params.p_swarm.Pos0)
        self.wind = np.zeros(6)
        self.dt = float(self.params.p_sim.dt)
        self.time = float(self.params.p_sim.start_time)
        self.end_time = float(end_time)
        self.current_state = "Normal"
        self.swarm.t_fault = 0.0
        self.done = False
        self.pos_history = []          # 每帧 (3,N)
        self.state_history = []        # 每帧 (N,) 状态字符串
        self.min_dist_history = []     # 每帧最小机间距（调试视图用）

    # ---- 实时参数（每步调用）----
    def apply_live(self, orientation_deg, d_ref, v_ref):
        p = self.params.p_swarm
        o = math.radians(orientation_deg)
        p.u_ref = np.array([[-math.cos(o)], [-math.sin(o)], [0.0]])   # 复刻 runner: u_ref=[-cos,-sin,0]
        p.d_ref = float(d_ref)
        p.v_ref = float(v_ref)
        if hasattr(p, "r0_rep"):        # Vasarhelyi 派生量随 d_ref 更新
            p.r0_rep = float(d_ref)

    def set_state(self, state, loss_radio):
        """运行中切换故障状态；切回 Normal 即全体恢复（不预设时间）。"""
        if state == self.current_state:
            return
        changed = 2 if state == "Drifting" else 1
        if state == "Drifting":
            self.swarm.t_fault = self.time
        self.current_state = state
        self.swarm.set_dronestate(state, changed, loss_radio)

    def _fad(self):
        if self.current_state in ("Normal", "Stalled"):
            return 1.0
        if self.swarm.t_fault != 0:
            f = 1.0 / (1.0 + 0.2 * (self.time - self.swarm.t_fault))
            f = min(max(f, 0.0), 1.0)
            return 0.0 if f < 0.1 else f
        return self.swarm.fad

    def step(self):
        if self.done:
            return False
        self.swarm.update_command(self.params.p_swarm, self.params.p_swarm.r_coll, self.dt)
        fad = self._fad()
        self.swarm.fad = fad
        self.swarm.update_state(self.wind, self.time, fad)
        pos = self.swarm.get_pos_ned()
        self.pos_history.append(pos.copy())
        self.state_history.append([getattr(d.drone_state, "value", d.drone_state)
                                   for d in self.swarm.drones])
        # 最小机间距（调试视图）
        if pos.shape[1] > 1:
            d2 = ((pos[:, :, None] - pos[:, None, :]) ** 2).sum(axis=0)
            np.fill_diagonal(d2, np.inf)
            self.min_dist_history.append(float(np.sqrt(d2.min())))
        else:
            self.min_dist_history.append(float("nan"))
        self.time += self.dt
        if self.time > self.end_time + 1e-9:
            self.done = True
        return not self.done


# ======================================================================
# 自定义控件：前进方向罗盘 / 开始仿真拨动开关
# ======================================================================
class CompassWidget(QWidget):
    """前进方向罗盘（0-359°，180=N）。可拖拽旋转，valueChanged(int)。"""
    valueChanged = Signal(int)

    def __init__(self, value=180, parent=None):
        super().__init__(parent)
        self._value = int(value) % 360
        self.setMinimumSize(180, 180)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def value(self):
        return self._value

    def setValue(self, v):
        v = int(v) % 360
        if v != self._value:
            self._value = v
            self.valueChanged.emit(v)
            self.update()

    def _needle(self):
        # 行进方向在数学坐标(右=East,上=North)：(-sin, -cos)；θ=180 → (0,1)=North
        o = math.radians(self._value)
        return (-math.sin(o), -math.cos(o))

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        side = min(self.width(), self.height())
        cx, cy = self.width() / 2, self.height() / 2
        r = side / 2 - 14
        # 外圈刻度
        p.setPen(QPen(QColor(150, 150, 150), 1))
        for deg in range(0, 360, 10):
            a = math.radians(deg)
            ex, ey = math.sin(a), -math.cos(a)     # 屏幕刻度环（0=上）
            p.drawLine(int(cx + ex * r), int(cy + ey * r),
                       int(cx + ex * (r - 6)), int(cy + ey * (r - 6)))
        # 盘面
        p.setPen(QPen(QColor(120, 120, 120), 2))
        p.setBrush(QColor(200, 200, 200))
        p.drawEllipse(int(cx - r * 0.72), int(cy - r * 0.72), int(r * 1.44), int(r * 1.44))
        p.setBrush(QColor(225, 225, 225))
        p.setPen(Qt.NoPen)
        p.drawEllipse(int(cx - r * 0.42), int(cy - r * 0.42), int(r * 0.84), int(r * 0.84))
        # 方位标签
        p.setPen(QPen(QColor(0, 0, 0), 1))
        f = QFont(); f.setBold(True); p.setFont(f)
        p.drawText(int(cx - 20), int(cy - r - 2), 40, 16, Qt.AlignCenter, "North")
        p.drawText(int(cx - 20), int(cy + r - 12), 40, 16, Qt.AlignCenter, "South")
        p.drawText(int(cx + r - 6), int(cy - 8), 40, 16, Qt.AlignLeft, "East")
        p.drawText(int(cx - r - 34), int(cy - 8), 40, 16, Qt.AlignRight, "West")
        # 指针（行进方向）
        ex, ey = self._needle()
        p.setPen(QPen(QColor(200, 30, 30), 3))
        p.drawLine(int(cx), int(cy), int(cx + ex * r * 0.66), int(cy - ey * r * 0.66))
        p.setBrush(QColor(200, 30, 30))
        p.drawEllipse(int(cx - 4), int(cy - 4), 8, 8)

    def _set_from_pos(self, pos):
        cx, cy = self.width() / 2, self.height() / 2
        ex, ey = pos.x() - cx, -(pos.y() - cy)      # 数学坐标
        if abs(ex) < 1e-6 and abs(ey) < 1e-6:
            return
        # (ex,ey)=(-sinθ,-cosθ) → θ=atan2(-ex,-ey)
        theta = math.degrees(math.atan2(-ex, -ey)) % 360
        self.setValue(int(round(theta)))

    def mousePressEvent(self, ev):
        self._set_from_pos(ev.position())

    def mouseMoveEvent(self, ev):
        if ev.buttons() & Qt.LeftButton:
            self._set_from_pos(ev.position())


class Switch(QWidget):
    """开始仿真拨动开关（关闭/开启）。toggled_sig(bool)。"""
    toggled_sig = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._on = False
        self.setFixedSize(52, 26)

    def value(self):
        return self._on

    def setValue(self, on):
        if on != self._on:
            self._on = bool(on)
            self.toggled_sig.emit(self._on)
            self.update()

    def mousePressEvent(self, ev):
        self.setValue(not self._on)

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        col = QColor(70, 160, 80) if self._on else QColor(160, 160, 160)
        p.setPen(Qt.NoPen); p.setBrush(col)
        p.drawRoundedRect(0, 0, self.width(), self.height(), 13, 13)
        p.setBrush(QColor(240, 240, 240))
        kx = self.width() - 22 if self._on else 2
        p.drawEllipse(kx, 3, 20, 20)


# ======================================================================
# 主窗口
# ======================================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SwarmLab 迁移版 · 群体故障仿真（point_mass）")
        self.controller = None
        self.running = False
        self._view_tick = 0

        self.timer = QTimer(self)
        self.timer.setInterval(40)
        self.timer.timeout.connect(self._tick)

        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        # ---------------- 控制面板（复刻原版两列 + 底部开关）----------------
        panel = QWidget()
        panel.setFixedWidth(520)
        pv = QVBoxLayout(panel)
        pv.setContentsMargins(14, 14, 14, 14)
        pv.setSpacing(12)
        cols = QHBoxLayout()
        cols.setSpacing(28)
        pv.addLayout(cols)
        cols.addWidget(self._build_left_column())
        cols.addWidget(self._build_right_column())
        # 底部：开始仿真
        pv.addStretch()
        pv.addWidget(self._center_label("开始仿真", bold=True))
        tog = QHBoxLayout()
        tog.addStretch()
        tog.addWidget(QLabel("关闭"))
        self.start_switch = Switch()
        self.start_switch.toggled_sig.connect(self._on_start_toggled)
        tog.addWidget(self.start_switch)
        tog.addWidget(QLabel("开启"))
        tog.addStretch()
        pv.addLayout(tog)
        root.addWidget(panel)

        # ---------------- 3D 视图 + 调试视图 ----------------
        view = QWidget()
        vv = QVBoxLayout(view)
        vv.setContentsMargins(10, 10, 10, 10)
        vv.setSpacing(10)
        vv.addWidget(self._build_view_controls())
        self.time_label = QLabel("Simulation time: 0.0 seconds")
        self.time_label.setAlignment(Qt.AlignCenter)
        vv.addWidget(self.time_label)
        self.use_pv = HAVE_PYVISTA
        if self.use_pv:
            self.pview = QtInteractor(parent=view)
            vv.addWidget(self.pview, stretch=3)
            self.pview.set_background("white")
            # 地形交互风格：自由旋转时锁定世界 Z 为 up，地面保持水平（类似 matplotlib 3D），不滚转斜置
            self.pview.enable_terrain_style()
            self.fig = None
            self.canvas = None
        else:
            self.pview = None
            self.fig = Figure(figsize=(7, 6))
            self.canvas = FigureCanvasQTAgg(self.fig)
            self.canvas.mpl_connect("scroll_event", self._on_scroll)   # 滚轮缩放
            self.canvas.mpl_connect("button_press_event", self._on_press)
            self.canvas.mpl_connect("motion_notify_event", self._on_move)
            self.canvas.mpl_connect("button_release_event", self._on_release)
            vv.addWidget(self.canvas, stretch=3)
        self.debug_fig = Figure(figsize=(7, 2))
        self.debug_canvas = FigureCanvasQTAgg(self.debug_fig)
        self.debug_canvas.setVisible(False)
        self.debug_checkbox.toggled.connect(self.debug_canvas.setVisible)
        vv.addWidget(self.debug_canvas, stretch=1)
        root.addWidget(view, stretch=1)

        self._scatters = []
        self.ax = None
        self._elev = 30.0          # 固定俯仰（不随拖拽上下翻转，仅预设/重置可改）
        self._azim = -60.0         # 水平方位（左键拖拽仅改它）
        self._dragging = False
        self._drag_x = None
        self._drag_y = None
        self._pv_bld = None
        self._pv_drone_actors = {}
        self._pv_trail_actors = {}
        self._pv_tick = 0
        self.follow_swarm = True
        self.resize(1500, 900)

    # ------------------------------------------------------------------
    def _center_label(self, text, bold=False):
        lb = QLabel(text)
        lb.setAlignment(Qt.AlignCenter)
        if bold:
            f = lb.font(); f.setBold(True); f.setPointSize(11); lb.setFont(f)
        return lb

    def _slider(self, lo, hi, val):
        s = QSlider(Qt.Horizontal)
        s.setRange(lo, hi); s.setValue(val)
        s.setTickPosition(QSlider.TicksBelow); s.setTickInterval(max(1, (hi - lo) // 10))
        s.setMinimumHeight(26)
        return s

    def _build_left_column(self):
        w = QWidget(); v = QVBoxLayout(w)
        v.setContentsMargins(4, 4, 4, 4); v.setSpacing(10)
        v.addWidget(self._center_label("类型选择", bold=True))

        g = QGroupBox("无人机类型"); gv = QVBoxLayout(g)
        self.type_pm = QRadioButton("质点型"); self.type_pm.setChecked(True)
        gv.addWidget(self.type_pm)
        v.addWidget(g)

        g = QGroupBox("群体分布式优化算法"); gv = QVBoxLayout(g)
        self.alg_vas = QRadioButton("Vasarhelyi"); self.alg_vas.setChecked(True)
        self.alg_olf = QRadioButton("Olfati-Saber")
        gv.addWidget(self.alg_vas); gv.addWidget(self.alg_olf)
        v.addWidget(g)

        g = QGroupBox("无人机状态"); gv = QVBoxLayout(g)
        self.st_normal = QRadioButton("正常"); self.st_normal.setChecked(True)
        self.st_stall = QRadioButton("动力锁死")
        self.st_drift = QRadioButton("失控漂移")
        gv.addWidget(self.st_normal); gv.addWidget(self.st_stall); gv.addWidget(self.st_drift)
        v.addWidget(g)

        v.addWidget(self._center_label("无人机失控比例设置", bold=True))
        _lr = QHBoxLayout(); _lr.setSpacing(6)
        _lr.addWidget(QLabel("0"))
        self.loss_slider = self._slider(0, 60, 20)
        _lr.addWidget(self.loss_slider, stretch=1)
        _lr.addWidget(QLabel("60"))
        v.addLayout(_lr)
        self.loss_label = self._center_label("20 %")
        self.loss_slider.valueChanged.connect(lambda v_: self.loss_label.setText(f"{v_} %"))
        v.addWidget(self.loss_label)

        v.addWidget(self._center_label("地图设置", bold=True))
        self.map_checkbox = QCheckBox("显示地图障碍"); self.map_checkbox.setChecked(True)
        v.addWidget(self.map_checkbox)

        v.addWidget(self._center_label("调试设置", bold=True))
        self.debug_checkbox = QCheckBox("绘制调试视图")
        v.addWidget(self.debug_checkbox)
        v.addStretch()
        return w

    def _build_right_column(self):
        w = QWidget(); v = QVBoxLayout(w)
        v.setContentsMargins(4, 4, 4, 4); v.setSpacing(10)
        v.addWidget(self._center_label("集群参数设置", bold=True))

        v.addWidget(self._center_label("无人机数量设置", bold=True))
        row = QHBoxLayout(); row.addStretch()
        row.addWidget(QLabel("无人机的数量"))
        self.nb_spin = QSpinBox(); self.nb_spin.setRange(1, 30); self.nb_spin.setValue(3)
        row.addWidget(self.nb_spin); row.addStretch()
        v.addLayout(row)

        v.addWidget(self._center_label("无人机间距设置 [m]", bold=True))
        _dr = QHBoxLayout(); _dr.setSpacing(6)
        _dr.addWidget(QLabel("5"))
        self.dref_slider = self._slider(5, 15, 10)
        _dr.addWidget(self.dref_slider, stretch=1)
        _dr.addWidget(QLabel("15"))
        v.addLayout(_dr)
        self.dref_label = self._center_label("10 m")
        self.dref_slider.valueChanged.connect(lambda v_: self.dref_label.setText(f"{v_} m"))
        v.addWidget(self.dref_label)

        v.addWidget(self._center_label("无人机速度设置 [m/s]", bold=True))
        _vr = QHBoxLayout(); _vr.setSpacing(6)
        _vr.addWidget(QLabel("2"))
        self.vref_slider = self._slider(2, 8, 5)
        _vr.addWidget(self.vref_slider, stretch=1)
        _vr.addWidget(QLabel("8"))
        v.addLayout(_vr)
        self.vref_label = self._center_label("5 m/s")
        self.vref_slider.valueChanged.connect(lambda v_: self.vref_label.setText(f"{v_} m/s"))
        v.addWidget(self.vref_label)

        v.addWidget(self._center_label("仿真参数设置", bold=True))
        row = QHBoxLayout(); row.addStretch()
        row.addWidget(QLabel("仿真时长 [s]"))
        self.end_spin = QSpinBox(); self.end_spin.setRange(1, 3600); self.end_spin.setValue(100)
        row.addWidget(self.end_spin); row.addStretch()
        v.addLayout(row)

        v.addWidget(self._center_label("前进方向 (180=N)", bold=True))
        self.compass = CompassWidget(value=180)
        v.addWidget(self.compass)
        v.addStretch()
        return w

    # ------------------------------------------------------------------
    def _set_at_start_widgets_enabled(self, on):
        for wgt in (self.nb_spin, self.alg_vas, self.alg_olf, self.map_checkbox, self.end_spin):
            wgt.setEnabled(on)

    def _on_start_toggled(self, on):
        if on:
            algorithm = "vasarhelyi" if self.alg_vas.isChecked() else "olfati_saber"
            self.controller = SimController(
                drone_type="point_mass", algorithm=algorithm,
                nb_agents=int(self.nb_spin.value()),
                active_environment=bool(self.map_checkbox.isChecked()),
                end_time=float(self.end_spin.value()))
            self._init_view()
            self.running = True
            self._set_at_start_widgets_enabled(False)
            self.timer.start()
        else:
            self.running = False
            self.timer.stop()
            self._set_at_start_widgets_enabled(True)

    def _current_state(self):
        if self.st_stall.isChecked():
            return "Stalled"
        if self.st_drift.isChecked():
            return "Drifting"
        return "Normal"

    # ------------------------------------------------------------------
    def _tick(self):
        c = self.controller
        if c is None or not self.running:
            return
        if c.done:
            self.running = False
            self.timer.stop()
            self.start_switch.setValue(False)
            self._set_at_start_widgets_enabled(True)
            self._update_view()
            return
        # 实时参数 → 每步生效
        c.apply_live(self.compass.value(), self.dref_slider.value(), self.vref_slider.value())
        c.set_state(self._current_state(), self.loss_slider.value() / 100.0)
        steps = max(1, int(round(0.04 / c.dt)))     # ≈ 实时
        for _ in range(steps):
            if not c.step():
                break
        self._view_tick += 1
        if self._view_tick % 2 == 0:                # 视图 ~12fps
            self._update_view()
            if self.debug_checkbox.isChecked():
                self._update_debug()

    # ------------------------------------------------------------------
    def _build_view_controls(self):
        """视角控制：预设/重置/聚焦集群 + 相机跟随；拖拽旋转、滚轮缩放、平移由渲染后端提供
        （pyvista=GPU 原生 OrbitControls；matplotlib 回退=限位轨道+滚轮缩放）。"""
        w = QWidget(); h = QHBoxLayout(w)
        h.setContentsMargins(0, 0, 0, 0); h.setSpacing(6)
        h.addWidget(QLabel("视角:"))
        for text, key in (("俯视", "top"), ("正视", "front"), ("侧视", "side"), ("等轴", "iso")):
            b = QPushButton(text)
            b.setFixedWidth(52)
            b.clicked.connect(lambda _=False, k=key: self._preset(k))
            h.addWidget(b)
        rst = QPushButton("重置")
        rst.setFixedWidth(52)
        rst.clicked.connect(lambda _=False: self._reset_view())
        h.addWidget(rst)
        foc = QPushButton("聚焦集群")
        foc.setFixedWidth(70)
        foc.clicked.connect(lambda _=False: self._focus_swarm())
        h.addWidget(foc)
        self.follow_checkbox = QCheckBox("相机跟随集群")
        self.follow_checkbox.setChecked(True)
        self.follow_checkbox.toggled.connect(self._set_follow)
        h.addWidget(self.follow_checkbox)
        h.addSpacing(8)
        h.addWidget(QLabel("拖拽=旋转  滚轮=缩放  平移=拖动集群入视野"))
        h.addStretch()
        return w

    def _set_view(self, elev, azim):
        self._elev = float(elev)
        self._azim = float(azim)
        if self.ax is None:
            return
        self._apply_limits()          # 预设/重置同时恢复默认取景
        self.ax.view_init(self._elev, self._azim)
        self.canvas.draw_idle()

    # ---- 限位轨道拖拽：水平 yaw 自由 + 俯仰 clamp[0,90]，XOY 不倒扣、可平滑俯视 ----
    def _on_press(self, event):
        if self.ax is None or event.inaxes is not self.ax or event.button != 1:
            return
        self._dragging = True
        self._drag_x = event.x
        self._drag_y = event.y

    def _on_move(self, event):
        if not self._dragging or self.ax is None or event.x is None or event.y is None:
            return
        dx = event.x - (self._drag_x if self._drag_x is not None else event.x)
        dy = event.y - (self._drag_y if self._drag_y is not None else event.y)
        self._drag_x, self._drag_y = event.x, event.y
        self._azim = (self._azim + dx * 0.5) % 360.0            # 水平自由旋转
        # 俯仰限位 [0,90]：可平滑拖到俯视(90)/平视(0)，但不会翻到地面以下或倒扣
        self._elev = min(90.0, max(0.0, self._elev + dy * 0.3))
        self.ax.view_init(self._elev, self._azim)
        self.canvas.draw_idle()

    def _on_release(self, event):
        self._dragging = False

    def _apply_limits(self):
        mp = self.controller.params.map
        if getattr(mp, "ACTIVE_ENVIRONMENT", False):
            set_map_axes(self.ax, mp)
        else:
            pos = self.controller.swarm.get_pos_ned()
            mn = pos.min(axis=1); mx = pos.max(axis=1)
            edge = max(float((mx - mn).max()), 40.0)
            ctr = (mn + mx) / 2.0
            self.ax.set_xlim(ctr[1] - edge, ctr[1] + edge)
            self.ax.set_ylim(ctr[0] - edge, ctr[0] + edge)
            self.ax.set_zlim(0, max(edge, 40.0))

    def _on_scroll(self, event):
        """滚轮缩放：以当前视野中心缩放三轴范围（放大看局部碰撞/避障，缩小看整体）。"""
        if self.ax is None or event.inaxes is not self.ax:
            return
        scale = 0.85 if event.button == "up" else 1.0 / 0.85
        for getter, setter in ((self.ax.get_xlim, self.ax.set_xlim),
                               (self.ax.get_ylim, self.ax.set_ylim),
                               (self.ax.get_zlim, self.ax.set_zlim)):
            lo, hi = getter()
            c = (lo + hi) / 2.0
            half = (hi - lo) / 2.0 * scale
            setter(c - half, c + half)
        self.canvas.draw_idle()

    def _init_view(self):
        if self.use_pv:
            self._pv_init()
            return
        self.fig.clear()
        self.ax = self.fig.add_subplot(111, projection="3d")
        # 禁用 matplotlib 原生“球式”旋转/平移；改用限位轨道拖拽(yaw+clamp俯仰)+滚轮缩放
        self.ax.set_navigate(False)
        mp = self.controller.params.map
        if getattr(mp, "ACTIVE_ENVIRONMENT", False):
            if mp.building_shape == "rectangle":
                draw_buildings(self.ax, mp)
            elif mp.building_shape == "cylinder":
                draw_cylinders(self.ax, mp)
        self._apply_limits()
        self.ax.view_init(self._elev, self._azim)
        self.ax.set_xlabel("Y position [m]"); self.ax.set_ylabel("X position [m]")
        self.ax.set_zlabel("Z position [m]")
        self._scatters = []
        self._trails = []

    def _update_view(self):
        c = self.controller
        if c is None:
            return
        self.time_label.setText(f"Simulation time: {c.time:.1f} seconds   状态 = {c.current_state}")
        if self.use_pv:
            self._pv_update()
            return
        if self.ax is None:
            return
        for art in self._scatters + self._trails:
            art.remove()
        self._scatters = []
        self._trails = []
        pos = c.swarm.get_pos_ned()                    # (3,N)
        x, y, z = pos[1, :], pos[0, :], -pos[2, :]     # ENU
        colors = c.swarm.get_colors()                  # (3,N) jet 每机一色
        n = pos.shape[1]
        hist = c.pos_history
        if len(hist) > 1:
            arr = np.asarray(hist)                     # (T,3,N)
            k = max(1, arr.shape[0] // 600)
            sub = arr[::k]
            for m in range(n):
                tr = sub[:, :, m]
                ln, = self.ax.plot(tr[:, 1], tr[:, 0], -tr[:, 2],
                                   color=colors[:, m], lw=0.8, alpha=0.8)
                self._trails.append(ln)
        states = [getattr(d.drone_state, "value", d.drone_state) for d in c.swarm.drones]
        for name, marker in _STATE_MARKER.items():
            idx = [i for i, s in enumerate(states) if s == name]
            if idx:
                sc = self.ax.scatter(x[idx], y[idx], z[idx], c=colors[:, idx].T,
                                     marker=marker, s=45, depthshade=False)
                self._scatters.append(sc)
        if self.follow_swarm:
            self._apply_limits_swarm()
        self.canvas.draw_idle()

    # ================= pyvista (GPU) 视图 =================
    def _pv_init(self):
        self.pview.clear()
        mp = self.controller.params.map
        if getattr(mp, "ACTIVE_ENVIRONMENT", False):
            bn = np.asarray(mp.buildings_north).ravel()
            be = np.asarray(mp.buildings_east).ravel()
            bh = np.asarray(mp.buildings_heights).ravel()
            cubes = [pv.Cube(center=(float(e), float(n), float(h) / 2),
                             x_length=float(mp.building_width),
                             y_length=float(mp.building_width),
                             z_length=float(h))
                     for n, e, h in zip(bn, be, bh)]
            if cubes:
                self._pv_bld = self.pview.add_mesh(pv.merge(cubes), color="lightgray",
                                                   opacity=0.5, show_edges=False)
        self._pv_drone_actors = {}
        self._pv_trail_actors = {}
        self._preset("iso")

    def _pv_update(self):
        c = self.controller
        pos = c.swarm.get_pos_ned()
        n = pos.shape[1]
        enu = np.vstack([pos[1, :], pos[0, :], -pos[2, :]]).T      # (N,3)
        colors = c.swarm.get_colors()
        states = [getattr(d.drone_state, "value", d.drone_state) for d in c.swarm.drones]
        for a in list(self._pv_drone_actors.values()):
            self.pview.remove_actor(a, reset_camera=False)
        self._pv_drone_actors = {}
        sphere_g, cube_g, pyr_g = _marker_geoms(DRONE_MARKER_SIZE)
        idx = [i for i, s in enumerate(states) if s == "Normal"]
        if idx:
            rgb = (colors[:, idx].T * 255).astype(np.uint8)
            pd = pv.PolyData(enu[idx]); pd.point_data["rgb"] = rgb
            g = pd.glyph(geom=sphere_g, scale=False, orient=False)
            self._pv_drone_actors["Normal"] = self.pview.add_mesh(
                g, scalars="rgb", rgb=True, reset_camera=False)
        idx = [i for i, s in enumerate(states) if s == "Stalled"]
        if idx:
            # 动力锁死 = 正方体
            g = pv.PolyData(enu[idx]).glyph(geom=cube_g, scale=False, orient=False)
            self._pv_drone_actors["Stalled"] = self.pview.add_mesh(
                g, color="#ff7f0e", reset_camera=False)
        idx = [i for i, s in enumerate(states) if s == "Drifting"]
        if idx:
            # 失控漂移 = 四棱锥
            g = pv.PolyData(enu[idx]).glyph(geom=pyr_g, scale=False, orient=False)
            self._pv_drone_actors["Drifting"] = self.pview.add_mesh(
                g, color="#d62728", reset_camera=False)
        # 尾迹（每 5 帧重建一次，降开销）
        self._pv_tick += 1
        if self._pv_tick % 5 == 0:
            for a in list(self._pv_trail_actors.values()):
                self.pview.remove_actor(a, reset_camera=False)
            self._pv_trail_actors = {}
            hist = c.pos_history
            if len(hist) > 1:
                arr = np.asarray(hist)
                k = max(1, arr.shape[0] // 400)
                sub = arr[::k]
                for m in range(n):
                    tr = sub[:, :, m]
                    et = np.vstack([tr[:, 1], tr[:, 0], -tr[:, 2]]).T
                    if len(et) < 2:
                        continue
                    lines = np.hstack([[2, i, i + 1] for i in range(len(et) - 1)])
                    self._pv_trail_actors[m] = self.pview.add_mesh(
                        pv.PolyData(et, lines=lines), color=colors[:, m],
                        line_width=1.5, reset_camera=False)
        if self.follow_swarm:
            self.pview.camera.focal_point = tuple(enu.mean(axis=0))
        self.pview.update()

    def _cam_set(self, d, up, parallel=False):
        """设置相机：焦点=集群质心，位置=质心+方向*距离，显式 view_up，避免地图斜置。
        parallel=True 时用正交投影（俯视=真二维平面，无透视倾角/放大）。"""
        c = self.controller
        pos = c.swarm.get_pos_ned()
        enu = np.vstack([pos[1, :], pos[0, :], -pos[2, :]]).T
        cent = enu.mean(axis=0)
        mp = c.params.map
        dist = mp.width * 0.9 if getattr(mp, "ACTIVE_ENVIRONMENT", False) else 120.0
        d = np.asarray(d, dtype=float); d = d / np.linalg.norm(d)
        cam = self.pview.camera
        cam.focal_point = tuple(cent)
        cam.position = tuple(cent + d * dist)
        cam.view_up = tuple(up)
        if parallel:
            self.pview.enable_parallel_projection()
            cam.parallel_scale = mp.width * 0.55 if getattr(mp, "ACTIVE_ENVIRONMENT", False) else 60.0
        else:
            self.pview.disable_parallel_projection()
        self.pview.reset_camera_clipping_range()
        self.pview.update()

    def _preset(self, key):
        if self.use_pv:
            if self.pview is None:
                return
            # 方向/up 选取保证建筑网格在画面中“横平竖直”，不斜置
            views = {"top": ((0, 0, 1), (0, 1, 0)),
                     "front": ((0, -1, 0.2), (0, 0, 1)),
                     "side": ((1, 0, 0.2), (0, 0, 1)),
                     "iso": ((0, -1, 0.7), (0, 0, 1))}
            d, up = views[key]
            self._cam_set(d, up, parallel=(key == "top"))
            return
        self._set_view(*{"top": (90, -90), "front": (0, -90),
                         "side": (0, 0), "iso": (30, -60)}[key])

    def _reset_view(self):
        if self.use_pv:
            self.follow_checkbox.blockSignals(True)
            self.follow_checkbox.setChecked(False)
            self.follow_checkbox.blockSignals(False)
            self.follow_swarm = False
            self._preset("iso")
            return
        self._set_view(30, -60)

    def _focus_swarm(self):
        c = self.controller
        if c is None:
            return
        pos = c.swarm.get_pos_ned()
        enu = np.vstack([pos[1, :], pos[0, :], -pos[2, :]]).T
        cent = enu.mean(axis=0)
        if self.use_pv:
            rad = max(float(np.linalg.norm(enu - cent, axis=1).max()), 5.0) * 2.5
            direc = np.asarray(self.pview.camera.direction, dtype=float)
            self.pview.camera.focal_point = tuple(cent)
            self.pview.camera.position = tuple(cent - direc * rad)
            self.pview.reset_camera_clipping_range()
            self.pview.update()
            return
        self._apply_limits_swarm()
        self.canvas.draw_idle()

    def _set_follow(self, on):
        self.follow_swarm = bool(on)

    def _apply_limits_swarm(self):
        pos = self.controller.swarm.get_pos_ned()
        enu = np.vstack([pos[1, :], pos[0, :], -pos[2, :]])
        mn = enu.min(axis=1); mx = enu.max(axis=1)
        edge = max(float((mx - mn).max()) / 2 * 1.4, 10.0)
        ctr = (mn + mx) / 2
        self.ax.set_xlim(ctr[0] - edge, ctr[0] + edge)
        self.ax.set_ylim(ctr[1] - edge, ctr[1] + edge)
        self.ax.set_zlim(max(ctr[2] - edge, 0.0), ctr[2] + edge)

    def _update_debug(self):
        c = self.controller
        self.debug_fig.clear()
        ax = self.debug_fig.add_subplot(111)
        t = np.arange(len(c.min_dist_history)) * c.dt
        ax.plot(t, c.min_dist_history, lw=1.2, label="min inter-drone dist")
        ch = c.swarm.collisions_history
        if len(ch) > 0:
            ax.plot(np.arange(len(ch)) * c.dt, ch[:, 0], lw=1.0, label="agent-agent coll")
        ax.set_xlabel("time [s]"); ax.legend(fontsize=8); ax.grid(True)
        self.debug_canvas.draw_idle()

    def closeEvent(self, ev):
        self.timer.stop()
        super().closeEvent(ev)


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
