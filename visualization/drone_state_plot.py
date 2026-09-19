# -*- coding: utf-8 -*-
"""DroneStatePlotter —— 单机状态时序曲线（对应 MATLAB @Drone/plot_state.m）。

7 组（Position/Velocity/Acceleration/Air data/Attitude/Angle rates/Actuators），每变量画
real(蓝实线)/estimated(绿虚)/command(红点划)。仅 quad/wing（point_mass 跳过，同 MATLAB）。
MATLAB uitab → matplotlib 单 figure GridSpec（每组一行）；persistent 图形句柄 → Line2D 实例。
z_hat 用 canonical 布局（B-11 修复后与 true_states 一致：phi@9,theta@10,psi@11,chi@12,p@13,q@14,r@15）。
"""
import math

import numpy as np
import matplotlib.pyplot as plt

from ..math_tools import Rb2i

_R2D = 180.0 / math.pi


class DroneStatePlotter:
    """单机状态曲线查看器（在线追加）。"""

    def __init__(self, drone_type, output_rate=0.1, figsize=(11, 15)):
        self.drone_type = str(drone_type)
        self.output_rate = output_rate
        self.figsize = figsize
        self.fig = None
        self._built = False
        self._lines = {}          # (gi, si, series) -> Line2D
        self._t = []
        self._data = {}           # key -> list[float]
        self._time_of_last = -1e9
        self._groups = self._define_groups()

    # ------------------------------------------------------------------
    def _define_groups(self):
        """按 drone_type 组装分组：group=(title, [sub])，sub=(ylabel, real, est|None, cmd|None)。"""
        is_wing = self.drone_type == "fixed_wing"
        att3 = ("\\chi [deg]", "chi", "chi_hat", "chi_c") if is_wing \
            else ("\\psi [deg]", "psi", "psi_hat", "psi_c")
        if is_wing:
            act = [("\\delta_e [deg]", "de", None, None), ("\\delta_a [deg]", "da", None, None),
                   ("\\delta_r [deg]", "dr", None, None), ("\\delta_t [0..1]", "dt", None, None)]
        else:
            act = [("\\delta_T%d [0..1]" % (i + 1), "d%d" % (i + 1), None, None) for i in range(4)]
        return [
            ("Position", [("p_n [m]", "pn", "pn_hat", "pn_c"),
                          ("p_e [m]", "pe", "pe_hat", "pe_c"),
                          ("h [m]", "h", "h_hat", "h_c")]),
            ("Velocity", [("v_n [m/s]", "vn", None, "vn_c"),
                          ("v_e [m/s]", "ve", None, "ve_c"),
                          ("v_h [m/s]", "vh", None, "vh_c")]),
            ("Acceleration", [("a_n [m/s²]", "an", None, "an_c"),
                              ("a_e [m/s²]", "ae", None, "ae_c"),
                              ("a_d [m/s²]", "ad", None, "ad_c")]),
            ("Air data", [("V_a [m/s]", "Va", "Va_hat", "Va_c"),
                          ("\\alpha [deg]", "alpha", "alpha_hat", "alpha_c"),
                          ("\\beta [deg]", "beta", "beta_hat", "beta_c")]),
            ("Attitude", [("\\phi [deg]", "phi", "phi_hat", "phi_c"),
                          ("\\theta [deg]", "theta", "theta_hat", "theta_c"), att3]),
            ("Angle rates", [("p [deg/s]", "p", "p_hat", "p_c"),
                             ("q [deg/s]", "q", "q_hat", "q_c"),
                             ("r [deg/s]", "r", "r_hat", "r_c")]),
            ("Actuators", act),
        ]

    def _build(self):
        from matplotlib.gridspec import GridSpec
        self.fig = plt.figure(figsize=self.figsize)
        self.fig.suptitle("Drone state viewer (%s)" % self.drone_type)
        gs = GridSpec(len(self._groups), 4, figure=self.fig, hspace=0.5, wspace=0.35)
        for gi, (title, subs) in enumerate(self._groups):
            for si in range(4):
                ax = self.fig.add_subplot(gs[gi, si])
                if si >= len(subs):
                    ax.axis("off")
                    continue
                ylabel, real, est, cmd = subs[si]
                ax.set_ylabel(ylabel, fontsize=8)
                ax.grid(True)
                ax.tick_params(labelsize=7)
                if si == 0:
                    ax.set_title(title, fontsize=9, loc="left")
                ln, = ax.plot([], [], "b", lw=1.0)
                self._lines[(gi, si, "real")] = (ln, real)
                self._data.setdefault(real, [])
                if est is not None:
                    le, = ax.plot([], [], "g--", lw=1.0)
                    self._lines[(gi, si, "est")] = (le, est)
                    self._data.setdefault(est, [])
                if cmd is not None:
                    lc, = ax.plot([], [], "r-.", lw=1.0)
                    self._lines[(gi, si, "cmd")] = (lc, cmd)
                    self._data.setdefault(cmd, [])
        self._built = True

    # ------------------------------------------------------------------
    def _extract(self, drone):
        """从 drone 提取当前所有量（复刻 plot_state.m 的 rename + Rb2i 计算），返回 dict。"""
        d = {}
        pn, pe, pd = drone.pos_ned
        vx, vy, vz = drone.vel_xyz
        phi, theta, psi = drone.attitude
        d["pn"] = pn; d["pe"] = pe; d["h"] = -pd
        d["phi"] = phi * _R2D; d["theta"] = theta * _R2D; d["psi"] = psi * _R2D
        d["p"] = drone.rates[0] * _R2D; d["q"] = drone.rates[1] * _R2D; d["r"] = drone.rates[2] * _R2D

        ad = np.asarray(drone.airdata, float)
        Va, wn, we = ad[0], ad[3], ad[4]
        d["Va"] = Va; d["alpha"] = ad[1] * _R2D; d["beta"] = ad[2] * _R2D
        d["chi"] = math.atan2(Va * math.sin(psi) + we, Va * math.cos(psi) + wn) * _R2D

        Rbi = Rb2i(phi, theta, psi)
        v_ned = Rbi @ np.array([vx, vy, vz])
        d["vn"] = v_ned[0]; d["ve"] = v_ned[1]; d["vh"] = -v_ned[2]
        a_ned = Rbi @ np.asarray(drone.forces, float) / drone.p_drone.mass
        d["an"] = a_ned[0]; d["ae"] = a_ned[1]; d["ad"] = a_ned[2]

        # full_command（19，0-based）
        fc = np.asarray(drone.full_command, float)
        fc = np.concatenate([fc, np.zeros(max(0, 19 - fc.size))]) if fc.size < 19 else fc
        d["pn_c"] = fc[0]; d["pe_c"] = fc[1]; d["h_c"] = fc[2]
        d["vn_c"] = fc[3]; d["ve_c"] = fc[4]; d["vh_c"] = -fc[5]
        d["an_c"] = fc[6]; d["ae_c"] = fc[7]; d["ad_c"] = fc[8]
        d["Va_c"] = fc[9]; d["alpha_c"] = fc[10] * _R2D; d["beta_c"] = fc[11] * _R2D
        d["phi_c"] = fc[12] * _R2D; d["theta_c"] = fc[13] * _R2D
        d["psi_c"] = fc[14] * _R2D; d["chi_c"] = fc[15] * _R2D
        d["p_c"] = fc[16] * _R2D; d["q_c"] = fc[17] * _R2D; d["r_c"] = fc[18] * _R2D

        # z_hat（canonical 布局，0-based）
        z = np.asarray(drone.z_hat, float)
        d["pn_hat"] = z[0]; d["pe_hat"] = z[1]; d["h_hat"] = z[2]
        d["Va_hat"] = z[6]; d["alpha_hat"] = z[7] * _R2D; d["beta_hat"] = z[8] * _R2D
        d["phi_hat"] = z[9] * _R2D; d["theta_hat"] = z[10] * _R2D
        d["psi_hat"] = z[11] * _R2D; d["chi_hat"] = z[12] * _R2D
        d["p_hat"] = z[13] * _R2D; d["q_hat"] = z[14] * _R2D; d["r_hat"] = z[15] * _R2D

        # delta
        dl = np.asarray(drone.delta, float)
        if self.drone_type == "fixed_wing":
            d["de"] = dl[0] * _R2D; d["da"] = dl[1] * _R2D; d["dr"] = dl[2] * _R2D; d["dt"] = dl[3]
        else:
            for i in range(4):
                d["d%d" % (i + 1)] = dl[i]
        return d

    # ------------------------------------------------------------------
    def update(self, time, drone):
        """追加当前状态并刷新（节流到 output_rate）。对应 plot_state(time, output_rate)。"""
        if self.drone_type == "point_mass":
            return
        if (time - self._time_of_last) < self.output_rate - 1e-9:
            return
        self._time_of_last = time
        if not self._built:
            self._build()
        vals = self._extract(drone)
        self._t.append(time)
        for key in self._data:
            self._data[key].append(vals.get(key, 0.0))
        for (gi, si, series), (ln, key) in self._lines.items():
            ln.set_data(self._t, self._data[key])
            ax = ln.axes
            ax.relim(); ax.autoscale_view()
        self.fig.canvas.draw_idle()

    def save(self, path, dpi=90):
        if self._built:
            self.fig.savefig(path, dpi=dpi, bbox_inches="tight")

    def close(self):
        if self.fig is not None:
            plt.close(self.fig)
            self.fig = None
            self._built = False
