# -*- coding: utf-8 -*-
"""SwarmLab 迁移版 GUI —— Streamlit + Plotly（Stage 3；对应 MATLAB GUI_swarm.mlapp，计划 Phase 13）。

严格对照 MIGRATION_LOG.md 附录 A 的接口与导入约定：
  - A.0 导入：入口把 _REPO_ROOT(=c:\\SwarmLab) 加入 sys.path，用 ``from SwarmLab_py.xxx import ...``；
  - A.2 签名：Simulation/FaultEvent、compute_swarm_performance、plot_trajectories/plot_performance；
  - A.3 易错点：建筑取自 ``sim.params.map``(MapParams)，无 get_buildings；历史用 get_pos_history()；
    state_history 是 List[str]；碰撞在 swarm.collisions_history / sim.collisions_sum。
坐标 NED→ENU：x=east(pe)、y=north(pn)、z=up(-pd)；故障 marker ●/■/▲ 对应 SwarmViewer 的 ○/□/△。

当前物理为 point_mass（运动学）；真实动力学见 Phase 8 / Stage 4。

启动：
    C:\\Python\\python.exe -m streamlit run SwarmLab_py/gui/app.py
"""
import os
import sys
import io

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import numpy as np                              # noqa: E402
import matplotlib                               # noqa: E402
matplotlib.use("Agg")                           # headless，先于 pyplot
import matplotlib.pyplot as plt                 # noqa: E402
import streamlit as st                          # noqa: E402
import plotly.graph_objects as go               # noqa: E402

from SwarmLab_py.simulation import Simulation, FaultEvent                  # noqa: E402
from SwarmLab_py.visualization import plot_trajectories, plot_performance  # noqa: E402
from SwarmLab_py.math_tools import compute_swarm_performance               # noqa: E402

st.set_page_config(page_title="SwarmLab 迁移版", layout="wide")

# 故障状态 -> (plotly 3D marker symbol, color)，对应 SwarmViewer 的 ○/□/△
_STATE_STYLE = {
    "Normal":   ("circle",      "#1f77b4"),
    "Stalled":  ("square",      "#ff7f0e"),
    "Drifting": ("triangle-up", "#d62728"),
}


# ---------------------------------------------------------------------- helpers
def _building_meshes(map_params):
    """MapParams 建筑 -> Plotly Mesh3d（镜像 visualization.map_drawing._building_boxes 的几何）。"""
    if not getattr(map_params, "ACTIVE_ENVIRONMENT", False):
        return []
    bn = np.asarray(map_params.buildings_north).ravel()
    be = np.asarray(map_params.buildings_east).ravel()
    bh = np.asarray(map_params.buildings_heights).ravel()
    d = map_params.building_width / 2.0
    i = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2]
    j = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3]
    k = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6]
    meshes = []
    for n, e, h in zip(bn, be, bh):
        xs = [e - d, e + d, e + d, e - d, e - d, e + d, e + d, e - d]
        ys = [n - d, n - d, n + d, n + d, n - d, n - d, n + d, n + d]
        zs = [0.0, 0.0, 0.0, 0.0, h, h, h, h]
        meshes.append(go.Mesh3d(x=xs, y=ys, z=zs, i=i, j=j, k=k, opacity=0.25,
                                color="lightgray", showlegend=False, hoverinfo="skip"))
    return meshes


def _scene(fig, map_params, title=""):
    """3D 场景：有环境时按地图范围定轴（镜像 set_map_axes），否则数据自适应。"""
    if getattr(map_params, "ACTIVE_ENVIRONMENT", False):
        w = map_params.width
        m = w / 5.0
        scene = dict(aspectmode="manual", aspectratio=dict(x=1, y=1, z=0.45),
                     xaxis=dict(title="East [m]", range=[-m, w + m]),
                     yaxis=dict(title="North [m]", range=[-m, w + m]),
                     zaxis=dict(title="Up [m]", range=[0, 1.2 * map_params.max_height]))
    else:
        scene = dict(aspectmode="data", xaxis_title="East [m]",
                     yaxis_title="North [m]", zaxis_title="Up [m]")
    fig.update_layout(title=title, margin=dict(l=0, r=0, t=40, b=0), scene=scene)
    return fig


def _fig_to_png(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=120, bbox_inches="tight")
    plt.close(fig)
    return buf.getvalue()


# ---------------------------------------------------------------------- 侧边栏
st.sidebar.header("仿真参数")
algorithm = st.sidebar.selectbox("算法 algorithm", ["olfati_saber", "vasarhelyi"], key="algorithm")
nb_agents = st.sidebar.slider("无人机数量 nb_agents", 3, 30, 10, key="nb_agents")
d_ref = st.sidebar.number_input("期望间距 d_ref (m)", value=25.0, step=1.0, key="d_ref")
v_ref = st.sidebar.number_input("期望速度 v_ref (m/s)", value=6.0, step=0.5, key="v_ref")
loss_radio = st.sidebar.slider("失控比例 loss_radio", 0.0, 1.0, 0.3, 0.05, key="loss_radio")
orientation = st.sidebar.slider("编队朝向 orientation (°)", 0, 360, 0, key="orientation")
end_time = st.sidebar.number_input("仿真时长 end_time (s)", value=60.0, step=5.0, key="end_time")
seed = st.sidebar.number_input("随机种子 seed", value=0, step=1, key="seed")
active_env = st.sidebar.checkbox("启用建筑环境 active_environment", value=True, key="active_env")

st.sidebar.subheader("故障时间表 (FaultEvent)")
stall_on = st.sidebar.checkbox("注入 Stalled（动力锁死）", value=True, key="stall_on")
stall_t = st.sidebar.number_input("Stalled 时刻 (s)", value=15.0, key="stall_t", disabled=not stall_on)
drift_on = st.sidebar.checkbox("注入 Drifting（失控漂移）", value=True, key="drift_on")
drift_t = st.sidebar.number_input("Drifting 时刻 (s)", value=30.0, key="drift_t", disabled=not drift_on)
rec_on = st.sidebar.checkbox("恢复 Normal", value=True, key="rec_on")
rec_t = st.sidebar.number_input("Normal 时刻 (s)", value=50.0, key="rec_t", disabled=not rec_on)

run_clicked = st.sidebar.button("▶ 运行仿真", key="run", type="primary")

# ---------------------------------------------------------------------- 运行
if run_clicked:
    fault = []
    if stall_on:
        fault.append(FaultEvent(float(stall_t), "Stalled"))
    if drift_on:
        fault.append(FaultEvent(float(drift_t), "Drifting"))
    if rec_on:
        fault.append(FaultEvent(float(rec_t), "Normal"))

    sim = Simulation(
        drone_type="point_mass", algorithm=algorithm,
        nb_agents=int(nb_agents), d_ref=float(d_ref), v_ref=float(v_ref),
        loss_radio=float(loss_radio), orientation_deg=int(orientation),
        active_environment=bool(active_env), end_time=float(end_time),
        seed=int(seed), fault_schedule=fault)

    frame_states = []

    def _record(s):
        # 逐帧记录每架无人机状态（drone_state.value → "Normal"/"Stalled"/"Drifting"）
        frame_states.append([getattr(d.drone_state, "value", d.drone_state) for d in s.swarm.drones])

    with st.spinner("仿真运行中..."):
        sim.run(on_step=_record)

        # 对齐历史：pos 去掉 set_pos 首帧（与 time_history / frame_states 对齐）
        pos = sim.get_pos_history()[1:]
        vel = sim.get_vel_history()
        n = min(pos.shape[0], vel.shape[0], len(sim.time_history), len(frame_states))
        pos, vel = pos[:n], vel[:n]
        fs = np.array(frame_states[:n])
        th = np.array(sim.time_history[:n])

        # 性能指标（子采样至 ~200 帧，避免逐帧过慢）
        step = max(1, n // 200)
        idx = np.arange(0, n, step)
        metrics = compute_swarm_performance(pos[idx], vel[idx], sim.p_swarm)
        ch = sim.swarm.collisions_history[:n][idx]

        # 复用已迁移的 matplotlib 出图，一次性渲染为 PNG 字节缓存（不随滑杆重算）
        traj_png = _fig_to_png(plot_trajectories(pos, sim.p_swarm.nb_agents,
                                                 sim.swarm.get_colors(), sim.params.map))
        perf_pngs = {key: _fig_to_png(f) for key, f in
                     plot_performance(th[idx], *metrics, collisions_history=ch).items()}

    st.session_state.update(dict(
        sim=sim, pos=pos, fs=fs, th=th, n=n,
        traj_png=traj_png, perf_pngs=perf_pngs))

# ---------------------------------------------------------------------- 主体
st.title("SwarmLab 迁移版 · 故障群仿真 GUI")
st.caption("point_mass 运动学；坐标 ENU（x=East, y=North, z=Up=-pd）；真实动力学见 Phase 8 / Stage 4。")

if "sim" not in st.session_state:
    st.info("👈 在左侧设置参数并点击『运行仿真』。")
    st.stop()

sim = st.session_state["sim"]
pos = st.session_state["pos"]
fs = st.session_state["fs"]
th = st.session_state["th"]
n = st.session_state["n"]
N = sim.p_swarm.nb_agents

st.markdown("### 概览")
c = st.columns(6)
c[0].metric("无人机数", N)
c[1].metric("仿真步数", n)
c[2].metric("碰撞 机-机", int(round(sim.collisions_sum[0])))
c[3].metric("碰撞 机-障", int(round(sim.collisions_sum[1])))
c[4].metric("圆柱障碍 n_cyl", sim.p_swarm.n_cyl)
c[5].metric("末帧 Normal", int((fs[-1] == "Normal").sum()) if n > 0 else 0)

tab3d, tabtraj, tabperf, tabcoll = st.tabs(["3D 回放", "轨迹", "性能指标", "碰撞"])

with tab3d:
    if n == 0:
        st.warning("无帧数据。")
    else:
        sel = np.linspace(0, n - 1, min(n, 250)).astype(int)
        kk = st.slider("帧（回放）", 0, len(sel) - 1, 0, key="frame_slider")
        fi = int(sel[kk])
        pk = pos[fi].reshape(N, 3)                 # 每行 [pn, pe, pd]
        x, y, z = pk[:, 1], pk[:, 0], -pk[:, 2]    # ENU
        states = fs[fi]
        data = _building_meshes(sim.params.map)
        for name, (sym, col) in _STATE_STYLE.items():
            m = [q for q in range(N) if states[q] == name]
            if m:
                data.append(go.Scatter3d(x=x[m], y=y[m], z=z[m], mode="markers",
                                         marker=dict(size=5, symbol=sym, color=col),
                                         name=name, text=[f"drone {q}" for q in m]))
        fig = go.Figure(data=data)
        _scene(fig, sim.params.map,
               f"t = {th[fi]:.2f} s　指令状态 = {sim.state_history[fi]}　fad = {sim.fad_history[fi]:.3f}")
        st.plotly_chart(fig, width="stretch")
        st.caption("Normal ● 蓝 / Stalled ■ 橙 / Drifting ▲ 红（对应 SwarmViewer ○/□/△）")

with tabtraj:
    st.image(st.session_state["traj_png"], width="stretch")

with tabperf:
    pngs = st.session_state["perf_pngs"]
    st.image(pngs["performance"], width="stretch")
    if "obstacle_distance" in pngs:
        st.image(pngs["obstacle_distance"], width="stretch")
    with st.expander("各指标单独图"):
        for key in ("performance_safety", "performance_order", "performance_union",
                    "performance_connectivity", "performance_safety_obstacles"):
            if key in pngs:
                st.image(pngs[key], width="stretch")

with tabcoll:
    st.markdown(f"**累计碰撞 [机-机, 机-障, min_dist累计] = {np.round(sim.collisions_sum, 2).tolist()}**")
    if "collision_history" in st.session_state["perf_pngs"]:
        st.image(st.session_state["perf_pngs"]["collision_history"], width="stretch")
    ch_full = sim.swarm.collisions_history[:n]
    nz = [i for i in range(ch_full.shape[0]) if ch_full[i, 0] > 0 or ch_full[i, 1] > 0]
    if nz:
        st.dataframe([{"t": round(float(th[i]), 2), "机-机": int(ch_full[i, 0]),
                       "机-障": int(ch_full[i, 1]), "min_dist_obs": round(float(ch_full[i, 2]), 3)}
                      for i in nz[:500]])
    else:
        st.success("✅ 全程无碰撞。")
