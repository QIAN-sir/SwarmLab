"""offline_plots —— 离线轨迹图与性能指标图（matplotlib）。

对应 MATLAB:
    graphics/graphics_swarm/plot_trajectories_offline.m
    graphics/graphics_swarm/plot_swarm_performance.m
坐标约定：plot x=east(pe), y=north(pn), z=up(-pd)（NED→ENU）；轨迹图用俯视视角（MATLAB view(2)）。

这些函数返回 matplotlib Figure，dirname 非空时另存 PNG（对应 MATLAB print -dpng）。
"""

import os

import numpy as np
import matplotlib.pyplot as plt

from .map_drawing import draw_buildings, draw_cylinders, set_map_axes

_METRIC_COLORS = {
    "Safety": "#0072BD", "Order": "#D95319", "Union": "#EDB120",
    "Connectivity": "#7E2F8E", "Safety Obstacles": "#77AC30",
}


def plot_trajectories(pos_history, nb_agents, colors=None, map_params=None,
                      save_path=None, figsize=(6, 6)):
    """绘制群体离线轨迹（对应 plot_trajectories_offline.m）。

    pos_history: (T, 3N)，各机占 3 列 (pn, pe, pd)。
    """
    pos_history = np.asarray(pos_history)
    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection="3d")

    if map_params is not None and getattr(map_params, "ACTIVE_ENVIRONMENT", False):
        if map_params.building_shape == "rectangle":
            draw_buildings(ax, map_params)
        elif map_params.building_shape == "cylinder":
            draw_cylinders(ax, map_params)
        set_map_axes(ax, map_params)

    for agent in range(nb_agents):
        pn = pos_history[:, agent * 3 + 0]
        pe = pos_history[:, agent * 3 + 1]
        pd = pos_history[:, agent * 3 + 2]
        c = colors[:, agent] if colors is not None else None
        ax.plot(pe, pn, -pd, color=c, lw=1.0)     # x=east, y=north, z=up

    ax.set_xlabel("Y Position [m]")
    ax.set_ylabel("X Position [m]")
    ax.set_zlabel("Z Position [m]")
    ax.view_init(elev=90, azim=-90)               # 俯视（对应 view(2)）
    ax.grid(True)
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
    return fig


def plot_performance(time_history, safety, order, union, alg_conn, safety_obs, min_d_obs,
                     collisions_history=None, dirname=None, fontsize=12):
    """绘制性能指标曲线（对应 plot_swarm_performance.m）。返回 {name: Figure} 字典。"""
    time_history = np.asarray(time_history)
    figs = {}

    # ---- 汇总图 ----
    fig = plt.figure(figsize=(10, 6))
    plt.plot(time_history, safety, lw=1.5, label="safety")
    plt.plot(time_history, order, lw=1.5, label="order")
    plt.plot(time_history, union, lw=1.5, label="union")
    plt.plot(time_history, alg_conn, lw=1.5, label="connectivity")
    plt.plot(time_history, safety_obs, lw=1.5, label="safety obstacles")
    plt.xlabel("Time [s]", fontsize=fontsize)
    plt.ylabel("Performance", fontsize=fontsize)
    plt.legend()
    plt.grid(True)
    figs["performance"] = fig
    if dirname:
        fig.savefig(os.path.join(dirname, "performance.png"), dpi=150, bbox_inches="tight")

    # ---- 各指标单独图 ----
    metrics = [("Safety", safety), ("Order", order), ("Union", union),
               ("Connectivity", alg_conn), ("Safety Obstacles", safety_obs)]
    for name, data in metrics:
        f = plt.figure(figsize=(6, 4))
        plt.plot(time_history, data, lw=1.5, color=_METRIC_COLORS[name])
        plt.xlabel("Time [s]", fontsize=fontsize)
        plt.ylabel(name, fontsize=fontsize)
        plt.title(f"{name} over Time", fontsize=fontsize)
        plt.grid(True)
        key = "performance_" + name.lower().replace(" ", "_")
        figs[key] = f
        if dirname:
            f.savefig(os.path.join(dirname, key + ".png"), dpi=150, bbox_inches="tight")

    # ---- 障碍物最小距离 ----
    f = plt.figure(figsize=(6, 4))
    plt.plot(time_history, np.min(np.asarray(min_d_obs), axis=1), lw=1.5)
    plt.axhline(0, lw=1.5, color="k")
    plt.xlabel("Time [s]", fontsize=fontsize)
    plt.ylabel("Distance to obstacles", fontsize=fontsize)
    plt.title("Minimum Distance to Obstacles", fontsize=fontsize)
    plt.grid(True)
    figs["obstacle_distance"] = f
    if dirname:
        f.savefig(os.path.join(dirname, "obstacle_distance.png"), dpi=150, bbox_inches="tight")

    # ---- 碰撞历史（二次开发新增）----
    if collisions_history is not None:
        ch = np.asarray(collisions_history)
        f = plt.figure(figsize=(6, 4))
        plt.plot(time_history, ch[:, 0], lw=1.5, color="#0072BD", label="Agent-Agent Collisions")
        plt.plot(time_history, ch[:, 1], lw=1.5, color="#D95319", label="Agent-Obstacle Collisions")
        plt.xlabel("Time [s]", fontsize=fontsize)
        plt.ylabel("Number of Collisions", fontsize=fontsize)
        plt.title("Collision History over Time", fontsize=fontsize)
        plt.legend(fontsize=fontsize - 2, loc="best")
        plt.grid(True)
        figs["collision_history"] = f
        if dirname:
            f.savefig(os.path.join(dirname, "collision_history.png"), dpi=150, bbox_inches="tight")

    return figs
