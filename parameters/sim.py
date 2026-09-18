"""SimParams —— 仿真通用参数。

对应 MATLAB: parameters/param_sim.m
    p_sim.dt = 0.01;          % simulation dt [s]
    p_sim.dt_plot = 0.5;      % period of plots update [s]
    p_sim.dt_video = 0.1;     % period of video update [s]
    p_sim.start_time = 0;     % [s]
    if ~isfield(p_sim, 'end_time')
        p_sim.end_time = 120; % [s]
    end

说明: end_time 在 MATLAB 中可被 GUI(app.sim_time) 覆盖；此处作为可覆盖字段，默认 120。
"""

from dataclasses import dataclass


@dataclass
class SimParams:
    """仿真时间与刷新周期参数。"""

    dt: float = 0.01          # 仿真步长 [s]
    dt_plot: float = 0.5      # 绘图刷新周期 [s]
    dt_video: float = 0.1     # 视频刷新周期 [s]
    start_time: float = 0.0   # 起始时间 [s]
    end_time: float = 120.0   # 结束时间 [s]（可被覆盖，如 GUI app.sim_time）
