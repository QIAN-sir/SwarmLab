"""Simulation —— point_mass 故障群仿真的 headless 主循环引擎（Stage 1）。

对应 MATLAB: examples/examples_swarm/example_olfati_saber.m 与 example_vasarhelyi.m 的主循环
（脱离 GUI 与可视化，计划 §14「核心仿真须先脱离 GUI 独立运行」）。
Stage 2（可视化）与 Stage 3（GUI）将复用本引擎：GUI 通过反复调用 step() 驱动，
并用 on_step / should_stop 钩子接入实时绘图与启停。

主循环每一步（dt）忠实复刻 MATLAB 顺序:
    1) 故障注入（替代 GUI 的 drone_state 下拉变更）：到点触发 set_dronestate，Drifting 记 t_fault
    2) swarm.update_command → collisions；collisions_sum 累加
    3) 计算 fad：Normal/Stalled → 1；Drifting → 1/(1+0.2*(t-t_fault))，截断[0,1]，<0.1→0
    4) swarm.update_state(wind, time, fad)

注：群体示例中 wind 恒为 zeros(6,1)（未调用 get_wind，那是单机真实动力学路径）。
"""

import math
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple, Union

import numpy as np

from ..core import Swarm
from ..enums import DroneState
from ..parameters import build_params, ParamsBundle


@dataclass
class FaultEvent:
    """故障注入事件（脚本驱动，替代 GUI 的 drone_state 变更）。

    time       : 触发时刻 [s]（在该时刻及之后的第一个步进生效）
    state      : "Normal" / "Stalled" / "Drifting"
    loss_radio : 失控比例；None → 用仿真级 loss_radio
    """
    time: float
    state: str
    loss_radio: Optional[float] = None


class Simulation:
    """point_mass 故障群仿真引擎。

    用法（headless 批量）::
        sim = Simulation(algorithm="olfati_saber", nb_agents=10,
                         fault_schedule=[FaultEvent(30, "Drifting")], seed=0)
        sim.run()
        pos = sim.get_pos_history()

    用法（GUI 逐步驱动）::
        sim = Simulation(...)
        while not sim.done and not gui_stop:
            sim.step()
            viewer.update(sim.time, sim.swarm, sim.params.map)
    """

    def __init__(self, drone_type: str = "point_mass", algorithm: str = "olfati_saber", *,
                 nb_agents: Optional[int] = None, d_ref: Optional[float] = None,
                 v_ref: Optional[float] = None, loss_radio: Optional[float] = None,
                 u_ref: Optional[Sequence[float]] = None,
                 orientation_deg: Optional[float] = None,
                 active_environment: bool = True, end_time: Optional[float] = None,
                 seed: Optional[int] = None, wind: Optional[Sequence[float]] = None,
                 fault_schedule: Optional[Sequence[Union[FaultEvent, Tuple]]] = None):

        # orientation_deg → u_ref（复刻 MATLAB: u_ref=[-cosd,-sind,0]'）
        if u_ref is None and orientation_deg is not None:
            o = math.radians(orientation_deg)
            u_ref = [-math.cos(o), -math.sin(o), 0.0]
        if u_ref is not None:
            u_ref = np.asarray(u_ref, dtype=float).reshape(3, 1)

        # 装配参数（复刻示例顶部 run(param_*) 顺序）
        self.params: ParamsBundle = build_params(
            drone_type, algorithm, active_environment=active_environment,
            end_time=end_time, nb_agents=nb_agents, d_ref=d_ref, v_ref=v_ref,
            loss_radio=loss_radio, u_ref=u_ref, seed=seed)

        self.drone_type = drone_type
        self.algorithm = algorithm
        self.p_sim = self.params.p_sim
        self.p_swarm = self.params.p_swarm

        # 建群（初始全 Normal；故障由 fault_schedule 注入）
        self.swarm = Swarm()
        self.swarm.algorithm = algorithm
        for _ in range(self.p_swarm.nb_agents):
            self.swarm.add_drone(drone_type, DroneState.NORMAL, self.params.p_drone,
                                 self.params.p_battery, self.p_sim, self.params.p_physics,
                                 self.params.map)
        self.swarm.set_pos(self.p_swarm.Pos0)

        # 风（群体示例恒为 0）
        self.wind = np.zeros(6) if wind is None else np.asarray(wind, dtype=float).reshape(6)

        # 时间轴（start + i*dt，匹配 MATLAB start:dt:end）
        self.start_time = float(self.p_sim.start_time)
        self.end_time = float(self.p_sim.end_time)
        self.dt = float(self.p_sim.dt)
        self.time = self.start_time
        self._step_idx = 0
        self.done = False

        # 故障注入状态
        self.current_state = DroneState.NORMAL.value   # 当前指令状态字符串（对应 MATLAB DRONE_STATE）
        self.loss_radio = self.p_swarm.loss_radio if loss_radio is None else loss_radio
        self.swarm.t_fault = 0.0
        self.fault_schedule: List[FaultEvent] = self._normalize_schedule(fault_schedule)
        self._event_idx = 0

        # 记录
        self.collisions_sum = np.zeros(3)
        self.time_history: List[float] = []
        self.fad_history: List[float] = []
        self.state_history: List[str] = []

    # ------------------------------------------------------------------
    @staticmethod
    def _normalize_schedule(schedule) -> List[FaultEvent]:
        events: List[FaultEvent] = []
        for ev in (schedule or []):
            if isinstance(ev, FaultEvent):
                events.append(ev)
            else:  # 允许 (time, state) 或 (time, state, loss_radio) 元组
                t, st = ev[0], ev[1]
                lr = ev[2] if len(ev) > 2 else None
                events.append(FaultEvent(float(t), st, lr))
        events.sort(key=lambda e: e.time)
        return events

    # ------------------------------------------------------------------
    def _apply_fault_events(self, time: float) -> None:
        """触发所有 time 之前（含）尚未处理的故障事件。"""
        while self._event_idx < len(self.fault_schedule):
            ev = self.fault_schedule[self._event_idx]
            if ev.time <= time + 1e-9:
                self._fire_event(ev, time)
                self._event_idx += 1
            else:
                break

    def _fire_event(self, ev: FaultEvent, time: float) -> None:
        loss = self.loss_radio if ev.loss_radio is None else ev.loss_radio
        # DRONE_STATE_CHANGED：Drifting→2（记录 t_fault），其余→1（对应 MATLAB）
        if ev.state == "Drifting":
            changed = 2
            self.swarm.t_fault = time
        else:
            changed = 1
        self.current_state = ev.state.value if isinstance(ev.state, DroneState) else str(ev.state)
        self.swarm.set_dronestate(ev.state, changed, loss)

    def _compute_fad(self, time: float) -> float:
        """复刻 MATLAB 主循环的 fad 计算。"""
        if self.current_state == "Normal" or self.current_state == "Stalled":
            return 1.0
        # Drifting
        if self.swarm.t_fault != 0:
            k = 0.2
            fad = 1.0 / (1.0 + k * (time - self.swarm.t_fault))
            fad = min(max(fad, 0.0), 1.0)
            if fad < 0.1:
                fad = 0.0
            return fad
        # t_fault==0：MATLAB 不更新 fad，保留上一值
        return self.swarm.fad

    # ------------------------------------------------------------------
    def step(self) -> bool:
        """推进一个 dt。返回推进后是否仍未结束。"""
        if self.done:
            return False
        time = self.time

        # 1) 故障注入  2) 速度指令  3) fad  4) 状态更新
        self._apply_fault_events(time)
        _, collisions = self.swarm.update_command(self.p_swarm, self.p_swarm.r_coll, self.dt)
        self.collisions_sum = self.collisions_sum + np.asarray(collisions, dtype=float)
        self.swarm.fad = self._compute_fad(time)
        self.swarm.update_state(self.wind, time, self.swarm.fad)

        # 记录
        self.time_history.append(time)
        self.fad_history.append(float(self.swarm.fad))
        self.state_history.append(str(self.current_state))

        # 推进时间
        self._step_idx += 1
        self.time = self.start_time + self._step_idx * self.dt
        if self.time > self.end_time + 1e-9:
            self.done = True
        return not self.done

    def run(self, until: Optional[float] = None,
            on_step: Optional[Callable[["Simulation"], None]] = None,
            should_stop: Optional[Callable[[], bool]] = None) -> "Simulation":
        """批量运行主循环。

        until       : 运行到该时刻为止（None→到 end_time）
        on_step     : 每步回调 fn(sim)（供可视化/GUI 刷新）
        should_stop : 每步前检查，返回 True 则中断（供 GUI 停止按钮）
        """
        while not self.done:
            if should_stop is not None and should_stop():
                break
            if until is not None and self.time > until + 1e-9:
                break
            self.step()
            if on_step is not None:
                on_step(self)
        return self

    # ------------------------------------------------------------------
    def get_pos_history(self) -> np.ndarray:
        """(T, 3N) 位置历史（各机 3 列横向拼接）。"""
        return self.swarm.get_pos_ned_history()

    def get_vel_history(self) -> np.ndarray:
        """(T, 3N) 速度历史。"""
        return self.swarm.get_vel_xyz_history()
