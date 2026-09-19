# result/ —— 群体仿真结果存放目录

对应 MATLAB 的 `results/results_swarm/` 惯例：每次群体仿真运行在
`result/<algorithm>_<yyyy_mm_dd_HH_MM_SS>[_tag]/` 下集中存放该次运行的全部产物，
便于后续**挑选 / 对比多次运行**。

## 子目录命名

- `<algorithm>`：`olfati_saber` / `vasarhelyi`
- `<timestamp>`：运行时刻 `yyyy_mm_dd_HH_MM_SS`
- `[_tag]`：可选附加标签（故障配置等），由 `new_result_dir(algorithm, tag=...)` 传入

## 每次运行典型产物

| 文件 | 内容 |
|---|---|
| `swarm_simulation.mp4` | 群体仿真全过程视频（SwarmViewer + VideoWriterWithRate） |
| `frame_*_normal/stalled/drifting.png` | 故障三态关键帧 |
| `trajectories.png` | 轨迹俯视图 |
| `performance.png` + `performance_*.png` | 6 项性能指标曲线（safety/order/union/alg_conn/safety_obs/min_d_obs） |
| `collision_history.png` | 碰撞累计曲线 |
| `obstacle_distance.png` | 障碍最小距离曲线 |
| `performance.npz` | 性能指标数值（供后续分析/复画） |

## 生成方式

```python
from SwarmLab_py.simulation import new_result_dir
rdir = new_result_dir("olfati_saber", tag="fault30")   # 建目录并返回路径
# ... 把视频/图存到 rdir ...
```

或直接运行 demo：`python SwarmLab_py/examples/run_swarm_result.py [algorithm]`
