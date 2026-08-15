# scripts/delay — 延迟/sim2real 对齐分析工具

实机采集与离线分析均在本仓库；采集协议、判据与决策树见 RL 仓库 `docs/delay/`。

| 脚本 | 作用 | 解释器要求 |
|---|---|---|
| `fit_actuator_id.py` | 测试 B/C：将 `actuator_id_node.py` session 的阶跃/扫频数据拟合为 v2 模型、Markdown、JSON 和可选图 | numpy/scipy/matplotlib；本机使用 conda `legged38` |
| `test_fit_actuator_id.py` | 合成真值自检，直接运行、不依赖 pytest | 同上 |
| `analyze_dq_noise.py` | 测试 F：dq 观测噪声频谱 + 低通 α 定标 | 见脚本 docstring |
| `analyze_ros_deploy_bag.py` | 部署 bag 的震荡/观测链分析 | 系统 python + rosbag |

## v2 阶跃模型与质量门

每个边沿先从边前窗口估计 `cmd_pre`、`q_pre`，在局部增量坐标中拟合：

```text
u = cmd - cmd_pre
q_model = q_pre + K * dynamic(u)
```

其中 `dynamic` 为 `纯延迟 → 速率限制 → 一阶 τm`（默认），或
`纯延迟 → 二阶 (ωn, ζ)`（`--second-order`）。二阶采用 ZOH 精确离散
`scipy.linalg.expm`，避免高 `ωn` 时显式积分溢出。拟合输出 `K`、实测
`dc_gain`、`overshoot`、`steady_error`，以及 least-squares 的
`success/status/message/cost/optimality/nfev/active_mask`、逐参数 `bound_hits` 和
`fit_valid`。

聚合质量门：`pass_optimizer`、`pass_no_bounds`、`pass_all`；仅 `pass_all`
候选可进入 `model_by_joint`。一阶中 `rate_identifiable` 仅当实测
`rate_sat_frac > 0.6` 且 `vmax` 未命中边界时为真；否则 `vmax` 不会进入批准模型。
`pass_all` 仅表示阶跃拟合通过；`model_by_joint` 只是代表候选。进入 P2 前仍需检查
候选模型能否解释实测过冲，并与分段 Bode 的峰值、幅值和相位一致。

## v2 扫频切段

`process_sweep_group` 遵循采集节点协议，不再把低频拼接组的整个时间窗拿来分别拟合：

- 每个频点跳过其前的 1 秒 hold；
- `f < 1 Hz` 使用 6 周期，其他频点使用 10 周期；
- 丢弃正弦段首周期，在余下稳态窗口拟合 `[sin, cos, constant, linear trend]`；
- 高频独立组也按同一规则处理。

每个点含 `amp_cmd`、`amp_ratio`、wrapped/unwrapped 相位、窗口起止、样本数、
周期数、`residual_rms` 和 `residual_ratio = residual_rms / amp_q`。
`sweep_points()` 保留旧整窗 API，只用于兼容旧调用方；组处理始终走协议切段。

## 使用

```bash
conda run -n legged38 python b29_control/scripts/delay/test_fit_actuator_id.py
conda run -n legged38 python b29_control/scripts/delay/fit_actuator_id.py \
  --rundir ~/actuator_id_runs/<stamp> \
  --outdir ~/actuator_id_runs/<stamp>/fit
# 二阶候选：追加 --second-order
```

`summary.json` 的 `schema_version=2`，仍保留 `step`、`sweep` 和
`model_by_joint` 顶层字段，以便旧消费者读取。报告表和 Bode 图会显示 K、质量诊断、
残差、有效周期及 wrapped/unwrapped 相位；无批准模型的关节明确不输出代表模型。

注意：前向链仍与 RL 仓库 `sim2deploy/gp11_reach/injection_chain.py` 的顺序同构。
若改变链路结构，必须同步两侧，否则拟合参数用于注入实验会失真。
