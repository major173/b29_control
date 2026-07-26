# scripts/delay — 延迟/sim2real 对齐分析工具

从 RL 仓库（b29_locomotion）迁入统一管理：loco 仓库不做部署，实机数据的采集与
分析工具都放本仓库。协议、判据与决策树见 RL 仓库 `docs/delay/`（01 文档）。

| 脚本 | 作用 | 解释器要求 |
|---|---|---|
| `fit_actuator_id.py` | 01 测试 B/C：`../actuator_id_node.py` 的 session 数据 → (τd, τm 或 ωn/ζ, v_max) 参数表 + Bode | numpy/scipy/matplotlib（本机用 conda `legged38`；**不要**用 `/usr/bin/python3`，其 scipy 损坏） |
| `test_fit_actuator_id.py` | 上者的 13 项自检（合成真值反演），改完先跑这个 | 同上 |
| `analyze_dq_noise.py` | 01 测试 F：dq 观测噪声频谱 + 低通 α 定标（两段式：extract 用系统 python + rosbag，analyze 用 conda） | 见脚本 docstring |
| `analyze_ros_deploy_bag.py` | 部署 bag（/gp11/rl/* 话题）的震荡/观测链分析 | 系统 python + rosbag |

注意：`fit_actuator_id.py` 的前向模型与 RL 仓库
`sim2deploy/gp11_reach/injection_chain.py` 同构（延迟 → 速率限制 → 惯性，顺序一致），
改模型结构必须两边同步，否则拟合参数喂回注入实验会失真。
