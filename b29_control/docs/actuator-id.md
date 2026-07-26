# 执行器辨识（actuator_id_node）使用与调试

给实机测「执行器有多慢」用的激励脚本。产出的数据交给
`scripts/delay/fit_actuator_id.py` 拟合成三参数模型 (τd, τm 或 ωn/ζ, v_max)，
供训练侧对齐（协议与判据见 RL 仓库 `docs/delay/01-identification.md` 测试 B/C）。

- 激励节点：`scripts/actuator_id_node.py`
- 配置：`config/actuator_id.yaml`
- 无硬件假被控对象：`scripts/mock_plant_node.py`
- 离线分析工具（拟合/噪声/部署 bag）：`scripts/delay/`（见其 README）

---

## 全流程总览（按顺序做，每步有验收）

| # | 阶段 | 在哪跑 | 命令 | 验收标准 |
|---|---|---|---|---|
| 0 | 拟合工具自检 | 带 scipy 的解释器（本机 `legged38`） | `python3 scripts/delay/test_fit_actuator_id.py` | 13 项全 PASS |
| 1 | 无硬件联调 | 本仓库，`b29` + roscore | 见下节「无硬件联调」 | 2 组跑完，npz/log 齐全 |
| 2 | 故障注入 | 同上 | `--fault freeze/stuck/runaway` 各跑一遍 | 每种故障都触发安全停，且指令冻结在安全位姿 |
| 3 | mock 数据过拟合 | 同步骤 0 | `scripts/delay/fit_actuator_id.py --rundir <mock session>` | 反演 ≈ mock 真值（τd≈60–80ms、τm≈50ms、NRMSE<10%） |
| 4 | dry-run | 实机侧，不连 ROS | `actuator_id_node.py --dry-run` | 计划/时长合理，限位校验通过 |
| 5 | 实机 minimal | 实机，**人守急停** | `actuator_id_node.py --minimal` | 每组小结：稳态误差→0、阶跃次数=reps×2 |
| 6 | minimal 拟合 | 同步骤 0 | `scripts/delay/fit_actuator_id.py --rundir ~/actuator_id_runs/<stamp>` | τd σ<10ms、NRMSE<10%（❌ 则按 01 测试 B 分支改协议/模型再回步骤 5） |
| 7 | 实机全量 | 实机 | 去掉 `--minimal` | 同上判据，4 关节 × 2 幅值 × 2 方向全过 |
| 8 | 换姿态重跑 | 实机 | 手动摆位姿后重跑 session | 参数随姿态漂移 <50%（超了 → 01 测试 B 的负载敏感分支） |
| 9 | 参数回填 | RL 仓库 | 把参数表喂给 `injection_chain.py` 注入实验 | 见 `docs/delay/03` 下一步 |

步骤 0–3 已于 2026-07-26 完成并通过（见下文验证记录，及 RL 仓库
`docs/delay/work-steps/` 下 `2026-07-26-04`、`2026-07-26-05` 两篇日志）；
从步骤 4 起需要实机。

---

## ⚠️ 安全须知（先看这段）

1. **session 全程必须有人守在急停旁。** 脚本的看门狗只能减少损失，不能替代急停。
2. 首次上机前，先按下面「无硬件联调」跑通一遍，确认 TUI、日志、看门狗都正常。
3. 上机第一次只跑 `--minimal`（单关节、最小幅值），确认曲线合理再跑全量。
4. 激励幅值很小（0.03/0.08 rad ≈ 1.7°/4.6°），但**关节必须处在能自由小幅运动的位姿**，
   不要卡在限位、不要压在地面/障碍上——被约束住会直接触发跟踪误差看门狗（这是预期行为，
   但反复触发说明起始位姿选错了）。
5. 脚本**只发位置指令，不改任何底层参数**（不动 kp/kd、不动 jointSpeedTarget）。
   要测 jointSpeedTarget 的影响（01 测试 A），单独改配置后重跑，别在 session 中途改。

### 四层安全防护

| 层 | 作用 | 触发后行为 |
|---|---|---|
| 前置自检 | /joint_states 新鲜（10s 内）、起始位姿匹配、全轨迹距硬限位有 `limit_margin` 余量；再保持起始位姿 2s 量**静置误差基线**（重力下垂/稳态误差） | 静置误差 ≥ `track_err_rad` 直接退出；≥ 一半则告警继续 |
| 运行时看门狗 | 反馈断流 / 跟踪误差 / 速度异常 / 力矩超限 / 循环卡顿；**闸门等待期间同样在线** | 立即安全停；组内已采数据存 `aborted_group_*.npz` |
| 固定位姿安全停 | 把目标**冻结在 session 开始时记录的安全位姿**，持续重发 `safe_stop_publish_s` 秒 | — |
| 分组人工闸门 | 每组开始前等按键；>3Hz 的高频组额外标 `[需确认]` | 可 s 跳过 / a 中止 |

注意 `tau_limits` 的单位必须与实机 `/joint_states` 的 `effort` 字段一致（N·m？电流？
原始计数？）——上机前先 `rostopic echo -n1 /joint_states` 核对；拿不准就把阈值设很大
（如 `1.0e9`）等效停用力矩层，其余三层不受影响。mock 联调里 effort 恒为 0，测不出这个问题。

**为什么安全停是「冻结到固定位姿」而不是「停在当前测量位置」**：追测量值会和下位机
SafetyLimiter 形成互相追赶的正反馈，越追越跑。固定位姿是唯一确定的落点。

看门狗用**漏积分计数**（超限 +1、正常 −1）而不是「连续超限才算」：自激震荡每周期都会
短暂回落到阈值以下，连续计数对震荡是盲的，漏积分不会被这种回落清零。

---

## 无硬件联调（上机前必做）

`mock_plant_node.py` 是个假执行器：订阅指令，按「纯延迟 60ms → 速率限制 6 rad/s →
一阶惯性 50ms」算出位置，50Hz 发 `/joint_states`。它还能注入故障，用来验证看门狗真的会响。

```bash
# 终端 1
roscore

# 终端 2：假被控对象（注意 --ns 要和真实控制器命名空间一致）
conda activate b29
python3 src/b29_control/b29_control/scripts/mock_plant_node.py --ns /b29_controller

# 终端 3：激励节点
conda activate b29
python3 src/b29_control/b29_control/scripts/actuator_id_node.py --minimal
```

### 故障注入

`--fault <类型>:<秒>`，第 N 秒起开始故障：

| 故障 | 模拟的现实情况 | 预期看门狗 |
|---|---|---|
| `freeze:6` | 串口断连 / 下位机死机 → 反馈断流 | `反馈断流 ...ms > 100ms` |
| `stuck:6` | 关节被卡住 / 堵转 | `跟踪误差异常 joint[k]` |
| `runaway:6` | 自激震荡 / 疯机（注入 3Hz、0.30rad 振荡） | `跟踪误差异常` 或 `速度异常（疑似自激/疯机）` |

`runaway` 默认会先撞上跟踪误差看门狗（tick 里跟踪误差先判、且 persist 更短 0.20s
vs 0.30s）。要单独验证速度分支，临时把配置改成 `track_err_rad: 1.20` +
`dq_limit_rad_s: 1.0` 再跑——**两条路径都算安全停成功**，实机上谁先响都无所谓。

验证已跑过的结果（2026-07-26，mock 环境）：

```
freeze:6   → SAFE-STOP: 反馈断流 118ms > 100ms
runaway:6  → SAFE-STOP: 跟踪误差异常 joint[0] err=0.695rad 持续超限
             （抬阈值后）SAFE-STOP: 速度异常（疑似自激/疯机）joint[0] |dq|=1.67rad/s
stuck:6    → SAFE-STOP: 跟踪误差异常 joint[1] err=0.030rad 持续超限
             （需把 track_err_rad 调到 0.02，否则卡死幅度不够 0.20rad）
             并存下 aborted_group_*.npz（组内已采 209 拍 + 中止原因）
freeze 在闸门等待时发生
           → SAFE-STOP: 闸门等待中 反馈断流 111ms > 100ms（未进入任何激励组）
安全停后末 2s 指令恒为安全位姿，std = 0（确认冻结、未追测量值）
```

### 只看计划不动机器人

```bash
python3 src/b29_control/b29_control/scripts/actuator_id_node.py --minimal --dry-run
```

打印分组、时长、峰值偏移、`[需确认]` 标记，并做限位校验（`start_pose: current`
时限位校验推迟到上机自检）。**改完配置先跑这个**。

---

## 实机运行

前置：`roslaunch b29_control start.launch` 已起、`/joint_states` 有数据、急停在手边。

```bash
conda activate b29
python3 src/b29_control/b29_control/scripts/actuator_id_node.py --minimal
```

按键：`SPACE`/`Enter` 开始下一组，`s` 跳过，`r` 重跑上一组，`a`/`q` 安全中止。
`--no-tui` 走纯文本（闸门用回车确认），适合 ssh 或需要抓完整 stdout 的场合。

TUI 显示每个关节的 cmd/q/误差/dq（超阈值变黄/红）、反馈时延、看门狗状态、
进度条、上一组小结、最近事件。

### 时长

时长以 `--dry-run` 打印的为准（下表为默认配置实测）：

| 模式 | 内容 | 组数 | 净激励时长 |
|---|---|---|---|
| `--minimal` | 2 关节 × 1 幅值 × 2 方向 阶跃 + 扫频 | 14 | 5.5 min |
| 全量（默认配置） | 4 关节 × 2 幅值 × 2 方向 + 全频段扫频 | 36 | 14.6 min |

「净激励时长」不含闸门等待。每组之间要人按键确认，全量实际挂钟时间按 20–30 min 预留。

「阶跃 5 次重复」是为了算 τd 的组内标准差（判据 σ < 10ms），不是冗余。
姿态维度（改变重力负载）目前**没有**编进脚本：换姿态 = 手动摆好后重跑一次 session，
产出两个目录分别拟合再对比。

---

## 输出与后续分析

每次 session 建一个带时间戳的目录（默认 `~/actuator_id_runs/<stamp>/`）：

| 文件 | 内容 |
|---|---|
| `session.log` | 全过程日志：自检（含静置误差基线）、每组小结、安全停原因 |
| `group_*.npz` | 每组的 `t/cmd/q/dq`（4 关节全量）+ 元数据，拟合工具直接吃这个 |
| `aborted_group_*.npz` | 安全停/中止那一组**已采到的部分数据**，meta 里带中止原因；拟合工具默认忽略，诊断震荡时手动看它 |
| `session.bag` | `/joint_states` 与 4 路 command，归档用 |

拟合（本仓库 `scripts/delay/`，需带 numpy/scipy 的解释器——本机用 conda `legged38`，
**别用 `/usr/bin/python3`**，其 scipy 损坏）：

```bash
cd ~/usetest/B29/src/b29_control/b29_control
python3 scripts/delay/fit_actuator_id.py \
    --rundir ~/actuator_id_runs/<stamp> \
    --outdir ~/actuator_id_runs/<stamp>/fit/
```

结果表要归档进 RL 仓库 `docs/delay/data/actuator-id/` 时，把 `--outdir` 指过去即可。

产出 `actuator-id-table.md`（τd/τm/v_max + σ + NRMSE 判据）、`bode.md`（测试 C 幅频相频，
附阶跃模型预测对比）、逐组拟合叠图。一阶拟合不上（振铃/超调）时加 `--second-order`
改 (ωn, ζ)——对应 01 测试 B 的 ❌ 分支。

---

## 常见问题

**前置自检失败：10s 内没有新鲜的 /joint_states**
硬件接口没起或串口断了。`rostopic hz /joint_states` 确认，再看 `start.launch` 那个终端。

**前置自检失败：当前位姿与配置起始位姿偏差 >0.1rad**
`start_pose` 写了显式值但机器人不在那儿。要么手动摆过去，要么改成 `start_pose: current`
（推荐，以启动时实际位姿为基准最安全）。

**限位校验失败：轨迹范围超出限位安全区**
当前位姿离硬限位太近，激励会撞上去。挪个位姿，或减小 `protocol.step.amplitudes`。

**一上机就安全停（跟踪误差）**
先看 `session.log` 里报的是哪个 joint。常见原因：该关节被机械约束住、或下位机
position_controller 没使能。用 `rostopic echo` 对比 command 和 /joint_states 确认指令进去了。

**前置自检失败：静置跟踪误差 ≥ track_err_rad**
非激励关节在重力下垂或控制器稳态误差太大，跑组途中必然误触发安全停，所以自检阶段直接拦下。
换一个各关节都能稳住的姿态；确认下垂无危险后也可调大 `track_err_rad`（代价：卡死检测变钝）。
日志里的「静置误差基线」按关节给出数值，看哪个关节在垂。

**一开力矩层就安全停（力矩超限）**
八成是 `tau_limits` 单位和实机 effort 字段不一致（见上文四层防护表的注意事项）。

**看曲线想诊断安全停那一刻发生了什么**
读 `aborted_group_*.npz`（字段与 `group_*.npz` 相同，meta 多一个 `aborted` 原因）。
确认数据可用后，把它改名成 `group_*.npz` 也能喂给拟合工具。

**控制循环卡顿 XXms（WARNING，不中止）**
本机负载高。关掉 RViz/rqt 之类，或把 `control_rate_hz` 降到 30。偶发几次不影响拟合，
因为拟合用的是记录的实际时间戳而不是标称周期。

**改了扫频频率后 build_plan 报 StopIteration**
已修（全低频配置无高频确认组时，低频组追加到末尾）。如果还遇到，先跑 `--dry-run`。

**`session.log` 空 / 没生成**
`rospy.init_node` 会先配置 root logger，`logging.basicConfig` 之后就成空操作了——
节点里改用显式 FileHandler 解决。若自行加日志，别用 `basicConfig`。
