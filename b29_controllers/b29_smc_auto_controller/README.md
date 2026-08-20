# b29_smc_auto_controller

这是 B29 的自动控制器。它决定机器人当前处于巡航、越障、通信丢失还是安全停机，并生成最终的轮子、夹爪和腿部关节命令。

所有命令都必须经过：

```text
buildEffectiveCommand()
  -> CommandDispatcher::dispatch()
  -> ros_control hardware handles
```

Planner、GUI 和调试输入都不能绕过这条链路。

## 1. 控制器负责什么

- 外层状态机：`Idle / AutoInit / Traversing / CommsLoss / SafeStop`。
- 双侧越障状态机。
- 夹爪张开、重力补偿和脱缆动作。
- PlannerControl 会话。
- 下位机 RemoteControl 增量控制。
- Regrip、retry 和人工介入。
- 软件急停、安全仲裁和 `state_trace`。

主要内部对象：

| 对象 | 职责 |
|---|---|
| `RobotContext` | 推进外层 RobotFSM，生成巡航或安全基础命令 |
| `AutoInputMux` | 合并硬件、sensor input、控制请求和 debug override |
| `ObstacleCrossingRuntime` | 保存越障阶段、侧别、retry、人工恢复和重力补偿状态 |
| `DisconnectCableProcess` | 执行脱缆插值和 Step7 速度稳定判断 |
| `PlannerControlCoordinator` | 管理 normal/debug PlannerControl 的进入、完成和超时 |
| `RemoteControlSession` | 每个下位机遥控样本只应用一次，并检测完成上升沿 |
| `CommandDispatcher` | 将最终命令写入 ros_control 句柄 |
| `ControllerTraceBuilder` | 组装 GUI 使用的完整 trace |

完整转换表见 [`OBSTACLE_CROSSING_WORKFLOW.md`](OBSTACLE_CROSSING_WORKFLOW.md)。

## 2. 输入和输出

控制器从已注册的 ros_control 接口读取：

- 关节位置和速度。
- `base_imu`。
- `AutoStateInterface`。
- `RemoteControlInterface`。

真机常用业务输入：

```text
lower_alive
imu_ready
grip_confirmed
joint_fault / grip_fault
cruise_drive_request
auto_start / manual_reset / obstacle_crossing_trigger
```

`controller.yaml` 必须在 `use_auto_state` 和 `use_sensor_input` 中选择一个输入源，不能同时启用或同时关闭。

输出包括：

- 左右轮速度。
- 四个腿部关节位置。
- 两个夹爪目标。
- 1 字节重力补偿模式。
- `state_trace` 和 `planner_control_state`。

重力补偿模式：

```text
0 = Off
1 = 左一关节
2 = 右一关节
```

当前侧表示张开的夹爪侧，因此 Left 越障时实际运动和补偿的是右一关节；Right 越障时对应左一关节。

## 3. 外层状态

| 状态 | 简单说明 | 如何离开 |
|---|---|---|
| `Idle` | 等待 AutoStart | 新 AutoStart 上升沿且通信、IMU 正常 |
| `AutoInit` | 启动检查 | 条件满足进入 Traversing；2 秒超时进入 SafeStop |
| `Traversing` | 巡航并允许越障 | 暂停、通信丢失或安全故障 |
| `CommsLoss` | 等待通信恢复 | 恢复后回 Idle；5 秒超时进入 SafeStop |
| `SafeStop` | 安全锁存 | 故障恢复后收到新 ManualReset |

事件优先级以 `RobotContext::tick(period)` 为准：安全故障最高，然后处理 CommsLoss、AutoInit、通信丢失、复位、启动和暂停。

`posture_ready` 只用于诊断，不参与 AutoStart，也不触发运行时 SafeStop。

Idle 保持策略：

- 初始 Idle 和正常暂停：每周期跟随当前反馈，属于软保持。
- 从 CommsLoss 或 SafeStop 恢复：保留安全路径锁存目标，属于硬保持。

## 4. 巡航和越障触发

下位机巡航请求：

```text
0 = Stop
1 = Forward，左右轮发送 +0.10
2 = Reverse，左右轮发送 -0.10
```

只有外层为 `Traversing` 且没有活动越障时允许非零轮速。活动越障阶段会强制停轮。

越障使用下位机 `obstacle_crossing_trigger`：

- 新的 `0 -> 1` 上升沿启动越障。
- 信号应保持为 1，直到进入 `CompleteWaitObstacleClear`。
- 进入完成等待后新的 `1 -> 0` 下降沿结束本轮越障。
- 越障中途提前出现的下降沿会被丢弃。

首侧由下位机反馈帧中最近一次非零的 `cruise_drive_request` 决定：Forward 先 Left，Reverse 先 Right。停车帧不覆盖方向记忆；如果尚无非零方向则保持 `Idle`。轮位置累计值只用于诊断。

## 5. PlannerControl

normal 模式使用三个接口：

| 接口 | 作用 |
|---|---|
| `planner_control_state` | 发布 session、序号 ACK、限制和退出原因 |
| `planner_joint_command` | 接收一个四关节位置点 |
| `complete_planner_control` | Adapter 请求正常结束会话 |

固定关节顺序：

```text
[left_first, left_second, right_first, right_second]
```

SMC 会检查 session、严格递增 sequence、时间戳、有限值和单点最大变化。合法点只有成为 effective command 并由 dispatcher 写入句柄后才 ACK，因此下一点不会覆盖尚未下发的点。

两个超时：

- `command_timeout`：当前 Planner 点不再 fresh，但仍允许接收同会话下一点。
- `total_watchdog_timeout`：整个 PlannerControl 超时，进入人工介入。

完成后进入 `RemoteControl`，不是直接进入 `Regrip`。

debug 模式不接收正式 Planner 点。只有测试已让控制器进入 PlannerControl 时，`planner_release` 才能手动放行。当前 debug 标准链停在 `DisconnectDoneWaitFlip`。

## 6. RemoteControl 和 Regrip

RemoteControl 数据只来自下位机反馈帧。四关节原始增量按以下顺序处理：

```text
死区 -> 缩放 -> 方向符号 -> 单样本限幅 -> 累加目标
```

顺序为 `[左一, 左二, 右一, 右二]`，当前方向符号为 `[-1, +1, +1, +1]`。每个反馈样本只应用一次。

RemoteControl 没有业务超时。完成位必须在当前阶段产生新的 `0 -> 1` 上升沿，然后进入 Regrip。

Regrip 要求在当前阶段先看到 `grip_confirmed=false`，再看到新的 `false -> true`。当前 Regrip
使用独立的 `regrip_confirmation_timeout=20.0s`；超时但未达到 `retry_limit` 时，程序重新张开夹爪并回到
RemoteControl，达到上限后进入人工介入。曾加入的回夹后双 first 归零阶段已因实机效果不好删除，当前确认成功后直接切换侧别或进入完成等待。

## 7. 输出模式

在 `b29_control/config/controller.yaml` 设置：

```yaml
output_mode: "normal"    # 或 "safe_hold"
```

- `normal`：执行状态机生成的命令。
- `safe_hold`：轮速强制为 0，六个位置关节保持当前反馈。

`safe_hold` 仍然运行硬件节点和串口，也不会关闭已经锁存的重力补偿。它不能代替硬件急停。

## 8. 启动和观察

完整正式启动：

```bash
roslaunch b29_control start.launch mode:=dual_role planner_mode:=normal
```

调试模式：

```bash
roslaunch b29_control start.launch mode:=dual_role \
  planner_mode:=debug \
  launch_single_flip_moveit:=false \
  launch_automatic_flip:=false
```

两个命令都会连接真机。调试步骤见
[`b29_smc_obstacle_crossing_debug_validation.md`](../../b29_control/docs/b29_smc_obstacle_crossing_debug_validation.md)。

观察：

```bash
rostopic echo /b29_controller/b29_smc_auto_controller/state_trace
rostopic echo /b29_controller/b29_smc_auto_controller/planner_control_state
rosrun rqt_b29_smc_console rqt_b29_smc_console
```

## 9. SMC 代码生成

状态机定义：`sm/RobotFSM.sm`。生成文件位于 `gen/`。

在本包目录执行：

```bash
java -jar third_party/smc/bin/Smc.jar \
  -c++ -d gen/src -headerd gen/include sm/RobotFSM.sm
```

生成状态图：

```bash
java -jar third_party/smc/bin/Smc.jar -graph -glevel 1 -d gen sm/RobotFSM.sm
dot -Tpng gen/RobotFSM_sm.dot -o gen/RobotFSM_sm.png
```

修改 `.sm` 后必须重新生成并重新构建。

## 10. 构建和当前限制

```bash
catkin build b29_control b29_smc_auto_controller
```

当前需要注意：

- `CloseBothGrippers` 是保留类型，当前自动入口不使用。
- 单侧夹爪没有独立张开反馈，使用固定等待时间。
- 脱缆 Step7 没有超时，速度不稳定时会一直等待。
- `temporary_allow_start_without_grip_confirmed` 当前默认开启，只绕过独立 `start_disconnect` 的入口检查。
- 当前没有 Gazebo 启动入口，也没有纯离线 `dry_run`。
- 软件停止不能替代下位机 watchdog 和硬件急停。

更完整的工程风险见根目录 [`B29_AUTOMATION_HANDOFF.md`](../../B29_AUTOMATION_HANDOFF.md)。
