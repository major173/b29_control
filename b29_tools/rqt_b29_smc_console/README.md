# rqt_b29_smc_console

这是 B29 SMC 的监控和调试 GUI。它把状态、输入覆盖、预设流程和安全按钮集中在一个窗口中。

GUI 不是控制器。GUI 关闭或连接失败不会停止 SMC；按钮产生的请求仍要经过正式状态机和命令仲裁。

## 1. 启动

```bash
catkin build rqt_b29_smc_console --no-deps
source ~/桌面/B29_ws/devel/setup.bash
rosrun rqt_b29_smc_console rqt_b29_smc_console
```

默认 namespace：

```text
/b29_controller/b29_smc_auto_controller
```

界面支持中英文切换和滚动。代码中的状态名、事件名和字段名不会翻译，便于对照日志。

## 2. 页面说明

| 页面 | 主要内容 |
|---|---|
| `Overview` | RobotFSM、最近事件、命令原因和运行模式 |
| `Crossing` | 越障阶段、侧别、脱缆 Step、retry、Planner 和 RemoteControl |
| `Command & Safety` | 最终轮速、关节/夹爪目标、dispatch 和急停状态 |
| `Inputs` | SMC 合并后的通信、IMU、夹爪、巡航和越障输入 |
| `Actions` | 急停、复位、Planner Release 和调试事件 |
| `Trace` | 关键状态变化或原始 trace 流 |

`Command & Safety` 显示的是控制器下发意图，不是执行器真实反馈。真实关节位置和速度查看 `/joint_states`。

## 3. 三种输入工具

### Sensor Input

向 `sensor_input` topic 发布完整输入。只有 controller 配置为：

```text
use_sensor_input=true
use_auto_state=false
```

时才会成为正式基础输入。当前真机默认使用 `AutoStateInterface`，因此 topic 有消息不代表控制器采用了它。

### Override Composer

用于 debug 模式覆盖指定字段：

- `触发一次`：先置高，短暂保持后自动释放。
- `持续覆盖`：保持指定值。
- `取消覆盖`：恢复底层真实输入。

面板会自动生成 `field_mask`。每条 override 消息会替换上一条覆盖配置，因此建议通过 Composer 组合多个字段。

### Workflow

用于执行预设状态转换，例如 AutoStart、急停和复位。Workflow 只是顺序发送输入并等待 trace，不会强制修改 FSM 状态。

已知限制：

- 真机默认不采用 workflow 中的 `sensor_input` 步骤。
- `CommsLoss -> Idle` 预设同时令 `imu_ready=false`，实际会优先进入 SafeStop，不适合作为 CommsLoss 验收。
- AutoInit 可能只持续一个控制周期，因此启动 workflow 直接等待最终 Traversing。

## 4. normal 和 debug

### normal

- 正式 Planner、GP11 和 Adapter 可用。
- debug override 被忽略。
- Planner Release 被拒绝。

### debug

- debug override 生效。
- 标准越障链停在 `DisconnectDoneWaitFlip`。
- 没有 `start_flip`，所以 Planner Release 不能把该阶段变成 PlannerControl。
- 只有专用测试已经进入 PlannerControl 时，Planner Release 才有效。

RemoteControl 增量和完成信号在两种模式下都只来自下位机，GUI 不能伪造。

## 5. 安全按钮

- `EMERGENCY STOP`：立即请求软件急停，不弹确认框。
- `Manual Reset`：解除软件锁存并请求状态机复位，需要确认。
- `Planner Release`：debug 模式手动结束当前 PlannerControl，需要确认。

按钮可能改变真机状态。软件急停不能代替硬件急停；通信、IMU 或硬件故障仍存在时，Manual Reset 不会恢复运行。

## 6. 越障触发规则

- 新的 `0 -> 1` 上升沿启动越障。
- 触发必须保持为高，直到 `CompleteWaitObstacleClear`。
- 进入完成等待后，使用 `Clear Trigger` 产生下降沿。
- 越障中途提前拉低不会被延后记住。
- 巡航请求：`0=Stop`、`1=Forward`、`2=Reverse`。

## 7. Trace 页面

- `Key Events`：只显示关键字段变化，重复采样会折叠。
- `Raw`：显示每条原始 `state_trace`。
- `Pause`：暂停列表，顶部最新状态仍会更新。
- `Clear`：清空当前显示。

SafeStop 和 CommsLoss 会高亮。排查问题时优先记录：

```text
current_state
obstacle_crossing_stage
last_event
transition_reason
command_reason
lower_alive
imu_ready
software_emergency_stop_latched
```

## 8. GUI 没有数据

1. 检查 `state_trace` 是否存在并持续发布。
2. 检查顶部 namespace 是否正确。
3. 多机环境检查 ROS master、主机名解析、双向网络和防火墙。
4. 检查所有机器是否 source 了相同版本的工作空间。
5. 不要用原始 `debug_override` 消息直接对比 GUI Inputs；GUI 显示的是控制器合并后的 trace。

## 9. 离线模式

缺少 ROS 运行时依赖时，插件会显示最小界面，但不会：

- 发布 sensor input 或 override。
- 执行 workflow。
- 调用急停、复位或 Planner Release 服务。
- 订阅真实 trace。

离线模式只用于检查界面布局，不能替代控制器联调。
