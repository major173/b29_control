# b29_smc_auto_controller Gazebo 调试流程

## 目标

本文档用于指导当前 `b29_smc_auto_controller` 在 Gazebo 中做状态机联调，覆盖：

- 如何启动 Gazebo 专用 FSM 调试链路
- 如何使用 `/b29_controller/b29_smc_auto_controller/sensor_input`
- 如何使用 `/b29_controller/b29_smc_auto_controller/debug_override`
- 当前已实现状态、事件的逐项验证方法
- 每一步的验收标准

当前文档只覆盖已经实现的状态机范围：

- 状态：`Idle`、`AutoInit`、`Traversing`、`CommsLoss`、`SafeStop`
- 事件：`evAutoStart`、`evTick`、`evCommsLost`、`evCommsRestored`、`evReconnectTimeout`、`evEmergencyStop`、`evManualReset`

当前不覆盖的状态：

- `ObstacleDetected`
- `Approaching`
- `PreObstacleStop`
- `CrossingLineClamp`
- `CrossingDamper`
- `PostObstacleCheck`

这些状态在当前版本里还没有展开成独立状态，因此不能按“状态切换”验收。

## 前提

- 已完成工作区构建：

```bash
cd /home/yuchen/usetest/B29
catkin build b29_smc_auto_controller b29_control
source /home/yuchen/usetest/B29/devel/setup.bash
```

- 当前 Gazebo 调试专用启动链路是：

```bash
roslaunch b29_control start_smc_in_gazebo.launch
```

这个 launch 的设计点是：

- 只启动 `joint_state_controller`
- 只启动 `robot_state_controller`
- 只启动 `b29_smc_auto_controller`
- `output_mode` 固定为 `safe_hold`

所以它验证的是状态机和输入/追踪链路，不验证真实关节动作执行。

## Topic 说明

当前调试主要看 3 个 topic：

- `/b29_controller/b29_smc_auto_controller/state_trace`
- `/b29_controller/b29_smc_auto_controller/sensor_input`
- `/b29_controller/b29_smc_auto_controller/debug_override`

推荐先开一个终端持续观察 trace：

```bash
source /home/yuchen/usetest/B29/devel/setup.bash
rostopic echo /b29_controller/b29_smc_auto_controller/state_trace
```

主验收字段优先看：

- `current_state`
- `transition_reason`
- `command_reason`
- `output_mode`
- `stop_all`
- `freeze_joints`

注意：

- `last_event` 当前更适合作为辅助信息，不建议把它当成唯一判据，因为故障类信息可能会保留上一条记录。
- `AutoInit` 是瞬时态，`50Hz` 下通常只持续一个周期，肉眼容易错过。
- 如果要验证 `AutoInit`，优先看连续 trace 或看 `transition_reason` 是否先出现 `Idle->AutoInit`，再出现 `AutoInit->Traversing`。

## 两类输入的职责分工

### `sensor_input`

适合提供持续状态量：

- `lower_alive`
- `imu_ready`
- `grip_confirmed`
- `joint_fault`
- `grip_fault`
- `obstacle_detected`
- `obstacle_type`
- `classification_stable`
- `range_to_obstacle`
- `at_crossing_position`
- `post_check_passed`
- `post_check_failed`

### `debug_override`

适合提供事件脉冲或强制覆盖：

- `auto_start_requested`
- `manual_reset_requested`
- `emergency_stop`
- `posture_ready`
- 以及任何需要临时强制覆盖的字段

非常重要：

- `debug_override` 不是“一次性事件总线”，而是“当前覆盖配置”。
- 你发布一次 `enabled=true` 的覆盖后，这个覆盖会一直保留，直到你下一次发布新覆盖把它改掉或关闭。
- 所以对 `auto_start_requested`、`manual_reset_requested`、`emergency_stop` 这种事件型字段，必须显式发“释放脉冲”的消息。

## 当前常用 `field_mask`

只列当前联调最常用的值：

- `FIELD_AUTO_START_REQUESTED = 1`
- `FIELD_MANUAL_RESET_REQUESTED = 2`
- `FIELD_EMERGENCY_STOP = 4`
- `FIELD_LOWER_ALIVE = 8`
- `FIELD_IMU_READY = 16`
- `FIELD_POSTURE_READY = 32`
- `FIELD_GRIP_CONFIRMED = 64`
- `FIELD_OBSTACLE_DETECTED = 512`
- `FIELD_OBSTACLE_TYPE = 1024`

## 推荐的基础命令

定义 3 个环境变量，后面命令会短很多：

```bash
CTRL_NS=/b29_controller/b29_smc_auto_controller
TRACE_TOPIC=$CTRL_NS/state_trace
SENSOR_TOPIC=$CTRL_NS/sensor_input
OVERRIDE_TOPIC=$CTRL_NS/debug_override
```

### 1. 清空 override

```bash
rostopic pub -1 "$OVERRIDE_TOPIC" b29_smc_auto_controller/AutoDebugOverride \
'{enabled: false, field_mask: 0}'
```

### 2. 发布基础 `sensor_input`

这条命令用于把系统带到“通信正常、IMU 正常、夹紧确认、无障碍”的基础工况：

```bash
rostopic pub -1 "$SENSOR_TOPIC" b29_smc_auto_controller/AutoSensorInput \
'{
  lower_alive: true,
  imu_ready: true,
  grip_confirmed: true,
  joint_fault: false,
  grip_fault: false,
  obstacle_detected: false,
  obstacle_type: 0,
  classification_stable: false,
  range_to_obstacle: 0.0,
  at_crossing_position: false,
  post_check_passed: false,
  post_check_failed: false
}'
```

### 3. 如果 IMU 初始姿态不稳定，强制 `posture_ready=true`

当前 `posture_ready` 默认来自 Gazebo IMU 姿态判定。若机器人初始姿态导致始终无法启动自动流程，可临时强制覆盖：

```bash
rostopic pub -1 "$OVERRIDE_TOPIC" b29_smc_auto_controller/AutoDebugOverride \
'{
  enabled: true,
  field_mask: 32,
  posture_ready: true
}'
```

如果你这样做了，后续所有新的 override 消息如果还想继续保留 `posture_ready=true`，就必须继续把 `32` 这个 bit 带上。

## 推荐调试顺序

建议严格按下面顺序联调：

1. 先验证 `CommsLoss -> Idle`
2. 再验证 `Idle -> AutoInit -> Traversing`
3. 再验证 `evEmergencyStop -> SafeStop`
4. 最后验证 `SafeStop -> Idle`

原因是：

- 控制器默认没有业务输入时，`lower_alive=false`
- 所以系统很容易先进入 `CommsLoss`
- 从 `CommsLoss` 拉回 `Idle` 后，再测后面的链路会更稳定

## 调试步骤与验收标准

### 步骤 0：启动 Gazebo FSM 调试链路

```bash
source /home/yuchen/usetest/B29/devel/setup.bash
roslaunch b29_control start_smc_in_gazebo.launch
```

验收标准：

- `rostopic list | grep b29_smc_auto_controller` 能看到：
  - `/b29_controller/b29_smc_auto_controller/debug_override`
  - `/b29_controller/b29_smc_auto_controller/sensor_input`
  - `/b29_controller/b29_smc_auto_controller/state_trace`
- `state_trace.output_mode` 为 `safe_hold`

### 步骤 1：验证 `evCommsLost`

#### 触发方式

当前默认没有业务输入时，`lower_alive=false`，控制器在启动后通常会很快进入 `CommsLoss`。

如果你想显式触发一次，也可以发送：

```bash
rostopic pub -1 "$SENSOR_TOPIC" b29_smc_auto_controller/AutoSensorInput \
'{
  lower_alive: false,
  imu_ready: false,
  grip_confirmed: false,
  joint_fault: false,
  grip_fault: false,
  obstacle_detected: false,
  obstacle_type: 0,
  classification_stable: false,
  range_to_obstacle: 0.0,
  at_crossing_position: false,
  post_check_passed: false,
  post_check_failed: false
}'
```

#### 对应事件

- `evCommsLost`

#### 预期状态

- `CommsLoss`

#### 验收标准

- `current_state == "CommsLoss"`
- `output_mode == "safe_hold"`
- `stop_all == true`
- `freeze_joints == true`
- `CommsLoss` 进入后就开始累计重连超时计数

### 步骤 2：验证 `evCommsRestored`

#### 触发方式

发布基础 `sensor_input`，把 `lower_alive` 拉回 `true`：

```bash
rostopic pub -1 "$SENSOR_TOPIC" b29_smc_auto_controller/AutoSensorInput \
'{
  lower_alive: true,
  imu_ready: true,
  grip_confirmed: true,
  joint_fault: false,
  grip_fault: false,
  obstacle_detected: false,
  obstacle_type: 0,
  classification_stable: false,
  range_to_obstacle: 0.0,
  at_crossing_position: false,
  post_check_passed: false,
  post_check_failed: false
}'
```

#### 对应事件

- `evCommsRestored`

#### 预期状态

- `Idle`

#### 验收标准

- `current_state == "Idle"`
- `transition_reason == "CommsLoss->Idle"` 或至少观察到状态从 `CommsLoss` 回到 `Idle`
- 状态恢复后不会自动重新进入 `AutoInit` 或 `Traversing`
- 这个恢复路径必须在 `evReconnectTimeout` 触发之前完成；一旦超时进入 `SafeStop`，仅靠 `lower_alive=true` 不会自动回到 `Idle`

### 步骤 2b：验证 `evReconnectTimeout`

#### 触发方式

保持 `lower_alive=false`，不要发送恢复输入，等待超过 5 秒。50Hz 下对应约 250 个 tick。

#### 对应事件

- `evReconnectTimeout`

#### 预期状态

- `SafeStop`

#### 验收标准

- `current_state == "SafeStop"`
- `transition_reason == "CommsLoss->SafeStop"`
- `command_reason == "reconnect_timeout"`
- `last_event == "reconnect_timeout"`
- 超时后再把 `lower_alive` 拉回 `true`，状态仍应停留在 `SafeStop`，直到人工复位

### 步骤 3：验证 `Idle`

#### 触发方式

- 保持步骤 2 的基础 `sensor_input`
- 清空 override

```bash
rostopic pub -1 "$OVERRIDE_TOPIC" b29_smc_auto_controller/AutoDebugOverride \
'{enabled: false, field_mask: 0}'
```

#### 预期状态

- `Idle`

#### 验收标准

- `current_state == "Idle"`
- 不会自动前进到 `AutoInit`
- `output_mode == "safe_hold"`

### 步骤 4：验证 `evAutoStart`

#### 触发方式

如果 Gazebo 中 `posture_ready` 已经正常为 `true`，可以直接发自动启动脉冲：

```bash
rostopic pub -1 "$OVERRIDE_TOPIC" b29_smc_auto_controller/AutoDebugOverride \
'{
  enabled: true,
  field_mask: 1,
  auto_start_requested: true
}'
```

随后立即释放：

```bash
rostopic pub -1 "$OVERRIDE_TOPIC" b29_smc_auto_controller/AutoDebugOverride \
'{
  enabled: true,
  field_mask: 1,
  auto_start_requested: false
}'
```

如果 `Idle` 一直不进入 `AutoInit`，大概率是 `posture_ready` 没满足。这时改用下面的确定性版本：

```bash
rostopic pub -1 "$OVERRIDE_TOPIC" b29_smc_auto_controller/AutoDebugOverride \
'{
  enabled: true,
  field_mask: 33,
  auto_start_requested: true,
  posture_ready: true
}'
```

然后释放 `auto_start_requested`，但继续保留 `posture_ready=true`：

```bash
rostopic pub -1 "$OVERRIDE_TOPIC" b29_smc_auto_controller/AutoDebugOverride \
'{
  enabled: true,
  field_mask: 33,
  auto_start_requested: false,
  posture_ready: true
}'
```

#### 对应事件

- `evAutoStart`

#### 预期状态

- 瞬时进入 `AutoInit`

#### 验收标准

- 连续观察 trace 时能看到一次 `current_state == "AutoInit"`，或者
- 能看到 `transition_reason == "Idle->AutoInit"`

注意：

- `AutoInit` 是瞬时态，通常会在下一个 `evTick` 很快离开。
- 如果只看单次 `rostopic echo -n 1`，很容易直接看到 `Traversing`，这不代表 `AutoInit` 没发生。

### 步骤 5：验证 `evTick` 驱动 `AutoInit -> Traversing`

#### 触发方式

- 步骤 4 成功后，不需要额外发命令
- 只要 `lower_alive=true`、`imu_ready=true`、`grip_confirmed=true` 且 `posture_ready=true`，下一周期会自动推进

#### 对应事件

- `evTick`

#### 预期状态

- `Traversing`

#### 验收标准

- `current_state == "Traversing"`
- `transition_reason == "AutoInit->Traversing"` 或至少观察到状态从 `AutoInit` 进入 `Traversing`
- `command_reason` 会切到巡航相关命令

### 步骤 6：验证 `Traversing`

#### 触发方式

- 保持步骤 5 后的输入，不要再发故障或复位类指令

#### 预期状态

- `Traversing`

#### 验收标准

- `current_state == "Traversing"`
- 状态保持稳定，不自动跳回 `Idle` 或 `SafeStop`
- 当前因为 `output_mode=safe_hold`，即使状态机处于巡航态，也不会真的驱动 Gazebo 关节动作

### 步骤 7：验证 `Traversing` 下当前障碍输入行为

#### 触发方式

在 `Traversing` 状态下，发布障碍输入：

```bash
rostopic pub -1 "$SENSOR_TOPIC" b29_smc_auto_controller/AutoSensorInput \
'{
  lower_alive: true,
  imu_ready: true,
  grip_confirmed: true,
  joint_fault: false,
  grip_fault: false,
  obstacle_detected: true,
  obstacle_type: 1,
  classification_stable: true,
  range_to_obstacle: 0.2,
  at_crossing_position: false,
  post_check_passed: false,
  post_check_failed: false
}'
```

#### 对应事件

- 仍然是 `evTick`

#### 当前实现预期

- 继续停留在 `Traversing`

#### 验收标准

- `current_state` 仍然是 `Traversing`
- 不会进入 `SafeStop`

说明：

- 这一步的目的不是验证“越障状态切换”
- 只是验证当前实现里“检测到障碍后不会误入故障态”

### 步骤 8：验证 `evEmergencyStop`

#### 触发方式

发布急停脉冲：

```bash
rostopic pub -1 "$OVERRIDE_TOPIC" b29_smc_auto_controller/AutoDebugOverride \
'{
  enabled: true,
  field_mask: 4,
  emergency_stop: true
}'
```

随后释放：

```bash
rostopic pub -1 "$OVERRIDE_TOPIC" b29_smc_auto_controller/AutoDebugOverride \
'{
  enabled: true,
  field_mask: 4,
  emergency_stop: false
}'
```

#### 对应事件

- `evEmergencyStop`

#### 预期状态

- `SafeStop`

#### 验收标准

- `current_state == "SafeStop"`
- `stop_all == true`
- `freeze_joints == true`
- 无论当前在 `Idle`、`AutoInit` 还是 `Traversing`，都应该优先进入 `SafeStop`

### 步骤 9：验证 `SafeStop`

#### 触发方式

- 保持步骤 8 结束后的输入

#### 预期状态

- `SafeStop`

#### 验收标准

- `current_state == "SafeStop"`
- 状态不会自动恢复
- 即使 `lower_alive=true`，也不会自动回到 `Idle`

### 步骤 10：验证 `evManualReset`

#### 触发方式

仅在 `SafeStop` 中发布人工复位脉冲：

```bash
rostopic pub -1 "$OVERRIDE_TOPIC" b29_smc_auto_controller/AutoDebugOverride \
'{
  enabled: true,
  field_mask: 2,
  manual_reset_requested: true
}'
```

随后释放：

```bash
rostopic pub -1 "$OVERRIDE_TOPIC" b29_smc_auto_controller/AutoDebugOverride \
'{
  enabled: true,
  field_mask: 2,
  manual_reset_requested: false
}'
```

#### 对应事件

- `evManualReset`

#### 预期状态

- `Idle`

#### 验收标准

- `current_state == "Idle"`
- `transition_reason == "SafeStop->Idle"` 或至少观察到状态从 `SafeStop` 回到 `Idle`
- 回到 `Idle` 后不会自动重新启动自动流程

## 完整最小验收链

建议最终按下面顺序走一遍，作为当前版本的 Gazebo FSM 最小回归：

1. 启动 Gazebo
2. 确认 `state_trace.output_mode == safe_hold`
3. 默认或显式触发 `CommsLoss`
4. 用 `sensor_input.lower_alive=true` 验证 `CommsLoss -> Idle`
5. 用 `debug_override.auto_start_requested=true` 验证 `Idle -> AutoInit -> Traversing`
6. 在 `Traversing` 下发 `obstacle_detected=true`，确认当前仍停留在 `Traversing`
7. 用 `debug_override.emergency_stop=true` 验证 `* -> SafeStop`
8. 用 `debug_override.manual_reset_requested=true` 验证 `SafeStop -> Idle`

如果这 8 步全部通过，说明当前版本的 Gazebo 状态机最小闭环是成立的。

## 当前版本的验收边界

当前通过 Gazebo 可以确认的内容：

- 控制器能正常启动
- `sensor_input` 能驱动持续状态输入
- `debug_override` 能驱动事件脉冲和覆盖输入
- `state_trace` 能反映当前状态、主要转移和 `output_mode`
- `safe_hold` 模式下状态机可运行而不执行真实动作

当前通过 Gazebo 还不能确认的内容：

- 完整越障状态图
- 真实关节轨迹执行正确性
- 真实轮速闭环控制
- 实机侧通信时序和实时性问题

## 常见问题

### 1. 为什么一直在 `CommsLoss`

通常是因为你还没有发布 `sensor_input.lower_alive=true`。

### 2. 为什么发了自动启动还是不进 `AutoInit`

检查以下条件是否同时满足：

- `lower_alive == true`
- `imu_ready == true`
- `grip_confirmed == true`
- `posture_ready == true`

其中 `posture_ready` 默认来自 Gazebo IMU 姿态判定。如果不稳定，先用 `debug_override` 强制为 `true`。

### 3. 为什么我发过一次 override，后面状态一直不对

因为 `debug_override` 会持久保留，直到被下一条消息替换。对事件型字段一定要记得发“释放脉冲”。

### 4. 为什么在 `Traversing` 看不到机器人真的动

因为 Gazebo 调试链路固定使用 `output_mode=safe_hold`。这是设计目标，不是故障。

### 5. 为什么 `last_event` 看起来不像当前这一步

当前建议以 `current_state` 为主验收依据，`transition_reason` 和 `command_reason` 为辅，`last_event` 不作为唯一判据。
