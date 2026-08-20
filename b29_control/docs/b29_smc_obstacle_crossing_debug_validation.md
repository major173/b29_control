# B29 越障调试验证

本文给出当前工程可直接执行的调试步骤。第一次接手请先阅读根目录
[`B29_AUTOMATION_HANDOFF.md`](../../B29_AUTOMATION_HANDOFF.md)。

## 1. 先理解边界

- `start.launch` 会连接真机，不是仿真入口。
- `planner_mode:=debug` 开启调试输入，但不会注册正式 `start_flip` 接口。
- debug 标准流程最多运行到 `DisconnectDoneWaitFlip`。
- `Planner Release` 只能结束已经进入的 `PlannerControl`，不能代替 `start_flip`。
- RemoteControl 数据始终来自下位机，GUI 和 debug topic 不能伪造。
- `safe_hold` 仍会连接硬件，只是停轮并保持关节，不能代替硬件急停。

## 2. 启动

修改 `b29_control/config/controller.yaml`：

```yaml
output_mode: "safe_hold"
```

然后启动 debug 模式：

```bash
source ~/桌面/B29_ws/devel/setup.bash
roslaunch b29_control start.launch \
  planner_mode:=debug \
  launch_single_flip_moveit:=false \
  launch_automatic_flip:=false
```

打开 GUI：

```bash
rosrun rqt_b29_smc_console rqt_b29_smc_console
```

或直接观察 trace：

```bash
rostopic echo /b29_controller/b29_smc_auto_controller/state_trace
```

## 3. 调试输入

调试 topic：

```text
/b29_controller/b29_smc_auto_controller/debug_override
```

建议使用 GUI 的 `Override Composer`，它会自动组合 `field_mask`。手工发布时，新消息会替换旧覆盖配置；需要同时保持多个字段时，必须把字段和 mask 合并到同一条消息。

下面的 mask 同时覆盖巡航请求和越障触发：

```text
FIELD_OBSTACLE_CROSSING_TRIGGER = 512
FIELD_CRUISE_DRIVE_REQUEST = 4096
field_mask = 4608
```

设置正轮速巡航，越障触发保持低：

```bash
rostopic pub -1 /b29_controller/b29_smc_auto_controller/debug_override \
  b29_smc_auto_controller/AutoDebugOverride \
  '{enabled: true, field_mask: 4608, obstacle_crossing_trigger: false, cruise_drive_request: 1}'
```

产生越障上升沿：

```bash
rostopic pub -1 /b29_controller/b29_smc_auto_controller/debug_override \
  b29_smc_auto_controller/AutoDebugOverride \
  '{enabled: true, field_mask: 4608, obstacle_crossing_trigger: true, cruise_drive_request: 1}'
```

巡航请求含义：

```text
0 = Stop
1 = Forward，左右轮发送正值
2 = Reverse，左右轮发送负值
```

越障触发必须保持为高，直到控制器进入 `CompleteWaitObstacleClear`。进入该阶段后再产生 `1 -> 0` 下降沿。同一轮越障中提前发送的下降沿会被丢弃。

## 4. 推荐验证顺序

1. 保持 `safe_hold`，检查 `debug_validation_enabled=true`。
2. 用 GUI 触发 AutoStart，确认 `Idle -> AutoInit -> Traversing`。
3. 设置巡航请求，确认 trace 中出现对应轮速意图。
4. 产生越障上升沿，确认进入 `OpenGripperBeforeGravityCompensation`。
5. 检查当前侧夹爪张开、另一侧夹爪闭合、轮速为零。
6. 等待进入 `EnableGravityCompensation` 和 `Disconnecting`。
7. 在受控真机条件下改为 `output_mode: normal`，验证 Step3/Step5 实际运动。
8. 确认 Step7 速度稳定后进入 `DisconnectDoneWaitFlip`。
9. debug 模式在此停止属于预期行为。
10. 在任意阶段测试软件急停和人工复位。

需要验证完整 Planner、RemoteControl、Regrip 和第二侧流程时，使用 `planner_mode:=normal` 和正式 GP11/Adapter 链。

## 5. 各阶段看什么

### 越障开始

```text
obstacle_crossing_stage = OpenGripperBeforeGravityCompensation
stop_all = true
current side gripper_target = OPEN
other side gripper_target = CLOSED
```

### 脱缆

重点字段：

```text
disconnect_step
disconnect_step_elapsed_sec
disconnect_max_abs_pose_joint_velocity
disconnect_settle_velocity_threshold
disconnect_velocity_stable_elapsed_sec
```

Step7 要求三个位姿关节最大绝对速度连续一段时间不超过阈值。当前没有 Step7 超时，速度不稳定时会一直等待。

### PlannerControl

normal 模式重点看：

```text
planner_control_active
planner_point_available
planner_point_fresh
planner_override_applied
```

如果专用测试已经让 debug 模式进入 `PlannerControl`，可以手动放行：

```bash
rosservice call /b29_controller/b29_smc_auto_controller/planner_release
```

服务成功后进入 `RemoteControl`。不在该阶段时会返回 `success=false`。

### RemoteControl

下位机四关节增量顺序为：

```text
[左一, 左二, 右一, 右二]
```

原始增量依次经过死区、缩放、方向映射和单样本限幅。GUI 中：

- `remote_control_raw_increments`：下位机原始值。
- `remote_control_applied_increments`：实际累加值。

当前方向符号为 `[-1, +1, +1, +1]`。完成位必须在当前阶段产生新的 `0 -> 1` 上升沿。

## 6. 软件急停和复位

软件急停：

```bash
rosservice call /b29_controller/b29_smc_auto_controller/software_emergency_stop
```

控制器会在下一个控制周期进入 `SafeStop`，停轮并停止 Planner、脱缆和越障覆盖。

人工复位：

```bash
rosservice call /b29_controller/b29_smc_auto_controller/manual_reset
```

通信、IMU 或硬件故障仍存在时不会恢复。Ctrl+C、进程崩溃和断电不能依靠软件服务保证安全，必须使用下位机 watchdog 和硬件急停。

## 7. 首侧方向与人工恢复

首侧使用下位机通信帧中最近一次非零的 `cruise_drive_request`：`Forward (1)` 先左侧，
`Reverse (2)` 先右侧。翻越前的 `Stop (0)` 或非法值不会覆盖方向记忆；如果尚无历史非零方向，保持 `Idle`，不启动动作。
trace 中的 `signed_wheel_travel` 和轮位置基准仅保留用于诊断，不参与首侧判断。

人工恢复必须确认：

```text
grip_confirmed == true
```

| 失败动作 | 人工确认后的下一阶段 |
|---|---|
| `CloseBothGrippers` | 兼容状态超时直接人工介入，不自动重试 |
| `DisconnectCable` / `PlannerControl` / `Regrip`，当前为首侧 | 当前侧视为已人工完成并回夹，开始另一侧 |
| `DisconnectCable` / `PlannerControl` / `Regrip`，当前为第二侧 | 当前侧视为已人工完成并回夹，进入 `CompleteWaitObstacleClear` |

## 8. 常见问题

| 现象 | 先检查什么 |
|---|---|
| override 没有效果 | `planner_mode=debug`、`debug_validation_enabled=true` |
| Workflow AutoStart 失败 | trace 的 `current_state`、`lower_alive`、`imu_ready` 和故障位 |
| 停在 `DisconnectDoneWaitFlip` | debug 模式下这是预期终点 |
| Planner Release 被拒绝 | 当前是否真的处于 `PlannerControl` |
| Step7 一直不结束 | 三个位姿关节速度和稳定累计时间 |
| RemoteControl 不结束 | 下位机完成位是否先回 0，再产生新上升沿 |
| reset 后仍在 SafeStop | 通信、IMU、关节和夹爪故障是否已恢复 |
| GUI 与 debug topic 字段不同 | GUI 显示的是合并后的 trace，不是原始 override 消息 |

`AutoStateTrace.msg` 中的障碍物距离相关字段是兼容遗留字段。当前越障只看 `obstacle_crossing_trigger` 及其边沿，不使用距离。

## 9. 离线检查

```bash
git diff --check
python3 -m py_compile \
  b29_controllers/b29_smc_auto_controller/scripts/replay_scenario.py \
  b29_tools/rqt_b29_smc_console/src/rqt_b29_smc_console/*.py
catkin build b29_control b29_smc_auto_controller b29_planner_adapter rqt_b29_smc_console
```

这些命令不会启动 ROS。修改消息定义后，所有依赖包必须一起重新构建。
