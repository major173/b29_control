# B29 越障调试验证模式

## 1. 目的与边界

该模式用于在 Planner 和正式障碍物传感器尚未接入时验证：

- RobotFSM：`Idle / AutoInit / Traversing / CommsLoss / SafeStop`
- 越障主 FSM：双夹爪闭合、左右脱缆、PlannerControl、回夹、完成等待障碍物清除
- 脱缆子流程：Step1 至 Step8、插值、Step7 判定、失败回零和 retry
- PlannerControl 手动等待与放行
- ManualIntervention 恢复映射
- 软件急停、人工复位和 Ctrl+C 优雅退出
- effective command 和 `CommandDispatcher` 写入尝试结果

调试模式复用正式控制链：

```text
input -> AutoInputMux -> RobotContext::tick50Hz()
      -> buildEffectiveCommand()
      -> command_dispatcher_.dispatch(effective)
      -> state_trace
```

不复制 FSM，不提供直接跳转状态接口。`simulation_only` 是界面标识，不是硬隔离。禁止在实机正式 launch 中加载调试 overlay。

## 2. 参数与启动

正式默认值位于 `b29_control/config/controller.yaml`：

```yaml
debug_validation:
  enabled: false
  simulation_only: false
planner_control:
  manual_release_enabled: false
```

正式启动默认忽略 `debug_override`：

```bash
roslaunch b29_control start.launch
```

Gazebo 调试启动会显式加载 `controller_gazebo_smc_debug_validation.yaml`：

```bash
roslaunch b29_control start_smc_in_gazebo.launch
```

默认 `debug_output_mode=normal`，会推动 Gazebo 机构。只验证 trace 和仲裁时使用：

```bash
roslaunch b29_control start_smc_in_gazebo.launch debug_output_mode:=safe_hold
```

关闭调试门禁：

```bash
roslaunch b29_control start_smc_in_gazebo.launch enable_debug_validation:=false
```

启动 GUI：

```bash
rosrun rqt_b29_smc_console rqt_b29_smc_console
```

观察 trace：

```bash
rostopic echo /b29_controller/b29_smc_auto_controller/state_trace
```

## 3. 障碍物调试输入

```bash
OVERRIDE_TOPIC=/b29_controller/b29_smc_auto_controller/debug_override
```

障碍物字段 mask：

```text
FIELD_OBSTACLE_DETECTED = 512
FIELD_RANGE_TO_OBSTACLE = 4096
field_mask = 4608
```

未检测到障碍或清除障碍：

```bash
rostopic pub -1 "$OVERRIDE_TOPIC" b29_smc_auto_controller/AutoDebugOverride \
'{enabled: true, field_mask: 4608, obstacle_detected: false, range_to_obstacle: 0.0}'
```

远距离障碍，不进入当前 `0.30m` 翻越阈值：

```bash
rostopic pub -1 "$OVERRIDE_TOPIC" b29_smc_auto_controller/AutoDebugOverride \
'{enabled: true, field_mask: 4608, obstacle_detected: true, range_to_obstacle: 0.50}'
```

阈值内障碍：

```bash
rostopic pub -1 "$OVERRIDE_TOPIC" b29_smc_auto_controller/AutoDebugOverride \
'{enabled: true, field_mask: 4608, obstacle_detected: true, range_to_obstacle: 0.20}'
```

每条 override 消息会替换上一条覆盖配置。需要同时保持 `lower_alive`、`posture_ready` 或 `grip_confirmed` 时，必须合并字段和 mask。GUI 的 Override Composer 会自动组合 mask。

## 4. PlannerControl 手动放行

调试 overlay 默认启用：

```yaml
planner_control:
  manual_release_enabled: true
```

进入 `PlannerControl` 后不会再因固定时间自动进入 `Regrip`。每次进入阶段会清除旧 release，必须在当前阶段重新调用：

```bash
rosservice call /b29_controller/b29_smc_auto_controller/planner_release
```

非 `PlannerControl` 阶段调用会返回 `success=false`。正式模式中 `manual_release_enabled=false`，仍按 `wait_for_planner_point_control_time` 自动进入 `Regrip`。

重点观察：

```text
planner_control_active
planner_manual_release_enabled
planner_release_received
planner_control_wait_elapsed_sec
planner_point_available
planner_point_fresh
planner_override_applied
```

## 5. 软件急停与人工复位

软件急停服务始终可用，不受调试门禁影响：

```bash
rosservice call /b29_controller/b29_smc_auto_controller/software_emergency_stop
```

控制器会锁存急停，并在下一个 50Hz 控制周期进入 `SafeStop`：

- 轮速清零
- 停止 Planner 覆盖
- 清空越障 FSM 和脱缆插值
- 清空 Planner 最新点
- 由现有冻结语义保持关节

解除锁存并发出单周期人工复位请求：

```bash
rosservice call /b29_controller/b29_smc_auto_controller/manual_reset
```

若 IMU、通信或关节故障仍存在，RobotFSM 不会恢复运行。

硬件节点在 Ctrl+C、SIGTERM 和正常 ROS shutdown 后调用一次 `safeStopAndWrite()`：轮速和速度目标清零，位置目标保持当前反馈，并尝试最后一次串口写入。`SIGKILL`、进程崩溃、断电和内核故障无法由进程自身保证安全写入，必须依赖下位机 watchdog、硬件急停或断电保护。

## 6. GUI

`rqt_b29_smc_console` 增加验证 tabs：

- `Overview`：RobotFSM、基础命令、输出模式、调试标识
- `Crossing`：主 FSM、脱缆 Step、判定值、retry、人工干预、PlannerControl
- `Command & Safety`：effective command、六关节目标、dispatch 结果、急停锁存
- `Inputs`：调试输入和 RobotContext 输入
- `Actions`：急停、reset、Planner release、障碍物输入、合法事件模拟

动作按钮会影响仿真机构。`EMERGENCY STOP` 不弹确认框；`Manual Reset` 和 `Planner Release` 会要求确认。GUI 启动失败不会影响 controller。

## 7. Trace 验收重点

越障进入阈值后：

```text
obstacle_crossing_stage = CloseBothGrippers
stop_all = true
left_gripper_target = CLOSED
right_gripper_target = CLOSED
```

脱缆过程：

```text
disconnect_step
disconnect_step_elapsed_sec
disconnect_step_transition_reason
disconnect_step_expected_condition
disconnect_check_displacement
to_check_joint_pos
succeed_disconnect_cable_threshold
```

Step7 使用：

```text
abs(current_second_joint_feedback - to_check_joint_pos)
  >= succeed_disconnect_cable_threshold
```

失败时先进入 `Step8ReturnToZero`，回零完成后才增加 retry。达到上限后进入 `ManualIntervention`。

人工确认条件统一为：

```text
grip_confirmed == true
```

恢复映射：

| 失败动作 | 人工确认后的下一阶段 |
| --- | --- |
| `CloseBothGrippers` | `FirstSide Disconnecting` |
| `Left DisconnectCable` | `Left PlannerControl` |
| `Left PlannerRegrip` | `Right Disconnecting` |
| `Right DisconnectCable` | `Right PlannerControl` |
| `Right PlannerRegrip` | `CompleteWaitObstacleClear` |

首侧由巡航速度符号决定：正速度先左侧，负速度先右侧。

## 8. 推荐验证顺序

1. 使用 `debug_output_mode:=safe_hold` 验证 RobotFSM、override 门禁、急停和 reset。
2. 使用 `debug_output_mode:=normal` 验证夹爪命令、Step3/4/5/6 插值和 Step8 回零。
3. 阈值内障碍进入 `CloseBothGrippers` 后，使用 `grip_confirmed` override 推进闭合确认。
4. 等待 Step7 成功进入 `PlannerControl`，确认等待时间持续增长且不自动离开。
5. 调用 `planner_release`，确认进入 `Regrip`。
6. 完成双侧流程后清除障碍，确认回到 `Idle` 且不重复触发。
7. 在任意阶段调用软件急停，确认进入 `SafeStop` 且越障内部状态清空。

可使用场景脚本：

```bash
rosrun b29_smc_auto_controller replay_scenario.py \
  _scenario:=b29_controllers/b29_smc_auto_controller/scenarios/software_emergency_stop_and_reset.yaml
```

`obstacle_crossing_manual_planner_release.yaml` 中的 release 时间仅用于 Gazebo 初始参考。若机构反馈时序不同，服务会明确拒绝提前放行；此时使用 GUI 在实际 `PlannerControl` 阶段手动放行。

## 9. 常见失败原因

- `debug_override` 无效：检查 `debug_validation_enabled` 是否为 `true`。
- Planner release 被拒绝：当前不在 `PlannerControl`，或 `planner_manual_release_enabled=false`。
- Planner 点不覆盖：检查 `planner_point_available`、`planner_point_fresh` 和 `planner_override_applied`。
- `safe_hold` 下 Step7 不成功：该模式不会推动机构，使用 Gazebo `normal` 验证物理反馈路径。
- reset 后仍在 `SafeStop`：检查 `lower_alive`、`imu_ready`、`joint_fault` 和 `grip_fault`。
- 完成后重复翻越：清除障碍，使 `obstacle_detected=false`。

## 10. 构建与静态检查

```bash
git diff --check
python3 -m py_compile \
  b29_controllers/b29_smc_auto_controller/scripts/replay_scenario.py \
  b29_control/scripts/validate_b29_planner_hardware_control.py \
  b29_tools/rqt_b29_smc_console/src/rqt_b29_smc_console/*.py
catkin build b29_smc_auto_controller rqt_b29_smc_console b29_control
```

修改 `AutoStateTrace.msg` 会改变 ROS message MD5。依赖节点必须统一重新构建并同时部署。
