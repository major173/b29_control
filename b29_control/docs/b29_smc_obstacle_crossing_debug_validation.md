# B29 越障调试验证模式

## 1. 目的与边界

该模式用于在 Planner 和正式障碍物传感器尚未接入时验证：

- RobotFSM：`Idle / AutoInit / Traversing / CommsLoss / SafeStop`
- 越障主 FSM：左右脱缆、PlannerControl、RemoteControl、回夹、完成等待障碍物清除
- 脱缆子流程：首侧夹爪张开、Step3/Step5 first joint 插值、Step7 速度稳定判定和失败人工保持
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

`start.launch` 的 `planner_mode` 是 SMC 与 Adapter 的唯一模式入口。调试时使用：

```bash
roslaunch b29_control start.launch planner_mode:=debug
```

该模式会同时启用 `debug_validation` 和 `planner_release`，不需要修改 SMC 或 Adapter 的 YAML。

正式 Planner 运行使用：

```bash
roslaunch b29_control start.launch planner_mode:=normal
```

`normal` 会关闭调试门禁和手动放行，并启用正式 Planner 会话接口。

`start.launch` 会启动硬件节点，不能作为无执行风险的检查命令。B29 工作区已移除
Gazebo 专用 launch、配置和 debug overlay；越障 FSM 只通过真机 `start.launch` 验证。

只观察仲裁结果时，将 `controller.yaml` 中的 `output_mode` 设为 `safe_hold`
并重启控制器；需要验证机构实际运动时才设为 `normal`。

启动 GUI：

```bash
rosrun rqt_b29_smc_console rqt_b29_smc_console
```

观察 trace：

```bash
rostopic echo /b29_controller/b29_smc_auto_controller/state_trace
```

## 3. 巡航与越障触发调试输入

```bash
OVERRIDE_TOPIC=/b29_controller/b29_smc_auto_controller/debug_override
```

相关字段 mask：

```text
FIELD_OBSTACLE_CROSSING_TRIGGER = 512
FIELD_CRUISE_DRIVE_REQUEST = 4096
field_mask = 4608
```

设置正值巡航并保持越障触发为低：

```bash
rostopic pub -1 "$OVERRIDE_TOPIC" b29_smc_auto_controller/AutoDebugOverride \
'{enabled: true, field_mask: 4608, obstacle_crossing_trigger: false, cruise_drive_request: 1}'
```

启动越障，必须从低电平切换到高电平：

```bash
rostopic pub -1 "$OVERRIDE_TOPIC" b29_smc_auto_controller/AutoDebugOverride \
'{enabled: true, field_mask: 4608, obstacle_crossing_trigger: true, cruise_drive_request: 1}'
```

完成两侧越障并进入 `CompleteWaitObstacleClear` 后，再发送下降沿：

```bash
rostopic pub -1 "$OVERRIDE_TOPIC" b29_smc_auto_controller/AutoDebugOverride \
'{enabled: true, field_mask: 4608, obstacle_crossing_trigger: false, cruise_drive_request: 1}'
```

越障期间提前发送下降沿无效，不会在进入 `CompleteWaitObstacleClear` 后补记。
若提前拉低，必须先重新拉高，再在完成等待阶段拉低。
巡航请求含义为 `0=Stop`、`1=Forward(+0.10)`、`2=Reverse(-0.10)`。
Forward/Reverse 仅表示左右轮下发速度的数值正负。

每条 override 消息会替换上一条覆盖配置。需要同时保持 `lower_alive`、`posture_ready` 或 `grip_confirmed` 时，必须合并字段和 mask。GUI 的 Override Composer 会自动组合 mask。

## 4. PlannerControl 手动放行

调试配置由启动参数 `planner_mode:=debug` 选择。该模式会让 `PlannerControlCoordinator` 使用手动放行路径

每次进入`PlannerControl` 后会清除旧 release，必须在当前阶段重新调用：

```bash
rosservice call /b29_controller/b29_smc_auto_controller/planner_release
```

非 `PlannerControl` 阶段调用会返回 `success=false`。服务只在下一控制周期将
`PlannerControl` 切换到 `RemoteControl`，不会跳过 `PlannerControl`。

`normal` 模式中，`planner_release` 服务会被明确拒绝，由正式 adapter 调用
`complete_planner_control` 完成会话；`normal` 和调试放行后都进入 `RemoteControl`。

### 4.1 RemoteControl 下位机输入

`RemoteControl` 的四关节增量、样本序号、完成信号和完成上升沿只来自下位机反馈帧，经 `RemoteControlInterface` 进入控制器。`AutoDebugOverride` 不提供对应字段，GUI 和 `rostopic pub` 都不能伪造遥控增量或结束信号。

每个新遥控样本依次经过 `remote_control/increment_deadband` 死区过滤、
`remote_control/increment_scale` 灵敏度缩放和 `remote_control/max_increment_per_sample` 单帧限幅，
之后才累加到四个腿部关节目标。GUI 中的 `remote_control_raw_increments` 显示下位机原始值，
`remote_control_applied_increments` 显示实际参与累计的处理后增量。
方向映射由 `remote_control/joint_direction_signs` 配置，当前左一关节为 `-1`，其余三个关节为
`+1`。

调试模式只能验证 `PlannerControl` 的人工放行；后续 `RemoteControl -> Regrip` 需要下位机先将完成信号置为 `0`，再在当前 RemoteControl 阶段产生新的 `0 -> 1` 上升沿。

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

动作按钮会影响控制器输出；在 `output_mode: normal` 的真机环境中可能驱动实际机构。
`EMERGENCY STOP` 不弹确认框；`Manual Reset` 和 `Planner Release` 会要求确认。GUI 启动失败不会影响 controller。

## 7. Trace 验收重点

越障触发上升沿后：

```text
obstacle_crossing_stage = OpenGripperBeforeGravityCompensation
stop_all = true
当前首侧 gripper_target = OPEN
另一侧 gripper_target = CLOSED
```

脱缆过程：

```text
disconnect_step
disconnect_step_elapsed_sec
disconnect_step_transition_reason
disconnect_step_expected_condition
disconnect_max_abs_pose_joint_velocity
disconnect_settle_velocity_threshold
disconnect_velocity_stable_elapsed_sec
```

Step7 使用三个位姿关节低速稳定门控：

```text
disconnect_max_abs_pose_joint_velocity
  <= disconnect_settle_velocity_threshold
```

脱缆只运动锁定臂 first joint；抬升和回落完成后，速度稳定即可进入等待翻越状态。

人工确认条件统一为：

```text
grip_confirmed == true
```
恢复映射：
| 失败动作 | 人工确认后的下一阶段 |
| --- | --- |
| `CloseBothGrippers` | 兼容状态超时直接人工介入，不自动重试 |
| `DisconnectCable` / `PlannerControl` / `Regrip`，当前为首侧 | 当前侧视为已人工完成并回夹，开始另一侧：`OpenGripperBeforeGravityCompensation` -> `EnableGravityCompensation` -> `Disconnecting` |
| `DisconnectCable` / `PlannerControl` / `Regrip`，当前为第二侧 | 当前侧视为已人工完成并回夹，进入 `CompleteWaitObstacleClear` |

首侧由上次完整越障成功后累计的双轮实际位置反馈净角行程决定：正值先左侧，负值先右侧。
trace 中的 `signed_wheel_travel`、轮位置基准和当前轮位置可用于核对该判断；进入
`CompleteWaitObstacleClear` 后该基准会立即重建并从零重新累计。

## 8. 推荐验证顺序

1. 配置 `output_mode: safe_hold` 后重启控制器，验证 RobotFSM、override 门禁、急停和 reset。
2. 配置 `output_mode: normal` 后重启控制器，验证首侧夹爪命令、Step3/Step5 插值和 Step7 稳定门控。
3. 发送越障触发上升沿后，确认控制器按轮里程选择首侧并直接进入首侧脱缆。
4. 等待 Step7 成功进入 `PlannerControl`，确认等待时间持续增长且不自动离开。
5. 调用 `planner_release`，确认进入 `RemoteControl`。
6. 通过下位机发送遥控增量，确认每个新反馈样本只累加一次；再由下位机产生完成位 `0 -> 1`，确认进入 `Regrip`。
7. 完成双侧流程后清除障碍，确认回到 `Idle` 且不重复触发。
8. 在任意阶段调用软件急停，确认进入 `SafeStop` 且越障内部状态清空。

可使用场景脚本：

```bash
rosrun b29_smc_auto_controller replay_scenario.py \
  _scenario:=b29_controllers/b29_smc_auto_controller/scenarios/software_emergency_stop_and_reset.yaml
```

## 9. 常见失败原因

- `debug_override` 无效：检查 `debug_validation_enabled` 是否为 `true`。
- Planner release 被拒绝：当前不在 `PlannerControl`，或当前为 `normal` 模式。
- Planner 点不覆盖：检查 `planner_point_available`、`planner_point_fresh` 和 `planner_override_applied`。
- `safe_hold` 下 Step7 不成功：该模式不会推动机构；必须在受控环境中切换为 `normal` 才能验证物理反馈路径。
- reset 后仍在 `SafeStop`：检查 `lower_alive`、`imu_ready`、`joint_fault` 和 `grip_fault`。
- 完成后仍停留在等待阶段：确认进入 `CompleteWaitObstacleClear` 后才发送了新的
  `obstacle_crossing_trigger: 1 -> 0` 下降沿；越障期间提前下降不会放行。

## 10. 构建与静态检查

```bash
git diff --check
python3 -m py_compile \
  b29_controllers/b29_smc_auto_controller/scripts/replay_scenario.py \
  b29_tools/rqt_b29_smc_console/src/rqt_b29_smc_console/*.py
catkin build b29_smc_auto_controller b29_planner_adapter rqt_b29_smc_console b29_control
```

修改 `AutoStateTrace.msg` 会改变 ROS message MD5。依赖节点必须统一重新构建并同时部署。
