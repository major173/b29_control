# b29_planner_adapter

`b29_planner_adapter` 将 GP11 MoveIt 的四关节 `FollowJointTrajectory` 转换为 B29 SMC 的正式 PlannerControl 会话协议。它不获取 ros_control `JointHandle`，不控制夹爪或驱动轮，也不直接访问串口；所有最终命令仍由 `B29SmcAutoController::buildEffectiveCommand()` 仲裁并通过 `CommandDispatcher` 写入。

## 正式数据链

```text
MoveIt FollowJointTrajectory
  -> b29_planner_adapter
  -> PlannerJointCommand（每次一个带会话和序号的四关节点）
  -> B29SmcAutoController
  -> buildEffectiveCommand()
  -> CommandDispatcher
```

adapter 只使用以下正式接口：

- `planner_control_state`：SMC 发布当前会话、接收状态、序号确认和退出结果。
- `planner_joint_command`：adapter 发布四关节命令。
- `complete_planner_control`：最终反馈稳定后申请结束当前会话。

adapter 不订阅 `state_trace`，也不会调用调试服务 `planner_release`。

## 关节契约

`PlannerJointCommand.positions` 的固定顺序为：

```text
[left_first, left_second, right_first, right_second]
```

GP11 Action 必须直接使用相同名称和相同顺序：

| GP11/Action 名称 | B29 输出名称 |
|---|---|
| `left_first_leg_joint` | `left_first_leg_joint` |
| `left_second_leg_joint` | `left_second_leg_joint` |
| `right_first_leg_joint` | `right_first_leg_joint` |
| `right_second_leg_joint` | `right_second_leg_joint` |

Adapter 不再维护别名或输入重排。第二关节按最短角连续展开；adapter 不做左右交换和符号翻转，B29 硬件层继续负责模型坐标到串口协议的方向处理。旧 GP11 模型使用的 `left_second_leg_z` 不属于真机关节名，adapter 会拒绝包含该名称的轨迹。

## 轨迹执行

- 轨迹必须按 `[left_first_leg_joint, left_second_leg_joint, right_first_leg_joint, right_second_leg_joint]` 提供四个关节。
- 每个点必须包含四个有限位置值，`time_from_start` 必须严格递增。
- 只有位置时使用线性插值；全部点有速度时使用三次插值；全部点同时有速度和加速度时使用五次插值。
- `time_scale` 统一缩放整条轨迹。
- 若 50 Hz 相邻输出超过 SMC 的单命令 delta 上限，会继续统一延长整条轨迹，不独立钳制某个关节。
- 每次只发送一个序号，收到 `PlannerControlState.last_accepted_sequence` 确认后才推进下一点；未确认时只重发同一序号和同一内容。SMC callback 只把合法点放入单槽 pending；该点成为 effective command，并在 `normal`、非冻结状态下由 dispatcher 写入关节句柄后，SMC 在同一 update 周期发布 ACK。Adapter 由状态 callback 条件变量立即唤醒，不再固定休眠一个发布周期。
- 新会话在尚未接受序号 `1` 前，SMC 持续以实时编码器刷新首点参考；adapter 只有在该参考与收到 Action 时的反馈一致后才开始发送。
- 轨迹在下发前检查相对当前反馈的总位移；运行中额外检查连续反向反馈证据。
- 最终点确认后继续重发最终序号，等待四关节位置误差和可选速度误差连续满足 `settle_time`。
- 完成服务受理后，还必须等到 SMC 发布匹配的 `last_completed_session_id` 和 `EXIT_COMPLETED`，Action 才返回成功。SMC 此时进入 `RemoteControl`，后续遥控和回夹不属于 MoveIt Action 的执行范围。

会话变化、状态超时、反馈超时、安全撤权、命令拒绝、完成服务失败或异常退出都不会向 MoveIt 误报成功。已接受命令后的可恢复失败或 Action 取消会先以不超过 `max_output_delta` 的步长下发当前位置保持；保持失败、反馈丢失或确认反向运动时，adapter 调用 SMC 的 `software_emergency_stop` 服务。首条命令在被 SMC 明确拒绝前不会触发软件急停。

## 配置与启动

默认参数位于 `config/planner_adapter.yaml`，启动入口为：

```bash
roslaunch b29_planner_adapter planner_adapter.launch
```

默认 Action 名称：

```text
/gp11_moveit/reach_arm_controller/follow_joint_trajectory
```

`interface_mode` 允许 `normal` 或 `debug`。通过 `b29_control/launch/start.launch` 启动时，只修改一次 `planner_mode`，该值会同时传递给 SMC 和 adapter。`normal` 启用正式会话协议；`debug` 用于 SMC 的手动 `planner_release` 验证，不执行 MoveIt 轨迹。Planner topic/service 地址由同一 launch 参数 `smc_controller_namespace` 派生，不在两个 YAML 中重复维护。

以下参数在节点启动时读取，活动轨迹期间不热更新：

- Action、SMC controller namespace 和 joint state 名称；
- 固定关节契约、连续关节属性和非连续关节范围；
- 发布频率、`time_scale`、最大输出 delta；
- 起点、路径、最终位置和最终速度容差；
- `settle_time`、状态/反馈/确认/Action/完成握手超时；
- 完成服务重试次数和间隔。

未在 YAML 中显式覆盖时，安全默认值为：总位移上限每轴 `4π rad`；当目标误差至少 `25°` 且单次反馈向远离目标方向变化至少 `0.2°` 时，连续 `15` 个样本触发反向运动保护。`max_output_delta` 和现有容差仍以 `planner_adapter.yaml` 为准。

空接口名、非法关节配置、非有限参数或非正超时会使节点启动失败。收到 Action 时，adapter 使用 `PlannerControlState.max_delta_per_command` 核对 SMC 当前真实上限；`max_output_delta` 超过该上限时拒绝 Action，采样使用两者较小值。

## GP11 侧边界

正式联调前必须在 GP11 工程中单独完成：

1. 禁用旧 `trajectory_bridge_node.py`，避免两个 Action Server 占用同一名称。
2. 保证 MoveIt controller 指向 adapter 的 Action。
3. 将 GP11 的 `robot_description` 放入独立命名空间，避免覆盖 B29 全局参数。

B29 工作区不修改 GP11 工程。联调时应以本 README、`planner_adapter.yaml`、SMC 发布的
`PlannerControlState` 和当前 MoveIt 配置为准。

## 构建与离线测试

```bash
catkin build b29_smc_auto_controller b29_planner_adapter
catkin run_tests b29_smc_auto_controller b29_planner_adapter
catkin_test_results
```

这些命令不启动 ROS 节点。任何 launch、topic、service、Gazebo 或实机操作必须由操作者另行确认环境和安全条件。
