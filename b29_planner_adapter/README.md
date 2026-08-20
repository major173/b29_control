# b29_planner_adapter

Planner Adapter 位于 MoveIt 和 B29 SMC 之间。它接收一整条 MoveIt 轨迹，检查并采样后，逐点交给 SMC。

Adapter 不直接控制硬件、夹爪或驱动轮。最终命令仍由 SMC 仲裁和下发。

## 1. 数据链

```text
MoveIt FollowJointTrajectory
  -> Planner Adapter
  -> PlannerJointCommand，单个四关节点
  -> SMC PlannerControl
  -> CommandDispatcher
  -> hardware interface
```

Adapter 使用三个 SMC 接口：

| 接口 | 方向 | 作用 |
|---|---|---|
| `planner_control_state` | SMC -> Adapter | 当前 session、ACK、限制和退出结果 |
| `planner_joint_command` | Adapter -> SMC | 一个带 session 和 sequence 的四关节点 |
| `complete_planner_control` | Adapter -> SMC | 请求正常结束 PlannerControl |

Adapter 不使用 `state_trace`，也不调用调试服务 `planner_release`。

## 2. 关节顺序

轨迹必须包含以下四个关节，并保持相同顺序：

```text
left_first_leg_joint
left_second_leg_joint
right_first_leg_joint
right_second_leg_joint
```

Adapter 不做左右交换和符号翻转。旧名称 `left_second_leg_z` 会被拒绝。第二关节会按最短角展开，避免连续角跨越 `±pi` 时跳变。

## 3. 一条轨迹如何执行

1. MoveIt 一次性发送完整轨迹。
2. Adapter 检查关节、有限值、时间、起点和总位移。
3. 根据轨迹内容选择插值：
   - 只有位置：线性。
   - 位置和速度：三次。
   - 位置、速度和加速度：五次。
4. 轨迹按配置频率采样，默认目标是 50 Hz。
5. 如果相邻输出变化过大，统一延长整条轨迹，而不是单独钳制某个关节。
6. Adapter 每次只发送一个 sequence。
7. SMC 真正应用并 dispatch 后发布 ACK。
8. Adapter 收到 ACK 才发送下一点，因此不会覆盖尚未下发的点。
9. 最终点下发后，Adapter 根据编码器反馈检查位置、速度和稳定时间。
10. Adapter 调用完成服务，并等待 SMC 确认退出 PlannerControl。
11. Action 向 MoveIt 返回成功，SMC 进入 RemoteControl。

MoveIt Action 成功只表示 PlannerControl 轨迹完成，不表示后续 RemoteControl 和 Regrip 已完成。

## 4. 主要安全检查

- SMC 当前必须处于可接收的 PlannerControl session。
- 起点不能离当前反馈太远。
- 轨迹时间必须严格递增。
- 所有位置、速度和加速度必须是有限值。
- 单点变化不能超过 SMC 当前上限。
- 运行中检查 path tolerance、反馈新鲜度和持续反向运动。
- 终点必须连续满足位置和可选速度公差。
- 会话变化、超时、拒绝或安全撤权都不会被报告为成功。

大角度翻越允许配置“功能完成”备用条件，但仍必须依赖真实编码器反馈和稳定时间。

取消或可恢复失败后，Adapter 会尝试以受限步长保持当前位置。保持失败、反馈丢失或确认反向运动时，会请求 SMC 软件急停。

## 5. 关键参数

配置文件：`config/planner_adapter.yaml`。

常用参数分组：

- `publish_rate`、`time_scale`、`max_output_delta`。
- 起点、path、终点位置和终点速度容差。
- 状态、反馈、ACK、Action 和完成服务超时。
- `settle_time`。
- 总位移和反向运动保护。
- 大翻越功能完成条件。

Adapter 启动时会读取 SMC 发布的 `max_delta_per_command`。`max_output_delta` 大于 SMC 上限时拒绝执行；实际采样使用两者较小值。

参数在节点启动时读取，活动轨迹期间不会热更新。

## 6. 启动

完整工程使用：

```bash
roslaunch b29_control start.launch mode:=dual_role planner_mode:=normal
```

主 launch 会同时启动 SMC、Adapter 和 GP11，不要再单独重复启动 Adapter。

独立启动入口：

```bash
roslaunch b29_planner_adapter planner_adapter.launch
```

独立启动不会自动提供硬件、SMC 或 MoveIt，主要用于受控分段联调。

运行模式：

- `normal`：执行正式 Planner 会话。
- `debug`：不执行 MoveIt 轨迹。

通过 `start.launch` 时只设置一次 `planner_mode`，它会同时传给 SMC 和 Adapter。

## 7. 与 GP11 的关系

GP11 负责目标、IK 和 MoveIt 规划；Adapter 负责轨迹检查、采样、逐点确认和 Action 结果；SMC 负责阶段权限和最终命令。

旧 `trajectory_bridge` 不启动，避免两个 Action Server 占用同一名称。详细流程见
[`../b29_sim_utils/gp11/README.md`](../b29_sim_utils/gp11/README.md)。

## 8. 构建

```bash
catkin build b29_smc_auto_controller b29_planner_adapter
```

当前分支没有独立的 `test/test_trajectory_processor.cpp`。该文件曾随已撤销的 Adapter
启动确认/重试实验加入，回退实验时已删除；不要按旧文档恢复。构建本包不会主动启动 ROS 或硬件。

架构取舍见 [`docs/adapter_vs_smc_direct_action.md`](docs/adapter_vs_smc_direct_action.md)。
