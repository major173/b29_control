# B29 SMC Gazebo 调试状态

## 当前状态

当前工作区没有可用的 SMC Gazebo 越障调试启动链路。

- `start_smc_in_gazebo.launch` 及其 debug overlay 已删除。
- `b29_control/launch/start_in_gazebo.launch` 和
  `b29_control/launch/start_in_empty_gazebo.launch` 只加载传统关节控制器，
  不会启动 `b29_smc_auto_controller`。
- 因此，不能用上述两个 Gazebo launch 验证 RobotFSM、越障 FSM、PlannerControl、
  RemoteControl 或 `state_trace`。

## 当前调试入口

越障控制器的参数、调试 override、PlannerControl、RemoteControl、软件急停和
场景回放说明，见：

- `b29_control/docs/b29_smc_obstacle_crossing_debug_validation.md`
- `b29_controllers/b29_smc_auto_controller/OBSTACLE_CROSSING_WORKFLOW.md`

当前控制器启动入口为 `b29_control/launch/start.launch`。该 launch 会启动硬件节点，
不得把它当成无执行风险的 Gazebo 替代品。

## 恢复 Gazebo 验证的前提

需要新增并维护专用 launch，至少应：

1. 加载 `b29_control/config/controller.yaml`。
2. 启动 `b29_smc_auto_controller` 及其依赖的状态、IMU 和关节控制器。
3. 明确设置 `debug_validation`、`planner_interface/mode` 和 `output_mode`。
4. 在 `safe_hold` 与 `normal` 下分别验证 trace 仲裁和仿真机构运动。

在该专用 launch 恢复前，本文不提供可执行 Gazebo 越障验证命令，以避免误导。
