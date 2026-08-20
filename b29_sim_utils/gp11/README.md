# GP11 MoveIt 真机越障组件

GP11 负责为 B29 计算 MoveIt 轨迹。它不直接控制硬件，轨迹必须经过 Planner Adapter 和 SMC。

## 1. 数据链

```text
gp11_automatic_flip
  -> ReachPoint Action
  -> Cartesian goal server
  -> MoveIt
  -> FollowJointTrajectory
  -> Planner Adapter
  -> SMC PlannerControl
  -> hardware
```

旧 `trajectory_bridge` 不启动。正式 FollowJointTrajectory Action Server 由 Planner Adapter 提供。

## 2. 如何启动

正式运行只启动主 launch：

```bash
roslaunch b29_control start.launch mode:=dual_role planner_mode:=normal
```

它已经包含 GP11 MoveIt、Adapter 和自动双侧翻越。不要再次单独启动 GP11，否则可能重复启动 MoveGroup、TF、Action Server 或监控节点。

`gp11_moveit_real.launch` 单独启动时默认不允许真实执行，只用于受控规划和观察。它也不会自动提供 B29 硬件、`/joint_states` 或 Adapter。

## 3. 自动双侧翻越

`gp11_automatic_flip.py` 对每一侧执行：

1. 等待 SMC 进入 `DisconnectDoneWaitFlip`。
2. 读取当前 `crossing_side`。
3. 等待关节反馈稳定和正确锚点就绪。
4. 发布 `start_flip`，让 SMC 进入 PlannerControl。
5. 发送当前侧的 ReachPoint 目标。
6. MoveIt 完成 IK、碰撞检查、规划和轨迹审计。
7. Adapter 执行轨迹并完成 SMC 会话。
8. SMC 进入 RemoteControl，之后由下位机微调和回夹。

第一侧回夹完成后，SMC 自动切到另一侧；两侧都完成后等待越障触发下降沿。当前
`start.launch` 默认 `continuous_obstacles=true`：下降沿使 SMC 回到 `Idle` 后，自动节点继续等待下一个障碍，而不是退出或要求固定延时。

每个障碍的目标 profile 由该障碍开始时 SMC 锁存的首侧选择，正反向使用两套笛卡尔预测点。两遍都沿用同一障碍的 profile；不要按当前左右臂自行切换目标点。

## 4. 锚点是什么意思

当前侧是张开的夹爪侧，另一侧夹爪仍在线缆上，作为规划锚点。

- Left 越障：左夹爪张开，右侧作为锚点。
- Right 越障：右夹爪张开，左侧作为锚点。

`gp11_moveit_anchor_runtime.py` 负责切换锚点模型和 MoveGroup。`gp11_generate_anchored_urdf.py` 生成对应重定根 URDF。

锁定臂 second joint 的最终角度由目标点 IK 决定，不能简单写成“当前位置加 pi”。

## 5. 关键接口

```text
/gp11_moveit/reach_point
/gp11_moveit/reach_arm_controller/follow_joint_trajectory
/gp11_moveit/runtime_anchor
/gp11_moveit/runtime_status
/move_group
/compute_ik
/execute_trajectory
```

自动客户端会根据 SMC 锁存的 `first_crossing_side` 选择目标 profile，再根据当前
`crossing_side` 选择锚点；调用者不应自行猜测方向或锚点侧。

## 6. 主要文件

| 文件 | 作用 |
|---|---|
| `launch/gp11_moveit_real.launch` | 真机 MoveIt 节点组合 |
| `scripts/gp11_automatic_flip.py` | 自动执行左右两侧 |
| `scripts/gp11_single_flip.py` | 单侧分段验证 |
| `src/gp11/single_flip_client.py` | 等待状态、同步锚点和发送 Action |
| `scripts/gp11_cartesian_goal_server.py` | IK、规划、审计和执行 |
| `scripts/gp11_moveit_anchor_runtime.py` | 锚点和 MoveGroup 生命周期 |
| `scripts/gp11_generate_anchored_urdf.py` | 生成锚定 URDF |
| `scripts/gp11_real_execution_monitor.py` | 只读执行监控 |
| `config/*_real.yaml` | 真机 MoveIt 参数 |

## 7. 单侧分段验证

先关闭自动调度：

```bash
roslaunch b29_control start.launch mode:=dual_role launch_automatic_flip:=false
```

SMC 到达 `DisconnectDoneWaitFlip` 后运行：

```bash
rosrun gp11 gp11_single_flip.py
```

这些命令会连接并可能驱动真机。客户端会检查当前侧、锚点、PlannerControl 和 Action 服务，不满足条件时应拒绝执行。

## 8. 构建和安全

```bash
catkin build gp11 b29_planner_adapter b29_smc_auto_controller
python3 -m py_compile \
  b29_sim_utils/gp11/scripts/*.py \
  b29_sim_utils/gp11/src/gp11/*.py
```

当前安全边界：

- GP11 规划成功不等于机构执行成功，最终结果取决于编码器反馈、Adapter 公差和 SMC 握手。
- `allow_direct_execution=true` 只跳过旧 plan-id 往返，不跳过 IK、碰撞、轨迹和 SMC 检查。
- 自动节点失败后停止后续翻越，不应通过重复发布 topic 绕过门禁。
- 黑匣子位于 `~/.ros/gp11_blackbox`，只记录数据，不参与控制。
- 本包的 `test/test_dual_anchor_flip.py` 由 Git 跟踪，并在 `CATKIN_ENABLE_TESTING` 时由 CMake 注册。
