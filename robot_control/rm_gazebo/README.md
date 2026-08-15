# rm_gazebo（上游遗留仿真包）

该包提供 RM 上游 Gazebo ros_control 插件、世界和通用 launch。它仍可能被其他遗留仿真包引用，但不是当前 B29 自动脱缆和翻越流程的一部分。

当前 B29 事实：

- 真机入口是 `roslaunch b29_control start.launch`。
- B29 当前仓库不提供受支持的 Gazebo 启动或验证流程。
- 不能用本包的仿真结果替代双锚 MoveIt、Planner Adapter、SMC 和真实编码器闭环验证。
- 不要为解决 B29 真机问题恢复已经移除的 `b29_control/start_in_gazebo*.launch`。

本包主要产物：

- Gazebo ros_control 库 `rm_robot_hw_sim`
- 插件描述 `rm_robot_hw_sim_plugins.xml`
- `config/`、`launch/` 和 `worlds/` 中的上游仿真资源

只在明确维护遗留 RM 仿真时构建：

```bash
catkin build rm_gazebo
```

B29 当前总流程见仓库根目录 `B29_AUTOMATION_HANDOFF.md`。
