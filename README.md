# B29

> B29 实机控制、双角色翻越与上位机重力补偿工作区。

## 当前主流程

```bash
cd ~/B29_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch b29_control start.launch
```

`start.launch` 统一启动硬件接口、SMC 状态机、Planner Adapter、GP11 MoveIt、
自动双遍翻越和相关监控节点。当前分支不再包含旧 Reach RL/DLS 推理、键盘目标、
关节目标 bridge 或锚点 TF 发布流程。

重力补偿由上位机根据翻越状态机的固定锚点计算并通过 V2 `tau_ff` 下发；
旧下位机重力补偿字段始终为 `0`。补偿覆盖脱离、等待翻转、Planner、遥控、
重新夹紧以及脱离失败后的保持阶段，确认重新夹紧后关闭。

## 构建

```bash
cd ~/B29_ws
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

## 文档入口

- [工程总交接手册](B29_AUTOMATION_HANDOFF.md)
- [越障流程](b29_controllers/b29_smc_auto_controller/OBSTACLE_CROSSING_WORKFLOW.md)
- [SMC 控制器](b29_controllers/b29_smc_auto_controller/README.md)
- [Planner Adapter](b29_planner_adapter/README.md)
- [GP11 MoveIt](b29_sim_utils/gp11/README.md)
- [越障调试验证](b29_control/docs/b29_smc_obstacle_crossing_debug_validation.md)

当前仓库不提供 B29 Gazebo 启动入口。实机操作前应完成交接手册中的安全检查。
