# b29_smc_auto_controller

## 目标

提供 B29 SMC 自动控制器的新包入口，先完成包骨架、插件导出和构建入口，后续再逐步补齐控制器实现、消息接口和测试。

## 当前范围

- 可加载的最小 controller stub
- 统一输入输出消息定义
- 统一类型头 `auto_types.h`
- 最小 SMC 状态机 owner / generated code 迁移
- 最小输入适配层 `AutoInputMux`
- 最小命令执行层 `CommandDispatcher`
- 基础测试覆盖默认安全值与 mask 常量
- 状态流转最小验证：`Idle -> AutoInit`、`* -> SafeStop`

## 数据面对齐

自动控制器当前通过控制器壳读取 `b29_control` 已注册接口，再在包内转换成统一输入缓存：

- `PositionJointInterface`
- `VelocityJointInterface`
- `JointStateInterface`
- `ImuSensorInterface(base_imu)`

同时，自动控制链路仍需要额外业务输入来闭合控制语义：

- `lower_alive`
- `grip_confirmed / grip_fault`
- `obstacle_detected / obstacle_type / classification_stable / range_to_obstacle`
- `at_crossing_position`
- `post_check_passed / post_check_failed`

这些字段由统一输入输出模型承载，不在本包内直接假设其他业务 handle 已经可用。
当前边界是：

- `B29SmcAutoController` 从 `JointStateInterface / PositionJointInterface / VelocityJointInterface / ImuSensorInterface(base_imu)` 读取已注册 handle
- 控制器壳把硬件读数整理成 `sensor_msgs::JointState` 和 `sensor_msgs::Imu`
- `AutoInputMux` 负责把这些缓存与 `AutoSensorInput / AutoDebugOverride / AutoControlRequest` 合并成 `AutoInputSnapshot`

## AutoInputMux

`AutoInputMux` 只负责输入缓存与合并，不负责 ROS 订阅和控制器调度：

- 硬件缓存：
  - `sensor_msgs/JointState`
  - `sensor_msgs/Imu(base_imu)`
- 业务输入：
  - `AutoSensorInput`
- 控制请求：
  - `AutoControlRequest`
- 调试覆盖：
  - `AutoDebugOverride`

当前最小合并规则：

- `imu_ready` 当前保留为业务输入语义，来自 `AutoSensorInput`
- `base_imu` 当前只负责生成 `posture_ready`，不覆盖业务态 `imu_ready`
- `posture_ready` 由 roll/pitch 阈值直接判定
- `joint state` 当前先承担已注册硬件接口对齐与时间戳合并，不额外引入业务语义
- `AutoDebugOverride` 只覆盖 `field_mask` 标记字段，未标记字段保持原合并结果

## CommandDispatcher

`CommandDispatcher` 负责把 `AutoControlCommand` 写入硬件句柄：

- 速度输出写入左右摩擦轮 `VelocityJointInterface`
- 位置输出写入 6 个机构关节 `PositionJointInterface`
- `stop_all=true` 时强制清零轮速
- `freeze_joints=true` 时强制用当前位置锁住关节，而不是发送新的目标位姿

当前 `B29SmcAutoController` 已切到 `MultiInterfaceController`，最小闭环为：

- 从 `JointStateInterface` / `ImuSensorInterface(base_imu)` 读取硬件快照
- 通过 `AutoInputMux` 生成 `AutoInputSnapshot`
- 驱动 `RobotContext`
- 通过 `CommandDispatcher` 下发 `AutoControlCommand`

当前仍不包含业务订阅器，因此控制器默认保持安全闭环，不在 Task 5 提前引入外部请求链路。
- 作为公共接口时，`AutoInputMux` 依赖本包导出的消息头，外部工程应通过 catkin 依赖本包而不是手工拼 include 路径

## 开发约束

- 输入优先复用 `b29_control` 已注册接口
- 额外业务输入通过统一输入适配层接入
- 修改状态、输入、调试方式时必须同步更新本 README

## 运行与调试

启动控制器：

```bash
roslaunch b29_control start.launch
```

调试注入 topic：

- `/b29_controller/b29_smc_auto_controller/debug_override`
- `/b29_controller/b29_smc_auto_controller/sensor_input`

状态追踪 topic：

- `/b29_controller/b29_smc_auto_controller/state_trace`

当前 `state_trace` 会发布：

- 当前状态
- 最近一次状态转移摘要
- 最近一次错误/告警摘要
- 当前命令原因与轮速/冻结标志

## SMC 最小落地

- 状态机定义位于 `sm/RobotFSM.sm`
- generated code 位于 `gen/include/RobotFSM_sm.h` 与 `gen/src/RobotFSM_sm.cpp`
- `RobotContext` 作为 SMC owner，对外暴露最小接口：
  - `start()`
  - `tick50Hz()`
  - `setInputSnapshot(...)`
  - `requestAutoStart()`
  - `currentCommand()`
  - `currentStateName()`
- 当前只保证最小状态优先级：
  - `emergency_stop`
  - `!lower_alive`
  - `CommsLoss && lower_alive -> evCommsRestored`
  - `SafeStop && manual_reset_requested -> evManualReset`
  - `Idle && auto_start_requested`
  - 其余进入 `evTick`
- 当前不接控制器执行层，动作实现只更新内存态 `AutoControlCommand`

## SMC 代码生成

在包目录 `src/b29_control/b29_controllers/b29_smc_auto_controller/` 执行：

```bash
java -jar ../../../../smc_7_6_0/bin/Smc.jar -c++ -d gen/src -headerd gen/include sm/RobotFSM.sm
```

当前生成使用仓库自带 `smc_7_6_0/bin/Smc.jar`。本地这个版本使用 `-c++`，不是旧文档里的 `-lang c++`。如果仓库内存在 `smc_7_6_0/lib/C++/statemap.h`，构建会强制优先使用这份头文件；否则再回退到宿主环境中的 `smclib`。本包会同时导出 `gen/include/RobotFSM_sm.h` 与 `include/statemap.h`，保证下游通过 catkin 依赖时公共头链路可用。

如果需要 dot 状态图，单独执行：

```bash
java -jar ../../../../smc_7_6_0/bin/Smc.jar -graph -glevel 1 -d gen sm/RobotFSM.sm
```

生成后的 `RobotFSM_sm.h/.cpp` 应直接覆盖 `gen/` 目录中的同名文件，并重新构建。

## 场景回放

通过场景文件向 `debug_override` 回放调试事件：

```bash
rosrun b29_smc_auto_controller replay_scenario.py \
  _scenario:=$(find b29_smc_auto_controller)/scenarios/line_clamp_nominal.yaml
```

当前示例场景基于已实现的最小状态集：

- `line_clamp_nominal.yaml`
- `comms_loss_during_approach.yaml`

其中第二个场景文件名沿用计划命名，但当前实际覆盖的是 `Traversing -> CommsLoss -> Idle` 这条已实现链路。
