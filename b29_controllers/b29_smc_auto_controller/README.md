# b29_smc_auto_controller

> 提供 B29 SMC 自动控制器的新包入口，先完成包骨架、插件导出和构建入口，后续再逐步补齐控制器实现、消息接口和测试。

**当前范围**

- 可加载的最小 controller stub
- 统一输入输出消息定义
- 统一类型头 `auto_types.h`
- 最小 SMC 状态机 owner / generated code 迁移
- 最小输入适配层 `AutoInputMux`
- 最小命令执行层 `CommandDispatcher`
- 基础测试覆盖默认安全值与 mask 常量
- 状态流转最小验证：`Idle -> AutoInit -> Traversing`、`AutoInit -> SafeStop (evInitFailed)`、`Traversing -> Idle (evAutoRunPause)`、`* -> SafeStop`

**开发约束**

- 输入优先复用 `b29_control` 已注册接口
- 额外业务输入通过统一输入适配层接入
- 修改状态、输入、调试方式时必须同步更新本 README

## 推荐调试方式

当前联调优先使用 [`rqt_b29_smc_console`](/home/yuchen/usetest/B29/src/b29_control/b29_tools/rqt_b29_smc_console/README.md)，而不是手写 `rostopic pub`。

这个插件可以直接完成以下工作：

- 通过 `Sensor Input` 预设快速构造基础态、失联态和障碍态
- 通过 `Override Composer` 发送单次触发、持续覆盖或取消覆盖
- 通过 `Workflow` 执行预置场景，验证状态机链路
- 通过 `Trace` 观察 `state_trace`、状态迁移和命令原因

`scripts/replay_scenario.py` 仍然保留，适合作为场景回放补充，但不作为日常调试主入口。



## 数据面对齐

自动控制器当前通过控制器壳读取 `b29_control` 已注册接口，再在包内转换成统一输入缓存：

- `PositionJointInterface`
- `VelocityJointInterface`
- `JointStateInterface`
- `ImuSensorInterface(base_imu)`
- `AutoStateInterface`

同时，自动控制链路仍需要额外业务输入来闭合控制语义：

- `lower_alive`
- `grip_confirmed / grip_fault`
- `obstacle_detected / obstacle_type / classification_stable / range_to_obstacle`
- `at_crossing_position`
- `post_check_passed / post_check_failed`
- `auto_run_pause`

这些字段由统一输入输出模型承载，不在本包内直接假设其他业务 handle 已经可用。
当前边界是：

- `B29SmcAutoController` 从 `JointStateInterface / PositionJointInterface / VelocityJointInterface / ImuSensorInterface(base_imu)` 读取已注册 handle
- 控制器壳把硬件读数整理成 `sensor_msgs::JointState` 和 `sensor_msgs::Imu`
- `AutoInputMux` 负责把这些缓存与 `AutoSensorInput / AutoDebugOverride / AutoControlRequest` 合并成 `AutoInputSnapshot`



在 `controller.yaml` 文件中选择输入数据来源为 `sensor_input` 还是 `AutoStateInterface`

## 模块说明

### AutoInputMux

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

- `imu_ready` 当前由 `base_imu` 数据是否有效进行判断，超过 ` 三个周期`  ` base_imu`  数据没有更新则判断为  `false`
- `base_imu` 当前只负责生成 `posture_ready`
- `posture_ready` 由 roll/pitch 阈值直接判定
- `joint state` 当前先承担已注册硬件接口对齐与时间戳合并，不额外引入业务语义
- `AutoDebugOverride` 只覆盖 `field_mask` 标记字段，未标记字段保持原合并结果
- `auto_run_pause` 当前通过 `AutoDebugOverride` / `AutoInputMux` 注入，用于恢复最小暂停链路

### CommandDispatcher

`CommandDispatcher` 负责把 `AutoControlCommand` 写入硬件句柄：

- 速度输出写入左右摩擦轮 `VelocityJointInterface`
- 位置输出写入 6 个机构关节 `PositionJointInterface`
- `stop_all=true` 时强制清零轮速
- `freeze_joints=true` 时强制用当前位置锁住关节，而不是发送新的目标位姿
- `output_mode=normal` 时按状态机命令执行
- `output_mode=safe_hold` 时忽略运动请求，强制输出安全保持命令：
  - 左右轮速度为 `0.0`
  - 6 个位置关节锁当前位置

当前 `B29SmcAutoController` 已切到 `MultiInterfaceController`，最小闭环为：

- 从 `JointStateInterface` / `ImuSensorInterface(base_imu)` 读取硬件快照
- 通过 `AutoInputMux` 生成 `AutoInputSnapshot`
- 驱动 `RobotContext`
- 通过 `CommandDispatcher` 下发 `AutoControlCommand`

当前仍不包含业务订阅器，因此控制器默认保持安全闭环，不在 Task 5 提前引入外部请求链路。
- 作为公共接口时，`AutoInputMux` 依赖本包导出的消息头，外部工程应通过 catkin 依赖本包而不是手工拼 include 路径



### 状态说明

以下说明以当前 `sm/RobotFSM.sm` 与 `RobotContext::tick50Hz()` 的已实现语义为准。

#### `Idle`

- 自动运行空闲态，也是默认启动态
- 机器人在该状态下不主动推进自动任务，等待自动启动条件满足
- 只有在 `Idle` 下，`auto_start_requested` 才会被消费为一次真正的启动请求
- 收到 `evAutoStart` 且 `canStartAuto()` 成立时，转入 `AutoInit`
- 收到 `evCommsLost` 时，转入 `CommsLoss`
- 收到 `evEmergencyStop` 时，转入 `SafeStop`

#### `AutoInit`

- 自动模式初始化态，用于在真正进入巡航前完成自动链路初始化
- 进入状态时执行 `startInitSequence()`
- 离开状态时执行 `clearInitFlags()`
- 当前最小实现里，当 `isReadyToTraverse()` 成立时，通过 `evTick` 转入 `Traversing`
- 当前最小实现里，如果 `AutoInit` 持续 100 个 `tick50Hz()` 周期仍未满足 `isReadyToTraverse()`，会触发 `evInitFailed` 转入 `SafeStop`
- 收到 `evCommsLost` 时，转入 `CommsLoss`
- 收到 `evEmergencyStop` 时，转入 `SafeStop`

#### `Traversing`

- 自动巡航前进态，表示机器人已经进入自动运行主链
- 进入状态时执行 `setCruiseCommand()`，给出向前巡航命令
- 当前最小实现中：
  - 若未检测到障碍，`evTick` 会继续刷新巡航命令
  - 若检测到障碍，当前仍停留在 `Traversing`，只是不额外刷新巡航动作
- 这意味着“识别到障碍后的细分停障/接近/越障状态”还没有在当前版本展开
- 收到 `evCommsLost` 时，转入 `CommsLoss`
- 收到 `evEmergencyStop` 时，转入 `SafeStop`
- 收到 `evAutoRunPause` 时，转入 `Idle`，并停止当前自动运行输出

#### `CommsLoss`

- 下位机通信丢失保护态
- 进入状态时执行：
  - `startReconnectTimer()`
  - `alertCommsLoss()`
- 离开状态时执行 `stopReconnectTimer()`
- 该状态先等待通信恢复；如果连续 5 秒都没有恢复，则转入 `SafeStop`
- 收到 `evCommsRestored` 时，回到 `Idle`
- 收到 `evReconnectTimeout` 时，转入 `SafeStop`
- 收到 `evEmergencyStop` 时，转入 `SafeStop`

#### `SafeStop`

- 安全停机态，用于承接急停或严重异常
- 进入状态时执行：
  - `disableAutoMode()`
  - `alertSafeStop()`
  - `logSafeStopEntry()`
- 该状态下不会自动恢复，必须人工确认后复位
- 收到 `evManualReset` 时，清理故障标志并回到 `Idle`
- 在 `SafeStop` 中再次收到 `evEmergencyStop` 或 `evCommsLost`，当前实现均保持原地不动

### 事件说明

所有事件都由 `RobotContext::tick50Hz()` 或 `SMC` 状态表消费，当前已实现事件如下。

#### `evAutoStart`

- 含义：请求进入自动模式
- 来源：`Idle` 状态下检测到 `auto_start_requested`
- 作用：若 `canStartAuto()` 成立，则从 `Idle` 转入 `AutoInit`
- 在其他状态下，当前实现中该事件要么不会主动发出，要么即使发出也会被状态表忽略

#### `evTick`

- 含义：50Hz 周期推进事件，是状态机的基础驱动事件
- 来源：控制器每周期调用 `tick50Hz()` 时，在没有更高优先级事件要处理时发出
- 作用：
  - 驱动 `AutoInit -> Traversing`
  - 驱动 `AutoInit -> SafeStop` 的初始化失败闭环
  - 驱动 `Traversing` 内的巡航命令刷新
  - 在 `CommsLoss` 中推进通信恢复计时
  - 在 `SafeStop` 中维持原状态等待人工复位

#### `evCommsLost`

- 含义：判定下位机通信丢失
- 来源：当前周期检测到 `lower_alive == false`
- 作用：从普通运行态切到 `CommsLoss`
- 当前优先级高于自动启动和普通 `evTick`

#### `evCommsRestored`

- 含义：判定下位机通信恢复
- 来源：当前处于 `CommsLoss` 且检测到 `lower_alive == true`
- 作用：从 `CommsLoss` 回到 `Idle`
- 当前设计明确要求“恢复后回空闲态，不自动续跑”

#### `evReconnectTimeout`

- 含义：通信恢复等待超时
- 来源：当前处于 `CommsLoss`，且 50Hz 计时达到 5 秒仍未检测到 `lower_alive == true`
- 作用：从 `CommsLoss` 转入 `SafeStop`
- `transition_reason` 记录为 `CommsLoss->SafeStop`
- `command_reason` 和 `last_event` 记录为 `reconnect_timeout`
- 当前设计明确要求“超时后不再自动回 `Idle`，必须人工复位”

#### `evInitFailed`

- 含义：`AutoInit` 初始化等待超时，无法继续进入 `Traversing`
- 来源：当前处于 `AutoInit`，且连续 `100 tick50Hz()` 周期未满足 `isReadyToTraverse()`
- 作用：从 `AutoInit` 转入 `SafeStop`
- `transition_reason` 记录为 `AutoInit->SafeStop`
- `command_reason` 和 `last_event` 记录为 `auto_init_failed`
- 当前设计明确要求“超时后不再自动回到 `AutoInit`，必须人工复位”

#### `evAutoRunPause`

- 含义：暂停当前自动运行，并回到 `Idle`
- 来源：当前处于 `Traversing` 且检测到 `auto_run_pause == true`
- 当前输入来源：仅通过 `AutoDebugOverride -> AutoInputMux -> AutoInputSnapshot` 注入，尚未扩展到 `AutoControlRequest`
- 作用：停止当前巡航输出，返回 `Idle`
- 当前不会进入 `SafeStop`，也不会展开越障细分状态

#### `evEmergencyStop`

- 含义：急停事件
- 来源：当前周期检测到 `emergency_stop == true`
- 作用：无论当前是否在自动链路中，优先转入 `SafeStop`
- 这是当前状态机中的最高优先级事件

#### `evManualReset`

- 含义：人工复位事件
- 来源：当前处于 `SafeStop` 且检测到 `manual_reset_requested == true`
- 作用：清理故障标志并从 `SafeStop` 回到 `Idle`
- 当前不会恢复到故障前状态，也不会自动重新启动自动模式

### 判定函数说明

这些判定函数虽然不是 SMC 事件名，但直接决定事件是否触发或状态是否转移。

#### `canStartAuto()`

- 含义：自动启动前的最小就绪判定
- 当前要求同时满足：
  - `lower_alive`
  - `imu_ready`
  - `posture_ready`
  - `grip_confirmed`

#### `isReadyToTraverse()`

- 含义：是否可以从 `AutoInit` 进入 `Traversing`
- 当前实现直接复用 `canStartAuto()`，后续可以单独细化

#### `isObstacleDetected()`

- 含义：当前是否检测到障碍
- 当前只影响 `Traversing` 中 `evTick` 的分支选择
- 还没有展开成独立的“停障/接近/越障”状态

#### `isObstacleNotDetected()`

- 含义：`isObstacleDetected()` 的反条件
- 当前用于在 `Traversing` 中持续刷新巡航命令



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

## 输出模式

控制器通过参数 `output_mode` 控制执行层语义。

- `normal`
  - 默认模式
  - `CommandDispatcher` 直接执行状态机产出的轮速和关节目标
- `safe_hold`
  - 状态机、输入适配层和 `state_trace` 发布继续正常运行
  - 执行层不放行动作请求，而是显式输出安全保持命令
  - 左右轮速度强制为 `0.0`
  - 6 个位置关节目标强制为当前位置

`safe_hold` 的设计目的不是“什么都不做”，而是提供一个对 Gazebo 和实机早期联调都更可控的安全模式。这样即使状态机已经进入 `AutoInit` 或 `Traversing`，执行层也不会真正推动机构动作。

当前未实现 `dry_run`。如果后续需要完全不写句柄的纯离线联调，再单独扩展，不在本包当前范围内。

当前 `state_trace` 会发布：

- 当前状态
- 最近一次状态转移摘要
- 最近一次错误/告警摘要
- 当前命令原因与轮速/冻结标志

建议在命令摘要或调试日志中同时带出当前 `output_mode`，避免现场误判“状态机没出命令”和“命令被安全保持覆盖”为同一种问题。

## Gazebo 状态机验证

如果目标是先验证状态机而不是验证执行层，建议使用 Gazebo 专用启动链路，并将控制器参数设为 `output_mode: safe_hold`。

验证重点：

- `Idle -> AutoInit -> Traversing`
- `Idle -> CommsLoss -> Idle`
- `* -> SafeStop -> Idle`

这条链路当前验证的是：

- 关节和 `base_imu` 接口是否能正常初始化
- `debug_override` / `sensor_input` 是否能驱动状态转移
- `state_trace` 是否能稳定反映当前状态和命令摘要

这条链路当前不验证：

- 真实轮速执行
- 真实关节轨迹执行
- 完整的停障 / 接近 / 越障细分状态链

## 实机早期调试建议

后续进入实机联调时，也可以先使用 `output_mode: safe_hold` 做第一阶段验证，但需要遵守以下约束：

- 同一时刻只加载 `b29_smc_auto_controller`，不要与旧动作控制器并行抢占同一组关节接口
- 先确认 `state_trace`、输入快照和事件优先级行为正确
- 确认状态机逻辑无误后，再切回 `output_mode: normal` 验证真实执行



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
  - `CommsLoss && reconnect timeout -> evReconnectTimeout`
  - `Traversing && auto_run_pause -> evAutoRunPause`
  - `SafeStop && manual_reset_requested -> evManualReset`
  - `Idle && auto_start_requested`
  - 其余进入 `evTick`



## SMC 代码生成

在包目录 `src/b29_control/b29_controllers/b29_smc_auto_controller/` 执行：

```bash
java -jar third_party/smc/bin/Smc.jar -c++ -d gen/src -headerd gen/include sm/RobotFSM.sm
```

当前生成使用包内 vendored 的 `third_party/smc/bin/Smc.jar`。本地这个版本使用 `-c++`，不是旧文档里的 `-lang c++`。如果包内存在 `third_party/smc/include/statemap.h`，构建会优先使用这份头文件；否则再回退到宿主环境中的 `smclib`。本包会同时导出 `gen/include/RobotFSM_sm.h` 与 `include/statemap.h`，保证下游通过 catkin 依赖时公共头链路可用。

如果需要 dot 状态图，单独执行：

```bash
java -jar third_party/smc/bin/Smc.jar -graph -glevel 1 -d gen sm/RobotFSM.sm

# 转化为PNG图片
dot -Tpng gen/RobotFSM_sm.dot -o gen/RobotFSM_sm.png
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
- `auto_init_timeout.yaml`

其中第二个场景文件名沿用计划命名，但当前实际覆盖的是 `Traversing -> CommsLoss -> Idle` 这条已实现链路。
