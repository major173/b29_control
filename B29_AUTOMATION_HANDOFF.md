# B29 双臂机器人自动脱缆与翻越交接

更新时间：2026-07-27

## 1. 当前最重要的状态

自动流程已经整合为“一条启动命令 + 一条开始命令”。2026-07-27 用户先明确反馈 position-only IK 与完成公差修正后的第一遍流程“这次一下就可以了”，即右/后臂脱缆、左锚 MoveIt 翻越和下位机 `RemoteControl` 交权已经实机走通。双遍版本部署后，用户再次实机反馈“一次就成功了”，确认从第一遍交权、夹紧确认与角色切换，到第二遍右锚 MoveIt 翻越和再次进入下位机控制的完整双遍流程已走通一次。本轮没有重新读取成功运行的黑匣子，因此不要补写不存在的时间戳或编码器数值。

两套黑匣子和只读 `/compute_ik`/`/compute_fk` 诊断已确认准确根因：锚定模型和 kinematics YAML 位于 `/gp11_moveit`，但实际 `move_group` 运行在根命名空间，KDL 没有读取到 `position_only_ik: true`，实际在用四个关节求完整姿态。它只在极窄的几何条件下偶发收敛，造成同一目标有时成功、有时所有种子无解。这不是 Adapter、完成公差、碰撞检查或硬件故障。

该问题已做最小修正并部署到 NUC：`gp11_moveit_anchor_runtime.py` 在启动 MoveGroup 前把同一份 kinematics 配置同步到 MoveGroup 实际搜索的命名空间。主机和 NUC 均编译通过；全新 ROS master 启动后日志明确打印 `Using position only ik`，并且在 `avoid_collisions=true` 的生产条件下，原始目标从实时状态直接 IK 成功。2026-07-27 的用户实机反馈确认这项修正和放宽后的完成公差已能完成第一遍交权。

用户随后要求增加第二遍：第一次下位机完成并由上位机发夹紧命令后，必须等新的夹紧确认低到高沿，再把原自由右臂变为锁定臂、原锁定左臂变为自由臂，然后按镜像侧完整重复脱缆、MoveIt 翻越和下位机交权。现有 SMC 下层状态机本来就有这条 `RemoteControl -> Regrip -> Left` 镜像流程，本轮没有重写它；只补齐了上层双锚 MoveIt、第二遍自动调度和双侧 Adapter 完成判断。

双遍修改已于 2026-07-27 部署到 NUC。部署前已正常停止旧 `roslaunch b29_control start.launch`，备份到 `/home/yxj01/B29_ws/.codex_backups/20260727_dual_role_flip`；15 个部署文件主机/NUC SHA256 完全一致。NUC Python、launch XML 和 `catkin build gp11 b29_smc_auto_controller b29_planner_adapter b29_control` 均通过且编译无警告。部署后的完整双遍实机流程已由用户确认一次成功；没有为该次成功重新读取黑匣子。

当前 NUC 已部署内容：

- 确保根命名空间 MoveGroup 实际加载 `position_only_ik: true`。
- 放宽三个影响位姿关节的翻越完成公差。
- 保留自由臂 `right_second=-pi` 的严格要求。
- 自动节点直接接收 Adapter 的详细失败原因。
- 执行监视节点不再把 50 Hz 的轨迹序号、ACK 和编码器过程刷到终端，但黑匣子仍完整记录。
- 发生失败后，详细错误会锁存；自动节点保持运行，每 5 秒只重复一条醒目的 `FATAL`，底层 SMC 继续安全保持当前位置。

本轮新增的双遍内容：

- `start.launch` 改用 `anchor_mode=auto`，自动节点执行 `Right -> Left` 两遍。
- 第一次夹紧确认后，attach bridge 才从左锚切到右锚；不会刚发夹紧命令就释放旧锁定臂。
- 右锚第二遍仍由同一世界目标点实时 IK 求 `right_second`，并选择锁定臂应走的逆时针正向约 180° 等价分支，不能写成当前值加 `pi` 代替 IK。
- 第二遍自由侧 `left_second` 从第二遍实时起点顺时针转 `-pi`，不再依赖绝对 `0.0 rad` 假设。
- Adapter 第二遍检查 `right_second` 正向至少 150°，并把第一遍宽位置公差按角色镜像。
- 等待第二遍期间若下层已进入 `ManualIntervention` 或安全锁存，自动节点立即进入既有失败锁存和每 5 秒 `FATAL`，不无限等待。

## 2. 工程位置与 NUC

- 当前主机开发工程：`/home/unity/B29_ws_0725`
- 之前的参考工程：`/home/unity/B29_ws`
- NUC：`yxj01@192.168.3.252`
- NUC 部署工程：`/home/yxj01/B29_ws`
- 串口：`/dev/ttyUSB0`，波特率 `921600`

此前六个部署文件的主机与 NUC SHA256 已核对一致。

本轮新增部署文件 `gp11_moveit_anchor_runtime.py` 的主机与 NUC SHA256 也已核对一致：

```text
225ccf2d930b35a6c0e40b94c74504e5f593faffde7655a2313ac9174c65691b
```

本轮 MoveGroup position-only IK 修正前备份：

```text
/home/yxj01/B29_ws/.codex_backups/20260726_position_only_ik_namespace
```

NUC 最新修改前备份：

```text
/home/yxj01/B29_ws/.codex_backups/20260726_completion_handoff_logging
```

上一轮稳定等待与 IK 种子修改备份：

```text
/home/yxj01/B29_ws/.codex_backups/20260726_stable_dual_second_ik
```

本轮双遍角色和方向修改备份：

```text
/home/yxj01/B29_ws/.codex_backups/20260727_dual_role_flip
```

## 3. 用户明确规定，不能误改

1. 第一遍固定为右/后臂脱缆，左臂为锁定臂，MoveIt 使用左侧锚定模型；第一次夹紧确认后第二遍镜像为左臂自由、右臂锁定、MoveIt 使用右侧锚定模型。
2. 第一遍判断机器人位姿只使用三个关节：
   - `left_first_leg_joint`
   - `left_second_leg_joint`
   - `right_first_leg_joint`
   第二遍按角色镜像为 `right_first_leg_joint`、`right_second_leg_joint`、`left_first_leg_joint`。
3. 自由臂的 `second` 不参与位姿分析。第一遍自由侧 `right_second` 必须顺时针到 `-pi` 的等价分支；第二遍自由侧 `left_second` 必须从第二遍实时起点顺时针转 `-pi`。
4. 正方向定义为逆时针。锁定臂 `second` 的最终空间角度必须由目标点 IK 求出，不能用“当前值 + pi”代替 IK：第一遍 `left_second` 和第二遍 `right_second` 都选择逆时针约 `+pi` 分支。
5. 脱缆阶段不再让锁定臂 `second` 先顺时针再逆时针。脱缆只移动锁定臂 `first`。
6. 不再使用旧的 `plan_id` 人工确认机制。规划成功后直接发布轨迹。
7. 姿态/IMU 摇摆不能阻止脱缆启动；`posture_ready` 已从启动门槛移除。但 `imu_ready`、`lower_alive`、急停和硬件故障检查仍保留。
8. 脱缆失败不自动重试，应保持不动并明确报错。
9. 现在夹爪确认通信还未正式完成，`start.launch` 默认临时允许在 `grip_confirmed=false` 时启动脱缆。后续通信协议完成后应取消这个临时旁路。

## 4. 当前自动流程

```text
Idle/Traversing
  -> 收到 start_disconnect
  -> 固定 Right/rear crossing side
  -> 打开右夹爪并启用重力补偿
  -> 左锁定臂 first 抬升到 -0.53 rad
  -> 左锁定臂 first 回到 -0.05 rad
  -> Step7：三个姿态关节速度连续稳定
  -> DisconnectDoneWaitFlip
  -> 自动节点等待实时编码器稳定
  -> 自动发布 start_flip
  -> PlannerControl
  -> 实时 IK，失败后尝试备用 IK 种子
  -> MoveIt 规划并直接下发轨迹
  -> 翻越完成判断
  -> complete_planner_control 服务
  -> RemoteControl（控制权交给下位机）
  -> 下位机把自由臂移动到线缆处并发完成上升沿
  -> Regrip（上位机命令夹爪闭合）
  -> 必须收到新的 grip_confirmed 低到高沿
  -> 角色切换：右臂锁定、左臂自由，MoveIt 切到右锚
  -> 打开左夹爪并启用右锁定臂重力补偿
  -> 使用 Left crossing profile 重复镜像脱缆
  -> 第二次 DisconnectDoneWaitFlip
  -> 右锚实时 IK、right_second 逆时针正向翻越、Adapter 完成判断
  -> 第二次 complete_planner_control
  -> 第二次 RemoteControl（再次交给下位机）
```

`RemoteControl` 的四关节增量和完成上升沿来自下位机反馈帧，不能用现有调试 override 伪造。上下位机正式通信协议后续还会修改。

## 5. 启动与测试命令

不需要另外启动 GP11 launch，`b29_control/start.launch` 已包含 MoveIt、Adapter、自动翻越节点和两个黑匣子。

终端 1：

```bash
roslaunch b29_control start.launch
```

终端 2：

```bash
rostopic pub -1 /b29_controller/b29_smc_auto_controller/start_disconnect std_msgs/Empty "{}"
```

监听状态：

```bash
rostopic echo /b29_controller/b29_smc_auto_controller/state_trace
```

修改部署后必须停止旧的 `roslaunch` 再重新启动，否则旧进程仍使用旧参数和旧 Python 代码。

## 6. 当前关键参数

### 脱缆

文件：`src/b29_control/config/controller.yaml`

```yaml
planner_interface:
  max_delta_per_command: 0.06

obstacle_crossing:
  disconnect_cable_first_joint_up_position: -0.53
  disconnect_cable_first_joint_down_position: -0.05
  disconnect_settle_velocity_threshold: 0.02
  disconnect_settle_duration: 0.5
```

Step7 只判断三个姿态关节的绝对速度都不超过 `0.02 rad/s`，并连续保持 `0.5 s`。不再判断固定关节角度，因为重力会造成每次静止角度不同。

### 脱缆后开始 IK 前的稳定门槛

文件：`src/b29_control/launch/start.launch`

```text
连续稳定：1.0 s
位置窗口跨度：<= 0.01 rad
绝对速度：<= 0.02 rad/s
joint_states 最大年龄：0.5 s
最长等待：20 s
```

每一遍只检查当前角色对应的三个影响位姿关节。超时则不进入 IK。

### IK 目标与备用种子

目标点：

```text
frame: world
x:  0.0060528
y: -0.00000272
z: -0.7399961
```

第一遍左锚实时 IK 失败时的三组种子规则：

```text
两个 first：保留实时编码器值
left_second：0、0.5*pi、0.9*pi
right_second：三组都固定为 -pi
```

IK 成功后仍强制：

```text
left_second：选择正向大翻越分支
right_second：最终目标固定 -pi
```

第二遍右锚按返回方向配置：

```text
两个 first：保留第二遍实时编码器值
right_second 备用种子：从实时 right_second 起点依次加 0、0.5*pi、0.9*pi
right_second 最终目标：仍由同一目标点 IK 求出，再选择锁定臂逆时针正向约 pi 的等价分支
left_second：从第二遍实时 left_second 起点减 pi；备用种子和最终目标使用同一实时相对目标
```

注意：连续关节在 Adapter 或编码器中可能显示为约 `+pi`，它与 `-pi` 空间姿态等价；不要只凭显示正负号判断目标是否失效，应结合规划分支和实际运动方向。

### Adapter 与翻越完成

文件：`src/b29_planner_adapter/config/planner_adapter.yaml`

```yaml
max_output_delta: 0.05

large_flip_completion:
  min_left_second_displacement: 2.6179938779914944  # 兼容字段名；按每遍指定方向检查150 deg
  position_tolerance: [0.10, 0.18, 0.12, 0.05]
  velocity_tolerance: [0.02, 0.02, 0.02, 0.02]
  settle_time: 1.0
  timeout: 20.0
```

配置数组顺序固定为：左一、左二、右一、右二。第一遍按原数组使用；第二遍按角色镜像为 `[0.12, 0.05, 0.10, 0.18]`，因此第二遍自由臂 `left_second` 仍使用严格 `0.05 rad`，锁定臂 `right_second` 使用 `0.18 rad`。

完成条件为：

1. 当前锁定侧 `second` 正向转过至少 150°：第一遍为 `left_second`，第二遍为 `right_second`，均逆时针；
2. 四关节位于上述宽终点公差内；
3. 四关节速度均小于 `0.02 rad/s`；
4. 连续保持 1 秒。

全部满足后 Adapter 调用 `complete_planner_control`，SMC 才进入 `RemoteControl`。

## 7. 最近实机数据与根因

### 2026-07-27：用户确认完整双遍一次成功

用户先反馈第一遍按既有启动方式“一下就可以了”，确认了 position-only IK 修正、既有正向分支、`right_second=-pi` 和放宽后的完成公差已完成一次第一遍实机闭环并进入下位机控制。双遍版本部署后，用户再次反馈“一次就成功了”，确认夹紧确认后的角色切换、第二遍右锚 predicted-point IK、锁定侧 `right_second` 逆时针正向约 `+pi`、自由侧 `left_second` 从实时位置顺时针 `-pi`，以及第二次下位机交权已完整走通一次。本轮没有为成功运行重新读取最新两套黑匣子，因此这里只记录用户确认，不虚构具体事件时间或编码器数据。

下面两组 2026-07-26 数据保留为此前故障根因和公差修改依据。

### 2026-07-26 15:31：最近一次已记录失败，IK 前停止

文件：

```text
/home/yxj01/.ros/gp11_blackbox/gp11_motion_20260726_153152_events.jsonl
/home/yxj01/.ros/gp11_blackbox/gp11_motion_20260726_153152_samples.csv
/home/yxj01/.ros/b29_disconnect_blackbox/disconnect_20260726_153152_events.jsonl
/home/yxj01/.ros/b29_disconnect_blackbox/disconnect_20260726_153152_samples.csv
```

准确事件顺序：

```text
15:33:03.030  DisconnectDoneWaitFlip
15:33:04.030  三个姿态关节连续稳定 1.00 s
15:33:04.050  PlannerControl session 1
15:33:04.054  开始 IK/终点检查
15:33:05.058  实时 IK 返回 NO_IK_SOLUTION
15:33:06-08 三组备用种子全部返回 -31
15:33:08.078  自动流程锁存安全失败；未生成、未发布轨迹
```

当时实时四关节：

```text
[0.0122014, 0.0102254, 0.00989456, -0.00006966]
```

黑匣子中的三组备用种子只改变 `left_second`，均保持实时两个 `first` 和 `right_second=-pi`；它们都失败。随后只读诊断进一步确认：

- 关闭碰撞检查仍为 `-31`，排除终点碰撞；
- MoveIt FK 与实时 TF 一致；
- 用 FK 的完整姿态做 IK 成功，但同一位置改为单位姿态即失败，证明 KDL 没有启用 position-only；
- 临时把 kinematics YAML 放到根 `/robot_description_kinematics` 并重同步 MoveIt 后，原始目标立即成功；
- 永久修正部署并清空 ROS master 重启后，日志打印 `Using position only ik`；`avoid_collisions=true` 时原始目标仍成功。

不要为了这次失败继续扩充 IK 种子。当前种子规则和 `right_second=-pi` 要求不需要改变。

### 2026-07-26 15:09：机械翻越成功但旧完成公差未通过

该次运行文件：

```text
/home/yxj01/.ros/gp11_blackbox/gp11_motion_20260726_150902_events.jsonl
/home/yxj01/.ros/gp11_blackbox/gp11_motion_20260726_150902_samples.csv
/home/yxj01/.ros/b29_disconnect_blackbox/disconnect_20260726_150902_events.jsonl
/home/yxj01/.ros/b29_disconnect_blackbox/disconnect_20260726_150902_samples.csv
```

该次实时 IK 直接成功，没有使用备用种子。机械翻越也成功。20 秒完成超时实际触发于 `15:10:20.467`，数据为：

```text
left_second displacement = 3.14183 rad       通过
position_errors = [-0.00638, 0.07071, -0.08376, -0.00078]
velocities = [-0.00244, -0.00244, -0.00244, -0.00733] rad/s
```

旧的 `right_first` 公差为 `0.08 rad`，实际误差为 `0.08376 rad`，只超出约 `0.00376 rad`（0.22°）。因此 Adapter 没有调用完成服务，而是发送当前位置保持并留在 `PlannerControl`。现在 `right_first` 公差已放宽为 `0.12 rad`，同样的数据会通过。

## 8. 日志与失败行为

以前终端刷屏的主要来源是 `gp11_real_execution_monitor.py`：它会对每个 50 Hz 轨迹序号、SMC ACK、dispatch 和编码器变化调用 `loginfo`。

现在：

- 高频过程仍发布到监视话题和黑匣子，但不输出到终端。
- Adapter 失败会锁存为终端状态，普通过程回调不能覆盖它。
- `SingleFlipClient` 直接订阅 Adapter action result，因此会打印完整 `error_string`，不再只有泛化的 `MoveIt code -4`。
- 自动翻越失败后不退出并消失，而是每 5 秒重复一条 `AUTOMATIC FLIP STOPPED SAFELY` 的 `FATAL`。
- 失败不会继续规划或重发动作；SMC 保持当前位置。需要排查后重启整个 `start.launch` 才进行下一次测试。

黑匣子位置：

```text
~/.ros/gp11_blackbox/
~/.ros/b29_disconnect_blackbox/
```

查看最新文件：

```bash
find ~/.ros/gp11_blackbox ~/.ros/b29_disconnect_blackbox -type f -printf '%T@ %p\n' | sort -nr | head
```

## 9. 后续测试与故障排查顺序

1. 当前成功基线已经部署并完成过一次完整双遍实机流程。后续复测启动整套 `start.launch` 后，仍应先只观察初始左锚/Right 状态；不要立即发送 `start_disconnect`。
2. 确认日志显示 `anchor=left`、`Using position only ik`，并确认控制器、串口和两个黑匣子节点正常后，再进行完整双遍复测。
3. 第一遍交权成功时，在 `state_trace` 中应看到：
   - `obstacle_crossing_stage: "RemoteControl"`
   - `planner_control_active: false`
   - `remote_control_active: true`
4. 第一遍启动终端应打印：
   - `Commissioned large flip accepted...`
   - `MoveIt flip completed... lower-level RemoteControl handoff is active`
   - `Lower-level handoff confirmed...`
5. 第一遍下位机完成后，必须观察到 `Regrip`、新的夹紧确认低到高沿、`crossing_side: "Left"`、`anchor=right`，然后才会启动第二遍；第二遍同样应回到 `RemoteControl`。
6. 如果任一遍仍未交权，不要猜测，也不要立刻再发动作。读取最新两套黑匣子，重点找：
   - `adapter_action_result`
   - `cartesian_goal_result`
   - `planner_state_changed`
   - 最后一段 samples 的四关节实际位置、速度、目标和 session 状态
7. 如果出现 `AUTOMATIC FLIP STOPPED SAFELY`，它后面的 `adapter_detail` 就是根因；若是第二遍等待失败，先确认是 `ManualIntervention`、锚同步失败还是右侧 IK/完成判断失败。
8. 若机械动作正确但仍只差少量完成公差，应基于实际黑匣子调整；不要取消 150° 条件、指定方向和静止条件。

## 10. 最新修改涉及的文件

```text
src/b29_planner_adapter/config/planner_adapter.yaml
src/b29_planner_adapter/include/b29_planner_adapter/planner_adapter.h
src/b29_planner_adapter/src/planner_adapter.cpp
src/b29_controllers/b29_smc_auto_controller/test/test_obstacle_crossing_runtime.cpp
src/b29_sim_utils/gp11/CMakeLists.txt
src/b29_sim_utils/gp11/test/test_dual_anchor_flip.py
src/b29_sim_utils/gp11/src/gp11/cartesian_goal_core.py
src/b29_sim_utils/gp11/src/gp11/single_flip_client.py
src/b29_sim_utils/gp11/scripts/gp11_automatic_flip.py
src/b29_sim_utils/gp11/scripts/gp11_b29_attach_bridge.py
src/b29_sim_utils/gp11/scripts/gp11_plan_direction_audit.py
src/b29_sim_utils/gp11/scripts/gp11_cartesian_goal_server.py
src/b29_sim_utils/gp11/launch/gp11_moveit_real.launch
src/b29_control/launch/start.launch
```

验证结果：

- 本轮主机 Python `py_compile`：通过
- 本轮主机 launch XML：通过；`roslaunch b29_control start.launch --nodes` 可解析全部节点
- 本轮主机 `catkin build gp11 b29_smc_auto_controller b29_planner_adapter b29_control`：通过
- 本轮 GP11 双锚与双方向离线测试：8/8 通过
- 本轮 SMC 自动控制器测试：28/28 通过（含夹紧确认后角色切换和镜像脱缆用例）
- 本轮 15 个部署文件主机/NUC SHA256：完全一致
- NUC Python `py_compile` 和 launch XML：通过
- NUC `catkin build gp11 b29_smc_auto_controller b29_planner_adapter b29_control`：通过，无警告
- NUC GP11 双锚方向测试：8/8 通过；SMC 本轮相关角色切换测试：6/6 通过
- NUC 全套 SMC 测试另有一个既有测试数据不一致：`FreezesReferenceAfterFirstCommand` 用默认 `max_delta_per_command=0.06` 却期望 `0.05 -> 0.14`（差 `0.09`）被接受；生产实现正确拒绝。该测试文件不在主机当前工程中，本轮未改生产代码迁就此旧测试
- 部署后的完整双遍实机流程已由用户确认一次成功；本轮未读取该次成功黑匣子，不补写具体事件数据
- 之前 position-only IK 修正的 NUC SHA256、全新 ROS master 日志和生产条件 IK 证据仍见前述历史记录
