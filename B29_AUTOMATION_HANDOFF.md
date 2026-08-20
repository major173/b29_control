# B29 工程总交接手册

更新时间：2026-08-13

适用代码基线：`dual-role-flip--20260808`，提交 `181e66d`

仓库：`git@github.com:yi-ybk/b29_asncy.git`

> 这是下一个开发会话的首要信息源。开始处理工程前必须完整阅读本文，再按需阅读各包 README。若本文与历史聊天、旧备份或其他 Markdown 冲突，以“当前分支代码 + 本文”优先。不要仅凭旧 NUC 文件反向覆盖主机代码。

## 1. 当前结论

- 主机正式开发工程：`/home/unity/B29_ws_0725`
- Git 根目录：`/home/unity/B29_ws_0725/src`
- 当前开发分支：`dual-role-flip--20260808`
- 当前远端同名分支：`origin/dual-role-flip--20260808`
- 当前提交：`181e66dbb50143857d6b9a73310b5f560434aecc`
- 同事基线分支：`origin/dev/controllerfix`，当前可见提交 `890c567`
- 新 NUC：`yxj02@192.168.3.121`，工程 `/home/yxj02/B29_ws`
- 旧 NUC `yxj01@192.168.3.252` 和旧主机工程 `/home/unity/B29_ws` 只可作历史参考，不是当前部署目标或开发基线。
- 本分支建立在最新同事 `controllerfix` 基线上，主要增加/修改 MoveIt、Planner Adapter、ROS control 越障衔接和相关文档；不要把旧 NUC 的仓库历史或无关 RL 改动混入本分支。
- 当前主机工作区在本文重写前是干净的，当前文档整理尚未提交。
- 2026-08-13 无法连接新 NUC（`No route to host`），因此本文中的 NUC 服务配置来自已部署记录，未做本次在线复核。再次部署前必须检查 NUC 分支、状态和差异。

在本开发链路中已由用户实机确认过的能力（后续提交均是在这些成功流程上做的定向调整；重新部署后仍应按本文检查）：

- 单个障碍的双遍角色切换与翻越完整成功。
- 连续处理多个障碍的上层外循环成功。
- 默认前进方向连续翻过两个障碍成功。
- 正反向使用两套预测点；后向目标点已按实机误差调整。
- 下位机 RemoteControl、回夹、角色切换、下一遍脱缆和下一障碍复位已经串起来。
- 下位机新反馈帧和真实轮子方向请求已验证可用。

当前最新代码还包含：

- 最近一次非零巡航方向决定每个障碍的第一遍侧别；停止帧不覆盖方向。
- 每个障碍固定两遍，完成后等待正常回夹、障碍触发下降沿和 SMC 回到 `Idle`，再接收下一个障碍。
- 回夹确认独立等待 20 秒；超时只保留既有“重新张开并回 RemoteControl”的回夹重试。
- RemoteControl 增量缩放由 `0.02` 提高到 `0.026`。
- 曾加过的“回夹后双 first_joint 自动归零”效果不好，已经删除，绝对不要误以为仍存在。
- 曾加过的 Planner Adapter“启动确认、5 次重试、每次 1 秒、轨迹重锚”导致一次翻越都无法成功，已经完整回退，绝对不要重新带回。

## 2. 开始下一会话时先做什么

只读确认：

```bash
cd /home/unity/B29_ws_0725/src
git branch --show-current
git status --short --branch
git log -5 --oneline --decorate
```

预期：

```text
branch = dual-role-flip--20260808
HEAD   = 181e66d（除非本文之后有明确的新提交）
```

如果用户要求改代码：

1. 先确认工作区没有同事正在进行 merge/rebase，也没有不属于当前任务的改动。
2. 先读本文，再读与任务直接相关的包文档和代码。
3. 不要重新设计已经实机成功的流程。
4. 不要控制真机，除非用户明确要求启动、部署或测试。
5. 若要部署新 NUC，先检查 NUC 的 `src` Git 根、分支和状态；不要把整个工作区或旧 `.git` 历史盲目覆盖过去。

## 3. 工程边界和主要组件

```text
b29_control
  真机串口、上下位机帧、硬件接口、start.launch

b29_controllers/b29_smc_auto_controller
  RobotFSM、巡航、越障子状态机、脱缆、PlannerControl、RemoteControl、Regrip

b29_planner_adapter
  MoveIt FollowJointTrajectory -> SMC PlannerControl 单点命令与反馈监控

b29_sim_utils/gp11
  双锚 MoveIt、实时 IK、目标点规划、方向审计、连续障碍上层调度、黑匣子

b29_tools/rqt_b29_smc_console
  调试观察工具；不是正式业务输入源
```

正式真机入口只有：

```bash
roslaunch b29_control start.launch
```

`start.launch` 默认同时启动硬件、控制器、Adapter、MoveIt、自动翻越节点和两套黑匣子。不要再单独启动 GP11 MoveIt，否则会重复启动 MoveGroup、TF、Action Server 或监控节点。

当前仓库不提供 B29 Gazebo 启动入口。`robot_control/rm_gazebo` 的模板 README 和残留包不是当前越障流程依据。

## 4. 必须先搞清楚的术语和物理映射

`crossing_side` 表示“当前张开/自由夹爪的一侧”，不是锚定侧，也不是锁定臂一侧。

| crossing_side | 张开/自由夹爪 | 仍在线缆上的锚 | 脱缆移动的锁定 first | 锁定 second | 自由 second |
|---|---|---|---|---|---|
| `Right` | 右 | 左锚 | `left_first` | `left_second` | `right_second` |
| `Left` | 左 | 右锚 | `right_first` | `right_second` | `left_second` |

代码中的 `CrossingSideProfile` 已固化此映射：

- `Right`：打开右夹爪、移动左锁定臂、左一重力补偿。
- `Left`：打开左夹爪、移动右锁定臂、右一重力补偿。

合并分支使用上位机重力前馈，不使用下位机旧重力模型。状态机的
`gravity_compensation_mode` 直接驱动上位机支撑侧：`LeftFirstLeg` 对应左锚，
`RightFirstLeg` 对应右锚。V2 控制帧中的旧下位机重力字段始终发送 `0`。

上位机力矩前馈从 `EnableGravityCompensation` 开始，覆盖 `Disconnecting`、
`DisconnectDoneWaitFlip`、Planner、遥控、重夹和脱离失败后的人工介入；确认重新夹紧后关闭。
因此巡航和普通 Idle 状态不会发送重力力矩，而脱离电缆及脱离后等待翻转时会持续补偿。

行进方向和首遍侧别：

| 下位机巡航请求 | 语义 | 第一遍 crossing_side | 说明 |
|---|---|---|---|
| `1` | Forward | `Left` | 当前修正后的物理映射；`Left` 是张开侧，右臂锁定 |
| `2` | Reverse | `Right` | 镜像流程 |
| `0` | Stop | 不更新 | 翻越前停车不会抹掉上一个非零方向 |

如果启动后从未收到非零方向，越障保持 `Idle`，不会猜方向，也不会用编码器累计值替代。轮位置累计值现在只用于诊断。

注意目标 profile 命名中的历史兼容：`first_crossing_side=Right` 选 `default_forward` 点，`first_crossing_side=Left` 选 `reverse` 点。不要按字符串名字猜物理方向，应以 SMC 侧别和实机已验证映射为准。

## 5. 当前完整工作流程

### 5.1 外层运行与触发

```text
Idle
  -> 下位机 AutoStart 0->1
AutoInit
  -> 通信、IMU、硬件条件满足
Traversing
  -> 接收巡航请求并记住最近非零方向
  -> 下位机 obstacle_crossing_trigger 0->1，或人工 start_disconnect
```

正式业务应使用下位机触发。人工 `start_disconnect` 主要用于受控测试；当前 `start.launch` 暂时允许它绕过初始 `grip_confirmed=false` 门槛。

### 5.2 每个障碍的第一遍

```text
依据最近非零巡航请求锁存 first_crossing_side
  -> OpenGripperBeforeGravityCompensation
  -> 固定等待夹爪张开 15 s（没有单侧张开反馈）
  -> EnableGravityCompensation
  -> Disconnecting
       只移动锁定臂 first：抬升 -> 回落 -> 速度稳定
  -> DisconnectDoneWaitFlip
  -> 自动节点等待三个相关姿态关节稳定
  -> 自动发布 start_flip
  -> PlannerControl
  -> 按本障碍方向选择预测点
  -> 当前锚实时 IK、备用种子、碰撞与轨迹方向审计
  -> MoveIt -> Adapter -> SMC -> 编码器闭环执行
  -> Adapter 判断功能完成并 complete_planner_control
  -> RemoteControl
```

### 5.3 下位机交权、回夹和角色切换

```text
RemoteControl
  -> 下位机发送四关节增量，人工把自由臂挂回线缆
  -> 当前阶段出现新的 completion 0->1 上升沿
Regrip
  -> 上位机关闭当前自由侧夹爪
  -> 必须先看到 grip_confirmed=false，再看到新的 false->true
  -> 第一遍成功：切换到 oppositeSide，原自由臂成为锁定臂
  -> 自动开始第二遍的张爪、脱缆、MoveIt 和 RemoteControl
```

回夹确认超时为 20 秒。仅回夹超时保留既有重试：重新张开夹爪，回到 RemoteControl，让下位机再次调整；`retry_limit=2`。脱缆失败和规划失败不自动重发动作。

### 5.4 第二遍和多障碍外循环

第二遍是第一遍的镜像侧，仍用同一障碍开始时锁存的方向 profile。第二遍交权并回夹成功后：

```text
CompleteWaitObstacleClear
  -> 等待 obstacle_crossing_trigger 新的 1->0 下降沿
  -> SMC reset 到 Idle
  -> 上层丢弃本障碍侧别
  -> 等待下一障碍的新非零巡航方向和新上升沿
```

`continuous_obstacles=true`，外层不需要固定等待时间，也不需要重新启动节点。没有正常第二次回夹、完成等待和障碍下降沿，就不会接受下一障碍。

## 6. MoveIt、双锚、IK 和方向硬要求

### 6.1 两个方向使用两套预测点

文件：`b29_control/launch/start.launch`

```yaml
# first_crossing_side=Right 使用
target_frame: world
target_x:  0.0060528
target_y: -0.00000272
target_z: -0.7399961

# first_crossing_side=Left 使用
reverse_target_frame: world
reverse_target_x:  0.1700000
reverse_target_y: -0.00000272
reverse_target_z:  0.7399961
```

后向 `x=0.17` 是基于实机约 10°～11°稳定跟踪偏差做的 IK 预测点补偿。它仍然是笛卡尔目标点，锁定 second 仍由实时 IK 求出，不能改成写死关节角度。

### 6.2 position-only IK 不能丢

`b29_sim_utils/gp11/config/gp11_moveit_kinematics_real.yaml`：

```yaml
reach_arm:
  position_only_ik: true
```

历史根因：锚定 kinematics 曾只加载在 `/gp11_moveit`，而实际 `move_group` 在根命名空间，KDL 没读到 `position_only_ik`，导致相同目标偶发有解、偶发全部 `NO_IK_SOLUTION`。当前 `gp11_moveit_anchor_runtime.py` 会把配置同步到 MoveGroup 实际命名空间。启动日志必须能看到：

```text
Using position only ik
```

看到 `moveit_ros_move_group not found` 或 `/get_planning_scene unavailable` 时，首先是 NUC 缺 MoveIt 依赖/环境，不是目标点或自动触发逻辑。

### 6.3 锁定臂 second 的绝对硬要求

- 正方向定义为逆时针。
- 两遍锁定臂 second 都必须走逆时针正向约 180°的等价连续关节分支。
- `Right` 遍：`left_second` 是锁定 second。
- `Left` 遍：`right_second` 是锁定 second。
- 最终空间姿态必须由当前预测点实时 IK 产生；只允许选择 IK 解的等价 `2*pi` 分支，不能用“当前值 + pi”代替 IK。
- 必须保留至少 150° 的指定方向大翻越检查。

### 6.4 自由臂 second 的绝对硬要求

- 左锚/`Right` 遍：自由 `right_second` 目标固定为 `-pi` 等价分支。
- 右锚/`Left` 遍：自由 `left_second` 从该遍实时起点顺时针转 `-pi`。
- 自由 second 不参与三关节位姿完成分析，但仍参与四关节轨迹执行、公差和安全检查。
- 连续关节显示 `+pi` 或 `-pi` 可能空间姿态等价；判断必须结合展开分支和实际运动方向，不能只看最终打印符号。

### 6.5 实时种子与备用种子

- 两个 first 始终从当前实时编码器值出发。
- `flip_ik_seed_fractions = [0.0, 0.50, 0.90]`。
- 左锚锁定 `left_second` 种子为 `fraction*pi`；自由 `right_second` 固定 `-pi`。
- 右锚锁定 `right_second` 种子为实时 `right_second + fraction*pi`；自由 `left_second` 使用实时起点减 `pi` 的目标。
- 不要因为一次 IK 失败就无限扩充种子；先确认 position-only 参数、锚点、实时关节和目标 profile。

### 6.6 不能误改重定根轴符号

`gp11_moveit_real.launch` 故意不覆盖生成器的反向链轴符号。锚点重定根时默认 `-1` 是正确逆运动学变换。曾错误强制 `+1`，会让正 yaw 反馈在锚定模型里看成负 yaw，形成越来越大的闭环误差。下位机没改也会只在角色切换后的锚模型暴露这个问题。

## 7. 当前关键参数

### 7.1 硬件与控制周期

文件：`b29_control/config/hardware.yaml`、`b29_control/src/main.cpp`

```text
串口       /dev/ttyUSB0
波特率     921600
硬件主循环 50 Hz
```

### 7.2 SMC 与脱缆

文件：`b29_control/config/controller.yaml`

```yaml
planner_interface:
  max_delta_per_command: 0.06
  command_timeout: 1.0
  total_watchdog_timeout: 120.0

obstacle_crossing:
  wait_for_grip_respond_time: 15.0
  regrip_confirmation_timeout: 20.0
  disconnect_cable_step_duration: 5.0
  disconnect_cable_step_interval: 1.0
  disconnect_cable_first_joint_up_position: -0.53
  disconnect_cable_first_joint_down_position: -0.05
  disconnect_settle_velocity_threshold: 0.02
  disconnect_settle_duration: 0.5
  retry_limit: 2

remote_control:
  increment_deadband: 0.005
  increment_scale: 0.026
  max_increment_per_sample: 0.05
  joint_direction_signs: [-1, +1, +1, +1]
```

脱缆阶段只移动锁定臂 first，不再给锁定 second 加偏移。Step7 只要求当前 profile 的三个姿态关节速度绝对值不大于 `0.02 rad/s` 并连续稳定 `0.5 s`，没有固定角度门槛，也没有 Step7 超时。

### 7.3 脱缆后开始 IK 的稳定门槛

文件：`b29_control/launch/start.launch`

```text
最长等待                 20.0 s
连续稳定                 1.0 s
位置窗口跨度             <= 0.01 rad
绝对速度                 <= 0.02 rad/s
joint_states 最大年龄    0.5 s
```

每遍只看与当前锚姿态有关的三个关节：

- `Right`：`left_first`、`left_second`、`right_first`
- `Left`：`right_first`、`right_second`、`left_first`

### 7.4 Planner Adapter

文件：`b29_planner_adapter/config/planner_adapter.yaml`

```yaml
publish_rate: 50.0
time_scale: 1.0
max_output_delta: 0.05

tolerances:
  start: [0.08, 0.08, 0.08, 0.08]
  path: [0.50, 0.50, 0.50, 0.50]
  goal_position: [0.02, 0.02, 0.02, 0.02]
  goal_velocity: [0.05, 0.05, 0.05, 0.05]

large_flip_completion:
  min_left_second_displacement: 2.6179938779914944  # 150 deg，兼容字段名
  position_tolerance: [0.15, 0.23, 0.17, 0.05]
  velocity_tolerance: [0.02, 0.02, 0.02, 0.02]
  settle_time: 1.0
  timeout: 20.0
```

数组固定顺序为 `[左一, 左二, 右一, 右二]`。`Left` 遍会在代码中按角色镜像，因此宽公差随锁定/自由角色镜像。2026-08-10 前后曾按用户要求把三个相关完成公差各放宽 `0.05 rad`，当前配置 `[0.15, 0.23, 0.17, 0.05]` 是现行实机基线，不能被旧文档中的 `[0.10, 0.18, 0.12, 0.05]` 覆盖。

Adapter `max_output_delta=0.05` 必须不大于 SMC `0.06`。之前 NUC 参数漂移曾触发：

```text
adapter max_output_delta exceeds the active SMC session limit
```

遇到该错误先对比两端加载参数，不要盲目扩大 SMC 上限。

## 8. 上下位机通信中与本流程直接相关的字段

完整帧见根 README。当前要点：

- 控制帧 52 字节，11 个 float payload + 独立重力补偿字节。
- 反馈帧 173 字节，payload 长度 166。
- 反馈 payload 偏移 144～159（帧绝对字节 149～164）：四个 RemoteControl `float32`，顺序左一、左二、右一、右二。
- payload 偏移 160（绝对字节 165）：RemoteControl 完成位。
- payload 偏移 161（绝对字节 166）：`cruise_drive_request`，`0=Stop, 1=Forward, 2=Reverse`。
- payload 偏移 162（绝对字节 167）：自动控制 bit mask，bit0 AutoStart、bit1 ManualReset、bit2 ObstacleCrossingTrigger。
- payload 偏移 164（绝对字节 169）：两个夹爪都夹紧的 `grip_confirmed`。

当前方向判断必须使用最近一次非零 `cruise_drive_request`，不是轮子里程累计值。翻越前请求为 0 是正常停车，不应触发动作或清空方向。

## 9. 启动、服务和观察

### 9.1 手动启动

在 NUC：

```bash
cd /home/yxj02/B29_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch b29_control start.launch mode:=dual_role
```

不要同时运行 systemd 服务和手动 `roslaunch`。如果服务正在运行，先停服务；否则会争用串口、ROS master 和控制器。

### 9.2 新 NUC systemd

已部署记录：

```text
/etc/systemd/system/b29-control.service
/etc/systemd/system/b29-control.service.d/10-start-delay.conf
```

drop-in 内容：

```ini
[Service]
ExecStartPre=/bin/sleep 10
```

这个 10 秒延时是为避免 NUC 服务和下位机上电初始化重合。它对开机启动和手动 `restart` 都生效。

```bash
sudo systemctl restart b29-control.service
systemctl status b29-control.service
journalctl -u b29-control.service -f
```

服务只启动 ROS 整套流程，不应自动发布脱缆/翻越动作。不要给 systemd 增加自动 `start_disconnect`。

### 9.3 常用只读观察

```bash
rostopic echo /b29_controller/b29_smc_auto_controller/state_trace
rostopic echo /b29_controller/b29_smc_auto_controller/planner_control_state
rostopic echo /gp11_moveit/runtime_anchor
rostopic echo /gp11_moveit/runtime_status
```

受控人工测试入口：

```bash
rostopic pub -1 /b29_controller/b29_smc_auto_controller/start_disconnect std_msgs/Empty "{}"
```

这会触发真机动作，只能在用户明确要求、现场确认安全后执行。

## 10. 黑匣子与故障排查纪律

黑匣子：

```text
~/.ros/gp11_blackbox/
~/.ros/b29_disconnect_blackbox/
```

查看最新：

```bash
find ~/.ros/gp11_blackbox ~/.ros/b29_disconnect_blackbox \
  -type f -printf '%T@ %p\n' | sort -nr | head -20
```

铁律：脱缆后没有翻越、未进入 RemoteControl、第二遍失败或行为与肉眼观察不一致时，不要猜，不要立即重复发送动作。先读取同一次启动的最新两套黑匣子。

重点事件：

```text
adapter_action_result
cartesian_goal_result
planner_state_changed
smc_trace_changed
```

重点样本：

- 四关节 actual position / velocity
- MoveIt/Adapter goal
- Planner session / sequence / ACK
- `dispatch_succeeded`
- SMC hold/freeze 状态
- `crossing_side`、`first_crossing_side`、锚点
- `grip_confirmed` 和阶段转换

先区分：

1. MoveIt/IK 根本没有生成轨迹。
2. Adapter 因配置或安全门禁拒绝。
3. SMC 接受并下发，但下位机/机构没有响应。
4. 夹爪没有真正张开造成机械阻塞。
5. 实际在动但 MoveIt 外层先超时。
6. 功能完成公差没有满足，尚未交权。

## 11. 已踩过的坑和准确结论

### 11.1 TF 全是 NaN

新 NUC 初次部署出现整树 NaN，不是少传 URDF。根因是硬件反馈/串口数据未形成有效有限关节状态，NaN 经 `/joint_states` 传播到 `robot_state_controller` 和 TF。先检查 `/dev/ttyUSB0`、权限、波特率、反馈帧和下位机初始化，不要改 TF 数学或 URDF掩盖 NaN。

### 11.2 脱缆成功但一直停在 DisconnectDoneWaitFlip

曾经是自动翻越节点未同步/未启动或旧进程仍在运行。当前 `start.launch` 会启动 `gp11_automatic_flip`。检查节点、`start_flip` 订阅者、MoveIt Action Server 和服务环境，不要不断重发 `start_disconnect`。

### 11.3 `moveit_ros_move_group` 找不到

这是 NUC 缺 MoveIt 包或 ROS 环境没有 source，导致 `/get_planning_scene` 不可用；不是自动逻辑错误。依赖补齐后再测。

### 11.4 IK 偶发全部失败

历史根因是 `position_only_ik` 命名空间错位。确认日志 `Using position only ik`，不要先扩大种子或改目标方向。

### 11.5 `code -4`

MoveIt `-4` 不是单一根因，必须读 `adapter_detail`：

- `max_output_delta exceeds ...`：Adapter/SMC 参数漂移。
- `joint feedback exceeded configured path tolerance`：实际反馈与轨迹误差超过 path tolerance，可能是机构跟不上、机械阻塞、下位机冻结或目标方向/坐标模型错误。

不要只看到 `-4` 就放宽公差。

### 11.6 `code -6`

MoveIt 外层执行超时。曾出现 Adapter/SMC 最后实际完成，但 MoveIt 在预计上限先 cancel，随后 Adapter 报 preempt。功能完成公差已按实机放宽，但仍需用黑匣子判断是实际太慢、冻结还是只差完成门槛。

### 11.7 第二遍 yaw 方向看似对但约 30°停住

最终围绕“第一遍成功、第二遍失败”的差异定位到锚定重定根轴符号，而不是下位机代码和目标点。不要再次强制 inverse axis `+1`。

### 11.8 后向从脱缆后报大翻越不足 150°

正反向不能共用同一个预测点。当前已经有两套点，并且两个 pass 都必须沿同一障碍的方向 profile 选择点。若第一遍/第二遍中只有一只臂失败，检查该 pass 是否仍使用本障碍的正确 profile，不要按左右臂单独硬编码方向。

### 11.9 RemoteControl 完成后夹爪/second 完全不能动

曾出现实际四个腿关节反馈整体冻结，而 Adapter 连续收发轨迹命令。这类现象更像下位机/机构未响应、机械阻塞或夹爪没真正张开，不是自动把 second 写反。应对比四关节是否一起冻结。

### 11.10 回夹最后一遍总重试

原因是 `grip_confirmed` 必须在 Regrip 阶段出现新的低到高沿，且旧确认不会直接放行。当前把 Regrip 独立超时调为 20 秒，并保留只针对回夹的重开重试。不要把脱缆或 Planner 也纳入自动重试。

### 11.11 first_joint 自动归零已撤销

曾实现回夹后两个 first 缓慢归零，但实机效果不好，用户要求只删除这部分，其他改动保留。当前状态机在 RemoteControl 完成后直接进入 Regrip，没有归零阶段。

### 11.12 Adapter 启动确认/重试机制已撤销

曾实现启动小幅确认、最多 5 次、每次等待 1 秒和轨迹重锚。实测加入后一次翻越都没成功，已用 Git 精确恢复 Planner Adapter 六个文件并删除测试文件。当前没有该机制。不要从聊天、NUC旧文件或临时工作树恢复它。

### 11.13 Gazebo 和 RL 边界

- 当前越障真机流程不用 Gazebo，相关启动入口已从 B29 流程移除。
- 本分支不应主动修改同事 RL 工作；合并时曾明确只保留巡航/通信等同事负责内容和当前 MoveIt/ROS control 逻辑。
- 对比冲突时，RL 无关改动应以 `dev/controllerfix` 为准，本分支只保留有明确实机依据的越障差异。

## 12. 绝对不能擅自修改

1. 不要改变 `crossing_side` 的物理含义和两侧 profile 映射。
2. 不要恢复“固定右臂/左臂永远是谁锁定”的策略；锁定角色由行进方向和前后臂决定。
3. 不要改回编码器累计值决定方向；使用最近非零巡航请求。
4. 不要让 Stop 清空方向，也不要为方向增加超时。
5. 不要把两套预测点合并回一套。
6. 不要写死锁定 second 为 `current+pi`；必须实时 IK 后选等价分支。
7. 不要改变两遍锁定 second 均逆时针约 180°的方向。
8. 不要改变 `Right` 侧遍自由 `right_second=-pi` 和 `Left` 侧遍自由 `left_second=live-pi`；动态方向下它们不一定分别是时间上的第一遍、第二遍。
9. 不要取消 150°大翻越、终点静止和方向审计。
10. 不要在脱缆阶段给锁定 second 增加旧偏移。
11. 不要恢复旧 `plan_id` 人工确认链；当前自动链是规划后直接受控执行。
12. 不要恢复回夹后 first 自动归零。
13. 不要恢复 Adapter 启动探测、5 次重试或轨迹重锚。
14. 不要给脱缆或 MoveIt 失败增加自动重发；失败后保持并读取黑匣子。
15. 不要为了消除报错盲目放大 path/goal 公差或 SMC delta 上限。
16. 不要让感知 marker 直接进入正式状态机；当前 GP11 perception 尚未接入自动越障。
17. 不要让 systemd 开机自动发布动作触发。

## 13. 构建、测试和部署

### 13.1 主机构建

```bash
cd /home/unity/B29_ws_0725
source /opt/ros/noetic/setup.bash
catkin build b29_control b29_smc_auto_controller b29_planner_adapter gp11
```

如果 NUC 缺 `rqt_b29_smc_console` 依赖且不需要 GUI，可永久跳过：

```bash
touch /home/yxj02/B29_ws/src/b29_tools/rqt_b29_smc_console/CATKIN_IGNORE
```

删除该文件即可恢复编译。不要用全局配置误跳过核心包。

### 13.2 离线检查

```bash
python3 -m py_compile \
  b29_sim_utils/gp11/scripts/*.py \
  b29_sim_utils/gp11/src/gp11/*.py

catkin build b29_smc_auto_controller --no-deps
catkin build b29_planner_adapter --no-deps
catkin build gp11 --no-deps
```

历史已通过过 SMC 和 GP11 单元测试，但本文整理不重新声称最新测试数量；代码改变后应重新运行并记录实际结果。

### 13.3 部署新 NUC 的纪律

1. 先 `ssh yxj02@192.168.3.121`。
2. 在 `/home/yxj02/B29_ws/src` 检查 Git 根、分支、HEAD、工作区。
3. 当前 NUC 应使用独立的 `dual-role-flip--20260808` 工作分支，不要建立在旧 NUC 混杂历史上。
4. 优先用 Git 拉取当前远端分支；若必须 rsync，先 dry-run，并明确是否排除 `.git`、build、devel、logs。
5. 不要从 NUC 把未知旧代码同步回主机覆盖正式分支。
6. 编译后是否重启服务由用户明确决定；重启会等待 10 秒。

## 14. 分支与提交历史

当前主线关系：

```text
origin/dev/controllerfix @ 890c567
  -> 32c21a0 Integrate validated dual-role flip workflow on controllerfix baseline
  -> 3aa6d06 Use last nonzero cruise direction for crossing
  -> bfdacdf Merge dev/controllerfix documentation updates
  -> 181e66d Tune reverse crossing and regrip timing
```

历史旧分支：

- `dual-role-flip--20260727` / `origin/dual-role-flip--20260727`：早期连续翻越分支，不能作为当前基线。
- `origin/controllers`：更早控制器历史。

当前分支相对 `origin/dev/controllerfix` 的差异集中在：

- `b29_control` 的越障参数和主启动组合。
- `b29_smc_auto_controller` 的首侧选择、双遍/连续障碍和回夹时序。
- `b29_planner_adapter` 的实机轨迹安全与功能完成。
- `gp11` 的双锚 IK、方向约束、两套目标和自动循环。
- 文档、测试和构建注册。

## 15. 重要文件索引

| 文件 | 作用 |
|---|---|
| `B29_AUTOMATION_HANDOFF.md` | 本总交接，下一会话先读 |
| `README.md` | 工程概览和完整通信帧 |
| `b29_control/docs/b29_smc_obstacle_crossing_debug_validation.md` | normal/debug 边界、只读观察和人工恢复验证 |
| `b29_control/launch/start.launch` | 正式真机总入口、两套目标、连续翻越参数 |
| `b29_control/config/controller.yaml` | SMC、脱缆、RemoteControl 参数 |
| `b29_control/config/hardware.yaml` | 串口和电机映射 |
| `b29_controllers/.../OBSTACLE_CROSSING_WORKFLOW.md` | 越障状态机细节 |
| `b29_controllers/.../README.md` | SMC 控制器组件、接口和限制 |
| `b29_controllers/.../crossing_side_profile.h` | 左右侧物理映射真值 |
| `b29_controllers/.../obstacle_crossing_runtime.cpp` | 双遍、回夹、连续障碍底层流程 |
| `b29_planner_adapter/README.md` | 当前 Adapter 数据链和安全边界 |
| `b29_planner_adapter/docs/adapter_vs_smc_direct_action.md` | 架构取舍报告，仅供未来重构评估，尚未实施 |
| `b29_planner_adapter/config/planner_adapter.yaml` | Adapter 公差、delta、超时 |
| `b29_planner_adapter/src/planner_adapter.cpp` | 轨迹下发、反馈监控、功能完成 |
| `b29_sim_utils/gp11/README.md` | GP11 双锚 MoveIt 使用说明 |
| `b29_sim_utils/gp11/config/gp11_moveit_kinematics_real.yaml` | position-only IK |
| `b29_sim_utils/gp11/launch/gp11_moveit_real.launch` | 双锚 MoveIt 与方向参数 |
| `b29_sim_utils/gp11/scripts/gp11_automatic_flip.py` | 多障碍上层外循环 |
| `b29_sim_utils/gp11/src/gp11/single_flip_client.py` | 两套目标、状态等待、每遍调度 |
| `b29_sim_utils/gp11/src/gp11/cartesian_goal_core.py` | IK 种子、连续关节分支和方向审计 |
| `b29_sim_utils/gp11/scripts/gp11_real_motion_blackbox.py` | MoveIt/Adapter/编码器黑匣子 |
| `b29_controllers/.../scripts/disconnect_blackbox.py` | SMC/脱缆黑匣子 |
| `b29_sim_utils/gp11_perception/README.md` | 相机线缆感知实验；当前未接入正式状态机 |
| `b29_tools/rqt_b29_smc_console/README.md` | rqt 观察/调试工具；不能伪造正式 RemoteControl |
| `robot_control/rm_*/README.md` | 上游遗留包简要边界，不是 B29 越障流程依据 |

## 16. 当前已知未完成项

- `temporary_allow_start_without_grip_confirmed=true` 仍是临时旁路，只绕过人工 `start_disconnect` 的入口夹紧门槛。下位机夹爪确认协议稳定后应评估关闭，但不要在没有实机验证时直接改。
- 单侧夹爪没有独立“已张开”反馈，当前用 15 秒固定等待；这也是偶发未及时脱缆/机械阻塞排查的重点。
- 脱缆 Step7 无超时，速度不稳定会一直等待。
- GP11 perception 尚未进入正式越障状态机。
- 当前 Adapter 仍是独立进程逐点 ACK 方案；`adapter_vs_smc_direct_action.md` 只是架构比较，不是已实施方案。
- 本次无法在线确认新 NUC 的当前 HEAD、工作区和 systemd 现状；下一次连接后先只读核对。

## 17. 下一部分开发的建议起点

开始新功能前，先回答三个问题：

1. 新功能属于巡航/下位机协议、SMC 越障、MoveIt/IK、Adapter，还是纯观测？
2. 它是否会改变上面列出的实机成功不变量？若会，必须先明确提出影响和回退方案。
3. 能否先用单元测试、黑匣子或只读 ROS 数据验证，而不直接驱动真机？

对现有翻越问题，默认处理顺序始终是：当前代码与分支确认 -> 最新两套黑匣子 -> 精确定位一层 -> 最小修改 -> 包级测试/编译 -> 用户授权后部署与实机复测。
