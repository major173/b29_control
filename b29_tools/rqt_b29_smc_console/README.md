# rqt_b29_smc_console

`rqt_b29_smc_console` 是 `b29_smc_auto_controller` 的调试面板插件，用于在 `rqt` 中集中完成传感器输入构造、debug override 发送、预设 workflow 执行和 trace 观察。

插件默认使用英文和代码内名称，顶部可在 `English / 中文` 之间切换显示语言。状态值、事件值、workflow 名称和字段真实 `name` 保持原始代码名，便于直接对照日志和源码。

面板默认使用英文展示，并在顶部提供 `English / 中文` 的轻量语言切换；运行时状态值、`current_state`、`last_event`、workflow 名称和真实字段名保持原样不翻译。

## 包目的

- 提供面向自动控制器的统一调试入口
- 用图形界面替代手工拼 `rostopic pub`
- 让传感器状态、override 状态和状态机 trace 保持同屏可见

## 默认 Namespace

默认连接到：

`/b29_controller/b29_smc_auto_controller`

启动后可以在面板顶部直接看到当前 namespace，必要时也可以按实际控制器实例调整。

## 面板说明

### Overview

- 显示当前 namespace
- 显示最新 `current_state`、`previous_state`、`last_event`、`output_mode`
- 显示最近一次状态切换或命令原因 `reason`

### Sensor Input

- 提供传感器输入编辑区
- 支持常用预设按钮：
  - `基础可启动`
  - `通信丢失`
  - `障碍出现`
  - `全部清空`
- `发布当前 Sensor Input` 会把当前编辑值发布到 `sensor_input`

### Override Composer

- 选择要覆盖的字段
- 选择动作类型：
  - `触发一次`：发送一次脉冲，随后自动释放
  - `持续覆盖`：保持字段处于激活状态
  - `取消覆盖`：移除该字段的覆盖
- 对布尔、枚举、浮点字段使用对应编辑控件
- 面板会实时预览当前 `field_mask`
- 英文模式下，字段下拉优先显示代码风格文本，例如 `Base / lower_alive`
- 对 `触发一次` 这类事件型字段，插件会先发送按下态，再延迟一个很短的时间窗发送释放态，避免 50Hz 控制周期完整错过脉冲
- 如果 `sensor_input` 或 `debug_override` 发布失败，插件会在界面中显示错误提示，而不是把异常直接抛到 UI 外层

### Workflow

- 提供预置场景入口
- 默认流程用于快速验证控制链路，例如：
  - `CommsLoss -> Idle`
  - `Idle -> AutoInit -> Traversing`
  - `EmergencyStop -> SafeStop`
  - `SafeStop -> Idle`

### Trace

- 以列表方式展示最近的 `state_trace`
- 便于观察当前状态、上一个状态、事件和原因
- 对 `SafeStop`、`CommsLoss` 这类关键状态会做高亮提示

## 预设按钮与 Override 语义

- `基础可启动`：写入一组可进入自动链路的基础传感器值
- `通信丢失`：将 `lower_alive` 等关键输入切到失联语义
- `障碍出现`：构造障碍检测相关输入，便于验证障碍分支
- `全部清空`：恢复为默认基线值
- `触发一次`：适合 `auto_start_requested`、`emergency_stop` 这类瞬时事件
- `持续覆盖`：适合 `posture_ready`、`lower_alive` 这类需要保持的条件
- `取消覆盖`：移除当前字段的覆盖，回到未覆盖状态

## 运行方式

建议先构建当前包，再启动插件：

```bash
catkin build rqt_b29_smc_console --no-deps
rosrun rqt_b29_smc_console rqt_b29_smc_console
```

如果在 `rqt` 中手动加载插件，也可以通过插件列表打开 `B29 SMC Console`。

## 离线模式

当缺少 ROS 运行时依赖，或当前环境未就绪时，插件会退化为最小 UI：

- 仍可显示基础界面结构
- 不会发布真实 `sensor_input`、`debug_override` 或 `workflow` 请求
- 不会订阅真实 `state_trace`
- 适合本地检查布局和文案，不适合作为联调替代
