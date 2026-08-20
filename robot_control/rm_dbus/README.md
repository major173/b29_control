# rm_dbus（上游遗留包）

该包读取传统 DBUS 遥控器串口并发布 `rm_msgs/DbusData`。它来自 RM 上游代码，不在 B29 当前自动脱缆、MoveIt 翻越或下位机 RemoteControl 数据链中。

## 当前接口

- 节点/可执行文件：`rm_dbus`
- 默认串口：`/dev/usbDbus`
- 参数：`serial_port`，配置文件 `config/dbus.yaml`
- 发布话题：节点私有命名空间下的 `dbus_data`
- 发布类型：`rm_msgs/DbusData`
- 主循环：60 Hz
- 依赖：`roscpp`、`rm_common`、`rm_msgs`

构建：

```bash
catkin build rm_dbus
```

除非任务明确涉及 RM DBUS 遥控器，不要把该包接入 B29 越障状态机，也不要用它替代 B29 反馈帧中的四关节 RemoteControl 增量、完成位或巡航方向字段。

B29 当前总流程见仓库根目录 `B29_AUTOMATION_HANDOFF.md`。
