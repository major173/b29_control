# rm_msgs（上游通用消息包）

该包集中定义 RM 上游通用 ROS message、service 和 action，供 `robot_control` 下的遗留组件使用。它不是 B29 上下位机二进制串口协议文档。

接口清单应直接以以下源码为准：

- `msg/`：执行器、底盘、云台、DBUS、裁判系统和其他通用消息
- `srv/`：状态切换、速度限制、相机和 IMU 控制服务
- `action/Engineer.action`
- `CMakeLists.txt`：实际注册并生成的接口完整列表

构建：

```bash
catkin build rm_msgs
```

注意：

- B29 52 字节控制帧和 173 字节反馈帧见仓库根 `README.md`。
- B29 越障所用 RemoteControl 增量、完成位、巡航方向、自动触发和夹爪确认来自 `b29_control` 硬件接口，不要根据本包消息名猜串口字段。
- 新增或修改消息后必须同步检查依赖包并重新构建；不要为越障局部需求无端改动这个共享包。

B29 当前总流程见仓库根目录 `B29_AUTOMATION_HANDOFF.md`。
