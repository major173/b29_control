# GP11 Perception

这是一个独立于 `gp11/` 的真实相机感知包，目标是：

- 只用一台双目深度相机
- 从深度图中提取单根细长目标
- 在相机/世界坐标系下估计 3D 轴线
- 输出可直接桥接到 GP11 MoveIt 目标点的话题

当前版本优先服务于你现在的“类绳子刚体 / 水平刚体杆”测试，不直接做柔性 cable 重建。

## 当前结构

当前包里用于 D435i 实时跟踪的实现只保留一条：

- `launch/gp11_d435i_cable_tracking.launch`
  - 默认主链路
  - 使用 C++ 节点 `gp11_d435i_cable_tracker_node`
  - 这是当前唯一保留的 D435i 实时跟踪实现

## D435i 一键联调

如果你用的是 Intel RealSense D435i，这个包现在提供了一个联合 launch：

- 启动 `realsense2_camera` 驱动
- 启动 D435i 跟踪节点
- 启动 RViz，并直接显示跟踪出来的线和目标点

联合 launch 文件：

- `launch/gp11_d435i_tracking.launch`
- `launch/gp11_d435i_cable_tracking.launch`

注意：

- 现在已经按当前机器环境验证过，`gp11_d435i_cable_tracking.launch` 可以直接拉起 `realsense2_camera` 和 C++ 跟踪节点。
- `gp11_d435i_cable_tracker_node` 依赖的是 `color + aligned_depth_to_color + color/camera_info` 这一组话题。
- `realsense2_camera/demo_pointcloud.launch` 默认 `align_depth:=false`，因此它默认不适合作为这个 tracker 的外部驱动启动方式。

## 包含节点

- `gp11_d435i_cable_tracker_node`
  - C++ 主节点
  - 订阅 D435i 的 `color + aligned depth + camera_info`
  - 用单帧线段检测 + 深度回投 + 原生曲线优化 + 时序平滑做实时跟踪
  - 发布 RViz 里的跟踪线、抓取点、调试图像

- `gp11_capture_depth_background.py`
  - 抓取当前空场景的一帧深度图，保存为 `.npy`
  - 供后续前景减法使用

- `gp11_single_camera_depth_line.py`
  - 订阅一台深度相机的 `depth + camera_info`
  - 用背景减法提取前景
  - 回投 3D 点并拟合 3D 轴线
  - 发布目标点、可视化 marker、调试 mask

## 推荐模式

这个包现在有两条链路：

- `depth_line`
  - 适合固定背景、单根刚体杆、快速调试
  - 依赖可选背景参考

- `d435i_cable_tracking`
  - 适合真实 D435i 的实时 RGB-D 跟踪
  - 不依赖背景图
  - 当前是 C++ 实现
  - 推荐作为真实线缆/细长目标主模式

## 为什么单台双目深度相机也能做

你只有一台双目深度相机时，不能做真正的多视角三角化融合，但仍然可以：

1. 直接使用相机给出的深度图作为 3D 来源
2. 在受控工作空间里做背景减法
3. 拟合出单根细长目标的 3D 中心轴
4. 通过时序平滑降低抖动

这条路线适合：

- 单根目标
- 相机能完整看到主要抓取区域
- 工作空间背景基本固定
- 当前目标更像“细长刚体”而不是大幅弯曲 cable

## 典型话题

- 输入
  - `/camera/depth/image_rect_raw`
  - `/camera/depth/camera_info`

- 输出
  - `/gp11_perception/target_point`
  - `/gp11_perception/line_marker`
  - `/gp11_perception/debug_mask`
  - 可选 `/gp11_moveit/reach_arm/target_point`

## 使用步骤

### 1. 先抓空场景背景

确保相机对着工作空间，但目标杆/类绳子刚体不在视野中：

```bash
rosrun gp11_perception gp11_capture_depth_background.py \
  _depth_topic:=/camera/depth/image_rect_raw \
  _output_path:=/tmp/gp11_depth_background.npy
```

### 2. 启动单相机检测

```bash
roslaunch gp11_perception gp11_single_camera_depth_line.launch \
  depth_topic:=/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/depth/camera_info \
  background_depth_path:=/tmp/gp11_depth_background.npy \
  output_frame:=world
```

### 2A. 直接用 D435i 联合启动

如果你已经安装好了 `realsense2_camera`，推荐直接用这个：

```bash
roslaunch gp11_perception gp11_d435i_tracking.launch \
  background_depth_path:=/tmp/gp11_depth_background.npy \
  launch_realsense:=true \
  launch_rviz:=true
```

如果你要走真正的实时 RGB-D 线缆跟踪，而不是背景减法，推荐这个 launch：

```bash
roslaunch gp11_perception gp11_d435i_cable_tracking.launch \
  launch_realsense:=true \
  launch_rviz:=true
```

如果你只想复用已经单独启动好的 RealSense 驱动，推荐不要直接用 `demo_pointcloud.launch`，而是显式打开 `align_depth`：

```bash
roslaunch realsense2_camera rs_camera.launch \
  camera:=camera \
  align_depth:=true \
  enable_sync:=true \
  enable_color:=true \
  enable_gyro:=false \
  enable_accel:=false \
  enable_confidence:=false \
  depth_width:=640 depth_height:=480 depth_fps:=30 \
  color_width:=640 color_height:=480 color_fps:=30
```

然后再单独启动 tracker：

```bash
roslaunch gp11_perception gp11_d435i_cable_tracking.launch \
  launch_realsense:=false \
  launch_rviz:=true
```

如果你继续沿用：

```bash
rosmon launch realsense2_camera demo_pointcloud.launch
```

那默认不会发布 `/camera/aligned_depth_to_color/image_raw`，因此 `gp11_d435i_cable_tracker_node` 不会正常进入 RGB-D 同步跟踪。

旧的背景减法链路如果你只想复用已经单独启动好的 RealSense 驱动，可以这样：

```bash
roslaunch gp11_perception gp11_d435i_tracking.launch \
  launch_realsense:=false \
  background_depth_path:=/tmp/gp11_depth_background.npy \
  launch_rviz:=true
```

### 3. 在 RViz 里看

- `Marker`：`/gp11_perception/line_marker`
- `PointStamped`：`/gp11_perception/target_point`
- `Image`：`/gp11_perception/debug_mask`

联合 launch 默认会直接打开 RViz，并加载：

- 跟踪线：`/gp11_perception/line_marker`
- 目标点：`/gp11_perception/target_point`
- TF：相机坐标系树

如果走新的 D435i 实时跟踪模式，RViz 重点看：

- 跟踪线：`/gp11_perception/cable_marker`
- 抓取点：`/gp11_perception/grasp_point`
- 调试图像：`/gp11_perception/debug_tracking`
- 状态文本：`/gp11_perception/status`

### 4. 确认稳定后再接到 MoveIt

```bash
roslaunch gp11_perception gp11_single_camera_depth_line.launch \
  depth_topic:=/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/depth/camera_info \
  background_depth_path:=/tmp/gp11_depth_background.npy \
  output_frame:=world \
  publish_target_to_moveit:=true
```

## 关键限制

- 只有单视角时，被夹爪遮挡后，估计会明显变差
- 当前输出的是“可见主线段/主轴线”，不是整根柔性线缆的完整 3D 形状
- 若背景不稳定，背景减法效果会下降
- 若目标和背景深度太接近，需要调 `foreground_depth_delta_m`
- `d435i_cable_tracking` 当前第一版更适合“主导方向明显的细长目标”，对大幅弯曲线缆还不是最终方案

## 参数建议

- `foreground_depth_delta_m`
  - 前景必须比背景更近多少米才算前景
  - 经验起点：`0.01 ~ 0.03`

- `min_component_area_px`
  - 太小会引入噪声
  - 经验起点：`150`

- `smoothing_alpha`
  - 越大越相信当前帧，越小越平滑
  - 经验起点：`0.35`

- `target_ratio`
  - 目标点位于拟合线段上的比例
  - `0.5` 是中点

- `pixel_roi`
  - 若工作空间固定，建议限制 ROI
  - 能明显减少误检

## D435i 相关参数建议

- `launch_realsense:=true`
  - 联合启动 RealSense 驱动

- `enable_color:=true`
  - 新的 `d435i_cable_tracking` 依赖 RGB 做单帧检测

- `enable_gyro:=false enable_accel:=false`
  - 当前这条感知链路不依赖 IMU
  - 在当前机器上，这样启动更接近 `demo_pointcloud.launch` 的稳定配置

- `align_depth:=true`
  - 因为新的跟踪节点默认使用 `aligned_depth_to_color`

- `depth_width:=640 depth_height:=480 color_width:=640 color_height:=480`
  - 当前已验证可用的一组分辨率

- `enable_confidence:=false`
  - D435i 本机驱动日志里会提示 confidence sensor 不支持，直接关闭更干净

## 当前产出

如果你用 `gp11_d435i_cable_tracking.launch`，包的核心产出是：

- RViz 中的实时跟踪线：`/gp11_perception/cable_marker`
- 当前抓取点：`/gp11_perception/grasp_point`
- 调试图像：`/gp11_perception/debug_tracking`
- 可选 MoveIt 目标点：`/gp11_moveit/reach_arm/target_point`
