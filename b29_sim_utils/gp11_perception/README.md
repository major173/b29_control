# GP11 Perception

这是一个独立的相机感知包，用一台深度相机估计细长目标的 3D 方向和抓取点。

它当前不参与 B29 自动越障闭环，也不会被 `b29_control/launch/start.launch` 启动。第一次交接自动越障时可以先跳过本包。

## 1. 两种检测方式

| 模式 | 适用情况 | 是否需要背景图 |
|---|---|---|
| `depth_line` | 固定背景、单根刚体杆、快速验证 | 建议使用 |
| `d435i_cable_tracking` | D435i 实时 RGB-D 细长目标跟踪 | 不需要 |

当前推荐的 D435i 主节点是 C++ 节点 `gp11_d435i_cable_tracker_node`。

## 2. D435i 快速启动

联合启动相机、跟踪器和 RViz：

```bash
roslaunch gp11_perception gp11_d435i_cable_tracking.launch \
  launch_realsense:=true \
  launch_rviz:=true
```

主要输出：

```text
/gp11_perception/cable_marker
/gp11_perception/grasp_point
/gp11_perception/debug_tracking
/gp11_perception/status
```

如果 RealSense 已经单独启动：

```bash
roslaunch gp11_perception gp11_d435i_cable_tracking.launch \
  launch_realsense:=false \
  launch_rviz:=true
```

相机必须提供同步的 color、`aligned_depth_to_color` 和 color camera info。默认未开启深度对齐的 `demo_pointcloud.launch` 不能直接满足该节点。

## 3. 背景减法模式

先在目标不在视野中时保存背景深度：

```bash
rosrun gp11_perception gp11_capture_depth_background.py \
  _depth_topic:=/camera/depth/image_rect_raw \
  _output_path:=/tmp/gp11_depth_background.npy
```

再启动检测：

```bash
roslaunch gp11_perception gp11_single_camera_depth_line.launch \
  depth_topic:=/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/depth/camera_info \
  background_depth_path:=/tmp/gp11_depth_background.npy \
  output_frame:=world
```

输出：

```text
/gp11_perception/target_point
/gp11_perception/line_marker
/gp11_perception/debug_mask
```

确认输出稳定后，才考虑设置 `publish_target_to_moveit:=true`。不要在感知结果尚未验证时连接真实执行链。

## 4. 主要节点

| 节点 | 作用 |
|---|---|
| `gp11_d435i_cable_tracker_node` | D435i RGB-D 实时跟踪、时序平滑和抓取点输出 |
| `gp11_capture_depth_background.py` | 保存一帧空场景深度图 |
| `gp11_single_camera_depth_line.py` | 背景减法、3D 回投和轴线拟合 |

## 5. 常用参数

### 背景减法

| 参数 | 含义 | 建议起点 |
|---|---|---|
| `foreground_depth_delta_m` | 前景比背景至少近多少米 | `0.01 ~ 0.03` |
| `min_component_area_px` | 最小连通区域 | `150` |
| `smoothing_alpha` | 当前帧权重，越小越平滑 | `0.35` |
| `target_ratio` | 目标点在线段上的比例 | `0.5` |
| `pixel_roi` | 只检测固定图像区域 | 现场标定 |

### D435i

- `align_depth:=true`：跟踪器需要对齐到彩色图的深度。
- `enable_color:=true`：实时跟踪需要彩色图。
- `enable_gyro:=false`、`enable_accel:=false`：当前算法不使用 IMU。
- 推荐先使用 `640x480 @ 30 Hz`。
- `enable_confidence:=false`：D435i 不提供该传感器时关闭提示更清晰。

## 6. 当前限制

- 单相机有遮挡时，3D 估计会明显变差。
- 输出是可见主线段，不是整根柔性线缆的完整形状。
- 背景减法要求相机和背景基本固定。
- 目标与背景深度太接近时容易丢失。
- D435i 跟踪更适合主方向明显的细长目标，不是完整柔性线缆重建方案。
- 感知输出目前没有进入正式越障状态机，不能把 marker 出现理解为机器人会自动执行。

## 7. 接入 MoveIt 前的检查

1. RViz 中线段方向稳定。
2. 抓取点位于真实目标上。
3. `output_frame` 的 TF 正确且没有跳变。
4. 相机遮挡或目标丢失时不会继续输出危险旧目标。
5. 在不执行机构的环境中验证 MoveIt 目标转换。
6. 由负责人评审后，再接入 B29 真机执行链。
