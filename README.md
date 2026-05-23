# B29

> B29机器人控制系统

---

## RL Reach Sim2Sim 启动流程

---

### 环境准备（首次配置）

**1. 创建 conda 环境**

```bash
conda create -n b29 python=3.8 -y
conda activate b29
```

**2. 安装推理依赖**

```bash
# 始终用 python3 -m pip，避免 PYTHONPATH 污染导致系统 pip 被调用
python3 -m pip install numpy==1.23.5
python3 -m pip install onnxruntime==1.19.2
python3 -m pip install pyyaml
python3 -m pip install rospkg catkin-pkg   # rospy 依赖，pip 安装替代系统路径
```

> **注意**：不要将 `/usr/lib/python3/dist-packages` 加入 `PYTHONPATH`，会导致 conda 工具链崩溃。

**3. 配置 ROS 激活脚本**

```bash
CONDA_ENV_PATH=$(PYTHONPATH="" conda info --envs | grep "^b29 " | awk '{print $NF}')
mkdir -p "$CONDA_ENV_PATH/etc/conda/activate.d"
cat > "$CONDA_ENV_PATH/etc/conda/activate.d/ros_setup.sh" << 'EOF'
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages
source /opt/ros/noetic/setup.bash
for _ws in "$HOME/usetest/B29" "$HOME/catkin_ws"; do
    if [ -f "$_ws/devel/setup.bash" ]; then
        source "$_ws/devel/setup.bash"
        break
    fi
done
unset _ws
EOF
conda deactivate && conda activate b29
```

**4. 验证环境**

```bash
python3 -c "import numpy, onnxruntime, yaml, rospkg; print('deps ok')"
python3 -c "import rospy; from sensor_msgs.msg import JointState; print('ros ok')"
```

---

### Gazebo 端到端闭环启动

**终端 1：Gazebo + 控制器 + TF**

```bash
cd ~/usetest/B29
source devel/setup.bash
roslaunch b29_control reach_gazebo_stage1.launch
```

**终端 2：RL bridge（关节目标 → position controller）**

```bash
cd ~/usetest/B29
source devel/setup.bash
conda activate b29
rosrun b29_control gazebo_rl_bridge_node.py
```

**终端 3：RL 推理节点**

```bash
cd ~/usetest/B29
conda activate b29
python3 src/b29_control/b29_control/scripts/rl_inference_node.py
```

启动后日志应显示：
```
[rl_inference] anchor_side=left active_dof=[left_first_leg_joint, ...]
[rl_inference] onnx=.../models/reach/stage0.onnx
[rl_inference] waiting for target_point...
```

**终端 4：目标点键盘控制**

```bash
cd ~/usetest/B29
conda activate b29
python3 src/b29_control/b29_control/scripts/reach_goal_keyboard_node.py --anchor_side left
```

启动后推理节点打印 `target_point received`，之后每 5 秒输出一次推理状态。

键盘操作：

| 按键 | 动作 |
|------|------|
| W/S | 目标点 X +/- |
| A/D | 目标点 Y +/- |
| Q/E | 目标点 Z +/- |
| R | 重置目标点到当前末端位置 |
| [ / ] | 步长 -/+ |
| Ctrl+C | 退出 |

**验证闭环**

```bash
# 推理输出频率（应稳定 50Hz）
rostopic hz /gp11/rl/joint_targets

# 查看关节目标
rostopic echo /gp11/rl/joint_targets -n 3

# 查看观测向量（32D，obs[30:32] 应为 [0,1] 表示 left anchor）
rostopic echo /gp11/rl/observation -n 1

# 查看原始 action（4D，值在 [-1,1]）
rostopic echo /gp11/rl/action_raw -n 1
```

---

### RViz 可视化

```bash
# 随 launch 启动
roslaunch b29_control reach_gazebo_stage1.launch rviz:=true
# 或单独启动
rviz
```

| 设置项 | 值 |
|--------|-----|
| Fixed Frame | `world` |
| RobotModel | 添加 |
| TF | 添加 |
| Marker (红球) | `/gp11/rl/goal_marker`（目标点，`capture_output_ref` 坐标系） |
| Marker (绿球) | `/gp11/rl/tool_marker`（运动端末端） |

**关键 TF 帧**

| TF 帧 | 含义 |
|-------|------|
| `world` | 世界固定系，以 `left_second_leg` 为原点 |
| `left_second_leg` | 固定端，在 `world` 下静止不动 |
| `base_link` | 机体，随关节运动漂移 |
| `capture_output_ref` | capture 网络输出参考坐标系 |
| `r_gripper_left_uprod` | 运动端末端（绿球跟踪） |

---

### 实机启动

**终端 1：硬件接口**

```bash
cd ~/usetest/B29 && source devel/setup.bash
roslaunch b29_control start.launch
```

**终端 2：TF 发布**

```bash
conda activate b29
rosrun b29_control anchor_world_tf_publisher.py \
  _anchor_link:=left_second_leg _anchor_side:=left _rate:=50.0
```

**终端 3：RL 推理节点**

```bash
conda activate b29
python3 src/b29_control/b29_control/scripts/rl_inference_node.py
```

**终端 4：目标点键盘控制**

```bash
conda activate b29
python3 src/b29_control/b29_control/scripts/reach_goal_keyboard_node.py \
  --anchor_side left --world_frame base_link
```

> 实机无 `world` frame，RViz Fixed Frame 设为 `left_second_leg`。

---

### 节点与话题速查

| 节点 | 订阅 | 发布 |
|------|------|------|
| `rl_inference_node` | `/joint_states`，`/gp11/rl/target_point_local` | `/gp11/rl/joint_targets`，`/gp11/rl/observation`，`/gp11/rl/action_raw` |
| `gazebo_rl_bridge_node` | `/gp11/rl/joint_targets` | `*_position_controller/command` ×4 |
| `reach_goal_keyboard_node` | `/tf` | `/gp11/rl/target_point_local`，`/gp11/rl/goal_marker`，`/gp11/rl/tool_marker` |
| `anchor_world_tf_publisher` | `/tf`（TF buffer） | `/tf`（`world→base_link`，`world→capture_output_ref`） |
| `gripper_passive_joint_relay` | `/joint_states` | `/joint_states`（从动夹爪，仅实机） |

---



### 硬件接口测试

> 检查串口输出->启动hardware->观察电机数据

- 串口检查

```bash
# 列出可用串口
ls /sys/class/tty/ttyUSB* -l
# 打开串口配置工具,需要先安装minicom或cutecom
sudo minicom -s  # 无可视化界面
cutecom			 # 可视化界面

# minicom部分
# 接着选择serial port set up（第三项），回车，按a键，命令口输入ttyUSB1，exit
# 完成，接着观察是否有接受乱码输出。

# cutecom部分
# 选择端口open,点击hex输出

# 确认发来的数据是否符合数据帧
```

- 启动hardware

```bash
# 启动硬件接口
mon launch b29_control start.launch

# 打开rqt
rqt
```

进入rqt,在`controller manager`启动`joint_state_controller`

- 观察电机数据

```bash
# 启动plotjugger
rosrun plotjuggler plotjuggler
```

订阅`/joint_state`话题

## 通信协议

[TOC]

### 数据包与校验位

### 控制帧结构

**流向**：上位机 (PC) →→ 下位机 (Robot)
**作用**：告诉机器人各个部件应该怎么动（设定期望值）。

**A. 数据包结构 (51 Bytes)**

| 顺序         | 字段名         | 长度           | 值/类型        | 说明               |
|------------|-------------|--------------|-------------|------------------|
| 1          | 帧头1         | 1 Byte       | `0x55`      | 固定头              |
| 2          | 帧头2         | 1 Byte       | `0xAA`      | 固定头              |
| 3          | 命令字         | 1 Byte       | `0x01`      | 控制指令             |
| 4          | 长度位         | 1 Byte       | `0x2C` (44) | 后续数据净荷长度         |
| **5 ~ 48** | **Payload** | **44 Bytes** | **Mixed**   | **核心控制数据 (见下表)** |
| 49         | CRC         | 1 Byte       | Calc        | CRC8 校验码         |
| 50         | 帧尾1         | 1 Byte       | `0x0D`      | CR               |
| 51         | 帧尾2         | 1 Byte       | `0x0A`      | LF               |

**B. Payload 数据内容 (44 Bytes) （注：float数据按小端序发送）**

驱动轮的速度期望 *2、夹爪速度期望 *2、夹爪位置期望 *2、关节速度 *1，关节角度期望 *4。

这 44 个字节是纯二进制数据，由 11 个 `float` (单精度浮点数，小端模式) 组成，顺序如下：

1. **左驱动轮速度** (`float`, 4B) - `wheelSpeedTarget[0]`
2. **右驱动轮速度** (`float`, 4B) - `wheelSpeedTarget[1]`
3. **左夹爪速度** (`float`, 4B) - `clawSpeedTarget[0]`
4. **右夹爪速度** (`float`, 4B) - `clawSpeedTarget[1]`
5. **左夹爪位置** (`float`, 4B) - `clawAngleTarget[0]`
6. **右夹爪位置** (`float`, 4B) - `clawAngleTarget[1]`
7. **关节统一速度** (`float`, 4B) - `jointSpeedTarget` (所有关节共用此限速或目标速度)
8. **左1关节 角度** (`float`, 4B) - `jointAngleTarget[0]`
9. **左2关节 角度** (`float`, 4B) - `jointAngleTarget[1]`
10. **右1关节 角度** (`float`, 4B) - `jointAngleTarget[2]`
11. **右2关节 角度** (`float`, 4B) - `jointAngleTarget[3]`

```c
float wheelSpeedTarget[2];
float clawSpeedTarget[2];
float clawAngleTarget[2];
float jointSpeedTarget;
float jointAngleTarget[4];
```

**C. 如何解析与使用**

当下位机收到这串数据后：

1. **校验**：计算前 48 字节的 CRC8 是否等于第 49 字节。
2. **映射**：将字节流按每 4 个字节强转为 `float`。
3. 应用：
    - 将 **轮子速度** 赋值给底盘运动学解算模块。
    - 将 **关节角度** 发送给机械臂驱动模块（如位置模式控制）。
    - 将 **夹爪位置/速度** 发送给夹爪电机。

### 反馈帧结构

**流向**：下位机 (Robot) →→ 上位机 (PC)
**作用**：告诉上位机现在各个电机的实际状态（位置、速度、力矩）。

**A. 数据包结构 (154 Bytes )**

电机数量 `num = 8`。长度 = `7 + (8 \* 13) + 2 * 3 * 4 + 4 * 4 + 2` = 153字节。

| **顺序**        | **字段名**     | **长度**        | **值/类型**         | **说明**                                        |
|---------------|-------------|---------------|------------------|-----------------------------------------------|
| **1**         | **帧头1**     | **1 Byte**    | **`0x55`**       | **固定头**                                       |
| **2**         | **帧头2**     | **1 Byte**    | **`0xAA`**       | **固定头**                                       |
| **3**         | **命令字**     | **1 Byte**    | **`0x01`**       | **反馈指令**                                      |
| **4**         | **长度位**     | **1 Byte**    | **`0x92` (146)** | **后续数据净荷长度 (8 * 13 + 2 * 3 * 4 + 4 * 4 + 2)** |
| **5 ~ 108**   | **Payload** | **104 Bytes** | **Mixed**        | **所有电机的状态列表**                                 |
| **109 ~ 148** | **Payload** | **40  Bytes** | **Mixed**        | **IMU数据**                                     |
| **149**       | **Payload** | **1 Bytes**   | **Mixed**        | **电机异常状态位**                                   |
| **150**       | **Payload** | **1 Bytes**   | **Mixed**        | **夹爪初始化标志位**                                  |
| 151           | CRC         | 1 Byte        | Calc             | CRC8 校验码                                      |
| 152           | 帧尾1         | 1 Byte        | `0x0D`           | CR                                            |
| 153           | 帧尾2         | 1 Byte        | `0x0A`           | LF                                            |

**B. Payload 数据内容 (8个电机 IMU数据)**

数据是一个**列表**，每个电机占用 **13 个字节**，依次排列 (Motor1 → Motor8)。
**单个电机的数据块结构 (13 Bytes)**：**(注： float数据按小端序发送)**

1. **ID** (`uint8`, 1B) - 电机物理ID (如 0x01)

2. **位置 Pos** (`float`, 4B) - 当前角度 (rad)

3. **速度 Vel** (`float`, 4B) - 当前角速度 (rad/s)

4. **力矩 Tor** (`float`, 4B) - 当前输出力矩 (N·m)

**注：POS，VEL和T数据为经过处理，减速后的数据**

**排列顺序**：

Block 1 (Byte 0-12): **电机 1** 的 ID, Pos, Vel, Tor

Block 2 (Byte 13-25): **电机 2** 的 ID, Pos, Vel, Tor

...

Block 8 (Byte 91-103): **电机 8** 的 ID, Pos, Vel, Tor

**IMU 数据（加速度 角速度 四元数）**

- X 加速度
- Y 加速度
- Z 加速度
- X 角速度
- Y 角速度
- Z 角速度
- 四元数 W
- 四元数 X
- 四元数 Y
- 四元数 Z

**电机异常状态位**

- 以 `一个字节` 表示 `8个电机` 异常状态, `1` 表示正常  `0` 表示异常
- 因此上位机接收此字节为 `0xFF` 时表示所有电机均 `正常`，否则有电机处于 `异常状态`

| 字位      | 对应电机         |
|---------|--------------|
| `第 0 位` | `左pitch关节电机` |
| `第 1 位` | `左yaw关节电机`   |
| `第 2 位` | `右pitch关节电机` |
| `第 3 位` | `右yaw关节电机`   |
| `第 4 位` | `左驱动轮电机`     |
| `第 5 位` | `右驱动轮电机`     |
| `第 6 位` | `左夹爪电机`      |
| `第 7 位` | `右夹爪电机`      |

**夹爪初始化状态位**

- 此状态位用以判断夹爪是否初始化完成， `0` 表示未完成  `1` 表示完成

**C. 上位机如何解析**

1. **寻找帧头：在串口流中寻找 `55 AA`。**
2. **读取长度：读取 Length 字节 (0x92 = 146)。**
3. **读取 Payload：读取接下来的 146 字节。**

### **校验相关代码**

**下位机相关代码：**

~~~c
//rx_buffer 中存放一个完整的数据帧
uint8_t crc_rx = rx_buffer[4 + len];  //提取控制帧的crc校验位
//此处计算控制帧的crc，用于校对
uint8_t crc_calc = Get_CRC8_Check_Sum(rx_buffer, (uint16_t)len + 4u, 0xFF);
if (crc_calc != crc_rx) {  
	memmove(rx_buffer, rx_buffer + 1, rx_len - 1);
	rx_len -= 1;
	continue;
}

-----------------------------------------------------------------

//填充反馈帧的数据
//tx_buff中包含一个完整的反馈帧
void slave_send_packet(void){
    const uint8_t motor_count = (uint8_t)num;
    const uint8_t motor_payload_len = (uint8_t)(motor_count * 13u);
    const uint8_t imu_payload_len = 4u * 4u;
    const uint8_t payload_len = (uint8_t)(motor_payload_len + imu_payload_len);
    const imuDataStruct_t *imu = get_imu_data();
    
    static uint8_t tx_buf[256];
    
    uint16_t index = 0;
    // Clear buffer to ensure no garbage data
    memset(tx_buf, 0, 256);
    
    tx_buf[index++] = UART_HEADER1;
    tx_buf[index++] = UART_HEADER2;
    tx_buf[index++] = 0x01;
    tx_buf[index++] = payload_len;
    /*电机部分*/
    for (uint8_t i = 0; i < motor_count; i++) {
        formatTrans32Struct_t motor_data;
        const motor_t *m = &motor[i];
        // id
        tx_buf[index++] = (uint8_t)(m->id & 0xFF);
        // pos
        if (i == Motor3 || i == Motor7) {
            motor_data.f_temp = motor_to_mechanism_rad(i, m->pos_track.theta_total_rad);
        }
        else if(i == Motor4 || i == Motor8)
        {
            motor_data.f_temp = motor_to_mechanism_position(i, m->pos_track.theta_total_rad); 
        }
        else 
        {
            motor_data.f_temp = m->para.pos;
        }
        
        memcpy(&tx_buf[index], motor_data.u8_temp, 4); index += 4;
        // vel
        motor_data.f_temp = m->para.vel;
        memcpy(&tx_buf[index], motor_data.u8_temp, 4); index += 4;
        // tor

        motor_data.f_temp = m->para.tor;
        memcpy(&tx_buf[index], motor_data.u8_temp, 4); index += 4; 
    }
    
    for (uint8_t i = 0; i < 4u; i++)
    {
        memcpy(&tx_buf[index], imu->quat[i].u8_temp, 4u);
        index += 4u;
    }
    
    tx_buf[index++] = Get_CRC8_Check_Sum(tx_buf, (uint16_t)payload_len + 4u, 0xFF);
    tx_buf[index++] = UART_END1;
    tx_buf[index++] = UART_END2;
    
    (void)HAL_UART_Transmit_DMA(&huart1, tx_buf, index);
}


--------------------------------------------------------------------------------
//计算crc的相关代码

    
//crc8 generator polynomial:G(x)=x8+x5+x4+1
static const unsigned char CRC8_INIT = 0xff;
static const unsigned char CRC8_TAB[256] = {
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};


/**
  * @brief          Calculate the CRC8 checksum
  * @param[in]      pchMessage: message to calculate
  * @param[in]      dwLength: length of the message
  * @param[in]      ucCRC8: initial CRC8 value
  * @retval         ucCRC8
**/
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8){
	unsigned char ucIndex;
	while (dwLength--){
		ucIndex = ucCRC8^(*pchMessage++);
		ucCRC8 = CRC8_TAB[ucIndex];
	}
	return(ucCRC8);
}
~~~

