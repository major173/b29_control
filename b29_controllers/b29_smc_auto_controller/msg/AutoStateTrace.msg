# 越障策略枚举：不越障、线夹、阻尼器。
uint8 CROSSING_NONE=0
uint8 CROSSING_LINE_CLAMP=1
uint8 CROSSING_DAMPER=2

# 消息时间戳和 frame 信息。
std_msgs/Header header

# RobotFSM 外层状态机状态和最近一次状态转换信息。
string current_state
string previous_state
string last_event
string transition_reason

# 当前最终命令原因、基础命令原因和输出模式。
string command_reason
string base_command_reason
string output_mode

# 最终 effective command 的安全控制标志。
bool stop_all
bool freeze_joints

# 最终 effective command 的驱动轮命令。
string drive_mode
float64 left_wheel_speed
float64 right_wheel_speed

# 最终 effective command 的六个位置关节目标。
# 顺序：left_first_leg, left_second_leg, left_gripper,
#      right_first_leg, right_second_leg, right_gripper。
float64[6] joint_targets
float64 left_gripper_target
float64 right_gripper_target
uint8 crossing_strategy

# CommandDispatcher 是否尝试写入，以及写入句柄是否成功。
# 该字段不代表下位机已经物理响应。
bool command_dispatch_attempted
bool command_dispatch_succeeded

# 越障主 FSM 当前状态、侧别、首个越障侧和阶段计时。
bool obstacle_crossing_active
string obstacle_crossing_stage
string obstacle_crossing_side
string first_crossing_side
time obstacle_crossing_stage_enter_time
float64 obstacle_crossing_stage_elapsed_sec
string obstacle_crossing_transition_reason

# 脱缆子流程当前步骤、步骤计时和转移说明。
string disconnect_step
time disconnect_step_enter_time
float64 disconnect_step_elapsed_sec
string disconnect_step_transition_reason
string disconnect_step_expected_condition

# Step7 脱缆判定相关数据。
float64 disconnect_check_displacement
float64 to_check_joint_pos
float64 succeed_disconnect_cable_threshold
string current_side_gripper
float64 current_side_gripper_target

# 人工干预状态和导致人工干预的失败动作。
bool manual_intervention_active
string manual_intervention_reason
string failed_action

# 当前失败动作的 retry 计数，以及各动作 retry 计数和上限。
uint32 retry_count
uint32 close_grippers_retry_count
uint32 left_disconnect_retry_count
uint32 right_disconnect_retry_count
uint32 left_regrip_retry_count
uint32 right_regrip_retry_count
uint32 close_gripper_retry_limit
uint32 disconnect_cable_retry_limit
uint32 regrip_retry_limit

# 等待 grip_confirmed 的状态、耗时和超时标志。
bool waiting_for_grip_confirmed
float64 grip_wait_elapsed_sec
bool grip_wait_timed_out
string last_failure_reason

# PlannerControl 阶段状态、手动放行和 planner 点是否可用。
bool planner_control_active
bool planner_manual_release_enabled
bool planner_release_received
float64 planner_control_wait_elapsed_sec
bool planner_point_available
bool planner_point_fresh
bool planner_override_applied

# PlannerControl 之后的人工遥控阶段输入、实际应用增量和累计目标。
bool remote_control_active
float64[4] remote_control_raw_increments
float64[4] remote_control_applied_increments
float64[4] remote_control_joint_targets
bool remote_control_input_valid
uint64 remote_control_sample_sequence
bool remote_control_complete
bool remote_control_completion_rising_edge

# 调试验证模式和 debug override 当前输入。
bool debug_validation_enabled
bool simulation_only
bool debug_override_active
bool debug_obstacle_detected
float64 debug_obstacle_distance

# 软件急停锁存和 manual reset 请求状态。
bool software_emergency_stop_latched
bool manual_reset_requested

# RobotContext 最终使用的输入快照。
bool lower_alive
bool imu_ready
bool posture_ready
bool grip_confirmed
bool joint_fault
bool grip_fault
bool obstacle_detected
float64 range_to_obstacle
