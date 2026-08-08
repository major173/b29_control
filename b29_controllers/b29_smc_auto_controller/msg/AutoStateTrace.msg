# Obstacle crossing strategy enumeration: No crossing, Line clamp, Damper
uint8 CROSSING_NONE=0
uint8 CROSSING_LINE_CLAMP=1
uint8 CROSSING_DAMPER=2

# Message timestamp and coordinate frame information
std_msgs/Header header

# Outer RobotFSM state and information of the latest state transition
string current_state
string previous_state
string last_event
string transition_reason

# Root cause of the final effective command, root cause of base command, and output mode
string command_reason
string base_command_reason
string output_mode

# Safety control flags for the final effective command
bool stop_all
bool freeze_joints

# Gravity compensation mode of the issued final command: 0=Disabled, 1=Left first leg, 2=Right first leg
uint8 gravity_compensation_mode

# Drive wheel commands contained in the final effective command
string drive_mode
float64 left_wheel_speed
float64 right_wheel_speed

# Six position joint targets in the final effective command
# Order: left_first_leg, left_second_leg, left_gripper,
#        right_first_leg, right_second_leg, right_gripper
float64[6] joint_targets
float64 left_gripper_target
float64 right_gripper_target
uint8 crossing_strategy

# Flags indicating whether CommandDispatcher attempted writing and whether the write handle succeeded
# This field does NOT indicate that the lower controller has physically responded
bool command_dispatch_attempted
bool command_dispatch_succeeded

# Current state, side, initial crossing side and stage timing of the main obstacle-crossing FSM
bool obstacle_crossing_active
string obstacle_crossing_stage
string obstacle_crossing_side
string first_crossing_side
time obstacle_crossing_stage_enter_time
float64 obstacle_crossing_stage_elapsed_sec
string obstacle_crossing_transition_reason

# Current step, step timing and transition description of the cable-disconnect sub-flow
string disconnect_step
time disconnect_step_enter_time
float64 disconnect_step_elapsed_sec
string disconnect_step_transition_reason
string disconnect_step_expected_condition

# Three-pose-joint settling data used by Step7
float64 disconnect_max_abs_pose_joint_velocity
float64 disconnect_settle_velocity_threshold
bool disconnect_velocity_within_threshold
float64 disconnect_velocity_stable_elapsed_sec
float64 disconnect_settle_duration
string current_side_gripper
float64 current_side_gripper_target

# Manual intervention active flag and the failed action that triggered manual intervention
bool manual_intervention_active
string manual_intervention_reason
string failed_action

# Retry counter for the current failed action, individual retry counters for each action, and unified retry upper limit
uint32 retry_count
uint32 close_grippers_retry_count
uint32 left_disconnect_retry_count
uint32 right_disconnect_retry_count
uint32 left_regrip_retry_count
uint32 right_regrip_retry_count
uint32 retry_limit

# State, elapsed time and timeout flag while waiting for grip confirmation signal
bool waiting_for_grip_confirmed
float64 grip_wait_elapsed_sec
bool grip_wait_timed_out
string last_failure_reason

# PlannerControl stage state, manual release switch, and flag indicating whether planner target point is available
bool planner_control_active
bool planner_manual_release_enabled
bool planner_release_received
float64 planner_control_wait_elapsed_sec
bool planner_point_available
bool planner_point_fresh
bool planner_override_applied

# Manual remote control stage input after PlannerControl, actually applied increments and accumulated joint targets
bool remote_control_active
float64[4] remote_control_raw_increments
float64[4] remote_control_applied_increments
float64[4] remote_control_joint_targets
# Flag indicating whether the current four-joint increment sample from remote control can be applied
bool remote_control_increments_valid
uint64 remote_control_sample_sequence
bool remote_control_complete
bool remote_control_completion_rising_edge

# 双轮实际位置反馈相对本次越障周期基准的有符号净角行程，单位 rad
float64 left_wheel_travel_baseline_position
float64 right_wheel_travel_baseline_position
float64 left_wheel_travel_current_position
float64 right_wheel_travel_current_position
float64 signed_wheel_travel
bool wheel_travel_baseline_initialized

# Debug validation mode switch and current input of debug override function
bool debug_validation_enabled
bool simulation_only
bool debug_override_active
bool temporary_allow_start_without_grip_confirmed
bool debug_obstacle_crossing_trigger
bool debug_obstacle_detected
float64 debug_obstacle_distance

# Software emergency stop latch status and manual reset request flag
bool software_emergency_stop_latched
bool manual_reset_requested

# Snapshot of raw sensor & controller inputs finally consumed by RobotContext
bool lower_alive
bool imu_ready
bool posture_ready
bool grip_confirmed
bool joint_fault
bool grip_fault
bool obstacle_detected
float64 range_to_obstacle

# 下位机巡航与自动流程输入，以及控制器实际消费到的边沿
uint8 cruise_drive_request_raw
string cruise_drive_request
bool cruise_drive_request_valid
bool auto_start
bool manual_reset
bool obstacle_crossing_trigger
uint64 auto_start_rising_edge_sequence
uint64 manual_reset_rising_edge_sequence
bool obstacle_trigger_rising_edge
bool obstacle_trigger_falling_edge
uint64 obstacle_trigger_rising_edge_sequence
uint64 obstacle_trigger_falling_edge_sequence
