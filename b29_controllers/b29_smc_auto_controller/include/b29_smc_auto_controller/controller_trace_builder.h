// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <ros/time.h>

#include <b29_smc_auto_controller/AutoStateTrace.h>
#include <b29_smc_auto_controller/disconnect_cable_process.h>
#include <b29_smc_auto_controller/robot_context.h>

namespace b29_smc_auto_controller
{
// Observation states collected by the Controller within one update cycle.
// This structure is only used for trace generation,
// and does not participate in FSM, command arbitration, or any hardware write operations.
struct ControllerTraceState
{
  ObstacleCrossingStage obstacle_crossing_stage{ObstacleCrossingStage::Idle};
  CrossingSide crossing_side{CrossingSide::None};
  CrossingSide first_crossing_side{CrossingSide::Left};
  ros::Time obstacle_crossing_stage_enter_time{};
  std::string obstacle_crossing_transition_reason;

  DisconnectCableStep disconnect_step{DisconnectCableStep::Idle};
  ros::Time disconnect_step_enter_time{};
  std::string disconnect_step_transition_reason;
  double disconnect_check_displacement{0.0};
  double to_check_joint_pos{0.0};
  double disconnect_cable_second_joint_success_threshold{0.0};

  FailedOperation failed_operation{FailedOperation::None};
  std::uint32_t retry_count{0};
  std::uint32_t close_grippers_retry_count{0};
  std::uint32_t left_disconnect_retry_count{0};
  std::uint32_t right_disconnect_retry_count{0};
  std::uint32_t left_regrip_retry_count{0};
  std::uint32_t right_regrip_retry_count{0};
  std::uint32_t retry_limit{0};
  ros::Time gripper_wait_start_time{};
  double wait_for_grip_respond_time{0.0};
  std::string last_failure_reason;

  bool planner_manual_release_enabled{false};
  bool planner_release_received{false};
  bool planner_point_available{false};
  bool planner_point_fresh{false};
  bool planner_override_applied{false};

  std::array<double, 4> remote_control_raw_increments{};
  std::array<double, 4> remote_control_applied_increments{};
  std::array<double, 4> remote_control_joint_targets{};
  bool remote_control_increments_valid{false};
  std::uint64_t remote_control_sample_sequence{0};
  bool remote_control_complete{false};
  bool remote_control_completion_rising_edge{false};

  bool command_dispatch_attempted{false};
  bool command_dispatch_succeeded{false};

  bool debug_validation_enabled{false};
  bool simulation_only{false};
  bool debug_override_active{false};
  bool debug_obstacle_detected{false};
  double debug_obstacle_distance{0.0};
  bool software_emergency_stop_latched{false};

  bool manual_reset_requested{false};
  bool lower_alive{false};
  bool imu_ready{false};
  bool posture_ready{false};
  bool grip_confirmed{false};
  bool joint_fault{false};
  bool grip_fault{false};
  bool obstacle_detected{false};
  double range_to_obstacle{0.0};
};

class ControllerTraceBuilder
{
public:
  static AutoStateTrace build(const ros::Time& stamp, const AutoControlCommand& effective_command,
                              const RobotContextTraceState& robot_state,
                              const ControllerTraceState& controller_state);
};
}  // namespace b29_smc_auto_controller
