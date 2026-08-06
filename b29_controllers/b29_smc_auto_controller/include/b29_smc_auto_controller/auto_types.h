// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <ros/time.h>

namespace b29_smc_auto_controller
{
enum class ObstacleType : uint8_t
{
  Unknown = 0,
  LineClamp = 1,
  Damper = 2,
};

enum class DriveMode : uint8_t
{
  Stop = 0,
  Forward = 1,
  Reverse = 2,
};

enum class CruiseDriveRequest : uint8_t
{
  Stop = 0,
  Forward = 1,
  Reverse = 2,
  Invalid = 255,
};

enum class CrossingStrategy : uint8_t
{
  None = 0,
  LineClamp = 1,
  Damper = 2,
};

enum class GravityCompensationMode : uint8_t
{
  Off = 0,
  LeftFirstLeg = 1,
  RightFirstLeg = 2,
};

enum class GripperState : uint8_t
{
  Open = 1,
  Closed = 2,
  HalfOpen = 3,
};

enum class JointIndex : uint8_t
{
  LeftFirstLeg = 0,
  LeftSecondLeg,
  LeftGripper,
  RightFirstLeg,
  RightSecondLeg,
  RightGripper,
};

enum class ObstacleCrossingStage : uint8_t
{
  Idle,
  CloseBothGrippers,
  OpenGripperBeforeGravityCompensation,
  EnableGravityCompensation,
  Disconnecting,
  PlannerControl,
  RemoteControl,
  Regrip,
  ReopenBeforeRemoteControl,
  CompleteWaitObstacleClear,
  ManualIntervention,
};

enum class CrossingSide : uint8_t
{
  None,
  Left,
  Right,
};

enum class FailedOperation : uint8_t
{
  None,
  CloseBothGrippers,
  DisconnectCable,
  PlannerControl,
  Regrip,
};

struct DebugOverrideMask
{
  static constexpr uint32_t AutoStartRequested = 1u << 0;
  static constexpr uint32_t ManualResetRequested = 1u << 1;
  static constexpr uint32_t EmergencyStop = 1u << 2;
  static constexpr uint32_t LowerAlive = 1u << 3;
  static constexpr uint32_t ImuReady = 1u << 4;
  static constexpr uint32_t PostureReady = 1u << 5;
  static constexpr uint32_t GripConfirmed = 1u << 6;
  static constexpr uint32_t JointFault = 1u << 7;
  static constexpr uint32_t GripFault = 1u << 8;
  static constexpr uint32_t ObstacleCrossingTrigger = 1u << 9;
  static constexpr uint32_t ObstacleType = 1u << 10;
  static constexpr uint32_t ClassificationStable = 1u << 11;
  static constexpr uint32_t CruiseDriveRequest = 1u << 12;
  static constexpr uint32_t AtCrossingPosition = 1u << 13;
  static constexpr uint32_t CrossingStepDone = 1u << 14;
  static constexpr uint32_t CrossingComplete = 1u << 15;
  static constexpr uint32_t PostCheckPassed = 1u << 16;
  static constexpr uint32_t PostCheckFailed = 1u << 17;
  static constexpr uint32_t AutoRunPause = 1u << 18;
};

struct DebugOverrideData
{
  ros::Time stamp{};
  bool enabled{false};
  uint32_t field_mask{0};
  bool auto_start_requested{false};
  bool manual_reset_requested{false};
  bool emergency_stop{false};
  bool lower_alive{false};
  bool imu_ready{false};
  bool posture_ready{false};
  bool grip_confirmed{false};
  bool joint_fault{false};
  bool grip_fault{false};
  bool obstacle_crossing_trigger{false};
  ObstacleType obstacle_type{ObstacleType::Unknown};
  bool classification_stable{false};
  uint8_t cruise_drive_request_raw{0};
  bool at_crossing_position{false};
  bool crossing_step_done{false};
  bool crossing_complete{false};
  bool post_check_passed{false};
  bool post_check_failed{false};
  bool auto_run_pause{false};
};

struct AutoControlRequest
{
  ros::Time stamp{};
  bool auto_start_requested{false};
  bool manual_reset_requested{false};
  bool emergency_stop{false};
};

struct AutoInputMuxConfig
{
  double max_abs_roll_rad{0.35};
  double max_abs_pitch_rad{0.35};
};

struct AutoInputSnapshot
{
  ros::Time stamp{};
  bool auto_start_requested{false};
  bool manual_reset_requested{false};
  bool emergency_stop{false};
  bool lower_alive{false};
  bool imu_ready{false};
  bool posture_ready{false};
  bool grip_confirmed{false};
  bool joint_fault{false};
  bool grip_fault{false};
  CruiseDriveRequest cruise_drive_request{CruiseDriveRequest::Stop};
  uint8_t cruise_drive_request_raw{0};
  bool cruise_drive_request_valid{true};
  bool obstacle_crossing_trigger{false};
  bool auto_start_edge_sequence_valid{false};
  bool manual_reset_edge_sequence_valid{false};
  bool obstacle_trigger_edge_sequences_valid{false};
  std::uint64_t auto_start_rising_edge_sequence{0};
  std::uint64_t manual_reset_rising_edge_sequence{0};
  std::uint64_t obstacle_trigger_rising_edge_sequence{0};
  std::uint64_t obstacle_trigger_falling_edge_sequence{0};
  ObstacleType obstacle_type{ObstacleType::Unknown};
  bool classification_stable{false};
  bool at_crossing_position{false};
  bool crossing_step_done{false};
  bool crossing_complete{false};
  bool post_check_passed{false};
  bool post_check_failed{false};
  bool auto_run_pause{false};
  std::array<double, 4> remote_control_joint_increments{};
  bool remote_control_complete{false};
  bool remote_control_increments_valid{false};
  std::uint64_t remote_control_sample_sequence{0};
  std::uint64_t remote_control_completion_rising_edge_sequence{0};
};

struct AutoControlCommand
{
  DriveMode drive_mode{DriveMode::Stop};
  double left_wheel_speed{0.0};
  double right_wheel_speed{0.0};
  std::array<double, 6> joint_targets{};
  bool stop_all{true};
  bool freeze_joints{true};
  GravityCompensationMode gravity_compensation_mode{GravityCompensationMode::Off};
  CrossingStrategy crossing_strategy{CrossingStrategy::None};
  std::string command_reason{"default_safe"};
};
}  // namespace b29_smc_auto_controller
