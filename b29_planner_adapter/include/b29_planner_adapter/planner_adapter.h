// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <b29_smc_auto_controller/CompletePlannerControl.h>
#include <b29_smc_auto_controller/PlannerControlState.h>
#include <b29_smc_auto_controller/PlannerJointCommand.h>
#include <b29_planner_adapter/trajectory_processor.h>

namespace b29_planner_adapter
{
class PlannerAdapter
{
public:
  PlannerAdapter(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
  bool init();

private:
  struct Config
  {
    std::string interface_mode{"production"};
    std::string action_name{"/gp11_moveit/reach_arm_controller/follow_joint_trajectory"};
    std::string planner_state_topic{"/b29_controller/b29_smc_auto_controller/planner_control_state"};
    std::string planner_command_topic{"/b29_controller/b29_smc_auto_controller/planner_joint_command"};
    std::string completion_service{"/b29_controller/b29_smc_auto_controller/complete_planner_control"};
    std::string joint_states_topic{"/joint_states"};
    double publish_rate{50.0};
    double time_scale{1.0};
    double max_output_delta{0.10};
    double expected_smc_max_delta{0.10};
    JointVector start_tolerance{{0.08, 0.08, 0.08, 0.08}};
    JointVector path_tolerance{{0.50, 0.50, 0.50, 0.50}};
    JointVector goal_position_tolerance{{0.02, 0.02, 0.02, 0.02}};
    JointVector goal_velocity_tolerance{{0.05, 0.05, 0.05, 0.05}};
    bool require_goal_velocity{true};
    double settle_time{0.5};
    double joint_state_timeout{0.5};
    double planner_state_timeout{0.25};
    double command_ack_timeout{1.0};
    double action_timeout{180.0};
    double completion_service_timeout{1.0};
    double completion_retry_interval{0.2};
    int completion_retry_count{3};
    double completion_transition_timeout{2.0};
  };

  struct PlannerStateSnapshot
  {
    b29_smc_auto_controller::PlannerControlState state;
    ros::WallTime received_time{};
    bool available{false};
  };

  struct JointStateSnapshot
  {
    JointVector positions{};
    JointVector velocities{};
    bool has_velocities{false};
    ros::WallTime received_time{};
    bool available{false};
  };

  bool loadParameters();
  bool loadJointSpecs(std::array<JointSpec, kPlannerJointCount>& specs);
  bool loadJointVector(const std::string& name, JointVector& values, bool allow_zero) const;
  void plannerStateCallback(const b29_smc_auto_controller::PlannerControlState::ConstPtr& msg);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

  bool getFreshPlannerState(b29_smc_auto_controller::PlannerControlState& state,
                            std::string& error) const;
  bool getFreshJointState(JointStateSnapshot& state, std::string& error) const;
  bool buildGoalIndexMap(const std::vector<std::string>& goal_names,
                         std::array<std::size_t, kPlannerJointCount>& goal_index_for_output,
                         std::string& error) const;
  bool sendCommandAndAwait(uint32_t session_id, uint32_t sequence, const JointVector& positions,
                           const ros::WallTime& action_deadline, std::string& error);
  bool validateSession(uint32_t session_id, b29_smc_auto_controller::PlannerControlState& state,
                       std::string& error) const;
  bool validatePathTolerance(const JointVector& desired, const JointStateSnapshot& actual,
                             std::string& error) const;
  bool goalWithinTolerance(const JointVector& desired, const JointStateSnapshot& actual) const;
  void publishFeedback(const std::vector<std::string>& goal_joint_names,
                       const std::array<std::size_t, kPlannerJointCount>& goal_index_for_output,
                       const JointVector& desired, const JointStateSnapshot& actual);
  bool waitForFinalSettle(uint32_t session_id, uint32_t final_sequence, const JointVector& final_positions,
                          const std::vector<std::string>& goal_joint_names,
                          const std::array<std::size_t, kPlannerJointCount>& goal_index_for_output,
                          const ros::WallTime& action_deadline, std::string& error);
  bool requestCompletion(uint32_t session_id, uint32_t final_sequence, std::string& error);
  bool waitForCompletionState(uint32_t session_id, const ros::WallTime& action_deadline,
                              std::string& error);
  bool preemptRequested(std::string& error);
  void abortAction(int32_t error_code, const std::string& message);

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  Config config_;
  std::array<JointSpec, kPlannerJointCount> joint_specs_{};
  std::unique_ptr<TrajectoryProcessor> trajectory_processor_;
  std::unique_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>> action_server_;
  ros::Subscriber planner_state_subscriber_;
  ros::Subscriber joint_state_subscriber_;
  ros::Publisher planner_command_publisher_;
  ros::ServiceClient completion_client_;
  mutable std::mutex planner_state_mutex_;
  mutable std::mutex joint_state_mutex_;
  PlannerStateSnapshot planner_state_snapshot_{};
  JointStateSnapshot joint_state_snapshot_{};
};
}  // namespace b29_planner_adapter
