#include <gtest/gtest.h>

#include <b29_smc_auto_controller/obstacle_crossing_runtime.h>

namespace b29_smc_auto_controller
{
namespace
{
ros::Time at(double seconds)
{
  ros::Time time;
  time.fromSec(seconds);
  return time;
}

ObstacleCrossingRuntime makeRuntime()
{
  ObstacleCrossingRuntime runtime;
  ObstacleCrossingRuntime::Config config;
  config.wait_for_grip_respond_time = 0.1;
  config.retry_limit = 2;
  runtime.configure(config);
  return runtime;
}

TEST(ObstacleCrossingRuntime, NegativeWheelTravelStartsRightSideAndSkipsCloseBoth)
{
  ObstacleCrossingRuntime runtime = makeRuntime();
  ObstacleCrossingRuntime::Inputs inputs;
  inputs.signed_wheel_travel = -0.5;
  inputs.grip_confirmed = true;
  ObstacleCrossingRuntime::Events events;
  events.start_disconnect_requested = true;

  const ObstacleCrossingRuntime::Actions actions = runtime.update(at(1.0), inputs, events);

  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::OpenGripperBeforeGravityCompensation);
  EXPECT_EQ(runtime.crossingSide(), CrossingSide::Right);
  EXPECT_EQ(runtime.traceState().first_crossing_side, CrossingSide::Right);
  EXPECT_FALSE(actions.start_disconnect_process);
  EXPECT_EQ(runtime.gravityCompensationMode(), GravityCompensationMode::Off);
}

TEST(ObstacleCrossingRuntime, NonNegativeWheelTravelStartsLeftSide)
{
  ObstacleCrossingRuntime runtime = makeRuntime();
  ObstacleCrossingRuntime::Inputs inputs;
  inputs.signed_wheel_travel = 0.5;
  inputs.grip_confirmed = true;
  ObstacleCrossingRuntime::Events events;
  events.start_disconnect_requested = true;

  runtime.update(at(1.0), inputs, events);

  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::OpenGripperBeforeGravityCompensation);
  EXPECT_EQ(runtime.crossingSide(), CrossingSide::Left);
  EXPECT_EQ(runtime.traceState().first_crossing_side, CrossingSide::Left);
}

TEST(ObstacleCrossingRuntime, CompletedDisconnectWaitsForSeparateFlipStart)
{
  ObstacleCrossingRuntime runtime = makeRuntime();
  ObstacleCrossingRuntime::Inputs inputs;
  inputs.signed_wheel_travel = -0.5;
  inputs.grip_confirmed = true;
  ObstacleCrossingRuntime::Events events;
  events.start_disconnect_requested = true;
  runtime.update(at(1.0), inputs, events);

  events = ObstacleCrossingRuntime::Events{};
  runtime.update(at(1.11), inputs, events);
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::EnableGravityCompensation);
  EXPECT_EQ(runtime.gravityCompensationMode(), GravityCompensationMode::LeftFirstLeg);

  const ObstacleCrossingRuntime::Actions start_actions = runtime.update(at(1.12), inputs, events);
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::Disconnecting);
  EXPECT_TRUE(start_actions.start_disconnect_process);

  events.disconnect = DisconnectCableProcess::Outcome::Completed;
  const ObstacleCrossingRuntime::Actions completed_actions = runtime.update(at(1.20), inputs, events);
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::DisconnectDoneWaitFlip);
  EXPECT_EQ(runtime.crossingSide(), CrossingSide::Right);
  EXPECT_TRUE(completed_actions.reset_disconnect_process);
  EXPECT_EQ(completed_actions.planner_action, ObstacleCrossingRuntime::PlannerAction::None);
  EXPECT_EQ(runtime.gravityCompensationMode(), GravityCompensationMode::LeftFirstLeg);

  events = ObstacleCrossingRuntime::Events{};
  const ObstacleCrossingRuntime::Actions wait_actions = runtime.update(at(200.0), inputs, events);
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::DisconnectDoneWaitFlip);
  EXPECT_FALSE(wait_actions.start_disconnect_process);
  EXPECT_EQ(wait_actions.planner_action, ObstacleCrossingRuntime::PlannerAction::None);

  events.start_flip_requested = true;
  const ObstacleCrossingRuntime::Actions flip_actions = runtime.update(at(200.1), inputs, events);
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::PlannerControl);
  EXPECT_EQ(runtime.traceState().transition_reason, "start_flip_requested");
  EXPECT_EQ(flip_actions.planner_action, ObstacleCrossingRuntime::PlannerAction::Start);
}

TEST(ObstacleCrossingRuntime, SafetyResetLeavesDisconnectWaitState)
{
  ObstacleCrossingRuntime runtime = makeRuntime();
  ObstacleCrossingRuntime::Inputs inputs;
  inputs.signed_wheel_travel = -0.5;
  inputs.grip_confirmed = true;
  ObstacleCrossingRuntime::Events events;
  events.start_disconnect_requested = true;
  runtime.update(at(1.0), inputs, events);
  events = ObstacleCrossingRuntime::Events{};
  runtime.update(at(1.11), inputs, events);
  runtime.update(at(1.12), inputs, events);
  events.disconnect = DisconnectCableProcess::Outcome::Completed;
  runtime.update(at(1.20), inputs, events);

  inputs.safety_blocked = true;
  const ObstacleCrossingRuntime::Actions reset_actions = runtime.update(at(1.30), inputs, events);
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::Idle);
  EXPECT_EQ(runtime.crossingSide(), CrossingSide::None);
  EXPECT_EQ(runtime.gravityCompensationMode(), GravityCompensationMode::Off);
  EXPECT_TRUE(reset_actions.reset_disconnect_process);
  EXPECT_EQ(reset_actions.planner_action,
            ObstacleCrossingRuntime::PlannerAction::RevokeForSafety);
}

TEST(ObstacleCrossingRuntime, PlannerCompletionHandsOffToRemoteAndRemoteCompletionStartsRegrip)
{
  ObstacleCrossingRuntime runtime = makeRuntime();
  ObstacleCrossingRuntime::Inputs inputs;
  inputs.signed_wheel_travel = -0.5;
  inputs.grip_confirmed = true;
  ObstacleCrossingRuntime::Events events;
  events.start_disconnect_requested = true;
  runtime.update(at(1.0), inputs, events);

  events = ObstacleCrossingRuntime::Events{};
  runtime.update(at(1.11), inputs, events);
  runtime.update(at(1.12), inputs, events);
  events.disconnect = DisconnectCableProcess::Outcome::Completed;
  runtime.update(at(1.20), inputs, events);

  events = ObstacleCrossingRuntime::Events{};
  events.start_flip_requested = true;
  const ObstacleCrossingRuntime::Actions flip_actions = runtime.update(at(1.21), inputs, events);
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::PlannerControl);
  EXPECT_EQ(flip_actions.planner_action, ObstacleCrossingRuntime::PlannerAction::Start);

  events = ObstacleCrossingRuntime::Events{};
  events.planner = ObstacleCrossingRuntime::PlannerOutcome::Completed;
  const ObstacleCrossingRuntime::Actions handoff_actions = runtime.update(at(1.30), inputs, events);
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::RemoteControl);
  EXPECT_EQ(runtime.crossingSide(), CrossingSide::Right);
  EXPECT_TRUE(handoff_actions.start_remote_control_session);
  EXPECT_FALSE(handoff_actions.remote_control_completed_regrip_entered);

  events = ObstacleCrossingRuntime::Events{};
  events.remote_control_completion_rising_edge = true;
  const ObstacleCrossingRuntime::Actions regrip_actions = runtime.update(at(1.40), inputs, events);
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::Regrip);
  EXPECT_EQ(runtime.crossingSide(), CrossingSide::Right);
  EXPECT_TRUE(regrip_actions.remote_control_completed_regrip_entered);

  const ObstacleCrossingRuntime::Actions retry_actions =
      runtime.update(at(1.51), inputs, ObstacleCrossingRuntime::Events{});
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::ReopenBeforeRemoteControl);
  EXPECT_EQ(runtime.traceState().right_regrip_retry_count, 1u);
  EXPECT_FALSE(retry_actions.start_remote_control_session);

  const ObstacleCrossingRuntime::Actions retry_handoff_actions =
      runtime.update(at(1.62), inputs, ObstacleCrossingRuntime::Events{});
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::RemoteControl);
  EXPECT_TRUE(retry_handoff_actions.start_remote_control_session);
}

TEST(ObstacleCrossingRuntime, ConfirmedFirstRegripSwitchesRolesAndStartsMirroredDisconnect)
{
  ObstacleCrossingRuntime runtime = makeRuntime();
  ObstacleCrossingRuntime::Inputs inputs;
  inputs.signed_wheel_travel = -0.5;
  inputs.grip_confirmed = true;
  ObstacleCrossingRuntime::Events events;
  events.start_disconnect_requested = true;
  runtime.update(at(1.0), inputs, events);

  events = ObstacleCrossingRuntime::Events{};
  runtime.update(at(1.11), inputs, events);
  runtime.update(at(1.12), inputs, events);
  events.disconnect = DisconnectCableProcess::Outcome::Completed;
  runtime.update(at(1.20), inputs, events);

  events = ObstacleCrossingRuntime::Events{};
  events.start_flip_requested = true;
  runtime.update(at(1.21), inputs, events);
  events = ObstacleCrossingRuntime::Events{};
  events.planner = ObstacleCrossingRuntime::PlannerOutcome::Completed;
  const ObstacleCrossingRuntime::Actions handoff_actions =
      runtime.update(at(1.30), inputs, events);
  EXPECT_TRUE(handoff_actions.reset_disconnect_process);
  EXPECT_TRUE(handoff_actions.start_remote_control_session);

  events = ObstacleCrossingRuntime::Events{};
  events.remote_control_completion_rising_edge = true;
  runtime.update(at(1.40), inputs, events);
  ASSERT_EQ(runtime.stage(), ObstacleCrossingStage::Regrip);
  ASSERT_EQ(runtime.crossingSide(), CrossingSide::Right);

  // A stale high confirmation cannot switch roles.  Regrip requires a fresh
  // low-to-high edge after the close command has been issued.
  events = ObstacleCrossingRuntime::Events{};
  runtime.update(at(1.41), inputs, events);
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::Regrip);
  EXPECT_EQ(runtime.crossingSide(), CrossingSide::Right);

  inputs.grip_confirmed = false;
  runtime.update(at(1.42), inputs, events);
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::Regrip);

  inputs.grip_confirmed = true;
  const ObstacleCrossingRuntime::Actions role_switch_actions =
      runtime.update(at(1.43), inputs, events);
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::OpenGripperBeforeGravityCompensation);
  EXPECT_EQ(runtime.crossingSide(), CrossingSide::Left);
  EXPECT_EQ(runtime.gravityCompensationMode(), GravityCompensationMode::Off);
  EXPECT_FALSE(role_switch_actions.start_disconnect_process);

  const CrossingSideProfile* profile = runtime.currentProfile();
  ASSERT_NE(profile, nullptr);
  EXPECT_EQ(profile->gripper, JointIndex::LeftGripper);
  EXPECT_EQ(profile->actuator_first_leg, JointIndex::RightFirstLeg);
  EXPECT_EQ(profile->actuator_second_leg, JointIndex::RightSecondLeg);
  EXPECT_EQ(profile->pose_joints[0], JointIndex::RightFirstLeg);
  EXPECT_EQ(profile->pose_joints[1], JointIndex::RightSecondLeg);
  EXPECT_EQ(profile->pose_joints[2], JointIndex::LeftFirstLeg);

  runtime.update(at(1.54), inputs, events);
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::EnableGravityCompensation);
  EXPECT_EQ(runtime.gravityCompensationMode(), GravityCompensationMode::RightFirstLeg);

  const ObstacleCrossingRuntime::Actions second_disconnect_actions =
      runtime.update(at(1.55), inputs, events);
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::Disconnecting);
  EXPECT_TRUE(second_disconnect_actions.start_disconnect_process);
  EXPECT_EQ(runtime.crossingSide(), CrossingSide::Left);
}

TEST(ObstacleCrossingRuntime, DisconnectFailureHoldsWithoutRetryOrGripAutoRecovery)
{
  ObstacleCrossingRuntime runtime = makeRuntime();
  ObstacleCrossingRuntime::Inputs inputs;
  inputs.signed_wheel_travel = -0.5;
  inputs.grip_confirmed = true;
  ObstacleCrossingRuntime::Events events;
  events.start_disconnect_requested = true;
  runtime.update(at(1.0), inputs, events);
  events = ObstacleCrossingRuntime::Events{};
  runtime.update(at(1.11), inputs, events);
  runtime.update(at(1.12), inputs, events);
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::Disconnecting);

  events.disconnect = DisconnectCableProcess::Outcome::Failed;
  const ObstacleCrossingRuntime::Actions failed_actions = runtime.update(at(1.20), inputs, events);
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::ManualIntervention);
  EXPECT_TRUE(failed_actions.disconnect_failed_hold_entered);
  EXPECT_TRUE(failed_actions.reset_disconnect_process);
  EXPECT_EQ(runtime.traceState().retry_count, 0u);
  EXPECT_EQ(runtime.traceState().last_failure_reason, "disconnect_right_validation_failed_hold");

  events = ObstacleCrossingRuntime::Events{};
  const ObstacleCrossingRuntime::Actions hold_actions = runtime.update(at(10.0), inputs, events);
  EXPECT_EQ(runtime.stage(), ObstacleCrossingStage::ManualIntervention);
  EXPECT_EQ(runtime.crossingSide(), CrossingSide::Right);
  EXPECT_FALSE(hold_actions.start_disconnect_process);
}
}  // namespace
}  // namespace b29_smc_auto_controller

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
