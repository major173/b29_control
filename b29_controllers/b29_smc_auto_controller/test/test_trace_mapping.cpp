#include <gtest/gtest.h>

#include <b29_smc_auto_controller/robot_context.h>

TEST(StateTraceTest, ReportsCurrentStateAndCommandReason)
{
  b29_smc_auto_controller::RobotContext context;

  const auto trace = context.buildTraceMessage(ros::Time(0));

  EXPECT_FALSE(trace.current_state.empty());
  EXPECT_FALSE(trace.command_reason.empty());
}

TEST(StateTraceTest, ReportsTransitionAndCommandAfterAutoStart)
{
  b29_smc_auto_controller::RobotContext context;
  b29_smc_auto_controller::AutoInputSnapshot input;
  input.lower_alive = true;
  input.imu_ready = true;
  input.posture_ready = true;
  input.grip_confirmed = true;

  context.start();
  context.setInputSnapshot(input);
  context.requestAutoStart();
  context.tick50Hz();

  const auto trace = context.buildTraceMessage(ros::Time(1, 0));

  EXPECT_EQ(trace.current_state, "AutoInit");
  EXPECT_EQ(trace.previous_state, "Idle");
  EXPECT_EQ(trace.transition_reason, "Idle->AutoInit");
  EXPECT_FALSE(trace.command_reason.empty());
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
