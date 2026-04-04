#include <gtest/gtest.h>

#include <b29_smc_auto_controller/auto_types.h>
#include <b29_smc_auto_controller/robot_context.h>

TEST(RobotContextTest, AutoStartTransitionsToAutoInit)
{
  b29_smc_auto_controller::AutoInputSnapshot input;
  input.lower_alive = true;
  input.imu_ready = true;
  input.posture_ready = true;
  input.grip_confirmed = true;

  b29_smc_auto_controller::RobotContext context;
  context.start();
  context.setInputSnapshot(input);
  context.requestAutoStart();
  context.tick50Hz();

  EXPECT_EQ(context.currentStateName(), "AutoInit");
}

TEST(RobotContextTest, EmergencyStopAlwaysWins)
{
  b29_smc_auto_controller::AutoInputSnapshot input;
  input.emergency_stop = true;

  b29_smc_auto_controller::RobotContext context;
  context.start();
  context.setInputSnapshot(input);
  context.tick50Hz();

  EXPECT_EQ(context.currentStateName(), "SafeStop");
  EXPECT_TRUE(context.currentCommand().stop_all);
}

TEST(RobotContextTest, AutoStartRequestLatchesUntilReady)
{
  b29_smc_auto_controller::AutoInputSnapshot input;
  input.lower_alive = true;

  b29_smc_auto_controller::RobotContext context;
  context.start();
  context.setInputSnapshot(input);
  context.requestAutoStart();
  context.tick50Hz();

  EXPECT_EQ(context.currentStateName(), "Idle");

  input.imu_ready = true;
  input.posture_ready = true;
  input.grip_confirmed = true;
  context.setInputSnapshot(input);
  context.tick50Hz();

  EXPECT_EQ(context.currentStateName(), "AutoInit");
}

TEST(RobotContextTest, ContinuousAutoStartInputDoesNotBlockAutoInitProgress)
{
  b29_smc_auto_controller::AutoInputSnapshot input;
  input.lower_alive = true;
  input.imu_ready = true;
  input.posture_ready = true;
  input.grip_confirmed = true;
  input.auto_start_requested = true;

  b29_smc_auto_controller::RobotContext context;
  context.start();
  context.setInputSnapshot(input);
  context.tick50Hz();

  ASSERT_EQ(context.currentStateName(), "AutoInit");

  context.setInputSnapshot(input);
  context.tick50Hz();

  EXPECT_EQ(context.currentStateName(), "Traversing");
}

TEST(RobotContextTest, CommsLossRestoresToIdleWhenLowerAliveReturns)
{
  b29_smc_auto_controller::RobotContext context;
  context.start();

  b29_smc_auto_controller::AutoInputSnapshot input;
  input.lower_alive = false;
  context.setInputSnapshot(input);
  context.tick50Hz();

  ASSERT_EQ(context.currentStateName(), "CommsLoss");

  input.lower_alive = true;
  context.setInputSnapshot(input);
  context.tick50Hz();

  EXPECT_EQ(context.currentStateName(), "Idle");
}

TEST(RobotContextTest, SafeStopResetsToIdleWhenManualResetRequested)
{
  b29_smc_auto_controller::RobotContext context;
  context.start();

  b29_smc_auto_controller::AutoInputSnapshot input;
  input.emergency_stop = true;
  context.setInputSnapshot(input);
  context.tick50Hz();

  ASSERT_EQ(context.currentStateName(), "SafeStop");

  input.emergency_stop = false;
  input.manual_reset_requested = true;
  input.lower_alive = true;
  context.setInputSnapshot(input);
  context.tick50Hz();

  EXPECT_EQ(context.currentStateName(), "Idle");
}

TEST(RobotContextTest, AutoStartRaisedDuringCommsLossIsDiscardedAfterRecovery)
{
  b29_smc_auto_controller::RobotContext context;
  context.start();

  b29_smc_auto_controller::AutoInputSnapshot input;
  input.lower_alive = false;
  context.setInputSnapshot(input);
  context.tick50Hz();
  ASSERT_EQ(context.currentStateName(), "CommsLoss");

  input.auto_start_requested = true;
  input.imu_ready = true;
  input.posture_ready = true;
  input.grip_confirmed = true;
  context.setInputSnapshot(input);
  context.tick50Hz();
  ASSERT_EQ(context.currentStateName(), "CommsLoss");

  input.auto_start_requested = false;
  input.lower_alive = true;
  context.setInputSnapshot(input);
  context.tick50Hz();
  ASSERT_EQ(context.currentStateName(), "Idle");

  context.tick50Hz();
  EXPECT_EQ(context.currentStateName(), "Idle");
}

TEST(RobotContextTest, AutoStartRaisedDuringSafeStopIsDiscardedAfterReset)
{
  b29_smc_auto_controller::RobotContext context;
  context.start();

  b29_smc_auto_controller::AutoInputSnapshot input;
  input.emergency_stop = true;
  context.setInputSnapshot(input);
  context.tick50Hz();
  ASSERT_EQ(context.currentStateName(), "SafeStop");

  input.emergency_stop = false;
  input.auto_start_requested = true;
  input.lower_alive = true;
  input.imu_ready = true;
  input.posture_ready = true;
  input.grip_confirmed = true;
  context.setInputSnapshot(input);
  context.tick50Hz();
  ASSERT_EQ(context.currentStateName(), "SafeStop");

  input.auto_start_requested = false;
  input.manual_reset_requested = true;
  context.setInputSnapshot(input);
  context.tick50Hz();
  ASSERT_EQ(context.currentStateName(), "Idle");

  input.manual_reset_requested = false;
  context.setInputSnapshot(input);
  context.tick50Hz();
  EXPECT_EQ(context.currentStateName(), "Idle");
}

TEST(RobotContextTest, PublicCommandHelpersAreAvailable)
{
  b29_smc_auto_controller::RobotContext context;

  context.setCruiseCommand();
  EXPECT_EQ(context.currentCommand().drive_mode, b29_smc_auto_controller::DriveMode::Forward);
  EXPECT_FALSE(context.currentCommand().stop_all);

  context.setApproachCommand();
  EXPECT_EQ(context.currentCommand().drive_mode, b29_smc_auto_controller::DriveMode::Forward);
  EXPECT_FALSE(context.currentCommand().stop_all);

  context.setSafeStopCommand("review_fix");
  EXPECT_EQ(context.currentCommand().drive_mode, b29_smc_auto_controller::DriveMode::Stop);
  EXPECT_TRUE(context.currentCommand().stop_all);
  EXPECT_EQ(context.currentCommand().command_reason, "review_fix");
}

TEST(RobotContextTest, StopAndFreezeHelpersArePublic)
{
  b29_smc_auto_controller::RobotContext context;

  context.setCruiseCommand();
  context.stopAllMotors();
  context.freezeAllJoints();

  EXPECT_EQ(context.currentCommand().drive_mode, b29_smc_auto_controller::DriveMode::Stop);
  EXPECT_TRUE(context.currentCommand().stop_all);
  EXPECT_TRUE(context.currentCommand().freeze_joints);
}

TEST(RobotContextTest, ObstacleDetectedWhileTraversingDoesNotEnterSafeStop)
{
  b29_smc_auto_controller::AutoInputSnapshot input;
  input.lower_alive = true;
  input.imu_ready = true;
  input.posture_ready = true;
  input.grip_confirmed = true;

  b29_smc_auto_controller::RobotContext context;
  context.start();
  context.setInputSnapshot(input);
  context.requestAutoStart();
  context.tick50Hz();
  ASSERT_EQ(context.currentStateName(), "AutoInit");

  context.tick50Hz();
  ASSERT_EQ(context.currentStateName(), "Traversing");

  input.obstacle_detected = true;
  context.setInputSnapshot(input);
  context.tick50Hz();

  EXPECT_NE(context.currentStateName(), "SafeStop");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
