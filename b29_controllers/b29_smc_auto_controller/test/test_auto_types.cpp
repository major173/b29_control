#include <gtest/gtest.h>

#include <cstddef>

#include <b29_smc_auto_controller/auto_types.h>
#include <b29_smc_auto_controller/AutoDebugOverride.h>
#include <b29_smc_auto_controller/AutoSensorInput.h>
#include <b29_smc_auto_controller/AutoStateTrace.h>

TEST(AutoTypesTest, AutoInputSnapshotDefaultsSafe)
{
  b29_smc_auto_controller::AutoInputSnapshot input;

  EXPECT_FALSE(input.auto_start_requested);
  EXPECT_FALSE(input.manual_reset_requested);
  EXPECT_FALSE(input.emergency_stop);
  EXPECT_FALSE(input.lower_alive);
  EXPECT_FALSE(input.imu_ready);
  EXPECT_FALSE(input.posture_ready);
  EXPECT_EQ(input.obstacle_type, b29_smc_auto_controller::ObstacleType::Unknown);
  EXPECT_FALSE(input.obstacle_detected);
}

TEST(AutoTypesTest, AutoControlCommandDefaultsSafe)
{
  b29_smc_auto_controller::AutoControlCommand command;

  EXPECT_EQ(command.drive_mode, b29_smc_auto_controller::DriveMode::Stop);
  EXPECT_DOUBLE_EQ(command.left_wheel_speed, 0.0);
  EXPECT_DOUBLE_EQ(command.right_wheel_speed, 0.0);
  EXPECT_TRUE(command.stop_all);
  EXPECT_TRUE(command.freeze_joints);
  EXPECT_EQ(command.crossing_strategy, b29_smc_auto_controller::CrossingStrategy::None);
  EXPECT_EQ(command.command_reason, "default_safe");
}

TEST(AutoTypesTest, DebugOverrideDataDefaultsSafe)
{
  b29_smc_auto_controller::DebugOverrideData data;

  EXPECT_FALSE(data.enabled);
  EXPECT_EQ(data.field_mask, 0u);
  EXPECT_FALSE(data.auto_start_requested);
  EXPECT_FALSE(data.manual_reset_requested);
  EXPECT_FALSE(data.emergency_stop);
  EXPECT_FALSE(data.lower_alive);
  EXPECT_FALSE(data.imu_ready);
  EXPECT_FALSE(data.posture_ready);
  EXPECT_FALSE(data.grip_confirmed);
  EXPECT_FALSE(data.joint_fault);
  EXPECT_FALSE(data.grip_fault);
  EXPECT_FALSE(data.obstacle_detected);
  EXPECT_EQ(data.obstacle_type, b29_smc_auto_controller::ObstacleType::Unknown);
  EXPECT_FALSE(data.classification_stable);
  EXPECT_DOUBLE_EQ(data.range_to_obstacle, 0.0);
  EXPECT_FALSE(data.at_crossing_position);
  EXPECT_FALSE(data.crossing_step_done);
  EXPECT_FALSE(data.crossing_complete);
  EXPECT_FALSE(data.post_check_passed);
  EXPECT_FALSE(data.post_check_failed);
}

TEST(AutoTypesTest, DebugOverrideMaskValuesAreUnique)
{
  constexpr std::array<uint32_t, 18> masks = {
    b29_smc_auto_controller::DebugOverrideMask::AutoStartRequested,
    b29_smc_auto_controller::DebugOverrideMask::ManualResetRequested,
    b29_smc_auto_controller::DebugOverrideMask::EmergencyStop,
    b29_smc_auto_controller::DebugOverrideMask::LowerAlive,
    b29_smc_auto_controller::DebugOverrideMask::ImuReady,
    b29_smc_auto_controller::DebugOverrideMask::PostureReady,
    b29_smc_auto_controller::DebugOverrideMask::GripConfirmed,
    b29_smc_auto_controller::DebugOverrideMask::JointFault,
    b29_smc_auto_controller::DebugOverrideMask::GripFault,
    b29_smc_auto_controller::DebugOverrideMask::ObstacleDetected,
    b29_smc_auto_controller::DebugOverrideMask::ObstacleType,
    b29_smc_auto_controller::DebugOverrideMask::ClassificationStable,
    b29_smc_auto_controller::DebugOverrideMask::RangeToObstacle,
    b29_smc_auto_controller::DebugOverrideMask::AtCrossingPosition,
    b29_smc_auto_controller::DebugOverrideMask::CrossingStepDone,
    b29_smc_auto_controller::DebugOverrideMask::CrossingComplete,
    b29_smc_auto_controller::DebugOverrideMask::PostCheckPassed,
    b29_smc_auto_controller::DebugOverrideMask::PostCheckFailed};

  for (std::size_t i = 0; i < masks.size(); ++i)
  {
    EXPECT_NE(masks[i], 0u);
    EXPECT_EQ(masks[i] & (masks[i] - 1), 0u);
    for (std::size_t j = i + 1; j < masks.size(); ++j)
    {
      EXPECT_NE(masks[i], masks[j]);
    }
  }
}

TEST(AutoTypesTest, MessageConstantsAreAvailable)
{
  EXPECT_EQ(b29_smc_auto_controller::AutoDebugOverride::OBSTACLE_UNKNOWN, 0u);
  EXPECT_EQ(b29_smc_auto_controller::AutoDebugOverride::FIELD_POSTURE_READY, 32u);
  EXPECT_EQ(b29_smc_auto_controller::AutoStateTrace::CROSSING_DAMPER, 2u);
  EXPECT_EQ(b29_smc_auto_controller::AutoSensorInput::OBSTACLE_LINE_CLAMP, 1u);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
