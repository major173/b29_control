#include <gtest/gtest.h>

#include <b29_smc_auto_controller/disconnect_cable_process.h>

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

DisconnectCableProcess makeProcess()
{
  DisconnectCableProcess process;
  DisconnectCableProcess::Config config;
  config.step_duration = 0.0;
  config.step_interval = 0.0;
  config.step3_first_joint_target = -0.53;
  config.step5_first_joint_target = -0.05;
  config.settle_velocity_threshold = 0.02;
  config.settle_duration = 0.5;
  process.configure(config);
  return process;
}

void advanceRightSideToStep7(DisconnectCableProcess& process,
                             DisconnectCableProcess::JointTargets& commands,
                             DisconnectCableProcess::JointTargets& positions,
                             DisconnectCableProcess::JointTargets& velocities)
{
  const CrossingSideProfile* profile = crossingSideProfile(CrossingSide::Right);
  ASSERT_NE(profile, nullptr);
  process.start(at(0.0), *profile, "test_start");
  process.update(at(0.0), commands, positions, velocities);
  process.update(at(0.1), commands, positions, velocities);
  ASSERT_EQ(process.traceState().step, DisconnectCableStep::Step7CheckIfCableDisconnected);
}

TEST(DisconnectCableProcess, CommandsOnlyTheLockedArmFirstJoint)
{
  DisconnectCableProcess process = makeProcess();
  DisconnectCableProcess::JointTargets commands{{0.11, 0.22, 0.0, 0.33, 0.44, 0.0}};
  DisconnectCableProcess::JointTargets positions{};
  DisconnectCableProcess::JointTargets velocities{};
  const CrossingSideProfile* profile = crossingSideProfile(CrossingSide::Right);
  ASSERT_NE(profile, nullptr);
  process.start(at(0.0), *profile, "test_start");

  const DisconnectCableProcess::Result up =
      process.update(at(0.0), commands, positions, velocities);
  EXPECT_DOUBLE_EQ(up.joint_targets[static_cast<std::size_t>(JointIndex::LeftFirstLeg)], 0.53);
  EXPECT_DOUBLE_EQ(up.joint_targets[static_cast<std::size_t>(JointIndex::LeftSecondLeg)], 0.22);
  EXPECT_DOUBLE_EQ(up.joint_targets[static_cast<std::size_t>(JointIndex::RightFirstLeg)], 0.33);
  EXPECT_DOUBLE_EQ(up.joint_targets[static_cast<std::size_t>(JointIndex::RightSecondLeg)], 0.44);

  const DisconnectCableProcess::Result down =
      process.update(at(0.1), commands, positions, velocities);
  EXPECT_DOUBLE_EQ(down.joint_targets[static_cast<std::size_t>(JointIndex::LeftFirstLeg)], 0.05);
  EXPECT_DOUBLE_EQ(down.joint_targets[static_cast<std::size_t>(JointIndex::LeftSecondLeg)], 0.22);
  EXPECT_DOUBLE_EQ(down.joint_targets[static_cast<std::size_t>(JointIndex::RightFirstLeg)], 0.33);
  EXPECT_DOUBLE_EQ(down.joint_targets[static_cast<std::size_t>(JointIndex::RightSecondLeg)], 0.44);
  EXPECT_EQ(process.traceState().step, DisconnectCableStep::Step7CheckIfCableDisconnected);
}

TEST(DisconnectCableProcess, CompletesAfterStableThreeJointVelocityWithoutDisplacementGate)
{
  DisconnectCableProcess process = makeProcess();
  DisconnectCableProcess::JointTargets commands{};
  DisconnectCableProcess::JointTargets positions{};
  DisconnectCableProcess::JointTargets velocities{};
  advanceRightSideToStep7(process, commands, positions, velocities);

  velocities[static_cast<std::size_t>(JointIndex::LeftFirstLeg)] = 0.01;
  velocities[static_cast<std::size_t>(JointIndex::LeftSecondLeg)] = -0.01;
  velocities[static_cast<std::size_t>(JointIndex::RightFirstLeg)] = 0.005;
  process.update(at(0.2), commands, positions, velocities);
  EXPECT_TRUE(process.traceState().velocity_within_threshold);
  EXPECT_EQ(process.consumeOutcome(), DisconnectCableProcess::Outcome::None);

  process.update(at(0.71), commands, positions, velocities);
  EXPECT_EQ(process.consumeOutcome(), DisconnectCableProcess::Outcome::Completed);
}

TEST(DisconnectCableProcess, VelocityMustRemainLowForTheWholeSettleWindow)
{
  DisconnectCableProcess process = makeProcess();
  DisconnectCableProcess::JointTargets commands{};
  DisconnectCableProcess::JointTargets positions{};
  DisconnectCableProcess::JointTargets velocities{};
  advanceRightSideToStep7(process, commands, positions, velocities);

  process.update(at(0.2), commands, positions, velocities);

  velocities[static_cast<std::size_t>(JointIndex::LeftFirstLeg)] = 0.03;
  process.update(at(0.4), commands, positions, velocities);
  EXPECT_FALSE(process.traceState().velocity_within_threshold);
  EXPECT_EQ(process.consumeOutcome(), DisconnectCableProcess::Outcome::None);

  velocities[static_cast<std::size_t>(JointIndex::LeftFirstLeg)] = 0.0;
  process.update(at(0.5), commands, positions, velocities);
  process.update(at(0.99), commands, positions, velocities);
  EXPECT_EQ(process.consumeOutcome(), DisconnectCableProcess::Outcome::None);
  process.update(at(1.01), commands, positions, velocities);
  EXPECT_EQ(process.consumeOutcome(), DisconnectCableProcess::Outcome::Completed);
}

TEST(DisconnectCableProcess, LockedArmSecondJointVelocityRemainsPartOfThreeJointGate)
{
  DisconnectCableProcess process = makeProcess();
  DisconnectCableProcess::JointTargets commands{};
  DisconnectCableProcess::JointTargets positions{};
  DisconnectCableProcess::JointTargets velocities{};
  advanceRightSideToStep7(process, commands, positions, velocities);

  velocities[static_cast<std::size_t>(JointIndex::LeftSecondLeg)] = 0.03;
  process.update(at(0.2), commands, positions, velocities);
  EXPECT_FALSE(process.traceState().velocity_within_threshold);
  EXPECT_EQ(process.consumeOutcome(), DisconnectCableProcess::Outcome::None);
}

TEST(DisconnectCableProcess, FreeArmSecondJointVelocityIsNotPartOfThreeJointGate)
{
  DisconnectCableProcess process = makeProcess();
  DisconnectCableProcess::JointTargets commands{};
  DisconnectCableProcess::JointTargets positions{};
  DisconnectCableProcess::JointTargets velocities{};
  advanceRightSideToStep7(process, commands, positions, velocities);

  velocities[static_cast<std::size_t>(JointIndex::RightSecondLeg)] = 100.0;
  process.update(at(0.2), commands, positions, velocities);
  EXPECT_TRUE(process.traceState().velocity_within_threshold);
  process.update(at(0.71), commands, positions, velocities);
  EXPECT_EQ(process.consumeOutcome(), DisconnectCableProcess::Outcome::Completed);
}
}  // namespace
}  // namespace b29_smc_auto_controller

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
