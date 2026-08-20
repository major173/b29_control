#include <limits>

#include <gtest/gtest.h>

#include <b29_smc_auto_controller/remote_control_session.h>

namespace b29_smc_auto_controller
{
namespace
{
RemoteControlSession makeSession()
{
  RemoteControlSession session;
  RemoteControlSession::Config config;
  config.increment_deadband = 0.005;
  config.increment_scale = 0.02;
  config.max_increment_per_sample = 0.05;
  config.joint_direction_signs = {{-1.0, 1.0, 1.0, 1.0}};
  session.configure(config);
  return session;
}

TEST(RemoteControlSession, AppliesEachFreshSampleOnceWithScaleDirectionAndClamp)
{
  RemoteControlSession session = makeSession();
  RemoteControlSession::Input input;
  input.sample_sequence = 10;
  input.completion_rising_edge_sequence = 3;
  session.start({{1.0, 2.0, 3.0, 4.0}}, input);

  input.sample_sequence = 11;
  input.increments = {{1.0, -2.0, 0.001, 4.0}};
  input.increments_valid = true;
  const RemoteControlSession::UpdateResult result = session.update(input);

  EXPECT_TRUE(result.sample_consumed);
  EXPECT_TRUE(result.increments_applied);
  EXPECT_FALSE(result.completion_rising_edge);
  EXPECT_DOUBLE_EQ(session.appliedIncrements()[0], -0.02);
  EXPECT_DOUBLE_EQ(session.appliedIncrements()[1], -0.04);
  EXPECT_DOUBLE_EQ(session.appliedIncrements()[2], 0.0);
  EXPECT_DOUBLE_EQ(session.appliedIncrements()[3], 0.05);
  EXPECT_DOUBLE_EQ(session.targets()[0], 0.98);
  EXPECT_DOUBLE_EQ(session.targets()[1], 1.96);
  EXPECT_DOUBLE_EQ(session.targets()[2], 3.0);
  EXPECT_DOUBLE_EQ(session.targets()[3], 4.05);

  const RemoteControlSession::UpdateResult duplicate = session.update(input);
  EXPECT_FALSE(duplicate.sample_consumed);
  EXPECT_DOUBLE_EQ(session.targets()[0], 0.98);
  EXPECT_DOUBLE_EQ(session.targets()[1], 1.96);
  EXPECT_DOUBLE_EQ(session.targets()[2], 3.0);
  EXPECT_DOUBLE_EQ(session.targets()[3], 4.05);
}

TEST(RemoteControlSession, RejectsInvalidOrNonFiniteIncrementSamples)
{
  RemoteControlSession session = makeSession();
  RemoteControlSession::Input input;
  input.sample_sequence = 1;
  session.start({{0.0, 0.0, 0.0, 0.0}}, input);

  input.sample_sequence = 2;
  input.increments = {{1.0, 1.0, 1.0, 1.0}};
  input.increments_valid = false;
  RemoteControlSession::UpdateResult result = session.update(input);
  EXPECT_TRUE(result.sample_consumed);
  EXPECT_FALSE(result.increments_applied);
  EXPECT_EQ(session.targets(), (RemoteControlSession::Positions{{0.0, 0.0, 0.0, 0.0}}));

  input.sample_sequence = 3;
  input.increments_valid = true;
  input.increments[2] = std::numeric_limits<double>::quiet_NaN();
  result = session.update(input);
  EXPECT_TRUE(result.sample_consumed);
  EXPECT_FALSE(result.increments_applied);
  EXPECT_EQ(session.targets(), (RemoteControlSession::Positions{{0.0, 0.0, 0.0, 0.0}}));
}

TEST(RemoteControlSession, RequiresANewCompletionRisingEdgeAfterHandoff)
{
  RemoteControlSession session = makeSession();
  RemoteControlSession::Input input;
  input.sample_sequence = 4;
  input.completion_rising_edge_sequence = 7;
  session.start({{0.0, 0.0, 0.0, 0.0}}, input);

  EXPECT_FALSE(session.update(input).completion_rising_edge);
  input.completion_rising_edge_sequence = 8;
  EXPECT_TRUE(session.update(input).completion_rising_edge);
}
}  // namespace
}  // namespace b29_smc_auto_controller

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
