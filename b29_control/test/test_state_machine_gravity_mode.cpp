#include <gtest/gtest.h>

#include "steering_engine/common/gravity_compensator.h"

namespace steering_engine_hw
{
TEST(StateMachineGravityMode, DisabledAndInvalidModesHaveNoSupport)
{
  SupportSide support = SupportSide::RIGHT;
  EXPECT_FALSE(stateMachineGravityModeToSupport(0u, support));
  EXPECT_FALSE(stateMachineGravityModeToSupport(3u, support));
  EXPECT_FALSE(stateMachineGravityModeToSupport(255u, support));
}

TEST(StateMachineGravityMode, LockedFirstLegSelectsSameSideAnchor)
{
  SupportSide support = SupportSide::RIGHT;
  ASSERT_TRUE(stateMachineGravityModeToSupport(1u, support));
  EXPECT_EQ(support, SupportSide::LEFT);

  ASSERT_TRUE(stateMachineGravityModeToSupport(2u, support));
  EXPECT_EQ(support, SupportSide::RIGHT);
}
}  // namespace steering_engine_hw

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
