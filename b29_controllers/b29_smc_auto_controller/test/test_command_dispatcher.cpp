#include <gtest/gtest.h>

#include <array>

#include <b29_smc_auto_controller/command_dispatcher.h>

TEST(CommandDispatcherTest, SafeStopClearsWheelOutputsAndFreezesJoints)
{
  std::array<double, 6> positions{{1.0, 2.0, 3.0, 4.0, 5.0, 6.0}};
  std::array<double, 6> velocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> efforts{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> position_commands{{9.0, 9.0, 9.0, 9.0, 9.0, 9.0}};

  std::array<double, 2> wheel_positions{{0.0, 0.0}};
  std::array<double, 2> wheel_velocities{{0.0, 0.0}};
  std::array<double, 2> wheel_efforts{{0.0, 0.0}};
  std::array<double, 2> wheel_commands{{4.2, -4.2}};

  b29_smc_auto_controller::CommandDispatcher dispatcher;

  std::array<hardware_interface::JointHandle, 6> position_handles{{
      hardware_interface::JointHandle(hardware_interface::JointStateHandle("left_first_leg_joint", &positions[0],
                                                                           &velocities[0], &efforts[0]),
                                      &position_commands[0]),
      hardware_interface::JointHandle(hardware_interface::JointStateHandle("left_second_leg_joint", &positions[1],
                                                                           &velocities[1], &efforts[1]),
                                      &position_commands[1]),
      hardware_interface::JointHandle(hardware_interface::JointStateHandle("left_rod_joint", &positions[2],
                                                                           &velocities[2], &efforts[2]),
                                      &position_commands[2]),
      hardware_interface::JointHandle(hardware_interface::JointStateHandle("right_first_leg_joint", &positions[3],
                                                                           &velocities[3], &efforts[3]),
                                      &position_commands[3]),
      hardware_interface::JointHandle(hardware_interface::JointStateHandle("right_second_leg_joint", &positions[4],
                                                                           &velocities[4], &efforts[4]),
                                      &position_commands[4]),
      hardware_interface::JointHandle(hardware_interface::JointStateHandle("right_rod_joint", &positions[5],
                                                                           &velocities[5], &efforts[5]),
                                      &position_commands[5]),
  }};

  std::array<hardware_interface::JointHandle, 2> wheel_handles{{
      hardware_interface::JointHandle(hardware_interface::JointStateHandle("left_friction_wheel_joint", &wheel_positions[0],
                                                                           &wheel_velocities[0], &wheel_efforts[0]),
                                      &wheel_commands[0]),
      hardware_interface::JointHandle(hardware_interface::JointStateHandle("right_friction_wheel_joint",
                                                                           &wheel_positions[1], &wheel_velocities[1],
                                                                           &wheel_efforts[1]),
                                      &wheel_commands[1]),
  }};

  dispatcher.configure(position_handles, wheel_handles);

  b29_smc_auto_controller::AutoControlCommand command;
  command.stop_all = true;
  command.freeze_joints = true;
  command.left_wheel_speed = 3.0;
  command.right_wheel_speed = -3.0;
  command.joint_targets = {10.0, 11.0, 12.0, 13.0, 14.0, 15.0};

  dispatcher.dispatch(command);

  EXPECT_DOUBLE_EQ(wheel_commands[0], 0.0);
  EXPECT_DOUBLE_EQ(wheel_commands[1], 0.0);
  for (std::size_t i = 0; i < position_commands.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(position_commands[i], positions[i]);
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
