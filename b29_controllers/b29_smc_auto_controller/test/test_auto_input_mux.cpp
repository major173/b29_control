#include <gtest/gtest.h>

#include <cmath>

#include <b29_smc_auto_controller/AutoDebugOverride.h>
#include <b29_smc_auto_controller/AutoSensorInput.h>
#include <b29_smc_auto_controller/auto_input_mux.h>

namespace
{
sensor_msgs::Imu makeImuWithRollPitch(const ros::Time& stamp, double roll, double pitch)
{
  sensor_msgs::Imu imu;
  imu.header.stamp = stamp;

  const double half_roll = roll * 0.5;
  const double half_pitch = pitch * 0.5;
  const double sr = std::sin(half_roll);
  const double cr = std::cos(half_roll);
  const double sp = std::sin(half_pitch);
  const double cp = std::cos(half_pitch);

  imu.orientation.x = sr * cp;
  imu.orientation.y = cr * sp;
  imu.orientation.z = -sr * sp;
  imu.orientation.w = cr * cp;
  return imu;
}
}  // namespace

TEST(AutoInputMuxTest, DebugOverrideReplacesOnlyMaskedFields)
{
  b29_smc_auto_controller::AutoInputMux mux;

  b29_smc_auto_controller::AutoSensorInput sensor_input;
  sensor_input.header.stamp = ros::Time(1, 0);
  sensor_input.lower_alive = true;
  sensor_input.grip_confirmed = true;
  sensor_input.obstacle_detected = true;
  sensor_input.obstacle_type = b29_smc_auto_controller::AutoSensorInput::OBSTACLE_LINE_CLAMP;
  sensor_input.range_to_obstacle = 1.5;
  mux.setSensorInput(sensor_input);
  mux.setBaseImu(makeImuWithRollPitch(ros::Time(2, 0), 0.05, 0.05));

  b29_smc_auto_controller::AutoDebugOverride debug_override;
  debug_override.enabled = true;
  debug_override.field_mask = b29_smc_auto_controller::AutoDebugOverride::FIELD_LOWER_ALIVE |
                              b29_smc_auto_controller::AutoDebugOverride::FIELD_OBSTACLE_TYPE |
                              b29_smc_auto_controller::AutoDebugOverride::FIELD_RANGE_TO_OBSTACLE;
  debug_override.lower_alive = false;
  debug_override.obstacle_type = b29_smc_auto_controller::AutoDebugOverride::OBSTACLE_DAMPER;
  debug_override.range_to_obstacle = 0.25;
  mux.setDebugOverride(debug_override);

  const auto snapshot = mux.buildSnapshot();

  EXPECT_FALSE(snapshot.lower_alive);
  EXPECT_TRUE(snapshot.grip_confirmed);
  EXPECT_FALSE(snapshot.imu_ready);
  EXPECT_TRUE(snapshot.posture_ready);
  EXPECT_TRUE(snapshot.obstacle_detected);
  EXPECT_EQ(snapshot.obstacle_type, b29_smc_auto_controller::ObstacleType::Damper);
  EXPECT_DOUBLE_EQ(snapshot.range_to_obstacle, 0.25);
}

TEST(AutoInputMuxTest, ControlRequestIsInjectedIntoSnapshot)
{
  b29_smc_auto_controller::AutoInputMux mux;

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time(4, 0);
  joint_state.name = {"left_first_leg_joint", "right_first_leg_joint"};
  joint_state.position = {0.1, -0.1};
  mux.setJointState(joint_state);

  b29_smc_auto_controller::AutoControlRequest request;
  request.stamp = ros::Time(3, 0);
  request.auto_start_requested = true;
  request.manual_reset_requested = true;
  request.emergency_stop = true;
  mux.setControlRequest(request);

  const auto snapshot = mux.buildSnapshot();

  EXPECT_TRUE(snapshot.auto_start_requested);
  EXPECT_TRUE(snapshot.manual_reset_requested);
  EXPECT_TRUE(snapshot.emergency_stop);
  EXPECT_EQ(snapshot.stamp, ros::Time(4, 0));
}

TEST(AutoInputMuxTest, BaseImuPostureThresholdDeterminesPostureReady)
{
  b29_smc_auto_controller::AutoInputMux mux(
      b29_smc_auto_controller::AutoInputMuxConfig{0.2, 0.2});

  mux.setBaseImu(makeImuWithRollPitch(ros::Time(1, 0), 0.05, 0.1));
  auto snapshot = mux.buildSnapshot();
  EXPECT_FALSE(snapshot.imu_ready);
  EXPECT_TRUE(snapshot.posture_ready);

  mux.setBaseImu(makeImuWithRollPitch(ros::Time(2, 0), 0.25, 0.1));
  snapshot = mux.buildSnapshot();
  EXPECT_FALSE(snapshot.imu_ready);
  EXPECT_FALSE(snapshot.posture_ready);
}

TEST(AutoInputMuxTest, SensorInputControlsImuReadyWhileBaseImuControlsPosture)
{
  b29_smc_auto_controller::AutoInputMux mux(
      b29_smc_auto_controller::AutoInputMuxConfig{0.2, 0.2});

  b29_smc_auto_controller::AutoSensorInput sensor_input;
  sensor_input.imu_ready = true;
  mux.setSensorInput(sensor_input);
  mux.setBaseImu(makeImuWithRollPitch(ros::Time(3, 0), 0.05, 0.05));

  const auto snapshot = mux.buildSnapshot();

  EXPECT_TRUE(snapshot.imu_ready);
  EXPECT_TRUE(snapshot.posture_ready);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
