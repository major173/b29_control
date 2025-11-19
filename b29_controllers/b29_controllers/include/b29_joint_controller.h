//
// Created by yuchen on 25-9-18.
//

#ifndef B29_JOINT_CONTROLLER_H
#define B29_JOINT_CONTROLLER_H

#include <ros/ros.h>
#include <rm_common/filters/filters.h>
#include <rm_msgs/DbusData.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <control_toolbox/pid.h>
#include <nav_msgs/Odometry.h>
#include <urdf/model.h>


namespace b29_controllers
{
class B29JointController
    : public controller_interface::MultiInterfaceController<
      rm_control::RobotStateInterface,
      hardware_interface::ImuSensorInterface,
      hardware_interface::PositionJointInterface,
      hardware_interface::VelocityJointInterface>
{
public:
  B29JointController() = default;
  ~B29JointController() override = default;

  // Controller interface methods
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

private:
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& msg);
  void updateArmMode(const ros::Time& time, const ros::Duration& period);
  void updateTrackMode(const ros::Time& time, const ros::Duration& period);

  rm_control::RobotStateHandle robot_state_handle_{};
  hardware_interface::PositionJointInterface* position_joint_interface_{};
  hardware_interface::VelocityJointInterface* velocity_joint_interface_{};

  double publish_rate_{};
  double max_odom_vel_{};
  bool enable_odom_tf_ = false;
  bool topic_update_ = false;
  bool publish_odom_tf_ = false;
  bool state_changed_ = false;

  enum
  {
    TRACK,
    ARM,
  };

  int state_ = ARM;
  RampFilter<double>* ramp_x_{};
  std::string front_frame_{}, base_frame_{};

  ros::Time last_publish_time_;
  geometry_msgs::TransformStamped odom2base_{};
  tf2::Transform world2odom_;
  geometry_msgs::Vector3 vel_cmd_{}; // x, y
  control_toolbox::Pid pid_follow_;

  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
  rm_common::TfRtBroadcaster tf_broadcaster_{};
  ros::Subscriber outside_odom_sub_;
  ros::Subscriber cmd_chassis_sub_;
  ros::Subscriber dbus_sub_;
  rm_msgs::DbusData cmd_struct_;
  realtime_tools::RealtimeBuffer<rm_msgs::DbusData> cmd_rt_buffer_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> odom_buffer_;
};
} // namespace b29_controllers

#endif // B29_JOINT_CONTROLLER_H