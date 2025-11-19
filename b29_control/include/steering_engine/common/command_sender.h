//
// Created by yuchen on 2023/2/3.
//

#pragma once

#include <type_traits>
#include <utility>

#include <rm_common/ros_utilities.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

namespace steering_engine {
template <class MsgType> class CommandSenderBase {
public:
  explicit CommandSenderBase(ros::NodeHandle &nh) {
    if (!nh.getParam("topic", topic_))
      ROS_ERROR("Topic name no defined (namespace: %s)",
                nh.getNamespace().c_str());
    queue_size_ = getParam(nh, "queue_size", 1);
    pub_ = nh.advertise<MsgType>(topic_, queue_size_);
  }
  void setMode(int mode) {
    if (!std::is_same<MsgType, geometry_msgs::Twist>::value &&
        !std::is_same<MsgType, std_msgs::Float64>::value)
      msg_.mode = mode;
  }
  virtual void sendCommand(const ros::Time &time) { pub_.publish(msg_); }
  virtual void updateGameRobotStatus() {}
  virtual void updateGameStatus() {}
  virtual void updateCapacityData() {}
  virtual void updatePowerHeatData() {}
  virtual void setZero() = 0;
  MsgType *getMsg() { return &msg_; }

protected:
  std::string topic_;
  uint32_t queue_size_;
  ros::Publisher pub_;
  MsgType msg_;
};
} // namespace steering_engine
