//
// Created by yuchen on 2023/1/27.
//

#pragma once

#include <memory>
#include <std_msgs/String.h>
#include <string>
#include <unordered_map>
#include <vector>

#define __packed __attribute__((packed))

// ROS
#include <XmlRpcValue.h>
#include <geometry_msgs/TransformStamped.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <serial/serial.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/transform_broadcaster.h>
#include <urdf/model.h>

// ROS control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <transmission_interface/transmission_loader.h>

namespace steering_engine_hw {

class SegmentPair {
public:
  SegmentPair(const KDL::Segment &p_segment, std::string p_root,
              std::string p_tip)
      : segment(p_segment), root(std::move(p_root)), tip(std::move(p_tip)) {}

  KDL::Segment segment{};
  std::string root, tip;
};

class StRobotHW : public hardware_interface::RobotHW {
public:
  StRobotHW() = default;
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load
   * urdf of robot. Set up transmission and joint limit. Set configuration of
   * series.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;
  /** \brief Comunicate with hardware. Get datas, status of robot.
   *
   * Call @ref rm_hw::CanBus::read(). Check whether temperature of actuator is
   * too high and whether actuator is offline. Propagate actuator state to joint
   * state for the stored transmission. Set all cmd to zero to avoid crazy soft
   * limit oscillation when not controller loaded(all controllers update after
   * read()).
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time &time, const ros::Duration &period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref
   * rm_hw::CanBus::write(). Publish actuator current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time &time, const ros::Duration &period) override;

  void pack(unsigned char *tx_buffer, unsigned char ctrl, unsigned char *data);
  void unpack(std::vector<uint8_t> rx_buffer);

  bool loadUrdf(ros::NodeHandle &root_nh);
  void setInterface();
  bool setupTransmission(ros::NodeHandle &root_nh);

  void setKDLSegment();
  void addChildren(const KDL::SegmentMap::const_iterator segment);
  void updateTf(const ros::Time &time);
  //去除输入字符串开头的斜线
  std::string stripSlash(const std::string &in) {
    if (!in.empty() && in[0] == '/') {
      return in.substr(1);
    }
    return in;
  }

  void updateControllerManager(const ros::Time &time, const ros::Duration &dt) {
    controller_manager_->update(time, dt);
  }

  void clearTxBuffer() {
    for (int i = 0; i < k_frame_length_; i++)
      tx_buffer_[i] = 0;
    tx_len_ = 0;
  }
  void clearRxBuffer() {
    for (auto iter = rx_buffer_.begin(); iter != rx_buffer_.end();) {
      iter = rx_buffer_.erase(iter);
    }
  }
  static unsigned char getCrc8(unsigned char *ptr, unsigned short len) {
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while (len--) {
      crc ^= *ptr++;
      for (i = 0; i < 8; i++) {
        if (crc & 0x01)
          crc = (crc >> 1) ^ 0x8C;
        else
          crc >>= 1;
      }
    }
    return crc;
  }

private:
  double angle_[4], vel_[4], effort_[4];
  double cmd_[4];
  serial::Serial serial_;

  // interface of the robot
  hardware_interface::PositionActuatorInterface position_act_interface_;
  hardware_interface::ActuatorStateInterface act_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  std::vector<hardware_interface::JointStateHandle> joint_state_handles_{};
  std::vector<hardware_interface::JointHandle> position_joint_handles_{};

  // transmission of the robot
  transmission_interface::ActuatorToJointStateInterface
      *act_to_jnt_state_interface_{};
  transmission_interface::JointToActuatorPositionInterface
      *jnt_to_act_position_interface_{};
  transmission_interface::RobotTransmissions robot_transmissions_;
  std::unique_ptr<transmission_interface::TransmissionInterfaceLoader>
      transmission_interface_loader_;

  // URDF model of the robot
  std::string urdf_string_;                 // for transmission
  std::shared_ptr<urdf::Model> urdf_model_; // for limit

  // TF broader
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped transformStamped_;
  std::map<std::string, SegmentPair> segments_, segments_fixed_;
  std::map<std::string, hardware_interface::JointStateHandle>
      joint_states_segment_;

  int rx_len_;
  std::vector<uint8_t> rx_buffer_;
  uint8_t tx_buffer_[19];
  int tx_len_;
  const int k_frame_length_ = 19, k_header_length_ = 2, k_ctrl_length_ = 1,
            k_length_ = 1, k_data_length_ = 12, k_crc_length_ = 1,
            k_tail_length_ = 2;

  //通信协议常量
  const unsigned char header[2] = {0x55, 0xaa};
  const unsigned char ender[2] = {0x0d, 0x0a};

  // actor offset
  std::vector<double> offset_vector_ = {0, 90, 90, 0};
};

typedef struct {
  unsigned char header_[2]; // k_header_length_
  unsigned char ctrl_;      // k_ctrl_length_
  unsigned char length_;    // k_length_
  unsigned char data_[12];  // k_data_length_
  unsigned char crc_;       // k_crc_length_
  unsigned char ender_[2];  // k_tail_length_
} __packed SerialFrame;
} // namespace steering_engine_hw
