//
// Created by yuchen on 2023/1/27.
//

#pragma once

#include <array>
#include <memory>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include <unordered_map>
#include <vector>

#define __packed __attribute__((packed))

// ROS
#include <xmlrpcpp/XmlRpcValue.h>
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
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <transmission_interface/transmission_loader.h>

// SMC
#include "steering_engine/common/auto_state_interface.h"

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
  void unpack(std::vector<uint8_t> rx_buffer,const ros::Time &time);

  bool loadUrdf(ros::NodeHandle &root_nh);
  void loadImuCovarianceParams(ros::NodeHandle &root_nh);
  void setInterface();
  bool setupTransmission(ros::NodeHandle &root_nh);

  void setKDLSegment();
  void addChildren(const KDL::SegmentMap::const_iterator segment);

  bool loadProtocolConfig(ros::NodeHandle &root_nh);
  void jointSpeedTargetCallback(const std_msgs::Float64::ConstPtr &msg);
  void clawSpeedTargetCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
  
  void updateImuState(double acc_x, double acc_y, double acc_z,
                    double gyro_x, double gyro_y, double gyro_z,
                    double qw, double qx, double qy, double qz);

  bool initAutoStateData(AutoStateData &data);
  void tryReconnectSerial(const ros::Time& time);
  void handleSerialIoError(const std::string& context,const serial::IOException& e);

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
    for (size_t i = 0; i < k_frame_length_; i++)
      tx_buffer_[i] = 0;
    tx_len_ = 0;
  }
  void clearRxBuffer() {
    for (auto iter = rx_buffer_.begin(); iter != rx_buffer_.end();) {
      iter = rx_buffer_.erase(iter);
    }
  }
  void processRxBuffer(const ros::Time &time);
  static unsigned char getCrc8(unsigned char *ptr, unsigned short len) {
    unsigned char crc;
    unsigned char i;
    crc = 0xFF;
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
  enum ActuatorIndex {
    kLeftFirstLeg = 0,
    kLeftSecondLeg,
    kLeftRod,
    kLeftFrictionWheel,
    kRightFirstLeg,
    kRightSecondLeg,
    kRightRod,
    kRightFrictionWheel,
    kActuatorCount
  };

  struct ControlMap {
    std::array<ActuatorIndex, 2> wheel_speed{};
    std::array<ActuatorIndex, 2> claw_speed{};
    std::array<ActuatorIndex, 2> claw_angle{};
    std::array<ActuatorIndex, 4> joint_angle{};
  };

  struct ProtocolTopics {
    std::string joint_speed_target;
    std::string claw_speed_target;
  };

  double angle_[kActuatorCount]{}, vel_[kActuatorCount]{},
      effort_[kActuatorCount]{};
  double cmd_[kActuatorCount]{};
  serial::Serial serial_;

  // interface of the robot
  hardware_interface::PositionActuatorInterface position_act_interface_;
  hardware_interface::VelocityActuatorInterface velocity_act_interface_;
  hardware_interface::ActuatorStateInterface act_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::ImuSensorInterface imu_sensor_interface_;
  rm_control::RobotStateInterface robot_state_interface_;
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  std::vector<hardware_interface::JointStateHandle> joint_state_handles_{};
  std::vector<hardware_interface::JointHandle> position_joint_handles_{};
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_{};

  // interface for auto
  AutoStateInterface auto_state_interface_;

  // data for auto
  AutoStateData auto_state_data_{};
  ros::Time last_rx_time_{};

  // transmission of the robot
  transmission_interface::ActuatorToJointStateInterface
      *act_to_jnt_state_interface_{};
  transmission_interface::JointToActuatorPositionInterface
      *jnt_to_act_position_interface_{};
  transmission_interface::JointToActuatorVelocityInterface
      *jnt_to_act_velocity_interface_{};
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
  int tx_len_;
  static constexpr size_t k_frame_length_ = 52;
  static constexpr size_t k_header_length_ = 2;
  static constexpr size_t k_ctrl_length_ = 1;
  static constexpr size_t k_length_ = 1;
  static constexpr size_t k_data_length_ = 44;
  static constexpr size_t k_gravity_compensation_length_ = 1;
  static constexpr size_t k_crc_length_ = 1;
  static constexpr size_t k_tail_length_ = 2;
  static constexpr size_t k_feedback_motor_count_ = kActuatorCount;
  static constexpr size_t k_feedback_motor_entry_length_ = 13;
  static constexpr size_t k_feedback_imu_float_count_ = 10;
  static constexpr size_t k_feedback_remote_joint_count_ = 4;
  static constexpr size_t k_feedback_status_length_ = 4;
  static constexpr size_t k_feedback_payload_length_ =
      k_feedback_motor_count_ * k_feedback_motor_entry_length_ +
      k_feedback_imu_float_count_ * sizeof(float) +
      k_feedback_remote_joint_count_ * sizeof(float) +
      k_feedback_status_length_;
  static constexpr size_t k_feedback_frame_length_ =
      k_header_length_ + k_ctrl_length_ + k_length_ +
      k_feedback_payload_length_ + k_crc_length_ + k_tail_length_;
  uint8_t tx_buffer_[k_frame_length_];

  //通信协议常量
  const unsigned char header[2] = {0x55, 0xaa};
  const unsigned char ender[2] = {0x0d, 0x0a};
  const unsigned char control_code_ = 0x01;
  
  std::array<double, 4> imu_orientation_{{0.0, 0.0, 0.0, 1.0}};
  std::array<double, 9> imu_orientation_covariance_{{0.0}};
  std::array<double, 3> imu_angular_velocity_{{0.0, 0.0, 0.0}};
  std::array<double, 9> imu_angular_velocity_covariance_{{0.0}};
  std::array<double, 3> imu_linear_acceleration_{{0.0, 0.0, 0.0}};
  std::array<double, 9> imu_linear_acceleration_covariance_{{0.0}};

  // actor offset
  std::array<double, kActuatorCount> offset_vector_{};

  std::array<int, kActuatorCount> actuator_id_map_{};
  std::unordered_map<int, ActuatorIndex> id_to_actuator_{};
  std::unordered_map<std::string, ActuatorIndex> joint_to_actuator_{};
  ControlMap control_map_{};
  ProtocolTopics protocol_topics_{};
  uint8_t gravity_compensation_mode_{0};
  double joint_speed_target_{1.0};
  std::array<double, 2> claw_speed_target_{{1.0, 1.0}};
  ros::Subscriber joint_speed_sub_;
  ros::Subscriber claw_speed_sub_;
};

typedef struct {
  unsigned char header_[2]; // k_header_length_
  unsigned char ctrl_;      // k_ctrl_length_
  unsigned char length_;    // k_length_
  unsigned char data_[44];  // k_data_length_
  unsigned char gravity_compensation_mode_;
  unsigned char crc_;       // k_crc_length_
  unsigned char ender_[2];  // k_tail_length_
} __packed SerialFrame;
static_assert(sizeof(SerialFrame) == 52,
              "SerialFrame must match the 52-byte control protocol");
} // namespace steering_engine_hw
