//
// Created by yuchen on 2023/1/27.
//

#include "steering_engine/common/hardware_interface.h"

#include <cmath>
#include <cstring>

namespace steering_engine_hw {
bool StRobotHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
  //**series**//

  serial::Timeout to = serial::Timeout::simpleTimeout(100);
  serial::parity_t pt = serial::parity_t::parity_none;
  serial::bytesize_t bt = serial::bytesize_t::eightbits;
  serial::flowcontrol_t ft =
      serial::flowcontrol_t::flowcontrol_none;
  serial::stopbits_t st = serial::stopbits_t::stopbits_one;

  std::string port_name = "/dev/usbSteering";
  int baudrate = 115200;
  root_nh.getParam("/steering_engine_hw/serial/port", port_name);
  root_nh.getParam("/steering_engine_hw/serial/baudrate", baudrate);
  serial_.setPort(port_name);
  ROS_INFO("%s", port_name.c_str());
  serial_.setBaudrate(baudrate);
  serial_.setParity(pt);
  serial_.setBytesize(bt);
  serial_.setFlowcontrol(ft);
  serial_.setStopbits(st);
  serial_.setTimeout(to);
  loadImuCovarianceParams(root_nh);
  setInterface();

  if (!loadUrdf(root_nh)) {
    ROS_ERROR("Error occurred while setting up urdf");
    return false;
  }
  // Initialize transmission
  if (!setupTransmission(root_nh)) {
    ROS_ERROR("Error occurred while setting up transmission");
    return false;
  }
  //  Initialize joint limit if (!setupJointLimit(root_nh)) {
  //    ROS_ERROR("Error occurred while setting up joint limit");
  //    return false;
  //  }

  setKDLSegment();
  controller_manager_.reset(new controller_manager::ControllerManager(this));

  if (!loadProtocolConfig(root_nh)) {
    ROS_ERROR("Failed to load protocol config");
    return false;
  }

  if(!initAutoStateData(auto_state_data_)) {
    ROS_ERROR("Failed to init auto state data");
    return false;
  }

  if (serial_.isOpen())
    return true;
  try {
    serial_.open();
    return true;
  } catch (serial::IOException &e) {
    ROS_ERROR("Cannot open serial port %s",serial_.getPort().c_str());
    return false;
  }
}

void StRobotHW::read(const ros::Time &time, const ros::Duration &period) {
  constexpr double kLowerAliveTimeout = 0.2;
  if(!last_rx_time_.isZero() &&
      (time - last_rx_time_).toSec() < kLowerAliveTimeout)
  {
    auto_state_data_.lower_alive = true;
  } 
  else {
    auto_state_data_.lower_alive = false;
  }
  
  if (!serial_.isOpen()) {
    tryReconnectSerial(time);
    return;
  }
  
  size_t available = 0;
  try {
    available = serial_.available();
  } 
  catch (const serial::IOException& e) {
    handleSerialIoError("Serial available() failed", e);
    return;
  }

  if (available == 0) {
    return;
  }

  rx_len_ = static_cast<int>(available);
  std::vector<uint8_t> incoming;
  size_t bytes_read = 0;
  try {
    bytes_read = serial_.read(incoming, static_cast<size_t>(rx_len_));
  } 
  catch (const serial::IOException& e) {
    handleSerialIoError("Serial read() failed", e);
    return;
  }

  if (bytes_read == 0) {
    return;
  }

  incoming.resize(bytes_read);
  rx_buffer_.insert(rx_buffer_.end(), incoming.begin(), incoming.end());
  processRxBuffer(time);

  if (act_to_jnt_state_interface_) {
    act_to_jnt_state_interface_->propagate();
  }
}

void StRobotHW::write(const ros::Time &time, const ros::Duration &period) {
  if (!serial_.isOpen()) {
    tryReconnectSerial(time);
    return;
  }
  
  static std::array<uint8_t, k_data_length_> last_send_data{};
  std::array<uint8_t, k_data_length_> data{};

  if (jnt_to_act_position_interface_) {
    jnt_to_act_position_interface_->propagate();
  }
  if (jnt_to_act_velocity_interface_) {
    jnt_to_act_velocity_interface_->propagate();
  }

  auto sanitizeCommand = [](const char *name, double value, double fallback) {
    if (std::isfinite(value)) {
      return value;
    }
    ROS_WARN_STREAM_THROTTLE(1.0, "Replacing non-finite " << name
                                           << " command with " << fallback);
    return fallback;
  };
  auto positionFallback = [this](ActuatorIndex index) {
    return std::isfinite(angle_[index]) ? angle_[index] : 0.0;
  };

  const auto left_wheel_index    = control_map_.wheel_speed[0];
  const auto right_wheel_index   = control_map_.wheel_speed[1];
  const auto left_claw_index     = control_map_.claw_angle[0];
  const auto right_claw_index    = control_map_.claw_angle[1];
  const auto joint_angle_indices = control_map_.joint_angle;

  const double wheel_speed_left   =
      sanitizeCommand("left wheel speed" , cmd_[left_wheel_index], 0.0);
  const double wheel_speed_right  =
      sanitizeCommand("right wheel speed", cmd_[right_wheel_index], 0.0);
  const double claw_speed_left    =
      sanitizeCommand("left claw speed"  , claw_speed_target_[0], 0.0);
  const double claw_speed_right   =
      sanitizeCommand("right claw speed" , claw_speed_target_[1], 0.0);
  const double claw_angle_left    =
      sanitizeCommand("left claw angle"  , cmd_[left_claw_index],
                      positionFallback(left_claw_index));
  const double claw_angle_right   =
      sanitizeCommand("right claw angle" , cmd_[right_claw_index],
                      positionFallback(right_claw_index));
  const double joint_speed_target =
      sanitizeCommand("joint speed target", joint_speed_target_, 0.0);
  const double joint_angle_targets[4] = {
      sanitizeCommand("left first leg angle"  , cmd_[joint_angle_indices[0]],
                      positionFallback(joint_angle_indices[0])),
      sanitizeCommand("left second leg angle" , cmd_[joint_angle_indices[1]],
                      positionFallback(joint_angle_indices[1])),
      sanitizeCommand("right first leg angle" , cmd_[joint_angle_indices[2]],
                      positionFallback(joint_angle_indices[2])),
      sanitizeCommand("right second leg angle", cmd_[joint_angle_indices[3]],
                      positionFallback(joint_angle_indices[3])),
  };

  uint16_t index = 0;
  auto packFloat = [&data, &index](float value) {
    static_assert(sizeof(float) == 4, "float must be 4 bytes");
    std::memcpy(&data[index], &value, sizeof(float));
    index += sizeof(float);
  };

  packFloat(static_cast<float>(wheel_speed_left));
  packFloat(static_cast<float>(wheel_speed_right));
  packFloat(static_cast<float>(claw_speed_left));
  packFloat(static_cast<float>(claw_speed_right));
  packFloat(static_cast<float>(claw_angle_left));
  packFloat(static_cast<float>(claw_angle_right));
  packFloat(static_cast<float>(joint_speed_target));
  for (double joint_angle_target : joint_angle_targets) {
    packFloat(static_cast<float>(joint_angle_target));
  }

  if (memcmp(data.data(), last_send_data.data(), data.size()) != 0) {
    pack(tx_buffer_, control_code_, data.data());
    tx_len_ = sizeof(tx_buffer_);
    try {
      serial_.write(tx_buffer_, tx_len_);
    } 
    catch (const serial::IOException& e) {
      handleSerialIoError("Serial write() failed", e);
      return;
    }
    memcpy(last_send_data.data(), data.data(), data.size());
  }

  clearTxBuffer();
}

bool StRobotHW::loadUrdf(ros::NodeHandle &root_nh) {
  if (urdf_model_ == nullptr)
    urdf_model_ = std::make_shared<urdf::Model>();
  // get the urdf param on param server
  root_nh.getParam("/robot_description", urdf_string_);
  return !urdf_string_.empty() && urdf_model_->initString(urdf_string_);
}

bool StRobotHW::setupTransmission(ros::NodeHandle &root_nh) {
  try {
    transmission_interface_loader_ =
        std::make_unique<transmission_interface::TransmissionInterfaceLoader>(
            this, &robot_transmissions_);
  } catch (const std::invalid_argument &ex) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. "
                     << ex.what());
    return false;
  } catch (const pluginlib::LibraryLoadException &ex) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. "
                     << ex.what());
    return false;
  } catch (...) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
    return false;
  }

  // Perform transmission loading
  if (!transmission_interface_loader_->load(urdf_string_)) {
    return false;
  }

  act_to_jnt_state_interface_ =
      robot_transmissions_
          .get<transmission_interface::ActuatorToJointStateInterface>();
  jnt_to_act_position_interface_ =
      robot_transmissions_
          .get<transmission_interface::JointToActuatorPositionInterface>();
  jnt_to_act_velocity_interface_ =
      robot_transmissions_
          .get<transmission_interface::JointToActuatorVelocityInterface>();

  auto position_joint_interface =
      this->get<hardware_interface::PositionJointInterface>();
  auto velocity_joint_interface =
      this->get<hardware_interface::VelocityJointInterface>();
  auto joint_state_interface =
      this->get<hardware_interface::JointStateInterface>();

  if (!position_joint_interface) {
    ROS_ERROR("PositionJointInterface is not available in RobotHW");
    return false;
  }
  if (!velocity_joint_interface) {
    ROS_ERROR("VelocityJointInterface is not available in RobotHW");
    return false;
  }
  if (!joint_state_interface) {
    ROS_ERROR("JointStateInterface is not available in RobotHW");
    return false;
  }

  for (const auto &name : position_joint_interface->getNames()) {
    ROS_INFO("position_joint_handle: %s", name.c_str());
    position_joint_handles_.push_back(
        position_joint_interface->getHandle(name));
  }
  for (const auto &name : velocity_joint_interface->getNames()) {
    ROS_INFO("velocity_joint_handle: %s", name.c_str());
    velocity_joint_handles_.push_back(
        velocity_joint_interface->getHandle(name));
  }
  for (const auto &name : joint_state_interface->getNames()) {
    ROS_INFO("joint_state_handle: %s", name.c_str());
    joint_state_handles_.push_back(joint_state_interface->getHandle(name));
    joint_states_segment_.insert(
        std::make_pair(name, joint_state_interface->getHandle(name)));
  }

  return true;
}

void StRobotHW::loadImuCovarianceParams(ros::NodeHandle &root_nh) {
  auto toDouble = [](const XmlRpc::XmlRpcValue &value) -> double {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
      return static_cast<int>(value);
    }
    return static_cast<double>(value);
  };

  auto loadCovarianceDiagonal =
      [&](const std::string &param_name, std::array<double, 9> &covariance,
          const std::array<double, 3> &defaults) {
        covariance = {defaults[0], 0.0, 0.0,
                      0.0, defaults[1], 0.0,
                      0.0, 0.0, defaults[2]};

        XmlRpc::XmlRpcValue diag;
        if (!root_nh.getParam(param_name, diag)) {
          ROS_WARN_STREAM("Missing " << param_name << ", using defaults.");
          return;
        }
        if (diag.getType() != XmlRpc::XmlRpcValue::TypeArray || diag.size() != 3) {
          ROS_WARN_STREAM(param_name << " should be a 3-element array, using defaults.");
          return;
        }

        for (int i = 0; i < 3; ++i) {
          if (diag[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
              diag[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
            ROS_WARN_STREAM(param_name << "[" << i << "] is not numeric, using defaults.");
            return;
          }
        }

        covariance = {toDouble(diag[0]), 0.0, 0.0,
                      0.0, toDouble(diag[1]), 0.0,
                      0.0, 0.0, toDouble(diag[2])};
      };

  loadCovarianceDiagonal("/steering_engine_hw/imu/orientation_covariance_diagonal",
                         imu_orientation_covariance_,
                         {0.0012, 0.0012, 0.0012});

  loadCovarianceDiagonal("/steering_engine_hw/imu/angular_velocity_covariance_diagonal",
                         imu_angular_velocity_covariance_,
                         {0.0004, 0.0004, 0.0004});

  loadCovarianceDiagonal("/steering_engine_hw/imu/linear_acceleration_covariance_diagonal",
                         imu_linear_acceleration_covariance_,
                         {0.01, 0.01, 0.01});
}


void StRobotHW::setInterface() {
  struct ActuatorSpec {
    const char *name;
    ActuatorIndex index;
    bool velocity_interface;
  };

  const ActuatorSpec actuator_specs[] = {
      {"left_first_leg_motor", kLeftFirstLeg, false},
      {"left_second_leg_motor", kLeftSecondLeg, false},
      {"l_gripper_left_drive_motor", kLeftRod, false},
      {"left_friction_wheel_motor", kLeftFrictionWheel, true},
      {"right_first_leg_motor", kRightFirstLeg, false},
      {"right_second_leg_motor", kRightSecondLeg, false},
      {"r_gripper_left_drive_motor", kRightRod, false},
      {"right_friction_wheel_motor", kRightFrictionWheel, true},
  };

  for (const auto &spec : actuator_specs) {
    hardware_interface::ActuatorStateHandle state_handle(   
        spec.name, &angle_[spec.index], &vel_[spec.index],
        &effort_[spec.index]);

    act_state_interface_.registerHandle(state_handle);

    hardware_interface::ActuatorHandle cmd_handle(
        act_state_interface_.getHandle(spec.name), &cmd_[spec.index]);

    if (spec.velocity_interface) {
      velocity_act_interface_.registerHandle(cmd_handle);   
    } 
    else {
      position_act_interface_.registerHandle(cmd_handle);
    }
  }

  registerInterface(&act_state_interface_);
  registerInterface(&position_act_interface_);
  registerInterface(&velocity_act_interface_);

  // 占位 IMU 数据，后续由真实传感器填充
  hardware_interface::ImuSensorHandle imu_handle(
      "base_imu", "base_imu", imu_orientation_.data(),
      imu_orientation_covariance_.data(), imu_angular_velocity_.data(),
      imu_angular_velocity_covariance_.data(), imu_linear_acceleration_.data(),
      imu_linear_acceleration_covariance_.data());
  imu_sensor_interface_.registerHandle(imu_handle);
  registerInterface(&imu_sensor_interface_);
  registerInterface(&robot_state_interface_);

  // interface for auto
  AutoStateHandle auto_handle("auto_state", &auto_state_data_);
  auto_state_interface_.registerHandle(auto_handle);
  registerInterface(&auto_state_interface_);
}

bool StRobotHW::loadProtocolConfig(ros::NodeHandle &root_nh) {
  XmlRpc::XmlRpcValue id_map;
  if (!root_nh.getParam("/steering_engine_hw/protocol/id_map", id_map)) {
    ROS_ERROR("Missing steering_engine_hw/protocol/id_map");
    return false;
  }
  if (id_map.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR("protocol/id_map should be a map");
    return false;
  }

  joint_to_actuator_.clear();
  joint_to_actuator_["left_first_leg_joint"] = kLeftFirstLeg;
  joint_to_actuator_["left_second_leg_joint"] = kLeftSecondLeg;
  joint_to_actuator_["l_gripper_left_drive_joint"] = kLeftRod;
  joint_to_actuator_["left_friction_wheel_joint"] = kLeftFrictionWheel;
  joint_to_actuator_["right_first_leg_joint"] = kRightFirstLeg;
  joint_to_actuator_["right_second_leg_joint"] = kRightSecondLeg;
  joint_to_actuator_["r_gripper_left_drive_joint"] = kRightRod;
  joint_to_actuator_["right_friction_wheel_joint"] = kRightFrictionWheel;

  for (const auto &pair : joint_to_actuator_) {
    if (!id_map.hasMember(pair.first)) {
      ROS_ERROR("protocol/id_map missing joint: %s", pair.first.c_str());
      return false;
    }
    if (id_map[pair.first].getType() != XmlRpc::XmlRpcValue::TypeInt) {
      ROS_ERROR("protocol/id_map value must be int for joint: %s",
                pair.first.c_str());
      return false;
    }
    const int id = static_cast<int>(id_map[pair.first]);
    actuator_id_map_[pair.second] = id;
    id_to_actuator_[id] = pair.second;
  }

  XmlRpc::XmlRpcValue control_map;
  if (!root_nh.getParam("/steering_engine_hw/protocol/control_map",
                        control_map)) {
    ROS_ERROR("Missing steering_engine_hw/protocol/control_map");
    return false;
  }
  if (control_map.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR("protocol/control_map should be a map");
    return false;
  }

  auto loadJointList =
      [&](const std::string &key, size_t expected,
          std::vector<ActuatorIndex> &out_indices) -> bool {
    if (!control_map.hasMember(key)) {
      ROS_ERROR("protocol/control_map missing key: %s", key.c_str());
      return false;
    }
    XmlRpc::XmlRpcValue list = control_map[key];
    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_ERROR("protocol/control_map %s should be an array", key.c_str());
      return false;
    }
    if (list.size() != static_cast<int>(expected)) {
      ROS_ERROR("protocol/control_map %s size mismatch", key.c_str());
      return false;
    }
    out_indices.clear();
    out_indices.reserve(expected);
    for (int i = 0; i < list.size(); ++i) {
      if (list[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR("protocol/control_map %s entry must be string", key.c_str());
        return false;
      }
      const std::string joint_name =
          static_cast<std::string>(list[i]);
      auto it = joint_to_actuator_.find(joint_name);
      if (it == joint_to_actuator_.end()) {
        ROS_ERROR("Unknown joint name in %s: %s", key.c_str(),
                  joint_name.c_str());
        return false;
      }
      out_indices.push_back(it->second);
    }
    return true;
  };

  std::vector<ActuatorIndex> wheel_speed;
  std::vector<ActuatorIndex> claw_speed;
  std::vector<ActuatorIndex> claw_angle;
  std::vector<ActuatorIndex> joint_angle;
  if (!loadJointList("wheel_speed_targets", 2, wheel_speed) ||
      !loadJointList("claw_speed_targets", 2, claw_speed) ||
      !loadJointList("claw_angle_targets", 2, claw_angle) ||
      !loadJointList("joint_angle_targets", 4, joint_angle)) {
    return false;
  }

  control_map_.wheel_speed = {wheel_speed[0], wheel_speed[1]};
  control_map_.claw_speed = {claw_speed[0], claw_speed[1]};
  control_map_.claw_angle = {claw_angle[0], claw_angle[1]};
  control_map_.joint_angle = {joint_angle[0], joint_angle[1], joint_angle[2],
                              joint_angle[3]};

  root_nh.param<std::string>("/steering_engine_hw/protocol/topics/joint_speed_target",
                             protocol_topics_.joint_speed_target,
                             "/joint_speed_target");
  root_nh.param<std::string>("/steering_engine_hw/protocol/topics/claw_speed_target",
                             protocol_topics_.claw_speed_target,
                             "/claw_speed_target");

  joint_speed_sub_ = root_nh.subscribe(protocol_topics_.joint_speed_target, 1,
                                       &StRobotHW::jointSpeedTargetCallback,
                                       this);
  claw_speed_sub_ = root_nh.subscribe(protocol_topics_.claw_speed_target, 1,
                                      &StRobotHW::clawSpeedTargetCallback,
                                      this);

  return true;
}

void StRobotHW::jointSpeedTargetCallback(
    const std_msgs::Float64::ConstPtr &msg) {
  if (!msg) {
    return;
  }
  joint_speed_target_ = msg->data;
}

void StRobotHW::clawSpeedTargetCallback(
    const std_msgs::Float64MultiArray::ConstPtr &msg) {
  if (!msg || msg->data.size() < 2) {
    return;
  }
  claw_speed_target_[0] = msg->data[0];
  claw_speed_target_[1] = msg->data[1];
}

void StRobotHW::setKDLSegment() {
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(*urdf_model_, tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
  }
  addChildren(tree.getRootSegment());
}

void StRobotHW::addChildren(const KDL::SegmentMap::const_iterator segment) {
  const std::string &root = GetTreeElementSegment(segment->second).getName();

  const std::vector<KDL::SegmentMap::const_iterator> &children =
      GetTreeElementChildren(segment->second);
  for (auto i : children) {
    const KDL::Segment &child = GetTreeElementSegment(i->second);
    SegmentPair s(GetTreeElementSegment(i->second), root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None) {
      if (urdf_model_->getJoint(child.getJoint().getName()) &&
          urdf_model_->getJoint(child.getJoint().getName())->type ==
              urdf::Joint::FLOATING) {
        ROS_INFO("Floating joint. Not adding segment from %s to %s. This TF "
                 "can not be published based on joint_states info",
                 root.c_str(), child.getName().c_str());
      } else {
        segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
        ROS_DEBUG("Adding fixed segment from %s to %s", root.c_str(),
                  child.getName().c_str());
      }
    } else {
      segments_.insert(make_pair(child.getJoint().getName(), s));
      ROS_DEBUG("Adding moving segment from %s to %s", root.c_str(),
                child.getName().c_str());
    }
    addChildren(i);
  }
}

void StRobotHW::pack(unsigned char *tx_buffer, unsigned char ctrl,
                     unsigned char *data) {
  memset(tx_buffer, 0, k_frame_length_);
  auto *frame = reinterpret_cast<SerialFrame *>(tx_buffer);

  // set header
  for (int i = 0; i < 2; i++) {
    frame->header_[i] = header[i];
  }
  // set control
  frame->ctrl_ = ctrl;
  // set data length
  frame->length_ = k_data_length_;
  // set data
  memcpy(frame->data_, data, k_data_length_);
  // set crc
  frame->crc_ = getCrc8(tx_buffer, k_header_length_ + k_ctrl_length_ +
                                       k_length_ + k_data_length_);
  // set ender
  for (int i = 0; i < 2; i++) {
    frame->ender_[i] = ender[i];
  }
}

void StRobotHW::unpack(std::vector<uint8_t> rx_buffer,const ros::Time &time) {
  // check header and ender
  if (rx_buffer[0] != header[0] || rx_buffer[1] != header[1]) {
    return;
  }

  const uint8_t ctrl = rx_buffer[k_header_length_];
  if (ctrl != control_code_) {
    ROS_WARN_THROTTLE(10, "Received message ctrl %u is unexpected", ctrl);
    return;
  }

  const uint8_t length = rx_buffer[k_header_length_ + k_ctrl_length_];
  const size_t payload_start = k_header_length_ + k_ctrl_length_ + k_length_;
  const size_t expected_size =
      payload_start + static_cast<size_t>(length) + k_crc_length_ +
      k_tail_length_;
  const size_t data_length = rx_buffer.size() - payload_start - k_crc_length_ - k_tail_length_;
  if (rx_buffer.size() < expected_size) {
    ROS_WARN_THROTTLE(10, "Received message length %zu is inconsistent",
                      rx_buffer.size());
    return;
  }
  if(data_length != length) {
    ROS_WARN_THROTTLE(10, "Received message data length %zu does not match length bit %u",
                      data_length, length);
    return;
  }
  if (rx_buffer[expected_size - 2] != ender[0] ||
      rx_buffer[expected_size - 1] != ender[1]) {
    ROS_WARN("Received message ender error! Want: %x %x Real: %x %x", ender[0],
             ender[1], rx_buffer[expected_size - 2],
             rx_buffer[expected_size - 1]);
    return;
  }

  const size_t expected_crc_index =
      payload_start + static_cast<size_t>(length);

  if (rx_buffer[expected_crc_index] !=
      getCrc8(static_cast<unsigned char *>(&rx_buffer[0]),
              k_header_length_ + k_ctrl_length_ + k_length_ + length)) {
    ROS_WARN("Received message crc check error! Want: %02x Real: %02x",
             getCrc8(static_cast<unsigned char *>(&rx_buffer[0]),
                     k_header_length_ + k_ctrl_length_ + k_length_ + length),
             rx_buffer[expected_crc_index]);
    return;
  }

  auto unpackFloat = [&rx_buffer](size_t &offset) -> float {
    float value = 0.0f;
    std::memcpy(&value, &rx_buffer[offset], sizeof(float));
    offset += sizeof(float);
    return value;
  };

  constexpr size_t kImuFloatCount = 10;
  constexpr size_t kImuPayloadSize = kImuFloatCount * sizeof(float);
  constexpr size_t kStatusPayloadSize = 2;
  constexpr size_t kMotorIdLength = 1;
  const size_t entry_size = kMotorIdLength + 3 * sizeof(float);

  const size_t motor_payload_size = static_cast<size_t>(length) - kImuPayloadSize - kStatusPayloadSize;
  if (motor_payload_size % entry_size != 0) {
    ROS_WARN_THROTTLE(10, "Received message data length %u is invalid", length);
    return;
  }

  const size_t motor_count = motor_payload_size / entry_size;
  size_t index = payload_start;
  for (size_t i = 0; i < motor_count; ++i) {
    const int id = rx_buffer[index++];
    const float pos = unpackFloat(index);
    const float vel = unpackFloat(index);
    const float tor = unpackFloat(index);

    auto it = id_to_actuator_.find(id);
    if (it == id_to_actuator_.end()) {
      continue;
    }

    ActuatorIndex actuator_index = it->second;
    angle_[actuator_index]  = static_cast<double>(pos) - offset_vector_[actuator_index];
    vel_[actuator_index]    = static_cast<double>(vel);
    effort_[actuator_index] = static_cast<double>(tor);
  }

  if (index + kImuPayloadSize > payload_start + static_cast<size_t>(length)) {
    ROS_WARN_THROTTLE(10, "Received IMU payload is incomplete");
    return;
  }

  const double acc_x = static_cast<double>(unpackFloat(index));
  const double acc_y = static_cast<double>(unpackFloat(index));
  const double acc_z = static_cast<double>(unpackFloat(index));

  const double gyro_x = static_cast<double>(unpackFloat(index));
  const double gyro_y = static_cast<double>(unpackFloat(index));
  const double gyro_z = static_cast<double>(unpackFloat(index));

  const double qw = static_cast<double>(unpackFloat(index));
  const double qx = static_cast<double>(unpackFloat(index));
  const double qy = static_cast<double>(unpackFloat(index));
  const double qz = static_cast<double>(unpackFloat(index));

  updateImuState(acc_x, acc_y, acc_z,
                 gyro_x, gyro_y, gyro_z,
                 qw, qx, qy, qz);

  if(index + kStatusPayloadSize > payload_start + static_cast<size_t>(length)) {
    ROS_WARN_THROTTLE(10, "Received status payload is incomplete");
    return;
  }

  const uint8_t motor_fault     = rx_buffer[index++];
  const uint8_t grip_confirmed  = rx_buffer[index++];

  constexpr size_t joint_motor_fault_bit = 4;
  constexpr size_t wheel_motor_fault_bit = 2;
  constexpr size_t grip_motor_fault_bit  = 2;

  auto_state_data_.joint_fault = 0;
  for (size_t i = 0; i < joint_motor_fault_bit; ++i)
  {
    if (!(motor_fault & (1U << i)))
    {
      auto_state_data_.joint_fault = 1;
      break;
    }
  }

  auto_state_data_.grip_fault = 0;
  for (size_t i = 0; i < grip_motor_fault_bit; ++i)
  {
    size_t bit_pos = i + joint_motor_fault_bit + wheel_motor_fault_bit;
    if (!(motor_fault & (1U << bit_pos)))
    {
      auto_state_data_.grip_fault = 1;
      break;
    }
  }
  auto_state_data_.grip_confirmed = grip_confirmed;

  last_rx_time_ = time;
}

void StRobotHW::processRxBuffer(const ros::Time& time) {
  const size_t min_frame_length =
      k_header_length_ + k_ctrl_length_ + k_length_ + k_crc_length_ +
      k_tail_length_;
  const size_t payload_start = k_header_length_ + k_ctrl_length_ + k_length_;

  while (rx_buffer_.size() >= min_frame_length) {
    if (rx_buffer_[0] != header[0] || rx_buffer_[1] != header[1]) {
      bool found = false;
      size_t header_pos = 0;
      for (size_t i = 1; i + 1 < rx_buffer_.size(); ++i) {
        if (rx_buffer_[i] == header[0] && rx_buffer_[i + 1] == header[1]) {
          header_pos = i;
          found = true;
          break;
        }
      }
      if (!found) {
        rx_buffer_.clear();
        return;
      }
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + header_pos);
      if (rx_buffer_.size() < min_frame_length) {
        return;
      }
    }

    if (rx_buffer_.size() < payload_start) {
      return;
    }

    const uint8_t length = rx_buffer_[k_header_length_ + k_ctrl_length_];
    const size_t expected_size =
        payload_start + static_cast<size_t>(length) + k_crc_length_ +
        k_tail_length_;
    if (expected_size < min_frame_length) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }
    if (rx_buffer_.size() < expected_size) {
      return;
    }

    const uint8_t ctrl = rx_buffer_[k_header_length_];
    if (ctrl != control_code_) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }
    if (rx_buffer_[expected_size - 2] != ender[0] ||
        rx_buffer_[expected_size - 1] != ender[1]) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    const size_t expected_crc_index =
        payload_start + static_cast<size_t>(length);
    const unsigned char crc = getCrc8(
        static_cast<unsigned char *>(&rx_buffer_[0]),
        k_header_length_ + k_ctrl_length_ + k_length_ + length);
    if (rx_buffer_[expected_crc_index] != crc) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    std::vector<uint8_t> frame(rx_buffer_.begin(),
                               rx_buffer_.begin() + expected_size);
    unpack(frame,time);
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + expected_size);
  }
}

void StRobotHW::updateImuState(double acc_x, double acc_y, double acc_z,
                               double gyro_x, double gyro_y, double gyro_z,
                               double qw, double qx, double qy, double qz) {
  imu_linear_acceleration_[0] = acc_x;
  imu_linear_acceleration_[1] = acc_y;
  imu_linear_acceleration_[2] = acc_z;

  imu_angular_velocity_[0] = gyro_x;
  imu_angular_velocity_[1] = gyro_y;
  imu_angular_velocity_[2] = gyro_z;

  tf2::Quaternion q(qx, qy, qz, qw);

  const double norm2 = q.length2();
  if (!std::isfinite(norm2) || norm2 < 1e-12) {
    ROS_WARN_THROTTLE(1.0, "Received invalid IMU quaternion, fallback to RPY");
  }

  q.normalize();

  // Maintain quaternion sign continuity to avoid q / -q jitter
  const double dot = q.x() * imu_orientation_[0] +
                     q.y() * imu_orientation_[1] +
                     q.z() * imu_orientation_[2] +
                     q.w() * imu_orientation_[3];
  if (dot < 0.0) {
    q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
  }

  imu_orientation_[0] = q.x();
  imu_orientation_[1] = q.y();
  imu_orientation_[2] = q.z();
  imu_orientation_[3] = q.w();
}

bool StRobotHW::initAutoStateData(AutoStateData &data) {
  data.lower_alive = false;
  data.grip_confirmed = false;
  data.joint_fault = false;
  data.grip_fault = false;
  return true;
}

void StRobotHW::tryReconnectSerial(const ros::Time& time) {
  static ros::Time last_reconnect_attempt;

  if (serial_.isOpen()) {
    return;
  }

  if (!last_reconnect_attempt.isZero() &&
      (time - last_reconnect_attempt).toSec() < 1.0) {
    return;
  }

  last_reconnect_attempt = time;

  try {
    serial_.open();
    ROS_INFO_STREAM("Serial reconnected: " << serial_.getPort());
  } catch (const serial::IOException& e) {
    ROS_WARN_THROTTLE(2.0, "Serial reconnect failed: %s", e.what());
  }
}

void StRobotHW::handleSerialIoError(const std::string& context,
                                    const serial::IOException& e) {
  rx_buffer_.clear();

  ROS_ERROR_STREAM_THROTTLE(0.5, context << " : " << e.what());

  if (!serial_.isOpen()) {
    return;
  }

  try {
    serial_.close();
  } 
  catch (const serial::IOException& close_error) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "Serial close() after " << context
             << " : " << close_error.what());
  }
}

} // namespace steering_engine_hw
