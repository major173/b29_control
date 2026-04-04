//
// Created by yuchen on 2023/1/27.
//

#include "steering_engine/common/hardware_interface.h"

#include <cmath>
#include <cstring>

namespace steering_engine_hw {
bool StRobotHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
  //**series**//

  serial::Timeout to = serial::Timeout::simpleTimeout(100); //创建timeout
  serial::parity_t pt = serial::parity_t::parity_none; //创建校验位为0位
  serial::bytesize_t bt = serial::bytesize_t::eightbits; //创建发送字节数为8位
  serial::flowcontrol_t ft =
      serial::flowcontrol_t::flowcontrol_none; //创建数据流控制，不使用
  serial::stopbits_t st = serial::stopbits_t::stopbits_one; //创建终止位为1位

  std::string port_name = "/dev/usbSteering";
  int baudrate = 115200;
  root_nh.getParam("/steering_engine_hw/serial/port", port_name);
  root_nh.getParam("/steering_engine_hw/serial/baudrate", baudrate);
  serial_.setPort(port_name);
  ROS_INFO("%s", port_name.c_str());
  serial_.setBaudrate(baudrate);
  serial_.setParity(pt);      //设置校验位
  serial_.setBytesize(bt);    //设置发送字节数
  serial_.setFlowcontrol(ft); //设置数据流控制
  serial_.setStopbits(st);    //设置终止位
  serial_.setTimeout(to);
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
  if (serial_.available()) {
    rx_len_ = static_cast<int>(serial_.available());
    std::vector<uint8_t> incoming;
    const size_t bytes_read = serial_.read(incoming, static_cast<size_t>(rx_len_));
    if (bytes_read == 0) {
      return;
    }
    incoming.resize(bytes_read);
    rx_buffer_.insert(rx_buffer_.end(), incoming.begin(), incoming.end());
    processRxBuffer();
    if (act_to_jnt_state_interface_) {
      act_to_jnt_state_interface_->propagate();
    }
  } else {
    return;
  }
}

void StRobotHW::write(const ros::Time &time, const ros::Duration &period) {
  static std::array<uint8_t, k_data_length_> last_send_data{};
  std::array<uint8_t, k_data_length_> data{};

  if (jnt_to_act_position_interface_) {
    jnt_to_act_position_interface_->propagate();
  }
  if (jnt_to_act_velocity_interface_) {
    jnt_to_act_velocity_interface_->propagate();
  }

  const double wheel_speed_left = cmd_[control_map_.wheel_speed[0]];
  const double wheel_speed_right = cmd_[control_map_.wheel_speed[1]];
  const double claw_speed_left = claw_speed_target_[0];
  const double claw_speed_right = claw_speed_target_[1];
  const double claw_angle_left = cmd_[control_map_.claw_angle[0]];
  const double claw_angle_right = cmd_[control_map_.claw_angle[1]];
  const double joint_speed_target = joint_speed_target_;
  const double joint_angle_targets[4] = {
      cmd_[control_map_.joint_angle[0]],
      cmd_[control_map_.joint_angle[1]],
      cmd_[control_map_.joint_angle[2]],
      cmd_[control_map_.joint_angle[3]],
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

  if (true) {
    pack(tx_buffer_, control_code_, data.data());
    tx_len_ = sizeof(tx_buffer_);
    try {
      serial_.write(tx_buffer_, tx_len_);
    } catch (serial::PortNotOpenedException &e) {
      ROS_ERROR_STREAM("Failed to usart data. " << e.what());
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

void StRobotHW::setInterface() {
  struct ActuatorSpec {
    const char *name;
    ActuatorIndex index;
    bool velocity_interface;
  };

  const ActuatorSpec actuator_specs[] = {
      {"left_first_leg_motor", kLeftFirstLeg, false},
      {"left_second_leg_motor", kLeftSecondLeg, false},
      {"left_rod_motor", kLeftRod, false},
      {"left_friction_wheel_motor", kLeftFrictionWheel, true},
      {"right_first_leg_motor", kRightFirstLeg, false},
      {"right_second_leg_motor", kRightSecondLeg, false},
      {"right_rod_motor", kRightRod, false},
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
    } else {
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
  joint_to_actuator_["left_rod_joint"] = kLeftRod;
  joint_to_actuator_["left_friction_wheel_joint"] = kLeftFrictionWheel;
  joint_to_actuator_["right_first_leg_joint"] = kRightFirstLeg;
  joint_to_actuator_["right_second_leg_joint"] = kRightSecondLeg;
  joint_to_actuator_["right_rod_joint"] = kRightRod;
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

void StRobotHW::updateTf(const ros::Time &time) {
  //  std::vector<std::string> link_names;
  //  for (const auto &link : urdf_model_->links_) {
  //    link_names.push_back(link.first);

  std::vector<geometry_msgs::TransformStamped> tf_transforms;
  geometry_msgs::TransformStamped tf_transform;
  // Loop over all float segments
  for (auto &item : segments_) {
    auto jnt_iter = joint_states_segment_.find(item.first);
    if (jnt_iter != joint_states_segment_.end())
      tf_transform = tf2::kdlToTransform(
          item.second.segment.pose(jnt_iter->second.getPosition()));
    else {
      ROS_WARN_THROTTLE(
          10,
          "Joint state with name: \"%s\" was received but not found in URDF",
          item.first.c_str());
      continue;
    }
    tf_transform.header.stamp = time;
    tf_transform.header.frame_id = stripSlash(item.second.root);
    tf_transform.child_frame_id = stripSlash(item.second.tip);
    tf_transforms.push_back(tf_transform);
  }
  tf_broadcaster_.sendTransform(tf_transforms);

  tf_transforms.clear();
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
  // ser ender
  for (int i = 0; i < 2; i++) {
    frame->ender_[i] = ender[i];
  }
}

void StRobotHW::unpack(std::vector<uint8_t> rx_buffer) {
  const size_t min_frame_length =
      k_header_length_ + k_ctrl_length_ + k_length_ + k_crc_length_ +
      k_tail_length_;
  if (rx_buffer.size() < min_frame_length) {
    ROS_WARN_THROTTLE(10, "Received message length %zu is too short",
                      rx_buffer.size());
    return;
  }

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
  if (rx_buffer.size() < expected_size) {
    ROS_WARN_THROTTLE(10, "Received message length %zu is inconsistent",
                      rx_buffer.size());
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

  const size_t entry_size = 1 + 4 + 4 + 4;
  if (length % entry_size != 0) {
    ROS_WARN_THROTTLE(10, "Received message data length %u is invalid", length);
    return;
  }

  const size_t motor_count = length / entry_size;
  size_t index = payload_start;
  for (size_t i = 0; i < motor_count; ++i) {
    const int id = rx_buffer[index++];
    auto it = id_to_actuator_.find(id);
    if (it == id_to_actuator_.end()) {
      index += 12;
      continue;
    }

    ActuatorIndex actuator_index = it->second;
    float pos = 0.0f;
    float vel = 0.0f;
    float tor = 0.0f;
    std::memcpy(&pos, &rx_buffer[index], sizeof(float));
    index += sizeof(float);
    std::memcpy(&vel, &rx_buffer[index], sizeof(float));
    index += sizeof(float);
    std::memcpy(&tor, &rx_buffer[index], sizeof(float));
    index += sizeof(float);

    angle_[actuator_index] = static_cast<double>(pos) - offset_vector_[actuator_index];
    vel_[actuator_index] = static_cast<double>(vel);
    effort_[actuator_index] = static_cast<double>(tor);
  }
}

void StRobotHW::processRxBuffer() {
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
    unpack(frame);
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + expected_size);
  }
}
} // namespace steering_engine_hw
