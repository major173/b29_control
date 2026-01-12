//
// Created by yuchen on 2023/1/27.
//

#include "steering_engine/common/hardware_interface.h"

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
  serial_.setPort(port_name);
  ROS_INFO("%s", port_name.c_str());
  serial_.setBaudrate(115200);
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

  if (serial_.isOpen())
    return true;
  try {
    serial_.open();
    return true;
  } catch (serial::IOException &e) {
    ROS_ERROR("Cannot open Steering port");
    return false;
  }
}

void StRobotHW::read(const ros::Time &time, const ros::Duration &period) {
  if (serial_.available()) {
    rx_len_ = static_cast<int>(serial_.available());
    serial_.read(rx_buffer_, rx_len_);

    unpack(rx_buffer_);
    if (act_to_jnt_state_interface_) {
      act_to_jnt_state_interface_->propagate();
    }
  } else {
    return;
  }
  clearRxBuffer();
}

void StRobotHW::write(const ros::Time &time, const ros::Duration &period) {
  static std::array<uint8_t, 12> last_send_data{};
  const uint8_t ctrl = 0xc0;
  std::array<uint8_t, 12> data{};

  if (jnt_to_act_position_interface_) {
    jnt_to_act_position_interface_->propagate();
  }
  if (jnt_to_act_velocity_interface_) {
    jnt_to_act_velocity_interface_->propagate();
  }

  const size_t payload_count =
      static_cast<size_t>(kActuatorCount) <
              static_cast<size_t>(k_data_length_)
          ? static_cast<size_t>(kActuatorCount)
          : static_cast<size_t>(k_data_length_);

  for (size_t i = 0; i < payload_count; ++i) {
    data[i] = static_cast<uint8_t>(cmd_[i] + offset_vector_[i]);
  }

  if (memcmp(data.data(), last_send_data.data(), data.size()) != 0) {
    pack(tx_buffer_, ctrl, data.data());
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
  if (rx_buffer.size() < static_cast<size_t>(k_frame_length_)) {
    ROS_WARN_THROTTLE(10, "Received message length %zu is too short",
                      rx_buffer.size());
    return;
  }

  // check header and ender
  if (rx_buffer[0] != header[0] || rx_buffer[1] != header[1]) {
    return;
  }
  if (rx_buffer[k_frame_length_ - 2] != ender[0] ||
      rx_buffer[k_frame_length_ - 1] != ender[1]) {
    ROS_WARN("Received message ender error! Want: %x %x Real: %x %x", ender[0],
             ender[1], rx_buffer[k_frame_length_ - 2],
             rx_buffer[k_frame_length_ - 1]);
    return;
  }

  const uint8_t length = rx_buffer[k_header_length_ + k_ctrl_length_];
  if (length != k_data_length_) {
    ROS_WARN_THROTTLE(10, "Received message data length %u is unexpected",
                      length);
    return;
  }

  const size_t crc_index =
      k_header_length_ + k_ctrl_length_ + k_length_ + length;
  if (rx_buffer[crc_index] !=
      getCrc8(static_cast<unsigned char *>(&rx_buffer[0]),
              k_header_length_ + k_ctrl_length_ + k_length_ + length)) {
    ROS_WARN("Received message crc check error! Want: %02x Real: %02x",
             getCrc8(static_cast<unsigned char *>(&rx_buffer[0]),
                     k_header_length_ + k_ctrl_length_ + k_length_ + length),
             rx_buffer[crc_index]);
    return;
  }

  const size_t payload_start = k_header_length_ + k_ctrl_length_ + k_length_;
  const size_t payload_count =
      static_cast<size_t>(kActuatorCount) < static_cast<size_t>(length)
          ? static_cast<size_t>(kActuatorCount)
          : static_cast<size_t>(length);

  for (size_t i = 0; i < payload_count; ++i) {
    angle_[i] = static_cast<double>(rx_buffer[payload_start + i]) -
                offset_vector_[i];
    effort_[i] = 0.;
    vel_[i] = 0.;
  }
}
} // namespace steering_engine_hw
