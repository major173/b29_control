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
    act_to_jnt_state_interface_->propagate();
  } else {
    return;
  }
  clearRxBuffer();
}

void StRobotHW::write(const ros::Time &time, const ros::Duration &period) {
  static uint8_t last_send_data[12];
  uint8_t ctrl = 0xc0;
  uint8_t data[12] = {0};

  jnt_to_act_position_interface_
      ->propagate(); // now cmd[]'s data will be changed, pack
                     // cmd[] and send it.

  for (int i = 1; i < 4; i++) {
    *(data + i) = static_cast<uint8_t>(
        cmd_[i] + offset_vector_[i]); // set offset, ugly and unfetchable code
  }
  cmd_[3] = cmd_[3] + cmd_[2];

  if (memcmp(data, last_send_data, 12) != 0) {
    ROS_INFO("send rotation_baselink:%d  middle_rotation:%d left3_middle:%d",
             data[1], data[2], data[3]);
    pack(tx_buffer_, ctrl, data);
    tx_len_ = sizeof(tx_buffer_);
    try {
      serial_.write(tx_buffer_, tx_len_);
    } catch (serial::PortNotOpenedException &e) {
      ROS_ERROR_STREAM("Failed to usart data. " << e.what());
    }
    memcpy(last_send_data, data, 12);
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

  auto position_joint_interface =
      this->get<hardware_interface::PositionJointInterface>();
  std::vector<std::string> p_j_names = position_joint_interface->getNames();
  auto joint_state_interface =
      this->get<hardware_interface::JointStateInterface>();
  std::vector<std::string> j_s_names = joint_state_interface->getNames();

  for (const auto &name : p_j_names) {
    ROS_INFO("position_joint_handle: %s", name.c_str());
    position_joint_handles_.push_back(
        position_joint_interface->getHandle(name));
  }
  for (const auto &name : j_s_names) {
    ROS_INFO("joint_state_handle: %s", name.c_str());
    joint_state_handles_.push_back(joint_state_interface->getHandle(name));
    joint_states_segment_.insert(
        std::make_pair(name, joint_state_interface->getHandle(name)));
  }

  return true;
}

void StRobotHW::setInterface() {
  //  hardware_interface::JointStateHandle rotation_baselink_joint_state_handle(
  //      "rotation_baselink_joint", &angle_[0], &vel_[1], &effort_[1]);
  //  joint_state_interface_.registerHandle(rotation_baselink_joint_state_handle);
  //  hardware_interface::JointHandle joint_state_handle(
  //      joint_state_interface_.getHandle("rotation_baselink_joint"),
  //      &cmd_[1]);
  //  position_joint_interface_.registerHandle(joint_state_handle);
  hardware_interface::ActuatorStateHandle rotation_baselink_act_state(
      "rotation_baselink_joint_motor", &angle_[1], &vel_[1], &effort_[1]);
  act_state_interface_.registerHandle(rotation_baselink_act_state);
  hardware_interface::ActuatorHandle rotation_baselink_act_handle(
      act_state_interface_.getHandle("rotation_baselink_joint_motor"),
      &cmd_[1]);
  position_act_interface_.registerHandle(rotation_baselink_act_handle);

  hardware_interface::ActuatorStateHandle middle_rotation_act_state(
      "middle_rotation_joint_motor", &angle_[2], &vel_[2], &effort_[2]);
  act_state_interface_.registerHandle(middle_rotation_act_state);
  hardware_interface::ActuatorHandle middle_rotation_act_handle(
      act_state_interface_.getHandle("middle_rotation_joint_motor"), &cmd_[2]);
  position_act_interface_.registerHandle(middle_rotation_act_handle);

  hardware_interface::ActuatorStateHandle left3_middle_act_state(
      "left3_middle_joint_motor", &angle_[3], &vel_[3], &effort_[3]);
  act_state_interface_.registerHandle(left3_middle_act_state);
  hardware_interface::ActuatorHandle left3_middle_act_handle(
      act_state_interface_.getHandle("left3_middle_joint_motor"), &cmd_[3]);
  position_act_interface_.registerHandle(left3_middle_act_handle);

  // registerInterface(&joint_state_interface_);
  // registerInterface(&position_joint_interface_);
  registerInterface(&act_state_interface_);
  registerInterface(&position_act_interface_);
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
  uint8_t ctrl, length;

  // check header and ender
  if (rx_buffer[0] != header[0] || rx_buffer[1] != header[1]) // buf[0] buf[1]
  {
    return;
  } else if (rx_buffer[17] != ender[0] || rx_buffer[18] != ender[1]) {
    ROS_WARN("Received message ender error! Want: %x %x Real: %x %x", ender[0],
             ender[1], rx_buffer[17], rx_buffer[18]);
    return;
  }

  // read ctrl
  ctrl = rx_buffer_[2]; // buf[2]
  // read len
  length = rx_buffer[3]; // buf[3]
  // check crc
  if (rx_buffer[16] !=
      getCrc8(static_cast<unsigned char *>(&rx_buffer[0]), 4 + length)) {
    ROS_WARN("Received message crc check error! Want: %02x Real: %02x",
             getCrc8(static_cast<unsigned char *>(&rx_buffer[0]), 4 + length),
             rx_buffer[16]);
    return;
  }
  // read data
  for (int i = 0; i < 5; i++) {
    angle_[i] = static_cast<double>(rx_buffer[4 + i]);
    effort_[i] = 0.;
    vel_[i] = 0.;
  }
  // set offset, ugly and unfetchable code
  for (int i = 0; i < 4; i++) {
    angle_[i] -= offset_vector_[i - 1];
  }
}
} // namespace steering_engine_hw