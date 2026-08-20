//
// Created by yuchen on 2023/1/27.
//

#include "steering_engine/common/hardware_interface.h"

#include <algorithm>
#include <cctype>
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
  int baudrate = 921600;
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
  if (!loadGravityCompensation(root_nh)) {
    ROS_ERROR("Failed to initialize gravity compensation");
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
  updateGravityCompensation(time, period);

  if (!serial_.isOpen()) {
    tryReconnectSerial(time);
    return;
  }
  
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
  std::array<double, k_torque_ff_count_> joint_torque_feedforward{};
  const uint8_t requested_gravity_mode =
      auto_state_data_.gravity_compensation_mode;
  if (gravity_controller_mode_enabled_) {
    // The named leg is the moving leg, so the opposite side is the support.
    if (requested_gravity_mode == 1u) {
      support_side_.store(static_cast<std::uint8_t>(SupportSide::RIGHT));
    } else if (requested_gravity_mode == 2u) {
      support_side_.store(static_cast<std::uint8_t>(SupportSide::LEFT));
    }
  }
  const bool torque_ff_valid =
      gravity_transmit_enabled_ && gravity_feedforward_valid_ &&
      (!gravity_controller_mode_enabled_ || requested_gravity_mode != 0u);
  if (torque_ff_valid) {
    for (std::size_t i = 0; i < joint_torque_feedforward.size(); ++i) {
      joint_torque_feedforward[i] = tau_gravity_[i];
    }
  }

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

  pack(tx_buffer_, data.data(), joint_torque_feedforward,
       torque_ff_valid);
  tx_len_ = sizeof(tx_buffer_);
  try {
    serial_.write(tx_buffer_, tx_len_);
  }
  catch (const serial::IOException& e) {
    handleSerialIoError("Serial write() failed", e);
    return;
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

  RemoteControlHandle remote_control_handle("remote_control",
                                             &remote_control_data_);
  remote_control_interface_.registerHandle(remote_control_handle);
  registerInterface(&remote_control_interface_);
}

bool StRobotHW::loadProtocolConfig(ros::NodeHandle &root_nh) {
  int gravity_compensation_mode = 0;
  root_nh.param<int>(
      "/steering_engine_hw/protocol/gravity_compensation_mode",
      gravity_compensation_mode, 0);
  if (gravity_compensation_mode < 0 || gravity_compensation_mode > 2) {
    ROS_ERROR("protocol/gravity_compensation_mode must be 0, 1, or 2");
    return false;
  }
  gravity_compensation_mode_ =
      static_cast<uint8_t>(gravity_compensation_mode);
  if (gravity_compensation_mode_ != 0u) {
    ROS_WARN("V2 control frames always send legacy gravity mode 0; "
             "host tau_ff is used instead");
  }

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

bool StRobotHW::loadGravityCompensation(ros::NodeHandle &root_nh) {
  const std::string param_ns = "/steering_engine_hw/gravity_compensation";
  if (!gravity_compensator_.init(*urdf_model_, root_nh, param_ns)) {
    return false;
  }
  if (!gravity_compensator_.enabled()) {
    return true;
  }

  root_nh.param(param_ns + "/transmit_enabled",
                gravity_transmit_enabled_, gravity_transmit_enabled_);
  root_nh.param(param_ns + "/controller_mode_enabled",
                gravity_controller_mode_enabled_,
                gravity_controller_mode_enabled_);
  XmlRpc::XmlRpcValue torque_scales;
  if (root_nh.getParam(param_ns + "/torque_scales", torque_scales)) {
    if (torque_scales.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        torque_scales.size() !=
            static_cast<int>(gravity_torque_scales_.size())) {
      ROS_ERROR_STREAM(param_ns << "/torque_scales must contain four numbers");
      return false;
    }
    for (int i = 0; i < torque_scales.size(); ++i) {
      double scale = 0.0;
      if (torque_scales[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        scale = static_cast<double>(torque_scales[i]);
      } else if (torque_scales[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeInt) {
        scale = static_cast<int>(torque_scales[i]);
      } else {
        ROS_ERROR_STREAM(param_ns << "/torque_scales[" << i
                                  << "] must be numeric");
        return false;
      }
      if (!std::isfinite(scale)) {
        ROS_ERROR_STREAM(param_ns << "/torque_scales[" << i
                                  << "] must be finite");
        return false;
      }
      gravity_torque_scales_[i] = scale;
    }
  }

  std::string initial_support_side = "LEFT";
  root_nh.param(param_ns + "/support_side", initial_support_side,
                initial_support_side);
  std::transform(initial_support_side.begin(), initial_support_side.end(),
                 initial_support_side.begin(), [](unsigned char value) {
                   return static_cast<char>(std::toupper(value));
                 });
  if (initial_support_side == "LEFT") {
    support_side_.store(static_cast<std::uint8_t>(SupportSide::LEFT));
  } else if (initial_support_side == "RIGHT") {
    support_side_.store(static_cast<std::uint8_t>(SupportSide::RIGHT));
  } else {
    ROS_ERROR_STREAM(param_ns << "/support_side must be LEFT or RIGHT");
    return false;
  }

  root_nh.param(param_ns + "/torque_slew_rate",
                gravity_torque_slew_rate_, gravity_torque_slew_rate_);
  root_nh.param(param_ns + "/publish_rate",
                gravity_publish_rate_, gravity_publish_rate_);
  if (!std::isfinite(gravity_torque_slew_rate_) ||
      gravity_torque_slew_rate_ <= 0.0 ||
      !std::isfinite(gravity_publish_rate_) || gravity_publish_rate_ <= 0.0) {
    ROS_ERROR("Gravity torque_slew_rate and publish_rate must be finite and positive");
    return false;
  }

  std::string support_side_topic = "/support_side";
  std::string torque_topic = "/gravity_compensation/torque_ff";
  root_nh.param(param_ns + "/support_side_topic", support_side_topic,
                support_side_topic);
  root_nh.param(param_ns + "/torque_topic", torque_topic, torque_topic);
  support_side_sub_ = root_nh.subscribe(support_side_topic, 1,
                                       &StRobotHW::supportSideCallback, this);
  gravity_torque_pub_ =
      root_nh.advertise<std_msgs::Float64MultiArray>(torque_topic, 1);

  ROS_INFO("Gravity feedforward support=%s, transmit=%s, input=%s, output=%s",
           initial_support_side.c_str(),
           gravity_transmit_enabled_ ? "enabled" : "disabled",
           support_side_topic.c_str(),
           torque_topic.c_str());
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

void StRobotHW::supportSideCallback(const std_msgs::String::ConstPtr &msg) {
  if (!msg) {
    return;
  }

  std::string side = msg->data;
  std::transform(side.begin(), side.end(), side.begin(),
                 [](unsigned char value) {
                   return static_cast<char>(std::toupper(value));
                 });
  if (side == "LEFT") {
    support_side_.store(static_cast<std::uint8_t>(SupportSide::LEFT));
  } else if (side == "RIGHT") {
    support_side_.store(static_cast<std::uint8_t>(SupportSide::RIGHT));
  } else {
    ROS_WARN_STREAM_THROTTLE(1.0, "Ignoring invalid support side: " << msg->data);
  }
}

void StRobotHW::updateGravityCompensation(const ros::Time &time,
                                          const ros::Duration &period) {
  gravity_feedforward_valid_ = false;
  if (!gravity_compensator_.enabled()) {
    tau_gravity_.fill(0.0);
    return;
  }

  constexpr double kFeedbackTimeoutSec = 0.2;
  if (last_rx_time_.isZero() ||
      (time - last_rx_time_).toSec() > kFeedbackTimeoutSec) {
    tau_gravity_.fill(0.0);
    return;
  }

  GravityCompensator::JointVector q_actual;
  const auto &joint_names = gravity_compensator_.jointNames();
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    const auto handle = joint_states_segment_.find(joint_names[i]);
    if (handle == joint_states_segment_.end()) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "Gravity compensation has no joint-state handle for "
                   << joint_names[i]);
      tau_gravity_.fill(0.0);
      return;
    }
    q_actual[i] = handle->second.getPosition();
    if (!std::isfinite(q_actual[i])) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "Gravity compensation received non-finite position for "
                   << joint_names[i]);
      tau_gravity_.fill(0.0);
      return;
    }
  }

  const SupportSide support =
      support_side_.load() == static_cast<std::uint8_t>(SupportSide::RIGHT)
          ? SupportSide::RIGHT
          : SupportSide::LEFT;

  // The lower controller reports an IMU online/ready bit in every feedback
  // frame. Only use the fused quaternion when both that bit and the local
  // quaternion sanity checks pass; otherwise keep the original
  // support-frame-is-vertical gravity model.
  const bool imu_usable = imu_orientation_valid_ && auto_state_data_.imu_ready;
  GravityCompensator::JointVector raw_torque;
  if (imu_usable) {
    const Eigen::Quaterniond base_imu_orientation(
        imu_orientation_[3], imu_orientation_[0], imu_orientation_[1],
        imu_orientation_[2]);
    const Eigen::Quaterniond support_world_orientation =
        gravity_compensator_.supportOrientationInWorld(
            q_actual, support, base_imu_orientation);
    raw_torque = gravity_compensator_.compute(q_actual, support,
                                              support_world_orientation);
  } else {
    raw_torque = gravity_compensator_.compute(q_actual, support);
  }
  ROS_INFO_THROTTLE(5.0,
                    "[gravity] imu_used=%s imu_online=%s quaternion_valid=%s",
                    imu_usable ? "true" : "false",
                    auto_state_data_.imu_ready ? "true" : "false",
                    imu_orientation_valid_ ? "true" : "false");
  const double max_step = gravity_torque_slew_rate_ *
                          std::max(0.0, period.toSec());
  for (std::size_t i = 0; i < tau_gravity_.size(); ++i) {
    const double scaled_torque = raw_torque[i] * gravity_torque_scales_[i];
    if (!std::isfinite(raw_torque[i]) || !std::isfinite(scaled_torque)) {
      ROS_ERROR_THROTTLE(1.0, "Gravity compensation produced non-finite torque");
      tau_gravity_.fill(0.0);
      return;
    }
    const double delta = scaled_torque - tau_gravity_[i];
    tau_gravity_[i] += std::max(-max_step, std::min(max_step, delta));
  }
  gravity_feedforward_valid_ = true;

  if (last_gravity_publish_time_.isZero() ||
      (time - last_gravity_publish_time_).toSec() >=
          1.0 / gravity_publish_rate_) {
    std_msgs::Float64MultiArray torque_message;
    torque_message.layout.dim.resize(1);
    torque_message.layout.dim[0].label =
        "left_first_leg_joint,left_second_leg_joint,"
        "right_first_leg_joint,right_second_leg_joint";
    torque_message.layout.dim[0].size = tau_gravity_.size();
    torque_message.layout.dim[0].stride = tau_gravity_.size();
    torque_message.data.assign(tau_gravity_.begin(), tau_gravity_.end());
    gravity_torque_pub_.publish(torque_message);
    last_gravity_publish_time_ = time;
  }
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

void StRobotHW::pack(unsigned char *tx_buffer, const unsigned char *data,
                     const std::array<double, 4> &torque_ff,
                     bool torque_ff_valid) {
  memset(tx_buffer, 0, k_frame_length_);
  auto *frame = reinterpret_cast<SerialFrame *>(tx_buffer);

  // set header
  for (int i = 0; i < 2; i++) {
    frame->header_[i] = header[i];
  }
  // set control
  frame->ctrl_ = tx_control_code_;
  // set data length
  frame->length_ = k_payload_length_;
  // set data
  memcpy(frame->data_, data, k_data_length_);
  // V2 host torque feedforward replaces the legacy lower-controller model.
  frame->legacy_gravity_compensation_mode_ = 0u;

  std::array<float, k_torque_ff_count_> packed_torque{};
  bool valid_group = torque_ff_valid;
  for (std::size_t i = 0; i < packed_torque.size(); ++i) {
    packed_torque[i] = static_cast<float>(torque_ff[i]);
    if (!std::isfinite(torque_ff[i]) || !std::isfinite(packed_torque[i])) {
      valid_group = false;
    }
  }
  frame->torque_flags_ = valid_group ? 0x01u : 0x00u;
  if (valid_group) {
    for (std::size_t i = 0; i < packed_torque.size(); ++i) {
      std::memcpy(frame->torque_ff_ + i * sizeof(float),
                  &packed_torque[i], sizeof(float));
    }
  }
  // set crc
  frame->crc_ = getCrc8(tx_buffer, k_header_length_ + k_ctrl_length_ +
                                       k_length_ + k_payload_length_);
  // set ender
  for (int i = 0; i < 2; i++) {
    frame->ender_[i] = ender[i];
  }
}

void StRobotHW::unpack(std::vector<uint8_t> rx_buffer,const ros::Time &time) {
  if (rx_buffer.size() != k_feedback_frame_length_) {
    ROS_WARN_THROTTLE(10, "Received message length %zu is invalid; expected %zu",
                      rx_buffer.size(), k_feedback_frame_length_);
    return;
  }

  // check header and ender
  if (rx_buffer[0] != header[0] || rx_buffer[1] != header[1]) {
    return;
  }

  const uint8_t ctrl = rx_buffer[k_header_length_];
  if (ctrl != feedback_control_code_) {
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

  struct MotorState {
    int id;
    float pos;
    float vel;
    float tor;
  };
  std::array<MotorState, k_feedback_motor_count_> motor_states{};
  std::array<double, k_feedback_remote_joint_count_> remote_joint_increment{};

  size_t index = payload_start;
  for (auto &motor_state : motor_states) {
    motor_state.id = rx_buffer[index++];
    motor_state.pos = unpackFloat(index);
    motor_state.vel = unpackFloat(index);
    motor_state.tor = unpackFloat(index);
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

  bool remote_control_increments_valid = true;
  for (double &increment : remote_joint_increment) {
    increment = static_cast<double>(unpackFloat(index));
    remote_control_increments_valid =
        remote_control_increments_valid && std::isfinite(increment);
  }

  const uint8_t remote_control_stage_complete = rx_buffer[index++];
  const uint8_t cruise_drive_request = rx_buffer[index++];
  const uint8_t auto_control_signals = rx_buffer[index++];
  const uint8_t motor_health = rx_buffer[index++];
  const uint8_t grip_confirmed = rx_buffer[index++];
  const uint8_t imu_ready = rx_buffer[index++];

  if (index != payload_start + k_feedback_payload_length_) {
    ROS_WARN_THROTTLE(10, "Received payload was not consumed completely");
    return;
  }

  const bool cruise_drive_request_valid = cruise_drive_request <= 2u;
  if (cruise_drive_request > 2u) {
    ROS_WARN_THROTTLE(1.0,
                      "Invalid cruise drive request %u; keeping wheels stopped",
                      cruise_drive_request);
  }
  if ((auto_control_signals & 0xf8u) != 0u) {
    ROS_WARN_THROTTLE(1.0,
                      "Reserved auto control signal bits must be zero: 0x%02x",
                      auto_control_signals);
  }

  bool joint_fault = false;
  for (size_t i = 0; i < 4; ++i) {
    if (!(motor_health & (1U << i))) {
      joint_fault = true;
      break;
    }
  }

  bool grip_fault = false;
  for (size_t i = 6; i < 8; ++i) {
    if (!(motor_health & (1U << i))) {
      grip_fault = true;
      break;
    }
  }

  for (const auto &motor_state : motor_states) {
    auto it = id_to_actuator_.find(motor_state.id);
    if (it == id_to_actuator_.end()) {
      continue;
    }

    const ActuatorIndex actuator_index = it->second;
    angle_[actuator_index] =
        static_cast<double>(motor_state.pos) - offset_vector_[actuator_index];
    vel_[actuator_index] = static_cast<double>(motor_state.vel);
    effort_[actuator_index] = static_cast<double>(motor_state.tor);
  }

  updateImuState(acc_x, acc_y, acc_z,
                 gyro_x, gyro_y, gyro_z,
                 qw, qx, qy, qz);

  const bool remote_control_complete = remote_control_stage_complete == 1u;
  remote_control_data_.header.stamp = time;
  remote_control_data_.joint_increments =
      remote_control_increments_valid
          ? remote_joint_increment
          : std::array<double, k_feedback_remote_joint_count_>{};
  remote_control_data_.stage_complete = remote_control_complete;
  remote_control_data_.increments_valid = remote_control_increments_valid;
  ++remote_control_data_.sample_sequence;
  if (!previous_remote_control_complete_ && remote_control_complete) {
    ++remote_control_data_.completion_rising_edge_sequence;
  }
  previous_remote_control_complete_ = remote_control_complete;
  if (!remote_control_increments_valid) {
    ROS_WARN_THROTTLE(1.0,
                      "Ignoring non-finite remote control joint increment");
  }

  const bool auto_start = (auto_control_signals & (1u << 0)) != 0u;
  const bool manual_reset = (auto_control_signals & (1u << 1)) != 0u;
  const bool obstacle_crossing_trigger =
      (auto_control_signals & (1u << 2)) != 0u;
  auto_state_data_.cruise_drive_request_raw = cruise_drive_request;
  auto_state_data_.cruise_drive_request_valid = cruise_drive_request_valid;
  auto_state_data_.auto_start = auto_start;
  auto_state_data_.manual_reset = manual_reset;
  auto_state_data_.obstacle_crossing_trigger = obstacle_crossing_trigger;
  if (!previous_auto_start_ && auto_start) {
    ++auto_state_data_.auto_start_rising_edge_sequence;
  }
  if (!previous_manual_reset_ && manual_reset) {
    ++auto_state_data_.manual_reset_rising_edge_sequence;
  }
  if (!previous_obstacle_crossing_trigger_ && obstacle_crossing_trigger) {
    ++auto_state_data_.obstacle_trigger_rising_edge_sequence;
  }
  if (previous_obstacle_crossing_trigger_ && !obstacle_crossing_trigger) {
    ++auto_state_data_.obstacle_trigger_falling_edge_sequence;
  }
  previous_auto_start_ = auto_start;
  previous_manual_reset_ = manual_reset;
  previous_obstacle_crossing_trigger_ = obstacle_crossing_trigger;
  auto_state_data_.joint_fault = joint_fault;
  auto_state_data_.grip_fault = grip_fault;
  auto_state_data_.grip_confirmed = grip_confirmed != 0;
  auto_state_data_.imu_ready = imu_ready != 0;
  auto_state_data_.header.stamp = time;
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
        const bool keep_header_prefix =
            !rx_buffer_.empty() && rx_buffer_.back() == header[0];
        if (keep_header_prefix) {
          rx_buffer_.assign(1, header[0]);
        } else {
          rx_buffer_.clear();
        }
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

    const uint8_t ctrl = rx_buffer_[k_header_length_];
    const uint8_t length = rx_buffer_[k_header_length_ + k_ctrl_length_];
    if (ctrl != feedback_control_code_ ||
        static_cast<size_t>(length) != k_feedback_payload_length_) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    const size_t expected_size = k_feedback_frame_length_;
    if (rx_buffer_.size() < expected_size) {
      return;
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

  if (!std::isfinite(qw) || !std::isfinite(qx) || !std::isfinite(qy) ||
      !std::isfinite(qz)) {
    imu_orientation_valid_ = false;
    ROS_WARN_THROTTLE(1.0,
                      "Received non-finite IMU quaternion; gravity falls back "
                      "to support-frame-vertical model");
    return;
  }

  tf2::Quaternion q(qx, qy, qz, qw);
  const double norm2 = q.length2();
  if (!std::isfinite(norm2) || norm2 < 1e-12) {
    imu_orientation_valid_ = false;
    ROS_WARN_THROTTLE(1.0,
                      "Received near-zero IMU quaternion norm; gravity falls "
                      "back to support-frame-vertical model");
    return;
  }

  q.normalize();

  // Maintain quaternion sign continuity to avoid q / -q jitter.
  if (imu_orientation_valid_) {
    const double dot = q.x() * imu_orientation_[0] +
                       q.y() * imu_orientation_[1] +
                       q.z() * imu_orientation_[2] +
                       q.w() * imu_orientation_[3];
    if (dot < 0.0) {
      q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
    }
  }

  imu_orientation_[0] = q.x();
  imu_orientation_[1] = q.y();
  imu_orientation_[2] = q.z();
  imu_orientation_[3] = q.w();
  imu_orientation_valid_ = true;
}

bool StRobotHW::initAutoStateData(AutoStateData &data) {
  data.lower_alive = false;
  data.imu_ready = false;
  data.grip_confirmed = false;
  data.joint_fault = false;
  data.grip_fault = false;
  data.cruise_drive_request_raw = 0u;
  data.cruise_drive_request_valid = true;
  data.auto_start = false;
  data.manual_reset = false;
  data.obstacle_crossing_trigger = false;
  data.auto_start_rising_edge_sequence = 0;
  data.manual_reset_rising_edge_sequence = 0;
  data.obstacle_trigger_rising_edge_sequence = 0;
  data.obstacle_trigger_falling_edge_sequence = 0;
  data.gravity_compensation_mode = 0u;
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
