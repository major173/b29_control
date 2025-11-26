//
// Created by yuchen on 25-9-18.
//

#include "b29_joint_controller.h"
#include <algorithm>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <pluginlib/class_list_macros.hpp>

namespace b29_controllers
{
// ============================================================================
// 初始化函数
// ============================================================================
bool B29JointController::init(hardware_interface::RobotHW* robot_hw,
                              ros::NodeHandle& root_nh,
                              ros::NodeHandle& controller_nh)
{
  // --------------------------------------------------------------------------
  // 1. 获取硬件接口
  // --------------------------------------------------------------------------
  robot_state_handle_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");
  position_joint_interface_ = robot_hw->get<hardware_interface::PositionJointInterface>();
  velocity_joint_interface_ = robot_hw->get<hardware_interface::VelocityJointInterface>();
  imu_sensor_interface_ = robot_hw->get<hardware_interface::ImuSensorInterface>();
  if (!position_joint_interface_ || !velocity_joint_interface_ || !imu_sensor_interface_)
  {
    ROS_ERROR("Failed to get position/velocity/imu interface");
    return false;
  }

  // TODO: 获取机器人状态接口
  // robot_state_handle_ = ...

  // --------------------------------------------------------------------------
  // 2. 从参数服务器加载配置参数
  // --------------------------------------------------------------------------

  // 加载关节名称
  std::string left_first_leg_joint, left_second_leg_joint, left_rod_joint;
  std::string right_first_leg_joint, right_second_leg_joint, right_rod_joint;
  std::string left_friction_wheel_joint, right_friction_wheel_joint;
  std::string base_imu;

  controller_nh.param<std::string>(
      "joint_names/left_first_leg_joint", left_first_leg_joint, "left_first_leg_joint" // 默认值=变量名
      );
  controller_nh.param<std::string>(
      "joint_names/left_second_leg_joint", left_second_leg_joint, "left_second_leg_joint"
      );
  controller_nh.param<std::string>(
      "joint_names/left_rod_joint", left_rod_joint, "left_rod_joint"
      );
  controller_nh.param<std::string>(
      "joint_names/right_first_leg_joint", right_first_leg_joint, "right_first_leg_joint"
      );
  controller_nh.param<std::string>(
      "joint_names/right_second_leg_joint", right_second_leg_joint, "right_second_leg_joint"
      );
  controller_nh.param<std::string>(
      "joint_names/right_rod_joint", right_rod_joint, "right_rod_joint"
      );
  controller_nh.param<std::string>(
      "joint_names/left_friction_wheel_joint", left_friction_wheel_joint, "left_friction_wheel_joint"
      );
  controller_nh.param<std::string>(
      "joint_names/right_friction_wheel_joint", right_friction_wheel_joint,
      "right_friction_wheel_joint"
      );
  controller_nh.param<std::string>(
      "imu_names", base_imu, "base_imu"
      );

  lf_handle_ = robot_hw->get<hardware_interface::PositionJointInterface>()->getHandle(left_first_leg_joint);
  ls_handle_ = robot_hw->get<hardware_interface::PositionJointInterface>()->getHandle(left_second_leg_joint);
  lr_handle_ = robot_hw->get<hardware_interface::PositionJointInterface>()->getHandle(left_rod_joint);
  lf_wheel_handle_ = robot_hw->get<hardware_interface::VelocityJointInterface>()->getHandle(left_friction_wheel_joint);
  rf_handle_ = robot_hw->get<hardware_interface::PositionJointInterface>()->getHandle(right_first_leg_joint);
  rs_handle_ = robot_hw->get<hardware_interface::PositionJointInterface>()->getHandle(right_second_leg_joint);
  rr_handle_ = robot_hw->get<hardware_interface::PositionJointInterface>()->getHandle(right_rod_joint);
  rf_wheel_handle_ = robot_hw->get<hardware_interface::VelocityJointInterface>()->getHandle(right_friction_wheel_joint);
  base_imu_handle_ = imu_sensor_interface_->getHandle(base_imu);

  // 加载控制器参数
  controller_nh.param<double>("publish_rate", publish_rate_, 100.0);
  controller_nh.param<double>("max_odom_vel", max_odom_vel_, 2.0);
  controller_nh.param<double>("release_rod_pos", release_rod_pos_, 0.2);
  controller_nh.param<double>("move_rod_pos", move_rod_pos_, 0.4);
  controller_nh.param<double>("fixed_rod_pos", fixed_rod_pos_, 0.5);
  controller_nh.param<double>("fixed_check_vel", fixed_check_vel_, 0.3);
  controller_nh.param<double>("fixed_check_period", fixed_check_period_, 0.5);
  controller_nh.param<double>("dbus_online_threshold", dbus_online_threshold_, 0.3);
  controller_nh.param<bool>("enable_odom_tf", enable_odom_tf_, true);
  controller_nh.param<std::string>("base_frame", base_frame_, "base_link");

  // TODO: 加载模式切换相关参数（如切换阈值、安全检查参数等）
  // double mode_switch_threshold;
  // double arm_mode_height_min, arm_mode_height_max;

  // TODO: 加载轨道模式运动参数
  // double track_mode_max_vel, track_mode_max_acc;

  // TODO: 加载机械臂模式运动参数
  // double arm_swing_amplitude, arm_swing_frequency;

  // --------------------------------------------------------------------------
  // 4. 初始化PID控制器
  // --------------------------------------------------------------------------

  // TODO: 初始化跟踪模式的PID控制器
  // if (!pid_follow_.init(ros::NodeHandle(controller_nh, "pid_follow")))
  // {
  //   ROS_ERROR("Failed to initialize PID controller");
  //   return false;
  // }

  // TODO: 为各关节初始化独立的PID控制器（如果需要）
  // pid_left_first_leg_.init(...);
  // pid_left_second_leg_.init(...);

  // --------------------------------------------------------------------------
  // 5. 初始化滤波器
  // --------------------------------------------------------------------------

  // 初始化斜坡滤波器（用于平滑速度指令）
  double ramp_rate;
  controller_nh.param<double>("ramp_rate", ramp_rate, 1.0);
  ramp_x_ = new RampFilter<double>(ramp_rate, 0.0);

  // TODO: 初始化其他滤波器（如低通滤波器、卡尔曼滤波器等）
  // lpf_imu_acc_.init(...);
  // kf_position_.init(...);

  // --------------------------------------------------------------------------
  // 6. 初始化发布器和订阅器
  // --------------------------------------------------------------------------

  // 初始化里程计发布器
  odom_pub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::Odometry>>(
      root_nh, "odom", 100);

  // TODO: 初始化TF广播器
  // if (enable_odom_tf_)
  // {
  //   tf_broadcaster_.init(root_nh);
  // }

  // 初始化订阅器
  dbus_sub_ = root_nh.subscribe<rm_msgs::DbusData>(
      "/dbus_data", 1, &B29JointController::dbusDataCallback, this);

  // TODO: 订阅其他必要的话题
  // cmd_chassis_sub_ = root_nh.subscribe(...);
  // outside_odom_sub_ = root_nh.subscribe(...);
  // mode_switch_sub_ = root_nh.subscribe(...);

  // --------------------------------------------------------------------------
  // 7. 初始化状态变量
  // --------------------------------------------------------------------------

  state_ = DEBUG; // 默认调试模式
  front_frame_ = Left;
  state_changed_ = true;
  last_publish_time_ = ros::Time::now();

  last_joint_position_.resize(JointIndex::COUNT_P);
  last_joint_velocity_.resize(2);
  command_vector_.resize(JointIndex::COUNT_P);

  // TODO: 初始化运动学状态
  // odom2base_.setIdentity();
  // world2odom_.setIdentity();

  ROS_INFO("B29 Joint Controller initialized successfully");
  return true;
}

// ============================================================================
// 控制器启动
// ============================================================================
void B29JointController::starting(const ros::Time& time)
{
  updateLastJointState();

  // TODO: 重置PID控制器
  // pid_follow_.reset();

  // TODO: 重置滤波器
  ramp_x_->clear(0.0);

  // TODO: 重置里程计
  // odom2base_.setIdentity();
  // world2odom_.setIdentity();

  state_ = DEBUG;
  state_changed_ = true;

  last_publish_time_ = time;

  ROS_INFO("B29 Joint Controller started");
}

// ============================================================================
// 控制器停止
// ============================================================================
void B29JointController::stopping(const ros::Time& time)
{
  // TODO: 停止所有运动
  // left_friction_wheel_handle_.setCommand(0.0);
  // right_friction_wheel_handle_.setCommand(0.0);

  // TODO: 将关节移动到安全位置
  // moveToSafePosition();

  ROS_INFO("B29 Joint Controller stopped");
}

// ============================================================================
// 主更新循环
// ============================================================================
void B29JointController::update(const ros::Time& time, const ros::Duration& period)
{
  // --------------------------------------------------------------------------
  // 1. 读取传感器数据
  // --------------------------------------------------------------------------

  geometry_msgs::Vector3 gyro;
  gyro.x = base_imu_handle_.getAngularVelocity()[0];
  gyro.y = base_imu_handle_.getAngularVelocity()[1];
  gyro.z = base_imu_handle_.getAngularVelocity()[2];

  double roll, pitch, yaw;
  tf2::Quaternion odom2imu_quaternion;
  tf2::Vector3 odom2imu_origin;
  odom2imu_quaternion.setValue(base_imu_handle_.getOrientation()[0], base_imu_handle_.getOrientation()[1],
                               base_imu_handle_.getOrientation()[2], base_imu_handle_.getOrientation()[3]);
  quatToRPY(toMsg(odom2imu_quaternion), roll, pitch, yaw);

  // TODO: 读取关节状态
  // double left_first_leg_pos = left_first_leg_handle_.getPosition();
  // double left_first_leg_vel = left_first_leg_handle_.getVelocity();
  // ... (读取所有关节的位置、速度、力矩)

  // TODO: 读取摩擦轮状态
  // double left_wheel_vel = left_friction_wheel_handle_.getVelocity();
  // double right_wheel_vel = right_friction_wheel_handle_.getVelocity();

  // --------------------------------------------------------------------------
  // 2. 读取控制指令
  // --------------------------------------------------------------------------

  // TODO: 从实时缓冲区读取速度指令
  // geometry_msgs::Twist cmd_vel = *vel_cmd_buffer_.readFromRT();

  // TODO: 应用斜坡滤波
  // double filtered_vel_x = ramp_x_->update(cmd_vel.linear.x, period.toSec());

  // --------------------------------------------------------------------------
  // 3. 模式判断和切换
  // --------------------------------------------------------------------------

  // TODO: 根据传感器数据和指令判断应该处于哪个模式
  // state_ = determineMode(imu_data, joint_states, cmd_vel);

  // if (state_ != last_state_)
  // {
  //   state_changed_ = true;
  //   ROS_INFO("Mode switched: %s -> %s",
  //            last_state_ == TRACK ? "TRACK" : "ARM",
  //            state_ == TRACK ? "TRACK" : "ARM");
  //
  //   // TODO: 执行模式切换时的过渡动作
  //   // transitionToMode(state_);
  // }

  // --------------------------------------------------------------------------
  // 4. 根据模式执行不同的控制逻辑
  // --------------------------------------------------------------------------

  if (dbus_online_ && (ros::Time::now() - last_dbus_time_).sec > dbus_online_threshold_)
  {
    ROS_INFO("Dbus state change to [offline]");
    dbus_online_ = false;
  }
  else if (!dbus_online_ && (ros::Time::now() - last_dbus_time_).sec < dbus_online_threshold_)
  {
    ROS_INFO("Dbus state change to [online]");
    dbus_online_ = true;
  }

  if (!dbus_online_ && state_ != IDLE)
  {
    state_ = IDLE;
    state_changed_ = true;
  }

  switch (state_)
  {
    case TRACK:
      updateTrackMode(time, period);
      break;

    case ARM:
      updateArmMode(time, period);
      break;

    case DEBUG:
      updateDebugMode(time, period);
      break;

    case IDLE:
      updateIdleMode(time, period);
      break;

    default:
      ROS_ERROR_THROTTLE(1.0, "Unknown state: %d", state_);
      break;
  }

  // --------------------------------------------------------------------------
  // 5. 更新里程计和TF
  // --------------------------------------------------------------------------

  if ((time - last_publish_time_).toSec() >= 1.0 / publish_rate_)
  {
    //updateOdometry(time, period);
    //publishOdometry(time);
    last_publish_time_ = time;
  }

  // TODO: 发布调试信息
  // publishDebugInfo(time);

  applyJointCommand();
  updateLastJointState();
}

// ============================================================================
// 轨道模式控制
// ============================================================================
void B29JointController::updateTrackMode(const ros::Time& time, const ros::Duration& period)
{
  // TODO: 1. 计算目标位置和姿态
  // - 保持机械臂收拢以夹紧导线
  // - 计算左右摩擦轮速度以实现前进/后退

  // TODO: 2. 位置控制 - 保持夹紧姿态
  // double left_rod_target = calculateGripPosition();
  // double right_rod_target = calculateGripPosition();
  // left_rod_handle_.setCommand(left_rod_target);
  // right_rod_handle_.setCommand(right_rod_target);

  // TODO: 3. 保持机械臂稳定
  // - 计算一腿和二腿的目标角度以保持平衡
  // double left_first_leg_target = ...;
  // double left_second_leg_target = ...;
  // left_first_leg_handle_.setCommand(left_first_leg_target);
  // left_second_leg_handle_.setCommand(left_second_leg_target);

  // TODO: 4. 速度控制 - 摩擦轮驱动
  // double forward_vel = vel_cmd_.x;
  // double left_wheel_vel = calculateWheelVelocity(forward_vel);
  // double right_wheel_vel = calculateWheelVelocity(forward_vel);
  // left_friction_wheel_handle_.setCommand(left_wheel_vel);
  // right_friction_wheel_handle_.setCommand(right_wheel_vel);

  // TODO: 5. 安全检查
  // - 检查是否即将遇到障碍物
  // - 检查夹持力是否足够
  // if (shouldSwitchToArmMode())
  // {
  //   state_ = ARM;
  // }
}

// ============================================================================
// 机械臂模式控制
// ============================================================================
void B29JointController::updateArmMode(const ros::Time& time, const ros::Duration& period)
{
  // TODO: 1. 确定是摆动跨越还是静止状态
  // bool is_crossing = isCrossingObstacle();

  // TODO: 2. 固定一端
  // - 确定哪一端应该固定（前端或后端）
  // - 控制该端的夹爪夹紧
  // bool fix_left = shouldFixLeftSide();
  // if (fix_left)
  // {
  //   left_rod_handle_.setCommand(GRIP_POSITION);
  // }

  // TODO: 3. 释放另一端的摩擦轮
  // if (fix_left)
  // {
  //   right_friction_wheel_handle_.setCommand(0.0);
  // }

  // TODO: 4. 摆动控制
  // if (is_crossing)
  // {
  //   // 计算摆动轨迹
  //   double swing_angle = calculateSwingAngle(time);
  //   double swing_extension = calculateSwingExtension(time);
  //   
  //   // 控制自由端的关节实现摆动
  //   right_first_leg_handle_.setCommand(swing_angle);
  //   right_second_leg_handle_.setCommand(...);
  //   right_rod_handle_.setCommand(swing_extension);
  // }

  // TODO: 5. 着陆检测和夹紧
  // if (hasLanded())
  // {
  //   // 新的支撑点夹紧
  //   right_rod_handle_.setCommand(GRIP_POSITION);
  //   
  //   // 释放旧的支撑点
  //   left_rod_handle_.setCommand(RELEASE_POSITION);
  //   
  //   // 准备下一次摆动或切换回轨道模式
  // }

  // TODO: 6. 重心平衡控制
  // - 使用IMU数据调整姿态
  // - 避免过度摆动导致失稳
  // adjustBalanceControl(imu_roll, imu_pitch);

  // TODO: 7. 完成跨越后的模式切换
  // if (obstacleCrossed())
  // {
  //   state_ = TRACK;
  // }
}

// ============================================================================
// 调试模式控制
// ============================================================================
void B29JointController::updateDebugMode(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {
    ROS_INFO("State Enter DEBUG model");
    state_changed_ = false;
  }

  command_vector_.at(leftFriction) = dbus_data_.wheel;
  command_vector_.at(rightFriction) = dbus_data_.wheel;

  if (dbus_data_.s_l == rm_msgs::DbusData::MID)
  {
    command_vector_.at(leftFirst) += dbus_data_.ch_r_y * period.toSec();
    command_vector_.at(leftSecond) += dbus_data_.ch_l_x * period.toSec();
  }
  else if (dbus_data_.s_l == rm_msgs::DbusData::DOWN)
  {
    command_vector_.at(rightFirst) += dbus_data_.ch_r_y * period.toSec();
    command_vector_.at(rightSecond) += dbus_data_.ch_l_x * period.toSec();
  }

  switch (dbus_data_.s_l)
  {
    case rm_msgs::DbusData::DOWN:
      command_vector_.at(leftRod) = release_rod_pos_;
      break;
    case rm_msgs::DbusData::MID:
      command_vector_.at(leftRod) = move_rod_pos_;
      break;
    case rm_msgs::DbusData::UP:
      command_vector_.at(leftRod) = fixed_rod_pos_;
      break;
    default:
      break;
  }
  switch (dbus_data_.s_r)
  {
    case rm_msgs::DbusData::DOWN:
      command_vector_.at(rightRod) = release_rod_pos_;
      break;
    case rm_msgs::DbusData::MID:
      command_vector_.at(rightRod) = move_rod_pos_;
      break;
    case rm_msgs::DbusData::UP:
      command_vector_.at(rightRod) = fixed_rod_pos_;
      break;
    default:
      break;
  }
}


// ============================================================================
// 闲置模式控制
// ============================================================================
void B29JointController::updateIdleMode(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {
    ROS_INFO("State Enter IDLE model");
    state_changed_ = false;
  }

  if (dbus_data_.s_l == rm_msgs::DbusData::UP && dbus_data_.s_r == rm_msgs::DbusData::UP)
  {
    state_ = DEBUG;
    state_changed_ = true;
  }
}


// ============================================================================
// 里程计更新
// ============================================================================
/*
void B29JointController::updateOdometry(const ros::Time& time, const ros::Duration& period)
{
  // TODO: 根据当前模式使用不同的里程计计算方法

  if (state_ == TRACK)
  {
    // TODO: 轨道模式 - 基于摩擦轮速度计算
    // double left_wheel_vel = left_friction_wheel_handle_.getVelocity();
    // double right_wheel_vel = right_friction_wheel_handle_.getVelocity();
    // double linear_vel = (left_wheel_vel + right_wheel_vel) / 2.0 * wheel_radius;
    // 
    // // 更新位置
    // odom_x += linear_vel * cos(odom_yaw) * period.toSec();
    // odom_y += linear_vel * sin(odom_yaw) * period.toSec();
  }
  else // ARM mode
  {
    // TODO: 机械臂模式 - 基于关节位置计算末端位置
    // calculateEndEffectorPosition(joint_positions);
    // updateOdomFromEndEffector();
  }

  // TODO: 融合IMU数据提高精度
  // fuseWithIMU(imu_data);
}
*/

// ============================================================================
// 发布里程计信息
// ============================================================================
/*
void B29JointController::publishOdometry(const ros::Time& time)
{
  if (odom_pub_->trylock())
  {
    // TODO: 填充里程计消息
    // odom_pub_->msg_.header.stamp = time;
    // odom_pub_->msg_.header.frame_id = front_frame_;
    // odom_pub_->msg_.child_frame_id = base_frame_;
    // 
    // odom_pub_->msg_.pose.pose.position.x = odom_x;
    // odom_pub_->msg_.pose.pose.position.y = odom_y;
    // odom_pub_->msg_.pose.pose.position.z = odom_z;
    // 
    // odom_pub_->msg_.twist.twist.linear.x = vel_x;
    // odom_pub_->msg_.twist.twist.angular.z = vel_yaw;

    odom_pub_->unlockAndPublish();
  }

  // TODO: 发布TF变换
  // if (enable_odom_tf_ && publish_odom_tf_)
  // {
  //   geometry_msgs::TransformStamped tf_msg;
  //   tf_msg.header.stamp = time;
  //   tf_msg.header.frame_id = front_frame_;
  //   tf_msg.child_frame_id = base_frame_;
  //   // ... 填充变换数据
  //   tf_broadcaster_.sendTransform(tf_msg);
  // }
}
*/

// ============================================================================
// 辅助函数
// ============================================================================

// TODO: 实现模式判断函数
// int B29JointController::determineMode(const ImuData& imu, 
//                                        const JointStates& joints,
//                                        const geometry_msgs::Twist& cmd)
// {
//   // 根据传感器和指令判断应该在哪个模式
//   return TRACK;
// }

// TODO: 实现模式切换过渡函数
// void B29JointController::transitionToMode(int target_mode)
// {
//   // 平滑过渡到目标模式
// }

// TODO: 实现各种计算函数
// double B29JointController::calculateGripPosition() { return 0.0; }
// double B29JointController::calculateWheelVelocity(double forward_vel) { return 0.0; }
// double B29JointController::calculateSwingAngle(const ros::Time& time) { return 0.0; }
// bool B29JointController::shouldSwitchToArmMode() { return false; }
// bool B29JointController::isCrossingObstacle() { return false; }
// bool B29JointController::hasLanded() { return false; }
// bool B29JointController::obstacleCrossed() { return false; }

// ============================================================================
// 回调函数
// ============================================================================
void B29JointController::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& msg)
{
  // TODO: 将速度指令写入实时缓冲区
  dbus_data_ = *msg;

  last_dbus_time_ = dbus_data_.stamp;
}

// TODO: 实现其他回调函数
// void B29JointController::cmdChassisCallback(...) {}
// void B29JointController::outsideOdomCallback(...) {}
// void B29JointController::modeSwitchCallback(...) {}


void B29JointController::updateLastJointState()
{
  last_joint_position_.at(leftFriction) = lf_wheel_handle_.getPosition();
  last_joint_position_.at(rightFriction) = rf_wheel_handle_.getPosition();
  last_joint_position_.at(leftFirst) = lf_handle_.getPosition();
  last_joint_position_.at(leftSecond) = ls_handle_.getPosition();
  last_joint_position_.at(leftRod) = lr_handle_.getPosition();
  last_joint_position_.at(rightFirst) = rf_handle_.getPosition();
  last_joint_position_.at(rightSecond) = rs_handle_.getPosition();
  last_joint_position_.at(rightRod) = rr_handle_.getPosition();

  last_joint_velocity_.at(leftFriction) = lf_wheel_handle_.getVelocity();
  last_joint_velocity_.at(rightFriction) = rf_wheel_handle_.getVelocity();
}


// TODO: 动态检查推杆阈值适应不同线径
void B29JointController::updateRodThreshold()
{
}

void B29JointController::applyJointCommand()
{
  // Joint Limit
  auto clampFirst = [](double v) { return std::clamp(v, -M_PI, M_PI); };
  auto clampSecond = [](double v) { return std::clamp(v, -M_PI, M_PI); };
  auto clampRod = [](double v) { return std::clamp(v, 0.02, 0.058); };
  auto clampFriction = [](double v) { return std::clamp(v, -20.0, 20.0); };

  // --- Apply limits directly to command_vector ---
  command_vector_[leftFriction] = clampFriction(command_vector_.at(leftFriction));
  command_vector_[rightFriction] = clampFriction(command_vector_.at(rightFriction));

  command_vector_[leftFirst] = clampFirst(command_vector_.at(leftFirst));
  command_vector_[leftSecond] = clampSecond(command_vector_.at(leftSecond));
  command_vector_[leftRod] = clampRod(command_vector_.at(leftRod));
  command_vector_[rightFirst] = clampFirst(command_vector_.at(rightFirst));
  command_vector_[rightSecond] = clampSecond(command_vector_.at(rightSecond));
  command_vector_[rightRod] = clampRod(command_vector_.at(rightRod));

  // --- Set Command using clamped values ---
  lf_wheel_handle_.setCommand(command_vector_.at(leftFriction)); // TODO: velocity control has problem
  rf_wheel_handle_.setCommand(command_vector_.at(rightFriction));

  lf_handle_.setCommand(command_vector_.at(leftFirst));
  ls_handle_.setCommand(command_vector_.at(leftSecond));
  lr_handle_.setCommand(command_vector_.at(leftRod));
  rf_handle_.setCommand(command_vector_.at(rightFirst));
  rs_handle_.setCommand(command_vector_.at(rightSecond));
  rr_handle_.setCommand(command_vector_.at(rightRod));
}
} // namespace b29_controllers

// 注册控制器插件
PLUGINLIB_EXPORT_CLASS(b29_controllers::B29JointController,
                       controller_interface::ControllerBase)