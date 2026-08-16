#include "steering_engine/common/gravity_compensator.h"

#include <cmath>
#include <functional>
#include <unordered_map>

#include <ros/console.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace steering_engine_hw
{
namespace
{
bool xmlRpcToDouble(const XmlRpc::XmlRpcValue& value, double& result)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    result = static_cast<double>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    result = static_cast<int>(value);
    return true;
  }
  return false;
}

bool xmlRpcToVector3(const XmlRpc::XmlRpcValue& value, Eigen::Vector3d& result)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 3)
  {
    return false;
  }

  for (int i = 0; i < 3; ++i)
  {
    double component = 0.0;
    if (!xmlRpcToDouble(value[i], component) || !std::isfinite(component))
    {
      return false;
    }
    result[i] = component;
  }
  return true;
}

Eigen::Isometry3d poseToEigen(const urdf::Pose& pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond rotation(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  if (rotation.norm() > 0.0)
  {
    rotation.normalize();
    transform.linear() = rotation.toRotationMatrix();
  }
  return transform;
}
}  // namespace

bool GravityCompensator::init(const urdf::Model& urdf_model, ros::NodeHandle& nh,
                              const std::string& param_ns)
{
  initialized_ = false;
  links_.clear();
  joints_.clear();
  masses_.clear();
  support_link_indices_ = {{-1, -1}};
  root_link_index_ = -1;

  nh.param(param_ns + "/enabled", enabled_, false);
  if (!enabled_)
  {
    initialized_ = true;
    ROS_INFO("Gravity compensation is disabled");
    return true;
  }

  nh.param(param_ns + "/gravity", gravity_, 9.81);
  if (!std::isfinite(gravity_) || gravity_ <= 0.0)
  {
    ROS_ERROR_STREAM(param_ns << "/gravity must be finite and positive");
    return false;
  }

  const urdf::LinkConstSharedPtr root = urdf_model.getRoot();
  if (!root)
  {
    ROS_ERROR("Gravity compensation could not find the URDF root link");
    return false;
  }

  std::unordered_map<std::string, int> link_indices;
  std::array<int, kJointCount> controlled_joint_indices{{-1, -1, -1, -1}};
  std::function<int(const urdf::LinkConstSharedPtr&, int)> add_link;
  add_link = [&](const urdf::LinkConstSharedPtr& link, int parent_joint) -> int {
    const int link_index = static_cast<int>(links_.size());
    link_indices[link->name] = link_index;
    links_.push_back(LinkModel{});
    links_.back().name = link->name;
    links_.back().parent_joint = parent_joint;

    for (const urdf::JointSharedPtr& urdf_joint : link->child_joints)
    {
      JointModel joint;
      joint.name = urdf_joint->name;
      joint.parent_link = link_index;
      joint.parent_to_joint = poseToEigen(urdf_joint->parent_to_joint_origin_transform);
      joint.axis = Eigen::Vector3d(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);

      if (urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::CONTINUOUS)
      {
        joint.type = JointType::REVOLUTE;
      }
      else if (urdf_joint->type == urdf::Joint::PRISMATIC)
      {
        joint.type = JointType::PRISMATIC;
      }
      else
      {
        joint.type = JointType::FIXED;
      }

      for (std::size_t i = 0; i < joint_names_.size(); ++i)
      {
        if (joint.name == joint_names_[i])
        {
          joint.controlled_index = static_cast<int>(i);
          controlled_joint_indices[i] = static_cast<int>(joints_.size());
          break;
        }
      }

      const int joint_index = static_cast<int>(joints_.size());
      joints_.push_back(joint);
      links_[link_index].child_joints.push_back(joint_index);

      const urdf::LinkConstSharedPtr child_link = urdf_model.getLink(urdf_joint->child_link_name);
      if (!child_link)
      {
        ROS_ERROR_STREAM("Gravity compensation URDF joint " << urdf_joint->name
                         << " has no child link " << urdf_joint->child_link_name);
        return -1;
      }
      const int child_index = add_link(child_link, joint_index);
      if (child_index < 0)
      {
        return -1;
      }
      joints_[joint_index].child_link = child_index;
    }
    return link_index;
  };

  root_link_index_ = add_link(root, -1);
  if (root_link_index_ < 0)
  {
    return false;
  }

  // Cache the fixed base_link -> base_imu transform from the URDF. The IMU
  // quaternion is expressed in base_imu frame; if the mechanical mounting
  // rpy is updated in the URDF later, this transform keeps the math correct
  // without code changes.
  {
    std::vector<Eigen::Isometry3d> zero_poses;
    std::array<JointKinematics, kJointCount> zero_joint_kinematics;
    computeKinematics(JointVector::Zero(), zero_poses, zero_joint_kinematics);
    const auto base_imu_it = link_indices.find("base_imu");
    if (base_imu_it != link_indices.end())
    {
      base_imu_in_base_ = zero_poses[base_imu_it->second];
    }
    else
    {
      ROS_WARN("URDF has no base_imu link; assuming IMU frame equals base_link");
      base_imu_in_base_ = Eigen::Isometry3d::Identity();
    }
  }

  for (std::size_t i = 0; i < controlled_joint_indices.size(); ++i)
  {
    if (controlled_joint_indices[i] < 0)
    {
      ROS_ERROR_STREAM("Gravity compensation URDF is missing joint " << joint_names_[i]);
      return false;
    }
  }

  for (std::size_t link_index = 0; link_index < links_.size(); ++link_index)
  {
    int joint_index = links_[link_index].parent_joint;
    while (joint_index >= 0)
    {
      const int controlled_index = joints_[joint_index].controlled_index;
      if (controlled_index >= 0)
      {
        links_[link_index].affected_by_joint[controlled_index] = true;
      }
      joint_index = links_[joints_[joint_index].parent_link].parent_joint;
    }
  }

  std::string left_support_link = "left_second_leg";
  std::string right_support_link = "right_second_leg";
  nh.param(param_ns + "/support_links/left", left_support_link, left_support_link);
  nh.param(param_ns + "/support_links/right", right_support_link, right_support_link);
  const auto left_support = link_indices.find(left_support_link);
  const auto right_support = link_indices.find(right_support_link);
  if (left_support == link_indices.end() || right_support == link_indices.end())
  {
    ROS_ERROR_STREAM("Gravity compensation support link is absent from URDF: left="
                     << left_support_link << ", right=" << right_support_link);
    return false;
  }
  support_link_indices_[0] = left_support->second;
  support_link_indices_[1] = right_support->second;

  XmlRpc::XmlRpcValue link_params;
  if (!nh.getParam(param_ns + "/links", link_params) ||
      link_params.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_STREAM("Missing gravity mass/COM map: " << param_ns << "/links");
    return false;
  }

  for (auto it = link_params.begin(); it != link_params.end(); ++it)
  {
    const std::string link_name = it->first;
    const auto model_link = link_indices.find(link_name);
    if (model_link == link_indices.end())
    {
      ROS_ERROR_STREAM("Gravity config link is absent from URDF: " << link_name);
      return false;
    }

    XmlRpc::XmlRpcValue& params = it->second;
    if (params.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !params.hasMember("mass") || !params.hasMember("com"))
    {
      ROS_ERROR_STREAM("Gravity config for " << link_name << " requires mass and com");
      return false;
    }

    MassElement element;
    element.link_index = model_link->second;
    if (!xmlRpcToDouble(params["mass"], element.mass) ||
        !std::isfinite(element.mass) || element.mass < 0.0)
    {
      ROS_ERROR_STREAM("Gravity config mass must be finite and non-negative for " << link_name);
      return false;
    }
    if (!xmlRpcToVector3(params["com"], element.com))
    {
      ROS_ERROR_STREAM("Gravity config com must be a numeric three-vector for " << link_name);
      return false;
    }
    if (element.mass > 0.0)
    {
      masses_.push_back(element);
    }
  }

  if (masses_.empty())
  {
    ROS_ERROR("Gravity compensation has no positive-mass config entries");
    return false;
  }

  initialized_ = true;
  ROS_INFO("Gravity compensation initialized with %zu configured mass elements", masses_.size());
  return true;
}

void GravityCompensator::computeKinematics(
    const JointVector& q_actual, std::vector<Eigen::Isometry3d>& link_poses,
    std::array<JointKinematics, kJointCount>& joint_kinematics) const
{
  link_poses.assign(links_.size(), Eigen::Isometry3d::Identity());
  link_poses[root_link_index_] = Eigen::Isometry3d::Identity();

  for (const JointModel& joint : joints_)
  {
    const Eigen::Isometry3d joint_pose = link_poses[joint.parent_link] * joint.parent_to_joint;
    Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
    const double position = joint.controlled_index >= 0 ? q_actual[joint.controlled_index] : 0.0;

    if (joint.type == JointType::REVOLUTE)
    {
      motion.linear() = Eigen::AngleAxisd(position, joint.axis.normalized()).toRotationMatrix();
    }
    else if (joint.type == JointType::PRISMATIC)
    {
      motion.translation() = position * joint.axis.normalized();
    }

    if (joint.controlled_index >= 0)
    {
      JointKinematics& kinematics = joint_kinematics[joint.controlled_index];
      kinematics.origin = joint_pose.translation();
      kinematics.axis = joint_pose.linear() * joint.axis.normalized();
    }
    link_poses[joint.child_link] = joint_pose * motion;
  }
}

int GravityCompensator::supportLinkIndex(SupportSide support) const
{
  return support_link_indices_[support == SupportSide::LEFT ? 0 : 1];
}

GravityCompensator::JointVector GravityCompensator::compute(
    const JointVector& q_actual, SupportSide support) const
{
  return computeImpl(q_actual, support, Eigen::Quaterniond::Identity(), false);
}

Eigen::Quaterniond GravityCompensator::supportOrientationInWorld(
    const JointVector& q_actual, SupportSide support,
    const Eigen::Quaterniond& base_imu_orientation) const
{
  // Validity is guaranteed by the hardware layer (imu_ready and a per-frame
  // finite/non-zero-norm check). Normalization here is intentionally the only
  // quaternion processing inside the model.
  Eigen::Quaterniond orientation = base_imu_orientation.normalized();

  std::vector<Eigen::Isometry3d> link_poses;
  std::array<JointKinematics, kJointCount> joint_kinematics;
  computeKinematics(q_actual, link_poses, joint_kinematics);

  const Eigen::Matrix3d support_in_base =
      link_poses[supportLinkIndex(support)].linear();
  // q_imu rotates IMU-frame vectors into world.  base_imu_in_base_.linear()
  // rotates base-frame vectors into the IMU frame (from the URDF fixed joint).
  const Eigen::Matrix3d base_in_world =
      orientation.toRotationMatrix() * base_imu_in_base_.linear();
  const Eigen::Matrix3d support_in_world = base_in_world * support_in_base;
  return Eigen::Quaterniond(support_in_world);
}

GravityCompensator::JointVector GravityCompensator::compute(
    const JointVector& q_actual, SupportSide support,
    const Eigen::Quaterniond& support_world_orientation) const
{
  return computeImpl(q_actual, support, support_world_orientation, true);
}

double GravityCompensator::potentialEnergy(
    const JointVector& q_actual, SupportSide support) const
{
  return potentialEnergyImpl(q_actual, support, Eigen::Quaterniond::Identity(), false);
}

double GravityCompensator::potentialEnergy(
    const JointVector& q_actual, SupportSide support,
    const Eigen::Quaterniond& support_world_orientation) const
{
  return potentialEnergyImpl(q_actual, support, support_world_orientation, true);
}

Eigen::Vector3d GravityCompensator::potentialGradientInSupport(
    const Eigen::Quaterniond& support_world_orientation, bool use_imu) const
{
  // World gravity points down. The potential-energy gradient is +g along the
  // world vertical axis, so without IMU this is support-frame [0,0,+g].
  const Eigen::Vector3d gradient_world(0.0, 0.0, gravity_);
  if (!use_imu)
  {
    return gradient_world;
  }

  const Eigen::Quaterniond orientation =
      support_world_orientation.normalized();
  return orientation.conjugate() * gradient_world;
}

GravityCompensator::JointVector GravityCompensator::computeImpl(
    const JointVector& q_actual, SupportSide support,
    const Eigen::Quaterniond& support_world_orientation, bool use_imu) const
{
  JointVector torque = JointVector::Zero();
  if (!enabled_ || !initialized_ || !q_actual.allFinite())
  {
    return torque;
  }

  std::vector<Eigen::Isometry3d> link_poses;
  std::array<JointKinematics, kJointCount> joint_kinematics;
  computeKinematics(q_actual, link_poses, joint_kinematics);

  const int support_index = supportLinkIndex(support);
  const Eigen::Isometry3d& support_pose = link_poses[support_index];
  const Eigen::Matrix3d support_rotation_inverse = support_pose.linear().transpose();
  const Eigen::Vector3d support_origin = support_pose.translation();
  const Eigen::Vector3d potential_gradient_support =
      potentialGradientInSupport(support_world_orientation, use_imu);

  for (const MassElement& element : masses_)
  {
    const Eigen::Vector3d point = link_poses[element.link_index] * element.com;
    const Eigen::Vector3d support_to_point = point - support_origin;

    for (std::size_t joint_index = 0; joint_index < kJointCount; ++joint_index)
    {
      const JointKinematics& joint = joint_kinematics[joint_index];
      Eigen::Vector3d point_velocity = Eigen::Vector3d::Zero();
      if (links_[element.link_index].affected_by_joint[joint_index])
      {
        point_velocity = joint.axis.cross(point - joint.origin);
      }

      Eigen::Vector3d support_velocity = Eigen::Vector3d::Zero();
      Eigen::Vector3d support_angular_velocity = Eigen::Vector3d::Zero();
      if (links_[support_index].affected_by_joint[joint_index])
      {
        support_velocity = joint.axis.cross(support_origin - joint.origin);
        support_angular_velocity = joint.axis;
      }

      const Eigen::Vector3d relative_velocity = support_rotation_inverse *
          (point_velocity - support_velocity - support_angular_velocity.cross(support_to_point));
      torque[joint_index] += element.mass * relative_velocity.dot(potential_gradient_support);
    }
  }

  return torque;
}

double GravityCompensator::potentialEnergyImpl(
    const JointVector& q_actual, SupportSide support,
    const Eigen::Quaterniond& support_world_orientation, bool use_imu) const
{
  if (!enabled_ || !initialized_ || !q_actual.allFinite())
  {
    return 0.0;
  }

  std::vector<Eigen::Isometry3d> link_poses;
  std::array<JointKinematics, kJointCount> joint_kinematics;
  computeKinematics(q_actual, link_poses, joint_kinematics);

  const Eigen::Isometry3d& support_pose = link_poses[supportLinkIndex(support)];
  const Eigen::Isometry3d support_inverse = support_pose.inverse();
  const Eigen::Vector3d potential_gradient_support =
      potentialGradientInSupport(support_world_orientation, use_imu);

  double energy = 0.0;
  for (const MassElement& element : masses_)
  {
    const Eigen::Vector3d com_in_support = support_inverse * (link_poses[element.link_index] * element.com);
    energy += element.mass * com_in_support.dot(potential_gradient_support);
  }
  return energy;
}
}  // namespace steering_engine_hw
