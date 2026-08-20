#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/node_handle.h>
#include <urdf/model.h>

namespace steering_engine_hw
{
enum class SupportSide : std::uint8_t
{
  LEFT = 0,
  RIGHT = 1,
};

/// Decode the dual-role state-machine gravity command.
/// 0 means disabled; 1 means the left gripper is the anchor/support; 2 means
/// the right gripper is the anchor/support. Returns false for 0 or invalid data.
bool stateMachineGravityModeToSupport(std::uint8_t mode, SupportSide& support);

class GravityCompensator
{
public:
  static constexpr std::size_t kJointCount = 4;
  using JointVector = Eigen::Matrix<double, kJointCount, 1>;

  bool init(const urdf::Model& urdf_model, ros::NodeHandle& nh,
            const std::string& param_ns = "/steering_engine_hw/gravity_compensation");

  /// Gravity torque assuming the support frame +Z is vertical (fallback when IMU
  /// is unavailable or disabled).
  JointVector compute(const JointVector& q_actual, SupportSide support) const;
  double potentialEnergy(const JointVector& q_actual, SupportSide support) const;

  /// Convert the fused base_imu/base_link orientation into the support-link
  /// orientation in world. `base_imu_orientation` rotates a vector expressed
  /// in base_imu/base_link into world: v_world = q * v_base.
  /// The caller must provide a finite, non-zero-norm quaternion (the hardware
  /// layer owns that validity check); this method only normalizes and uses it.
  Eigen::Quaterniond supportOrientationInWorld(
      const JointVector& q_actual, SupportSide support,
      const Eigen::Quaterniond& base_imu_orientation) const;

  /// Gravity torque with the support-link orientation measured by IMU.
  /// The support link is assumed fixed in world; `support_world_orientation`
  /// is therefore constant while taking derivatives w.r.t. q. The quaternion
  /// must already be validity-checked by the caller.
  JointVector compute(const JointVector& q_actual, SupportSide support,
                      const Eigen::Quaterniond& support_world_orientation) const;
  double potentialEnergy(const JointVector& q_actual, SupportSide support,
                         const Eigen::Quaterniond& support_world_orientation) const;

  bool enabled() const { return enabled_; }
  const std::array<std::string, kJointCount>& jointNames() const { return joint_names_; }

private:
  enum class JointType
  {
    FIXED,
    REVOLUTE,
    PRISMATIC,
  };

  struct LinkModel
  {
    std::string name;
    int parent_joint{-1};
    std::vector<int> child_joints;
    std::array<bool, kJointCount> affected_by_joint{{false, false, false, false}};
  };

  struct JointModel
  {
    std::string name;
    int parent_link{-1};
    int child_link{-1};
    int controlled_index{-1};
    JointType type{JointType::FIXED};
    Eigen::Isometry3d parent_to_joint{Eigen::Isometry3d::Identity()};
    Eigen::Vector3d axis{Eigen::Vector3d::Zero()};
  };

  struct MassElement
  {
    int link_index{-1};
    double mass{0.0};
    Eigen::Vector3d com{Eigen::Vector3d::Zero()};
  };

  struct JointKinematics
  {
    Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
    Eigen::Vector3d axis{Eigen::Vector3d::Zero()};
  };

  void computeKinematics(const JointVector& q_actual,
                         std::vector<Eigen::Isometry3d>& link_poses,
                         std::array<JointKinematics, kJointCount>& joint_kinematics) const;
  int supportLinkIndex(SupportSide support) const;

  JointVector computeImpl(const JointVector& q_actual, SupportSide support,
                          const Eigen::Quaterniond& support_world_orientation,
                          bool use_imu) const;
  double potentialEnergyImpl(const JointVector& q_actual, SupportSide support,
                             const Eigen::Quaterniond& support_world_orientation,
                             bool use_imu) const;

  /// Potential-energy gradient expressed in support coordinates.
  /// Without IMU this is [0,0,+g]. With IMU it is
  /// R_support_world^T * [0,0,+g].
  Eigen::Vector3d potentialGradientInSupport(
      const Eigen::Quaterniond& support_world_orientation, bool use_imu) const;

  std::array<std::string, kJointCount> joint_names_{{
      "left_first_leg_joint",
      "left_second_leg_joint",
      "right_first_leg_joint",
      "right_second_leg_joint",
  }};
  std::vector<LinkModel> links_;
  std::vector<JointModel> joints_;
  std::vector<MassElement> masses_;
  std::array<int, 2> support_link_indices_{{-1, -1}};
  int root_link_index_{-1};
  Eigen::Isometry3d base_imu_in_base_{Eigen::Isometry3d::Identity()};
  double gravity_{9.81};
  bool enabled_{false};
  bool initialized_{false};
};
}  // namespace steering_engine_hw
