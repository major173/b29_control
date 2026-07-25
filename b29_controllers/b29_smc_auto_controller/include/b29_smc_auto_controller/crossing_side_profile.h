// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <b29_smc_auto_controller/auto_types.h>

namespace b29_smc_auto_controller
{
struct CrossingSideProfile
{
  CrossingSide side;
  JointIndex gripper;
  JointIndex actuator_first_leg;
  JointIndex actuator_second_leg;
  GravityCompensationMode gravity_compensation_mode;
  double motion_sign;
};

inline const CrossingSideProfile* crossingSideProfile(CrossingSide side)
{
  static constexpr CrossingSideProfile kProfiles[] = {
      {CrossingSide::Left, JointIndex::LeftGripper, JointIndex::RightFirstLeg, JointIndex::RightSecondLeg,
       GravityCompensationMode::RightFirstLeg, 1.0},
      {CrossingSide::Right, JointIndex::RightGripper, JointIndex::LeftFirstLeg, JointIndex::LeftSecondLeg,
       GravityCompensationMode::LeftFirstLeg, -1.0},
  };

  for (const CrossingSideProfile& profile : kProfiles)
  {
    if (profile.side == side)
    {
      return &profile;
    }
  }
  return nullptr;
}

inline const char* crossingSideReasonName(CrossingSide side)
{
  switch (side)
  {
    case CrossingSide::Left:
      return "left";
    case CrossingSide::Right:
      return "right";
    case CrossingSide::None:
    default:
      return "none";
  }
}
}  // namespace b29_smc_auto_controller
