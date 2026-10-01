#pragma once
#include <cmath>

namespace swerve_bringup
{
inline bool valid_motor_target(double target, double effort, double velocity)
{
  return std::isfinite(target) && std::isfinite(effort) && effort > 0 &&
         std::isfinite(velocity) && velocity > 0 && std::abs(target) <= velocity;
}

template<class Joint>
bool apply_motor_target(Joint & joint, double target, double effort, double velocity)
{
  if (!valid_motor_target(target, effort, velocity)) {
    joint.SetParam("vel", 0, 0.0);
    return false;
  }
  if (!joint.SetParam("fmax", 0, effort) || !joint.SetParam("vel", 0, target)) {
    joint.SetParam("vel", 0, 0.0);
    return false;
  }
  return true;
}
}  // namespace swerve_bringup
