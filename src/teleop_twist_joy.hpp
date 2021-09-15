#ifndef TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_H
#define TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_H

#include <rclcpp/rclcpp.hpp>
#include "teleop_twist_joy/teleop_twist_joy_export.h"

namespace teleop_twist_joy
{

/**
 * Class implementing a basic Joy -> Twist translation.
 */
class TELEOP_TWIST_JOY_EXPORT TeleopTwistJoy : public rclcpp::Node
{
public:
  explicit TeleopTwistJoy(const rclcpp::NodeOptions& options);

  virtual ~TeleopTwistJoy();

private:
  struct Impl;
  Impl* pimpl_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle;  
};

}  // namespace teleop_twist_joy

#endif  // TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_H