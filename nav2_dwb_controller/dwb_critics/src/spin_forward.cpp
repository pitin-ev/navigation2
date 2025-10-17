#include "dwb_critics/spin_forward.hpp"
#include <math.h>
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::SpinForwardCritic, dwb_core::TrajectoryCritic)

using nav2_util::declare_parameter_if_not_declared;

namespace dwb_critics
{

void SpinForwardCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".penalty", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".strafe_x", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".strafe_theta",
    rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".theta_scale",
    rclcpp::ParameterValue(10.0));

  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".penalty", penalty_);
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".strafe_x", strafe_x_);
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".strafe_theta", strafe_theta_);
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".theta_scale", theta_scale_);
}

double PreferForwardCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  // backward motions bad on a robot without backward sensors
  if (traj.velocity.x < 0.0) {
    return penalty_;
  }
  // strafing motions also bad on such a robot
  if (traj.velocity.x < strafe_x_ && fabs(traj.velocity.theta) < strafe_theta_) {
    return penalty_;
  }

  // the more we rotate, the less we progress forward
  return fabs(traj.velocity.theta) * theta_scale_;
}

}  // namespace dwb_critics
