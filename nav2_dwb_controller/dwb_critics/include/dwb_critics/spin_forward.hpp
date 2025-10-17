#ifndef DWB_CRITICS__SPIN_FORWARD_HPP_
#define DWB_CRITICS__SPIN_FORWARD_HPP_

#include "dwb_core/trajectory_critic.hpp"

namespace dwb_critics
{
/**
 * @class SpinForwardCritic
 * @brief Favor spin-forward (forward motion with concurrent yaw) by minimizing curvature deviation (κ=|Δθ|/Δs) and penalizing backward/low-progress motion; optionally disabled near goal.
 *
 * This class provides a cost based on how much a robot "twirls" on its way to the goal. With
 * differential-drive robots, there isn't a choice, but with holonomic or near-holonomic robots,
 * sometimes a robot spins more than you'd like on its way to a goal. This class provides a way
 * to assign a penalty purely to rotational velocities.
 */
class SpinForwardCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
};
}  // namespace dwb_critics

#endif  // DWB_CRITICS__SPIN_FORWARD_HPP_