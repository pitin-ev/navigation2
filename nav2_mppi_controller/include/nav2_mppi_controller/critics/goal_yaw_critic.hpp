// Copyright (c) 2026 PIT-IN
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__GOAL_YAW_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__GOAL_YAW_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::GoalYawCritic
 * @brief Distance-modulated penalty on trajectory yaw deviation from goal yaw.
 *        Effective weight: w_eff = cost_weight / (1 + dist / d_ref), where
 *        dist = current robot distance to goal. Becomes stronger as the robot
 *        approaches goal, providing a smooth ramp-up from YawDriftCritic.
 */
class GoalYawCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float d_ref_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__GOAL_YAW_CRITIC_HPP_
