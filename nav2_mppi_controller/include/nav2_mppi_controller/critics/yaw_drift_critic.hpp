// Copyright (c) 2026 PIT-IN
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__YAW_DRIFT_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__YAW_DRIFT_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::YawDriftCritic
 * @brief Penalize trajectories whose body yaw drifts from the current robot yaw.
 *        Provides default "hold current yaw" force for omni robots so that
 *        lateral + forward motion is preferred over body rotation in wide spaces.
 *        Disabled near goal so GoalYawCritic / GoalAngleCritic can take over.
 */
class YawDriftCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float threshold_to_consider_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__YAW_DRIFT_CRITIC_HPP_
