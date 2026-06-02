// Copyright (c) 2026 PIT-IN
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "nav2_mppi_controller/critics/goal_yaw_critic.hpp"
#include "angles/angles.h"

namespace mppi::critics
{

using xt::evaluation_strategy::immediate;

void GoalYawCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 10.0f);
  getParam(d_ref_, "d_ref", 1.0f);

  if (d_ref_ <= 0.0f) {
    d_ref_ = 1.0f;
    RCLCPP_WARN(
      logger_, "GoalYawCritic d_ref must be > 0; clamped to 1.0");
  }

  RCLCPP_INFO(
    logger_,
    "GoalYawCritic instantiated with %d power, %f weight, %f d_ref.",
    power_, weight_, d_ref_);
}

void GoalYawCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  const float dx = static_cast<float>(
    data.state.pose.pose.position.x - data.goal.position.x);
  const float dy = static_cast<float>(
    data.state.pose.pose.position.y - data.goal.position.y);
  const float dist = std::hypot(dx, dy);

  const float w_eff = weight_ / (1.0f + dist / d_ref_);

  const auto goal_idx = data.path.x.shape(0) - 1;
  const float goal_yaw = data.path.yaws(goal_idx);

  auto yaw_deltas = xt::eval(
    utils::shortest_angular_distance(data.trajectories.yaws, goal_yaw));

  if (power_ > 1u) {
    data.costs += xt::pow(
      xt::mean(yaw_deltas * yaw_deltas, {1}, immediate) * w_eff, power_);
  } else {
    data.costs += xt::mean(yaw_deltas * yaw_deltas, {1}, immediate) * w_eff;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::GoalYawCritic,
  mppi::critics::CriticFunction)
