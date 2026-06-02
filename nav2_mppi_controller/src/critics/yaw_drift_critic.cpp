// Copyright (c) 2026 PIT-IN
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "nav2_mppi_controller/critics/yaw_drift_critic.hpp"

namespace mppi::critics
{

void YawDriftCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 5.0f);
  getParam(threshold_to_consider_, "threshold_to_consider", 0.4f);

  RCLCPP_INFO(
    logger_,
    "YawDriftCritic instantiated with %d power, %f weight, %f goal threshold.",
    power_, weight_, threshold_to_consider_);
}

void YawDriftCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;
  if (!enabled_ || utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, data.goal))
  {
    return;
  }

  const float current_yaw =
    static_cast<float>(tf2::getYaw(data.state.pose.pose.orientation));

  auto yaw_deltas = xt::eval(
    utils::shortest_angular_distance(data.trajectories.yaws, current_yaw));

  if (power_ > 1u) {
    data.costs += xt::pow(
      xt::mean(yaw_deltas * yaw_deltas, {1}, immediate) * weight_, power_);
  } else {
    data.costs += xt::mean(yaw_deltas * yaw_deltas, {1}, immediate) * weight_;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::YawDriftCritic,
  mppi::critics::CriticFunction)
