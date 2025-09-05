// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
// Copyright (c) 2025 Fumiya Ohnishi
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_dynamic_window_pure_pursuit_controller
{

class DynamicWindowPurePursuitController
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  DynamicWindowPurePursuitController() = default;
  ~DynamicWindowPurePursuitController() override = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker) override;

  // （必要なら）setPlan等は親の実装をそのまま利用
  // using nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::setPlan;

private:
  // 追加パラメータ
  double desired_angular_vel_{0.0};   // new!
  double max_linear_accel_{0.5};      // new!
  // 既存RPPの desired_linear_vel / max_angular_accel は親が保持
  // ここではオーバーライド時に取得＆利用する（configure内でget_parameter）
};

}  // namespace nav2_dynamic_window_pure_pursuit_controller
