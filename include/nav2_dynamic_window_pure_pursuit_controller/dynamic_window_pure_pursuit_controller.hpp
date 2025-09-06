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

  void computeOptimalVelocityUsingDynamicWindow(
    const double curvature,
    const geometry_msgs::msg::Twist current_speed,
    const double regulated_linear_vel,
    double & optimal_linear_vel,
    double & optimal_angular_vel
  );

private:
  // 追加パラメータ
  double desired_angular_vel_{0.5};   // new!
  double max_linear_accel_{1.0};      // new!
  std::string velocity_feedback_{"OPEN_LOOP"};      // new!
  geometry_msgs::msg::Twist last_command_velocity_;
  // 既存RPPの desired_linear_vel / max_angular_accel は親が保持
  // ここではオーバーライド時に取得＆利用する（configure内でget_parameter）
};

}  // namespace nav2_dynamic_window_pure_pursuit_controller
