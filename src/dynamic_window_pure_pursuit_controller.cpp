// Copyright (c) 2025 Fumiya Onishi
// SPDX-License-Identifier: Apache-2.0

#include "nav2_dynamic_window_pure_pursuit_controller/dynamic_window_pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/exceptions.hpp"

namespace dwpp = nav2_dynamic_window_pure_pursuit_controller;
namespace rpp  = nav2_regulated_pure_pursuit_controller;

using nav2_util::declare_parameter_if_not_declared;

namespace nav2_dynamic_window_pure_pursuit_controller
{

void DynamicWindowPurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // まず親クラスのconfigureを呼び、RPPの全機能を初期化
  rpp::RegulatedPurePursuitController::configure(parent, name, tf, costmap_ros);

  RCLCPP_INFO(
    logger_,
    "========== Dynamic Window Pure Pursuit Controller =========="
    );

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock parent node in DynamicWindowPurePursuitController::configure");
  }

  // 追加パラメータ（新規）
  declare_parameter_if_not_declared(node.get(), name + ".desired_angular_vel",
                                    rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node.get(), name + ".max_linear_accel",
                                    rclcpp::ParameterValue(0.5));

  node->get_parameter(name + ".desired_angular_vel", desired_angular_vel_);
  node->get_parameter(name + ".max_linear_accel",    max_linear_accel_);

  // 既存パラメータ（RPP側）
  // desired_linear_vel, max_angular_accel 等は親クラスが既に取得済み。
  // 必要なら node->get_parameter(name + ".desired_linear_vel", desired_linear_vel_); のように参照可
}

geometry_msgs::msg::TwistStamped DynamicWindowPurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);

  // Check for reverse driving
  if (allow_reversing_) {
    // Cusp check
    double dist_to_cusp = findVelocitySignChange(transformed_plan);

    // if the lookahead distance is further than the cusp, use the cusp distance instead
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
  }

  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double linear_vel, angular_vel;

  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 =
    (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
    (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

  // Find curvature of circle (k = 1 / R)
  double curvature = 0.0;
  if (carrot_dist2 > 0.001) {
    curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
  }

  // Setting the velocity direction
  double sign = 1.0;
  if (allow_reversing_) {
    sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  linear_vel = desired_linear_vel_;

  // Make sure we're in compliance with basic constraints
  double angle_to_heading;
  if (shouldRotateToGoalHeading(carrot_pose)) {
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
  } else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
  } else {
    applyConstraints(
      curvature, speed,
      costAtPose(pose.pose.position.x, pose.pose.position.y), transformed_plan,
      linear_vel, sign);

    // Apply curvature to angular velocity after constraining linear velocity
    angular_vel = linear_vel * curvature;
  }

  // Collision checking on this velocity heading
  const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  if (use_collision_detection_ && isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist)) {
    throw nav2_core::PlannerException("DynamicWindowPurePursuitController detected collision ahead!");
  }

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  return cmd_vel;
}

}  // namespace nav2_dynamic_window_pure_pursuit_controller

// pluginlib 登録
PLUGINLIB_EXPORT_CLASS(
  nav2_dynamic_window_pure_pursuit_controller::DynamicWindowPurePursuitController,
  nav2_core::Controller)
