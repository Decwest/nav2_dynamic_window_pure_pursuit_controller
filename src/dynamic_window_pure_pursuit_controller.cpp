// Copyright (c) 2025 Fumiya Ohnishi
// SPDX-License-Identifier: Apache-2.0

#include "nav2_dynamic_window_pure_pursuit_controller/dynamic_window_pure_pursuit_controller.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/exceptions.hpp"

namespace dwpp = nav2_dynamic_window_pure_pursuit_controller;
namespace rpp = nav2_regulated_pure_pursuit_controller;

using nav2_util::declare_parameter_if_not_declared;

namespace nav2_dynamic_window_pure_pursuit_controller
{

void DynamicWindowPurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // First, call configure in the parent class to initialise all functions of the RPP
  rpp::RegulatedPurePursuitController::configure(parent, name, tf, costmap_ros);

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error(
            "Failed to lock parent node in DynamicWindowPurePursuitController::configure");
  }

  // Additional parameters
  declare_parameter_if_not_declared(
    node.get(), name + ".max_linear_vel",
    rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node.get(), name + ".min_linear_vel",
    rclcpp::ParameterValue(-0.0));
  declare_parameter_if_not_declared(
    node.get(), name + ".max_angular_vel",
    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node.get(), name + ".min_angular_vel",
    rclcpp::ParameterValue(-1.0));
  declare_parameter_if_not_declared(
    node.get(), name + ".max_linear_accel",
    rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node.get(), name + ".max_linear_decel",
    rclcpp::ParameterValue(-0.5));
  declare_parameter_if_not_declared(
    node.get(), name + ".max_angular_accel",
    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node.get(), name + ".max_angular_decel",
    rclcpp::ParameterValue(-1.0));

  node->get_parameter(name + ".max_linear_vel", max_linear_vel_);
  desired_linear_vel_ = max_linear_vel_;
  base_desired_linear_vel_ = max_linear_vel_;
  node->get_parameter(name + ".min_linear_vel", min_linear_vel_);
  node->get_parameter(name + ".max_angular_vel", max_angular_vel_);
  rotate_to_heading_angular_vel_ = max_angular_vel_;
  node->get_parameter(name + ".min_angular_vel", min_angular_vel_);
  node->get_parameter(name + ".max_linear_accel", max_linear_accel_);
  node->get_parameter(name + ".max_linear_decel", max_linear_decel_);
  node->get_parameter(name + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(name + ".max_angular_decel", max_angular_decel_);

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

    // Conventional Pure Pursuit:
    // Apply curvature to angular velocity after constraining linear velocity
    // angular_vel = linear_vel * curvature;

    // Dynamic Window Pure Pursuit:
    // compute optimal path tracking velocity commands
    // considering velocity and acceleration constraints (DWPP)

    const double regulated_linear_vel = linear_vel;
    // using last command velocity as a current velocity
    const geometry_msgs::msg::Twist current_speed = last_command_velocity_;

    std::tie(linear_vel, angular_vel) =
      DynamicWindowPurePursuitController::computeDynamicWindowVelocities(
      current_speed,
      max_linear_vel_,
      min_linear_vel_,
      max_angular_vel_,
      min_angular_vel_,
      max_linear_accel_,
      max_linear_decel_,
      max_angular_accel_,
      max_angular_decel_,
      regulated_linear_vel,
      curvature,
      sign,
      control_duration_);
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

  // For dynamic window scaling in open-loop speed control
  last_command_velocity_ = cmd_vel.twist;

  return cmd_vel;
}

void DynamicWindowPurePursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "dynamic_window_pure_pursuit_controller::DynamicWindowPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  carrot_arc_pub_->on_deactivate();
  dyn_params_handler_.reset();
  last_command_velocity_ = geometry_msgs::msg::Twist();
}

}  // namespace nav2_dynamic_window_pure_pursuit_controller

// pluginlib registration
PLUGINLIB_EXPORT_CLASS(
  nav2_dynamic_window_pure_pursuit_controller::DynamicWindowPurePursuitController,
  nav2_core::Controller)
