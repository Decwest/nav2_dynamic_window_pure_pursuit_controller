// Copyright (c) 2025 Fumiya Ohnishi
// SPDX-License-Identifier: Apache-2.0

#include "nav2_dynamic_window_pure_pursuit_controller/dynamic_window_pure_pursuit_controller.hpp"

namespace dwpp = nav2_dynamic_window_pure_pursuit_controller;
namespace rpp = nav2_regulated_pure_pursuit_controller;

using nav2_util::declare_parameter_if_not_declared;

namespace nav2_dynamic_window_pure_pursuit_controller
{

void DynamicWindowPurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  // Handles storage and dynamic configuration of parameters.
  // Returns pointer to data current param settings.
  param_handler_ = std::make_unique<rpp::ParameterHandler>(
    node, plugin_name_, logger_, costmap_->getSizeInMetersX());
  params_ = param_handler_->getParams();

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
  params_->desired_linear_vel = max_linear_vel_;
  params_->base_desired_linear_vel = max_linear_vel_;
  node->get_parameter(name + ".min_linear_vel", min_linear_vel_);
  node->get_parameter(name + ".max_angular_vel", max_angular_vel_);
  params_->rotate_to_heading_angular_vel = max_angular_vel_;
  node->get_parameter(name + ".min_angular_vel", min_angular_vel_);
  node->get_parameter(name + ".max_linear_accel", max_linear_accel_);
  node->get_parameter(name + ".max_linear_decel", max_linear_decel_);
  node->get_parameter(name + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(name + ".max_angular_decel", max_angular_decel_);

  // Handles global path transformations
  path_handler_ = std::make_unique<rpp::PathHandler>(
    tf2::durationFromSec(params_->transform_tolerance), tf_, costmap_ros_);

  // Checks for imminent collisions
  collision_checker_ = std::make_unique<rpp::CollisionChecker>(node, costmap_ros_, params_);

  double control_frequency = 20.0;
  goal_dist_tol_ = 0.25;  // reasonable default before first update

  node->get_parameter("controller_frequency", control_frequency);
  control_duration_ = 1.0 / control_frequency;

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  curvature_carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "curvature_lookahead_point", 1);
  is_rotating_to_heading_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "is_rotating_to_heading", 1);
}

double calculateCurvature(geometry_msgs::msg::Point lookahead_point)
{
  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 =
    (lookahead_point.x * lookahead_point.x) +
    (lookahead_point.y * lookahead_point.y);

  // Find curvature of circle (k = 1 / R)
  if (carrot_dist2 > 0.001) {
    return 2.0 * lookahead_point.y / carrot_dist2;
  } else {
    return 0.0;
  }
}

geometry_msgs::msg::TwistStamped DynamicWindowPurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

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
  auto transformed_plan = path_handler_->transformGlobalPlan(
    pose, params_->max_robot_pose_search_dist, params_->interpolate_curvature_after_goal);
  global_path_pub_->publish(transformed_plan);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);
  double curv_lookahead_dist = params_->curvature_lookahead_dist;

  // Check for reverse driving
  if (params_->allow_reversing) {
    // Cusp check
    const double dist_to_cusp = findVelocitySignChange(transformed_plan);

    // if the lookahead distance is further than the cusp, use the cusp distance instead
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
    if (dist_to_cusp < curv_lookahead_dist) {
      curv_lookahead_dist = dist_to_cusp;
    }
  }

  // Get the particular point on the path at the lookahead distance
  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  auto rotate_to_path_carrot_pose = carrot_pose;
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double linear_vel, angular_vel;

  double lookahead_curvature = calculateCurvature(carrot_pose.pose.position);

  double regulation_curvature = lookahead_curvature;
  if (params_->use_fixed_curvature_lookahead) {
    auto curvature_lookahead_pose = getLookAheadPoint(
      curv_lookahead_dist,
      transformed_plan, params_->interpolate_curvature_after_goal);
    rotate_to_path_carrot_pose = curvature_lookahead_pose;
    regulation_curvature = calculateCurvature(curvature_lookahead_pose.pose.position);
    curvature_carrot_pub_->publish(createCarrotMsg(curvature_lookahead_pose));
  }

  // Setting the velocity direction
  double x_vel_sign = 1.0;
  if (params_->allow_reversing) {
    x_vel_sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  linear_vel = params_->desired_linear_vel;

  // Make sure we're in compliance with basic constraints
  // For shouldRotateToPath, using x_vel_sign in order to support allow_reversing
  // and rotate_to_path_carrot_pose for the direction carrot pose:
  //        - equal to "normal" carrot_pose when curvature_lookahead_pose = false
  //        - otherwise equal to curvature_lookahead_pose (which can be interpolated after goal)
  double angle_to_heading;
  if (shouldRotateToGoalHeading(carrot_pose)) {
    is_rotating_to_heading_ = true;
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
  } else if (shouldRotateToPath(rotate_to_path_carrot_pose, angle_to_heading, x_vel_sign)) {
    is_rotating_to_heading_ = true;
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
  } else {
    is_rotating_to_heading_ = false;
    applyConstraints(
      regulation_curvature, speed,
      collision_checker_->costAtPose(pose.pose.position.x, pose.pose.position.y), transformed_plan,
      linear_vel, x_vel_sign);

    if (cancelling_) {
      const double & dt = control_duration_;
      linear_vel = speed.linear.x - x_vel_sign * dt * params_->cancel_deceleration;

      if (x_vel_sign > 0) {
        if (linear_vel <= 0) {
          linear_vel = 0;
          finished_cancelling_ = true;
        }
      } else {
        if (linear_vel >= 0) {
          linear_vel = 0;
          finished_cancelling_ = true;
        }
      }
    }

    // Conventional Pure Pursuit:
    // Apply curvature to angular velocity after constraining linear velocity
    // angular_vel = linear_vel * regulation_curvature;

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
      regulation_curvature,
      x_vel_sign,
      control_duration_);
  }

  // Collision checking on this velocity heading
  const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  if (params_->use_collision_detection &&
    collision_checker_->isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist))
  {
    throw nav2_core::NoValidControl("DynamicWindowPurePursuitController detected collision ahead!");
  }

  // Publish whether we are rotating to goal heading
  std_msgs::msg::Bool is_rotating_to_heading_msg;
  is_rotating_to_heading_msg.data = is_rotating_to_heading_;
  is_rotating_to_heading_pub_->publish(is_rotating_to_heading_msg);

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
  curvature_carrot_pub_->on_deactivate();
  is_rotating_to_heading_pub_->on_deactivate();
  last_command_velocity_ = geometry_msgs::msg::Twist();
}

void DynamicWindowPurePursuitController::reset()
{
  cancelling_ = false;
  finished_cancelling_ = false;
  has_reached_xy_tolerance_ = false;
  last_command_velocity_ = geometry_msgs::msg::Twist();
}

}  // namespace nav2_dynamic_window_pure_pursuit_controller

// pluginlib registration
PLUGINLIB_EXPORT_CLASS(
  nav2_dynamic_window_pure_pursuit_controller::DynamicWindowPurePursuitController,
  nav2_core::Controller)
