// Copyright (c) 2025 Fumiya Ohnishi
// SPDX-License-Identifier: Apache-2.0

#include "nav2_dynamic_window_pure_pursuit_controller/dynamic_window_pure_pursuit_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

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
    rclcpp::ParameterValue(0.0));
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
    rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node.get(), name + ".max_angular_accel",
    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node.get(), name + ".max_angular_decel",
    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node.get(), name + ".velocity_feedback",
    rclcpp::ParameterValue("OPEN_LOOP"));
  declare_parameter_if_not_declared(
    node.get(), name + ".use_dynamic_window",
    rclcpp::ParameterValue(true));

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
  node->get_parameter(name + ".velocity_feedback", velocity_feedback_);
  node->get_parameter(name + ".use_dynamic_window", use_dynamic_window_);

  // create publisher
  constraints_violation_flag_publisher_ =
    node->create_publisher<std_msgs::msg::Bool>(
    "constraints_violation_flag", 1);

}

bool DynamicWindowPurePursuitController::evaluateVelocityConstraints(
  const geometry_msgs::msg::Twist & next_cmd_vel,
  const geometry_msgs::msg::Twist & current_cmd_vel)
{
  const double & max_linear_vel = max_linear_vel_;
  const double & min_linear_vel = min_linear_vel_;
  const double & max_angular_vel = max_angular_vel_;
  const double & min_angular_vel = min_angular_vel_;

  const double & max_linear_accel = max_linear_accel_;
  const double & max_linear_decel = max_linear_decel_;
  const double & max_angular_accel = max_angular_accel_;
  const double & max_angular_decel = max_angular_decel_;

  const double & dt = control_duration_;

  constexpr double Eps = 1e-2;

  // function to evaluate velocity constraints for a single dimension
  auto evaluate_velocity =
    [&](const double & current_vel, const double & last_vel, const double & max_vel,
      const double & min_vel,
      const double & max_accel, const double & max_decel)
    {
      double candidate_max_vel = 0.0;
      double candidate_min_vel = 0.0;

      if (last_vel > Eps) {
        // if the last velocity is positive, acceleration means an increase in speed
        candidate_max_vel = last_vel + max_accel * dt;
        candidate_min_vel = last_vel - max_decel * dt;
      } else if (last_vel < -Eps) {
        // if the last velocity is negative, acceleration means a decrease in speed
        candidate_max_vel = last_vel + max_decel * dt;
        candidate_min_vel = last_vel - max_accel * dt;
      } else {
        // if the last velocity is zero, allow acceleration in both directions.
        candidate_max_vel = last_vel + max_accel * dt;
        candidate_min_vel = last_vel - max_accel * dt;
      }

      // clip to max/min velocity limits
      candidate_max_vel = std::min(candidate_max_vel, max_vel);
      candidate_min_vel = std::max(candidate_min_vel, min_vel);

      // check whether current_vel is within [candidate_min_vel, candidate_max_vel]
      if (current_vel > candidate_max_vel + Eps || current_vel < candidate_min_vel - Eps) {
        return true;  // violation
      } else {
        return false;  // no violation
      }
    };
  // linear velocity
  bool linear_violation = evaluate_velocity(
    next_cmd_vel.linear.x,
    current_cmd_vel.linear.x,
    max_linear_vel, min_linear_vel,
    max_linear_accel, max_linear_decel);
  // angular velocity
  bool angular_violation = evaluate_velocity(
    next_cmd_vel.angular.z,
    current_cmd_vel.angular.z,
    max_angular_vel, min_angular_vel,
    max_angular_accel, max_angular_decel);

  return linear_violation || angular_violation;
}

void DynamicWindowPurePursuitController::computeDynamicWindow(
  const geometry_msgs::msg::Twist & current_speed,
  double & dynamic_window_max_linear_vel,
  double & dynamic_window_min_linear_vel,
  double & dynamic_window_max_angular_vel,
  double & dynamic_window_min_angular_vel
)
{
  const double & max_linear_vel = max_linear_vel_;
  const double & min_linear_vel = min_linear_vel_;
  const double & max_angular_vel = max_angular_vel_;
  const double & min_angular_vel = min_angular_vel_;

  const double & max_linear_accel = max_linear_accel_;
  const double & max_linear_decel = max_linear_decel_;
  const double & max_angular_accel = max_angular_accel_;
  const double & max_angular_decel = max_angular_decel_;

  const double & dt = control_duration_;

  constexpr double Eps = 1e-2;

  // function to compute dynamic window for a single dimension
  auto compute_window = [&](const double & current_vel, const double & max_vel,
      const double & min_vel,
      const double & max_accel, const double & max_decel,
      double & dynamic_window_max_vel, double & dynamic_window_min_vel)
    {
      double candidate_max_vel = 0.0;
      double candidate_min_vel = 0.0;

      if (current_vel > Eps) {
        // if the current velocity is positive, acceleration means an increase in speed
        candidate_max_vel = current_vel + max_accel * dt;
        candidate_min_vel = current_vel - max_decel * dt;
      } else if (current_vel < -Eps) {
        // if the current velocity is negative, acceleration means a decrease in speed
        candidate_max_vel = current_vel + max_decel * dt;
        candidate_min_vel = current_vel - max_accel * dt;
      } else {
        // if the current velocity is zero, allow acceleration in both directions.
        candidate_max_vel = current_vel + max_accel * dt;
        candidate_min_vel = current_vel - max_accel * dt;
      }

      // clip to max/min velocity limits
      dynamic_window_max_vel = std::min(candidate_max_vel, max_vel);
      dynamic_window_min_vel = std::max(candidate_min_vel, min_vel);
    };

  // linear velocity
  compute_window(
    current_speed.linear.x,
    max_linear_vel, min_linear_vel,
    max_linear_accel, max_linear_decel,
    dynamic_window_max_linear_vel,
    dynamic_window_min_linear_vel);

  // angular velocity
  compute_window(
    current_speed.angular.z,
    max_angular_vel, min_angular_vel,
    max_angular_accel, max_angular_decel,
    dynamic_window_max_angular_vel,
    dynamic_window_min_angular_vel);
}

void DynamicWindowPurePursuitController::applyRegulationToDynamicWindow(
  const double & regulated_linear_vel,
  double & dynamic_window_max_linear_vel,
  double & dynamic_window_min_linear_vel)
{
  // Extract the portion of the dynamic window that lies within the range [0, regulated_linear_vel]
  double dynamic_window_max_linear_vel_temp;
  double dynamic_window_min_linear_vel_temp;
  if (regulated_linear_vel >= 0.0) {
    dynamic_window_max_linear_vel_temp = std::min(
      dynamic_window_max_linear_vel, regulated_linear_vel);
    dynamic_window_min_linear_vel_temp = std::max(
      dynamic_window_min_linear_vel, 0.0);
  } else {
    dynamic_window_max_linear_vel_temp = std::min(
      dynamic_window_max_linear_vel, 0.0);
    dynamic_window_min_linear_vel_temp = std::max(
      dynamic_window_min_linear_vel, regulated_linear_vel);
  }

  if (dynamic_window_max_linear_vel_temp >= dynamic_window_min_linear_vel_temp) {
    dynamic_window_max_linear_vel = dynamic_window_max_linear_vel_temp;
    dynamic_window_min_linear_vel = dynamic_window_min_linear_vel_temp;
  } else {
    // No valid portion of the dynamic window remains after applying the regulation
    if (dynamic_window_min_linear_vel > 0.0) {
      // If the dynamic window is entirely in the positive range,
      // collapse both bounds to dynamic_window_min_linear_vel
      dynamic_window_max_linear_vel = dynamic_window_min_linear_vel;
    } else {
      // If the dynamic window is entirely in the negative range,
      // collapse both bounds to dynamic_window_max_linear_vel
      dynamic_window_min_linear_vel = dynamic_window_max_linear_vel;
    }
  }

  return;
}

void DynamicWindowPurePursuitController::computeOptimalVelocityWithinDynamicWindow(
  const double & dynamic_window_max_linear_vel,
  const double & dynamic_window_min_linear_vel,
  const double & dynamic_window_max_angular_vel,
  const double & dynamic_window_min_angular_vel,
  const double & curvature,
  const double & sign,
  double & optimal_linear_vel,
  double & optimal_angular_vel
)
{
  // consider linear_vel - angular_vel space (horizontal and vertical axes respectively)
  // Select the closest point to the line
  // angular_vel = curvature * linear_vel within the dynamic window.
  // If multiple points are equally close, select the one with the largest linear_vel.

  // When curvature == 0, the line is angular_vel = 0
  if (abs(curvature) < 1e-3) {
    // linear velocity
    if (sign >= 0.0) {
      // If moving forward, select the max linear vel
      optimal_linear_vel = dynamic_window_max_linear_vel;
    } else {
      // If moving backward, select the min linear vel
      optimal_linear_vel = dynamic_window_min_linear_vel;
    }

    // angular velocity
    // If the line angular_vel = 0 intersects the dynamic window,angular_vel = 0.0
    if (dynamic_window_min_angular_vel <= 0.0 && 0.0 <= dynamic_window_max_angular_vel) {
      optimal_angular_vel = 0.0;
    } else {
      // If not, select angular vel within dynamic window closest to 0
      if (std::abs(dynamic_window_min_angular_vel) <= std::abs(dynamic_window_max_angular_vel)) {
        optimal_angular_vel = dynamic_window_min_angular_vel;
      } else {
        optimal_angular_vel = dynamic_window_max_angular_vel;
      }
    }
    return;
  }

  // When the dynamic window and the line angular_vel = curvature * linear_vel intersect,
  // select the intersection point that yields the highest linear velocity.

  // List the four candidate intersection points
  std::pair<double, double> candidates[] = {
    {dynamic_window_min_linear_vel, curvature * dynamic_window_min_linear_vel},
    {dynamic_window_max_linear_vel, curvature * dynamic_window_max_linear_vel},
    {dynamic_window_min_angular_vel / curvature, dynamic_window_min_angular_vel},
    {dynamic_window_max_angular_vel / curvature, dynamic_window_max_angular_vel}
  };

  double best_linear_vel = -std::numeric_limits<double>::infinity() * sign;
  double best_angular_vel = 0.0;

  for (auto [linear_vel, angular_vel] : candidates) {
    // Check whether the candidate lies within the dynamic window
    if (linear_vel >= dynamic_window_min_linear_vel &&
      linear_vel <= dynamic_window_max_linear_vel &&
      angular_vel >= dynamic_window_min_angular_vel &&
      angular_vel <= dynamic_window_max_angular_vel)
    {
      // Select the candidate with the largest linear velocity (considering moving direction)
      if (linear_vel * sign > best_linear_vel * sign) {
        best_linear_vel = linear_vel;
        best_angular_vel = angular_vel;
      }
    }
  }

  // If best_linear_vel was updated, it means that a valid intersection exists
  if (best_linear_vel != -std::numeric_limits<double>::infinity() * sign) {
    optimal_linear_vel = best_linear_vel;
    optimal_angular_vel = best_angular_vel;
    return;
  }

  // When the dynamic window and the line angular_vel = curvature * linear_vel have no intersection,
  // select the point within the dynamic window that is closest to the line.

  // Because the dynamic window is a convex region,
  // the closest point must be one of its four corners.
  const std::array<std::array<double, 2>, 4> corners = {{
    {dynamic_window_min_linear_vel, dynamic_window_min_angular_vel},
    {dynamic_window_min_linear_vel, dynamic_window_max_angular_vel},
    {dynamic_window_max_linear_vel, dynamic_window_min_angular_vel},
    {dynamic_window_max_linear_vel, dynamic_window_max_angular_vel}
  }};

  // Compute the distance from a point (linear_vel, angular_vel)
  // to the line angular_vel = curvature * linear_vel
  const double denom = std::sqrt(curvature * curvature + 1.0);
  auto compute_dist = [&](const std::array<double, 2> & corner) -> double {
      return std::abs(curvature * corner[0] - corner[1]) / denom;
    };

  double closest_dist = std::numeric_limits<double>::infinity();
  best_linear_vel = -std::numeric_limits<double>::infinity() * sign;
  best_angular_vel = 0.0;

  for (const auto & corner : corners) {
    const double dist = compute_dist(corner);
    // Update if this corner is closer to the line,
    // or equally close but has a larger linear velocity (considering moving direction)
    if (dist < closest_dist ||
      (std::abs(dist - closest_dist) <= 1e-3 && corner[0] * sign > best_linear_vel * sign))
    {
      closest_dist = dist;
      best_linear_vel = corner[0];
      best_angular_vel = corner[1];
    }
  }

  optimal_linear_vel = best_linear_vel;
  optimal_angular_vel = best_angular_vel;
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

    // Conventional Pure Pursuit
    if (!use_dynamic_window_) {
      // Apply curvature to angular velocity
      angular_vel = linear_vel * curvature;
    } else {

      // After here is original DWPP algorithm!!!
      // compute optimal path tracking velocity commands
      // considering velocity and acceleration constraints (DWPP)
      const double regulated_linear_vel = linear_vel;
      geometry_msgs::msg::Twist current_speed;
      if (velocity_feedback_ == "CLOSED_LOOP") {
        // using odom velocity as a current velocity (not recommended)
        current_speed = speed;
      } else {
        // using last command velocity as a current velocity (recommended)
        current_speed = last_command_velocity_;
      }
      double dynamic_window_max_linear_vel, dynamic_window_min_linear_vel,
        dynamic_window_max_angular_vel, dynamic_window_min_angular_vel;

      // compute Dynamic Window
      computeDynamicWindow(
        current_speed,
        dynamic_window_max_linear_vel,
        dynamic_window_min_linear_vel,
        dynamic_window_max_angular_vel,
        dynamic_window_min_angular_vel);

      // apply regulation to Dynamic Window
      applyRegulationToDynamicWindow(
        regulated_linear_vel,
        dynamic_window_max_linear_vel,
        dynamic_window_min_linear_vel);

      // compute optimal velocity within Dynamic Window
      computeOptimalVelocityWithinDynamicWindow(
        dynamic_window_max_linear_vel,
        dynamic_window_min_linear_vel,
        dynamic_window_max_angular_vel,
        dynamic_window_min_angular_vel,
        curvature,
        sign,
        linear_vel,
        angular_vel
      );
    }
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

  // evaluate whether the computed velocity is within velocity and acceleration constraints
  std_msgs::msg::Bool constraints_violation_flag_msg;
  constraints_violation_flag_msg.data = evaluateVelocityConstraints(
    cmd_vel.twist, last_command_velocity_);

  constraints_violation_flag_publisher_->publish(constraints_violation_flag_msg);

  last_command_velocity_ = cmd_vel.twist;

  return cmd_vel;
}

}  // namespace nav2_dynamic_window_pure_pursuit_controller

// pluginlib registration
PLUGINLIB_EXPORT_CLASS(
  nav2_dynamic_window_pure_pursuit_controller::DynamicWindowPurePursuitController,
  nav2_core::Controller)
