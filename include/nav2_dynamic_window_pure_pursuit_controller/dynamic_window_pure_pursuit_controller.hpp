// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
// Copyright (c) 2025 Fumiya Ohnishi
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <tuple>
#include <utility>
#include <limits>

#include "angles/angles.h"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using namespace nav2_costmap_2d;  // NOLINT

namespace nav2_dynamic_window_pure_pursuit_controller
{

class DynamicWindowPurePursuitController
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  DynamicWindowPurePursuitController() = default;
  ~DynamicWindowPurePursuitController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  void reset() override;

  struct DynamicWindowBounds
  {
    double max_linear_vel;
    double min_linear_vel;
    double max_angular_vel;
    double min_angular_vel;
  };

/**
 * @brief Compute the dynamic window (feasible velocity bounds) based on
 *        the current speed and the given velocity and acceleration constraints.
 * @param current_speed     Current linear and angular velocity of the robot
 * @param max_linear_vel    Maximum allowable linear velocity
 * @param min_linear_vel    Minimum allowable linear velocity
 * @param max_angular_vel   Maximum allowable angular velocity
 * @param min_angular_vel   Minimum allowable angular velocity
 * @param max_linear_accel  Maximum allowable linear acceleration
 * @param max_linear_decel  Maximum allowable linear deceleration
 * @param max_angular_accel Maximum allowable angular acceleration
 * @param max_angular_decel Maximum allowable angular deceleration
 * @param dt                Control duration
 * @return                  Computed dynamic window's velocity bounds
 */
  inline DynamicWindowBounds computeDynamicWindow(
    const geometry_msgs::msg::Twist & current_speed,
    const double & max_linear_vel,
    const double & min_linear_vel,
    const double & max_angular_vel,
    const double & min_angular_vel,
    const double & max_linear_accel,
    const double & max_linear_decel,
    const double & max_angular_accel,
    const double & max_angular_decel,
    const double & dt
  )
  {
    DynamicWindowBounds dynamic_window;
    constexpr double Eps = 1e-3;

    // function to compute dynamic window for a single dimension
    auto compute_window = [&](const double & current_vel, const double & max_vel,
        const double & min_vel, const double & max_accel, const double & max_decel)
      {
        double candidate_max_vel = 0.0;
        double candidate_min_vel = 0.0;

        if (current_vel > Eps) {
          // if the current velocity is positive, acceleration means an increase in speed
          candidate_max_vel = current_vel + max_accel * dt;
          candidate_min_vel = current_vel + max_decel * dt;
        } else if (current_vel < -Eps) {
          // if the current velocity is negative, acceleration means a decrease in speed
          candidate_max_vel = current_vel - max_decel * dt;
          candidate_min_vel = current_vel - max_accel * dt;
        } else {
          // if the current velocity is zero, allow acceleration in both directions.
          candidate_max_vel = current_vel + max_accel * dt;
          candidate_min_vel = current_vel - max_accel * dt;
        }

        // clip to max/min velocity limits
        double dynamic_window_max_vel = std::min(candidate_max_vel, max_vel);
        double dynamic_window_min_vel = std::max(candidate_min_vel, min_vel);
        return std::make_tuple(dynamic_window_max_vel, dynamic_window_min_vel);
      };

    // linear velocity
    std::tie(dynamic_window.max_linear_vel, dynamic_window.min_linear_vel) =
      compute_window(
      current_speed.linear.x, max_linear_vel, min_linear_vel,
      max_linear_accel, max_linear_decel);

    // angular velocity
    std::tie(dynamic_window.max_angular_vel, dynamic_window.min_angular_vel) =
      compute_window(
      current_speed.angular.z, max_angular_vel, min_angular_vel,
      max_angular_accel, max_angular_decel);

    return dynamic_window;
  }

/**
 * @brief                        Apply regulated linear velocity to the dynamic window
 * @param regulated_linear_vel   Regulated linear velocity
 * @param dynamic_window         Dynamic window to be regulated
 */
  inline void applyRegulationToDynamicWindow(
    const double & regulated_linear_vel,
    DynamicWindowBounds & dynamic_window)
  {
    // Create regulated bounds [0, v_reg] or [v_reg, 0]
    double v_reg_min = std::min(0.0, regulated_linear_vel);
    double v_reg_max = std::max(0.0, regulated_linear_vel);

    // Intersect the dynamic window with the regulated bounds
    dynamic_window.min_linear_vel = std::max(dynamic_window.min_linear_vel, v_reg_min);
    dynamic_window.max_linear_vel = std::min(dynamic_window.max_linear_vel, v_reg_max);

    // If min > max, collapse to the nearest boundary
    if (dynamic_window.min_linear_vel > dynamic_window.max_linear_vel) {
      if (dynamic_window.min_linear_vel > v_reg_max) {
        dynamic_window.max_linear_vel = dynamic_window.min_linear_vel;
      } else {
        dynamic_window.min_linear_vel = dynamic_window.max_linear_vel;
      }
    }
  }

/**
 * @brief                Compute the optimal velocity to follow the path within the dynamic window
 * @param dynamic_window Dynamic window defining feasible velocity bounds
 * @param curvature      Curvature of the path to follow
 * @param sign           Velocity sign (forward or backward)
 * @return               Optimal linear and angular velocity
 */
  inline std::tuple<double, double> computeOptimalVelocityWithinDynamicWindow(
    const DynamicWindowBounds & dynamic_window,
    const double & curvature,
    const double & sign
  )
  {
    double optimal_linear_vel;
    double optimal_angular_vel;

    // consider linear_vel - angular_vel space (horizontal and vertical axes respectively)
    // Select the closest point to the line
    // angular_vel = curvature * linear_vel within the dynamic window.
    // If multiple points are equally close, select the one with the largest linear_vel.

    // When curvature == 0, the line is angular_vel = 0
    if (abs(curvature) < 1e-3) {
      // linear velocity
      if (sign >= 0.0) {
        // If moving forward, select the max linear vel
        optimal_linear_vel = dynamic_window.max_linear_vel;
      } else {
        // If moving backward, select the min linear vel
        optimal_linear_vel = dynamic_window.min_linear_vel;
      }

      // angular velocity
      // If the line angular_vel = 0 intersects the dynamic window,angular_vel = 0.0
      if (dynamic_window.min_angular_vel <= 0.0 && 0.0 <= dynamic_window.max_angular_vel) {
        optimal_angular_vel = 0.0;
      } else {
        // If not, select angular vel within dynamic window closest to 0
        if (std::abs(dynamic_window.min_angular_vel) <= std::abs(dynamic_window.max_angular_vel)) {
          optimal_angular_vel = dynamic_window.min_angular_vel;
        } else {
          optimal_angular_vel = dynamic_window.max_angular_vel;
        }
      }
      return std::make_tuple(optimal_linear_vel, optimal_angular_vel);
    }

    // When the dynamic window and the line angular_vel = curvature * linear_vel intersect,
    // select the intersection point that yields the highest linear velocity.

    // List the four candidate intersection points
    std::pair<double, double> candidates[] = {
      {dynamic_window.min_linear_vel, curvature * dynamic_window.min_linear_vel},
      {dynamic_window.max_linear_vel, curvature * dynamic_window.max_linear_vel},
      {dynamic_window.min_angular_vel / curvature, dynamic_window.min_angular_vel},
      {dynamic_window.max_angular_vel / curvature, dynamic_window.max_angular_vel}
    };

    double best_linear_vel = -std::numeric_limits<double>::max() * sign;
    double best_angular_vel = 0.0;

    for (auto [linear_vel, angular_vel] : candidates) {
      // Check whether the candidate lies within the dynamic window
      if (linear_vel >= dynamic_window.min_linear_vel &&
        linear_vel <= dynamic_window.max_linear_vel &&
        angular_vel >= dynamic_window.min_angular_vel &&
        angular_vel <= dynamic_window.max_angular_vel)
      {
        // Select the candidate with the largest linear velocity (considering moving direction)
        if (linear_vel * sign > best_linear_vel * sign) {
          best_linear_vel = linear_vel;
          best_angular_vel = angular_vel;
        }
      }
    }

    // If best_linear_vel was updated, it means that a valid intersection exists
    if (best_linear_vel != -std::numeric_limits<double>::max() * sign) {
      optimal_linear_vel = best_linear_vel;
      optimal_angular_vel = best_angular_vel;
      return std::make_tuple(optimal_linear_vel, optimal_angular_vel);
    }

    // When the dynamic window and the line angular_vel = curvature * linear_vel have no intersection,
    // select the point within the dynamic window that is closest to the line.

    // Because the dynamic window is a convex region,
    // the closest point must be one of its four corners.
    const std::array<std::array<double, 2>, 4> corners = {{
      {dynamic_window.min_linear_vel, dynamic_window.min_angular_vel},
      {dynamic_window.min_linear_vel, dynamic_window.max_angular_vel},
      {dynamic_window.max_linear_vel, dynamic_window.min_angular_vel},
      {dynamic_window.max_linear_vel, dynamic_window.max_angular_vel}
    }};

    // Compute the distance from a point (linear_vel, angular_vel)
    // to the line angular_vel = curvature * linear_vel
    const double denom = std::sqrt(curvature * curvature + 1.0);
    auto compute_dist = [&](const std::array<double, 2> & corner) -> double {
        return std::abs(curvature * corner[0] - corner[1]) / denom;
      };

    double closest_dist = std::numeric_limits<double>::max();
    best_linear_vel = -std::numeric_limits<double>::max() * sign;
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

    return std::make_tuple(optimal_linear_vel, optimal_angular_vel);
  }

/**
 * @brief Compute velocity commands using Dynamic Window Pure Pursuit method
 * @param current_speed         Current linear and angular velocity of the robot
 * @param max_linear_vel        Maximum allowable linear velocity
 * @param min_linear_vel        Minimum allowable linear velocity
 * @param max_angular_vel       Maximum allowable angular velocity
 * @param min_angular_vel       Minimum allowable angular velocity
 * @param max_linear_accel      Maximum allowable linear acceleration
 * @param max_linear_decel      Maximum allowable linear deceleration
 * @param max_angular_accel     Maximum allowable angular acceleration
 * @param max_angular_decel     Maximum allowable angular deceleration
 * @param regulated_linear_vel  Regulated linear velocity
 * @param curvature             Curvature of the path to follow
 * @param sign                  Velocity sign (forward or backward)
 * @param dt                    Control duration
 * @return                      Optimal linear and angular velocity
 */
  inline std::tuple<double, double> computeDynamicWindowVelocities(
    const geometry_msgs::msg::Twist & current_speed,
    const double & max_linear_vel,
    const double & min_linear_vel,
    const double & max_angular_vel,
    const double & min_angular_vel,
    const double & max_linear_accel,
    const double & max_linear_decel,
    const double & max_angular_accel,
    const double & max_angular_decel,
    const double & regulated_linear_vel,
    const double & curvature,
    const double & sign,
    const double & dt
  )
  {
    // compute Dynamic Window
    DynamicWindowBounds dynamic_window = computeDynamicWindow(
      current_speed, max_linear_vel, min_linear_vel, max_angular_vel, min_angular_vel,
      max_linear_accel, max_linear_decel, max_angular_accel, max_angular_decel, dt);

    // apply regulation to Dynamic Window
    applyRegulationToDynamicWindow(regulated_linear_vel, dynamic_window);

    // compute optimal velocity within Dynamic Window
    auto [linear_vel, angular_vel] = computeOptimalVelocityWithinDynamicWindow(
      dynamic_window, curvature, sign);

    return std::make_tuple(linear_vel, angular_vel);
  }

private:
  // Additional parameters
  double max_linear_vel_{0.5};
  double min_linear_vel_{0.0};
  double max_angular_vel_{1.0};
  double min_angular_vel_{-1.0};
  double max_linear_accel_{0.5};
  double max_linear_decel_{-0.5};
  double max_angular_accel_{1.0};
  double max_angular_decel_{-1.0};
  geometry_msgs::msg::Twist last_command_velocity_;
};

}  // namespace nav2_dynamic_window_pure_pursuit_controller
