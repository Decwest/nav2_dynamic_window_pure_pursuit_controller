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

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker) override;


  /**
   * @brief Compute the dynamic window (feasible velocity bounds) based on the current speed and the given velocity and acceleration constraints.
   * @param current_speed  Current linear and angular velocity of the robot
   * @param dynamic_window_max_linear_vel  Computed upper bound of the linear velocity within the dynamic window
   * @param dynamic_window_min_linear_vel  Computed lower bound of the linear velocity within the dynamic window
   * @param dynamic_window_max_angular_vel Computed upper bound of the angular velocity within the dynamic window
   * @param dynamic_window_min_angular_vel Computed lower bound of the angular velocity within the dynamic window
   */
  void computeDynamicWindow(
    const geometry_msgs::msg::Twist & current_speed,
    double & dynamic_window_max_linear_vel,
    double & dynamic_window_min_linear_vel,
    double & dynamic_window_max_angular_vel,
    double & dynamic_window_min_angular_vel
  );

  /**
   * @brief Apply regulated linear velocity to the dynamic window
   * @param regulated_linear_vel   Regulated linear velocity
   * @param dynamic_window_max_linear_vel  Computed upper bound of the linear velocity within the dynamic window
   * @param dynamic_window_min_linear_vel  Computed lower bound of the linear velocity within the dynamic window
   */
  void applyRegulationToDynamicWindow(
    const double & regulated_linear_vel,
    double & dynamic_window_max_linear_vel,
    double & dynamic_window_min_linear_vel);

  /**
   * @brief Compute the optimal velocity to follow the path within the dynamic window
   * @param dynamic_window_max_linear_vel  Computed upper bound of the linear velocity within the dynamic window
   * @param dynamic_window_min_linear_vel  Computed lower bound of the linear velocity within the dynamic window
   * @param dynamic_window_max_angular_vel Computed upper bound of the angular velocity within the dynamic window
   * @param dynamic_window_min_angular_vel Computed lower bound of the angular velocity within the dynamic window
   * @param curvature      Curvature of the path to follow
   * @param sign           Velocity sign (forward or backward)
   * @param optimal_linear_vel   Optimal linear velocity to follow the path under velocity and acceleration constraints
   * @param optimal_angular_vel   Optimal angular velocity to follow the path under velocity and acceleration constraints
   */
  void computeOptimalVelocityWithinDynamicWindow(
    const double & dynamic_window_max_linear_vel,
    const double & dynamic_window_min_linear_vel,
    const double & dynamic_window_max_angular_vel,
    const double & dynamic_window_min_angular_vel,
    const double & curvature,
    const double & sign,
    double & optimal_linear_vel,
    double & optimal_angular_vel
  );

private:
  // Additional parameters
  double max_linear_vel_{0.5};
  double min_linear_vel_{0.0};
  double max_angular_vel_{1.0};
  double min_angular_vel_{-1.0};
  double max_linear_accel_{0.5};
  double max_linear_decel_{0.5};
  double max_angular_accel_{1.0};
  double max_angular_decel_{1.0};
  std::string velocity_feedback_{"OPEN_LOOP"};
  geometry_msgs::msg::Twist last_command_velocity_;
};

}  // namespace nav2_dynamic_window_pure_pursuit_controller
