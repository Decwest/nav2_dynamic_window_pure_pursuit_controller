// Copyright (c) 2025 Fumiya Onishi
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
                                    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node.get(), name + ".max_linear_accel",
                                    rclcpp::ParameterValue(0.5));

  node->get_parameter(name + ".desired_angular_vel", desired_angular_vel_);
  node->get_parameter(name + ".max_linear_accel",    max_linear_accel_);

  // 既存パラメータ（RPP側）
  // desired_linear_vel, max_angular_accel 等は親クラスが既に取得済み。
  // 必要なら node->get_parameter(name + ".desired_linear_vel", desired_linear_vel_); のように参照可
}


void DynamicWindowPurePursuitController::computeOptimalVelocityUsingDynamicWindow(
    const double curvature,
    const geometry_msgs::msg::Twist current_speed,
    const double regulated_linear_vel,
    double & optimal_linear_vel,
    double & optimal_angular_vel
  )
{
  // ---- Parameters (assumed available in this scope) ----
  const double A_MAX  = max_linear_accel_;      // A_MAX
  const double V_MAX  = desired_linear_vel_;    // V_MAX
  const double AW_MAX = max_angular_accel_;     // AW_MAX
  const double W_MAX  = desired_angular_vel_;   // W_MAX
  const double DT     = control_duration_;     // DT

  // 対称上下限（必要に応じて置換）
  const double V_MIN = -V_MAX;
  const double W_MIN = -W_MAX;

  // ---- 1) Dynamic Window ----
  double dw_vmax = std::min(current_speed.linear.x  + A_MAX  * DT, V_MAX);
  const double dw_vmin = std::max(current_speed.linear.x  - A_MAX  * DT, V_MIN);
  double dw_wmax = std::min(current_speed.angular.z + AW_MAX * DT, W_MAX);
  const double dw_wmin = std::max(current_speed.angular.z - AW_MAX * DT, W_MIN);

  // regulated v を反映（上限を絞る）
  if (dw_vmax > regulated_linear_vel) {
    dw_vmax = std::max(dw_vmin, regulated_linear_vel);
  }

  const double k = curvature;

  // ---- 早期決定: 曲率が 0（w = 0） ----
  if (k == 0.0) {
    // w=0 がDW内なら、そのまま最大並進速度を採用
    if (dw_wmin <= 0.0 && 0.0 <= dw_wmax) {
      optimal_linear_vel  = dw_vmax; // 常に最大v
      optimal_angular_vel = 0.0;
      return;
    }
    // w=0が外なら、|w|が小さい側を選ぶ
    const double w_choice = (std::abs(dw_wmin) <= std::abs(dw_wmax)) ? dw_wmin : dw_wmax;
    optimal_linear_vel  = dw_vmax;   // 最大v
    optimal_angular_vel = w_choice;
    return;
  }

  // ---- 2) 交点のうちDW内にある候補から「最大v」を1パスで選ぶ ----
  double best_v = -1e300;     // 最大化の初期値
  double best_w = 0.0;

  // 垂直辺との交点
  {
    const double v1 = dw_vmin;
    const double w1 = k * v1;
    if (w1 >= dw_wmin && w1 <= dw_wmax) {
      if (v1 > best_v) { best_v = v1; best_w = w1; }
    }
  }
  {
    const double v2 = dw_vmax;
    const double w2 = k * v2;
    if (w2 >= dw_wmin && w2 <= dw_wmax) {
      if (v2 > best_v) { best_v = v2; best_w = w2; }
    }
  }

  // 水平辺との交点（k != 0）
  {
    const double v3 = dw_wmin / k;
    if (v3 >= dw_vmin && v3 <= dw_vmax) {
      const double w3 = dw_wmin;
      if (v3 > best_v) { best_v = v3; best_w = w3; }
    }
  }
  {
    const double v4 = dw_wmax / k;
    if (v4 >= dw_vmin && v4 <= dw_vmax) {
      const double w4 = dw_wmax;
      if (v4 > best_v) { best_v = v4; best_w = w4; }
    }
  }

  if (best_v > -1e290) {
    // 交点が見つかった → 最大vを採用
    optimal_linear_vel  = best_v;
    optimal_angular_vel = best_w;
    return;
  }

  // ---- 3) 交点がない場合：4頂点のうち線 w = k v へのユークリッド距離が最小のもの
  struct Corner { double v; double w; };
  const Corner corners[4] = {
    {dw_vmin, dw_wmin},
    {dw_vmin, dw_wmax},
    {dw_vmax, dw_wmin},
    {dw_vmax, dw_wmax}
  };

  const double denom = std::sqrt(k * k + 1.0); // 一度だけsqrt

  auto euclid_dist = [&](const Corner &c) -> double {
    // 点 (v, w) と直線 w - k v = 0 の距離
    return std::abs(k * c.v - c.w) / denom;
  };

  double best_dist = 1e300;
  best_v = corners[0].v;
  best_w = corners[0].w;

  for (int i = 0; i < 4; ++i) {
    const double d = euclid_dist(corners[i]);
    // 1) より距離が小さい → 採用
    // 2) 距離が等しい（~1e-12）→ v が大きい方（加速方針）
    if (d < best_dist || (std::abs(d - best_dist) <= 1e-12 && corners[i].v > best_v)) {
      best_dist = d;
      best_v = corners[i].v;
      best_w = corners[i].w;
    }
  }

  optimal_linear_vel  = best_v;
  optimal_angular_vel = best_w;
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
    // Apply curvature to angular velocity after constraining linear velocity
    // angular_vel = linear_vel * curvature;

    // After here is original DWPP algorithm!!!
    const double regulated_linear_vel = linear_vel;
    computeOptimalVelocityUsingDynamicWindow(curvature, speed, regulated_linear_vel, linear_vel, angular_vel);

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
