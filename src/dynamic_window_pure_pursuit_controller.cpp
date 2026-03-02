// Copyright (c) 2025 Fumiya Ohnishi
// SPDX-License-Identifier: Apache-2.0

#include "nav2_dynamic_window_pure_pursuit_controller/dynamic_window_pure_pursuit_controller.hpp"

#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#if __has_include("nav2_core/controller_exceptions.hpp")
#include "nav2_core/controller_exceptions.hpp"
#elif __has_include("nav2_core/exceptions.hpp")
#include "nav2_core/exceptions.hpp"
#else
#error "Neither nav2_core/controller_exceptions.hpp nor nav2_core/exceptions.hpp was found."
#endif
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"

namespace rpp = nav2_regulated_pure_pursuit_controller;

using nav2_util::declare_parameter_if_not_declared;

namespace nav2_dynamic_window_pure_pursuit_controller
{
namespace
{
std::string getDefaultCsvLogDirectory(const rclcpp::Logger & logger)
{
  try {
    const auto pkg_share =
      ament_index_cpp::get_package_share_directory("nav2_dynamic_window_pure_pursuit_controller");
    const auto log_dir = std::filesystem::path(pkg_share) / "data";
    std::error_code ec;
    std::filesystem::create_directories(log_dir, ec);
    if (ec) {
      RCLCPP_WARN(
        logger,
        "Failed to create default csv directory '%s' (%s). Falling back to /tmp.",
        log_dir.string().c_str(), ec.message().c_str());
      return "/tmp";
    }
    return log_dir.string();
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger,
      "Failed to resolve package share for default csv directory (%s). Falling back to /tmp.",
      e.what());
    return "/tmp";
  }
}
}  // namespace

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

  const std::string default_csv_log_directory = getDefaultCsvLogDirectory(logger_);

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
  declare_parameter_if_not_declared(
    node.get(), name + ".enable_csv_logging",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node.get(), name + ".csv_log_directory",
    rclcpp::ParameterValue(default_csv_log_directory));
  declare_parameter_if_not_declared(
    node.get(), name + ".csv_filename_prefix",
    rclcpp::ParameterValue(std::string("dwpp_nav2")));

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
  node->get_parameter(name + ".enable_csv_logging", enable_csv_logging_);
  node->get_parameter(name + ".csv_log_directory", csv_log_directory_);
  node->get_parameter(name + ".csv_filename_prefix", csv_filename_prefix_);

  if (enable_csv_logging_) {
    if (csv_log_directory_.empty()) {
      csv_log_directory_ = default_csv_log_directory;
    }
    if (csv_filename_prefix_.empty()) {
      csv_filename_prefix_ = "dwpp_nav2";
    }

    std::error_code ec;
    std::filesystem::create_directories(csv_log_directory_, ec);
    if (ec) {
      RCLCPP_WARN(
        logger_,
        "Failed to create csv_log_directory '%s' (%s). Falling back to /tmp.",
        csv_log_directory_.c_str(), ec.message().c_str());
      csv_log_directory_ = "/tmp";
      std::filesystem::create_directories(csv_log_directory_, ec);
    }
  }
}

void DynamicWindowPurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  rpp::RegulatedPurePursuitController::setPlan(path);
  if (!enable_csv_logging_) {
    return;
  }
  openNewCsvLogFile(rclcpp::Clock(RCL_SYSTEM_TIME).now());
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

  double linear_vel = 0.0;
  double angular_vel = 0.0;
  double regulated_linear_vel = 0.0;
  double curvature = 0.0;
  DynamicWindowBounds dynamic_window {0.0, 0.0, 0.0, 0.0};
  bool used_dwpp = false;

  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 =
    (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
    (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

  // Find curvature of circle (k = 1 / R)
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

    // Dynamic Window Pure Pursuit:
    // compute optimal path tracking velocity commands considering velocity and acceleration constraints
    regulated_linear_vel = linear_vel;
    const geometry_msgs::msg::Twist current_speed = last_command_velocity_;

    dynamic_window = computeDynamicWindow(
      current_speed,
      max_linear_vel_,
      min_linear_vel_,
      max_angular_vel_,
      min_angular_vel_,
      max_linear_accel_,
      max_linear_decel_,
      max_angular_accel_,
      max_angular_decel_,
      control_duration_);
    applyRegulationToDynamicWindow(regulated_linear_vel, dynamic_window);
    std::tie(linear_vel, angular_vel) =
      computeOptimalVelocityWithinDynamicWindow(dynamic_window, curvature, sign);
    used_dwpp = true;
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

  if (enable_csv_logging_ && used_dwpp) {
    const bool constraints_violation =
      evaluateVelocityConstraints(cmd_vel.twist, last_command_velocity_);
    writeCsvLogLine(
      pose.header.stamp,
      pose,
      speed,
      last_command_velocity_,
      cmd_vel.twist,
      dynamic_window,
      curvature,
      regulated_linear_vel,
      constraints_violation);
  }

  // For dynamic window scaling in open-loop speed control
  last_command_velocity_ = cmd_vel.twist;

  return cmd_vel;
}

void DynamicWindowPurePursuitController::openNewCsvLogFile(const rclcpp::Time & stamp)
{
  closeCsvLogFile();

  const auto sec = static_cast<std::time_t>(stamp.seconds());
  const auto nsec = static_cast<uint32_t>(stamp.nanoseconds() % 1000000000LL);
  std::tm tm_snapshot {};
  if (const std::tm * tm_ptr = std::localtime(&sec)) {
    tm_snapshot = *tm_ptr;
  }

  std::ostringstream filename;
  filename << csv_filename_prefix_ << "_"
           << std::put_time(&tm_snapshot, "%Y%m%d_%H%M%S")
           << "_" << std::setw(9) << std::setfill('0') << nsec
           << ".csv";
  const std::filesystem::path path = std::filesystem::path(csv_log_directory_) / filename.str();
  csv_stream_.open(path.string(), std::ios::out | std::ios::trunc);
  if (!csv_stream_.is_open()) {
    RCLCPP_WARN(logger_, "Failed to open DWPP csv log file: %s", path.string().c_str());
    return;
  }

  csv_header_written_ = false;
  RCLCPP_INFO(logger_, "DWPP csv logging started: %s", path.string().c_str());
}

void DynamicWindowPurePursuitController::closeCsvLogFile()
{
  if (!csv_stream_.is_open()) {
    csv_header_written_ = false;
    return;
  }

  csv_stream_.flush();
  csv_stream_.close();
  csv_header_written_ = false;
}

bool DynamicWindowPurePursuitController::evaluateVelocityConstraints(
  const geometry_msgs::msg::Twist & next_cmd_vel,
  const geometry_msgs::msg::Twist & current_cmd_vel) const
{
  constexpr double Eps = 1e-2;

  auto evaluate_velocity =
    [&](const double current_vel, const double last_vel, const double max_vel, const double min_vel,
      const double max_accel, const double max_decel) -> bool
    {
      double candidate_max_vel = 0.0;
      double candidate_min_vel = 0.0;

      if (last_vel > Eps) {
        candidate_max_vel = last_vel + max_accel * control_duration_;
        candidate_min_vel = last_vel + max_decel * control_duration_;
      } else if (last_vel < -Eps) {
        candidate_max_vel = last_vel - max_decel * control_duration_;
        candidate_min_vel = last_vel - max_accel * control_duration_;
      } else {
        candidate_max_vel = last_vel + max_accel * control_duration_;
        candidate_min_vel = last_vel - max_accel * control_duration_;
      }

      candidate_max_vel = std::min(candidate_max_vel, max_vel);
      candidate_min_vel = std::max(candidate_min_vel, min_vel);

      return current_vel > candidate_max_vel + Eps || current_vel < candidate_min_vel - Eps;
    };

  const bool linear_violation = evaluate_velocity(
    next_cmd_vel.linear.x,
    current_cmd_vel.linear.x,
    max_linear_vel_,
    min_linear_vel_,
    max_linear_accel_,
    max_linear_decel_);

  const bool angular_violation = evaluate_velocity(
    next_cmd_vel.angular.z,
    current_cmd_vel.angular.z,
    max_angular_vel_,
    min_angular_vel_,
    max_angular_accel_,
    max_angular_decel_);

  return linear_violation || angular_violation;
}

void DynamicWindowPurePursuitController::writeCsvLogLine(
  const rclcpp::Time & stamp,
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  const geometry_msgs::msg::Twist & current_cmd_vel,
  const geometry_msgs::msg::Twist & cmd_velocity,
  const DynamicWindowBounds & dynamic_window,
  double curvature,
  double regulated_linear_vel,
  bool constraints_violation)
{
  if (!enable_csv_logging_) {
    return;
  }
  if (!csv_stream_.is_open()) {
    openNewCsvLogFile(stamp);
  }
  if (!csv_stream_.is_open()) {
    return;
  }

  if (!csv_header_written_) {
    csv_stream_
      << "sec,nsec,x,y,yaw,"
      << "v_real,w_real,v_now,w_now,v_cmd,w_cmd,v_nav,w_nav,"
      << "velocity_violation,curvature,dw_v_max,dw_v_min,dw_w_max,dw_w_min,v_reg\n";
    csv_header_written_ = true;
  }

  constexpr double Eps = 1e-3;
  auto calc_actual_velocity =
    [&](const double current_vel, const double last_vel, const double max_vel, const double min_vel,
      const double max_accel, const double max_decel) -> double
    {
      double candidate_max_vel = 0.0;
      double candidate_min_vel = 0.0;

      if (last_vel > Eps) {
        candidate_max_vel = last_vel + max_accel * control_duration_;
        candidate_min_vel = last_vel + max_decel * control_duration_;
      } else if (last_vel < -Eps) {
        candidate_max_vel = last_vel - max_decel * control_duration_;
        candidate_min_vel = last_vel - max_accel * control_duration_;
      } else {
        candidate_max_vel = last_vel + max_accel * control_duration_;
        candidate_min_vel = last_vel - max_accel * control_duration_;
      }

      candidate_max_vel = std::min(candidate_max_vel, max_vel);
      candidate_min_vel = std::max(candidate_min_vel, min_vel);

      if (current_vel > candidate_max_vel + Eps) {
        return candidate_max_vel;
      } else if (current_vel < candidate_min_vel - Eps) {
        return candidate_min_vel;
      } else {
        return current_vel;
      }
    };

  const double actual_linear_vel = calc_actual_velocity(
    cmd_velocity.linear.x,
    current_cmd_vel.linear.x,
    max_linear_vel_,
    min_linear_vel_,
    max_linear_accel_,
    max_linear_decel_);
  const double actual_angular_vel = calc_actual_velocity(
    cmd_velocity.angular.z,
    current_cmd_vel.angular.z,
    max_angular_vel_,
    min_angular_vel_,
    max_angular_accel_,
    max_angular_decel_);

  const auto sec = static_cast<int32_t>(stamp.seconds());
  const auto nsec = static_cast<uint32_t>(stamp.nanoseconds() % 1000000000LL);

  csv_stream_ << sec << ","
              << nsec << ","
              << std::fixed << std::setprecision(6)
              << pose.pose.position.x << ","
              << pose.pose.position.y << ","
              << tf2::getYaw(pose.pose.orientation) << ","
              << speed.linear.x << ","
              << speed.angular.z << ","
              << current_cmd_vel.linear.x << ","
              << current_cmd_vel.angular.z << ","
              << cmd_velocity.linear.x << ","
              << cmd_velocity.angular.z << ","
              << actual_linear_vel << ","
              << actual_angular_vel << ","
              << (constraints_violation ? 1 : 0) << ","
              << curvature << ","
              << dynamic_window.max_linear_vel << ","
              << dynamic_window.min_linear_vel << ","
              << dynamic_window.max_angular_vel << ","
              << dynamic_window.min_angular_vel << ","
              << regulated_linear_vel
              << "\n";
  csv_stream_.flush();
}

void DynamicWindowPurePursuitController::deactivate()
{
  rpp::RegulatedPurePursuitController::deactivate();
  closeCsvLogFile();
  last_command_velocity_ = geometry_msgs::msg::Twist();
}

void DynamicWindowPurePursuitController::cleanup()
{
  rpp::RegulatedPurePursuitController::cleanup();
  closeCsvLogFile();
}

}  // namespace nav2_dynamic_window_pure_pursuit_controller

// pluginlib registration
PLUGINLIB_EXPORT_CLASS(
  nav2_dynamic_window_pure_pursuit_controller::DynamicWindowPurePursuitController,
  nav2_core::Controller)
