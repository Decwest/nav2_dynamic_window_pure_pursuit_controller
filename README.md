# nav2_dynamic_window_pure_pursuit_controller
Nav2 plugin for Dynamic Window Pure Pursuit

Example

```txt

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugins: ["progress_checker"] # progress_checker_plugin: "progress_checker" For Humble and older
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
    FollowPath:
      plugin: "nav2_dynamic_window_pure_pursuit_controller::DynamicWindowPurePursuitController"
      desired_linear_vel: 0.5
      desired_angular_vel: 1.0
      max_linear_accel: 0.5
      max_angular_accel: 1.0
      velocity_feedback: "OPEN_LOOP"
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.8
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_regulated_linear_velocity_scaling: true
      use_fixed_curvature_lookahead: false
      curvature_lookahead_dist: 0.25
      use_cost_regulated_linear_velocity_scaling: false
      cost_scaling_dist: 0.3
      cost_scaling_gain: 1.0
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25
      use_rotate_to_heading: true
      allow_reversing: true
      rotate_to_heading_min_angle: 0.785
      max_robot_pose_search_dist: 10.0
      min_distance_to_obstacle: 0.0
      stateful: true
```
