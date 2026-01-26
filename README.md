> [!IMPORTANT]
> # DWPP is integrated into the official Nav2 repository!
> 
>👉 **Official Repository:** [navigation2/nav2_regulated_pure_pursuit_controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller)
> 
> **Official Integration:** As of January 21, 2026, the DWPP algorithm has been merged into the Nav2 main branch as an optional feature of the `RegulatedPurePursuitController` plugin.
>
> **Binary Distribution:** Starting with ROS 2 Lyrical (scheduled for release around May 2026), Nav2 will include DWPP in its binary distribution. You will be able to use DWPP simply by running `sudo apt install ros-<distro>-navigation2`.
>
> **Support for Older Versions (Humble, Jazzy)**: Since DWPP is not backported to versions prior to Lyrical, please continue to use the plugin provided in **this repository** for those distributions.
>
> **Future Maintenance:** Following the release of Lyrical, this repository will be transitioned to a **public archive**. All future bug fixes and feature enhancements will be directed to the official Nav2 repository. I strongly recommend migrating to the official Nav2 package from ROS 2 Lyrical onwards.
>

---
# nav2_dynamic_window_pure_pursuit_controller
**Nav2 plugin for Dynamic Window Pure Pursuit (DWPP) controller**

<div align="center">

| ROS 2 Distro | Branch |
| :--- | :--- |
| **Humble** | [`humble`](https://github.com/decwest/nav2_dynamic_window_pure_pursuit_controller/tree/humble) |
| **Jazzy** | [`jazzy`](https://github.com/decwest/nav2_dynamic_window_pure_pursuit_controller/tree/jazzy) |

[![License: Apache-2.0](https://img.shields.io/badge/License-Apache2.0-red.svg)](https://www.apache.org/licenses/LICENSE-2.0) [![ROS2 Distro: Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/index.html)

![demo](comparison.gif)
<sub>Note: Except for $v_{\max}=0.5$, $\omega_{\max}=1.0$, $a_{\max}=0.5$, $\alpha_{\max}=1.0$, and `use_rotate_to_heading=False`, all parameters are set to the default values of Nav2’s RPP.</sub>

</div>

Dynamic Window Pure Pursuit (DWPP) is a novel extension of the pure pursuit method that computes command velocities to track a path as accurately as possible while explicitly respecting velocity and acceleration constraints. For example, it automatically slows down in sharp turns without manual tuning, thereby reducing path tracking errors.

DWPP is designed as an incremental extension of Regulated Pure Pursuit (RPP)—the default pure pursuit controller in Nav2—so all RPP parameters can be used in the same way with DWPP.

The paper is available here:
- Fumiya Ohnishi, Masaki Takahashi, **"DWPP: Dynamic Window Pure Pursuit Considering Velocity and Acceleration Constraints,"** arXiv:2601.15006, 2026.
  https://arxiv.org/abs/2601.15006



## Installation
1. Clone this repository into your `src/` directory:
```shell
git clone -b jazzy https://github.com/decwest/nav2_dynamic_window_pure_pursuit_controller.git
```

2. Build the package:
```shell
colcon build --symlink-install
```

> **Note**: The DWPP Controller inherits the RPP controller class from the latest Jazzy version of Nav2.
If your installed Nav2 is not updated to the latest release, build errors may occur.

1. Configure Nav2 to use DWPP:
Set controller plugin name as `"nav2_dynamic_window_pure_pursuit_controller::DynamicWindowPurePursuitController"`

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_dynamic_window_pure_pursuit_controller::DynamicWindowPurePursuitController" # change here to use DWPP plugin
```

## Parameter Settings

### Example configuration

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugins: ["progress_checker"]
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
      plugin: "nav2_dynamic_window_pure_pursuit_controller::DynamicWindowPurePursuitController" # change here to use DWPP plugin
      max_linear_vel: 0.5 # DWPP parameter
      min_linear_vel: 0.0 # DWPP parameter
      max_angular_vel: 1.0 # DWPP parameter
      min_angular_vel: -1.0 # DWPP parameter
      max_linear_accel: 0.5 # DWPP parameter
      max_linear_decel: -0.5 # DWPP parameter
      max_angular_accel: 1.0 # DWPP parameter
      max_angular_decel: -1.0 # DWPP parameter
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: true # use adaptive pure pursuit
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_collision_detection: true
      use_regulated_linear_velocity_scaling: true # use regulated pure pursuit
      use_cost_regulated_linear_velocity_scaling: true # use regulated pure pursuit
      cost_scaling_dist: 0.6
      cost_scaling_gain: 1.0
      inflation_cost_scaling_factor: 3.0
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 3.14 # OFF since DWPP can handle steep curvature
      allow_reversing: false
      use_interpolation: true

velocity_smoother:
  ros__parameters:
    use_sim_time: True
    smoothing_frequency: 20.0
    scale_velocities: False
    feedback: "OPEN_LOOP"
    max_velocity: [0.5, 0.0, 1.0] # set same as dwpp setting
    min_velocity: [0.0, 0.0, -1.0] # set same as dwpp setting
    max_accel: [0.5, 0.0, 1.0] # set same as dwpp setting
    max_decel: [-0.5, 0.0, -1.0] # set same as dwpp setting
    odom_topic: "odom"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0
```

### Parameter Settings

- **max_linear_vel** (double): The maximum linear velocity (m/s) to use.
- **min_linear_vel** (double): The minimum linear velocity (m/s) to use.
- **max_angular_vel** (double): The maximum angular velocity (rad/s) to use.
- **min_angular_vel** (double): The minimum angular velocity (rad/s) to use.
- **max_linear_accel** (double): The maximum linear acceleration (m/s^2) to use.
- **max_linear_decel** (double): The maximum linear deceleration (m/s^2) to use.
- **max_angular_accel** (double): The maximum angular acceleration (rad/s^2) to use.
- **max_angular_decel** (double): The maximum angular deceleration (rad/s^2) to use.

Other parameter settings are same as RPP:
https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html

### Velocity Smoother Parameters
The velocity smoother clips velocity values from `cmd_vel_nav` (published by the controller) according to velocity and acceleration constraints, and publishes `cmd_vel`.
Therefore, `max_velocity`, `min_velocity`, `max_accel`, and `max_decel` must match the DWPP controller’s velocity and acceleration settings.

### Note: For users who want the robot to move backward

You can allow the robot to move in reverse by setting `allow_reversing` to `True`.
However, in the Humble version, enabling `allow_reversing` automatically disables `use_rotate_to_heading`. As a result, the robot will no longer adjust its orientation to match the goal heading upon arrival. This can cause the system to fail to recognize that the goal has been reached, which may lead to small forward and backward oscillations near the goal.

This issue is not specific to DWPP, but occurs with any Pure Pursuit controller, and it has been resolved in Jazzy and later releases. Therefore, if you want the robot to move backward, it is recommended to upgrade to Jazzy.

## Trying DWPP
The following repository provides simulations for comparing DWPP with conventional methods, and also includes Nav2 tutorials that run with DWPP.

https://github.com/decwest/dwpp_test_environment

## Citation

If you use DWPP in your research or project, please cite the latest preprint (Journal version) as the primary reference, as it covers the algorithm currently implemented in Nav2.

**Primary Reference (Preprint)**

```txt
Fumiya Ohnishi and Masaki Takahashi, "DWPP: Dynamic Window Pure Pursuit Considering Velocity and Acceleration Constraints", arXiv preprint arXiv:2601.15006, 2026.
```

```bibtex
@misc{ohnishi2026dwpp,
      title={DWPP: Dynamic Window Pure Pursuit Considering Velocity and Acceleration Constraints}, 
      author={Fumiya Ohnishi and Masaki Takahashi},
      year={2026},
      eprint={2601.15006},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2601.15006}, 
}
```

**Conference Paper (Original Concept)**

This work was first presented at IAS-19 and recommended for a special issue.

```txt
Fumiya Ohnishi and Masaki Takahashi, “Dynamic Window Pure Pursuit for Robot Path Tracking Considering Velocity and Acceleration Constraints”, Proceedings of the 19th International Conference on Intelligent Autonomous Systems, Genoa, Italy, 2025.
```

```bibtex
@inproceedings{ohnishi2025dwpp_ias,
  title     = {Dynamic Window Pure Pursuit for Robot Path Tracking Considering Velocity and Acceleration Constraints},
  author    = {Fumiya Ohnishi and Masaki Takahashi},
  booktitle = {Proceedings of the 19th International Conference on Intelligent Autonomous Systems (IAS-19)},
  year      = {2025},
  address   = {Genoa, Italy}
}
```

