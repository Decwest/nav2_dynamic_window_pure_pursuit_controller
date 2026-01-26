#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # ====== パス解決 ======
    pkg_this = get_package_share_directory('nav2_dynamic_window_pure_pursuit_controller')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_tb3_sim = get_package_share_directory('nav2_minimal_tb3_sim')

    # 既定ファイル
    default_params_file = os.path.join(pkg_this, 'config', 'test_params.yaml')
    default_world_file = os.path.join(pkg_this, 'world', 'test.sdf')
    default_robot_sdf = os.path.join(pkg_this, 'urdf', 'gz_waffle.sdf.xacro')  # 真値オドメトリ付きカスタムモデル
    default_rviz_config = os.path.join(pkg_this, 'rviz', 'dwpp_test.rviz')
    default_urdf_xacro = os.path.join(pkg_tb3_sim, 'urdf', 'turtlebot3_waffle.urdf')
    default_bridge_yaml = os.path.join(pkg_this, 'config', 'ros_gz_bridge.yaml')

    # ====== Launch 引数 ======
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    world_file = LaunchConfiguration('world_file')
    robot_sdf = LaunchConfiguration('robot_sdf')
    urdf_xacro = LaunchConfiguration('urdf_xacro')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    robot_name = LaunchConfiguration('robot_name')
    bridge_yaml = LaunchConfiguration('bridge_yaml')

    declare_namespace = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
    declare_params = DeclareLaunchArgument('params_file', default_value=default_params_file, description='Nav2 parameters YAML')
    declare_world = DeclareLaunchArgument('world_file', default_value=default_world_file, description='Gazebo world (SDF)')
    declare_robot = DeclareLaunchArgument('robot_sdf', default_value=default_robot_sdf, description='Robot SDF(Xacro) file to spawn')
    declare_urdfx = DeclareLaunchArgument('urdf_xacro', default_value=default_urdf_xacro, description='TB3 xacro for RSP')
    declare_use_sim = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use /clock from Gazebo')
    declare_rvizcf = DeclareLaunchArgument('rviz_config', default_value=default_rviz_config, description='RViz2 config file')
    declare_use_rv = DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2')
    declare_robot_name = DeclareLaunchArgument('robot_name', default_value='turtlebot3_waffle', description='Gazebo model/entity name')
    declare_bridge_yaml = DeclareLaunchArgument('bridge_yaml', default_value=default_bridge_yaml, description='ros_gz_bridge YAML config')

    # ====== Gazebo (Ignition/Gz) 起動 ======
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': PythonExpression(["'-r -v 4 ' + '", world_file, "'"]),
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ====== TurtleBot3 スポーン ======
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_tb3_sim, 'launch', 'spawn_tb3.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_name': robot_name,
            'robot_sdf': robot_sdf,
            'x_pose': LaunchConfiguration('x_pose', default='0.0'),
            'y_pose': LaunchConfiguration('y_pose', default='0.0'),
            'z_pose': LaunchConfiguration('z_pose', default='0.1'),
            'roll': LaunchConfiguration('roll', default='0.0'),
            'pitch': LaunchConfiguration('pitch', default='0.0'),
            'yaw': LaunchConfiguration('yaw', default='0.0'),
        }.items(),
    )

    # ====== ros_gz_bridge 起動 ======
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            "/world/test/set_pose@ros_gz_interfaces/srv/SetEntityPose"
        ],
        parameters=[{'config_file': bridge_yaml}],
    )

    # ====== map -> odom を恒等に固定 ======
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom'
        ],
    )

    gui_node = Node(
        package='nav2_dynamic_window_pure_pursuit_controller',
        executable='follow_path_test_gui.py',
        name='follow_path_gui',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ====== robot_state_publisher ======
    robot_description = Command(['xacro ', urdf_xacro, ' namespace:=', '', ' use_sim_time:=', use_sim_time])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # ====== Nav2（AMCL等なし） ======
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    # ====== RViz2 ======
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        declare_namespace,
        declare_params,
        declare_world,
        declare_robot,
        declare_urdfx,
        declare_use_sim,
        declare_rvizcf,
        declare_use_rv,
        declare_robot_name,
        declare_bridge_yaml,
        gz_sim,
        spawn_tb3,
        bridge,
        static_map_to_odom,
        robot_state_publisher,
        nav2_navigation,
        rviz,
        gui_node,
    ])
