#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # ====== パス解決 ======
    pkg_this   = get_package_share_directory('nav2_dynamic_window_pure_pursuit_controller')
    pkg_nav2   = get_package_share_directory('nav2_bringup')
    pkg_gazebo = get_package_share_directory('gazebo_ros')
    pkg_tb3_desc = get_package_share_directory('turtlebot3_description')

    # 既定ファイル
    default_params_file = os.path.join(pkg_this,  'config', 'test_params.yaml')
    default_world_file  = os.path.join(pkg_this,  'world',  'test.world')
    default_robot_sdf   = os.path.join(pkg_this,  'model',  'tb3_wo_odom.sdf')
    default_rviz_config = os.path.join(pkg_nav2,  'rviz',   'nav2_default_view.rviz')
    default_urdf_xacro  = os.path.join(pkg_tb3_desc, 'urdf', 'turtlebot3_waffle.urdf')

    # ====== Launch 引数 ======
    params_file   = LaunchConfiguration('params_file')
    world_file    = LaunchConfiguration('world_file')
    robot_sdf     = LaunchConfiguration('robot_sdf')
    urdf_xacro    = LaunchConfiguration('urdf_xacro')
    tf_namespace  = LaunchConfiguration('tf_namespace')   # ← xacroのnamespaceに渡す。既定は空。
    use_sim_time  = LaunchConfiguration('use_sim_time')
    rviz_config   = LaunchConfiguration('rviz_config')
    use_rviz      = LaunchConfiguration('use_rviz')
    robot_entity  = LaunchConfiguration('robot_entity')

    declare_params = DeclareLaunchArgument('params_file', default_value=default_params_file, description='Nav2 parameters YAML')
    declare_world  = DeclareLaunchArgument('world_file',  default_value=default_world_file,  description='Gazebo world (Classic)')
    declare_robot  = DeclareLaunchArgument('robot_sdf',   default_value=default_robot_sdf,   description='Robot SDF file to spawn')
    declare_urdfx  = DeclareLaunchArgument('urdf_xacro',  default_value=default_urdf_xacro,  description='TB3 xacro for RSP')
    declare_ns     = DeclareLaunchArgument('tf_namespace', default_value=TextSubstitution(text=''), description='Prefix for TF (empty to remove)')
    declare_use_sim= DeclareLaunchArgument('use_sim_time', default_value='True', description='Use /clock from Gazebo')
    declare_rvizcf = DeclareLaunchArgument('rviz_config', default_value=default_rviz_config, description='RViz2 config file')
    declare_use_rv = DeclareLaunchArgument('use_rviz',    default_value='True', description='Launch RViz2')
    declare_entity = DeclareLaunchArgument('robot_entity', default_value='tb3', description='Gazebo model/entity name')

    # TB3 環境変数
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    # ====== Gazebo (Classic) 起動 ======
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file}.items()
    )

    # ====== Gazebo に TB3（SDF）をスポーン ======
    spawn_tb3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', robot_sdf,
                   '-entity', robot_entity],
        output='screen'
    )

    # ====== map -> odom を恒等に固定 ======
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        name='map_to_odom_static_tf'
    )

    # ====== Gazebo 真値 → /odom ＆ TF(odom->base_link) ======
    ground_truth_odom = Node(
        package='nav2_dynamic_window_pure_pursuit_controller',
        executable='ground_truth_odom_tf.py',
        name='ground_truth_odom_tf',
        output='screen',
        parameters=[{
            'model_name': robot_entity,  # spawnする -entity と一致させる
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'use_sim_time': use_sim_time
        }]
    )

    gui_node = Node(
            package='nav2_dynamic_window_pure_pursuit_controller',
            executable='follow_path_test_gui.py',
            name='follow_path_gui',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}]
        )

    # ====== robot_state_publisher（xacro展開して渡す ← ここが肝） ======
    # namespace:=<空文字> なら ${namespace} は消えた状態でフレームが出る
    robot_description = Command(['xacro ', urdf_xacro, ' namespace:=', tf_namespace, ' use_sim_time:=', use_sim_time])

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
        launch_arguments={'use_sim_time': use_sim_time,
                          'params_file': params_file}.items()
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
        declare_params, declare_world, declare_robot, declare_urdfx, declare_ns,
        declare_use_sim, declare_rvizcf, declare_use_rv, declare_entity,
        set_tb3_model,
        gazebo,
        spawn_tb3,
        static_map_to_odom,
        ground_truth_odom,
        robot_state_publisher,
        nav2_navigation,
        rviz,
        gui_node,
    ])
