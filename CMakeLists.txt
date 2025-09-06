cmake_minimum_required(VERSION 3.8)
project(nav2_dynamic_window_pure_pursuit_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  nav2_regulated_pure_pursuit_controller
  tf2
  tf2_geometry_msgs
)

set(library_name nav2_dynamic_window_pure_pursuit_controller)

add_library(${library_name} SHARED
  src/dynamic_window_pure_pursuit_controller.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

# ライブラリ・ヘッダ
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
)

# launch/, config/, world/, model/ を share に入れる（ros2 launch / パラメータで必須）
install(DIRECTORY launch config world model
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/follow_path_test_gui.py
  scripts/ground_truth_odom_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

# pluginlib の XML（既にエクスポートされているが install 先を確実に）
pluginlib_export_plugin_description_file(nav2_core nav2_dynamic_window_pure_pursuit_controller.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
