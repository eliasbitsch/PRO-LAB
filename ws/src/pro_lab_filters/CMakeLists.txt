cmake_minimum_required(VERSION 3.14)
project(pro_lab_filters)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
set(CMAKE_AUTOMOC ON)

set(COMMON_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

include_directories(include ${EIGEN3_INCLUDE_DIR})

# ── Filter nodes ─────────────────────────────────────────────────────────
add_executable(kf_node src/kf_node.cpp)
ament_target_dependencies(kf_node ${COMMON_DEPS})

add_executable(ekf_node src/ekf_node.cpp)
ament_target_dependencies(ekf_node ${COMMON_DEPS})

add_executable(pf_node src/pf_node.cpp)
ament_target_dependencies(pf_node ${COMMON_DEPS})

# ── Experiment infrastructure (wrong-init study) ────────────────────────
add_executable(truth_relay_node src/truth_relay_node.cpp)
ament_target_dependencies(truth_relay_node ${COMMON_DEPS})

add_executable(metrics_node src/metrics_node.cpp)
ament_target_dependencies(metrics_node ${COMMON_DEPS})

add_executable(cmd_vel_watchdog src/cmd_vel_watchdog.cpp)
ament_target_dependencies(cmd_vel_watchdog rclcpp geometry_msgs)

add_executable(map_republisher src/map_republisher.cpp)
ament_target_dependencies(map_republisher rclcpp nav_msgs)

add_executable(robot_teleporter src/robot_teleporter.cpp)
ament_target_dependencies(robot_teleporter rclcpp geometry_msgs)

add_executable(map_odom_tf_publisher src/map_odom_tf_publisher.cpp)
ament_target_dependencies(map_odom_tf_publisher
    rclcpp geometry_msgs tf2 tf2_ros)

add_executable(teleop_marker_node src/teleop_marker_node.cpp)
ament_target_dependencies(teleop_marker_node
    rclcpp geometry_msgs visualization_msgs interactive_markers)

add_executable(landmark_detector_node src/landmark_detector_node.cpp)
ament_target_dependencies(landmark_detector_node
    rclcpp std_msgs geometry_msgs visualization_msgs tf2 tf2_ros)

install(TARGETS
    kf_node ekf_node pf_node
    truth_relay_node metrics_node
    cmd_vel_watchdog map_republisher robot_teleporter map_odom_tf_publisher
    teleop_marker_node landmark_detector_node
  DESTINATION lib/${PROJECT_NAME})

# ── RViz2 panel plugin (Foxglove-style teleop dpad) ─────────────────────
add_library(pro_lab_rviz_plugins SHARED
    src/teleop_panel.cpp
    src/kidnap_tool.cpp
    include/pro_lab_filters/teleop_panel.hpp
    include/pro_lab_filters/kidnap_tool.hpp)
target_include_directories(pro_lab_rviz_plugins PUBLIC include)
ament_target_dependencies(pro_lab_rviz_plugins
    rclcpp geometry_msgs rviz_common rviz_default_plugins rviz_rendering pluginlib tf2)
target_link_libraries(pro_lab_rviz_plugins Qt5::Core Qt5::Widgets)

install(TARGETS pro_lab_rviz_plugins
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)
install(FILES rviz_plugins.xml DESTINATION share/${PROJECT_NAME})
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

# Python scripts (CSV logger)
install(PROGRAMS
    scripts/csv_logger.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
