cmake_minimum_required(VERSION 3.8)
project(frc_nodes_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(frc_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(frc_health_aggregator src/health_aggregator_node.cpp)
target_include_directories(frc_health_aggregator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(frc_health_aggregator
  frc_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(frc_event_marker src/event_marker_node.cpp)
target_include_directories(frc_event_marker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(frc_event_marker
  frc_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
)

add_executable(frc_risk_pipeline src/risk_pipeline_node.cpp)
target_include_directories(frc_risk_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(frc_risk_pipeline
  frc_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

install(TARGETS frc_health_aggregator frc_event_marker frc_risk_pipeline
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_health_aggregator_core
    test/test_health_aggregator_core.py
  )
  ament_add_pytest_test(test_event_marker_core
    test/test_event_marker_core.py
  )
  ament_add_pytest_test(test_risk_pipeline_core
    test/test_risk_pipeline_core.py
  )
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  frc_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
ament_package()
