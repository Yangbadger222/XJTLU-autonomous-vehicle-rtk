cmake_minimum_required(VERSION 3.8)
project(rtk_fgo_localizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

add_library(rtk_fgo_core
  src/correction_smoother.cpp
  src/diagnostic_snapshot.cpp
  src/frame_anchor.cpp
  src/fgo_graph.cpp
  src/heading_conventions.cpp
  src/heading_stability.cpp
  src/rtk_quality.cpp
  src/state_machine.cpp
  src/topic_buffers.cpp
  src/yaw_factor.cpp
)
target_include_directories(rtk_fgo_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${diagnostic_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(rtk_fgo_core PUBLIC
  gtsam
  ${GeographicLib_LIBRARIES}
)

add_executable(rtk_fgo_node
  src/rtk_fgo_node.cpp
)
target_include_directories(rtk_fgo_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(rtk_fgo_node
  rtk_fgo_core
  ${GeographicLib_LIBRARIES}
)
ament_target_dependencies(rtk_fgo_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  nmea_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

install(TARGETS rtk_fgo_core
  EXPORT export_rtk_fgo_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS rtk_fgo_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_rtk_quality test/test_rtk_quality.cpp)
  target_include_directories(test_rtk_quality PRIVATE include)
  target_link_libraries(test_rtk_quality rtk_fgo_core)

  ament_add_gtest(test_gate_params test/test_gate_params.cpp)
  target_include_directories(test_gate_params PRIVATE include)
  target_link_libraries(test_gate_params rtk_fgo_core)

  ament_add_gtest(test_state_machine test/test_state_machine.cpp)
  target_include_directories(test_state_machine PRIVATE include)
  target_link_libraries(test_state_machine rtk_fgo_core)

  ament_add_gtest(test_correction_smoother test/test_correction_smoother.cpp)
  target_include_directories(test_correction_smoother PRIVATE include)
  target_link_libraries(test_correction_smoother rtk_fgo_core)

  ament_add_gtest(test_heading_conventions test/test_heading_conventions.cpp)
  target_include_directories(test_heading_conventions PRIVATE include)
  target_link_libraries(test_heading_conventions rtk_fgo_core)

  ament_add_gtest(test_heading_stability test/test_heading_stability.cpp)
  target_include_directories(test_heading_stability PRIVATE include)
  target_link_libraries(test_heading_stability rtk_fgo_core)

  ament_add_gtest(test_frame_anchor test/test_frame_anchor.cpp)
  target_include_directories(test_frame_anchor PRIVATE include)
  target_link_libraries(test_frame_anchor rtk_fgo_core)

  ament_add_gtest(test_fgo_graph test/test_fgo_graph.cpp)
  target_include_directories(test_fgo_graph PRIVATE include)
  target_link_libraries(test_fgo_graph rtk_fgo_core)

  ament_add_gtest(test_imu_preintegration test/test_imu_preintegration.cpp)
  target_include_directories(test_imu_preintegration PRIVATE include)
  target_link_libraries(test_imu_preintegration rtk_fgo_core)

  ament_add_gtest(test_topic_buffers test/test_topic_buffers.cpp)
  target_include_directories(test_topic_buffers PRIVATE include)
  target_link_libraries(test_topic_buffers rtk_fgo_core)

  ament_add_gtest(test_diagnostic_snapshot test/test_diagnostic_snapshot.cpp)
  target_include_directories(test_diagnostic_snapshot PRIVATE include)
  target_link_libraries(test_diagnostic_snapshot rtk_fgo_core)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(rtk_fgo_core)
ament_export_targets(export_rtk_fgo_core HAS_LIBRARY_TARGET)
ament_package()
