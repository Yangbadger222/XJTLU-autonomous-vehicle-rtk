cmake_minimum_required(VERSION 3.8)
project(fgo_gil_localizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gnss_raw_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(fgo_gil_core
  src/ecef_imu_preintegrator.cpp
  src/gnss_time.cpp
  src/imu_buffer.cpp
  src/keyframe_map.cpp
  src/lidar_deskewer.cpp
  src/lidar_factors.cpp
  src/lidar_feature_extractor.cpp
  src/lidar_matcher.cpp
  src/lidar_types.cpp
  src/livox_preprocessor.cpp
  src/math_types.cpp
  src/time_sync.cpp
)
target_include_directories(fgo_gil_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fgo_gil_core PUBLIC Eigen3::Eigen)

add_executable(fgo_gil_time_sync_node src/time_sync_node.cpp)
target_include_directories(fgo_gil_time_sync_node PRIVATE include)
target_link_libraries(fgo_gil_time_sync_node fgo_gil_core)
ament_target_dependencies(fgo_gil_time_sync_node
  diagnostic_msgs
  gnss_raw_msgs
  livox_ros_driver2
  rclcpp
  sensor_msgs
)

add_executable(fgo_gil_lidar_frontend_node src/lidar_frontend_node.cpp)
target_include_directories(fgo_gil_lidar_frontend_node PRIVATE include)
target_link_libraries(fgo_gil_lidar_frontend_node fgo_gil_core)
ament_target_dependencies(fgo_gil_lidar_frontend_node
  diagnostic_msgs
  gnss_raw_msgs
  livox_ros_driver2
  nav_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS fgo_gil_core
  EXPORT export_fgo_gil_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS fgo_gil_time_sync_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS fgo_gil_lidar_frontend_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_time_sync test/test_time_sync.cpp)
  target_include_directories(test_time_sync PRIVATE include)
  target_link_libraries(test_time_sync fgo_gil_core)

  ament_add_gtest(test_imu_buffer test/test_imu_buffer.cpp)
  target_include_directories(test_imu_buffer PRIVATE include)
  target_link_libraries(test_imu_buffer fgo_gil_core)

  ament_add_gtest(test_ecef_imu_preintegrator test/test_ecef_imu_preintegrator.cpp)
  target_include_directories(test_ecef_imu_preintegrator PRIVATE include)
  target_link_libraries(test_ecef_imu_preintegrator fgo_gil_core)

  ament_add_gtest(test_lidar_deskewer test/test_lidar_deskewer.cpp)
  target_include_directories(test_lidar_deskewer PRIVATE include)
  target_link_libraries(test_lidar_deskewer fgo_gil_core)

  ament_add_gtest(test_lidar_features test/test_lidar_features.cpp)
  target_include_directories(test_lidar_features PRIVATE include)
  target_link_libraries(test_lidar_features fgo_gil_core)

  ament_add_gtest(test_lidar_factors test/test_lidar_factors.cpp)
  target_include_directories(test_lidar_factors PRIVATE include)
  target_link_libraries(test_lidar_factors fgo_gil_core)

  ament_add_gtest(test_keyframe_map test/test_keyframe_map.cpp)
  target_include_directories(test_keyframe_map PRIVATE include)
  target_link_libraries(test_keyframe_map fgo_gil_core)

  ament_add_gtest(test_livox_preprocessor test/test_livox_preprocessor.cpp)
  target_include_directories(test_livox_preprocessor PRIVATE include)
  target_link_libraries(test_livox_preprocessor fgo_gil_core)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(fgo_gil_core)
ament_export_targets(export_fgo_gil_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
