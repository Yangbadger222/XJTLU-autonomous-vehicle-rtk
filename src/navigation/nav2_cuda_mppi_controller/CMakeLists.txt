cmake_minimum_required(VERSION 3.18)
project(nav2_cuda_mppi_controller LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
include(CheckLanguage)
check_language(CUDA)

if(NOT CMAKE_CUDA_COMPILER)
  message(WARNING "CUDA compiler not found; nav2_cuda_mppi_controller is disabled")
  ament_package()
  return()
endif()

find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mppi_cuda_backend REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)

add_library(${PROJECT_NAME} SHARED src/cuda_mppi_shadow_controller.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  diagnostic_msgs
  geometry_msgs
  mppi_cuda_backend
  nav2_core
  nav2_costmap_2d
  nav2_mppi_controller
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
)

pluginlib_export_plugin_description_file(nav2_core cuda_mppic.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES cuda_mppic.xml DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(
  diagnostic_msgs
  geometry_msgs
  mppi_cuda_backend
  nav2_core
  nav2_costmap_2d
  nav2_mppi_controller
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
)
ament_package()
