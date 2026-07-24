cmake_minimum_required(VERSION 3.18)
project(mppi_cuda_backend LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
include(CheckLanguage)
check_language(CUDA)

# Keep ordinary development machines and non-NVIDIA ROS targets buildable. The
# package becomes active only on a CUDA-enabled target such as the Orin NX.
if(NOT CMAKE_CUDA_COMPILER)
  message(WARNING "CUDA compiler not found; mppi_cuda_backend is disabled")
  ament_package()
  return()
endif()

enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/mppi_cuda_backend.cu
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME} PROPERTIES
  CUDA_STANDARD 17
  CUDA_STANDARD_REQUIRED ON
  CUDA_ARCHITECTURES 87
  POSITION_INDEPENDENT_CODE ON
)
target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math>
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC CUDA::cudart)

add_executable(mppi_cuda_benchmark src/mppi_cuda_benchmark.cpp)
target_compile_features(mppi_cuda_benchmark PRIVATE cxx_std_17)
target_link_libraries(mppi_cuda_benchmark PRIVATE ${PROJECT_NAME})

add_executable(mppi_cuda_bag_replay src/mppi_cuda_bag_replay.cpp)
target_compile_features(mppi_cuda_bag_replay PRIVATE cxx_std_17)
target_link_libraries(mppi_cuda_bag_replay PRIVATE ${PROJECT_NAME})
ament_target_dependencies(mppi_cuda_bag_replay
  geometry_msgs
  nav_msgs
  rclcpp
  rosbag2_cpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
)

install(TARGETS ${PROJECT_NAME} mppi_cuda_benchmark mppi_cuda_bag_replay
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mppi_cuda_backend test/test_mppi_cuda_backend.cpp)
  if(TARGET test_mppi_cuda_backend)
    target_link_libraries(test_mppi_cuda_backend ${PROJECT_NAME})
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(CUDAToolkit)
ament_package()
