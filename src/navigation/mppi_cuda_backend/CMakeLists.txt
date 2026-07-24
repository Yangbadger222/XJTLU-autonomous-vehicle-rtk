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

install(TARGETS ${PROJECT_NAME} mppi_cuda_benchmark
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
