cmake_minimum_required(VERSION 3.8)
project(rtk_fgo_localizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(rtk_fgo_core
  src/correction_smoother.cpp
  src/rtk_quality.cpp
  src/state_machine.cpp
)
target_include_directories(rtk_fgo_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS rtk_fgo_core
  EXPORT export_rtk_fgo_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_rtk_quality test/test_rtk_quality.cpp)
  target_include_directories(test_rtk_quality PRIVATE include)
  target_link_libraries(test_rtk_quality rtk_fgo_core)

  ament_add_gtest(test_state_machine test/test_state_machine.cpp)
  target_include_directories(test_state_machine PRIVATE include)
  target_link_libraries(test_state_machine rtk_fgo_core)

  ament_add_gtest(test_correction_smoother test/test_correction_smoother.cpp)
  target_include_directories(test_correction_smoother PRIVATE include)
  target_link_libraries(test_correction_smoother rtk_fgo_core)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(rtk_fgo_core)
ament_export_targets(export_rtk_fgo_core HAS_LIBRARY_TARGET)
ament_package()
