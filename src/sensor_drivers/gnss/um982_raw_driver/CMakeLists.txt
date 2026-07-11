cmake_minimum_required(VERSION 3.8)
project(um982_raw_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(gnss_raw_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)

add_library(um982_binary_framer
  src/binary_framer.cpp
)
target_include_directories(um982_binary_framer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(um982_raw_node
  src/um982_raw_node.cpp
)
target_include_directories(um982_raw_node PRIVATE include)
target_link_libraries(um982_raw_node um982_binary_framer)
ament_target_dependencies(um982_raw_node
  diagnostic_msgs
  gnss_raw_msgs
  rclcpp
  serial
)

install(TARGETS um982_binary_framer
  EXPORT export_um982_binary_framer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS um982_raw_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_binary_framer test/test_binary_framer.cpp)
  target_include_directories(test_binary_framer PRIVATE include)
  target_link_libraries(test_binary_framer um982_binary_framer)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(um982_binary_framer)
ament_export_targets(export_um982_binary_framer HAS_LIBRARY_TARGET)
ament_package()
