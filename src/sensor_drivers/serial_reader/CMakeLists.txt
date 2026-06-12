cmake_minimum_required(VERSION 3.8)
project(serial_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(frc_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(serial_reader_node src/serial_reader_node.cpp)

ament_target_dependencies(serial_reader_node
  rclcpp
  nav_msgs
  sensor_msgs
  frc_msgs
)

# 链接 yaml-cpp 库
target_link_libraries(serial_reader_node yaml-cpp)

# 如果在你的系统上需要显式链接 pthread，可以取消下面一行的注释
# target_link_libraries(serial_reader_node pthread)

install(TARGETS
  serial_reader_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过版权检查
  set(ament_cmake_copyright_FOUND TRUE)
  # 跳过 cpplint
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
