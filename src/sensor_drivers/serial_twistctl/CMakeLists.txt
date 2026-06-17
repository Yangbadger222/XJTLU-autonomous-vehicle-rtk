cmake_minimum_required(VERSION 3.8)
project(serial_twistctl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(serial_twistctl_node src/serial_twistctl_node.cpp)
target_include_directories(serial_twistctl_node PRIVATE include)

# 链接依赖项
ament_target_dependencies(serial_twistctl_node rclcpp geometry_msgs serial)

# 链接 yaml-cpp 与串口库
target_link_libraries(serial_twistctl_node yaml-cpp)
if(TARGET serial)
  target_link_libraries(serial_twistctl_node serial)
else()
  target_link_libraries(serial_twistctl_node ${serial_LIBRARIES})
endif()

# 安装目标
install(TARGETS
  serial_twistctl_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_gtest(test_twist_command test/test_twist_command.cpp)
  target_include_directories(test_twist_command PRIVATE include)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
