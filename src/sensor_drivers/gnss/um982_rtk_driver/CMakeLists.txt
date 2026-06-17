cmake_minimum_required(VERSION 3.8)
project(um982_rtk_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_library(um982_rtk_parser
  src/nmea_parser.cpp
  src/ntrip_response.cpp
)
target_include_directories(um982_rtk_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(um982_rtk_node
  src/um982_rtk_node.cpp
)
target_include_directories(um982_rtk_node PRIVATE include)
target_link_libraries(um982_rtk_node um982_rtk_parser)
ament_target_dependencies(um982_rtk_node
  geometry_msgs
  nmea_msgs
  rclcpp
  sensor_msgs
  serial
  std_msgs
  tf2
)

install(TARGETS um982_rtk_parser um982_rtk_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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
  ament_add_gtest(test_nmea_parser test/test_nmea_parser.cpp)
  target_include_directories(test_nmea_parser PRIVATE include)
  target_link_libraries(test_nmea_parser um982_rtk_parser)

  ament_add_gtest(test_ntrip_response test/test_ntrip_response.cpp)
  target_include_directories(test_ntrip_response PRIVATE include)
  target_link_libraries(test_ntrip_response um982_rtk_parser)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(um982_rtk_parser)
ament_package()
