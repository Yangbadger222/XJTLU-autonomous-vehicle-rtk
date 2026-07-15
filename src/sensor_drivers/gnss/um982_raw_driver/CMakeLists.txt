cmake_minimum_required(VERSION 3.8)
project(um982_raw_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
add_library(um982_binary_framer
  src/binary_framer.cpp
  src/mixed_stream_framer.cpp
  src/ephemeris_decoder.cpp
  src/epoch_deduplicator.cpp
  src/observation_decoder.cpp
)
target_include_directories(um982_binary_framer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS um982_binary_framer
  EXPORT export_um982_binary_framer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_binary_framer test/test_binary_framer.cpp)
  target_include_directories(test_binary_framer PRIVATE include)
  target_link_libraries(test_binary_framer um982_binary_framer)

  ament_add_gtest(test_mixed_stream_framer test/test_mixed_stream_framer.cpp)
  target_include_directories(test_mixed_stream_framer PRIVATE include)
  target_link_libraries(test_mixed_stream_framer um982_binary_framer)

  ament_add_gtest(test_observation_decoder test/test_observation_decoder.cpp)
  target_include_directories(test_observation_decoder PRIVATE include)
  target_link_libraries(test_observation_decoder um982_binary_framer)

  ament_add_gtest(test_epoch_deduplicator test/test_epoch_deduplicator.cpp)
  target_include_directories(test_epoch_deduplicator PRIVATE include)
  target_link_libraries(test_epoch_deduplicator um982_binary_framer)

  ament_add_gtest(test_ephemeris_decoder test/test_ephemeris_decoder.cpp)
  target_include_directories(test_ephemeris_decoder PRIVATE include)
  target_link_libraries(test_ephemeris_decoder um982_binary_framer)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(um982_binary_framer)
ament_export_targets(export_um982_binary_framer HAS_LIBRARY_TARGET)
ament_package()
