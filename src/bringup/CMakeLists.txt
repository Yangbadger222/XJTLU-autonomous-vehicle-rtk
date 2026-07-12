cmake_minimum_required(VERSION 3.5)
project(bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)
install(DIRECTORY foxglove/
  DESTINATION share/${PROJECT_NAME}/foxglove
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(PROGRAMS
  scripts/initialpose_relocalize_bridge.py
  scripts/nav2_cloud_retime.py
  scripts/localization_cmd_gate.py
  scripts/post_collision_cmd_conditioner.py
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/maps")
  install(DIRECTORY maps/
    DESTINATION share/${PROJECT_NAME}/maps
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
  install(DIRECTORY urdf/
    DESTINATION share/${PROJECT_NAME}/urdf
  )
endif()

ament_package()
