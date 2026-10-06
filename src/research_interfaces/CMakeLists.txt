cmake_minimum_required(VERSION 3.8)
project(research_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TimedTrajectoryPoint2D.msg"
  "msg/TimedTrajectory2D.msg"
  "msg/RoadEvent.msg"
  "msg/ObservationGoal.msg"
  "msg/ResearchStatus.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
