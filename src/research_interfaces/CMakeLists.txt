cmake_minimum_required(VERSION 3.8)
project(research_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoadGraphUpdate2D.msg"
  "msg/TimedTrajectoryPoint2D.msg"
  "msg/TimedTrajectory2D.msg"
  "msg/RoadEvent.msg"
  "msg/ObservationGoal.msg"
  "msg/ResearchStatus.msg"
  "msg/OperatorPermit.msg"
  "msg/RoadEvidence2D.msg"
  "msg/LocalEvidenceGrid2D.msg"
  "srv/PlanRoadReference.srv"
  "srv/ManageResearchTask.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs nav_msgs
)
ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
ament_package()
