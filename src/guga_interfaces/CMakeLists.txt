cmake_minimum_required(VERSION 3.5)
project(guga_interfaces VERSION 0.0.1)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/enemy/Armor.msg"
  "msg/enemy/Armors.msg"
  "msg/enemy/Target.msg"
  "msg/referee/GameStatus.msg"
  "msg/referee/RobotStatus.msg"
  "msg/referee/RfidStatus.msg"
  "msg/MincoTrajectory.msg"

  "action/EscapeObstacleLayer.action"

  DEPENDENCIES
  geometry_msgs
  std_msgs
  builtin_interfaces
)

ament_package()
