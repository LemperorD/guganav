cmake_minimum_required(VERSION 3.5)
project(guga_interfaces VERSION 0.0.1)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armor.msg"
  "msg/Armors.msg"
  "msg/Target.msg"
  "msg/GameStatus.msg"
  "msg/RobotStatus.msg"

  DEPENDENCIES
    geometry_msgs
    std_msgs
)


ament_package()
