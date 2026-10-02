cmake_minimum_required(VERSION 3.8)
project(fake_msg_source)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(guga_interfaces REQUIRED)
# 假位姿：发 odometry 话题与 odom -> base_footprint 变换，决策节点据此判断是否到达。
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(fake_msg_source_node
  src/fake_msg_source_node.cpp
)
target_include_directories(fake_msg_source_node PRIVATE include)
ament_target_dependencies(fake_msg_source_node
  rclcpp
  guga_interfaces
  nav_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS fake_msg_source_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
