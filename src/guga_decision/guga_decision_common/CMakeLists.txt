cmake_minimum_required(VERSION 3.8)
project(guga_decision_common VERSION 0.1.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(guga_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 与 guga_rmul_strategy 保持相同的 BehaviorTree.CPP v4 定位方式。Humble 的
# behaviortree_cpp ament 配置会把库路径解析错，因此这里直接找实际文件。
find_path(BT_CPP_INCLUDE_DIR behaviortree_cpp/bt_factory.h
  PATHS /opt/ros/humble/include
  NO_DEFAULT_PATH
  REQUIRED
)
find_library(BT_CPP_LIBRARY
  NAMES behaviortree_cpp
  PATHS /opt/ros/humble/lib/x86_64-linux-gnu /opt/ros/humble/lib
  NO_DEFAULT_PATH
  REQUIRED
)

add_library(guga_decision_common SHARED
  src/decision_input_hub.cpp
  src/goal_controller.cpp
  src/waypoint_navigator.cpp
)
target_include_directories(guga_decision_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(guga_decision_common PUBLIC
  rclcpp
  guga_interfaces
  geometry_msgs
  nav2_msgs
  std_msgs
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

# BehaviorTree.CPP 只在适配层暴露，核心输入缓存本身不依赖 BT。
add_library(guga_decision_bt_nodes INTERFACE)
target_include_directories(guga_decision_bt_nodes INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${BT_CPP_INCLUDE_DIR}
)
target_link_libraries(guga_decision_bt_nodes INTERFACE
  guga_decision_common
  ${BT_CPP_LIBRARY}
)

install(TARGETS guga_decision_common guga_decision_bt_nodes
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_waypoint_navigator
    test/test_waypoint_navigator.cpp
  )
  target_link_libraries(test_waypoint_navigator
    guga_decision_common
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  guga_interfaces
  geometry_msgs
  nav2_msgs
  std_msgs
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)
ament_package()
