cmake_minimum_required(VERSION 3.8)
project(rmul_decision)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(guga_decision_common REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(rmul_decision
  src/condition_node/checkHP.cpp
  src/condition_node/checkTime.cpp
  src/condition_node/IsOccupationsafe.cpp
  src/condition_node/IsRevival.cpp
  src/condition_node/IsGameStart.cpp
  src/action_node/changelowerstate.cpp
  src/action_node/changehigherstate.cpp
  src/action_node/GotoPoints.cpp
  src/main.cpp
)

target_include_directories(rmul_decision PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(rmul_decision 
   guga_decision_common::guga_decision_bt_nodes
) 
ament_target_dependencies(rmul_decision
  ament_index_cpp
  behaviortree_cpp
  diagnostic_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  std_msgs
)

install(TARGETS rmul_decision
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY bt_trees launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
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
