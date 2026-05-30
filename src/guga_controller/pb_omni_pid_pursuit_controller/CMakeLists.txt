cmake_minimum_required(VERSION 3.10)
project(pb_omni_pid_pursuit_controller)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
include_directories(include/pb_omni_pid_pursuit_controller)
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node/omni_pid_pursuit_controller_node.cpp
  src/core/pid.cpp
  src/core/geometry_utils.cpp
  src/core/visualise.cpp
  src/core/pathhandler.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_clang_format
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pid test/test_pid.cpp src/core/pid.cpp)
  target_include_directories(test_pid PRIVATE include)

  ament_add_gtest(test_geometry_utils test/test_geometry_utils.cpp src/core/geometry_utils.cpp)
  ament_target_dependencies(test_geometry_utils geometry_msgs nav_msgs)
  target_include_directories(test_geometry_utils PRIVATE include)

  ament_add_gtest(test_visualise test/test_visualise.cpp src/core/visualise.cpp)
  ament_target_dependencies(test_visualise geometry_msgs rclcpp_lifecycle visualization_msgs)
  target_include_directories(test_visualise PRIVATE include)

  ament_add_gtest(test_pathhandler test/test_pathhandler.cpp src/core/pathhandler.cpp)
  ament_target_dependencies(test_pathhandler nav2_costmap_2d nav_msgs tf2_ros)
  target_include_directories(test_pathhandler PRIVATE include)

  ament_add_gtest(test_types test/test_types.cpp)
  target_include_directories(test_types PRIVATE include)

  ament_add_gtest(test_approach_scaling test/test_approach_scaling.cpp
    src/node/omni_pid_pursuit_controller_node.cpp
    src/core/pathhandler.cpp
    src/core/pid.cpp
    src/core/geometry_utils.cpp
    src/core/visualise.cpp)
  ament_target_dependencies(test_approach_scaling
    nav2_costmap_2d nav_msgs tf2_ros rclcpp_lifecycle
    geometry_msgs visualization_msgs std_msgs)
  target_include_directories(test_approach_scaling PRIVATE include)
endif()

pluginlib_export_plugin_description_file(nav2_core pb_omni_pid_pursuit_controller.xml)

ament_auto_package()
