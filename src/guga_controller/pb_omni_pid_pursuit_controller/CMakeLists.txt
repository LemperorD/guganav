cmake_minimum_required(VERSION 3.10)
project(pb_omni_pid_pursuit_controller)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
include_directories(include/pb_omni_pid_pursuit_controller)
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/omni_pid_pursuit_controller.cpp
  src/pid.cpp
  src/geometry_utils.cpp
  src/visualise.cpp
  src/pathhandler.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pid test/test_pid.cpp src/pid.cpp)
  target_include_directories(test_pid PRIVATE include)

  ament_add_gtest(test_geometry_utils test/test_geometry_utils.cpp src/geometry_utils.cpp)
  ament_target_dependencies(test_geometry_utils geometry_msgs nav_msgs)
  target_include_directories(test_geometry_utils PRIVATE include)
endif()

pluginlib_export_plugin_description_file(nav2_core pb_omni_pid_pursuit_controller.xml)

ament_auto_package()
