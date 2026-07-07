cmake_minimum_required(VERSION 3.8)
project(jps_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ── B-spline 平滑依赖 ──
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_add_library(jps_planner SHARED
  src/jps_planner.cpp
  src/jps_algorithm.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../bspline_opt/src/bspline_optimizer.cpp
)
target_include_directories(jps_planner
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../bspline_opt/include
    ${EIGEN3_INCLUDE_DIRS})

pluginlib_export_plugin_description_file(nav2_core jps_planner_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_jps
    test/test_jps.cpp
    src/jps_algorithm.cpp
  )
  target_include_directories(test_jps PRIVATE include test)
  target_compile_definitions(test_jps PRIVATE TESTING)

  # JPS visualization data exporter (standalone, no ROS needed)
  add_executable(jps_viz_exporter
    test/jps_viz_exporter.cpp
    src/jps_algorithm.cpp
  )
  target_include_directories(jps_viz_exporter PRIVATE include test)
  target_compile_definitions(jps_viz_exporter PRIVATE TESTING)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
