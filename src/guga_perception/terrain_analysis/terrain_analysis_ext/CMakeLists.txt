cmake_minimum_required(VERSION 3.5)
project(terrain_analysis_ext)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Library
add_library(terrain_analysis_ext_lib src/terrain_analysis_ext_node.cpp src/core/algorithm.cpp)
target_include_directories(terrain_analysis_ext_lib PUBLIC include)
ament_target_dependencies(
  terrain_analysis_ext_lib
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions)

# Executable
add_executable(terrainAnalysisExt src/main_ext.cpp)
target_link_libraries(terrainAnalysisExt terrain_analysis_ext_lib)
install(TARGETS terrainAnalysisExt DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
