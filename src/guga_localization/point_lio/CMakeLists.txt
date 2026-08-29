cmake_minimum_required(VERSION 3.8)
project(point_lio LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

install(
  DIRECTORY include/point_lio
  DESTINATION include
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
)

# Declare a ROS2 executable
add_executable(pointlio_mapping
  src/laserMapping.cpp
  src/parameters.cpp
  src/preprocess.cpp
  src/Lidar.cpp
  src/Synchronizer.cpp
  src/Filter.cpp
  src/IMU_Processing.cpp
  src/Imu.cpp
  src/FrameProcessor.cpp
)

ament_target_dependencies(pointlio_mapping
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
)

target_link_libraries(pointlio_mapping ${PCL_LIBRARIES})

# Install the executable
install(TARGETS
  pointlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
  Eigen3
  PCL
)

ament_package()
