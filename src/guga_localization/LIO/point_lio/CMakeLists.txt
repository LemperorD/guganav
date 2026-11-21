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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
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

# Core library: 算法与调度, 不含节点层
add_library(point_lio_core STATIC
  src/utilities/OdomProcessor.cpp
  src/utilities/preprocess.cpp

  src/core/Lidar.cpp
  src/core/Synchronizer.cpp
  src/core/Imu.cpp
  src/core/FrameProcessor.cpp
  src/core/ManifoldEskf.cpp

  src/pcd/PriorMapManager.cpp
)

ament_target_dependencies(point_lio_core
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
)

target_link_libraries(point_lio_core ${PCL_LIBRARIES})

# ROS2 executable: 节点层
add_executable(pointlio_mapping
  src/ros2/laserMapping.cpp
  src/ros2/pub/ProjectOutputPublisher.cpp
  src/ros2/pub/FramePublisher.cpp
  src/ros2/sub/SensorSubscriptions.cpp
)

ament_target_dependencies(pointlio_mapping
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
)

target_link_libraries(pointlio_mapping point_lio_core ${PCL_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_odom_processor test/test_odom_processor.cpp)
  target_link_libraries(test_odom_processor point_lio_core ${PCL_LIBRARIES})
  ament_target_dependencies(test_odom_processor
    rclcpp
    pcl_ros
    pcl_conversions
  )

  ament_add_gtest(test_eskf_init test/test_eskf_init.cpp)
  target_link_libraries(test_eskf_init point_lio_core ${PCL_LIBRARIES})
  ament_target_dependencies(test_eskf_init
    rclcpp
    pcl_ros
    pcl_conversions
  )

  ament_add_gtest(test_eskf_propagation test/test_eskf_propagation.cpp)
  target_link_libraries(test_eskf_propagation point_lio_core ${PCL_LIBRARIES})
  ament_target_dependencies(test_eskf_propagation
    rclcpp
    pcl_ros
    pcl_conversions
  )

  ament_add_gtest(test_imu_calibration test/test_imu_calibration.cpp)
  target_link_libraries(test_imu_calibration point_lio_core ${PCL_LIBRARIES})
  ament_target_dependencies(test_imu_calibration
    rclcpp
    pcl_ros
    pcl_conversions
  )
endif()

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
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
  Eigen3
  PCL
)

ament_package()
