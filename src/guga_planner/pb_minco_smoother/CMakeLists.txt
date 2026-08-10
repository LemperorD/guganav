cmake_minimum_required(VERSION 3.8)
project(pb_minco_smoother)

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-O3)
set(CMAKE_BUILD_TYPE "RELEASE")
# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# 依赖包（IntensityVoxelLayer 所在的包）
find_package(pb_nav2_plugins REQUIRED)

# interfaces 依赖（用于 MINCO 轨迹消息）
find_package(interfaces REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)

# 设置库名称
set(library_name ${PROJECT_NAME}_core)

# 创建库
add_library(${library_name} SHARED
  src/minco_smoother.cpp
)

# 包含目录
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# 链接依赖
ament_target_dependencies(${library_name}
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
  pb_nav2_plugins
  interfaces
  pcl_conversions
)

target_link_libraries(${library_name}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

# 导出插件描述文件
pluginlib_export_plugin_description_file(nav2_core minco_smoother_plugin.xml)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 安装库
install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装插件描述文件
install(FILES minco_smoother_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_targets(${library_name})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
  Eigen3
  OpenCV
  pb_nav2_plugins
  pcl_conversions
)

ament_package()
