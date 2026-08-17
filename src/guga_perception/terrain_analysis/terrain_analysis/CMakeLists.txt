cmake_minimum_required(VERSION 3.5)
project(terrain_analysis)

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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Library（组件库必须 SHARED，plugin 动态加载）
add_library(terrain_analysis_lib SHARED
  src/terrain_analysis_node.cpp
  src/core/algorithm.cpp)

target_include_directories(terrain_analysis_lib PUBLIC include)
ament_target_dependencies(
  terrain_analysis_lib
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  pcl_ros
  pcl_conversions)



# 注册为 rclcpp component，同时生成独立可执行文件
rclcpp_components_register_node(terrain_analysis_lib
  PLUGIN "terrain_analysis::TerrainAnalysis"
  EXECUTABLE terrain_analysis_exe
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# 组件库运行时依赖：必须安装到 lib 才能被 pluginlib 加载
install(TARGETS terrain_analysis_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# ── 性能微基准（Google Benchmark）：-DBUILD_BENCHMARK=ON 开启 ──
option(BUILD_BENCHMARK "Build terrain_analysis benchmarks" OFF)
if(BUILD_BENCHMARK)
  add_subdirectory(benchmark)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_terrain_analysis test/test_terrain_analysis.cpp)
  target_include_directories(test_terrain_analysis PRIVATE test)
  target_link_libraries(test_terrain_analysis terrain_analysis_lib)

  ament_add_gtest(test_state_ingest test/test_state_ingest.cpp)
  target_include_directories(test_state_ingest PRIVATE test)
  target_link_libraries(test_state_ingest terrain_analysis_lib)

  ament_add_gtest(test_algorithm test/test_algorithm.cpp)
  target_include_directories(test_algorithm PRIVATE test)
  target_link_libraries(test_algorithm terrain_analysis_lib)

endif()

ament_export_dependencies(rclcpp_components)
ament_export_libraries(terrain_analysis_lib)

ament_package()
