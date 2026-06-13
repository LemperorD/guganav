cmake_minimum_required(VERSION 3.8)
project(serial_driver)

# C++17 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译器警告（编码规范要求 Wall/Wextra/Wpedantic）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(guga_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

# ---- 串口底层库 (lib) ----
add_library(serial_driver_main SHARED
  src/serial_driver_main.cpp)
target_include_directories(serial_driver_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(serial_driver_main
  rclcpp)
# 链接系统库: 串口 和 文件系统
target_link_libraries(serial_driver_main
  stdc++fs
)

# ---- ROS2 节点层库 (lib) ----
add_library(serial_driver_node SHARED
  src/serial_driver_node.cpp)
target_link_libraries(serial_driver_node
  serial_driver_main)
ament_target_dependencies(serial_driver_node
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  guga_interfaces
  nlohmann_json
)

# 注册为 rclcpp component，同时生成独立可执行文件
rclcpp_components_register_node(serial_driver_node
  PLUGIN "serial_driver::SerialDriverNode"
  EXECUTABLE serial_driver_node
)

# ---- 安装 ----
install(TARGETS
  serial_driver_main
  serial_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# ---- 测试 ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
