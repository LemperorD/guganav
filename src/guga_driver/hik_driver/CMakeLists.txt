cmake_minimum_required(VERSION 3.8)
project(hik_driver)

# C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---- Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(camera_interface REQUIRED)

# ---- HikCameraMain (HW layer, links MVS SDK, no ROS deps) ----
add_library(${PROJECT_NAME}_main SHARED
  src/hik_camera_main.cpp)
target_include_directories(${PROJECT_NAME}_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${camera_interface_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>
  /opt/MVS/include)
target_link_directories(${PROJECT_NAME}_main PUBLIC
  /opt/MVS/lib/64)
target_link_libraries(${PROJECT_NAME}_main
  MvCameraControl
  FormatConversion
  MediaProcess
  MVRender
  MvUsb3vTL
)

# ---- HikCameraNode (ROS2 layer) ----
add_library(${PROJECT_NAME}_node SHARED
  src/hik_camera_node.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_main)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  camera_info_manager
  camera_interface
)

# Register as rclcpp component + generate standalone executable
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "hik_driver::HikCameraNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# ---- Install ----
install(TARGETS
  ${PROJECT_NAME}_main
  ${PROJECT_NAME}_node
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

# ---- Testing ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
