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

# ---- Hikrobot MVS SDK ----
set(HIK_MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK root")
if(DEFINED ENV{HIK_MVS_ROOT} AND NOT "$ENV{HIK_MVS_ROOT}" STREQUAL "")
  set(HIK_MVS_ROOT "$ENV{HIK_MVS_ROOT}")
endif()

unset(HIK_MVS_INCLUDE_DIR CACHE)
find_path(HIK_MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  PATHS "${HIK_MVS_ROOT}/include"
  NO_DEFAULT_PATH)
if(NOT HIK_MVS_INCLUDE_DIR)
  message(FATAL_ERROR "Hikrobot MVS header not found under ${HIK_MVS_ROOT}/include")
endif()

set(HIK_MVS_LIBRARIES)
foreach(mvs_library MvCameraControl FormatConversion MediaProcess MVRender MvUsb3vTL)
  string(TOUPPER "${mvs_library}" mvs_library_upper)
  unset(HIK_MVS_${mvs_library_upper}_LIBRARY CACHE)
  find_library(HIK_MVS_${mvs_library_upper}_LIBRARY
    NAMES "${mvs_library}"
    PATHS "${HIK_MVS_ROOT}/lib/64"
    NO_DEFAULT_PATH)
  if(NOT HIK_MVS_${mvs_library_upper}_LIBRARY)
    message(FATAL_ERROR "Hikrobot MVS library lib${mvs_library}.so not found under ${HIK_MVS_ROOT}/lib/64")
  endif()
  list(APPEND HIK_MVS_LIBRARIES "${HIK_MVS_${mvs_library_upper}_LIBRARY}")
endforeach()

# ---- HikCameraMain (HW layer, links MVS SDK, no ROS deps) ----
add_library(${PROJECT_NAME}_main SHARED
  src/hik_camera_main.cpp)
target_include_directories(${PROJECT_NAME}_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${camera_interface_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>
  ${HIK_MVS_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_main
  ${HIK_MVS_LIBRARIES}
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
