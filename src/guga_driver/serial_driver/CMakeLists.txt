cmake_minimum_required(VERSION 3.8)
project(serial_driver)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译器警告（编码规范要求 Wall/Wextra/Wpedantic）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_main SHARED
  src/serial_driver_main.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/serial_driver_node.cpp
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${guga_common_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "serial_driver::SerialDriverNode"
  EXECUTABLE serial_driver_node_exe
)
target_link_libraries(${PROJECT_NAME}_node 
  ${PROJECT_NAME}_main
)

# ---- 测试 ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
    launch
    config
)

