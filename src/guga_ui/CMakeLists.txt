cmake_minimum_required(VERSION 3.8)
project(guga_ui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

qt5_wrap_cpp(MOC_FILES
  include/cmdvel_visualizer/cmdvel_visualizer.hpp)

add_library(cmdvel_visualizer SHARED
  src/cmdvel_visualizer/cmdvel_visualizer.cpp
  ${MOC_FILES})

target_include_directories(cmdvel_visualizer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(cmdvel_visualizer
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_rendering::rviz_rendering
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  pluginlib::pluginlib
  Qt5::Widgets)

ament_target_dependencies(cmdvel_visualizer
  rclcpp
  geometry_msgs)

install(TARGETS cmdvel_visualizer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(FILES plugins/cmdvel_visualizer_plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(
  rviz_common plugins/cmdvel_visualizer_plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
