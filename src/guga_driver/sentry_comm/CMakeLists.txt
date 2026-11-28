cmake_minimum_required(VERSION 3.16)
project(sentry_comm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(guga_interfaces REQUIRED)

# 客户端源码与共享定义来自 Sentry2Communicator。默认依次查找工作区同级与工作区内部，
# 两个位置都不对时用 -DSENTRY2_COMMUNICATOR_DIR=/path/to/Sentry2Communicator 指定。
set(SENTRY2_COMMUNICATOR_DIR "" CACHE PATH "Sentry2Communicator 源码目录")
if(NOT SENTRY2_COMMUNICATOR_DIR)
  foreach(_candidate
      "${CMAKE_CURRENT_SOURCE_DIR}/../../../../Sentry2Communicator"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../../Sentry2Communicator")
    if(EXISTS "${_candidate}/ipc_client/include/SerialCommClient.hpp")
      set(SENTRY2_COMMUNICATOR_DIR "${_candidate}")
      break()
    endif()
  endforeach()
endif()
if(NOT EXISTS "${SENTRY2_COMMUNICATOR_DIR}/ipc_client/include/SerialCommClient.hpp")
  message(FATAL_ERROR
    "找不到 Sentry2Communicator 客户端头文件，请用 -DSENTRY2_COMMUNICATOR_DIR 指定源码目录")
endif()

add_library(${PROJECT_NAME}_lib SHARED
  src/command_writer.cpp
  src/snapshot_reader.cpp
  ${SENTRY2_COMMUNICATOR_DIR}/ipc_client/src/IpcClient.cpp
  ${SENTRY2_COMMUNICATOR_DIR}/ipc_client/src/SerialCommClient.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${SENTRY2_COMMUNICATOR_DIR}/ipc_client/include>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/sentry_comm/s2c_client>
)

target_link_libraries(${PROJECT_NAME}_lib PUBLIC Threads::Threads)

install(TARGETS ${PROJECT_NAME}_lib EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ${SENTRY2_COMMUNICATOR_DIR}/ipc_client/include/
  DESTINATION include/sentry_comm/s2c_client
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Threads)

add_executable(nav_command_node src/nav_command_node.cpp)
target_link_libraries(nav_command_node
  ${PROJECT_NAME}_lib
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
)
install(TARGETS nav_command_node DESTINATION lib/${PROJECT_NAME})

add_executable(referee_bridge_node
  src/referee_bridge_node.cpp
  src/referee_mapping.cpp
)
target_include_directories(referee_bridge_node PRIVATE src)
target_link_libraries(referee_bridge_node
  ${PROJECT_NAME}_lib
  rclcpp::rclcpp
  ${guga_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
)
install(TARGETS referee_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 测试用共享内存服务端：发布 AllInfo、接收 VisWrite / NavWrite。
  add_library(${PROJECT_NAME}_test_support STATIC
    ${SENTRY2_COMMUNICATOR_DIR}/src/ipc/IpcServer.cpp
  )
  target_include_directories(${PROJECT_NAME}_test_support PUBLIC
    ${SENTRY2_COMMUNICATOR_DIR}/include
  )
  target_link_libraries(${PROJECT_NAME}_test_support PUBLIC ${PROJECT_NAME}_lib)

  ament_add_gtest(snapshot_reader_test test/snapshot_reader_test.cpp)
  target_link_libraries(snapshot_reader_test ${PROJECT_NAME}_test_support)

  ament_add_gtest(command_writer_test test/command_writer_test.cpp)
  target_link_libraries(command_writer_test ${PROJECT_NAME}_test_support)

  ament_add_gtest(referee_mapping_test
    test/referee_mapping_test.cpp
    src/referee_mapping.cpp
  )
  target_include_directories(referee_mapping_test PRIVATE src)
  target_link_libraries(referee_mapping_test
    ${PROJECT_NAME}_lib
    ${guga_interfaces_TARGETS}
    ${sensor_msgs_TARGETS}
  )
endif()

ament_package()
