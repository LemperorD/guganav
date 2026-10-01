cmake_minimum_required(VERSION 3.8)
# 版本号写在 project() 的参数里，CMake 没有独立的 version() 命令；
# 取值与 package.xml 的 <version> 一致。
project(guga_rmul_strategy VERSION 0.0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(guga_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# ===== BehaviorTree.CPP v4 =====
# 这里不用 find_package(behaviortree_cpp)：Humble 上这个包的 ament 配置只搜
# <前缀>/lib，而库实际装在 lib/x86_64-linux-gnu，配置阶段会直接报
# "exports the library 'behaviortree_cpp' which couldn't be found"。
# 所以头文件目录和库文件都手动定位。换机器时只需改下面两处 PATHS。
find_path(BT_CPP_INCLUDE_DIR behaviortree_cpp/bt_factory.h
  PATHS /opt/ros/humble/include
  NO_DEFAULT_PATH
  REQUIRED
)
find_library(BT_CPP_LIBRARY
  NAMES behaviortree_cpp
  PATHS /opt/ros/humble/lib/x86_64-linux-gnu /opt/ros/humble/lib
  NO_DEFAULT_PATH
  REQUIRED
)
message(STATUS "BehaviorTree.CPP v4: 头文件 ${BT_CPP_INCLUDE_DIR}，库 ${BT_CPP_LIBRARY}")

# ===== 可执行文件：自己建立 factory、自己 tick =====
# 与 Nav2 之间只通过话题通信（goal_pose），不共享 BT 库，
# 所以这里用 v4 而 Nav2 用 v3 并不冲突。
add_executable(rmul_strategy_node
  src/main.cpp
  src/ROS2Monitor.cpp
  src/GoalPoseSender.cpp
)
target_include_directories(rmul_strategy_node PRIVATE
  include
  ${BT_CPP_INCLUDE_DIR}
)
target_link_libraries(rmul_strategy_node
  ${BT_CPP_LIBRARY}
)
ament_target_dependencies(rmul_strategy_node
  rclcpp
  guga_interfaces
  geometry_msgs
  ament_index_cpp
)

install(TARGETS rmul_strategy_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 行为树 XML 随包安装，程序运行时按包名解析路径后加载。
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
