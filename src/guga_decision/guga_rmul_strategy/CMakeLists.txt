cmake_minimum_required(VERSION 3.8)
project(guga_rmul_strategy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(guga_interfaces REQUIRED)

if(TARGET behaviortree_cpp_v3::behaviortree_cpp_v3)
  set(BT_TARGET behaviortree_cpp_v3::behaviortree_cpp_v3)
elseif(TARGET BT::behaviortree_cpp_v3)
  set(BT_TARGET BT::behaviortree_cpp_v3)
else()
  message(FATAL_ERROR "behaviortree_cpp_v3 的导入目标未找到")
endif()

# ===== 可执行文件：本地验证节点逻辑用 =====
# 自己建立 factory、自己登记节点、自己 tick，不需要启动 Nav2。
# 头文件目录是 include/，所以源码里写 #include "node.hpp" 就能找到。
add_executable(rmul_strategy_node
  src/main.cpp
  src/ROS2Monitor.cpp
)
target_include_directories(rmul_strategy_node PRIVATE include)
target_link_libraries(rmul_strategy_node
  ${BT_TARGET}
)
ament_target_dependencies(rmul_strategy_node
  rclcpp
  guga_interfaces
  ament_index_cpp
)

# ===== 将来接入 Nav2 时在这里加回共享库目标 =====
# bt_navigator 只能加载库、不能加载可执行文件，那时需要一个只放注册块的源文件：
# add_library(${PROJECT_NAME}_bt_nodes SHARED src/register.cpp)
# target_include_directories(${PROJECT_NAME}_bt_nodes PRIVATE include)
# target_link_libraries(${PROJECT_NAME}_bt_nodes ${BT_TARGET})
# target_compile_definitions(${PROJECT_NAME}_bt_nodes PRIVATE BT_PLUGIN_EXPORT)
# register.cpp 的内容是 include 头文件 + BT_REGISTER_NODES 注册块。
install(TARGETS rmul_strategy_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 行为树 XML 随包安装，供 bt_navigator 通过 default_nav_to_pose_bt_xml 之类的参数加载。
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(FILES rmul.xml
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
