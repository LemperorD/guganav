cmake_minimum_required(VERSION 3.10)
project(mpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 required for structured bindings and constexpr math
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- dependencies ----
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- OSQP (bundled via casadi or system) ----
find_library(OSQP_LIB osqp
  PATHS /home/ld/3rdparty/acados/lib
  NO_DEFAULT_PATH
)
option(HAS_OSQP "Enable OSQP solver" OFF)
if(NOT OSQP_LIB)
  find_package(osqp_vendor QUIET)
  if(osqp_vendor_FOUND)
    set(OSQP_LIB osqp::osqp)
  else()
    message(WARNING "OSQP library not found — MPC solver will be disabled.")
  endif()
endif()

# ---- header-only include ----
include_directories(include)

# ---- source files ----
set(CORE_SRC
  src/core/omni_kinematics.cpp
  src/core/path_handler.cpp
  src/core/mpc_solver.cpp
)
set(TRAJECTORY_SRC
  src/trajectory/discrete_generator.cpp
  src/trajectory/bspline_generator.cpp
  src/trajectory/minco_generator.cpp
)
set(NODE_SRC
  src/node/mpc_controller_node.cpp
)

# ---- shared library (plugin) ----
add_library(${PROJECT_NAME} SHARED
  ${CORE_SRC}
  ${TRAJECTORY_SRC}
  ${NODE_SRC}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  EIGEN_NO_AUTOMATIC_RESIZING
)

ament_target_dependencies(${PROJECT_NAME}
  nav2_core
  nav2_util
  nav2_costmap_2d
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
  nav_msgs
  geometry_msgs
  visualization_msgs
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

if(OSQP_LIB)
  target_link_libraries(${PROJECT_NAME} ${OSQP_LIB})
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_OSQP=1)
endif()

# ---- plugin registration ----
pluginlib_export_plugin_description_file(nav2_core mpc_controller.xml)

# ---- install ----
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES mpc_controller.xml
  DESTINATION share/${PROJECT_NAME}/
)

# ---- testing ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
