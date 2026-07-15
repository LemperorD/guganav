cmake_minimum_required(VERSION 3.8)
project(mpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
ament_auto_find_build_dependencies()

# ---- acados paths ----
set(ACADOS_INCLUDE_DIR $ENV{ACADOS_SOURCE_DIR}/include)
set(ACADOS_LIB_DIR $ENV{ACADOS_SOURCE_DIR}/lib)

# ---- generated solver source directory ----
set(GENERATED_OCP_DIR ${CMAKE_SOURCE_DIR}/generated/omni/omni_ocp)

ament_auto_add_library(mpc_wrapper SHARED
  src/core/mpc_wrapper.cpp
)

# ---- include directories ----
target_include_directories(mpc_wrapper PRIVATE
  ${ACADOS_INCLUDE_DIR}
  ${ACADOS_INCLUDE_DIR}/acados
  ${ACADOS_INCLUDE_DIR}/blasfeo/include
  ${ACADOS_INCLUDE_DIR}/hpipm/include
  ${GENERATED_OCP_DIR}
)

ament_auto_add_library(nav2_wrapper SHARED
  src/core/nav2_wrapper.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/node/mpc_controller_node.cpp
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "mpc_controller::MpcControllerNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)
target_link_libraries(${PROJECT_NAME}_node 
  ${PROJECT_NAME}_main
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
