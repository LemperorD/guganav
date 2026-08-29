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
set(ACADOS_GENERATED_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/generated" CACHE PATH
  "Root directory containing generated acados solver code")
set(GENERATED_OCP_DIR ${ACADOS_GENERATED_ROOT}/omni/omni_ocp)

file(GLOB MODEL_SRC   ${GENERATED_OCP_DIR}/omni_model/*.c)
file(GLOB COST_SRC    ${GENERATED_OCP_DIR}/omni_cost/*.c)
file(GLOB SOLVER_SRC  ${GENERATED_OCP_DIR}/acados_solver_omni.c)

set(GENERATED_SRC
  ${MODEL_SRC}
  ${COST_SRC}
  ${SOLVER_SRC}
)

ament_auto_add_library(mpc_wrapper SHARED
  src/mpc_wrapper.cpp
  ${GENERATED_SRC}
)

target_include_directories(mpc_wrapper PUBLIC
  ${ACADOS_INCLUDE_DIR}
  ${ACADOS_INCLUDE_DIR}/acados
  ${ACADOS_INCLUDE_DIR}/blasfeo/include
  ${ACADOS_INCLUDE_DIR}/hpipm/include
  ${GENERATED_OCP_DIR}
)

target_link_directories(mpc_wrapper PUBLIC
  ${ACADOS_LIB_DIR}
)

target_link_libraries(mpc_wrapper
  acados
  hpipm
  blasfeo
  m
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/mpc_controller_node.cpp
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${guga_common_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "mpc_controller::MpcControllerNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)
target_link_libraries(${PROJECT_NAME}_node
  mpc_wrapper
  dw
  ${guga_common_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME}_node PRIVATE BACKWARD_HAS_DW=1)

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

pluginlib_export_plugin_description_file(nav2_core mpc_controller.xml)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
    generated
)
