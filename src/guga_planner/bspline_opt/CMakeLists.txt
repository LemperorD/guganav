cmake_minimum_required(VERSION 3.8)
project(bspline_opt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── ament_cmake (for colcon build system integration) ──
find_package(ament_cmake REQUIRED)

# ── Eigen3 (system) ──
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ── Ceres Solver (local build v2.2.0) ──
set(CERES_DIR /home/ld/3rdparty/ceres-solver-2.2.0/ceres-bin)
find_library(CERES_LIB ceres
  PATHS ${CERES_DIR}/lib
  NO_DEFAULT_PATH REQUIRED)
find_path(CERES_INC ceres/ceres.h
  PATHS /home/ld/3rdparty/ceres-solver-2.2.0/include
  NO_DEFAULT_PATH REQUIRED)

find_package(glog REQUIRED)

# ── Core library (static, no ROS dependency) ──
add_library(bspline_opt_core STATIC
  src/bspline_optimizer.cpp)
target_include_directories(bspline_opt_core
  PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INC})
target_link_libraries(bspline_opt_core
  ${CERES_LIB}
  glog::glog)

# ── Tests ──
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_bspline test/test_bspline.cpp)
  target_include_directories(test_bspline PRIVATE include)
  target_link_libraries(test_bspline bspline_opt_core)

  # Joint test: JPS → B-spline (standalone, no ROS)
  add_executable(joint_test test/joint_test.cpp)
  target_include_directories(joint_test PRIVATE include test)
  target_link_libraries(joint_test bspline_opt_core)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
