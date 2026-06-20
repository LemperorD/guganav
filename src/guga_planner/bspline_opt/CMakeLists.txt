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

# ── Core library (static, NO external deps beyond Eigen3) ──
add_library(bspline_opt_core STATIC
  src/bspline_optimizer.cpp)
target_include_directories(bspline_opt_core
  PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS})

# ── Tests ──
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_bspline test/test_bspline.cpp)
  target_include_directories(test_bspline PRIVATE include)
  target_link_libraries(test_bspline bspline_opt_core)

  add_executable(joint_test test/joint_test.cpp)
  target_include_directories(joint_test PRIVATE include test)
  target_link_libraries(joint_test bspline_opt_core)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
