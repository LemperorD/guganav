cmake_minimum_required(VERSION 3.8)
project(guga_ui_pangolin)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- 查找依赖 ----

# Pangolin (本地编译, 版本 0.9.4)
find_package(Pangolin REQUIRED)

# Eigen3 (Pangolin 依赖, 用于 3D 数学)
find_package(Eigen3 REQUIRED)

# OpenGL (平台图形)
find_package(OpenGL REQUIRED)

# guga_ui_common (共享内存通信库, 纯头文件)
# 先尝试在 worksapce 中查找
find_package(guga_ui_common REQUIRED)

# Threads (POSIX 线程, shm_reader/shm_writer 依赖)
find_package(Threads REQUIRED)

# librt (POSIX 共享内存)
if(UNIX)
  find_library(RT_LIBRARY rt)
endif()

# ---- 可执行文件 ----

add_executable(guga_ui
  src/main.cpp
)

ament_target_dependencies(guga_ui
  Pangolin
  Eigen3
  OpenGL
  guga_ui_common
)

target_include_directories(guga_ui PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
  ${guga_ui_common_INCLUDE_DIRS}
)

target_link_libraries(guga_ui
  ${Pangolin_LIBRARIES}
  ${OPENGL_LIBRARIES}
  Threads::Threads
  ${RT_LIBRARY}
)

# ---- 安装 ----

install(TARGETS guga_ui
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ---- 测试 ----

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

ament_package()
