cmake_minimum_required(VERSION 3.8)
project(guga_ui_common)

# C++17 for std::is_trivially_copyable_v and inline constexpr
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# Header-only INTERFACE library: no compilation needed
# Consumers only need to include the headers and link -lrt -pthread
add_library(guga_ui_common INTERFACE)
target_include_directories(guga_ui_common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# shm_writer/shm_reader use POSIX shm (librt) and std::atomic
target_link_libraries(guga_ui_common INTERFACE
  rt
  pthread
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Export as a proper CMake target for other packages
install(TARGETS guga_ui_common
  EXPORT ${PROJECT_NAME}Targets
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake)

ament_package()
