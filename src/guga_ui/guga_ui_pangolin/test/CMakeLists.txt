# guga_ui_pangolin unit tests and integration tests
#
# Test categories:
#   test_ui_data      — Pure logic: UiData / UiDataSource (no shm, no Pangolin)
#   test_data_source  — Integration: ShmWriter → ShmReader (needs shm, no OpenGL)
#   test_render       — End-to-end: shm write → Pangolin render → screenshot export
#                       (needs X11 + Pangolin + OpenGL)

find_package(ament_cmake_gtest REQUIRED)
find_package(guga_ui_common REQUIRED)

# test_ui_data — pure logic, no dependencies beyond guga_ui_common
ament_add_gtest(test_ui_data test_ui_data.cpp)
target_include_directories(test_ui_data PRIVATE
  ${CMAKE_SOURCE_DIR}/src
)
ament_target_dependencies(test_ui_data guga_ui_common)
target_link_libraries(test_ui_data pthread)

# test_data_source — needs shm (uses ShmWriter to inject data)
find_package(Threads REQUIRED)

ament_add_gtest(test_data_source test_data_source.cpp)
target_include_directories(test_data_source PRIVATE
  ${CMAKE_SOURCE_DIR}/src
)
ament_target_dependencies(test_data_source guga_ui_common)
target_link_libraries(test_data_source rt pthread)

# ---- End-to-end Pangolin rendering test ----
#
# This is a standalone executable (not gtest) because it requires:
#   - An X11 display server
#   - Pangolin with OpenGL context
#   - guga_ui_pangolin render headers
#
# Output images land in <source_dir>/test/results/

find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)

if(UNIX)
  find_library(RT_LIBRARY rt)
endif()

add_executable(test_render test_pangolin_render.cpp)
target_include_directories(test_render PRIVATE
  ${CMAKE_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIRS}
)
# Use the installed guga_ui_common namespace target for includes+deps
target_link_libraries(test_render
  guga_ui_common::guga_ui_common
  ${Pangolin_LIBRARIES}
  ${OPENGL_LIBRARIES}
  Threads::Threads
  ${RT_LIBRARY}
)
# CTest integration
add_test(NAME test_render_pangolin
         COMMAND test_render
         WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set_tests_properties(test_render_pangolin PROPERTIES
  LABELS "pangolin;rendering;integration")
