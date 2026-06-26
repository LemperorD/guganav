cmake_minimum_required(VERSION 3.16)

project(guga_test)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCL REQUIRED)
find_package(TBB REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
    include
)

add_executable(
    test_intelGpuMatrix
    src/test_intelGpuMatrix.cpp
)

add_executable(
    test_pangolin
    src/test_pangolin.cpp
)

target_link_libraries(
    test_intelGpuMatrix
    OpenCL::OpenCL
)

target_link_libraries(test_pangolin pango_display pango_python )

set_target_properties(
    test_intelGpuMatrix
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)