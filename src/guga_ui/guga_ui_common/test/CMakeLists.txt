# guga_ui_common 单元测试
find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_ui_types test_ui_types.cpp)
target_include_directories(test_ui_types PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(test_ui_types rt pthread)

ament_add_gtest(test_shm_layout test_shm_layout.cpp)
target_include_directories(test_shm_layout PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(test_shm_layout rt pthread)

ament_add_gtest(test_shm_write_read test_shm_write_read.cpp)
target_include_directories(test_shm_write_read PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(test_shm_write_read rt pthread)
