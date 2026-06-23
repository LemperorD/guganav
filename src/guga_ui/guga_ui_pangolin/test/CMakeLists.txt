# guga_ui_pangolin 单元测试
#
# 注意：渲染模块 (render_3d / render_hud / render_decision) 依赖
# Pangolin OpenGL 上下文，不能在纯 gtest 中测试。
# 此处测试的是不依赖 OpenGL 的纯逻辑层。

find_package(ament_cmake_gtest REQUIRED)
find_package(guga_ui_common REQUIRED)

# test_ui_data — 纯逻辑测试，不依赖 shm 或 Pangolin
ament_add_gtest(test_ui_data test_ui_data.cpp)
target_include_directories(test_ui_data PRIVATE
  ${CMAKE_SOURCE_DIR}/src
)
ament_target_dependencies(test_ui_data guga_ui_common)
target_link_libraries(test_ui_data pthread)

# test_data_source — 需要 shm（用 ShmWriter 注入数据）
find_package(Threads REQUIRED)

ament_add_gtest(test_data_source test_data_source.cpp)
target_include_directories(test_data_source PRIVATE
  ${CMAKE_SOURCE_DIR}/src
)
ament_target_dependencies(test_data_source guga_ui_common)
target_link_libraries(test_data_source rt pthread)
