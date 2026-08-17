find_package(benchmark REQUIRED)

set(BENCHMARK_NAMES
  terrain_analysis_benchmark
)

foreach(name IN LISTS BENCHMARK_NAMES)
  add_executable(${name}
    ${name}.cpp
  )
  ament_target_dependencies(${name}
    rclcpp
    sensor_msgs
    nav_msgs
    pcl_ros
    pcl_conversions
  )
  target_link_libraries(${name}
    terrain_analysis_lib benchmark
  )
endforeach()
