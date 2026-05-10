cmake_minimum_required(VERSION 3.8)
project(simple_decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(guga_interfaces REQUIRED)

include_directories(include)

# ===== Component library =====
add_library(simple_decision_component SHARED
  src/simple_decision.cpp
  src/decision.cpp
  src/transform.cpp
  src/context_environment_context.cpp
)

ament_target_dependencies(simple_decision_component
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  nav2_msgs
  guga_interfaces
)

# Register component plugin
rclcpp_components_register_nodes(simple_decision_component "simple_decision::DecisionSimple")

# ===== Install =====
install(TARGETS
  simple_decision_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tf2_ros REQUIRED)

  # Macro to reduce boilerplate
  macro(add_decision_test TEST_NAME TEST_SRC)
    ament_add_gtest(${TEST_NAME} ${TEST_SRC})
    if(TARGET ${TEST_NAME})
      target_link_libraries(${TEST_NAME} simple_decision_component)
      target_include_directories(${TEST_NAME} PRIVATE test)
      ament_target_dependencies(${TEST_NAME}
        rclcpp
        std_msgs
        geometry_msgs
        tf2
        tf2_ros
        guga_interfaces
      )
    endif()
  endmacro()

  add_decision_test(test_transform             test/test_transform.cpp)
  add_decision_test(test_environment_context   test/test_environment_context.cpp)
  add_decision_test(test_decision              test/test_decision.cpp)
  add_decision_test(test_simple_decision        test/test_simple_decision.cpp)
endif()

ament_package()
