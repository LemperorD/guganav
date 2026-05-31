cmake_minimum_required(VERSION 3.10)
project(buaa_sentry_relocalization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(teaserpp REQUIRED)
find_package(PCL REQUIRED)
find_package(robin REQUIRED)
find_package(kiss_matcher REQUIRED)
ament_auto_find_build_dependencies()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME}
  OpenMP::OpenMP_CXX
  kiss_matcher::kiss_matcher_core
  teaserpp::teaser_registration
  teaserpp::teaser_io
  TBB::tbb
  robin::robin
  ${PCL_LIBRARIES_DIRS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${PCL_INCLUDE_DIRS}
  /usr/local/include
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN buaa_sentry_relocalization::BuaaSentryRelocalizationNode
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)
