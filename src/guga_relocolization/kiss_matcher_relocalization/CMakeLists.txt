cmake_minimum_required(VERSION 3.10)
project(kiss_matcher_relocalization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

add_definitions(-Wall -Werror)

add_compile_options(-fPIC)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)
find_package(teaserpp REQUIRED)
find_package(PCL REQUIRED)
find_package(robin REQUIRED)
find_package(kiss_matcher REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME}
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
  PLUGIN kiss_matcher_relocalization::KissMatcherRelocalizationNode
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

# ament_package()
ament_auto_package(
  INSTALL_TO_SHARE
)
