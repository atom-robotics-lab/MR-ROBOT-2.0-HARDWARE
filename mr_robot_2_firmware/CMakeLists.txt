cmake_minimum_required(VERSION 3.8)
project(mr_robot_2_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Specify additional locations of header files
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
)

# Install scripts
# install(PROGRAMS
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY
  launch
  scripts
  DESTINATION
  share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
