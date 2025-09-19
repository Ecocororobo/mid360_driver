cmake_minimum_required(VERSION 3.8)
project(mid360_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Find Livox SDK
find_library(LIVOX_LIDAR_SDK_LIBRARY
  NAMES livox_lidar_sdk_shared
  PATHS /usr/local/lib
  REQUIRED
)

find_path(LIVOX_LIDAR_SDK_INCLUDE_DIR
  NAMES livox_lidar_api.h livox_lidar_def.h
  PATHS /usr/local/include
  REQUIRED
)

# Executable
add_executable(mid360_driver_node src/mid360_driver_node.cpp)
ament_target_dependencies(mid360_driver_node rclcpp sensor_msgs std_msgs)

# Include & link Livox SDK
target_include_directories(mid360_driver_node PRIVATE
  ${livox_sdk_INCLUDE_DIRS}
)
target_link_libraries(mid360_driver_node
  ${LIVOX_LIDAR_SDK_LIBRARY}
)

# Install
install(TARGETS
  mid360_driver_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()
