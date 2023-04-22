cmake_minimum_required(VERSION 3.5)
project(cpp_multiproc_tutorial)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(dead_lock src/dead_lock.cpp)
ament_target_dependencies(dead_lock rclcpp)

add_executable(multi_executor src/multi_executor.cpp)
ament_target_dependencies(multi_executor rclcpp)

add_executable(callback_group src/callback_group.cpp)
ament_target_dependencies(callback_group rclcpp)

add_executable(reentrant_callback_group src/reentrant_callback_group.cpp)
ament_target_dependencies(reentrant_callback_group rclcpp)


install(
  TARGETS
    dead_lock
    multi_executor
    callback_group
    reentrant_callback_group
  DESTINATION 
    lib/${PROJECT_NAME}
)

ament_package()
