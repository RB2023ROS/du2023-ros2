cmake_minimum_required(VERSION 3.5)
project(cpp_service_tutorial)

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

find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
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

# 작성한 코드들을 빌드합니다.
add_executable(turtle_spawn_client src/turtle_spawn.cpp)
ament_target_dependencies(turtle_spawn_client rclcpp turtlesim)

add_executable(turtle_circle_server src/turtle_circle_server.cpp)
ament_target_dependencies(turtle_circle_server rclcpp geometry_msgs std_srvs)

add_executable(turtle_circle_server_advanced src/turtle_circle_server_advanced.cpp)
ament_target_dependencies(turtle_circle_server_advanced rclcpp geometry_msgs custom_interfaces)

add_executable(turtle_jail src/turtle_jail.cpp)
ament_target_dependencies(turtle_jail rclcpp turtlesim custom_interfaces)


# 빌드 완료된 실행 프로그램들은 아래 폴더에 위치합니다.
install(
  TARGETS
    turtle_jail
    turtle_spawn_client
    turtle_circle_server
    turtle_circle_server_advanced
  DESTINATION 
    lib/${PROJECT_NAME}
)

ament_package()
