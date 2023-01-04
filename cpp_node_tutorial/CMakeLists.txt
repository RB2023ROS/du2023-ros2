cmake_minimum_required(VERSION 3.5)
project(cpp_node_tutorial)

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

# find_package를 통해 종속성들을 추가합니다.
find_package(rclcpp REQUIRED)

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
add_executable(example_node_1 src/node_example_1.cpp)
ament_target_dependencies(example_node_1 rclcpp)

add_executable(example_node_2 src/node_example_2.cpp)
ament_target_dependencies(example_node_2 rclcpp)

add_executable(example_node_3 src/node_example_3.cpp)
ament_target_dependencies(example_node_3 rclcpp)

add_executable(example_node_4 src/node_example_4.cpp)
ament_target_dependencies(example_node_4 rclcpp)

add_executable(example_node_5 src/node_example_5.cpp)
ament_target_dependencies(example_node_5 rclcpp)


# 빌드 완료된 실행 프로그램들은 아래 폴더에 위치합니다.
install(
  TARGETS
    example_node_1
    example_node_2
    example_node_3
    example_node_4
    example_node_5
  DESTINATION 
    lib/${PROJECT_NAME}
)

ament_package()
