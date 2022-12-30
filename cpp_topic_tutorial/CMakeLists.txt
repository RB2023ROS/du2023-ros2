cmake_minimum_required(VERSION 3.5)
project(cpp_topic_tutorial)

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
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

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
add_executable(topic_pub_node src/topic_example_1_publisher.cpp)
ament_target_dependencies(topic_pub_node rclcpp geometry_msgs)

add_executable(topic_sub_node src/topic_example_2_subscriber.cpp)
ament_target_dependencies(topic_sub_node rclcpp turtlesim)

add_executable(mimic_node src/topic_example_3_mimic.cpp)
ament_target_dependencies(mimic_node rclcpp geometry_msgs)

# 빌드 완료된 실행 프로그램들은 아래 폴더에 위치합니다.
install(
  TARGETS
    topic_pub_node
    topic_sub_node
    mimic_node
  DESTINATION 
    lib/${PROJECT_NAME}
)

ament_package()
