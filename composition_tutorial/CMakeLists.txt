cmake_minimum_required(VERSION 3.5)
project(composition_tutorial)

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
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(rclcpp_components REQUIRED)

include_directories(include)

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

add_executable(image_pipeline_all_in_one src/image_pipeline_all_in_one.cpp)
ament_target_dependencies(image_pipeline_all_in_one rclcpp sensor_msgs OpenCV)

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp sensor_msgs OpenCV)

add_executable(watermark_node src/watermark_node.cpp)
ament_target_dependencies(watermark_node rclcpp sensor_msgs OpenCV)

add_executable(image_view_node src/image_view_node.cpp)
ament_target_dependencies(image_view_node rclcpp sensor_msgs OpenCV)

add_executable(basic_component_node src/basic_component_node.cpp)
ament_target_dependencies(basic_component_node rclcpp sensor_msgs OpenCV)

#### Components ####

add_library(basic_component SHARED
  src/basic_component.cpp)
target_compile_definitions(basic_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(basic_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(basic_component "composition::BasicComponent")
set(node_plugins "${node_plugins}composition::BasicComponent;$<TARGET_FILE:basic_component>\n")

install(TARGETS
  basic_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(talker_component SHARED src/talker_component.cpp)
target_compile_definitions(talker_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(talker_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_nodes(talker_component "composition::Talker")
set(node_plugins "${node_plugins}composition::Talker;$<TARGET_FILE:talker_component>\n")

install(TARGETS
  talker_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

  # add_library(basic_component SHARED src/basic_component.cpp)
# target_compile_definitions(basic_component PRIVATE "COMPOSITION_BUILDING_DLL")
# ament_target_dependencies(basic_component rclcpp std_msgs rclcpp_components)
# rclcpp_components_register_nodes(basic_component "composition_tutorial::BasicComponent")
# set(node_plugins "${node_plugins}composition_tutorial::BasicComponent;$<TARGET_FILE:basic_component>\n")

# install(
#   TARGETS
#     basic_component
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
######################

install(
  TARGETS
    camera_node
    watermark_node
    image_view_node
    image_pipeline_all_in_one
    basic_component_node
  DESTINATION 
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
