cmake_minimum_required(VERSION 3.8)
project(a6x_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)
add_executable(hello_moveit src/hello_moveit.cpp)
add_executable(commander src/commander.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface geometry_msgs tf2 example_interfaces)
ament_target_dependencies(hello_moveit rclcpp moveit_ros_planning_interface moveit_visual_tools)
ament_target_dependencies(commander rclcpp moveit_ros_planning_interface geometry_msgs tf2 example_interfaces std_msgs)
install(TARGETS
  test_moveit
  hello_moveit
  commander
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
