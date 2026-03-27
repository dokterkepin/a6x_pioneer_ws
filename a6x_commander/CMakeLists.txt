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
find_package(rviz_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)
add_executable(hello_moveit src/hello_moveit.cpp)
add_executable(commander src/commander.cpp)
add_executable(mtc_tutorial src/mtc_tutorial.cpp)
add_executable(position src/position.cpp)

ament_target_dependencies(position rclcpp moveit_ros_planning_interface moveit_visual_tools rviz_visual_tools geometry_msgs graph_msgs tf2 example_interfaces)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface geometry_msgs graph_msgs tf2 example_interfaces)
ament_target_dependencies(hello_moveit rclcpp moveit_ros_planning_interface moveit_visual_tools rviz_visual_tools graph_msgs)
ament_target_dependencies(commander rclcpp moveit_ros_planning_interface geometry_msgs graph_msgs tf2 example_interfaces std_msgs)
ament_target_dependencies(mtc_tutorial 
  moveit_task_constructor_core 
  rclcpp 
  std_msgs 
  tf2_msgs 
  tf2_ros 
  geometry_msgs 
  graph_msgs
  moveit_ros_planning_interface
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  position
  test_moveit
  hello_moveit
  commander
  mtc_tutorial
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
