cmake_minimum_required(VERSION 3.8)
project(final_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(turtlebot_control_component SHARED src/waffle_turtlebot.cpp)
ament_target_dependencies(turtlebot_control_component rclcpp rclcpp_action rclcpp_lifecycle
  rclcpp_components geometry_msgs robot_interfaces)
rclcpp_components_register_nodes(turtlebot_control_component "waffle_project::TurtleBotController")


install(TARGETS
  turtlebot_control_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
