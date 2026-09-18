cmake_minimum_required(VERSION 3.8)
project(tf_maintainer)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(tf_maintainer_node src/tf_maintainer_node.cpp)
ament_target_dependencies(tf_maintainer_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros
)

install(TARGETS tf_maintainer_node DESTINATION lib/${PROJECT_NAME})
ament_package()
