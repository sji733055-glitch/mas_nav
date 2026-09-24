cmake_minimum_required(VERSION 3.8)
project(ros2_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RefereeData.msg"
  DEPENDENCIES geometry_msgs
)

add_executable(ros2_comm_node src/ros2_comm.cpp)
ament_target_dependencies(ros2_comm_node rclcpp geometry_msgs interfaces)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ros2_comm_node "${cpp_typesupport_target}")


install(TARGETS
  ros2_comm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
