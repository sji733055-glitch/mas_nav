cmake_minimum_required(VERSION 3.8)
project(map_server)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree filters segmentation)
find_package(tf2_ros REQUIRED)
find_path(MSGPACK_INCLUDE_DIR msgpack.hpp REQUIRED)

add_library(${PROJECT_NAME}_core SHARED
  src/map_server_node.cpp
  src/utils.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC include ${MSGPACK_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME}_core PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_core ${OpenCV_LIBS} ${PCL_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_core rclcpp rclcpp_components nav_msgs sensor_msgs tf2_ros)
rclcpp_components_register_nodes(${PROJECT_NAME}_core "map_server::MapServerNode")

add_executable(map_server_node src/main.cpp)
target_link_libraries(map_server_node ${PROJECT_NAME}_core)
ament_target_dependencies(map_server_node rclcpp)

install(TARGETS ${PROJECT_NAME}_core map_server_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/pgm_to_terrain_msgpack.py DESTINATION lib/${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()
