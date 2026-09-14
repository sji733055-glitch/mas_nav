cmake_minimum_required(VERSION 3.10)
project(mas2027_nav_executor)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  # PCLConfig resets its policy version to 3.10 before finding FLANN.
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(rog_map REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(mpc_source_dir "${CMAKE_CURRENT_SOURCE_DIR}/vendor/mpc")
file(GLOB qpoases_sources "${mpc_source_dir}/qpOASES/src/*.cpp")
add_library(${PROJECT_NAME}_qpoases STATIC ${qpoases_sources})
set_target_properties(${PROJECT_NAME}_qpoases PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_qpoases PUBLIC
  "${mpc_source_dir}/qpOASES/include")
target_compile_definitions(${PROJECT_NAME}_qpoases PRIVATE __NO_COPYRIGHT__ LINUX)
target_compile_options(${PROJECT_NAME}_qpoases PRIVATE -Wno-format-truncation)

add_library(${PROJECT_NAME}_mpc STATIC
  src/path_executor/mpc/mpc_solver.cpp)
target_include_directories(${PROJECT_NAME}_mpc PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(${PROJECT_NAME}_mpc PUBLIC Eigen3::Eigen ${PROJECT_NAME}_qpoases)

set(minco_source_dir "${CMAKE_CURRENT_SOURCE_DIR}/vendor/minco")
file(GLOB minco_utils_sources "${minco_source_dir}/src/utils/*.cpp")
file(GLOB minco_traj_sources "${minco_source_dir}/src/traj_opt/*.cpp")
add_library(${PROJECT_NAME}_planner STATIC
  ${minco_utils_sources}
  ${minco_traj_sources}
  src/path_planner/search/astar.cpp
  src/path_planner/search/global_path_searcher.cpp
  src/path_planner/search/omni_kino_astar.cpp
  src/path_planner/trajectory/corridor_generator.cpp
  src/path_planner/trajectory/arc_length_speed_profile.cpp
  src/path_planner/trajectory/local_path_processor.cpp
  src/path_planner/trajectory/minco_optimizer.cpp
  src/path_planner/trajectory/minco_planner.cpp
  src/path_planner/trajectory/trajectory_safety_checker.cpp
  "${minco_source_dir}/src/components/planner_mode_context.cpp"
  src/task_manager/recovery/recovery_behaivor.cpp
  "${minco_source_dir}/src/minco_utils.cpp"
  "${minco_source_dir}/src/visualizer.cpp"
  "${minco_source_dir}/src/performance/planner_performance_monitor.cpp")
target_include_directories(${PROJECT_NAME}_planner PUBLIC
  "${minco_source_dir}/include" "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_compile_definitions(${PROJECT_NAME}_planner PUBLIC
  ROOT_DIR="${minco_source_dir}/" ORIGIN_AT_CORNER USE_ROS2)
ament_target_dependencies(${PROJECT_NAME}_planner
  builtin_interfaces geometry_msgs interfaces nav_msgs rcl_interfaces rclcpp rclcpp_lifecycle
  rog_map sensor_msgs tf2_geometry_msgs tf2_ros visualization_msgs)
target_link_libraries(${PROJECT_NAME}_planner OpenMP::OpenMP_CXX yaml-cpp::yaml-cpp dw)

add_executable(${PROJECT_NAME}_node src/nav_executor_node.cpp
  src/path_planner/path_planner.cpp src/path_executor/path_executor.cpp
  src/path_executor/monitoring/command_safety.cpp
  src/task_manager/task_manager.cpp
  src/common/environment/terrain_grid.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  "${minco_source_dir}/include")
ament_target_dependencies(${PROJECT_NAME}_node
  example_interfaces
  geometry_msgs
  interfaces
  rog_map
  nav_msgs
  sensor_msgs
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_mpc ${PROJECT_NAME}_planner ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_terrain_grid test/test_terrain_grid.cpp src/common/environment/terrain_grid.cpp)
  target_include_directories(test_terrain_grid PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
  target_link_libraries(test_terrain_grid Eigen3::Eigen)
  ament_target_dependencies(test_terrain_grid nav_msgs sensor_msgs)
  add_test(NAME terrain_grid COMMAND test_terrain_grid)
  add_executable(test_terrain_map_query test/test_terrain_map_query.cpp
    src/common/environment/terrain_grid.cpp src/common/environment/terrain_map_query.cpp)
  target_include_directories(test_terrain_map_query PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include" "${minco_source_dir}/include")
  target_link_libraries(test_terrain_map_query Eigen3::Eigen ${OpenCV_LIBS})
  ament_target_dependencies(test_terrain_map_query nav_msgs rog_map sensor_msgs)
  add_test(NAME terrain_map_query COMMAND test_terrain_map_query)
  add_executable(test_mpc_horizon test/test_mpc_horizon.cpp)
  target_include_directories(test_mpc_horizon PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
  target_link_libraries(test_mpc_horizon ${PROJECT_NAME}_mpc)
  add_test(NAME mpc_horizon COMMAND test_mpc_horizon)
  add_executable(test_omni_kino_astar test/test_omni_kino_astar.cpp
    src/path_planner/search/omni_kino_astar.cpp src/common/environment/terrain_grid.cpp)
  target_include_directories(test_omni_kino_astar PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
  target_link_libraries(test_omni_kino_astar Eigen3::Eigen)
  ament_target_dependencies(test_omni_kino_astar nav_msgs sensor_msgs)
  add_test(NAME omni_kino_astar COMMAND test_omni_kino_astar)
  add_executable(test_arc_length_speed_profile test/test_arc_length_speed_profile.cpp
    src/path_planner/trajectory/arc_length_speed_profile.cpp)
  target_include_directories(test_arc_length_speed_profile PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
  add_test(NAME arc_length_speed_profile COMMAND test_arc_length_speed_profile)
  add_executable(test_rog_map_command_safety test/test_rog_map_command_safety.cpp
    src/path_executor/monitoring/command_safety.cpp src/common/environment/terrain_grid.cpp)
  target_include_directories(test_rog_map_command_safety PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
  ament_target_dependencies(test_rog_map_command_safety
    geometry_msgs interfaces nav_msgs rclcpp rog_map sensor_msgs tf2 tf2_geometry_msgs tf2_ros)
  add_test(NAME rog_map_command_safety COMMAND test_rog_map_command_safety)
endif()

ament_package()
