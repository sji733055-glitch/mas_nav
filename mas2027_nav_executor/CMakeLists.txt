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
find_package(Eigen3 REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(rog_map REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(minco_controller_source_dir "${CMAKE_CURRENT_SOURCE_DIR}/../mas2027_planner/minco_controller")
file(GLOB qpoases_sources "${minco_controller_source_dir}/third_party/qpOASES/src/*.cpp")
add_library(${PROJECT_NAME}_qpoases STATIC ${qpoases_sources})
set_target_properties(${PROJECT_NAME}_qpoases PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_qpoases PUBLIC
  "${minco_controller_source_dir}/third_party/qpOASES/include")
target_compile_definitions(${PROJECT_NAME}_qpoases PRIVATE __NO_COPYRIGHT__ LINUX)
target_compile_options(${PROJECT_NAME}_qpoases PRIVATE -Wno-format-truncation)

add_library(${PROJECT_NAME}_mpc STATIC
  "${minco_controller_source_dir}/src/mpc_solver.cpp")
target_include_directories(${PROJECT_NAME}_mpc PUBLIC
  "${minco_controller_source_dir}/include"
  "${minco_controller_source_dir}/include/minco_controller")
target_link_libraries(${PROJECT_NAME}_mpc PUBLIC Eigen3::Eigen ${PROJECT_NAME}_qpoases)

set(minco_planner_source_dir "${CMAKE_CURRENT_SOURCE_DIR}/../mas2027_planner/minco_planner")
file(GLOB minco_utils_sources "${minco_planner_source_dir}/src/utils/*.cpp")
file(GLOB minco_smac_sources "${minco_planner_source_dir}/src/smac_search/*.cpp")
file(GLOB minco_traj_sources "${minco_planner_source_dir}/src/traj_opt/*.cpp")
add_library(${PROJECT_NAME}_planner STATIC
  ${minco_utils_sources}
  ${minco_smac_sources}
  ${minco_traj_sources}
  "${minco_planner_source_dir}/src/minco_core/astar.cpp"
  "${minco_planner_source_dir}/src/minco_core/corridor_generator.cpp"
  "${minco_planner_source_dir}/src/minco_core/components/global_path_searcher.cpp"
  "${minco_planner_source_dir}/src/minco_core/components/local_path_processor.cpp"
  "${minco_planner_source_dir}/src/minco_core/components/map_query_adapters.cpp"
  "${minco_planner_source_dir}/src/minco_core/components/planner_mode_context.cpp"
  "${minco_planner_source_dir}/src/minco_core/components/recovery_behaivor.cpp"
  "${minco_planner_source_dir}/src/minco_core/components/trajectory_safety_checker.cpp"
  "${minco_planner_source_dir}/src/minco_core/minco_fsm.cpp"
  "${minco_planner_source_dir}/src/minco_core/minco_planner.cpp"
  "${minco_planner_source_dir}/src/minco_core/minco_utils.cpp"
  "${minco_planner_source_dir}/src/minco_core/visualizer.cpp"
  "${minco_planner_source_dir}/src/performance/planner_performance_monitor.cpp")
target_include_directories(${PROJECT_NAME}_planner PUBLIC "${minco_planner_source_dir}/include")
target_compile_definitions(${PROJECT_NAME}_planner PUBLIC
  ROOT_DIR="${minco_planner_source_dir}/" ORIGIN_AT_CORNER USE_ROS2)
ament_target_dependencies(${PROJECT_NAME}_planner
  builtin_interfaces geometry_msgs interfaces nav2_core nav2_costmap_2d nav2_util nav_msgs
  pluginlib rcl_interfaces rclcpp rclcpp_lifecycle rog_map tf2_ros)
target_link_libraries(${PROJECT_NAME}_planner OpenMP::OpenMP_CXX yaml-cpp::yaml-cpp dw)

add_executable(${PROJECT_NAME}_node src/nav_executor_node.cpp src/path_planner.cpp src/path_executor.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(${PROJECT_NAME}_node
  example_interfaces
  geometry_msgs
  interfaces
  rog_map
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_mpc ${PROJECT_NAME}_planner)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
