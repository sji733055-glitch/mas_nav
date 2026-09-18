cmake_minimum_required(VERSION 3.16)
project(odom_localizer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common io)

ament_auto_find_build_dependencies()

set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
set(BUILD_HELPER ON CACHE BOOL "" FORCE)
set(_odom_localizer_saved_shared_libs "${BUILD_SHARED_LIBS}")
set(BUILD_SHARED_LIBS OFF)
add_subdirectory(3rdparty/small_gicp EXCLUDE_FROM_ALL)
set(BUILD_SHARED_LIBS "${_odom_localizer_saved_shared_libs}")

file(GLOB_RECURSE sources CONFIGURE_DEPENDS "src/*.cpp")
ament_auto_add_library(${PROJECT_NAME} SHARED ${sources})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/small_gicp/include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(${PROJECT_NAME}
  OpenMP::OpenMP_CXX
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  small_gicp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN odom_localizer::OdomLocalizerNode
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
)
