cmake_minimum_required(VERSION 3.10)
project(interfaces)

set(MSG_DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB_RECURSE MSG_FILES_ABS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")
foreach(MSG_FILE ${MSG_FILES_ABS})
  file(RELATIVE_PATH REL_PATH "${CMAKE_CURRENT_SOURCE_DIR}" "${MSG_FILE}")
  list(APPEND MSG_FILES_REL "${REL_PATH}")
endforeach()
rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES_REL} DEPENDENCIES ${MSG_DEPENDENCIES})

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)