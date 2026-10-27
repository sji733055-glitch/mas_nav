cmake_minimum_required(VERSION 3.10)
project(mid360_driver)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB_RECURSE sources CONFIGURE_DEPENDS "src/*.cpp")
ament_auto_add_library(${PROJECT_NAME} SHARED ${sources})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN mid360_driver::Mid360DriverNode
  EXECUTABLE ${PROJECT_NAME}_node
)

target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<CONFIG:Debug>:-O0 -g -Wall -Wextra -Wpedantic -Wconversion -fsanitize=address,undefined -fstack-protector-strong -fcf-protection=full>
  $<$<CONFIG:Release>:-O3 -DNDEBUG -Wall -Wextra -Wpedantic -Wconversion -flto=auto -march=native>
)

target_link_options(${PROJECT_NAME} PRIVATE
  $<$<CONFIG:Debug>:-fsanitize=address,undefined>
  $<$<CONFIG:Release>:-flto=auto>
)

target_compile_options(${PROJECT_NAME}_node PRIVATE $<$<CONFIG:Debug>:-fsanitize=address,undefined>)
target_link_options(${PROJECT_NAME}_node PRIVATE $<$<CONFIG:Debug>:-fsanitize=address,undefined>)

# 2026-09-18：删除 INSTALL_TO_SHARE config launch。config/params.yaml 与 launch/mid360_driver.launch.py
# 是 bringup 那份配置的重复来源（validate_crc 等取值相反），且该 launch 从 share 目录 glob 配置、
# 本包又从不安装它（等于用默认参数起节点），全仓库无引用，已一并删除。
# 雷达与融合参数以 mas2027_nav_bringup/config/small_point_lio_params.yaml 为唯一真源。
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
