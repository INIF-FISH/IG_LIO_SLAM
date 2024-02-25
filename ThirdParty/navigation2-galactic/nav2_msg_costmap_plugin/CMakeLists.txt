cmake_minimum_required(VERSION 3.5)
project(nav2_msg_costmap_plugin)

set(lib_name ${PROJECT_NAME}_core)

# === Environment ===

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    grid_map_core
    grid_map_ros
    grid_map_msgs
    tf2
    tf2_ros
    tf2_eigen
    pluginlib)

# === Build ===

add_library(${lib_name} SHARED
            src/msg_layer.cpp)
include_directories(include)

# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)

# === Ament work ===

# pluginlib_export_plugin_description_file() installs msg_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
pluginlib_export_plugin_description_file(nav2_costmap_2d msg_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
