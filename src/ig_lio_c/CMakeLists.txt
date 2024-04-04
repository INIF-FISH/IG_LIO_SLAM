cmake_minimum_required(VERSION 3.8)
project(ig_lio_c)
add_compile_options(-std=c++17)
add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)
set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mpopcnt")

  if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
    include(ProcessorCount)
    ProcessorCount(N)
    message("Processer number:  ${N}")

    if(N GREATER 4)
      add_definitions(-DMP_EN)
      add_definitions(-DMP_PROC_NUM=3)
      message("core for MP: 3")
    elseif(N GREATER 3)
      add_definitions(-DMP_EN)
      add_definitions(-DMP_PROC_NUM=2)
      message("core for MP: 2")
    else()
      add_definitions(-DMP_PROC_NUM=1)
    endif()
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()

  find_package(OpenMP QUIET)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
endif()
find_package(ament_cmake REQUIRED COMPONENTS)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Sophus REQUIRED)
find_package(PCL REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_pcl REQUIRED)
find_package(filters REQUIRED)
find_package(grid_map_filters REQUIRED)
find_package(ig_lio_c_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(TBB REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)
find_package(Glog REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  pcl_ros
  std_msgs
  nav_msgs
  livox_ros_driver2
  geometry_msgs
  grid_map_msgs
  visualization_msgs
  GTSAM
  grid_map_core
  grid_map_ros
  grid_map_pcl
  filters
  ig_lio_c_msgs
  nav2_util
  TBB
  grid_map_filters
  nav2_map_server
  GRAPHICSMAGICKCPP
)

set(LINK_DIR ${PROJECT_SOURCE_DIR}/../../ThirdParty/octomap-Dynamic_point_cloud_removal/lib)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(sources_mapping_node
  src/ig_lio_c/ieskf/ieskf.cpp
  src/ig_lio_c/localizer/icp_localizer.cpp
  src/ig_lio_c/map_builder/imu_processor.cpp
  src/ig_lio_c/map_builder/iglio_builder.cpp
  src/ig_lio_c/voxel_map/voxel_map.cpp
  src/ig_lio_c/commons.cpp
  src/map_builder_node.cpp
)

set(sources_converter
  src/occupancy_grid_converter.cpp
)

add_executable(map_builder_node
  ${sources_mapping_node}
)

ament_target_dependencies(map_builder_node
  ${dependencies}
)

target_link_libraries(map_builder_node
  ${Eigen3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${LINK_DIR}/libdm_lib.so
  gtsam
  TBB::tbb
)

add_executable(occupancy_grid_converter
  ${sources_converter}
)

ament_target_dependencies(occupancy_grid_converter
  ${dependencies}
)

target_link_libraries(occupancy_grid_converter
  ${Eigen3_LIBRARIES}
  ${PCL_LIBRARIES}
  TBB::tbb
)

install(TARGETS
  map_builder_node
  occupancy_grid_converter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
