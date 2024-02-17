cmake_minimum_required(VERSION 3.8)
project(ig_lio_c)
add_compile_options(-std=c++17)
add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)
set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
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

# find dependencies
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

# PCL
# file(GLOB PCL_LIBRARIES /usr/local/lib/libpcl_*)
# set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.12)

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
  visualization_msgs
  GTSAM)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(sources
  src/ig_lio_c/ieskf/ieskf.cpp
  src/ig_lio_c/localizer/icp_localizer.cpp
  src/ig_lio_c/map_builder/imu_processor.cpp
  src/ig_lio_c/map_builder/iglio_builder.cpp
  src/ig_lio_c/voxel_map/voxel_map.cpp
  src/ig_lio_c/commons.cpp
  src/map_builder_node.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${sources}
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  PLUGIN "ig_lio_c::MapBuilderNode"
  EXECUTABLE map_builder_node
)

add_executable(map_builder_node
  ${sources}
)

ament_target_dependencies(map_builder_node
  ${dependencies}
)

target_link_libraries(map_builder_node
  ${Eigen3_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  map_builder_node

  # testing
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
