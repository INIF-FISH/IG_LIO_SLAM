CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
project(octomap_dynamic_map
	VERSION 1.0.0
	DESCRIPTION "Octomap in benchmark on dm, dynamic in map"
	LANGUAGES CXX
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # enables -fPIC in applicable compilers (required to avoid link errors in some cases)
ADD_SUBDIRECTORY(octomap)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/build/lib/cmake/octomap")

# see here: https://github.com/PointCloudLibrary/pcl/issues/3680#issuecomment-700642602
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()
# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

#############
# PACKAGES  #
#############

find_package(Glog REQUIRED)
find_package(gflags REQUIRED)
set(LOG_LIB glog::glog gflags)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)
set(FUN_LIB Eigen3::Eigen ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(octomap_mapping/include)
add_library(dm_lib SHARED octomap_mapping/src/octomapper.cpp)
add_dependencies(dm_lib octomap)
target_link_libraries(dm_lib ${LOG_LIB} yaml-cpp ${FUN_LIB})
set (LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

############
# BINARIES #
############
add_executable(octomap_run octomap_PCD.cpp)
target_link_libraries(octomap_run dm_lib)