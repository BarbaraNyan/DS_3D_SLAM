cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(PointCloudPython)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

add_library(PointCloudPython SHARED PointCloudPython.cpp)
target_link_libraries(PointCloudPython ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${OCTOMAP_LIBRARIES})
