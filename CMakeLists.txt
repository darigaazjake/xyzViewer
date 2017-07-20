cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(xyz_viewer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (xyz_viewer xyz_viewer)
target_link_libraries (xyz_viewer ${PCL_LIBRARIES})