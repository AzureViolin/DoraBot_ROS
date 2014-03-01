cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(my_rcg)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (my_rcg rcg.cpp)
target_link_libraries (my_rcg ${PCL_LIBRARIES})
