cmake_minimum_required(VERSION 2.8.3)
project (SkinSim)
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
 
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
 
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
 
include_directories("${PROJECT_INCLUDE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("./skinsim_msgs")

add_subdirectory(skinsim_msgs)
add_subdirectory(skinsim_plugins)
add_subdirectory(skinsim_test)
add_subdirectory(skinsim_gen)

