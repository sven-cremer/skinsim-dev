cmake_minimum_required(VERSION 2.8.3)
project(gen)

set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_C_COMPILER "clang")

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../CMake )

set (MIN_OGRE_VERSION 1.7.4 CACHE INTERNAL "Ogre version requirement" FORCE)

# Depend on system install of Gazebo
FIND_PACKAGE( PkgConfig REQUIRED )
find_package( gazebo    REQUIRED )
find_package( Eigen3    REQUIRED )
find_package( Boost     COMPONENTS
                        system
                        filesystem
                        thread
                        REQUIRED )

include_directories ("${PROJECT_SOURCE_DIR}/include")

include_directories ( ${Boost_INCLUDE_DIRS}
                      ${PROTOBUF_INCLUDE_DIR}
                      ${GAZEBO_INCLUDE_DIRS}
                      ${SDFormat_INCLUDE_DIRS}
                      ${EIGEN3_INCLUDE_DIR}    )

link_directories( ${Boost_LIBRARY_DIRS} )

# Build
add_library(model_builder src/model_builder.cpp)
target_link_libraries(model_builder ${GAZEBO_LIBRARIES} yaml-cpp  ${Boost_LIBRARIES} )

add_executable(skin_model_generator src/skin_model_generator.cc)
target_link_libraries(skin_model_generator model_builder ${GAZEBO_LIBRARIES} yaml-cpp  ${Boost_LIBRARIES} )

