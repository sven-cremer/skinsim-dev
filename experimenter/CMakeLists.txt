cmake_minimum_required(VERSION 2.8.3)
project(model)

set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_C_COMPILER "clang")

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../CMake )

set (MIN_OGRE_VERSION 1.7.4 CACHE INTERNAL "Ogre version requirement" FORCE)

# find_package( libyaml-cpp REQUIRED )

# Depend on system install of Gazebo
FIND_PACKAGE( PkgConfig REQUIRED )
find_package( gazebo    REQUIRED )
find_package( Eigen3    REQUIRED )
find_package( OGRE      REQUIRED )
find_package( Boost     COMPONENTS
                        system
                        filesystem
                        thread
                        REQUIRED )

include_directories ("${PROJECT_SOURCE_DIR}/../generator/include")
include_directories ("${PROJECT_SOURCE_DIR}/../plugins/include")

include_directories ( ${ODE_INCLUDE_DIRS}
                      ${OPENGL_INCLUDE_DIR}
                      ${OGRE_INCLUDE_DIRS}
                      ${Boost_INCLUDE_DIRS}
                      ${PROTOBUF_INCLUDE_DIR}
                      ${GAZEBO_INCLUDE_DIRS}
                      ${SDFormat_INCLUDE_DIRS}
                      ${EIGEN3_INCLUDE_DIR}    )

link_directories( ${Boost_LIBRARY_DIRS} )

if(HAVE_BULLET)
  include_directories ( ${BULLET_INCLUDE_DIRS} )
  link_directories ( ${BULLET_LIBRARY_DIRS} )
endif()
if(HAVE_DART)
  include_directories ( ${DARTCore_INCLUDE_DIRS} )
  link_directories ( ${DARTCore_LIBRARY_DIRS} )
endif()

add_executable( auto_experimenter src/auto_experimenter.cc )
target_link_libraries( auto_experimenter ${GAZEBO_LIBRARIES} yaml-cpp ${Boost_LIBRARIES} )

add_executable( generate_experiment_specification src/generate_experiment_specification.cc )
target_link_libraries( generate_experiment_specification ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp )

